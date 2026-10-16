# mixins/web_ui_mixin.py — The redesigned web UI, hosted in the desktop window
"""
Web UI mixin: opens the new HTML/CSS/JS Studio UI in a desktop window.

``--web-ui`` (or ``FIC_WEB_UI=1``, or ``general.ui: web`` in the settings
file) starts a second, local-only instance of the web remote's server: bound
to 127.0.0.1, sharing the first one's control lease, snapshot and command
dispatch, and guarded by a token generated for this run. The window is a
``QWebEngineView`` inside this process; if QtWebEngine cannot be imported the
page opens in the system browser instead. The classic Qt window is the
default and stays available (``--classic`` forces it); closing the web window closes the Studio.

No ROS, controller or safety logic lives here or in the page: every command
goes through ``web_remote.actions.handle_command``, the same dispatch the
phone page and the Qt control bar use.
"""
import os
import secrets
import sys
import webbrowser

from fic_studio.web_remote.lease import ControlLease  # noqa: F401  (documented dependency)
from fic_studio.web_remote.server import WebRemoteServer


def web_ui_requested(argv=None, settings=None, environ=None):
    """Say whether the new UI was asked for. ``--classic`` always wins."""
    argv = sys.argv if argv is None else argv
    environ = os.environ if environ is None else environ
    if '--classic' in argv:
        return False
    # No screen (the boot service on a board runs Qt offscreen): there is no one
    # to see a window, and an error dialog would block start-up. Run as before;
    # remote clients get the web UI from the web remote server.
    from fic_studio.qt_webengine import HEADLESS_PLATFORMS
    if environ.get('QT_QPA_PLATFORM', '') in HEADLESS_PLATFORMS:
        return False
    if '--web-ui' in argv or '--web-ui-browser' in argv or environ.get('FIC_WEB_UI', '') not in ('', '0'):
        return True
    return str((settings or {}).get('general', {}).get('ui', 'classic')).lower() == 'web'


def web_ui_browser_requested(argv=None, environ=None):
    """The browser is opt-in only: ``--web-ui-browser`` (or ``FIC_WEB_UI_BROWSER=1``)."""
    argv = sys.argv if argv is None else argv
    environ = os.environ if environ is None else environ
    return ('--web-ui-browser' in argv
            or environ.get('FIC_WEB_UI_BROWSER', '') not in ('', '0'))


class WebUIMixin:
    """Mixin: host the web UI for this Studio."""

    def _init_web_ui(self):
        """Start the local server and open the window, if the UI was asked for."""
        self._web_ui_server = None
        self._web_ui_window = None
        if not web_ui_requested(settings=getattr(self, 'settings', {})):
            return False
        return self.open_web_ui()

    def _web_ui_start_server(self):
        if getattr(self, '_web_ui_server', None) is not None:
            return self._web_ui_server
        self._web_ui_token = secrets.token_urlsafe(24)
        server = WebRemoteServer(dispatch=self._web_dispatch, snapshot=self._web_snapshot,
                                 lease=self._web_lease, on_event=self._web_on_event,
                                 scene=self._web_scene_provider, panels=self._web_panels_provider,
                                 extensions=(self._web_ext_file, self._web_ext_state_provider),
                                 host='127.0.0.1', local_token=self._web_ui_token)
        server.start(0)                        # any free port; nobody else needs it
        self._web_ui_server = server
        timer = getattr(self, '_web_state_timer', None)      # feeds the UI's event stream, with or without the remote server
        if timer is not None and not timer.isActive():
            from fic_studio.mixins.web_remote_mixin import STATE_INTERVAL_MS
            timer.start(STATE_INTERVAL_MS)
        return server

    def _web_ui_brightness(self):
        try:
            from fic_studio.web_remote.settings_page import appearance_state
            return appearance_state(self).get('brightness') or 'dark'
        except Exception:
            return 'dark'

    def web_ui_url(self):
        server = self._web_ui_start_server()
        return 'http://127.0.0.1:%d/ui/?t=%s&boot=%s' % (server.port, self._web_ui_token, self._web_ui_brightness())

    def open_web_ui(self):
        """Show the web UI in a QWebEngineView, or in the browser as a fallback."""
        try:
            url = self.web_ui_url()
        except OSError as exc:
            self.log_message('ERROR', 'Web UI: cannot start the local server (%s)' % exc, 'Web')
            return False
        if web_ui_browser_requested():
            self.log_message('INFO', 'Web UI: opening the system browser (--web-ui-browser)', 'Web')
            webbrowser.open(url)
            return True
        try:
            from PyQt5.QtCore import QUrl
            from PyQt5.QtWebEngineWidgets import QWebEngineView
        except ImportError as exc:
            return self._web_ui_unavailable(exc)
        host = self

        class _View(QWebEngineView):
            def closeEvent(self, event):       # closing the UI closes the Studio
                event.accept()
                if getattr(host, '_web_ui_closing', False):
                    return
                host._web_ui_closing = True
                host._web_ui_window = None
                host.close()
                from PyQt5.QtWidgets import QApplication
                QApplication.quit()            # explicit: do not rely on last-window-closed
        from PyQt5.QtCore import QTimer
        from PyQt5.QtGui import QColor
        view = _View()
        view.setAcceptDrops(True)              # files dragged in from the desktop (Robot Config's drop zone)
        view.setWindowTitle('FIC Studio')
        view.resize(1360, 860)
        # The theme's canvas before the page paints, not Chromium's white.
        light = self._web_ui_brightness() == 'light'
        canvas = QColor('#ffffff' if light else '#14161a')
        view.page().setBackgroundColor(canvas)
        # The canvas on the view itself only: a bare 'background' rule reaches every child, the right-click
        # menu too, which then drew the default black text on that background. The menu gets its own colours.
        menu = (('#ffffff', '#1f2937', '#eef1f4', '#9ca3af', '#d1d5db') if light
                else ('#1e2126', '#e6e8eb', '#2c3138', '#6b7280', '#3a3f47'))
        view.setStyleSheet(
            'QWebEngineView { background: %s; }'
            ' QMenu { background-color: %s; color: %s; border: 1px solid %s; padding: 4px 0; }'
            ' QMenu::item { padding: 6px 24px 6px 18px; color: %s; background: transparent; }'
            ' QMenu::item:selected { background-color: %s; color: %s; }'
            ' QMenu::item:disabled { color: %s; }'
            ' QMenu::separator { height: 1px; background-color: %s; margin: 4px 10px; }'
            % (canvas.name(), menu[0], menu[1], menu[4], menu[1], menu[2], menu[1], menu[3], menu[4]))
        self._web_ui_window = view             # keep a reference: a dropped view vanishes
        self._web_ui_shown = False
        # Shown once the page has loaded (the splash stays up until then), so the
        # window never opens on a blank page; at most 8 s later in any case.
        view.loadFinished.connect(lambda _ok: self._web_ui_reveal())
        QTimer.singleShot(8000, self._web_ui_reveal)
        view.load(QUrl(url))
        return True

    def _web_ui_reveal(self):
        view = getattr(self, '_web_ui_window', None)
        if view is None or getattr(self, '_web_ui_shown', False):
            return
        self._web_ui_shown = True
        view.showMaximized()                  # full screen when opened (the window can still be resized)
        view.raise_()
        view.activateWindow()
        done = getattr(self, '_web_ui_on_shown', None)
        if done is not None:
            done()

    def _web_ui_unavailable(self, exc):
        """QtWebEngine cannot load: say so, with the fix. Never fall back to a browser silently."""
        from fic_studio import qt_webengine
        self.log_message('ERROR', 'Web UI: QtWebEngine is unavailable (%s). %s'
                         % (exc, qt_webengine.fix_instructions().replace('\n', ' | ')), 'Web')
        self._web_ui_error = qt_webengine.error_text(exc)
        open_classic = self._web_ui_ask_classic(self._web_ui_error)
        self._web_ui_quit_requested = not open_classic
        return False

    def _web_ui_ask_classic(self, text):
        """Error dialog; True = open the classic interface, False = quit."""
        from PyQt5.QtWidgets import QMessageBox
        box = QMessageBox(QMessageBox.Critical, 'FIC Studio: web interface unavailable', text)
        classic = box.addButton('Open classic interface', QMessageBox.AcceptRole)
        box.addButton('Quit', QMessageBox.RejectRole)
        box.exec_()
        return box.clickedButton() is classic

    def _web_restart(self):
        """Re-launch the Studio (after the web UI re-pointed its station), once the reply is out."""
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(800, lambda: os.execv(sys.executable, [sys.executable] + sys.argv))

    def _web_close(self):
        """Close the Studio (File, Exit from the web UI), once the reply is out."""
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(300, self.close)

    def show_classic_ui(self):
        self.show()

    def _teardown_web_ui(self):
        server, self._web_ui_server = getattr(self, '_web_ui_server', None), None
        if server is not None:
            server.stop()
        window, self._web_ui_window = getattr(self, '_web_ui_window', None), None
        if window is not None:
            self._web_ui_closing = True
            window.close()
            from PyQt5.QtWidgets import QApplication
            QApplication.quit()        # the classic window was never shown, so Qt would not
