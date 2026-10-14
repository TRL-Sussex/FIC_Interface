"""The installed app opens the web UI in its own Qt window, never in a browser."""
import configparser
import os
import sys
import types
import webbrowser

import pytest

from fic_studio import qt_webengine
from fic_studio.mixins import web_ui_mixin as mixin

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))


def _read(*parts):
    with open(os.path.join(REPO, *parts)) as f:
        return f.read()


def test_app_entry_runs_the_launcher_script_and_the_script_passes_web_ui():
    cp = configparser.ConfigParser(interpolation=None)
    cp.optionxform = str
    cp.read_string(_read('fic_install', 'fic-control-studio.desktop'))
    entry = cp['Desktop Entry']
    assert entry['Exec'] == '__FIC_INSTALL__/fic-control-studio.sh'
    assert entry['Terminal'] == 'false'
    script = _read('fic_install', 'fic-control-studio.sh')
    assert 'exec ros2 run fic_studio fic_studio_pro --web-ui "$@"' in script
    assert 'xdg-open' not in script and 'firefox' not in script


def test_classic_action_keeps_the_old_ui_one_click_away():
    cp = configparser.ConfigParser(interpolation=None)
    cp.optionxform = str
    cp.read_string(_read('fic_install', 'fic-control-studio.desktop'))
    assert 'Classic' in cp['Desktop Entry']['Actions']
    assert cp['Desktop Action Classic']['Exec'].endswith('fic-control-studio.sh --classic')


def test_docker_entry_runs_web_ui():
    text = _read('docker', 'rootfs', 'usr', 'share', 'applications', 'fic-control-studio.desktop')
    assert 'fic studio --web-ui' in text and 'fic studio --classic' in text


@pytest.mark.parametrize('path', [('fic_install', 'install-deps.sh'),
                                  ('Install_FIC_Studio.desktop',),
                                  ('docker', 'Dockerfile')])
def test_installers_install_the_system_qtwebengine(path):
    assert 'python3-pyqt5.qtwebengine' in _read(*path)


def test_classic_overrides_web_ui_and_browser_is_opt_in_only():
    assert mixin.web_ui_requested(['x', '--web-ui'], {}, {}) is True
    assert mixin.web_ui_requested(['x', '--web-ui', '--classic'], {}, {}) is False
    assert mixin.web_ui_requested(['x'], {'general': {'ui': 'web'}}, {}) is True
    assert mixin.web_ui_requested(['x'], {}, {}) is False
    assert mixin.web_ui_browser_requested(['x', '--web-ui'], {}) is False
    assert mixin.web_ui_browser_requested(['x', '--web-ui-browser'], {}) is True
    assert mixin.web_ui_requested(['x', '--web-ui-browser'], {}, {}) is True


def test_error_text_has_the_exact_fix():
    text = qt_webengine.error_text(ImportError('undefined symbol'))
    assert 'sudo apt install python3-pyqt5.qtwebengine' in text
    assert 'pip uninstall' in text and 'PyQtWebEngine' in text and 'undefined symbol' in text


class _Host(mixin.WebUIMixin):
    def __init__(self, classic=True):
        self.logs = []
        self.asked = []
        self._classic = classic

    def log_message(self, level, text, tag=''):
        self.logs.append((level, text))

    def web_ui_url(self):
        return 'http://127.0.0.1:1/ui/?t=x'

    def _web_ui_ask_classic(self, text):
        self.asked.append(text)
        return self._classic


@pytest.fixture
def no_webengine(monkeypatch):
    monkeypatch.setitem(sys.modules, 'PyQt5.QtWebEngineWidgets', None)   # import raises ImportError
    opened = []
    monkeypatch.setattr(webbrowser, 'open', lambda url: opened.append(url))
    monkeypatch.setattr(sys, 'argv', ['fic_studio_pro', '--web-ui'])
    monkeypatch.delenv('FIC_WEB_UI_BROWSER', raising=False)
    return opened


@pytest.mark.parametrize('choose_classic', [True, False])
def test_missing_webengine_shows_the_fix_and_never_opens_a_browser(no_webengine, choose_classic):
    host = _Host(classic=choose_classic)
    assert host.open_web_ui() is False
    assert no_webengine == []                                    # no browser, silently or otherwise
    assert 'apt install python3-pyqt5.qtwebengine' in host.asked[0]
    assert host._web_ui_quit_requested is (not choose_classic)
    assert any(level == 'ERROR' for level, _ in host.logs)


def test_browser_flag_opens_the_browser(no_webengine, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['fic_studio_pro', '--web-ui-browser'])
    host = _Host()
    assert host.open_web_ui() is True
    assert no_webengine == ['http://127.0.0.1:1/ui/?t=x'] and host.asked == []


def test_web_window_is_kept_and_closing_it_closes_the_studio(monkeypatch):
    pytest.importorskip('PyQt5.QtWidgets')
    from PyQt5.QtWidgets import QApplication, QWidget
    app = QApplication.instance() or QApplication([])

    from PyQt5.QtCore import pyqtSignal

    class FakePage:
        def setBackgroundColor(self, colour):
            self.colour = colour

    class FakeView(QWidget):
        loadFinished = pyqtSignal(bool)

        def load(self, url):
            self.url = url

        def page(self):
            if not hasattr(self, '_page'):
                self._page = FakePage()
            return self._page
    monkeypatch.setitem(sys.modules, 'PyQt5.QtWebEngineWidgets',
                        types.SimpleNamespace(QWebEngineView=FakeView))
    monkeypatch.setattr(sys, 'argv', ['fic_studio_pro', '--web-ui'])
    quits = []
    monkeypatch.setattr(QApplication, 'quit', staticmethod(lambda: quits.append(1)))
    host = _Host()
    host.closed = []
    host.close = lambda: host.closed.append(1)
    assert host.open_web_ui() is True
    view = host._web_ui_window
    assert view is not None and not view.isVisible()             # referenced, hidden until the page loads
    shown = []
    host._web_ui_on_shown = lambda: shown.append(1)               # the splash closes then
    view.loadFinished.emit(True)
    assert view.isVisible() and shown == [1]
    assert view.page().colour.name() in ('#14161a', '#ffffff')    # the theme's canvas, not white by default
    view.close()
    assert host.closed == [1] and quits == [1]                   # the Studio closes, the app quits
    assert host._web_ui_window is None


def test_headless_boot_never_opens_the_web_window():
    """The board's boot service runs Qt offscreen with --web-ui: no window, no blocking dialog."""
    from fic_studio.mixins import web_ui_mixin as mixin
    assert mixin.web_ui_requested(['x', '--web-ui'], {}, {'QT_QPA_PLATFORM': 'offscreen'}) is False
    assert mixin.web_ui_requested(['x'], {'general': {'ui': 'web'}}, {'QT_QPA_PLATFORM': 'minimal'}) is False
    assert mixin.web_ui_requested(['x', '--web-ui'], {}, {'QT_QPA_PLATFORM': 'xcb'}) is True
