# mixins/web_remote_mixin.py — The Studio's controls in a browser, for phones
"""
Web remote mixin: the Studio's controls in a browser, for phones.

Serves the control bar, dashboard and session controls to any browser on the
network, for as long as the Studio runs.

Lives as a studio mixin rather than an extension, like the gamepad, because
it is another input device: it needs the E-stop and process start/stop,
which the extension host deliberately does not expose, and a status-bar
indicator.

Threading: the HTTP server answers on its own threads, but every command
and every snapshot is executed on the Qt thread through MainThreadInvoker,
the same thread the desktop's own buttons run on.

Who may open it: with accounts on (the default) a browser signs in first and
sees only the pages an admin allowed it, over HTTPS so the password does not
cross the lab in the clear. The user list is one store shared by every Studio
on the ROS domain, so an admin edits it once — on the Master — and it reaches
the rest. One browser at a time holds control, and any browser may E-stop,
signed in or not.
"""
import os

from fic_studio.config import WS_PATH
from fic_studio.fleet.stations import station_label
from fic_studio.web_remote import tls
from fic_studio.web_remote.accounts import Accounts
from fic_studio import paths as _paths
from fic_studio.web_remote.actions import build_snapshot, handle_command
from fic_studio.web_remote.auth import Auth
from fic_studio.web_remote.discovery import ANNOUNCE_S
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.netinfo import lan_addresses, network_label, published_port
from fic_studio.web_remote.server import WebRemoteServer
from fic_studio.web_remote.sync import AccountsSync
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QHBoxLayout, QInputDialog, QLabel, QLineEdit, QMessageBox,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

# Sign-up is off unless an admin turns it on: a visitor cannot make an account by themselves.
WEB_REMOTE_DEFAULTS = {'enabled': True, 'port': 8088,
                       'accounts': True, 'https': True, 'signup': False}

#: Where the user list and this machine's certificate live. Outside the
#: repository: they are this machine's, not the software's.
WEB_STATE_DIR = _paths.state_dir()
ACCOUNTS_FILE = os.path.join(WEB_STATE_DIR, 'accounts.json')
#: How long a Studio with no user list waits for the lab's before making its
#: own first admin. Without it, four robots switched on together would each
#: invent an admin password of their own.
ACCOUNTS_WAIT_S = 20

#: Set by the Docker launcher's --lan: sign-in is on whatever the settings say,
#: the account ``admin`` has this password (a new one every start), and
#: visitors cannot make accounts of their own (in Docker every account could
#: drive the simulation and run scripts).
ADMIN_PASSWORD_ENV = 'FIC_WEB_ADMIN_PASSWORD'

#: Studio pages the browser can use as well as the extensions, in order. The
#: control bar, the dashboard and the log have pages of their own in the web
#: app; these do not, and are worth having on a phone.
WEB_STUDIO_PAGES = (('multi_robot', 'Multi-Robot'), ('tuning', 'Tuning'),
                    ('safety', 'Safety'), ('locomotion', 'Locomotion'))

#: How often connected browsers get fresh state.
STATE_INTERVAL_MS = 100

#: Seconds a command may take on the Qt thread before the phone is told the
#: Studio is busy. E-stop and session start/stop stop processes and wait.
COMMAND_TIMEOUT_S = {'estop': 15.0, 'session': 10.0, 'process': 15.0, 'tuning_info': 6.0,
                     'tuning_get': 6.0, 'tuning_plot': 6.0, 'tuning_save': 10.0, 'tuning_profile': 10.0}
DEFAULT_COMMAND_TIMEOUT_S = 3.0


class WebRemoteMixin:
    """Mixin that runs the web remote server alongside the Studio."""

    # ── Lifecycle ────────────────────────────────────────────────────────
    def _init_web_remote(self):
        """Create the server plumbing; start serving if enabled in settings.

        Call once the control bar exists: commands drive its widgets.
        """
        from fic_studio.web_remote.invoker import MainThreadInvoker
        self._web_invoker = MainThreadInvoker(self)
        self._web_lease = ControlLease()
        self._web_server = None
        self._web_directory = None
        self._web_accounts = None
        self._web_auth = None
        self._web_sync = None
        self._web_urls = []
        self._web_error = ''
        self._web_state_timer = QTimer(self)
        self._web_state_timer.timeout.connect(self._web_push_state)
        self._web_announce_timer = QTimer(self)
        self._web_announce_timer.timeout.connect(self._web_announce)
        self._web_attach_statusbar()
        if self._web_settings()['enabled']:
            self._web_start()
        else:
            self._web_refresh_indicator()

    def _teardown_web_remote(self):
        try:
            self._web_stop()
        except Exception:
            pass

    def _web_settings(self):
        block = dict(WEB_REMOTE_DEFAULTS)
        block.update(getattr(self, 'settings', {}).get('web_remote', {}) or {})
        if os.environ.get(ADMIN_PASSWORD_ENV):
            block['accounts'] = True
            block['signup'] = False
        return block

    def _web_save_setting(self, key, value):
        self.settings.setdefault('web_remote', dict(WEB_REMOTE_DEFAULTS))[key] = value
        self._save_settings()

    def _web_start(self):
        if getattr(self, '_web_server', None) is not None:
            return True
        settings = self._web_settings()
        port = int(settings['port'])
        auth = self._web_open_accounts() if settings.get('accounts', True) else None
        tls_files = self._web_certificate() if settings.get('https', True) else None
        server = WebRemoteServer(dispatch=self._web_dispatch, snapshot=self._web_snapshot,
                                 lease=self._web_lease, on_event=self._web_on_event,
                                 scene=self._web_scene_provider,
                                 panels=self._web_panels_provider,
                                 extensions=(self._web_ext_file, self._web_ext_state_provider),
                                 auth=auth, tls_files=tls_files,
                                 allow_signup=bool(settings.get('signup', False)))
        server.on_accounts_changed = self._web_accounts_changed
        try:
            bound = server.start(port)
        except OSError as exc:
            self._web_error = 'Could not open port %d: %s' % (port, exc)
            self.log_message('ERROR', 'Web remote: %s' % self._web_error, 'Web')
            self._web_refresh_indicator()
            return False
        self._web_server, self._web_error = server, ''
        shown = published_port(bound)       # in a container, the published port
        self._web_urls = ['%s://%s:%d/' % (server.scheme, ip, shown)
                          for ip in lan_addresses()] \
            or ['%s://localhost:%d/' % (server.scheme, shown)]
        if bound != port:
            self.log_message('WARN', 'Web remote: port %d is taken (another Studio?) — '
                             'serving on %d instead' % (port, bound), 'Web')
        self.log_message('INFO', 'Web remote at %s' % '  ·  '.join(self._web_urls), 'Web')
        try:
            from fic_studio.web_remote.discovery import StudioDirectory
            self._web_directory = StudioDirectory(self.node, self._web_card)
            self._web_directory.announce()
        except Exception as exc:
            self._web_directory = None
            self.log_message('WARN', 'Web remote: cannot list other Studios (%s)' % exc, 'Web')
        self._web_start_accounts_sync()
        self._web_state_timer.start(STATE_INTERVAL_MS)
        self._web_announce_timer.start(int(ANNOUNCE_S * 1000))
        self._web_refresh_indicator()
        return True

    def _web_stop(self):
        for timer in (getattr(self, '_web_state_timer', None),
                      getattr(self, '_web_announce_timer', None)):
            if timer is not None:
                timer.stop()
        if getattr(self, '_web_ui_server', None) is not None:    # the UI's own stream still needs it
            self._web_state_timer.start(STATE_INTERVAL_MS)
        for attr in ('_web_directory', '_web_sync'):
            shared = getattr(self, attr, None)
            setattr(self, attr, None)
            if shared is not None:
                shared.close()
        server, self._web_server = getattr(self, '_web_server', None), None
        if server is not None:
            server.stop()
            self.log_message('INFO', 'Web remote stopped', 'Web')
        if getattr(self, '_web_lease', None) is not None:
            self._web_lease.revoke()
        self._web_auth = None
        self._web_urls = []
        self._web_refresh_indicator()

    # ── Accounts ─────────────────────────────────────────────────────────
    def _web_open_accounts(self):
        """Return an ``Auth`` over this lab's user list.

        A Studio that has never seen the lab's list waits ``ACCOUNTS_WAIT_S``
        for another Studio to send it. Only if none does does it make the
        first ``admin`` account, with a random password printed once in the
        log: that is how the first person in a new lab gets in.
        """
        accounts = Accounts.load(ACCOUNTS_FILE)
        self._web_accounts = accounts
        self._web_auth = Auth(accounts)
        given = os.environ.get(ADMIN_PASSWORD_ENV, '')
        if given:
            accounts.reset_admin(name='admin', password=given)
            try:
                accounts.save()
            except OSError as exc:
                self.log_message('ERROR', 'Web remote: cannot write %s (%s)'
                                 % (ACCOUNTS_FILE, exc), 'Web')
            self.log_message('INFO', 'Web remote: sign-in is on; the admin password '
                             'is the one the launcher printed', 'Web')
            return self._web_auth
        if accounts.admins():
            return self._web_auth
        self.log_message('INFO', 'Web remote: no users yet — waiting %d s for '
                         "another Studio to send the lab's list" % ACCOUNTS_WAIT_S, 'Web')
        QTimer.singleShot(int(ACCOUNTS_WAIT_S * 1000), self._web_first_admin)
        return self._web_auth

    def _web_first_admin(self):
        """Make the lab's first admin, unless another Studio has sent its list."""
        accounts = getattr(self, '_web_accounts', None)
        if accounts is None or accounts.admins():
            return
        first = accounts.ensure_admin()
        try:
            accounts.save()
        except OSError as exc:
            self.log_message('ERROR', 'Web remote: cannot write %s (%s)'
                             % (ACCOUNTS_FILE, exc), 'Web')
        line = ('Web remote: first sign-in is  admin / %s  '
                '— change it on the Account page' % first)
        self.log_message('WARN', line, 'Web')
        # The Studio's log is on screen, and a robot PC has no monitor. Say it
        # on stdout as well, where the journal keeps it:
        #   journalctl --user -u fic-studio | grep "first sign-in"
        print(line, flush=True)
        if getattr(self, '_web_sync', None) is not None:
            self._web_sync.share(force=True)

    def _web_certificate(self):
        """Return this machine's (certificate, key), or None if it cannot be made."""
        try:
            return tls.ensure_cert(WEB_STATE_DIR, lan_addresses())
        except Exception as exc:
            self.log_message('WARN', 'Web remote: no certificate (%s) — serving '
                             'without HTTPS' % exc, 'Web')
            return None

    def _web_accounts_changed(self):
        """Save the user list and send it to the other Studios (an admin edited it)."""
        accounts = getattr(self, '_web_accounts', None)
        if accounts is None:
            return
        try:
            accounts.save()
        except OSError as exc:
            self.log_message('ERROR', 'Web remote: cannot save users (%s)' % exc, 'Web')
        if getattr(self, '_web_sync', None) is not None:
            self._web_sync.share(force=True)

    def _web_start_accounts_sync(self):
        """Keep this Studio's user list in step with the others on the ROS domain."""
        if getattr(self, '_web_accounts', None) is None:
            return
        try:
            self._web_sync = AccountsSync(self.node, self._web_accounts,
                                          on_changed=self._web_accounts_taken)
            self._web_sync.share(force=True)
        except Exception as exc:
            self._web_sync = None
            self.log_message('WARN', 'Web remote: users are local to this Studio '
                             '(%s)' % exc, 'Web')

    def _web_reset_admin_password(self):
        """Set a new admin password from the Studio, for a lab that is locked out.

        Whoever is at this machine is trusted: the Studio is the machine. The
        old password is not needed, and the new one goes to the other Studios
        like any other change, so one reset covers the lab.
        """
        accounts = getattr(self, '_web_accounts', None) or Accounts.load(ACCOUNTS_FILE)
        typed, ok = QInputDialog.getText(
            self, 'Reset web remote admin password',
            'New password for the web remote admin.\n'
            'Leave it empty and the Studio will make one up.',
            QLineEdit.Password)
        if not ok:
            return
        try:
            name, password = accounts.reset_admin(password=(typed.strip() or None))
        except ValueError as exc:
            QMessageBox.warning(self, 'Reset admin password', str(exc))
            return
        try:
            accounts.save()
        except OSError as exc:
            QMessageBox.warning(self, 'Reset admin password',
                                'Could not write %s:\n%s' % (ACCOUNTS_FILE, exc))
            return
        self._web_accounts = accounts
        if getattr(self, '_web_auth', None) is not None:
            self._web_auth.accounts = accounts
            self._web_auth.sign_out_everyone()      # every browser signs in again
        if getattr(self, '_web_sync', None) is not None:
            self._web_sync.share(force=True)
        self.log_message('WARN', 'Web remote: admin password reset for "%s"' % name, 'Web')
        if typed.strip():
            QMessageBox.information(self, 'Reset admin password',
                                    'The web remote admin "%s" has the password you '
                                    'typed. Every browser has been signed out.' % name)
            return
        # Nobody typed one, so nobody knows it yet: show it, and say it where a
        # robot PC with no monitor keeps it as well.
        print('Web remote: admin password for "%s" is now  %s' % (name, password), flush=True)
        box = QMessageBox(QMessageBox.Information, 'Reset admin password',
                          'Sign in as <b>%s</b> with<br><br>'
                          '<span style="font-size:15pt"><b>%s</b></span><br><br>'
                          "Change it on the web page's Account sheet. Every browser "
                          'has been signed out.' % (name, password), parent=self)
        box.setTextInteractionFlags(Qt.TextSelectableByMouse)
        box.exec_()

    def _web_accounts_taken(self):
        """Another Studio had a newer user list: sign everyone out and say so."""
        if getattr(self, '_web_auth', None) is not None:
            self._web_auth.sign_out_everyone()
        self.log_message('INFO', 'Web remote: took the user list from another Studio', 'Web')

    # ── Extensions in the browser ────────────────────────────────────────
    def _web_panels(self):
        """Return the Studio pages and extensions a browser may use."""
        out = []
        for page_id, label in WEB_STUDIO_PAGES:
            if self.pages.widget(page_id) is not None:
                out.append({'id': page_id, 'label': label, 'section': 'Studio',
                            'description': ''})
        for ext_id, inst in (getattr(self, '_extension_instances', {}) or {}).items():
            if self.pages.widget(ext_id) is None:
                continue                      # its build_page() failed at start-up
            entry = {'id': ext_id,
                     'label': getattr(inst, 'NAME', '') or ext_id,
                     'section': getattr(inst, 'SIDEBAR_SECTION', 'Extensions'),
                     'description': getattr(inst, 'DESCRIPTION', '')[:200]}
            web = self._web_ext_manifest_web(ext_id)
            if web:
                entry['web'] = {'page': '/ext/%s/%s' % (ext_id, web['page']),
                                'css': ('/ext/%s/%s' % (ext_id, web['css'])) if web.get('css') else None,
                                'poll_ms': web['poll_ms']}
            out.append(entry)
        studio = [e for e in out if e['section'] == 'Studio']
        rest = sorted((e for e in out if e['section'] != 'Studio'),
                      key=lambda e: (e['section'], e['label']))
        return studio + rest

    def _web_panel(self, ext_id):
        """Return one extension's page as data, or None if there is no such page.

        Runs on the Qt thread: the widgets belong to it.
        """
        widget = self.pages.widget(str(ext_id))
        if widget is None:
            return None
        from fic_studio.web_remote import qtmirror
        described = qtmirror.describe(widget)
        described['id'] = str(ext_id)
        described['label'] = next((p['label'] for p in self._web_panels()
                                   if p['id'] == ext_id), str(ext_id))
        return described

    # ── Extensions with their own web page (manifest "web") ─────────────
    def _web_ext_manifest(self, ext_id):
        for m in getattr(self, '_extensions_manifests', None) or ():
            if m.id == ext_id:
                return m
        return None

    def _web_ext_manifest_web(self, ext_id):
        m = self._web_ext_manifest(ext_id)
        return dict(m.web) if m is not None and getattr(m, 'web', None) else {}

    def _web_ext_file(self, ext_id, rel):
        """The file ``rel`` from a loaded extension's ``web/`` folder, or None.

        Any thread: reads only the manifests, which never change while running.
        """
        if str(ext_id) not in (getattr(self, '_extension_instances', {}) or {}):
            return None
        m = self._web_ext_manifest(str(ext_id))
        if m is None or not getattr(m, 'web', None):
            return None
        root = (m.folder / 'web').resolve()
        target = (root / rel).resolve()
        if root not in target.parents or not target.is_file():
            return None
        return target

    def _web_ext_activate(self, inst):
        """Run on_activate() once before the first web use, as opening its page does."""
        if not getattr(inst, '_web_activated', False):
            try:
                inst.on_activate()
            except Exception as exc:
                self.log_message('WARN', '%s.on_activate failed: %s' % (getattr(inst, 'ID', '?'), exc), 'Extensions')
            inst._web_activated = True

    def _web_ext_state(self, ext_id):
        """An extension's ``web_state()``; None if it has no web page. Qt thread."""
        inst = (getattr(self, '_extension_instances', {}) or {}).get(str(ext_id))
        if inst is None or not self._web_ext_manifest_web(str(ext_id)):
            return None
        self._web_ext_activate(inst)
        state = inst.web_state()
        return state if isinstance(state, dict) else {'value': state}

    def _web_ext_call(self, ext_id, name, args, local):
        """Run an extension's ``web_action()``. Qt thread; the lease is already checked."""
        inst = (getattr(self, '_extension_instances', {}) or {}).get(str(ext_id))
        web = self._web_ext_manifest_web(str(ext_id))
        if inst is None or not web:
            raise LookupError('No such extension page')
        if name in web.get('local_only', ()) and not local:
            raise PermissionError('That is only available in the Studio window itself')
        self._web_ext_activate(inst)
        return inst.web_action(name, dict(args or {}))

    def _web_ext_state_provider(self, ext_id):
        """Answer the web server (its own thread) from the Qt thread."""
        return self._web_invoker.call(lambda: self._web_ext_state(ext_id), timeout=5.0)

    def _web_panel_act(self, ext_id, path, kind, value):
        """Do what a browser asked on an extension's page. Qt thread."""
        widget = self.pages.widget(str(ext_id))
        if widget is None:
            raise ValueError('no such extension')
        from fic_studio.web_remote import qtmirror
        inst = (getattr(self, '_extension_instances', {}) or {}).get(str(ext_id))
        if inst is not None and not getattr(inst, '_web_activated', False):
            # The Studio calls this when its page is opened; an extension that
            # only starts its timers there would otherwise sit dead in a browser.
            try:
                inst.on_activate()
                inst._web_activated = True
            except Exception:
                pass
        qtmirror.act(widget, path, kind, value)
        return self._web_panel(ext_id)

    # ── The 3D view's scene ──────────────────────────────────────────────
    def _web_scene_provider(self, mesh_index):
        """Return the current robot's 3D scene (None) or one of its meshes (an index).

        Built once per robot from its MuJoCo model and kept: the model is a few
        megabytes and compiling it takes about 0.1 s. Called on an HTTP thread,
        so it only reads what the Qt thread wrote.
        """
        from fic_studio.web_remote import scene as scene_mod
        robot = self.robot_dropdown.currentText()
        cached = getattr(self, '_web_scene_cache', None)
        if cached is None or cached[0] != robot:
            cfg = (getattr(self, 'robot_configs', {}) or {}).get(robot, {})
            rel = cfg.get('mujoco_xml_path')
            if not rel:
                raise RuntimeError('%s has no mujoco_xml_path in robots_config' % robot)
            # As elsewhere in the Studio: relative to fic_ws/src, straight from
            # the source tree, so an imported robot needs no rebuild.
            path = rel if os.path.isabs(rel) else os.path.join(WS_PATH, 'src', rel)
            model = scene_mod.load_model(path)
            data = scene_mod.scene_dict(model, robot=robot,
                                        joint_names=cfg.get('joint_names') or ())
            cached = (robot, model, data)
            self._web_scene_cache = cached
        if mesh_index is None:
            return cached[2]
        return scene_mod.mesh_bytes(cached[1], int(mesh_index))

    # ── Identity ─────────────────────────────────────────────────────────
    def _web_card(self):
        """Return this Studio's entry in every page's Studio switcher."""
        from fic_studio.ros_node import _host_tag
        host = _host_tag()
        server = getattr(self, '_web_server', None)
        port = server.port if server is not None else None
        return {
            'gui_id': '%s:%s' % (host, port),
            'label': '%s · %s' % (host, station_label(self.node.station_id)),
            'host': host,
            'station': self.node.station_id,
            'robot': self.robot_dropdown.currentText(),
            'urls': list(getattr(self, '_web_urls', [])),
            'version': getattr(self, 'APP_VERSION', ''),
        }

    # ── State and commands (server side of the invoker) ───────────────────
    def _web_state(self):
        """Full page state. Qt thread only."""
        state = build_snapshot(self)
        server = getattr(self, '_web_server', None)
        state['web'] = {
            'gui_id': self._web_card()['gui_id'],
            'urls': list(self._web_urls),
            'clients': len(server.clients()) if server is not None else 0,
        }
        directory = getattr(self, '_web_directory', None)
        state['studios'] = (directory.studios() if directory is not None
                            else [dict(self._web_card(), self=True)])
        return state

    def _web_snapshot(self):
        """Page state for an HTTP thread."""
        return self._web_invoker.call(self._web_state, timeout=2.0)

    def _web_push_state(self):
        """Hand the snapshot to every server that has a browser listening.

        There are two: the remote one (phones, accounts) and the desktop
        session's local one for the web UI (``--web-ui``). Each has its own
        event streams, and the UI's would never see a state after its first
        paint if only the remote server were fed.
        """
        servers = [srv for srv in (getattr(self, '_web_server', None),
                                   getattr(self, '_web_ui_server', None))
                   if srv is not None and srv.stream_count() > 0]
        if not servers:
            return
        try:
            state = self._web_state()
            for server in servers:
                server.publish(state)
        except Exception as exc:                          # pragma: no cover
            self.log_message('WARN', 'Web remote state failed: %s' % exc, 'Web')

    def _web_panels_provider(self, ext_id=None):
        """Answer the web server (its own thread) from the Qt thread."""
        timeout = 5.0
        if ext_id is None:
            return self._web_invoker.call(self._web_panels, timeout=timeout)
        return self._web_invoker.call(lambda: self._web_panel(ext_id), timeout=timeout)

    def _web_dispatch(self, action, args, client):
        """Run a command from an HTTP thread; returns the reply dict."""
        run = lambda: handle_command(self, action, args, client)  # noqa: E731
        timeout = COMMAND_TIMEOUT_S.get(action, DEFAULT_COMMAND_TIMEOUT_S)
        if action == 'estop':
            # Whoever holds control keeps it: the E-stop latch refuses motion,
            # processes and teleop until Reset (stamped first in actions._estop,
            # so commands already on their way are refused too), and the holder
            # is the one who checks the robot and resets.
            return self._web_invoker.call(run, timeout=timeout, expire=False)
        return self._web_invoker.call(run, timeout=timeout)

    def _web_on_event(self, kind, info):
        """Server notifications (HTTP thread) → Studio log (Qt thread)."""
        messages = {
            'control_taken': 'Web remote: %s took control',
            'control_released': 'Web remote: %s released control',
        }
        who = '%s @ %s' % (info.get('name') or 'browser', info.get('addr') or '?')

        def _apply():
            if kind in messages:
                self.log_message('INFO', messages[kind] % who, 'Web')
            self._web_refresh_indicator()
        self._web_invoker.post(_apply)

    def _web_announce(self):
        if getattr(self, '_web_directory', None) is not None:
            self._web_directory.announce()
        if getattr(self, '_web_sync', None) is not None:
            self._web_sync.share()
        self._web_refresh_indicator()

    def _web_revoke_control(self):
        holder = self._web_lease.revoke()
        if holder is not None:
            self.log_message('WARN', 'Web remote: took control back from %s @ %s'
                             % (holder['name'] or 'browser', holder['addr']), 'Web')
        self._web_refresh_indicator()

    # ── Status bar ───────────────────────────────────────────────────────
    def _web_attach_statusbar(self):
        bar = getattr(self, 'statusbar', None)
        if bar is None:
            return
        from fic_studio import theme
        from fic_studio.widgets import StatusLED
        holder = QWidget()
        row = QHBoxLayout(holder)
        row.setContentsMargins(8, 0, 8, 0)
        row.setSpacing(6)
        self._web_led = StatusLED()
        row.addWidget(self._web_led)
        self._web_status_label = QLabel('WEB')
        theme.bind(self._web_status_label, lambda t: (
            'color: %s; font-family: monospace; font-size: 10px;' % t['text_secondary']))
        row.addWidget(self._web_status_label)
        # Just after the connection LEDs, before the stretch.
        bar.insertWidget(1, holder)

    def _web_refresh_indicator(self):
        led = getattr(self, '_web_led', None)
        label = getattr(self, '_web_status_label', None)
        server = getattr(self, '_web_server', None)
        if led is None or label is None:
            return
        if server is None:
            error = getattr(self, '_web_error', '')
            led.set_status('disconnected' if error else 'idle')
            label.setText('WEB off')
            label.setToolTip(error or 'Web remote is turned off in Settings.')
            return
        clients = len(server.clients())
        holder = self._web_lease.holder()
        if holder is not None:
            led.set_status('active')
            label.setText('WEB · %s in control' % (holder['name'] or holder['addr']))
        else:
            led.set_status('connected' if clients else 'idle')
            label.setText('WEB %d' % clients)
        label.setToolTip('\n'.join(self._web_urls))

    # ── Settings card ────────────────────────────────────────────────────
    def _build_web_remote_settings_tab(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        row = QHBoxLayout()
        enable = QCheckBox('Serve the web remote')
        enable.setChecked(bool(self._web_settings()['enabled']))
        row.addWidget(enable)
        row.addSpacing(16)
        row.addWidget(QLabel('Port'))
        port = QSpinBox()
        port.setRange(1024, 65535)
        port.setValue(int(self._web_settings()['port']))
        row.addWidget(port)
        apply_btn = QPushButton('Apply')
        row.addWidget(apply_btn)
        row.addStretch(1)
        lay.addLayout(row)

        urls = QLabel()
        urls.setTextInteractionFlags(Qt.TextSelectableByMouse)
        urls.setWordWrap(True)
        lay.addWidget(urls)

        control_row = QHBoxLayout()
        clients = QLabel()
        clients.setWordWrap(True)
        control_row.addWidget(clients, 1)
        revoke = QPushButton('Take control back')
        revoke.setToolTip('Remove control from the browser that holds it.')
        control_row.addWidget(revoke)
        reset = QPushButton('Reset admin password…')
        reset.setToolTip('Set a new web remote admin password, for a lab that is '
                         'locked out. It reaches the other Studios too.')
        control_row.addWidget(reset)
        lay.addLayout(control_row)

        studios = QLabel()
        studios.setWordWrap(True)
        studios.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lay.addWidget(studios)

        hint = QLabel('Open a URL above on a phone on the same network. Anyone who can '
                      'reach this port can command the robot, one browser at a time; '
                      'any browser can E-stop. Turn it off on networks you do not trust.')
        hint.setObjectName('HintLabel')
        hint.setWordWrap(True)
        lay.addWidget(hint)

        def refresh():
            server = getattr(self, '_web_server', None)
            if server is None:
                urls.setText(getattr(self, '_web_error', '') or 'Not serving.')
                clients.setText('')
                studios.setText('')
                revoke.setEnabled(False)
                return
            urls.setText('<br>'.join('<b>%s</b> &nbsp;<span>%s</span>' % (
                u, network_label(u.split('//')[1].split(':')[0])) for u in self._web_urls))
            connected = server.clients()
            holder = self._web_lease.holder()
            text = '%d browser%s connected' % (len(connected), '' if len(connected) == 1 else 's')
            if holder is not None:
                text += ' · %s @ %s in control' % (holder['name'] or 'browser', holder['addr'])
            clients.setText(text)
            revoke.setEnabled(holder is not None)
            directory = getattr(self, '_web_directory', None)
            others = [s for s in (directory.studios() if directory else []) if not s.get('self')]
            studios.setText('Other Studios: ' + ('; '.join(
                '%s — %s' % (s['label'], (s.get('urls') or ['?'])[0]) for s in others)
                if others else 'none found on this ROS network'))

        def on_enable(checked):
            self._web_save_setting('enabled', bool(checked))
            if checked:
                self._web_start()
            else:
                self._web_stop()
            refresh()

        def on_apply():
            self._web_save_setting('port', int(port.value()))
            if getattr(self, '_web_server', None) is not None:
                self._web_stop()
                self._web_start()
            refresh()

        def on_revoke():
            self._web_revoke_control()
            refresh()

        enable.toggled.connect(on_enable)
        apply_btn.clicked.connect(on_apply)
        revoke.clicked.connect(on_revoke)
        reset.clicked.connect(self._web_reset_admin_password)

        timer = QTimer(page)
        timer.timeout.connect(lambda: refresh() if page.isVisible() else None)
        timer.start(1000)
        refresh()
        return page
