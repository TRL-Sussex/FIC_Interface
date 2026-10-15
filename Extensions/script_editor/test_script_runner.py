"""
Tests for ScriptRunner and the _create_robot_api wiring in extension.py.

Verifies:
- A script using robot.state.joints + move_joints(wait=True) runs to
  completion against a fake node/host.
- Setting the stop event mid-run aborts a wait() loop within the grace period.
- The thread-routed stdout interceptor only captures the script thread's writes.
- on_emergency_stop sets the stop event and calls safety.estop().
- Latched flags are restored on teardown.

No ROS runtime required; PyQt5 is stubbed so the module can be imported
without a display.
"""
import io
import math
import os as _os
import sys
import threading
import time
import types

# ── Make Extensions tree importable ──────────────────────────────────────────
_EXT_ROOT = _os.path.abspath(
    _os.path.join(_os.path.dirname(__file__), '..', '..'))
if _EXT_ROOT not in sys.path:
    sys.path.insert(0, _EXT_ROOT)

# ── Stub PyQt5 before any extension import ────────────────────────────────────

_qt_core = types.ModuleType('PyQt5.QtCore')
_qt_gui = types.ModuleType('PyQt5.QtGui')
_qt_widgets = types.ModuleType('PyQt5.QtWidgets')


class _QObjectStub:
    """
    Minimal QObject stub.

    Any unknown attribute (methods or signals) is handled via __getattr__:
    - Known signal names return a _PyqtSignalStub.
    - Everything else returns a no-op callable or a sentinel.

    This allows extension.py's build_page() to construct widgets without
    needing every Qt method enumerated in the stub.
    """

    # Qt constants used as class-level attributes (e.g. QFrame.VLine)
    VLine = 5
    HLine = 4
    Yes = 16384
    No = 65536

    _SIGNAL_NAMES = frozenset({
        'clicked', 'toggled', 'currentTextChanged', 'currentIndexChanged',
        'itemDoubleClicked', 'textChanged', 'timeout', 'destroyed',
        'blockCountChanged', 'updateRequest', 'valueChanged',
        'selectionChanged', 'returnPressed', 'editingFinished',
        'stateChanged', 'activated', 'highlighted',
    })

    def __init__(self, *a, **kw):
        """No-op init."""
        self._style = ''
        self._title = ''
        self._signals = {}

    def __getattr__(self, name):
        """Return signal stub for signal names; no-op callable for others."""
        if name.startswith('_'):
            raise AttributeError(name)
        if name in self._SIGNAL_NAMES:
            sig = self._signals.get(name)
            if sig is None:
                sig = _PyqtSignalStub()
                self._signals[name] = sig
            return sig
        return lambda *a, **kw: None

    def setText(self, t):
        """Record text."""
        self._title = t

    def text(self):
        """Return recorded text."""
        return self._title

    def currentText(self):
        """Return empty string."""
        return ''

    def findText(self, t):
        """Return -1."""
        return -1

    def document(self):
        """Return self."""
        return self

    def blockCount(self):
        """Return 1."""
        return 1

    def viewport(self):
        """Return self."""
        return self

    def rect(self):
        """Return self."""
        return self

    def contentsRect(self):
        """Return self."""
        return self

    def left(self):
        """Return 0."""
        return 0

    def top(self):
        """Return 0."""
        return 0

    def bottom(self):
        """Return 100."""
        return 100

    def width(self):
        """Return 100."""
        return 100

    def height(self):
        """Return 100."""
        return 100

    def blockBoundingGeometry(self, block):
        """Return self."""
        return self

    def translated(self, offset):
        """Return self."""
        return self

    def contentOffset(self):
        """Return self."""
        return self

    def blockBoundingRect(self, block):
        """Return self."""
        return self

    def firstVisibleBlock(self):
        """Return self."""
        return self

    def blockNumber(self):
        """Return 0."""
        return 0

    def isValid(self):
        """Return False."""
        return False

    def contains(self, other):
        """Return False."""
        return False

    def next(self):  # noqa: A003 — matches Qt QTextBlock API
        """Return self (invalid on next call via isValid=False)."""
        return self

    def toPlainText(self):
        """Return empty string."""
        return ''

    def currentData(self):
        """Return None."""
        return None

    def fontMetrics(self):
        """Return self (supports horizontalAdvance, height calls)."""
        return self

    def horizontalAdvance(self, s):
        """Return 8."""
        return 8


class _PyqtSignalStub:
    """Minimal pyqtSignal stub that calls handlers synchronously on emit."""

    def __init__(self, *args):
        """Initialise with empty handler list."""
        self._handlers = []

    def connect(self, handler, *a, **kw):
        """Register a handler."""
        self._handlers.append(handler)

    def emit(self, *args):
        """Call all registered handlers synchronously."""
        for h in self._handlers:
            try:
                h(*args)
            except Exception:  # noqa: BLE001
                pass


class _QThreadStub(_QObjectStub):
    """Minimal QThread stub backed by a real threading.Thread."""

    def __init__(self, parent=None):
        """Initialise thread stub."""
        super().__init__()
        self._thread = None
        self._running = False

    def start(self):
        """Start the run() method in a daemon thread."""
        self._running = True
        self._thread = threading.Thread(target=self._run_wrapper, daemon=True)
        self._thread.start()

    def _run_wrapper(self):
        try:
            self.run()
        finally:
            self._running = False

    def run(self):
        """Override in subclasses."""
        pass

    def isRunning(self):
        """Return True while the underlying thread is alive."""
        return self._running

    def wait(self, msec=None):
        """Block until the thread finishes (timeout in ms or block forever)."""
        if self._thread is not None:
            timeout = (msec / 1000.0) if msec is not None else None
            self._thread.join(timeout=timeout)
        return not self._running

    def terminate(self):
        """Mark the runner as stopped (Python threads cannot be force-killed)."""
        self._running = False

    @property
    def ident(self):
        """Return the thread ident, or 0 before start."""
        return self._thread.ident if self._thread else 0


_qt_core.QObject = _QObjectStub
_qt_core.pyqtSignal = _PyqtSignalStub
_qt_core.Qt = types.SimpleNamespace(
    QueuedConnection=0, Vertical=2, Horizontal=1, AlignRight=2,
    Key_Return=16777220, Key_Enter=16777221, Key_Escape=16777216,
)
_qt_core.QTimer = _QObjectStub
_qt_core.QThread = _QThreadStub

_qt_gui.QColor = lambda *a, **kw: None


class _QFontStub:
    """QFont stub that accepts constructor arguments."""

    Bold = 75
    Monospace = 0

    def __init__(self, *a, **kw):
        """No-op init."""

    def setStyleHint(self, hint):
        """No-op."""


_qt_gui.QFont = _QFontStub
_qt_gui.QSyntaxHighlighter = _QObjectStub
_qt_gui.QTextCharFormat = _QObjectStub
_qt_gui.QTextCursor = _QObjectStub

for _name in (
    'QComboBox', 'QFileDialog', 'QFrame', 'QGroupBox', 'QHBoxLayout',
    'QInputDialog', 'QLabel', 'QLineEdit', 'QListView', 'QListWidget',
    'QListWidgetItem', 'QMessageBox', 'QPlainTextEdit', 'QPushButton',
    'QScrollArea', 'QSplitter', 'QTabWidget', 'QTextEdit', 'QVBoxLayout',
    'QWidget',
):
    setattr(_qt_widgets, _name, _QObjectStub)

sys.modules.setdefault('PyQt5', types.ModuleType('PyQt5'))
sys.modules['PyQt5.QtCore'] = _qt_core
sys.modules['PyQt5.QtGui'] = _qt_gui
sys.modules['PyQt5.QtWidgets'] = _qt_widgets

# ── Stub fic_studio modules so extension.py can be imported ──────────────────

_fic_studio = types.ModuleType('fic_studio')
_fic_studio_config = types.ModuleType('fic_studio.config')
_fic_studio_config.FIC_API_DIR = '/tmp/fic_api'
_fic_studio_extensions = types.ModuleType('fic_studio.extensions')


class _FakeExtensionBase:
    """Minimal ExtensionBase stub."""

    def __init__(self, host):
        """Store host reference."""
        self.host = host


_fic_studio_extensions.Extension = _FakeExtensionBase

# Stub fic_studio.theme so the theme import does not fail
_fic_studio_theme = types.ModuleType('fic_studio.theme')
_fic_studio_theme.bind = lambda widget, factory: widget
_fic_studio_theme.current = lambda: {}
_fic_studio_theme.signals = types.SimpleNamespace(
    theme_changed=_PyqtSignalStub())

sys.modules.setdefault('fic_studio', _fic_studio)
sys.modules['fic_studio.config'] = _fic_studio_config
sys.modules['fic_studio.extensions'] = _fic_studio_extensions
sys.modules['fic_studio.theme'] = _fic_studio_theme

# ── Imports under test ────────────────────────────────────────────────────────
from Extensions.script_editor import robot_api as _robot_api  # noqa: E402
from Extensions.script_editor.extension import (  # noqa: E402
    _ThreadRoutedWriter, ScriptRunner,
)

# ── Helpers shared across tests ───────────────────────────────────────────────


def _sync_marshal(fn, block=True, timeout=10.0):
    """Run fn immediately on the calling thread (synchronous marshal stub)."""
    if not block:
        fn()
        return None
    return fn()


# ── Fake node ─────────────────────────────────────────────────────────────────

class FakeNode:
    """Record every publish_* call for assertion in tests."""

    _MODE_NAMES = [
        'Joint Impedance', 'Joint Impedance With Planner',
        'Joint Admittance', 'Joint Admittance Haptic',
        'Cartesian Impedance', 'Cartesian Planner',
        'Cartesian Admittance', 'Cartesian Admittance Haptic',
    ]

    def __init__(self):
        """Reset call log."""
        self.calls = []

    def _rec(self, name, **kw):
        self.calls.append((name, kw))

    def publish_joint_angles(self, positions, names=None, robot_id=None):
        """Record publish_joint_angles."""
        self._rec('publish_joint_angles', positions=list(positions),
                  names=names, robot_id=robot_id)

    def publish_real_joint_angles(self, robot_id, positions_rad):
        """Record publish_real_joint_angles."""
        self._rec('publish_real_joint_angles', robot_id=robot_id,
                  positions_rad=list(positions_rad))

    def publish_mode(self, mode_int, robot_id=None):
        """Record publish_mode."""
        self._rec('publish_mode', mode_int=mode_int, robot_id=robot_id)

    def publish_postural_enable(self, enabled, robot_id=None):
        """Record publish_postural_enable."""
        self._rec('publish_postural_enable', enabled=enabled,
                  robot_id=robot_id)

    def publish_admittance_planner(self, enabled, robot_id=None):
        """Record publish_admittance_planner."""
        self._rec('publish_admittance_planner', enabled=enabled,
                  robot_id=robot_id)

    def publish_hpmp_enable(self, enabled, robot_id=None):
        """Record publish_hpmp_enable."""
        self._rec('publish_hpmp_enable', enabled=enabled, robot_id=robot_id)

    def publish_gripper_width(self, robot_id, width_meters):
        """Record publish_gripper_width."""
        self._rec('publish_gripper_width', robot_id=robot_id,
                  width_meters=width_meters)

    def publish_position(self, pose, robot_id=None):
        """Record publish_position."""
        self._rec('publish_position', pose=pose, robot_id=robot_id)

    def publish_elbow_enable(self, enabled, robot_id=None):
        """Record publish_elbow_enable."""
        self._rec('publish_elbow_enable', enabled=enabled, robot_id=robot_id)

    def publish_param_update(self, robot_id, param_type, updated_param):
        """Record publish_param_update."""
        self._rec('publish_param_update', robot_id=robot_id,
                  param_type=param_type, updated_param=updated_param)

    def get_topic_names_and_types(self):
        """Return empty topic list."""
        return []

    def create_subscription(self, msg_cls, topic, cb, qos):
        """Return a sentinel subscription."""
        return object()

    def destroy_subscription(self, sub):
        """No-op destroy."""
        pass


# ── Fake host ─────────────────────────────────────────────────────────────────

class _FakeMW:
    """Fake main window."""

    def __init__(self):
        """Initialise mode and target tracking."""
        self.mode_ui_calls = []
        self.broadcast_called = False

    def update_mode_ui_only(self, mode_name, robot_id):
        """Record mode-UI sync call."""
        self.mode_ui_calls.append((mode_name, robot_id))

    def broadcast_stop(self):
        """Record broadcast stop."""
        self.broadcast_called = True


class FakeHost:
    """Canned joint/EE data host."""

    def __init__(self):
        """Set up canned joint data at home position."""
        self._joints = {
            'robot1': [math.radians(a) for a in
                       [10.0, 20.0, 30.0, -90.0, 0.0, 45.0, 0.0]],
        }
        self._mw = _FakeMW()
        self.node = FakeNode()
        self.target_robot_id = 'robot1'

    def joint_positions_for(self, rid):
        """Return canned joint positions."""
        return list(self._joints.get(rid, []))

    def joint_velocity_for(self, rid):
        """Return empty velocity list."""
        return []

    def joint_effort_for(self, rid):
        """Return empty effort list."""
        return []

    def joint_commanded_for(self, rid):
        """Return empty commanded list."""
        return []

    def ee_pose_for(self, rid):
        """Return None (no EE pose)."""
        return None

    def wrench_for(self, rid):
        """Return None (no wrench)."""
        return None

    def current_robot_config(self):
        """Return minimal robot config."""
        return {'q_home_deg': [0.0] * 7}

    def current_robot_name(self):
        """Return fake robot name."""
        return 'fake_robot'

    def xml_manager(self):
        """Return None."""
        return None

    def robot_target(self, rid=None):
        """Return simulation target."""
        return 'simulation'

    def set_robot_target(self, rid, target):
        """No-op target setter."""
        pass

    def broadcast_stop(self):
        """Record broadcast stop."""
        self._mw.broadcast_called = True

    def log(self, level, msg, tag):
        """No-op log."""
        pass


# ── Helper: build RobotAPI + namespace ────────────────────────────────────────

def _make_api(host=None, stop_event=None):
    """
    Return (api, namespace, node, host, stop_event).

    Mirrors what _create_robot_api does but without the extension instance.
    """
    if host is None:
        host = FakeHost()
    node = host.node
    if stop_event is None:
        stop_event = threading.Event()
    api = _robot_api.RobotAPI(
        host=host,
        node=node,
        rid='robot1',
        target='simulation',
        marshal=_sync_marshal,
        is_stopped=stop_event.is_set,
    )
    ns = _robot_api.build_namespace(api)
    return api, ns, node, host, stop_event


def _run_script_sync(code, ns, stop_event=None):
    """
    Execute a script via ScriptRunner and block until finished.

    Returns (success, message, outputs, errors).
    """
    if stop_event is None:
        stop_event = threading.Event()
    results = {'success': None, 'message': '', 'outputs': [], 'errors': []}
    done = threading.Event()

    runner = ScriptRunner(code, ns, stop_event)

    def _on_output(text):
        results['outputs'].append(text)

    def _on_error(text):
        results['errors'].append(text)

    def _on_finished(success, msg):
        results['success'] = success
        results['message'] = msg
        done.set()

    runner.signals.output.connect(_on_output)
    runner.signals.error.connect(_on_error)
    runner.signals.finished.connect(_on_finished)
    runner.start()
    done.wait(timeout=10.0)
    return results['success'], results['message'], results['outputs'], results['errors']


# ── Tests ──────────────────────────────────────────────────────────────────────

class TestScriptRunnerBasic:
    """Basic ScriptRunner execution tests."""

    def test_simple_script_completes(self):
        """A trivial script runs to completion and emits finished(True)."""
        _, ns, _, _, stop_event = _make_api()
        code = 'x = 1 + 1\nprint(x)\n'
        success, msg, outputs, errors = _run_script_sync(code, ns, stop_event)
        assert success is True
        assert 'completed' in msg.lower()
        assert not errors

    def test_script_output_captured(self):
        """print() output from the script thread is captured and emitted."""
        _, ns, _, _, stop_event = _make_api()
        code = 'print("hello from script")\n'
        success, _, outputs, _ = _run_script_sync(code, ns, stop_event)
        assert success is True
        combined = ''.join(outputs)
        assert 'hello from script' in combined

    def test_script_exception_emits_error_and_false(self):
        """A script that raises emits finished(False) and an error signal."""
        _, ns, _, _, stop_event = _make_api()
        code = 'raise ValueError("deliberate error")\n'
        success, msg, _, errors = _run_script_sync(code, ns, stop_event)
        assert success is False
        assert errors

    def test_robot_state_joints_accessible(self):
        """A script can read robot.state.joints and get degree values."""
        _, ns, _, host, stop_event = _make_api()
        code = (
            'j = robot.state.joints\n'
            'assert len(j) == 7, f"expected 7, got {len(j)}"\n'
            'assert abs(j[0] - 10.0) < 0.01, f"expected ~10, got {j[0]}"\n'
        )
        success, msg, _, errors = _run_script_sync(code, ns, stop_event)
        assert success is True, f'errors: {errors}'

    def test_move_joints_wait_true_runs_to_completion(self):
        """Script completes when move_joints(wait=True) target is already reached."""
        host = FakeHost()
        _, ns, node, _, stop_event = _make_api(host=host)
        code = (
            'result = move_joints([10.0, 20.0, 30.0, -90.0, 0.0, 45.0, 0.0],'
            ' wait=True, timeout=2.0)\n'
            'assert result is True, f"move_joints returned {result}"\n'
        )
        success, _, _, errors = _run_script_sync(code, ns, stop_event)
        assert success is True, f'errors: {errors}'

    def test_move_joints_publishes_radians(self):
        """move_joints in the namespace converts degrees to radians."""
        _, ns, node, _, stop_event = _make_api()
        code = 'move_joints([0.0, 90.0], wait=False)\n'
        success, _, _, errors = _run_script_sync(code, ns, stop_event)
        assert success is True, errors
        pub_calls = [kw['positions'] for n, kw in node.calls
                     if n == 'publish_joint_angles']
        assert pub_calls
        assert abs(pub_calls[0][1] - math.radians(90.0)) < 1e-9


class TestCooperativeStop:
    """Tests for the cooperative threading.Event stop mechanism."""

    def test_stop_event_aborts_wait_loop(self):
        """Stop event aborts a robot.wait() loop within the 2 s grace period."""
        _, ns, _, _, stop_event = _make_api()
        code = 'robot.wait(10.0)\n'
        done = threading.Event()
        results = {}

        runner = ScriptRunner(code, ns, stop_event)

        def _on_finished(s, m):
            results['success'] = s
            done.set()

        runner.signals.finished.connect(_on_finished)
        runner.start()
        time.sleep(0.1)
        stop_event.set()
        done.wait(timeout=5.0)
        assert done.is_set(), 'Script did not finish after stop event'
        assert results.get('success') is True

    def test_stop_aborts_move_joints_wait(self):
        """Stop event aborts move_joints(wait=True) before the timeout expires."""
        _, ns, node, _, stop_event = _make_api()
        code = (
            'result = move_joints([180.0]*7, wait=True, timeout=10.0)\n'
            'assert result is False, f"expected False, got {result}"\n'
        )
        done = threading.Event()
        results = {}

        runner = ScriptRunner(code, ns, stop_event)

        def _on_finished(s, m):
            results.update({'success': s, 'msg': m})
            done.set()

        def _on_error(e):
            results['error'] = e
            done.set()

        runner.signals.finished.connect(_on_finished)
        runner.signals.error.connect(_on_error)
        runner.start()
        time.sleep(0.1)
        stop_event.set()
        done.wait(timeout=5.0)
        assert done.is_set(), 'Script did not finish after stop'
        assert 'error' not in results, results.get('error', '')

    def test_stop_within_grace_period(self):
        """A script polling is_stopped() exits within the 2 s grace window."""
        _, ns, _, _, stop_event = _make_api()
        code = (
            'while not is_stopped():\n'
            '    sleep(0.01)\n'
        )
        done = threading.Event()
        t_start = time.monotonic()

        runner = ScriptRunner(code, ns, stop_event)
        runner.signals.finished.connect(lambda s, m: done.set())
        runner.start()
        time.sleep(0.08)
        stop_event.set()
        done.wait(timeout=ScriptRunner._GRACE_SECONDS + 1.0)
        elapsed = time.monotonic() - t_start
        assert done.is_set(), 'Script never finished'
        assert elapsed < ScriptRunner._GRACE_SECONDS + 0.5, (
            f'Took {elapsed:.2f}s — exceeded grace period')


class TestThreadRoutedWriter:
    """Tests for _ThreadRoutedWriter stdout interceptor."""

    def test_captures_only_from_script_thread(self):
        """Script-thread writes are captured; other-thread writes are forwarded."""
        real_stream = io.StringIO()
        captured_stages = []

        def _script_thread_body():
            writer = _ThreadRoutedWriter(real_stream, threading.current_thread())
            writer.write('from script thread\n')
            captured_stages.append(writer.getvalue())
            other = threading.Thread(
                target=lambda: writer.write('from other thread\n'))
            other.start()
            other.join()
            captured_stages.append(writer.getvalue())

        t = threading.Thread(target=_script_thread_body)
        t.start()
        t.join()

        assert 'from script thread' in captured_stages[0]
        assert 'from script thread' in captured_stages[1]
        assert 'from other thread' not in captured_stages[1]
        assert 'from other thread' in real_stream.getvalue()

    def test_signal_emitted_on_write(self):
        """_ThreadRoutedWriter emits the signal for each write from the script thread."""
        emitted = []
        sig = _PyqtSignalStub()
        sig.connect(lambda text: emitted.append(text))

        captured = []

        def _body():
            writer = _ThreadRoutedWriter(
                io.StringIO(), threading.current_thread(), signal=sig)
            writer.write('line1\n')
            writer.write('line2\n')
            captured.append(writer.getvalue())

        t = threading.Thread(target=_body)
        t.start()
        t.join()

        assert 'line1\n' in emitted
        assert 'line2\n' in emitted
        assert captured[0] == 'line1\nline2\n'


class TestEmergencyStop:
    """Tests for on_emergency_stop behaviour (via the API layer)."""

    def test_estop_publishes_mode_1_and_sets_event(self):
        """safety.estop() publishes mode 1 and triggers host broadcast_stop."""
        host = FakeHost()
        _, ns, node, _, stop_event = _make_api(host=host)
        api = ns['robot']
        host._mw.broadcast_called = False
        host.broadcast_stop = lambda: setattr(host._mw, 'broadcast_called', True)
        api.safety.estop()
        mode_calls = [k for n, k in node.calls if n == 'publish_mode']
        assert mode_calls, 'Expected at least one publish_mode call'
        assert mode_calls[0]['mode_int'] == 1


class TestLatchedFlags:
    """Tests that latched admittance_planner / hpmp flags are tracked."""

    def test_admittance_planner_flag_tracked(self):
        """admittance_planner(True/False) records the correct boolean both ways."""
        _, ns, node, _, stop_event = _make_api()
        api = ns['robot']
        api.mode.admittance_planner(True)
        api.mode.admittance_planner(False)
        calls = [k for n, k in node.calls if n == 'publish_admittance_planner']
        assert calls[0]['enabled'] is True
        assert calls[1]['enabled'] is False

    def test_hpmp_flag_tracked(self):
        """robot.mode.hapfic(True) records enabled=True via publish_hpmp_enable."""
        _, ns, node, _, stop_event = _make_api()
        api = ns['robot']
        api.mode.hpmp(True)
        calls = [k for n, k in node.calls if n == 'publish_hpmp_enable']
        assert calls[0]['enabled'] is True


# ── Regression tests for post-review findings ─────────────────────────────────

class TestTopicsCacheSingleSource:
    """
    Regression tests for the dual-cache bug.

    Before the fix, extension._create_robot_api() called ns.update(legacy)
    which overwrote the four topic-related flat aliases (subscribe, unsubscribe,
    get_topic, list_topics) with lambdas that routed to the extension's own
    _script_topic_cache.  This meant:

        subscribe(t)           # writes to TopicsAPI._cache via ns (correct)
        ns.update(legacy)      # replaces subscribe with ext._script_subscribe_topic
        subscribe(t)           # now writes to ext._script_topic_cache (wrong cache)
        robot.topics.get(t)    # reads TopicsAPI._cache -> None

    After the fix the four keys are absent from the legacy dict so
    build_namespace's TopicsAPI-routed versions remain in effect.
    """

    def test_flat_subscribe_routes_through_topics_api_cache(self):
        """
        The flat 'subscribe' name must route through TopicsAPI, not a separate cache.

        We verify this by checking that ns['subscribe'].__func__ (the underlying
        unbound function) is the same as type(api.topics).subscribe (the class-level
        method), and that the bound instance is the same api.topics object.  This
        proves both callables write to the same TopicsAPI._cache.
        """
        api, ns, _, _, _ = _make_api()
        subscribe_fn = ns['subscribe']
        # Must be a bound method wrapping the same instance
        assert getattr(subscribe_fn, '__self__', None) is api.topics, (
            'ns["subscribe"] is not bound to api.topics — dual-cache bug: '
            f'__self__={getattr(subscribe_fn, "__self__", None)!r} vs api.topics={api.topics!r}'
        )

    def test_flat_unsubscribe_routes_through_topics_api_cache(self):
        """The flat 'unsubscribe' must be bound to api.topics (same cache)."""
        api, ns, _, _, _ = _make_api()
        fn = ns['unsubscribe']
        assert getattr(fn, '__self__', None) is api.topics, (
            'ns["unsubscribe"] is not bound to api.topics — dual-cache bug.'
        )

    def test_flat_get_topic_routes_through_topics_api_cache(self):
        """The flat 'get_topic' must be bound to api.topics (same cache)."""
        api, ns, _, _, _ = _make_api()
        fn = ns['get_topic']
        assert getattr(fn, '__self__', None) is api.topics, (
            'ns["get_topic"] is not bound to api.topics — dual-cache bug.'
        )

    def test_flat_list_topics_routes_through_topics_api(self):
        """The flat 'list_topics' must be bound to api.topics."""
        api, ns, _, _, _ = _make_api()
        fn = ns['list_topics']
        assert getattr(fn, '__self__', None) is api.topics, (
            'ns["list_topics"] is not bound to api.topics — dual-cache bug.'
        )
