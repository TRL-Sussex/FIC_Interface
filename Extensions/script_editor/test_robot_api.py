"""
Tests for Extensions/script_editor/robot_api.py.

Runs with a fake node (records publish calls) and a fake host (returns canned
joint / EE / wrench data) — no ROS runtime required.  All tests are standard
pytest; run from fic_ws/src/fic_studio.
"""
import math
import os as _os
import sys
import threading
import time
import types

import pytest

# ── Make sure the Extensions tree is importable ───────────────────────────────
# (When run from fic_ws/src/fic_studio the Extensions/ dir is two levels up.)
_EXT_ROOT = _os.path.abspath(
    _os.path.join(_os.path.dirname(__file__), '..', '..'))
if _EXT_ROOT not in sys.path:
    sys.path.insert(0, _EXT_ROOT)

# ── Stub out PyQt5 so the module can be imported without a display ─────────────
_qt_core = types.ModuleType('PyQt5.QtCore')


class _QObjectStub:
    """Minimal QObject stub."""

    def __init__(self, *a, **kw):
        """No-op init."""
        pass


class _PyqtSignalStub:
    """Minimal pyqtSignal stub — records emit calls."""

    def __init__(self, *args):
        """Initialise with empty handler list."""
        self._handlers = []

    def connect(self, handler, *a, **kw):
        """Register a handler."""
        self._handlers.append(handler)

    def emit(self, *args):
        """Call all registered handlers synchronously."""
        for h in self._handlers:
            h(*args)


_qt_core.QObject = _QObjectStub
_qt_core.pyqtSignal = _PyqtSignalStub
_qt_core.Qt = types.SimpleNamespace(QueuedConnection=0)
sys.modules.setdefault('PyQt5', types.ModuleType('PyQt5'))
sys.modules['PyQt5.QtCore'] = _qt_core

from Extensions.script_editor import robot_api  # noqa: E402,I100


# ── Synchronous marshal (runs fn immediately in the same thread) ──────────────

def _sync_marshal(fn, block=True, timeout=10.0):
    """Fake marshal: call fn() immediately and return its result."""
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
        """Record a publish_joint_angles call."""
        self._rec('publish_joint_angles', positions=list(positions),
                  names=names, robot_id=robot_id)

    def publish_real_joint_angles(self, robot_id, positions_rad):
        """Record a publish_real_joint_angles call."""
        self._rec('publish_real_joint_angles', robot_id=robot_id,
                  positions_rad=list(positions_rad))

    def publish_position(self, pose, robot_id=None):
        """Record a publish_position call."""
        self._rec('publish_position', pose=pose, robot_id=robot_id)

    def publish_gripper_width(self, robot_id, width_meters):
        """Record a publish_gripper_width call."""
        self._rec('publish_gripper_width', robot_id=robot_id,
                  width_meters=width_meters)

    def publish_mode(self, mode_int, robot_id=None):
        """Record a publish_mode call."""
        self._rec('publish_mode', mode_int=mode_int, robot_id=robot_id)

    def publish_postural_enable(self, enabled, robot_id=None):
        """Record a publish_postural_enable call."""
        self._rec('publish_postural_enable', enabled=enabled,
                  robot_id=robot_id)

    def publish_elbow_enable(self, enabled, robot_id=None):
        """Record a publish_elbow_enable call."""
        self._rec('publish_elbow_enable', enabled=enabled, robot_id=robot_id)

    def publish_hpmp_enable(self, enabled, robot_id=None):
        """Record a publish_hpmp_enable call."""
        self._rec('publish_hpmp_enable', enabled=enabled, robot_id=robot_id)

    def publish_admittance_planner(self, enabled, robot_id=None):
        """Record a publish_admittance_planner call."""
        self._rec('publish_admittance_planner', enabled=enabled,
                  robot_id=robot_id)

    def publish_desired_wrench(self, wrench, robot_id=None):
        """Record a publish_desired_wrench call."""
        self._rec('publish_desired_wrench', wrench=list(wrench), robot_id=robot_id)

    def publish_param_update(self, robot_id, param_type, updated_param):
        """Record a publish_param_update call."""
        self._rec('publish_param_update', robot_id=robot_id,
                  param_type=param_type, updated_param=updated_param)

    def get_topic_names_and_types(self):
        """Return an empty topic list."""
        return []

    def create_subscription(self, msg_cls, topic, cb, qos):
        """Return a dummy subscription sentinel."""
        return object()

    def destroy_subscription(self, sub):
        """No-op subscription teardown."""
        pass


# ── Fake host ─────────────────────────────────────────────────────────────────

class _FakeMW:
    """Fake main window with a recorded update_mode_ui_only."""

    def __init__(self):
        """Initialise call log."""
        self.mode_ui_calls = []
        self.last_target = None
        self.broadcast_called = False

    def update_mode_ui_only(self, mode_name, robot_id):
        """Record mode-UI sync call."""
        self.mode_ui_calls.append((mode_name, robot_id))

    def _set_control_target(self, target):
        """Record control-target change."""
        self.last_target = target

    def broadcast_stop(self):
        """Record broadcast stop."""
        self.broadcast_called = True


class FakeHost:
    """Canned state host — returns preset joint/EE/wrench data."""

    def __init__(self):
        """Set up canned joint and wrench data."""
        # Joint positions in radians
        self._joints = {
            'robot1': [math.radians(10.0), math.radians(20.0),
                       math.radians(30.0), math.radians(-90.0),
                       0.0, math.radians(45.0), 0.0],
        }
        self._wrench = {'robot1': (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)}
        self._mw = _FakeMW()

    def joint_positions_for(self, rid):
        """Return canned joint positions for rid."""
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
        """Return None (no EE pose in the default fake)."""
        return None

    def wrench_for(self, rid):
        """Return canned wrench for rid."""
        return self._wrench.get(rid)

    def current_robot_config(self):
        """Return a minimal robot config dict."""
        return {'q_home_deg': [0.0] * 7}

    def current_robot_name(self):
        """Return a fake robot name."""
        return 'fake_robot'

    def xml_manager(self):
        """Return None (no XML in tests)."""
        return None

    def robot_target(self, rid=None):
        """Return simulation as the default target."""
        return 'simulation'

    def set_robot_target(self, rid, target):
        """Record target change."""
        self._mw.last_target = target

    def broadcast_stop(self):
        """Record broadcast stop."""
        self._mw.broadcast_called = True


# ── Fixture ───────────────────────────────────────────────────────────────────

@pytest.fixture()
def api():
    """Return a RobotAPI wired to fake node and host, with a synchronous marshal."""
    node = FakeNode()
    host = FakeHost()
    stop_event = threading.Event()
    return robot_api.RobotAPI(
        host=host,
        node=node,
        rid='robot1',
        target='simulation',
        marshal=_sync_marshal,
        is_stopped=stop_event.is_set,
    ), node, host, stop_event


# ── StateAPI tests ────────────────────────────────────────────────────────────

class TestStateAPI:
    """Tests for StateAPI property reads."""

    def test_joints_returns_degrees(self, api):
        """state.joints converts radians from the host to degrees."""
        rapi, node, host, _ = api
        joints = rapi.state.joints
        assert len(joints) == 7
        assert abs(joints[0] - 10.0) < 1e-9
        assert abs(joints[1] - 20.0) < 1e-9
        assert abs(joints[3] - (-90.0)) < 1e-9

    def test_joints_empty_when_no_data(self, api):
        """state.joints is empty when the host returns nothing."""
        rapi, node, host, _ = api
        host._joints = {}
        assert rapi.state.joints == []

    def test_wrench_returns_tuple(self, api):
        """state.wrench returns the (Fx, Fy, Fz, Tx, Ty, Tz) tuple."""
        rapi, _, host, _ = api
        w = rapi.state.wrench
        assert w == (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)

    def test_ee_pose_empty_when_none(self, api):
        """state.ee_pose returns [] when host has no EE pose."""
        rapi, _, _, _ = api
        assert rapi.state.ee_pose == []


# ── MotionAPI tests ───────────────────────────────────────────────────────────

class TestMotionAPI:
    """Tests for MotionAPI move_joints, move_cartesian, gripper commands."""

    def test_move_joints_sim_publishes_radians(self, api):
        """move_joints publishes via publish_joint_angles with radian values."""
        rapi, node, _, _ = api
        rapi.motion.move_joints([0.0, 90.0], wait=False)
        assert len(node.calls) == 1
        name, kw = node.calls[0]
        assert name == 'publish_joint_angles'
        pos = kw['positions']
        assert len(pos) == 2
        assert abs(pos[0] - 0.0) < 1e-9
        assert abs(pos[1] - math.radians(90.0)) < 1e-9
        assert kw['robot_id'] == 'robot1'

    def test_move_joints_hardware_uses_real_publisher(self, api):
        """move_joints routes to publish_real_joint_angles when target=hardware."""
        rapi, node, _, _ = api
        rapi.motion._target = 'hardware'
        rapi.motion.move_joints([0.0, 45.0], wait=False)
        assert node.calls[0][0] == 'publish_real_joint_angles'
        kw = node.calls[0][1]
        assert abs(kw['positions_rad'][1] - math.radians(45.0)) < 1e-9

    def test_move_joints_wait_true_returns_true_on_match(self, api):
        """move_joints(..., wait=True) returns True when joints reach target."""
        rapi, node, host, _ = api
        # Fake host already holds these positions
        target_deg = [10.0, 20.0, 30.0, -90.0, 0.0, 45.0, 0.0]
        result = rapi.motion.move_joints(target_deg, wait=True, timeout=1.0)
        assert result is True

    def test_move_joints_wait_returns_false_on_timeout(self, api):
        """move_joints(..., wait=True) returns False when target is never reached."""
        rapi, node, host, _ = api
        result = rapi.motion.move_joints(
            [180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0],
            wait=True, timeout=0.12)
        assert result is False

    def test_move_joints_wait_aborts_on_stop(self, api):
        """move_joints(..., wait=True) returns False when stop fires mid-wait."""
        rapi, node, host, stop_event = api

        def _set_stop():
            time.sleep(0.05)
            stop_event.set()

        threading.Thread(target=_set_stop, daemon=True).start()
        result = rapi.motion.move_joints(
            [180.0, 180.0, 180.0, 180.0, 180.0, 180.0, 180.0],
            wait=True, timeout=5.0)
        assert result is False

    def test_set_gripper_clamps_range(self, api):
        """set_gripper clamps values to [0, 0.08] metres."""
        rapi, node, _, _ = api
        rapi.motion.set_gripper(0.5)   # above max
        rapi.motion.set_gripper(-0.1)  # below min
        widths = [kw['width_meters'] for n, kw in node.calls
                  if n == 'publish_gripper_width']
        assert widths[0] == 0.08
        assert widths[1] == 0.0

    def test_open_close_gripper(self, api):
        """open_gripper and close_gripper publish the correct widths."""
        rapi, node, _, _ = api
        rapi.motion.open_gripper()
        rapi.motion.close_gripper()
        widths = [kw['width_meters'] for n, kw in node.calls
                  if n == 'publish_gripper_width']
        assert widths[0] == 0.08
        assert widths[1] == 0.0


# ── ModeAPI tests ─────────────────────────────────────────────────────────────

class TestModeAPI:
    """Tests for ModeAPI mode publishing and feature toggles."""

    def test_set_mode_publishes_int(self, api):
        """mode.set(2) calls publish_mode(2, rid)."""
        rapi, node, host, _ = api
        rapi.mode.set(2)
        calls = [(n, k) for n, k in node.calls if n == 'publish_mode']
        assert calls
        _, kw = calls[0]
        assert kw['mode_int'] == 2
        assert kw['robot_id'] == 'robot1'

    def test_set_mode_syncs_gui(self, api):
        """mode.set triggers update_mode_ui_only via marshal."""
        rapi, node, host, _ = api
        rapi.mode.set(2)
        mw = host._mw
        assert len(mw.mode_ui_calls) == 1
        mode_name, rid = mw.mode_ui_calls[0]
        assert mode_name == 'Joint Impedance With Planner'
        assert rid == 'robot1'

    def test_postural_enable(self, api):
        """mode.postural(True) calls publish_postural_enable(True, rid)."""
        rapi, node, _, _ = api
        rapi.mode.postural(True)
        calls = [k for n, k in node.calls if n == 'publish_postural_enable']
        assert calls
        assert calls[0]['enabled'] is True
        assert calls[0]['robot_id'] == 'robot1'

    def test_hpmp_enable(self, api):
        """mode.hpmp(False) calls publish_hpmp_enable(False, rid)."""
        rapi, node, _, _ = api
        rapi.mode.hpmp(False)
        calls = [k for n, k in node.calls if n == 'publish_hpmp_enable']
        assert calls[0]['enabled'] is False

    def test_desired_wrench(self, api):
        """mode.desired_wrench(force, torque) publishes six values; default is zero."""
        rapi, node, _, _ = api
        rapi.mode.desired_wrench(force=(5, 0, -2), torque=(0, 0.3, 0))
        rapi.mode.desired_wrench()
        calls = [k for n, k in node.calls if n == 'publish_desired_wrench']
        assert calls[0]['wrench'] == [5.0, 0.0, -2.0, 0.0, 0.3, 0.0]
        assert calls[0]['robot_id'] == 'robot1'
        assert calls[1]['wrench'] == [0.0] * 6
        with pytest.raises(ValueError):
            rapi.mode.desired_wrench(force=(1, 2))

    def test_admittance_planner(self, api):
        """mode.admittance_planner(True) calls publish_admittance_planner(True, rid)."""
        rapi, node, _, _ = api
        rapi.mode.admittance_planner(True)
        calls = [k for n, k in node.calls if n == 'publish_admittance_planner']
        assert calls[0]['enabled'] is True


# ── ProcessAPI tests ──────────────────────────────────────────────────────────

class TestProcessAPI:
    """Tests for ProcessAPI — verify marshal is called with the host action."""

    def test_start_sim_calls_marshal(self, api):
        """process.start_sim() invokes the main window _start_simulation."""
        rapi, node, host, _ = api
        called = []
        host._mw._start_simulation = lambda: called.append('start_simulation')
        rapi.process.start_sim()
        assert 'start_simulation' in called

    def test_stop_sim_calls_marshal(self, api):
        """process.stop_sim() invokes the main window _stop_simulation."""
        rapi, node, host, _ = api
        called = []
        host._mw._stop_simulation = lambda: called.append('stop_simulation')
        rapi.process.stop_sim()
        assert 'stop_simulation' in called

    def test_start_fic_calls_marshal(self, api):
        """process.start_fic() invokes the main window _start_fic_simulation."""
        rapi, node, host, _ = api
        called = []
        host._mw._start_fic_simulation = lambda: called.append('start_fic')
        rapi.process.start_fic()
        assert 'start_fic' in called


# ── SafetyAPI tests ───────────────────────────────────────────────────────────

class TestSafetyAPI:
    """Tests for SafetyAPI estop behaviour."""

    def test_estop_publishes_mode_1(self, api):
        """safety.estop() publishes mode 1 to park the robot."""
        rapi, node, _, _ = api
        host = rapi._host
        host._mw.broadcast_stop = lambda: None
        rapi.safety.estop()
        calls = [k for n, k in node.calls if n == 'publish_mode']
        assert calls
        assert calls[0]['mode_int'] == 1


# ── RobotAPI.wait + is_stopped tests ─────────────────────────────────────────

class TestRobotAPIWait:
    """Tests for the cooperative wait and is_stopped helpers."""

    def test_wait_completes_normally(self, api):
        """robot.wait(0.1) returns without error when stop is not set."""
        rapi, _, _, _ = api
        t0 = time.monotonic()
        rapi.wait(0.1)
        assert time.monotonic() - t0 >= 0.09

    def test_wait_aborts_on_stop(self, api):
        """robot.wait() returns early when the stop event fires mid-sleep."""
        rapi, _, _, stop_event = api

        def _set():
            time.sleep(0.05)
            stop_event.set()

        threading.Thread(target=_set, daemon=True).start()
        t0 = time.monotonic()
        rapi.wait(5.0)
        elapsed = time.monotonic() - t0
        assert elapsed < 1.0   # should abort well before 5 s

    def test_is_stopped_reflects_event(self, api):
        """is_stopped() tracks the underlying threading.Event."""
        rapi, _, _, stop_event = api
        assert rapi.is_stopped() is False
        stop_event.set()
        assert rapi.is_stopped() is True


# ── build_namespace tests ─────────────────────────────────────────────────────

class TestBuildNamespace:
    """Tests for the exec namespace produced by build_namespace."""

    def test_namespace_contains_robot(self, api):
        """build_namespace includes the 'robot' key pointing at the RobotAPI."""
        rapi, _, _, _ = api
        ns = robot_api.build_namespace(rapi)
        assert 'robot' in ns
        assert ns['robot'] is rapi

    def test_namespace_contains_flat_aliases(self, api):
        """build_namespace includes all expected flat aliases."""
        rapi, _, _, _ = api
        ns = robot_api.build_namespace(rapi)
        expected = (
            'move_joints', 'move_cartesian', 'home', 'set_gripper',
            'open_gripper', 'close_gripper', 'get_joint_positions',
            'get_cartesian_pose', 'set_mode', 'list_topics',
            'subscribe', 'unsubscribe', 'get_topic', 'wait', 'sleep',
            'is_stopped', 'get_target_mode', 'stop',
        )
        for name in expected:
            assert name in ns, f'Missing flat alias: {name}'

    def test_flat_move_joints_alias_works(self, api):
        """The flat move_joints alias publishes the same as robot.motion.move_joints."""
        rapi, node, _, _ = api
        ns = robot_api.build_namespace(rapi)
        ns['move_joints']([0.0, 45.0], wait=False)
        assert node.calls[0][0] == 'publish_joint_angles'

    def test_flat_get_joint_positions_returns_degrees(self, api):
        """get_joint_positions() in the flat namespace returns degrees."""
        rapi, _, host, _ = api
        ns = robot_api.build_namespace(rapi)
        joints = ns['get_joint_positions']()
        assert abs(joints[0] - 10.0) < 1e-9

    def test_flat_set_mode_calls_publish(self, api):
        """The flat set_mode alias triggers publish_mode."""
        rapi, node, _, _ = api
        ns = robot_api.build_namespace(rapi)
        ns['set_mode'](3)
        mode_calls = [k for n, k in node.calls if n == 'publish_mode']
        assert mode_calls[0]['mode_int'] == 3

    def test_namespace_has_math_and_time(self, api):
        """build_namespace includes the math and time modules."""
        import math as _math_mod
        import time as _time_mod
        rapi, _, _, _ = api
        ns = robot_api.build_namespace(rapi)
        assert ns['math'] is _math_mod
        assert ns['time'] is _time_mod


# ── api_reference tests ───────────────────────────────────────────────────────

class TestApiReference:
    """Tests for the api_reference function."""

    def test_api_reference_returns_list(self, api):
        """api_reference(api) returns a non-empty list of dicts."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        assert isinstance(ref, list)
        assert len(ref) > 0

    def test_each_entry_has_required_keys(self, api):
        """Every entry has name, signature, and doc keys."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        for entry in ref:
            assert 'name' in entry
            assert 'signature' in entry
            assert 'doc' in entry

    def test_move_joints_in_reference(self, api):
        """api_reference includes robot.motion.move_joints."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        names = [e['name'] for e in ref]
        assert 'robot.motion.move_joints' in names

    def test_state_joints_in_reference(self, api):
        """api_reference includes the robot.state.joints property."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        names = [e['name'] for e in ref]
        assert 'robot.state.joints' in names

    def test_flat_aliases_in_reference(self, api):
        """api_reference lists the flat alias move_joints."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        names = [e['name'] for e in ref]
        assert 'move_joints' in names

    def test_properties_have_no_signature(self, api):
        """A property is read, not called: robot.state.joints has no brackets."""
        rapi, _, _, _ = api
        ref = {e['name']: e for e in robot_api.api_reference(rapi)}
        assert ref['robot.state.joints']['signature'] == ''
        assert ref['robot.motion.move_joints']['signature'].startswith('(angles_deg')

    def test_no_entry_shows_self(self, api):
        """Signatures never show self."""
        rapi, _, _, _ = api
        assert not [e for e in robot_api.api_reference(rapi) if 'self' in e['signature']]

    def test_all_sub_apis_covered(self, api):
        """api_reference covers all eight sub-namespaces."""
        rapi, _, _, _ = api
        ref = robot_api.api_reference(rapi)
        prefixes = {e['name'].split('.')[1] for e in ref
                    if e['name'].startswith('robot.')}
        expected = {'state', 'motion', 'mode', 'process', 'target',
                    'tune', 'safety', 'topics'}
        assert expected.issubset(prefixes)


# ── Regression tests for post-review findings ─────────────────────────────────

class TestMoveJointsVelocityCompat:
    """
    Regression tests for the old velocity positional argument.

    Before the fix, any script calling move_joints(angles, velocity=0.3) would
    raise TypeError, and move_joints(angles, 0.3) would pass 0.3 to the 'wait'
    parameter (truthy) causing an unexpected blocking wait.  After the fix:

    - move_joints(angles, velocity=0.3) is accepted and the velocity value is
      silently ignored (with a DeprecationWarning).
    - move_joints(angles, 0.3) still maps 0.3 -> wait=0.3 (truthy), which is
      the unavoidable positional-API break; callers must use keyword syntax.
    - The new API reference HTML does NOT show 'velocity' as a parameter.
    """

    def test_velocity_kwarg_accepted_without_typeerror(self, api):
        """move_joints(angles_deg, velocity=0.3) does not raise TypeError."""
        rapi, node, _, _ = api
        # This must not raise — previously it raised TypeError
        rapi.motion.move_joints([0.0, 45.0], wait=False, velocity=0.3)
        assert node.calls[0][0] == 'publish_joint_angles'

    def test_velocity_kwarg_emits_deprecation_warning(self, api):
        """move_joints(..., velocity=0.3) emits a DeprecationWarning."""
        import warnings
        rapi, node, _, _ = api
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always')
            rapi.motion.move_joints([0.0], wait=False, velocity=0.5)
        dep_warnings = [w for w in caught if issubclass(w.category, DeprecationWarning)]
        assert dep_warnings, 'Expected a DeprecationWarning for the velocity kwarg'
        assert 'velocity' in str(dep_warnings[0].message).lower()

    def test_velocity_kwarg_does_not_block(self, api):
        """move_joints(..., velocity=0.3) does not trigger a blocking wait."""
        rapi, _, _, _ = api
        import time as _time
        t0 = _time.monotonic()
        # Target far from the canned state — if wait were True, this would block for timeout.
        rapi.motion.move_joints([180.0] * 7, wait=False, velocity=0.3)
        elapsed = _time.monotonic() - t0
        assert elapsed < 0.5, (
            f'move_joints blocked for {elapsed:.2f}s — velocity kwarg set wait=True'
        )

    def test_api_reference_html_shows_new_signature(self):
        """The HTML API reference does not contain the obsolete velocity parameter."""
        import sys as _sys
        import types as _types
        # Test the source text of extension.py directly — no import needed.
        for mod in ('std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
                    'rclpy', 'rclpy.node', 'scipy', 'scipy.spatial',
                    'scipy.spatial.transform'):
            if mod not in _sys.modules:
                _sys.modules[mod] = _types.ModuleType(mod)
        ext_path = _os.path.join(_os.path.dirname(__file__), 'extension.py')
        with open(ext_path, encoding='utf-8') as fh:
            source = fh.read()
        # Old positional signature must not appear in the HTML reference block
        assert 'move_joints(angles, velocity=0.5)' not in source, (
            'HTML API reference still shows the old move_joints(angles, velocity=0.5) signature'
        )
        # New signature must be present
        assert 'move_joints(angles_deg, wait=False' in source, (
            'HTML API reference does not show the new '
            'move_joints(angles_deg, wait=False, ...) signature'
        )
