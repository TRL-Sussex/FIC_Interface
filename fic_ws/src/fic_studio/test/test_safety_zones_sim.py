"""
Sim integration test: safety zones actually clamp commanded targets.

Launches the REAL fic_simulation controller headless (MUJOCO_GL=egl, no MuJoCo
physics needed — we feed /simulation/joint_states ourselves), publishes a
Cartesian safety limit + an out-of-range EE target, and reads the resulting
clamped target back from trajectory_data to prove enforcement end to end.

This is a slow ROS integration test and the only one here that launches a real
ROS stack, so it is opt-in and never runs as part of the unit suite. Run it
deliberately, with the workspace built and sourced:

    source fic_ws/install/setup.bash
    FIC_SIM_TESTS=1 QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl python3 -m pytest \
        test/test_safety_zones_sim.py -q -s

Without FIC_SIM_TESTS=1 it skips. The workspace is found relative to this file;
set FIC_WS to override.

Proven result (manual run): no-zone desired_z=0.850 -> zone(max 0.5) 0.500.
"""
import os
import shutil
import signal
import subprocess
import threading
import time

import pytest

# Resolve the workspace from this file, not from somebody's home directory: the
# path this used to hold existed on one developer's machine, so `_have_ros` was
# false everywhere else and the test silently skipped instead of running.
# test/ -> fic_studio/ -> src/ -> fic_ws/
_WS = os.environ.get('FIC_WS') or os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_SETUP = os.path.join(_WS, 'install', 'setup.bash')

# Opt-in, because the workspace now resolves on every machine rather than one.
# This is the only test in the suite that launches a real ROS stack, and the
# docstring's promise is that it never breaks the unit suite — which a path that
# happened not to exist was quietly keeping. Gate it explicitly instead:
# `FIC_SIM_TESTS=1` runs it, anything else skips it, so `pytest test/` stays
# fast and offline wherever it is run.
_OPTED_IN = os.environ.get('FIC_SIM_TESTS') == '1'
_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
_N = 7
_DESIRED_Z = 8 * _N + 11   # trajectory_data Cartesian-block desired-z index (empirically verified)

_have_ros = shutil.which('ros2') is not None and os.path.exists(_SETUP)
pytestmark = pytest.mark.skipif(
    not (_OPTED_IN and _have_ros),
    reason=('slow ROS integration test: set FIC_SIM_TESTS=1 with the workspace '
            'built' if _have_ros else 'ROS 2 workspace not sourced/available'))


@pytest.fixture(scope='module')
def sim_controller():
    """Launch the fic_simulation controller headless; tear down on exit."""
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', MUJOCO_GL='egl')
    cmd = ('source %s && exec ros2 launch fic_simulation fic_launch.py '
           'robot_name:=Franka_FR3 num_joints:=7 station_id:=1 '
           'source_prefix:=simulation' % _SETUP)
    proc = subprocess.Popen(['bash', '-c', cmd], env=env, preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(16)   # controller init + MuJoCo-less startup
    yield proc
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass
    # Anything left in the launch's own session; never a pkill by name (other users' sims share the PC).
    from fic_studio.mixins.process_mixin import _kill_session
    _kill_session(proc.pid)


def test_cartesian_zone_clamps_target(sim_controller):
    """With a z-zone (max 0.5) enabled, an EE target z=0.85 is clamped to 0.5."""
    import rclpy
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32, Float64MultiArray

    rclpy.init()
    try:
        n = rclpy.create_node('zone_sim_test')
        js = n.create_publisher(JointState, '/simulation/joint_states', 10)
        mode = n.create_publisher(Int32, 'robot1_control_mode', 10)
        saf = n.create_publisher(
            Float64MultiArray, 'robot1_safety/cartesian_limits', 10)
        tgt = n.create_publisher(
            Pose, 'command_ee_position_change_robot1', 10)
        traj = {'d': None}
        n.create_subscription(
            Float64MultiArray, 'robot1_trajectory_data',
            lambda m: traj.__setitem__('d', list(m.data)), 10)

        stop = threading.Event()

        def feed():
            while not stop.is_set():
                try:
                    m = JointState()
                    m.name = ['j%d' % i for i in range(_N)]
                    m.position = _HOME
                    js.publish(m)
                except Exception:
                    return   # context torn down — stop publishing cleanly
                time.sleep(0.02)
        threading.Thread(target=feed, daemon=True).start()

        def spin(t):
            end = time.monotonic() + t
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.02)

        spin(2.0)
        for _ in range(10):
            mode.publish(Int32(data=5))   # Cartesian Impedance
            spin(0.1)
        spin(1.0)

        def desired_z(zone_enabled, zmax, target_z):
            saf.publish(Float64MultiArray(
                data=[-1., 1., 0., -1., 1., 0., 0., zmax, float(zone_enabled)]))
            spin(0.5)
            p = Pose()
            p.position.x = 0.45
            p.position.z = target_z
            p.orientation.w = 1.0
            for _ in range(16):
                tgt.publish(p)
                spin(0.2)
            spin(1.5)
            assert traj['d'] is not None and len(traj['d']) > _DESIRED_Z
            return traj['d'][_DESIRED_Z]

        unclamped = desired_z(0, 2.0, 0.85)
        clamped = desired_z(1, 0.50, 0.85)
        stop.set()

        # The deterministic property: the z-zone (max 0.5) clamps the commanded
        # target down to ~0.5, clearly below the unclamped target.
        assert clamped == pytest.approx(0.50, abs=0.06), (unclamped, clamped)
        assert clamped < unclamped - 0.1, (unclamped, clamped)
    finally:
        rclpy.shutdown()
