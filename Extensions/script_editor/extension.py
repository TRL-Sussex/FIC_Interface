"""Script Editor extension: a Python editor whose scripts get the robot API.

Scripts run on a worker thread. Calls into the Studio go through
MainThreadProxy to the Qt thread; Stop and the E-stop set a threading.Event
the script's waits check (2 s grace, then the thread is terminated); print()
from the script thread goes to the output. Flags a script latches are reset
on teardown.

The Classic page has the editor and example gallery on the left, live
telemetry and the searchable API reference on the right, and the output
below. The web page is web/page.js (web_state / web_action below). Example
scripts are copied to ~/.fic_studio_scripts/examples/.
"""
import math
import os
import sys
import threading
import time
import traceback
from datetime import datetime
from io import StringIO

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QTextCursor
from PyQt5.QtWidgets import (
    QComboBox, QFileDialog, QFrame, QGroupBox, QHBoxLayout, QInputDialog,
    QLabel, QLineEdit, QListView, QListWidget, QListWidgetItem, QMessageBox,
    QPlainTextEdit, QPushButton, QSplitter, QTabWidget,
    QTextEdit, QVBoxLayout, QWidget,
)

from fic_studio import theme as _theme
from fic_studio.config import FIC_API_DIR as _FIC_API_DIR
from fic_studio.extensions import Extension as ExtensionBase

# robot_api lives next to this file in the Extensions/script_editor package
import pathlib as _pathlib
import sys as _sys

_EXT_DIR = _pathlib.Path(__file__).resolve().parent.parent.parent
if str(_EXT_DIR) not in _sys.path:
    _sys.path.insert(0, str(_EXT_DIR))

from Extensions.script_editor.robot_api import (  # noqa: E402
    MainThreadProxy, RobotAPI, build_namespace, make_marshal,
)
from Extensions.script_editor.web_backend import (  # noqa: E402
    OutputStore, SCRIPT_FILENAME, check_name, describe, failing_line, list_scripts, read_script,
)


SCRIPTS_DIR = os.path.expanduser('~/.fic_studio_scripts')
EXAMPLES_DIR = os.path.join(SCRIPTS_DIR, 'examples')


# ── Syntax highlighter (verbatim from script_mixin) ───────────────────
class PythonHighlighter(QSyntaxHighlighter):
    def __init__(self, document):
        super().__init__(document)
        self._setup_formats()

    def _setup_formats(self):
        self.keyword_format = QTextCharFormat()
        self.keyword_format.setForeground(QColor("#cc7832"))
        self.keyword_format.setFontWeight(QFont.Bold)
        self.builtin_format = QTextCharFormat()
        self.builtin_format.setForeground(QColor("#8888ff"))
        self.string_format = QTextCharFormat()
        self.string_format.setForeground(QColor("#6a8759"))
        self.comment_format = QTextCharFormat()
        self.comment_format.setForeground(QColor("#808080"))
        self.comment_format.setFontItalic(True)
        self.number_format = QTextCharFormat()
        self.number_format.setForeground(QColor("#6897bb"))
        self.function_format = QTextCharFormat()
        self.function_format.setForeground(QColor("#ffc66d"))
        self.robot_api_format = QTextCharFormat()
        self.robot_api_format.setForeground(QColor("#56c8d8"))
        self.robot_api_format.setFontWeight(QFont.Bold)
        self.keywords = [
            'and', 'as', 'assert', 'async', 'await', 'break', 'class', 'continue',
            'def', 'del', 'elif', 'else', 'except', 'finally', 'for', 'from',
            'global', 'if', 'import', 'in', 'is', 'lambda', 'nonlocal', 'not',
            'or', 'pass', 'raise', 'return', 'try', 'while', 'with', 'yield',
            'True', 'False', 'None'
        ]
        self.builtins = [
            'abs', 'all', 'any', 'bin', 'bool', 'bytes', 'callable', 'chr',
            'dict', 'dir', 'divmod', 'enumerate', 'eval', 'exec', 'filter',
            'float', 'format', 'frozenset', 'getattr', 'globals', 'hasattr',
            'hash', 'help', 'hex', 'id', 'input', 'int', 'isinstance', 'issubclass',
            'iter', 'len', 'list', 'locals', 'map', 'max', 'min', 'next',
            'object', 'oct', 'open', 'ord', 'pow', 'print', 'range', 'repr',
            'reversed', 'round', 'set', 'setattr', 'slice', 'sorted', 'str',
            'sum', 'super', 'tuple', 'type', 'vars', 'zip'
        ]
        self.robot_api = [
            'robot', 'move_joints', 'move_cartesian', 'get_joint_positions',
            'get_cartesian_pose', 'set_gripper', 'open_gripper', 'close_gripper',
            'home', 'sleep', 'wait', 'set_velocity', 'set_mode', 'stop',
            'get_gripper_width', 'is_moving', 'wait_until_stopped', 'get_target_mode',
            'list_topics', 'subscribe', 'unsubscribe', 'get_topic',
            'get_trajectory', 'get_current_joints', 'get_target_joints',
            'get_joint_errors', 'get_control_effort', 'get_planner_targets',
            'get_admittance_targets', 'get_desired_effort', 'get_measured_effort',
            'get_ee_position', 'get_ee_desired', 'get_ee_error',
            'get_rotation_error', 'get_ee_admittance_target',
            'get_force_torque', 'get_torque_commands',
        ]

    def highlightBlock(self, text):
        import re
        for keyword in self.keywords:
            for match in re.finditer(r'\b' + keyword + r'\b', text):
                self.setFormat(match.start(), match.end() - match.start(), self.keyword_format)
        for builtin in self.builtins:
            for match in re.finditer(r'\b' + builtin + r'\b', text):
                self.setFormat(match.start(), match.end() - match.start(), self.builtin_format)
        for api_func in self.robot_api:
            for match in re.finditer(r'\b' + api_func + r'\b', text):
                self.setFormat(match.start(), match.end() - match.start(), self.robot_api_format)
        for match in re.finditer(r'\bdef\s+(\w+)', text):
            self.setFormat(match.start(1), match.end(1) - match.start(1), self.function_format)
        for match in re.finditer(r'\b\d+\.?\d*\b', text):
            self.setFormat(match.start(), match.end() - match.start(), self.number_format)
        for match in re.finditer(r'"[^"\\]*(\\.[^"\\]*)*"', text):
            self.setFormat(match.start(), match.end() - match.start(), self.string_format)
        for match in re.finditer(r"'[^'\\]*(\\.[^'\\]*)*'", text):
            self.setFormat(match.start(), match.end() - match.start(), self.string_format)
        for match in re.finditer(r'#.*$', text):
            self.setFormat(match.start(), match.end() - match.start(), self.comment_format)


class LineNumberArea(QWidget):
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor

    def paintEvent(self, event):
        self.editor.lineNumberAreaPaintEvent(event)


class CodeEditor(QPlainTextEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        font = QFont("Consolas", 11)
        font.setStyleHint(QFont.Monospace)
        self.setFont(font)
        self.line_number_area = LineNumberArea(self)
        self.blockCountChanged.connect(self.updateLineNumberAreaWidth)
        self.updateRequest.connect(self.updateLineNumberArea)
        self.updateLineNumberAreaWidth(0)
        self.setTabStopDistance(self.fontMetrics().horizontalAdvance(' ') * 4)

    def lineNumberAreaWidth(self):
        digits = len(str(max(1, self.blockCount())))
        return 10 + self.fontMetrics().horizontalAdvance('9') * digits

    def updateLineNumberAreaWidth(self, _):
        self.setViewportMargins(self.lineNumberAreaWidth(), 0, 0, 0)

    def updateLineNumberArea(self, rect, dy):
        if dy:
            self.line_number_area.scroll(0, dy)
        else:
            self.line_number_area.update(0, rect.y(), self.line_number_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self.updateLineNumberAreaWidth(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self.line_number_area.setGeometry(cr.left(), cr.top(), self.lineNumberAreaWidth(), cr.height())

    def lineNumberAreaPaintEvent(self, event):
        from PyQt5.QtGui import QPainter
        painter = QPainter(self.line_number_area)
        painter.fillRect(event.rect(), QColor("#2b2b2b"))
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                painter.setPen(QColor("#606060"))
                painter.drawText(0, top, self.line_number_area.width() - 5,
                                 self.fontMetrics().height(), Qt.AlignRight, str(block_number + 1))
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            block_number += 1


class _ScriptSignals(QObject):
    output = pyqtSignal(str)
    error = pyqtSignal(str)
    finished = pyqtSignal(bool, str)
    progress = pyqtSignal(str)


class _StreamChunk(str):
    """Text a script printed: the output joins it to the line it continues."""


class _ThreadRoutedWriter:
    """
    stdout/stderr interceptor that captures writes from one specific thread.

    Writes originating from the script worker thread are forwarded to a
    StringIO buffer (and optionally emit a signal).  Writes from all other
    threads are forwarded to the real underlying stream so that the rest of
    the application is unaffected.
    """

    def __init__(self, real_stream, capture_thread, signal=None):
        """
        Initialise the routed writer.

        Parameters
        ----------
        real_stream :
            The original sys.stdout or sys.stderr to restore and to forward
            non-script-thread writes to.
        capture_thread : threading.Thread
            Only writes from this thread are captured.
        signal : pyqtSignal | None
            If given, each captured line is emitted via signal.emit(line).

        """
        self._real = real_stream
        self._tid = capture_thread.ident
        self._buf = StringIO()
        self._signal = signal
        self._lock = threading.Lock()

    def write(self, text):
        """Write text: capture if from the script thread, else forward."""
        if threading.current_thread().ident == self._tid:
            with self._lock:
                self._buf.write(text)
            if self._signal is not None and text:
                self._signal.emit(text)
        else:
            self._real.write(text)

    def flush(self):
        """Flush both the capture buffer and the real stream."""
        with self._lock:
            self._buf.flush()
        try:
            self._real.flush()
        except Exception:  # noqa: BLE001
            pass

    def getvalue(self):
        """Return all text captured from the script thread so far."""
        with self._lock:
            return self._buf.getvalue()

    # Make the wrapper look like a real file object
    @property
    def encoding(self):
        """Return the encoding of the real stream."""
        return getattr(self._real, 'encoding', 'utf-8')


class ScriptRunner(QThread):
    """
    QThread that executes a Python script in an isolated namespace.

    Stop protocol
    -------------
    1. ``stop()`` sets the ``stop_event`` threading.Event, which the script can
       poll via ``is_stopped()`` or ``wait()``.
    2. After 2 seconds, if the thread is still alive, ``terminate()`` is called
       as a last-resort fallback (matches the QThread contract).

    stdout is routed through a ``_ThreadRoutedWriter`` so only writes from THIS
    thread are captured; writes from other threads reach the real stdout
    unaffected.
    """

    _GRACE_SECONDS = 2.0   # cooperative-stop window before terminate()

    def __init__(self, code, namespace, stop_event, parent=None):
        """
        Initialise the runner.

        Parameters
        ----------
        code : str
            Python source code to execute.
        namespace : dict
            The exec() globals dict (from build_namespace or _create_robot_api).
        stop_event : threading.Event
            Shared event; set to request cooperative stop.

        """
        super().__init__(parent)
        self.code = code
        self.namespace = namespace
        self.stop_event = stop_event
        self.signals = _ScriptSignals()

    def stop(self):
        """
        Request cooperative stop.

        Sets stop_event immediately; the caller is responsible for waiting
        up to _GRACE_SECONDS and then calling terminate() if needed.
        """
        self.stop_event.set()

    def run(self):
        """Execute the script, capturing stdout from this thread only."""
        routed_out = _ThreadRoutedWriter(
            sys.stdout, threading.current_thread(), signal=self.signals.output)
        routed_err = _ThreadRoutedWriter(sys.stderr, threading.current_thread())
        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout = routed_out
        sys.stderr = routed_err
        try:
            ns = dict(self.namespace)
            ns['__builtins__'] = __builtins__
            # Compiled as <script> so a traceback names the script's own lines.
            exec(compile(self.code, SCRIPT_FILENAME, 'exec'), ns)  # noqa: S102 — intentional script sandbox
            self.signals.finished.emit(True, 'Script completed successfully')
        except Exception as exc:  # noqa: BLE001
            self.signals.error.emit(traceback.format_exc())
            self.signals.finished.emit(False, str(exc))
        finally:
            sys.stdout, sys.stderr = old_stdout, old_stderr


# ── Trajectory data parser (verbatim) ─────────────────────────────────
def _parse_trajectory_data(raw, num_joints=7):
    n = num_joints
    total = len(raw)
    if total == 0:
        return None
    base = n * 8
    is_cart = total >= base + 6 and any(abs(p) > 0.001 for p in raw[base + 3: base + 6])

    def _safe(idx):
        return raw[idx] if idx < total else 0.0

    result = {'is_cartesian': is_cart, 'num_joints': n, 'raw': raw}
    if is_cart:
        q_current = [_safe(i * 8 + 0) for i in range(n)]
        q_target = [_safe(i * 8 + 1) for i in range(n)]
        result['q_current'] = q_current
        result['q_target'] = q_target
        result['error'] = [q_target[i] - q_current[i] for i in range(n)]
        result['control_effort'] = [0.0] * n
        result['planner_target'] = [0.0] * n
        result['q_target_adm'] = [0.0] * n
        result['desired_effort'] = [0.0] * n
        result['measured_effort'] = [0.0] * n
        result['ee_admittance_target'] = [_safe(base + i) for i in range(3)]
        result['ee_position'] = [_safe(base + 3 + i) for i in range(3)]
        result['ee_error'] = [_safe(base + 6 + i) for i in range(3)]
        result['ee_desired'] = [_safe(base + 9 + i) for i in range(3)]
        result['rotation_error'] = [_safe(base + 12 + i) for i in range(3)]
        result['rotation_angle'] = _safe(base + 15)
    else:
        result['planner_target'] = [_safe(i + 0 * n) for i in range(n)]
        result['q_current'] = [_safe(i + 1 * n) for i in range(n)]
        result['error'] = [_safe(i + 2 * n) for i in range(n)]
        result['q_target'] = [_safe(i + 3 * n) for i in range(n)]
        result['control_effort'] = [_safe(i + 4 * n) for i in range(n)]
        result['q_target_adm'] = [_safe(i + 5 * n) for i in range(n)]
        result['desired_effort'] = [_safe(i + 6 * n) for i in range(n)]
        result['measured_effort'] = [_safe(i + 7 * n) for i in range(n)]
        for k in ('ee_position', 'ee_desired', 'ee_error',
                  'ee_admittance_target', 'rotation_error', 'rotation_angle'):
            result[k] = None
    return result


# ── ROS message parser (verbatim) ─────────────────────────────────────
def _parse_ros_msg(msg):
    msg_type = type(msg).__name__
    if msg_type in ('Float64MultiArray', 'Float32MultiArray'):
        return {'type': msg_type, 'data': list(msg.data)}
    if msg_type == 'JointState':
        d = {'type': 'JointState'}
        if msg.name: d['name'] = list(msg.name)
        if msg.position: d['position'] = list(msg.position)
        if msg.velocity: d['velocity'] = list(msg.velocity)
        if msg.effort: d['effort'] = list(msg.effort)
        return d
    if msg_type == 'Pose':
        return {'type': 'Pose',
                'position': {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z},
                'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y,
                                'z': msg.orientation.z, 'w': msg.orientation.w}}
    if msg_type == 'Wrench':
        return {'type': 'Wrench',
                'force': {'x': msg.force.x, 'y': msg.force.y, 'z': msg.force.z},
                'torque': {'x': msg.torque.x, 'y': msg.torque.y, 'z': msg.torque.z}}
    if msg_type == 'WrenchStamped':
        return {'type': 'WrenchStamped',
                'force': {'x': msg.wrench.force.x, 'y': msg.wrench.force.y, 'z': msg.wrench.force.z},
                'torque': {'x': msg.wrench.torque.x, 'y': msg.wrench.torque.y, 'z': msg.wrench.torque.z}}
    if hasattr(msg, 'data'):
        return {'type': msg_type, 'data': msg.data}
    try:
        d = {'type': msg_type}
        for slot in msg.__slots__:
            d[slot.lstrip('_')] = getattr(msg, slot)
        return d
    except Exception:
        return {'type': msg_type, 'raw': str(msg)}


_MSG_TYPE_REGISTRY = {
    'std_msgs/msg/Float64MultiArray': ('std_msgs.msg', 'Float64MultiArray'),
    'std_msgs/msg/Float32MultiArray': ('std_msgs.msg', 'Float32MultiArray'),
    'std_msgs/msg/Int32': ('std_msgs.msg', 'Int32'),
    'std_msgs/msg/Float32': ('std_msgs.msg', 'Float32'),
    'std_msgs/msg/Float64': ('std_msgs.msg', 'Float64'),
    'std_msgs/msg/String': ('std_msgs.msg', 'String'),
    'std_msgs/msg/Bool': ('std_msgs.msg', 'Bool'),
    'sensor_msgs/msg/JointState': ('sensor_msgs.msg', 'JointState'),
    'geometry_msgs/msg/Pose': ('geometry_msgs.msg', 'Pose'),
    'geometry_msgs/msg/Wrench': ('geometry_msgs.msg', 'Wrench'),
    'geometry_msgs/msg/WrenchStamped': ('geometry_msgs.msg', 'WrenchStamped'),
    'communication/msg/ActuatorCmds': ('communication.msg', 'ActuatorCmds'),
    'communication/msg/Jacobian': ('communication.msg', 'Jacobian'),
}


def _import_msg_class(type_str):
    entry = _MSG_TYPE_REGISTRY.get(type_str)
    if entry:
        import importlib
        return getattr(importlib.import_module(entry[0]), entry[1])
    parts = type_str.replace('/', '.')
    last_dot = parts.rfind('.')
    if last_dot > 0:
        try:
            import importlib
            return getattr(importlib.import_module(parts[:last_dot]), parts[last_dot + 1:])
        except Exception:
            pass
    return None


# ── Example script content ────────────────────────────────────────────
_EXAMPLE_01 = '''\
"""01_read_state.py — Read robot state using the new API.

Prints joint angles, EE pose, and wrench to the output console.
No motion commands are issued.
"""
# robot.state.* reads are thread-safe direct calls to host helpers.
print('=== Read State Example ===')

joints = robot.state.joints          # list[float], degrees
print(f'Joint positions (deg): {[f"{j:.1f}" for j in joints]}')

vel = robot.state.joint_velocity     # list[float], rad/s
print(f'Joint velocities (rad/s): {[f"{v:.3f}" for v in vel]}')

effort = robot.state.joint_effort    # list[float], Nm
print(f'Joint efforts    (Nm):  {[f"{e:.2f}" for e in effort]}')

ee = robot.state.ee_pose             # [x, y, z, qx, qy, qz, qw] or []
if ee:
    print(f'EE position (m): x={ee[0]:.3f}  y={ee[1]:.3f}  z={ee[2]:.3f}')
else:
    print('EE pose not available (joint-space mode?)')

wrench = robot.state.wrench          # (Fx, Fy, Fz, Tx, Ty, Tz) or None
if wrench:
    print(f'Wrench (N/Nm): Fx={wrench[0]:.2f}  Fy={wrench[1]:.2f}  Fz={wrench[2]:.2f}')
else:
    print('Wrench not available')

print('Done.')
'''

_EXAMPLE_02 = '''\
"""02_joint_trajectory.py — Move through a simple joint-space trajectory.

Uses robot.motion.move_joints(wait=True) to block until each waypoint is
reached.  The cooperative stop event is polled so pressing Stop aborts
cleanly between waypoints.
"""
import math as _m

WAYPOINTS_DEG = [
    [0.0, -45.0,  0.0, -90.0, 0.0,  45.0, 0.0],
    [30.0, -30.0, 20.0, -60.0, 0.0,  60.0, 0.0],
    [0.0, -45.0,  0.0, -90.0, 0.0,  45.0, 0.0],
]

print('=== Joint Trajectory Example ===')
print(f'Target: {robot.target_mode}')

for idx, wp in enumerate(WAYPOINTS_DEG):
    if is_stopped():
        print('Stopped by user.')
        break
    print(f'Moving to waypoint {idx + 1}/{len(WAYPOINTS_DEG)}…')
    reached = robot.motion.move_joints(wp, wait=True, timeout=15.0, tolerance=1.5)
    if reached:
        print(f'  Reached waypoint {idx + 1}.')
    else:
        print(f'  Waypoint {idx + 1}: timeout or stopped.')
    robot.wait(0.3)

print('Trajectory complete.')
'''

_EXAMPLE_03 = '''\
"""03_cartesian_circle.py — Sweep a small circle in Cartesian space.

Sends incremental position commands using robot.motion.move_cartesian.
Requires the robot to already be in a Cartesian mode (5–8).
"""
import math

# Circle parameters
RADIUS_M = 0.05      # 5 cm radius
N_STEPS  = 24        # 24 steps = 15° per step
CENTRE_X = 0.4
CENTRE_Y = 0.0
CENTRE_Z = 0.3

print('=== Cartesian Circle Example ===')
print(f'Centre: ({CENTRE_X}, {CENTRE_Y}, {CENTRE_Z}), r={RADIUS_M} m, {N_STEPS} steps')

for i in range(N_STEPS + 1):
    if is_stopped():
        print('Stopped.')
        break
    angle = 2 * math.pi * i / N_STEPS
    x = CENTRE_X + RADIUS_M * math.cos(angle)
    y = CENTRE_Y + RADIUS_M * math.sin(angle)
    z = CENTRE_Z
    robot.motion.move_cartesian(x, y, z)
    robot.wait(0.08)
    if i % 6 == 0:
        print(f'  Step {i}/{N_STEPS}: x={x:.3f}  y={y:.3f}  z={z:.3f}')

print('Circle complete.')
'''

_EXAMPLE_04 = '''\
"""04_mode_and_tune.py — Switch control modes and adjust parameters.

Demonstrates robot.mode.set(), robot.mode.postural(), and
robot.tune.param_update().  No blocking motion is used.
"""
import time as _time

print('=== Mode and Tune Example ===')

# Read current mode
print(f'Current target: {robot.target_mode}')

# Switch to Joint Impedance (mode 1)
print('Setting mode -> Joint Impedance (1)…')
robot.mode.set(1)
robot.wait(0.5)

# Enable postural null-space controller
print('Enabling postural null-space controller…')
robot.mode.postural(True)
robot.wait(0.3)

# Ask the controller to re-read the tuning in config.xml
print('Asking the controller to reload its tuning')
robot.tune.param_update(0, 0)
robot.wait(0.2)

# Restore postural off
robot.mode.postural(False)
print('Postural disabled.')

print('Done.')
'''

_EXAMPLE_05 = '''\
"""05_record_and_plot.py — Start bags and plotter, run a trajectory, then stop.

Demonstrates robot.process.start_bags / stop_bags and start_plotter /
stop_plotter.  The joint trajectory mirrors example 02.
"""

print('=== Record and Plot Example ===')

# Start the ROS bag recorder and the plotter
print('Starting bag recorder…')
robot.process.start_bags()
robot.wait(0.5)

print('Starting plotter…')
robot.process.start_plotter()
robot.wait(0.5)

# Execute a short trajectory
WAYPOINTS_DEG = [
    [0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],
    [20.0, -30.0, 10.0, -70.0, 0.0, 50.0, 0.0],
    [0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],
]

for idx, wp in enumerate(WAYPOINTS_DEG):
    if is_stopped():
        print('Stopped early — stopping recorder.')
        break
    print(f'Waypoint {idx + 1}…')
    reached = robot.motion.move_joints(wp, wait=True, timeout=12.0)
    print(f'  {"reached" if reached else "timeout/stopped"}')
    robot.wait(0.3)

# Stop the bag recorder (keep plotter open for inspection)
print('Stopping bag recorder…')
robot.process.stop_bags()
robot.wait(0.3)

print('Recording done. Plotter remains open.')
'''

# Map of filename → source for the seeder
_EXAMPLES = {
    '01_read_state.py': _EXAMPLE_01,
    '02_joint_trajectory.py': _EXAMPLE_02,
    '03_cartesian_circle.py': _EXAMPLE_03,
    '04_mode_and_tune.py': _EXAMPLE_04,
    '05_record_and_plot.py': _EXAMPLE_05,
}


def _seed_examples(examples_dir):
    """Write example scripts to examples_dir (skip if the file already exists).

    Parameters
    ----------
    examples_dir : str
        Absolute path to ~/.fic_studio_scripts/examples/.

    """
    for fname, source in _EXAMPLES.items():
        fpath = os.path.join(examples_dir, fname)
        if not os.path.exists(fpath):
            try:
                with open(fpath, 'w', encoding='utf-8') as fh:
                    fh.write(source)
            except OSError:
                pass


# ── Extension class ───────────────────────────────────────────────────
class Extension(ExtensionBase):
    ID = "script_editor"
    NAME = "Script Editor"
    DESCRIPTION = (
        "Python editor with a robot API namespace (move_joints, move_cartesian, "
        "set_gripper, get_trajectory, generic topic subscription, etc.)."
    )
    SIDEBAR_ICON = "𓆗"
    SIDEBAR_SECTION = "Programming"

    def __init__(self, host):
        """Initialise the Script Editor extension on the main (GUI) thread."""
        super().__init__(host)
        self.script_runner = None
        self.script_running = False
        self._script_output_buffer = []
        # The output both pages show, and the last run's state for the web
        # page's status line.
        self._output = OutputStore()
        self._run_status = 'idle'
        self._run_started = None
        self._run_elapsed = 0.0
        self._error_line = None
        self._example_cache = {}
        self.script_target_mode = 'simulation'
        # script_topics is now a property — recomputed each access so it
        # always reflects host.target_robot_id (which the user can change
        # via the per-extension Target Robot combo).
        os.makedirs(SCRIPTS_DIR, exist_ok=True)
        os.makedirs(EXAMPLES_DIR, exist_ok=True)
        _seed_examples(EXAMPLES_DIR)

        self._script_output_timer = QTimer()
        self._script_output_timer.timeout.connect(self._flush_script_output)
        self._script_output_timer.start(50)

        self._script_topic_cache = {}
        self._script_topic_subs = {}
        self._script_topic_lock = threading.Lock()
        self._script_traj_raw = []
        self._script_traj_parsed = None
        self._script_traj_sub = None

        self._page = None

        # MainThreadProxy created HERE on the GUI thread
        self._main_thread_proxy = MainThreadProxy()
        self._marshal = make_marshal(self._main_thread_proxy)

        # Per-run stop event; replaced before each run
        self._stop_event = threading.Event()

        # Active RobotAPI for on_emergency_stop access
        self._robot_api = None

        # Latched flags to restore on teardown
        self._latched_admittance_planner = False
        self._latched_hpmp = False

        # Live telemetry QTimer (~10 Hz); created here so teardown can
        # stop it; wired to the telemetry labels in build_page.
        self._telemetry_timer = QTimer()
        self._telemetry_timer.timeout.connect(self._update_telemetry)
        self._telemetry_timer.start(100)
        # References to the telemetry labels (set in build_page)
        self._telem_joint_labels = []
        self._telem_ee_label = None
        self._telem_wrench_label = None
        # API reference filter widget (set in build_page)
        self._api_ref_filter = None
        self._api_ref_list = None

    @property
    def script_topics(self) -> dict:
        """Topic names for this script's target robot. Recomputed on every
        access so changing the Target Robot combo takes effect immediately
        (instead of needing to reload the extension)."""
        rid = self.host.target_robot_id
        return {
            'simulation': {
                'desired_joints': f'{rid}_desired_joint_angles',
                'target_pose': f'command_ee_position_change_{rid}',
                'gripper': f'{rid}_gripper_width',
                'control_mode': f'{rid}_control_mode',
                'control_mode_type': 'int',
                'joint_states': f'/{rid}_simulation/joint_states',
                'ee_pose': f'{rid}_ee_current_pose',
            },
            'hardware': {
                'desired_joints': f'{rid}_desired_joint_angles',
                'target_pose': f'command_ee_position_change_{rid}',
                'gripper': f'{rid}_gripper_width',
                'control_mode': f'{rid}_mode_select',
                'control_mode_type': 'string',
                'joint_states': '/joint_states',
                'ee_pose': f'{rid}_ee_current_pose',
            },
        }

    # ── Lifecycle ─────────────────────────────────────────────────────
    def teardown(self) -> None:
        """Tear down the extension: stop any running script and restore flags."""
        if self.script_running:
            self._stop_script()
        # Restore latched flags to safe defaults so the controller
        # does not remain in an unexpected mode after the script ends.
        if self._robot_api is not None:
            try:
                rid = self.host.target_robot_id
                node = self.host.node
                if self._latched_admittance_planner:
                    node.publish_admittance_planner(False, rid)
                if self._latched_hpmp:
                    node.publish_hpmp_enable(False, rid)
            except Exception:  # noqa: BLE001
                pass
            self._robot_api = None
        self._script_cleanup_all_topic_subs()
        self._script_output_timer.stop()
        # Stop the telemetry timer
        self._telemetry_timer.stop()

    def on_emergency_stop(self) -> None:
        """Handle an emergency stop: set the cooperative stop event and estop."""
        if self.script_running:
            self.host.log('WARN', 'Emergency stop — signalling script', 'Script')
            # Set the event so blocking loops exit cooperatively
            self._stop_event.set()
            # Publish mode 1 via the active API if available
            if self._robot_api is not None:
                try:
                    self._robot_api.safety.estop()
                except Exception:  # noqa: BLE001
                    pass
            self._stop_script()

    @property
    def state_running(self) -> bool:
        return self.script_running

    # ── Web page (manifest "web"; web/page.js) ────────────────────────
    # The script being edited lives in the browser; the Studio keeps the saved
    # scripts, runs them and holds their output. No dialog boxes here: a
    # refusal is a ValueError, which the page shows.

    def web_state(self) -> dict:
        """What the web page shows; small, so it can be polled."""
        rid = self.host.target_robot_id
        if self.script_running and self._run_started is not None:
            elapsed = time.monotonic() - self._run_started
        else:
            elapsed = self._run_elapsed
        return {
            'scripts': list_scripts(SCRIPTS_DIR),
            'examples': self._web_examples(),
            'running': self.script_running,
            'status': self._run_status,
            'elapsed_s': round(elapsed, 1),
            'error_line': self._error_line,
            'output': self._output.tail(500, stamped=True),
            'output_seq': self._output.seq,
            'target': self.host.robot_target(rid),
            'robot': self._robot_label(rid),
        }

    def web_action(self, name: str, args: dict):
        """Run one of the web page's actions; always returns a dict."""
        if name in ('open', 'example'):
            folder = SCRIPTS_DIR if name == 'open' else EXAMPLES_DIR
            script = check_name(args.get('name'))
            return {'name': script, 'code': read_script(folder, script)}
        if name in ('save', 'save_as'):
            script = check_name(args.get('name'))
            path = os.path.join(SCRIPTS_DIR, script + '.py')
            if name == 'save_as' and os.path.exists(path):
                raise ValueError('A script with that name exists')
            os.makedirs(SCRIPTS_DIR, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(str(args.get('code') or ''))
            self._script_print(f"Script '{script}' saved")
            self._web_files_changed()
            return {'name': script}
        if name == 'delete':
            script = check_name(args.get('name'))
            path = os.path.join(SCRIPTS_DIR, script + '.py')
            if not os.path.isfile(path):
                raise ValueError('No such script')
            os.remove(path)
            self._script_print(f"Script '{script}' deleted")
            self._web_files_changed()
            return {'name': script}
        if name == 'run':
            # The Studio's own target, read now: the page asked about hardware
            # with the target of its last poll, so a run that has since become a
            # hardware run needs the page's confirmation again.
            target = self.host.robot_target(self.host.target_robot_id)
            if target == 'hardware' and args.get('hardware_ok') is not True:
                raise ValueError('Confirm running on the real robot')
            self._start_run(str(args.get('code') or ''), target)
            return {'target': target}
        if name == 'stop':
            self._stop_script()
            return {'running': self.script_running}
        if name == 'clear':
            self._clear_output()
            return {}
        raise ValueError('No such action: %s' % name)

    def _web_files_changed(self):
        """Keep the Classic page's script list in step with the web page's saves."""
        if hasattr(self, 'script_selector'):
            self._refresh_script_list()

    def _web_examples(self):
        """[{name, description}] of the example scripts; a file is read again only when it changes."""
        found = []
        for script in list_scripts(EXAMPLES_DIR):
            path = os.path.join(EXAMPLES_DIR, script + '.py')
            try:
                mtime = os.path.getmtime(path)
                cached = self._example_cache.get(script)
                if cached is None or cached[0] != mtime:
                    with open(path, encoding='utf-8', errors='replace') as fh:
                        cached = (mtime, describe(fh.read()))
                    self._example_cache[script] = cached
            except OSError:
                continue
            found.append({'name': script, 'description': cached[1]})
        return found

    def _robot_label(self, rid):
        """'Robot 1 (KinovaGen3_7DOF)': the web page's "Runs on" robot."""
        number = rid[len('robot'):] if rid.startswith('robot') else rid
        try:
            kind = self.host.current_robot_name()
        except Exception:  # noqa: BLE001
            kind = ''
        return 'Robot %s' % number + (' (%s)' % kind if kind else '')

    # ── Build ────────────────────────────────────────
    def build_page(self) -> QWidget:
        """Build the redesigned Script Editor page.

        Layout
        ------
        Header row (title + Target Robot combo) at the top.

        Below: a horizontal QSplitter split 60/40:
          * Left pane: QTabWidget with two tabs
              - "Editor" tab: toolbar (Run/Stop/Step; New/Open/Save; Target)
                              + CodeEditor
              - "Examples" tab: gallery list of ~/.fic_studio_scripts/examples/
          * Right pane: vertical QSplitter split 50/50
              - Top: Live Telemetry card (joints / EE pose / wrench at ~10 Hz)
              - Bottom: searchable API Reference

        Output console is a horizontal QSplitter below the main area, docked
        at the bottom with a fixed initial size.
        """
        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(15, 15, 15, 15)
        root.setSpacing(8)

        # ── Header row ──────────────────────────────────────────────────
        header_row = QHBoxLayout()
        header = QLabel('Script Editor')
        header.setObjectName('PageTitle')
        header_row.addWidget(header)
        header_row.addStretch()
        header_row.addWidget(QLabel('Target Robot:'))
        header_row.addWidget(self.host.make_target_robot_combo())
        root.addLayout(header_row)

        # ── Outer vertical splitter: [main area] / [console] ────────────
        outer_split = QSplitter(Qt.Vertical)

        # ── Main area: left tabs | right (telem + api ref) ──────────────
        main_split = QSplitter(Qt.Horizontal)

        # ── Left pane: Editor tab + Gallery tab ──────────────────────────
        left_tabs = QTabWidget()
        self._left_tabs = left_tabs

        # Editor tab
        editor_tab = self._build_editor_tab()
        left_tabs.addTab(editor_tab, '✎ Editor')

        # Gallery tab
        gallery_tab = self._build_gallery_tab()
        left_tabs.addTab(gallery_tab, '⬚ Examples')

        _theme.bind(
            left_tabs,
            lambda T: (
                'QTabWidget::pane {'
                f'  border: 1px solid {T["border_subtle"]};'
                f'  background: {T["bg_raised"]};'
                '}'
                'QTabBar::tab {'
                f'  background: {T["bg_overlay"]};'
                f'  color: {T["text_secondary"]};'
                '  padding: 4px 12px;'
                f'  border: 1px solid {T["border_subtle"]};'
                '}'
                'QTabBar::tab:selected {'
                f'  background: {T["bg_raised"]};'
                f'  color: {T["text_primary"]};'
                f'  border-bottom: 2px solid {T["accent_orange"]};'
                '}'
            ),
        )

        main_split.addWidget(left_tabs)

        # ── Right pane: Telemetry (top) + API Reference (bottom) ─────────
        right_split = QSplitter(Qt.Vertical)
        right_split.addWidget(self._build_telemetry_card())
        right_split.addWidget(self._build_api_reference_panel())
        right_split.setSizes([280, 280])

        main_split.addWidget(right_split)
        main_split.setSizes([600, 340])

        outer_split.addWidget(main_split)

        # ── Bottom: output console ────────────────────────────────────────
        outer_split.addWidget(self._build_output_console())
        outer_split.setSizes([560, 160])

        root.addWidget(outer_split)

        self._refresh_script_list()
        self._refresh_gallery()
        self._page = page
        return page

    # ── Editor tab ────────────────────────────────────────────────────
    def _build_editor_tab(self) -> QWidget:
        """Return the editor-tab widget: toolbar + CodeEditor."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # Toolbar row 1: script selector + file ops
        row1 = QHBoxLayout()
        row1.setSpacing(4)

        self.script_selector = QComboBox()
        self.script_selector.setMinimumWidth(160)
        self.script_selector.currentTextChanged.connect(self._on_script_selected)
        row1.addWidget(self.script_selector, 1)

        for lbl, slot, width, obj in [
            ('New', self._new_script, 56, 'BtnPrimary'),
            ('Open', self._open_script_file, 56, 'BtnPrimary'),
            ('Save', self._save_script, 56, 'BtnPrimary'),
            ('Save As…', self._save_script_as, 72, 'BtnPrimary'),
            ('Delete', self._delete_script, 60, 'BtnWarn'),
        ]:
            btn = QPushButton(lbl)
            btn.setObjectName(obj)
            btn.setFixedWidth(width)
            btn.clicked.connect(slot)
            row1.addWidget(btn)

        row1.addStretch()
        lay.addLayout(row1)

        # Toolbar row 2: Run / Stop / (Step placeholder) | Target
        row2 = QHBoxLayout()
        row2.setSpacing(4)

        self.script_run_btn = QPushButton('▶ Run')
        self.script_run_btn.setObjectName('BtnGreen')
        self.script_run_btn.setFixedWidth(80)
        self.script_run_btn.clicked.connect(self._run_script)
        row2.addWidget(self.script_run_btn)

        self.script_stop_btn = QPushButton('■ Stop')
        self.script_stop_btn.setObjectName('BtnRed')
        self.script_stop_btn.setFixedWidth(80)
        self.script_stop_btn.setEnabled(False)
        self.script_stop_btn.clicked.connect(self._stop_script)
        row2.addWidget(self.script_stop_btn)

        step_btn = QPushButton('⏩ Step')
        step_btn.setObjectName('BtnPrimary')
        step_btn.setFixedWidth(72)
        step_btn.setEnabled(False)
        step_btn.setToolTip('Single-step execution (not yet implemented)')
        row2.addWidget(step_btn)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        row2.addWidget(sep)

        row2.addWidget(QLabel('Target:'))
        self.script_target_dropdown = QComboBox()
        self.script_target_dropdown.addItems(['Simulation', 'Hardware', 'VR'])
        self.script_target_dropdown.setFixedWidth(110)
        self.script_target_dropdown.setToolTip(
            'Simulation = MuJoCo; Hardware = real robot; VR = VR relay')
        self.script_target_dropdown.currentTextChanged.connect(
            self._on_script_target_changed)
        row2.addWidget(self.script_target_dropdown)

        row2.addStretch()

        install_btn = QPushButton('Install API')
        install_btn.setObjectName('BtnPrimary')
        install_btn.setFixedWidth(90)
        install_btn.setToolTip('pip install -e fic_api')
        install_btn.clicked.connect(self._install_fic_api)
        row2.addWidget(install_btn)

        lay.addLayout(row2)

        self.script_editor = CodeEditor()
        self.script_editor.setObjectName('CodeEditor')
        self.script_highlighter = PythonHighlighter(self.script_editor.document())
        self.script_editor.setPlainText(self._get_default_script())
        lay.addWidget(self.script_editor, 1)

        return w

    # ── Gallery tab ───────────────────────────────────────────────────
    def _build_gallery_tab(self) -> QWidget:
        """Return the Example Gallery tab: list of example scripts."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        header = QLabel('Example Scripts')
        header.setObjectName('SectionTitle')
        lay.addWidget(header)

        hint = QLabel(
            'Double-click an example to load it into the editor. '
            'Examples live in ~/.fic_studio_scripts/examples/.')
        hint.setWordWrap(True)
        hint.setObjectName('HintLabel')
        lay.addWidget(hint)

        self._gallery_list = QListWidget()
        self._gallery_list.itemDoubleClicked.connect(self._on_gallery_item_double_clicked)
        _theme.bind(
            self._gallery_list,
            lambda T: (
                'QListWidget {'
                f'  background: {T["bg_overlay"]};'
                f'  color: {T["text_primary"]};'
                f'  border: 1px solid {T["border_subtle"]};'
                '}'
                'QListWidget::item:hover {'
                f'  background: {T["bg_hover"]};'
                '}'
                'QListWidget::item:selected {'
                f'  background: {T["bg_selected"]};'
                f'  color: {T["text_on_accent"]};'
                '}'
            ),
        )
        lay.addWidget(self._gallery_list, 1)

        refresh_btn = QPushButton('Refresh')
        refresh_btn.setObjectName('BtnPrimary')
        refresh_btn.clicked.connect(self._refresh_gallery)
        lay.addWidget(refresh_btn)

        return w

    # ── Live Telemetry card ───────────────────────────────────────────
    def _build_telemetry_card(self) -> QGroupBox:
        """Return the always-on Live Telemetry card.

        Reads host joint/EE/wrench helpers every 100 ms via a QTimer.
        Works even when no script is running.
        """
        grp = QGroupBox('Live Telemetry')
        _theme.bind(
            grp,
            lambda T: (
                f'QGroupBox {{ border: 1px solid {T["border_medium"]};'
                f'  border-radius: 4px;'
                f'  margin-top: 6px;'
                f'  color: {T["text_secondary"]};'
                '}}'
                f'QGroupBox::title {{ subcontrol-origin: margin;'
                f'  left: 8px; padding: 0 4px;'
                f'  color: {T["accent_orange"]};'
                '}}'
            ),
        )
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        # Joint positions row header
        j_hdr = QLabel('Joints (deg):')
        j_hdr.setObjectName('SectionSubtitle')
        lay.addWidget(j_hdr)

        joint_grid = QHBoxLayout()
        joint_grid.setSpacing(4)
        self._telem_joint_labels = []
        for i in range(7):
            lbl = QLabel(f'J{i + 1}: ---')
            lbl.setObjectName('TelemetryValue')
            _theme.bind(
                lbl,
                lambda T, _i=i: (
                    f'QLabel {{ color: {T["text_primary"]};'
                    f'  background: {T["bg_overlay"]};'
                    '  padding: 2px 4px;'
                    f'  border: 1px solid {T["border_subtle"]};'
                    '  border-radius: 2px;'
                    '}}'
                ),
            )
            joint_grid.addWidget(lbl)
            self._telem_joint_labels.append(lbl)
        lay.addLayout(joint_grid)

        # EE pose row
        ee_hdr = QLabel('EE Pose (x y z m):')
        ee_hdr.setObjectName('SectionSubtitle')
        lay.addWidget(ee_hdr)

        self._telem_ee_label = QLabel('x: ---  y: ---  z: ---')
        self._telem_ee_label.setObjectName('TelemetryValue')
        _theme.bind(
            self._telem_ee_label,
            lambda T: (
                f'QLabel {{ color: {T["text_primary"]};'
                f'  background: {T["bg_overlay"]};'
                '  padding: 2px 6px;'
                f'  border: 1px solid {T["border_subtle"]};'
                '  border-radius: 2px;'
                '}}'
            ),
        )
        lay.addWidget(self._telem_ee_label)

        # Wrench row
        w_hdr = QLabel('Wrench (N / Nm):')
        w_hdr.setObjectName('SectionSubtitle')
        lay.addWidget(w_hdr)

        self._telem_wrench_label = QLabel('Fx: ---  Fy: ---  Fz: ---  Tx: ---')
        self._telem_wrench_label.setObjectName('TelemetryValue')
        _theme.bind(
            self._telem_wrench_label,
            lambda T: (
                f'QLabel {{ color: {T["text_primary"]};'
                f'  background: {T["bg_overlay"]};'
                '  padding: 2px 6px;'
                f'  border: 1px solid {T["border_subtle"]};'
                '  border-radius: 2px;'
                '}}'
            ),
        )
        lay.addWidget(self._telem_wrench_label)
        lay.addStretch()

        return grp

    # ── Searchable API Reference panel ────────────────────────────────
    def _build_api_reference_panel(self) -> QGroupBox:
        """Return the searchable API Reference panel.

        A QLineEdit filter over api_reference(api) entries rendered as
        'name(signature) — doc'.  Filtering is case-insensitive substring.
        The api_reference is built lazily on first view (using a placeholder
        RobotAPI instance so no live node is needed).
        """
        grp = QGroupBox('API Reference')
        _theme.bind(
            grp,
            lambda T: (
                f'QGroupBox {{ border: 1px solid {T["border_medium"]};'
                f'  border-radius: 4px;'
                f'  margin-top: 6px;'
                f'  color: {T["text_secondary"]};'
                '}}'
                f'QGroupBox::title {{ subcontrol-origin: margin;'
                f'  left: 8px; padding: 0 4px;'
                f'  color: {T["accent_orange"]};'
                '}}'
            ),
        )
        lay = QVBoxLayout(grp)
        lay.setSpacing(4)

        self._api_ref_filter = QLineEdit()
        self._api_ref_filter.setPlaceholderText('Filter API…')
        self._api_ref_filter.textChanged.connect(self._on_api_filter_changed)
        _theme.bind(
            self._api_ref_filter,
            lambda T: (
                f'QLineEdit {{ background: {T["bg_overlay"]};'
                f'  color: {T["text_primary"]};'
                f'  border: 1px solid {T["border_medium"]};'
                '  border-radius: 3px; padding: 3px 6px;'
                '}}'
            ),
        )
        lay.addWidget(self._api_ref_filter)

        self._api_ref_list = QListWidget()
        _theme.bind(
            self._api_ref_list,
            lambda T: (
                'QListWidget {'
                f'  background: {T["bg_overlay"]};'
                f'  color: {T["text_primary"]};'
                f'  border: 1px solid {T["border_subtle"]};'
                '}'
                'QListWidget::item:hover {'
                f'  background: {T["bg_hover"]};'
                '}'
                'QListWidget::item:selected {'
                f'  background: {T["bg_selected"]};'
                '}}'
            ),
        )
        lay.addWidget(self._api_ref_list, 1)

        # Populate the list after the page is visible (deferred so host/node
        # are ready; triggered once by the filter signal with empty text).
        self._api_ref_entries = []
        self._api_ref_list.itemDoubleClicked.connect(
            self._on_api_ref_item_double_clicked)

        # Populate immediately (uses a stub if node is unavailable)
        self._populate_api_reference()

        return grp

    # ── Output console ────────────────────────────────────────────────
    def _build_output_console(self) -> QGroupBox:
        """Return the output console group."""
        grp = QGroupBox('Output Console')
        lay = QVBoxLayout(grp)
        self.script_output = QTextEdit()
        self.script_output.setReadOnly(True)
        self.script_output.setObjectName('OutputConsole')
        lay.addWidget(self.script_output)
        btns = QHBoxLayout()
        clear_btn = QPushButton('Clear')
        clear_btn.setObjectName('BtnPrimary')
        clear_btn.clicked.connect(self._clear_output)
        btns.addWidget(clear_btn)
        btns.addStretch()
        lay.addLayout(btns)
        return grp

    # ── Default script + API reference ────────────────────────────────
    def _get_default_script(self):
        return '''# Robot Control Script
# Functions highlighted in cyan are the robot API

mode = get_target_mode()
print(f"Running in {mode} mode")

q = get_current_joints()
qt = get_target_joints()
err = get_joint_errors()
tau = get_control_effort()

print(f"Joint positions (deg): {[f'{math.degrees(r):.1f}' for r in q]}")
print(f"Joint errors    (deg): {[f'{math.degrees(e):.1f}' for e in err]}")
print(f"Control effort  (Nm) : {[f'{t:.2f}' for t in tau]}")

ee = get_ee_position()
if ee:
    print(f"EE position: x={ee[0]:.3f}, y={ee[1]:.3f}, z={ee[2]:.3f}")
    ee_err = get_ee_error()
    print(f"EE error:    {[f'{e:.4f}' for e in ee_err]}")
else:
    print("(Joint mode - no Cartesian data)")

home()
joints = get_joint_positions()
print(f"GUI joint positions (deg): {[f'{j:.1f}' for j in joints]}")

print("Script completed!")
'''


    # ── Telemetry updater ──────────────────────────────────────
    def _update_telemetry(self):
        """Refresh the Live Telemetry labels from host helpers (~10 Hz).

        Safe to call even when no script is running; simply reads the host
        joint/EE/wrench data and updates the pre-built labels.  No-ops
        gracefully if the labels have not yet been built or if the host
        raises an exception.
        """
        if not self._telem_joint_labels:
            return
        try:
            rid = self.host.target_robot_id
            # Joint positions (degrees)
            joints_rad = self.host.joint_positions_for(rid) or []
            for i, lbl in enumerate(self._telem_joint_labels):
                if i < len(joints_rad):
                    deg = math.degrees(joints_rad[i])
                    lbl.setText(f'J{i + 1}: {deg:.1f}°')
                else:
                    lbl.setText(f'J{i + 1}: ---')
            # EE pose
            pose = self.host.ee_pose_for(rid)
            if pose is not None and self._telem_ee_label is not None:
                p = pose.position
                self._telem_ee_label.setText(
                    f'x: {p.x:.3f}  y: {p.y:.3f}  z: {p.z:.3f} m')
            elif self._telem_ee_label is not None:
                self._telem_ee_label.setText('x: ---  y: ---  z: ---')
            # Wrench
            wrench = self.host.wrench_for(rid) if hasattr(self.host, 'wrench_for') else None
            if wrench is not None and self._telem_wrench_label is not None:
                self._telem_wrench_label.setText(
                    f'Fx: {wrench[0]:.2f}  Fy: {wrench[1]:.2f}'
                    f'  Fz: {wrench[2]:.2f}  Tx: {wrench[3]:.2f} Nm')
            elif self._telem_wrench_label is not None:
                self._telem_wrench_label.setText('Fx: ---  Fy: ---  Fz: ---  Tx: ---')
        except Exception:  # noqa: BLE001 — host may not be fully initialised
            pass

    # ── Gallery helpers ────────────────────────────────────────
    def _refresh_gallery(self):
        """Repopulate the Example Gallery list from the examples directory."""
        if not hasattr(self, '_gallery_list') or self._gallery_list is None:
            return
        self._gallery_list.clear()
        if not os.path.isdir(EXAMPLES_DIR):
            return
        for fname in sorted(os.listdir(EXAMPLES_DIR)):
            if fname.endswith('.py'):
                item = QListWidgetItem(fname)
                item.setToolTip(os.path.join(EXAMPLES_DIR, fname))
                self._gallery_list.addItem(item)

    def _on_gallery_item_double_clicked(self, item):
        """Load the double-clicked example into the code editor."""
        fname = item.text()
        fpath = os.path.join(EXAMPLES_DIR, fname)
        if not os.path.exists(fpath):
            return
        try:
            with open(fpath, encoding='utf-8') as fh:
                code = fh.read()
            self.script_editor.setPlainText(code)
            # Switch to the Editor tab (index 0)
            self._left_tabs.setCurrentIndex(0)
        except Exception as exc:  # noqa: BLE001
            self._script_print(f'[Gallery] Could not load {fname}: {exc}')

    # ── Open file dialog ───────────────────────────────────────
    def _open_script_file(self):
        """Open a Python file from disk into the code editor."""
        fpath, _ = QFileDialog.getOpenFileName(
            self._page, 'Open Script', SCRIPTS_DIR, 'Python Files (*.py)')
        if fpath:
            try:
                with open(fpath, encoding='utf-8') as fh:
                    self.script_editor.setPlainText(fh.read())
            except Exception as exc:  # noqa: BLE001
                QMessageBox.warning(
                    self._page, 'Open Error', f'Could not open file:\n{exc}')

    # ── API reference helpers ──────────────────────────────────
    def _populate_api_reference(self):
        """Build the api_reference entries and populate the list widget.

        Uses a stub RobotAPI (no live node/host) so the panel works before
        any script runs.  Caches entries in self._api_ref_entries.
        """
        from Extensions.script_editor.robot_api import api_reference as _api_ref
        try:
            # Build a minimal stub API for reference generation
            api = self._robot_api
            if api is None:
                rid = self.host.target_robot_id
                stop_ev = self._stop_event
                api = RobotAPI(
                    host=self.host,
                    node=self.host.node,
                    rid=rid,
                    target=self.script_target_mode,
                    marshal=self._marshal,
                    is_stopped=stop_ev.is_set,
                )
            self._api_ref_entries = _api_ref(api)
        except Exception:  # noqa: BLE001
            self._api_ref_entries = []
        self._on_api_filter_changed(
            self._api_ref_filter.text() if self._api_ref_filter else '')

    def _on_api_filter_changed(self, text):
        """Filter the API reference list by the given text (case-insensitive)."""
        if self._api_ref_list is None:
            return
        self._api_ref_list.clear()
        needle = text.strip().lower()
        for entry in self._api_ref_entries:
            display = f'{entry["name"]}{entry["signature"]}'
            if entry.get('doc'):
                display += f'  — {entry["doc"]}'
            if needle and needle not in display.lower():
                continue
            item = QListWidgetItem(display)
            item.setToolTip(
                f'{entry["name"]}{entry["signature"]}\n\n{entry.get("doc", "")}')
            self._api_ref_list.addItem(item)

    def _on_api_ref_item_double_clicked(self, item):
        """Insert the selected API call name into the editor at the cursor."""
        text = item.text()
        # Extract just the name part (before the first '(')
        name = text.split('(')[0].split('—')[0].strip()
        if name and hasattr(self, 'script_editor'):
            cursor = self.script_editor.textCursor()
            cursor.insertText(name)
            self.script_editor.setFocus()

    # ── Trajectory subscriber ─────────────────────────────────────────
    def _ensure_traj_sub(self):
        if self._script_traj_sub is not None:
            return
        from std_msgs.msg import Float64MultiArray
        node = self.host.node

        def _cb(msg):
            self._script_traj_raw = list(msg.data)
            n = self.host.joint_count()
            self._script_traj_parsed = _parse_trajectory_data(self._script_traj_raw, n)

        rid = self.host.target_robot_id
        self._script_traj_sub = node.create_subscription(
            Float64MultiArray, f'{rid}_trajectory_data', _cb, 10)
        self._script_print(f"Subscribed to {rid}_trajectory_data")

    def _get_traj_field(self, field, wait_first=True):
        self._ensure_traj_sub()
        if self._script_traj_parsed is None and wait_first:
            time.sleep(0.15)
        return self._script_traj_parsed.get(field) if self._script_traj_parsed else None

    # ── Generic topic subscription ────────────────────────────────────
    def _script_subscribe_topic(self, topic_name):
        if topic_name in self._script_topic_subs:
            return
        node = self.host.node
        type_str = next((types[0] for name, types in node.get_topic_names_and_types()
                         if name == topic_name and types), None)
        if type_str is None:
            self._script_print(f"[WARN] Topic '{topic_name}' not found on ROS graph")
            return
        msg_cls = _import_msg_class(type_str)
        if msg_cls is None:
            self._script_print(f"[WARN] Cannot import msg type '{type_str}' for {topic_name}")
            return

        def _cb(msg, _topic=topic_name):
            with self._script_topic_lock:
                self._script_topic_cache[_topic] = _parse_ros_msg(msg)

        sub = node.create_subscription(msg_cls, topic_name, _cb, 10)
        self._script_topic_subs[topic_name] = sub
        self._script_print(f"Subscribed to {topic_name} [{type_str}]")

    def _script_cleanup_all_topic_subs(self):
        for sub in self._script_topic_subs.values():
            try:
                self.host.node.destroy_subscription(sub)
            except Exception:
                pass
        self._script_topic_subs.clear()
        with self._script_topic_lock:
            self._script_topic_cache.clear()
        if self._script_traj_sub is not None:
            try:
                self.host.node.destroy_subscription(self._script_traj_sub)
            except Exception:
                pass
            self._script_traj_sub = None
            self._script_traj_raw = []
            self._script_traj_parsed = None

    # ── Robot API ─────────────────────────────────────────────────────
    def _create_robot_api(self, target=None):
        """
        Build a RobotAPI + exec namespace for the current run.

        Returns a dict suitable for passing directly to exec().  The dict
        contains the grouped ``robot`` object plus all flat backward-compatible
        aliases (move_joints, set_mode, …) so existing scripts continue to work
        without modification.

        Legacy trajectory / topic helpers that pre-date the RobotAPI are also
        included as flat aliases so old scripts are fully backward-compatible.

        target: where the script's commands go; the Classic page's Target
        dropdown when None.
        """
        target = target or self.script_target_mode
        rid = self.host.target_robot_id
        node = self.host.node
        api = RobotAPI(
            host=self.host,
            node=node,
            rid=rid,
            target=target,
            marshal=self._marshal,
            is_stopped=self._stop_event.is_set,
        )
        self._robot_api = api

        # Build the namespace from the new API (includes flat aliases)
        ns = build_namespace(api)

        # ── Legacy helpers not yet in RobotAPI — kept for backward compat ──
        ext = self

        def get_gripper_width():
            """Return current gripper width from the slider, or 0.04 m."""
            mw = ext.host.main_window if hasattr(ext.host, 'main_window') else None
            if mw is not None and hasattr(mw, 'gripper_slider'):
                return mw.gripper_slider.value() / 1000.0
            return 0.04

        def set_velocity(v):
            """Log a velocity change (no publisher path in current firmware)."""
            ext._script_print(f'Velocity set to {max(0.0, min(1.0, v)):.1%}')

        def get_trajectory():
            """Return the last parsed trajectory dict from /trajectory_data."""
            ext._ensure_traj_sub()
            if ext._script_traj_parsed is None:
                time.sleep(0.15)
            return ext._script_traj_parsed or {}

        def _traj_field(f):
            return ext._get_traj_field(f) or [0.0] * 7

        def get_current_joints():
            """Return q_current from the trajectory topic (rad)."""
            return _traj_field('q_current')

        def get_target_joints():
            """Return q_target from the trajectory topic (rad)."""
            return _traj_field('q_target')

        def get_joint_errors():
            """Return joint errors from the trajectory topic (rad)."""
            return _traj_field('error')

        def get_control_effort():
            """Return control effort from the trajectory topic (Nm)."""
            return _traj_field('control_effort')

        def get_planner_targets():
            """Return planner targets from the trajectory topic (rad)."""
            return _traj_field('planner_target')

        def get_admittance_targets():
            """Return admittance targets from the trajectory topic (rad)."""
            return _traj_field('q_target_adm')

        def get_desired_effort():
            """Return desired effort from the trajectory topic (Nm)."""
            return _traj_field('desired_effort')

        def get_measured_effort():
            """Return measured effort from the trajectory topic (Nm)."""
            return _traj_field('measured_effort')

        def get_ee_desired():
            """Return EE desired position [x, y, z] or None."""
            return ext._get_traj_field('ee_desired')

        def get_ee_error():
            """Return EE position error [x, y, z] or None."""
            return ext._get_traj_field('ee_error')

        def get_ee_admittance_target():
            """Return EE admittance target [x, y, z] or None."""
            return ext._get_traj_field('ee_admittance_target')

        def get_rotation_error():
            """Return rotation error [ax, ay, az, angle] or None."""
            axis = ext._get_traj_field('rotation_error')
            if axis is None:
                return None
            return axis + [ext._get_traj_field('rotation_angle') or 0.0]

        def get_torque_commands():
            """Subscribe to and return the latest FIC torque commands."""
            t = f'/{rid}_fic_torque_commands'
            if t not in ext._script_topic_subs:
                ext._script_subscribe_topic(t)
                time.sleep(0.15)
            with ext._script_topic_lock:
                return ext._script_topic_cache.get(t)

        # Merge legacy helpers that are NOT already covered by build_namespace.
        # IMPORTANT: do NOT include subscribe/unsubscribe/get_topic/list_topics
        # here — build_namespace already maps those to api.topics.* which uses
        # the single TopicsAPI._cache.  Overwriting them here would create a
        # second independent cache (extension._script_topic_cache) and break
        # scripts that call subscribe(t) then robot.topics.get(t) or vice-versa.
        legacy = {
            'get_gripper_width': get_gripper_width,
            'set_velocity': set_velocity,
            'get_trajectory': get_trajectory,
            'get_current_joints': get_current_joints,
            'get_target_joints': get_target_joints,
            'get_joint_errors': get_joint_errors,
            'get_control_effort': get_control_effort,
            'get_planner_targets': get_planner_targets,
            'get_admittance_targets': get_admittance_targets,
            'get_desired_effort': get_desired_effort,
            'get_measured_effort': get_measured_effort,
            'get_ee_desired': get_ee_desired,
            'get_ee_error': get_ee_error,
            'get_ee_admittance_target': get_ee_admittance_target,
            'get_rotation_error': get_rotation_error,
            'get_torque_commands': get_torque_commands,
        }
        ns.update(legacy)
        # Restore the 'robot' key to the RobotAPI (not the extension self)
        ns['robot'] = api
        return ns

    # ── Output helpers ────────────────────────────────────────────────
    def _script_print(self, message):
        ts = datetime.now().strftime("%H:%M:%S")
        self._script_output_buffer.append(f"[{ts}] {message}")

    def _flush_script_output(self):
        if not self._script_output_buffer:
            return
        messages = self._script_output_buffer[:]
        self._script_output_buffer.clear()
        for msg in messages:
            # Printed text continues its line; every other message is a line.
            self._output.write(msg if isinstance(msg, _StreamChunk) else msg + '\n')
        if hasattr(self, 'script_output'):
            for msg in messages:
                self.script_output.append(msg)

    def _clear_output(self):
        """Empty the output on both pages."""
        self._script_output_buffer.clear()
        self._output.clear()
        if hasattr(self, 'script_output'):
            self.script_output.clear()

    def _on_script_target_changed(self, target):
        self.script_target_mode = target.lower()
        self._script_output_buffer.append(f"[Target changed to: {target}]")

    # ── Run / stop ────────────────────────────────────────────────────
    def _run_script(self):
        """The Classic page's Run: the editor's text, to the page's own target."""
        if self.script_running:
            return
        code = self.script_editor.toPlainText()
        if not code.strip():
            QMessageBox.warning(self._page, 'Empty Script', 'Please enter some code to run.')
            return
        self._start_run(code, self.script_target_mode)

    def _start_run(self, code, target):
        """
        Start ``code`` in a background QThread with a fresh stop event.

        Both pages run scripts through here; ``target`` is where its commands
        go ('simulation', 'hardware' or 'vr'). Raises ValueError when a script
        is already running or ``code`` is empty.
        """
        if self.script_running:
            raise ValueError('Stop the script first')
        if not str(code or '').strip():
            raise ValueError('Nothing to run')
        self._clear_output()
        self._script_output_buffer.append(f'=== Script Started (Target: {target.upper()}) ===\n')
        self.script_running = True
        self._run_status = 'running'
        self._run_started = time.monotonic()
        self._error_line = None
        if hasattr(self, 'script_run_btn'):
            self.script_run_btn.setEnabled(False)
            self.script_stop_btn.setEnabled(True)

        # Fresh stop event for each run
        self._stop_event = threading.Event()
        # Reset latched flags
        self._latched_admittance_planner = False
        self._latched_hpmp = False

        namespace = self._create_robot_api(target)
        self.script_runner = ScriptRunner(code, namespace, self._stop_event)
        self.script_runner.signals.output.connect(self._on_script_output)
        self.script_runner.signals.error.connect(self._on_script_error)
        self.script_runner.signals.finished.connect(self._on_script_finished)
        self.script_runner.start()

    def _stop_script(self):
        """
        Request cooperative stop; wait up to 2 s then terminate as fallback.

        1. Sets the stop_event so blocking loops inside the script can exit.
        2. Waits up to _GRACE_SECONDS for the QThread to finish.
        3. If the thread is still alive after the grace period, calls
           terminate() as a last resort.
        """
        if self.script_runner and self.script_runner.isRunning():
            self._run_status = 'stopped'
            # Step 1: cooperative signal
            self.script_runner.stop()
            # Step 2: wait for graceful exit
            grace = ScriptRunner._GRACE_SECONDS
            self.script_runner.wait(int(grace * 1000))  # QThread.wait() takes ms
            # Step 3: terminate fallback
            if self.script_runner.isRunning():
                self.script_runner.terminate()
                self.script_runner.wait()
                self._script_output_buffer.append('\n[FORCE TERMINATED — grace period expired]')
                self.host.log('WARN', 'Script terminated after grace period', 'Script')
            else:
                self._script_output_buffer.append('\n[Stopped cooperatively]')
            self.script_running = False
            self._run_elapsed = time.monotonic() - (self._run_started or time.monotonic())
            if hasattr(self, 'script_run_btn'):
                self.script_run_btn.setEnabled(True)
                self.script_stop_btn.setEnabled(False)
        self._script_cleanup_all_topic_subs()

    def _on_script_output(self, text):
        self._script_output_buffer.append(_StreamChunk(text))

    def _on_script_error(self, error):
        self._error_line = failing_line(error)
        self._script_output_buffer.append(f"\n*** ERROR ***\n{error}")

    def _on_script_finished(self, success, message):
        self.script_running = False
        if self._run_status == 'running':          # a Stop already said 'stopped'
            self._run_status = 'finished' if success else 'failed'
        self._run_elapsed = time.monotonic() - (self._run_started or time.monotonic())
        if hasattr(self, 'script_run_btn'):
            self.script_run_btn.setEnabled(True)
            self.script_stop_btn.setEnabled(False)
        status = "completed" if success else "failed"
        self._script_output_buffer.append(f"\n=== Script {status}: {message} ===")
        self._script_cleanup_all_topic_subs()

    # ── Script file management ────────────────────────────────────────
    def _refresh_script_list(self):
        current = self.script_selector.currentText()
        self.script_selector.blockSignals(True)
        self.script_selector.clear()
        self.script_selector.addItem("(Untitled)")
        if os.path.exists(SCRIPTS_DIR):
            for filename in sorted(os.listdir(SCRIPTS_DIR)):
                if filename.endswith('.py'):
                    self.script_selector.addItem(filename[:-3])
        idx = self.script_selector.findText(current)
        if idx >= 0:
            self.script_selector.setCurrentIndex(idx)
        self.script_selector.blockSignals(False)

    def _on_script_selected(self, name):
        if name in ("(Untitled)", ""):
            return
        filepath = os.path.join(SCRIPTS_DIR, f"{name}.py")
        if os.path.exists(filepath):
            try:
                with open(filepath) as f:
                    self.script_editor.setPlainText(f.read())
            except Exception as e:
                QMessageBox.warning(self._page, "Load Error", f"Could not load script:\n{e}")

    def _new_script(self):
        self.script_selector.setCurrentIndex(0)
        self.script_editor.setPlainText(self._get_default_script())

    def _save_script(self):
        name = self.script_selector.currentText()
        if name in ("(Untitled)", ""):
            self._save_script_as()
            return
        filepath = os.path.join(SCRIPTS_DIR, f"{name}.py")
        try:
            with open(filepath, 'w') as f:
                f.write(self.script_editor.toPlainText())
            self._script_print(f"Script '{name}' saved")
        except Exception as e:
            QMessageBox.warning(self._page, "Save Error", f"Could not save script:\n{e}")

    def _save_script_as(self):
        name, ok = QInputDialog.getText(self._page, "Save Script", "Script name:")
        if not ok or not name:
            return
        name = name.strip().replace(' ', '_')
        if not name:
            return
        filepath = os.path.join(SCRIPTS_DIR, f"{name}.py")
        try:
            with open(filepath, 'w') as f:
                f.write(self.script_editor.toPlainText())
            self._refresh_script_list()
            self.script_selector.setCurrentText(name)
            self._script_print(f"Script saved as '{name}'")
        except Exception as e:
            QMessageBox.warning(self._page, "Save Error", f"Could not save script:\n{e}")

    def _delete_script(self):
        name = self.script_selector.currentText()
        if name in ("(Untitled)", ""):
            return
        if QMessageBox.question(self._page, "Delete Script",
                                f"Are you sure you want to delete '{name}'?",
                                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            try:
                os.remove(os.path.join(SCRIPTS_DIR, f"{name}.py"))
                self._refresh_script_list()
                self._new_script()
                self._script_print(f"Script '{name}' deleted")
            except Exception as e:
                QMessageBox.warning(self._page, "Delete Error", f"Could not delete script:\n{e}")

    def _install_fic_api(self):
        if not os.path.isfile(os.path.join(_FIC_API_DIR, "setup.py")):
            QMessageBox.warning(
                self._page, "Not Found",
                f"fic_api package not found at:\n{_FIC_API_DIR}\n\n"
                "Expected a setup.py in that directory.")
            return
        from fic_studio.widgets import BuildDialog
        dlg = BuildDialog(
            title="Install FIC API",
            command=f"python3 -m pip install --break-system-packages -e {_FIC_API_DIR}",
            work_dir=_FIC_API_DIR,
            parent=self._page,
        )
        self.host.log("INFO", "Installing fic_api package", "Script")
        dlg.exec_()
