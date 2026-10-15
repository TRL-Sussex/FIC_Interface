# mixins/robot_config_mixin.py — Robot configuration management UI

import math
import os
import yaml
from functools import partial
from xml.etree import ElementTree as ET
from xml.dom import minidom

# Import XML manager for config.xml sync
try:
    from .xml_manager import sync_robots_to_config_xml
except ImportError:
    from xml_manager import sync_robots_to_config_xml

from PyQt5.QtCore import QSize, Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit,
    QComboBox, QCheckBox, QGroupBox, QGridLayout, QScrollArea, QFrame,
    QSpinBox, QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem,
    QHeaderView, QFileDialog, QMessageBox, QDialog, QDialogButtonBox,
    QTabWidget, QListWidget, QListWidgetItem, QSplitter, QFormLayout,
    QListView, QMenu, QToolButton, QApplication,
)

from fic_studio.config import WS_PATH as _FIC_WS, ROBOT_LIB as _FIC_ROBOT_LIB
from fic_studio import theme
from fic_studio.components.control_widgets import MetricField, CollapsibleSection
from fic_studio.components.joint_table import JointTable
from fic_studio.robot_import import check as rc_check


# ═══════════════════════════════════════════════════════════════════════════════
#  DEFAULT ROBOT CONFIGURATIONS (matches C++ nodes)
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_ROBOT_CONFIGS = {
    "Franka_FR3": {
        'display_name': 'Franka FR3',
        'num_joints': 7,
        'joint_names': ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
        'urdf_path': 'menagerie/model/Franka_FR3/URDF/Franka_FR3.urdf',
        'mujoco_xml_path': 'menagerie/model/Franka_FR3/xml/scene.xml',
        'ee_frame_name': 'ee_link',
        'elbow_frame_name': 'fr3_link4',
        'default_ee_target': {
            'position': [0.4709, -0.0136, -0.400],
            'orientation': [-1.0, 0.0, 0.0, 0.0]  # [x, y, z, w]
        },
        'q_home_deg': [-9.0, -16.6, 5.5, -126.2, 5.4, 110.7, 44.0],
        'q_home_rad': [-0.157, 0.290, 0.096, 2.203, 0.094, 1.932, 0.768],
        'joint_limits_deg': {
            'lower': [-166, -101, -166, -176, -166, -1, -166],
            'upper': [166, 101, 166, -4, 166, 215, 166]
        },
        'joint_torque_limits': {
            'min': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'max': [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
        },
        'has_gripper': True,
        'gripper_range': [0.0, 0.08],
        'torque_sign': -1,
        'gravity_compensated': True,
        'wrench_from_api': True,
        'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
        'is_supported': True,
        'driver_command': 'cd {robot_lib}/libfranka/build && ./examples/cartesian_impedance_control 172.16.0.2',
        'build_directory': '{robot_lib}/libfranka/build',
        'build_command': 'cmake .. && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)'
    },
    "KinovaGen3_7DOF": {
        'display_name': 'Kinova Gen3 7DOF',
        'num_joints': 7,
        'joint_names': ['Actuator1', 'Actuator2', 'Actuator3', 'Actuator4', 'Actuator5', 'Actuator6', 'Actuator7'],
        'urdf_path': 'menagerie/model/KinovaGen3_7DOF/URDF/KinovaGen3_7DOF.urdf',
        'mujoco_xml_path': 'menagerie/model/KinovaGen3_7DOF/xml/scene.xml',
        'ee_frame_name': 'ee_link',
        'elbow_frame_name': 'ForeArm_Link',
        'default_ee_target': {
            'position': [0.38, 0.0, 0.432],
            'orientation': [0.5, 0.5, 0.5, 0.5]
        },
        'q_home_deg': [0.0, 15.0, 180.0, -129.8, 0.0, 59.6, 90.0],
        'q_home_rad': [0.0, 0.262, 3.142, -2.268, 0.0, 1.04, 1.571],
        'sim_start_at_home': True,
        'joint_limits_deg': {
            'lower': [-360, -128, -360, -147, -360, -120, -360],
            'upper': [360, 128, 360, 147, 360, 120, 360]
        },
        'joint_torque_limits': {
            'min': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'max': [39.0, 39.0, 39.0, 39.0, 39.0, 9.0, 9.0]
        },
        'torque_sign': 1,
        'gravity_compensated': False,
        'wrench_from_api': False,
        'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
        'has_gripper': True,
        'gripper_range': [0.0, 0.085],
        'is_supported': True,
        'driver_command': 'cd {robot_lib}/Kinova/api_cpp/examples/build-gcc-release && ./01-torque-control_00-Torque_Controller',
        'build_directory': '{robot_lib}/Kinova/api_cpp/examples',
        'build_command': './scripts/build-gcc.sh'
    },
    "IIWA_14": {
        'display_name': 'KUKA iiwa 14',
        'num_joints': 7,
        'joint_names': ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7'],
        'urdf_path': 'menagerie/model/IIWA_14/URDF/IIWA_14.urdf',
        'mujoco_xml_path': 'menagerie/model/IIWA_14/XML/scene.xml',
        'ee_frame_name': 'iiwa_link_ee',
        'elbow_frame_name': 'iiwa_link_4',
        'default_ee_target': {
            'position': [0.4, 0.0, 0.6],
            'orientation': [0.0, 0.707, 0.707, 0.0]
        },
        'q_home_deg': [0.0, 0.0, 0.0, -90.0, 0.0, 90.0, 0.0],
        'q_home_rad': [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, 0.0],
        'joint_limits_deg': {
            'lower': [-170, -120, -170, -120, -170, -120, -175],
            'upper': [170, 120, 170, 120, 170, 120, 175]
        },
        'joint_torque_limits': {
            'min': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'max': [320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0]
        },
        'torque_sign': 1,
        'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
        'has_gripper': False,
        'gripper_range': [0.0, 0.0],
        'is_supported': False,
        'driver_command': '',
        'build_directory': '',
        'build_command': ''
    },
    "Pendulum": {
        'display_name': 'Simple Pendulum',
        'num_joints': 1,
        'joint_names': ['Actuator1'],
        'urdf_path': 'menagerie/model/Pendulum/URDF/Pendulum.urdf',
        'mujoco_xml_path': 'menagerie/model/Pendulum/xml/pendulum.xml',
        'ee_frame_name': 'ee_link',
        'elbow_frame_name': '',
        'default_ee_target': {
            'position': [0.0, 0.0, -1.0],
            'orientation': [0.0, 0.0, 0.0, 1.0]
        },
        'q_home_deg': [0.0],
        'q_home_rad': [0.0],
        'joint_limits_deg': {
            'lower': [-180],
            'upper': [180]
        },
        'joint_torque_limits': {
            'min': [0.0],
            'max': [50.0]
        },
        'torque_sign': 1,
        'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
        'has_gripper': False,
        'gripper_range': [0.0, 0.0],
        'is_supported': False,
        'driver_command': '',
        'build_directory': '',
        'build_command': ''
    }
}


# Keys the inline editor does not expose but that must survive a Deploy.
# _sync_to_fic_config rebuilds robots_config.yaml from an explicit whitelist,
# so anything absent here is silently dropped — which is how CSPM's closed-loop
# definition could be wiped by pressing Deploy.
OPTIONAL_ROBOT_KEYS = (
    'robot_category',
    'loop_closures',
    'ee_orientation_only',
    'view_joint_names',
    'safety_zones',
    'sim_backend',
    # With has_gripper, the controllers load this URDF instead of urdf_path
    # (fic/robot_config_loader.hpp), so gravity includes the gripper's mass.
    'urdf_path_gripper',
    # Inner-loop MCU (fic/robot_config_loader.hpp McuSettings): with an
    # adapter, the joint layer runs on the MCU whenever one answers.
    'mcu',
    # How the simulator reproduces the real arm (mujoco_native/hardware_model.py,
    # fic/robot_config_loader.hpp HardwareModel).
    'hardware_model',
)

#: Inner-loop MCU adapters the editor offers ('' = none: standard path only).
MCU_ADAPTERS = ('', 'kortex', 'canfd', 'loopback')


def merge_robot_config(old, new):
    """Merge ``new`` over ``old``, recursively for nested dicts.

    The editor rebuilds a config from its widgets, and has no widget for
    some keys -- ``loop_closures``, ``urdf_path_gripper``, a deadband's
    ``joint_tau_per_joint`` inside ``admittance_deadbands``. A plain
    ``{**old, **new}`` kept the top-level ones but replaced every nested dict
    whole, silently dropping keys like that last one on every save.
    """
    out = dict(old)
    for key, value in new.items():
        if isinstance(value, dict) and isinstance(old.get(key), dict):
            out[key] = merge_robot_config(old[key], value)
        else:
            out[key] = value
    return out


def _controller_defaults(robot_id, cfg):
    """robots_config.yaml's keys, in order, with their defaults."""
    return (
        ('display_name', robot_id),
        ('num_joints', 7),
        ('joint_names', []),
        ('urdf_path', ''),
        ('mujoco_xml_path', ''),
        ('ee_frame_name', 'ee_link'),
        ('elbow_frame_name', ''),
        # Pluggable solver backend: "pinocchio" (default), "none", or a
        # custom name registered in fic/solvers/solver_factory.hpp
        ('kinematics_solver', 'pinocchio'),
        ('default_ee_target', {}),
        ('q_home_deg', []),
        ('q_home_rad', []),
        ('joint_limits_deg', {}),
        ('joint_torque_limits', {}),
        ('torque_sign', 1),
        ('gravity_compensated', False),
        ('wrench_gravity_compensated', cfg.get('gravity_compensated', False)),
        ('wrench_from_api', False),
        ('admittance_deadbands', {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2}),
        ('has_gripper', False),
        ('gripper_range', [0.0, 0.08]),
        ('is_supported', False),
        ('driver_command', ''),
        ('build_directory', ''),
        ('build_command', ''),
    )


def controller_robots(gui_robots, existing=None):
    """Build the ``robots`` section of robots_config.yaml for the Studio's robots.

    The Studio's value wins for every key it has. What only the controllers'
    copy has -- values added there by hand, like UR5e's torque limits -- is
    kept rather than overwritten with a default, since every Save now
    rewrites this file. Robots the Studio no longer lists are left out.
    """
    existing = existing or {}
    out = {}
    for robot_id, cfg in gui_robots.items():
        old = existing.get(robot_id) or {}
        entry = {}
        for key, default in _controller_defaults(robot_id, cfg):
            entry[key] = cfg[key] if key in cfg else old.get(key, default)
        for key in OPTIONAL_ROBOT_KEYS:
            if key in cfg:
                entry[key] = cfg[key]
            elif key in old:
                entry[key] = old[key]
        for key, value in old.items():
            entry.setdefault(key, value)
        out[robot_id] = entry
    return out


def write_yaml_atomically(path, data):
    """Write ``data`` to ``path`` so a reader never sees half a file."""
    tmp = '%s.tmp%d' % (path, os.getpid())
    with open(tmp, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, path)


# ═══════════════════════════════════════════════════════════════════════════════
#  DEFAULT ROBOT TEMPLATE
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_ROBOT_TEMPLATE = {
    'display_name': 'New Robot',
    'num_joints': 7,
    'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
    'urdf_path': 'menagerie/model/NewRobot/URDF/NewRobot.urdf',
    'mujoco_xml_path': 'menagerie/model/NewRobot/xml/NewRobot.xml',
    'ee_frame_name': 'ee_link',
    'elbow_frame_name': 'link4',
    'default_ee_target': {
        'position': [0.4, 0.0, 0.4],
        'orientation': [0.0, 0.0, 0.0, 1.0]
    },
    'q_home_deg': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'q_home_rad': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'joint_limits_deg': {
        'lower': [-180, -180, -180, -180, -180, -180, -180],
        'upper': [180, 180, 180, 180, 180, 180, 180]
    },
    'joint_torque_limits': {
        'min': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'max': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
    },
    'torque_sign': 1,
    'has_gripper': False,
    'gripper_range': [0.0, 0.08],
    'admittance_deadbands': {
        'joint_tau': 1.5,
        'cart_force': 4.0,
        'cart_torque': 0.2,
    },
    'is_supported': False,
    'driver_command': '',
    'build_directory': '',
    'build_command': ''
}


# ═══════════════════════════════════════════════════════════════════════════════
#  ROBOT EDITOR DIALOG
# ═══════════════════════════════════════════════════════════════════════════════

class RobotEditorDialog(QDialog):
    """Dialog for editing robot configuration."""
    
    def __init__(self, parent=None, robot_name: str = "", robot_config: dict = None, is_new: bool = False, ws_path: str = ""):
        super().__init__(parent)
        self.robot_name = robot_name
        self.robot_config = robot_config.copy() if robot_config else DEFAULT_ROBOT_TEMPLATE.copy()
        self.is_new = is_new
        self.ws_path = ws_path or os.path.join(_FIC_WS, "src")
        self.result_config = None
        self.result_name = None
        
        self.setWindowTitle("Add New Robot" if is_new else f"Edit Robot: {robot_name}")
        self.setMinimumSize(750, 650)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Tabs for different config sections
        tabs = QTabWidget()
        
        # ── Basic Info Tab ──
        basic_tab = QWidget()
        basic_layout = QFormLayout(basic_tab)
        basic_layout.setSpacing(10)
        
        # Robot ID (internal name)
        self.robot_id_edit = QLineEdit()
        self.robot_id_edit.setText(self.robot_name)
        self.robot_id_edit.setPlaceholderText("e.g., MyRobot_7DOF (no spaces, matches URDF folder)")
        if not self.is_new:
            self.robot_id_edit.setEnabled(False)
        basic_layout.addRow("Robot ID:", self.robot_id_edit)
        
        # Display name
        self.display_name_edit = QLineEdit()
        self.display_name_edit.setText(self.robot_config.get('display_name', ''))
        self.display_name_edit.setPlaceholderText("e.g., My Custom Robot")
        basic_layout.addRow("Display Name:", self.display_name_edit)
        
        # Number of joints
        self.num_joints_spin = QSpinBox()
        self.num_joints_spin.setRange(1, 40)
        self.num_joints_spin.setValue(self.robot_config.get('num_joints', 7))
        self.num_joints_spin.valueChanged.connect(self._on_num_joints_changed)
        basic_layout.addRow("Number of Joints:", self.num_joints_spin)
        
        # Is supported (real robot control available)
        self.is_supported_check = QCheckBox("Real robot control available")
        self.is_supported_check.setChecked(self.robot_config.get('is_supported', False))
        basic_layout.addRow("Hardware Support:", self.is_supported_check)

        # Torque sign convention
        self.torque_sign_combo = QComboBox()
        self.torque_sign_combo.addItems(["Positive (+1)", "Negative (-1)"])
        current_sign = self.robot_config.get('torque_sign', 1)
        self.torque_sign_combo.setCurrentIndex(0 if current_sign >= 0 else 1)
        basic_layout.addRow("Torque Sign:", self.torque_sign_combo)

        # Gravity already compensated by driver?
        # Check this for drivers that publish gravity-removed torque
        # (e.g., Franka tau_ext_hat_filtered). Leave unchecked for raw
        # torque drivers (Kinova torque mode, most simulators).
        self.gravity_compensated_check = QCheckBox("ON = torques are gravity-free, OFF = torques include gravity")
        self.gravity_compensated_check.setChecked(self.robot_config.get('gravity_compensated', False))
        basic_layout.addRow("Joint Torque Gravity Compensated:", self.gravity_compensated_check)

        # Wrench gravity compensated?
        # Kinova's tool_external_wrench and Franka's O_F_ext_hat_K are external-only.
        # Most simulators and raw-torque drivers need this unchecked.
        self.wrench_gravity_compensated_check = QCheckBox("ON = wrench is gravity-free, OFF = wrench includes gravity")
        _wgc_default = self.robot_config.get('wrench_gravity_compensated',
                                              self.robot_config.get('gravity_compensated', False))
        self.wrench_gravity_compensated_check.setChecked(_wgc_default)
        basic_layout.addRow("Wrench Gravity Compensated:", self.wrench_gravity_compensated_check)

        self.wrench_from_api_check = QCheckBox("ON = use API wrench, OFF = compute from Jacobian")
        self.wrench_from_api_check.setChecked(self.robot_config.get('wrench_from_api', False))
        basic_layout.addRow("Wrench from API:", self.wrench_from_api_check)

        # Admittance deadbands
        db = self.robot_config.get('admittance_deadbands', {})
        deadband_row = QHBoxLayout()

        self.db_joint_tau = QDoubleSpinBox()
        self.db_joint_tau.setRange(0.0, 50.0)
        self.db_joint_tau.setDecimals(2)
        self.db_joint_tau.setSuffix(" Nm")
        self.db_joint_tau.setValue(db.get('joint_tau', 1.5))
        self.db_joint_tau.setToolTip("Joint admittance torque deadband — signals below this threshold are zeroed to suppress sensor noise/gravity-model bias.")
        deadband_row.addWidget(QLabel("Joint:"))
        deadband_row.addWidget(self.db_joint_tau)

        self.db_cart_force = QDoubleSpinBox()
        self.db_cart_force.setRange(0.0, 50.0)
        self.db_cart_force.setDecimals(2)
        self.db_cart_force.setSuffix(" N")
        self.db_cart_force.setValue(db.get('cart_force', 4.0))
        self.db_cart_force.setToolTip("Cartesian translational force deadband (N).")
        deadband_row.addWidget(QLabel("Cart Force:"))
        deadband_row.addWidget(self.db_cart_force)

        self.db_cart_torque = QDoubleSpinBox()
        self.db_cart_torque.setRange(0.0, 10.0)
        self.db_cart_torque.setDecimals(3)
        self.db_cart_torque.setSuffix(" Nm")
        self.db_cart_torque.setValue(db.get('cart_torque', 0.2))
        self.db_cart_torque.setToolTip("Cartesian rotational torque deadband (Nm).")
        deadband_row.addWidget(QLabel("Cart Torque:"))
        deadband_row.addWidget(self.db_cart_torque)

        deadband_row.addStretch()
        basic_layout.addRow("Admittance Deadbands:", deadband_row)

        # Has gripper
        self.has_gripper_check = QCheckBox()
        self.has_gripper_check.setChecked(self.robot_config.get('has_gripper', False))
        basic_layout.addRow("Has Gripper:", self.has_gripper_check)
        
        # Gripper range
        gripper_row = QHBoxLayout()
        self.gripper_min = QDoubleSpinBox()
        self.gripper_min.setRange(0.0, 0.2)
        self.gripper_min.setDecimals(4)
        self.gripper_min.setSuffix(" m")
        self.gripper_min.setValue(self.robot_config.get('gripper_range', [0.0, 0.08])[0])
        gripper_row.addWidget(QLabel("Closed:"))
        gripper_row.addWidget(self.gripper_min)
        
        self.gripper_max = QDoubleSpinBox()
        self.gripper_max.setRange(0.0, 0.2)
        self.gripper_max.setDecimals(4)
        self.gripper_max.setSuffix(" m")
        self.gripper_max.setValue(self.robot_config.get('gripper_range', [0.0, 0.08])[1])
        gripper_row.addWidget(QLabel("Open:"))
        gripper_row.addWidget(self.gripper_max)
        gripper_row.addStretch()
        basic_layout.addRow("Gripper Range:", gripper_row)
        
        tabs.addTab(basic_tab, "Basic Info")
        
        # ── File Paths Tab ──
        paths_tab = QWidget()
        paths_layout = QFormLayout(paths_tab)
        paths_layout.setSpacing(10)
        
        # URDF path
        urdf_row = QHBoxLayout()
        self.urdf_path_edit = QLineEdit()
        self.urdf_path_edit.setText(self.robot_config.get('urdf_path', ''))
        self.urdf_path_edit.setPlaceholderText("menagerie/model/Robot/URDF/Robot.urdf")
        urdf_row.addWidget(self.urdf_path_edit)
        urdf_browse = QPushButton("Browse...")
        urdf_browse.clicked.connect(lambda: self._browse_file(self.urdf_path_edit, "URDF Files (*.urdf)"))
        urdf_row.addWidget(urdf_browse)
        paths_layout.addRow("URDF Path:", urdf_row)
        
        # MuJoCo XML path
        xml_row = QHBoxLayout()
        self.mujoco_path_edit = QLineEdit()
        self.mujoco_path_edit.setText(self.robot_config.get('mujoco_xml_path', ''))
        self.mujoco_path_edit.setPlaceholderText("menagerie/model/Robot/xml/Robot.xml")
        xml_row.addWidget(self.mujoco_path_edit)
        xml_browse = QPushButton("Browse...")
        xml_browse.clicked.connect(lambda: self._browse_file(self.mujoco_path_edit, "XML Files (*.xml)"))
        xml_row.addWidget(xml_browse)
        paths_layout.addRow("MuJoCo XML Path:", xml_row)
        
        # Driver command
        self.driver_cmd_edit = QLineEdit()
        self.driver_cmd_edit.setText(self.robot_config.get('driver_command', ''))
        self.driver_cmd_edit.setPlaceholderText("Command to launch robot driver (use {robot_lib} for robot_library path)")
        paths_layout.addRow("Driver Command:", self.driver_cmd_edit)
        
        # Note about relative paths
        note = QLabel("Paths should be relative to ROS2 workspace src directory.\n"
                      "Use {robot_lib} placeholder for robot_library path in driver command.\n"
                      "Use {robot_name} placeholder for the robot ID.")
        theme.bind(note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        note.setWordWrap(True)
        paths_layout.addRow("", note)
        
        tabs.addTab(paths_tab, "File Paths")
        
        # ── Build Tab (NEW) ──
        build_tab = QWidget()
        build_layout = QFormLayout(build_tab)
        build_layout.setSpacing(10)
        
        build_desc = QLabel(
            "Configure how to compile this robot's driver/API.\n"
            "The GUI can launch the build in a terminal from the Robot Config page."
        )
        theme.bind(build_desc, lambda t: "color: %s; font-size: 11px;" % t["text_secondary"])
        build_desc.setWordWrap(True)
        build_layout.addRow("", build_desc)
        
        # Build directory
        build_dir_row = QHBoxLayout()
        self.build_dir_edit = QLineEdit()
        self.build_dir_edit.setText(self.robot_config.get('build_directory', ''))
        self.build_dir_edit.setPlaceholderText("e.g., {robot_lib}/libfranka/build")
        build_dir_row.addWidget(self.build_dir_edit)
        build_dir_browse = QPushButton("Browse...")
        build_dir_browse.clicked.connect(self._browse_build_directory)
        build_dir_row.addWidget(build_dir_browse)
        build_layout.addRow("Build Directory:", build_dir_row)
        
        # Build command
        self.build_cmd_edit = QTextEdit()
        self.build_cmd_edit.setText(self.robot_config.get('build_command', ''))
        self.build_cmd_edit.setPlaceholderText(
            "e.g., cmake .. && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)\n"
            "or: mkdir -p build && cd build && cmake .. && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)\n"
            "or: catkin_make --pkg my_robot_driver"
        )
        self.build_cmd_edit.setMaximumHeight(100)
        build_layout.addRow("Build Command:", self.build_cmd_edit)
        
        build_note = QLabel(
            "Available placeholders:\n"
            "  {robot_lib}  → the FIC_Interface/robot_library directory\n"
            "  {robot_name} → the robot's ID (e.g., Franka_FR3)\n\n"
            "The build command runs inside the build directory.\n"
            "Multi-line commands are joined with ' && '."
        )
        theme.bind(build_note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        build_note.setWordWrap(True)
        build_layout.addRow("", build_note)
        
        tabs.addTab(build_tab, "Build")
        
        # ── Frame Names Tab ──
        frames_tab = QWidget()
        frames_layout = QFormLayout(frames_tab)
        frames_layout.setSpacing(10)
        
        self.ee_frame_edit = QLineEdit()
        self.ee_frame_edit.setText(self.robot_config.get('ee_frame_name', 'ee_link'))
        self.ee_frame_edit.setPlaceholderText("e.g., ee_link, tool0, link_ee")
        frames_layout.addRow("End-Effector Frame:", self.ee_frame_edit)
        
        self.elbow_frame_edit = QLineEdit()
        self.elbow_frame_edit.setText(self.robot_config.get('elbow_frame_name', ''))
        self.elbow_frame_edit.setPlaceholderText("e.g., link4, forearm_link (for null-space control)")
        frames_layout.addRow("Elbow Frame:", self.elbow_frame_edit)

        # Kinematics solver backend: pinocchio (default), none (joint-space
        # only), or a custom name registered in fic/solvers/solver_factory.hpp
        self.kin_solver_combo = QComboBox()
        self.kin_solver_combo.setEditable(True)
        self.kin_solver_combo.addItems(["pinocchio", "none"])
        self.kin_solver_combo.setCurrentText(
            self.robot_config.get('kinematics_solver', 'pinocchio'))
        self.kin_solver_combo.setToolTip(
            "Kinematics/dynamics solver for this robot:\n"
            "  pinocchio — built-in solver (default)\n"
            "  none — joint-space control only (Cartesian modes disabled)\n"
            "  <custom> — a solver registered in fic/solvers/solver_factory.hpp\n"
            "See docs/custom-kinematics-solver.md")
        frames_layout.addRow("Kinematics Solver:", self.kin_solver_combo)
        
        frames_note = QLabel("Frame names must match exactly with your URDF link names.\n"
                             "Check your URDF for available link names.")
        theme.bind(frames_note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        frames_note.setWordWrap(True)
        frames_layout.addRow("", frames_note)
        
        tabs.addTab(frames_tab, "Frame Names")
        
        # ── Joint Names Tab ──
        joints_tab = QWidget()
        joints_layout = QVBoxLayout(joints_tab)
        
        joints_label = QLabel("Joint Names (must match URDF joint names exactly):")
        joints_layout.addWidget(joints_label)
        
        self.joint_names_edit = QTextEdit()
        joint_names = self.robot_config.get('joint_names', [])
        self.joint_names_edit.setPlainText('\n'.join(joint_names))
        self.joint_names_edit.setPlaceholderText("Enter one joint name per line:\njoint1\njoint2\n...")
        joints_layout.addWidget(self.joint_names_edit)
        
        joints_note = QLabel("Enter one joint name per line. These names are used by the FIC controller\n"
                             "to map joint states from MuJoCo/hardware to the controller.")
        theme.bind(joints_note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        joints_layout.addWidget(joints_note)
        
        tabs.addTab(joints_tab, "Joint Names")
        
        # ── Default Pose Tab ──
        pose_tab = QWidget()
        pose_layout = QVBoxLayout(pose_tab)
        
        # Home joint angles
        home_group = QGroupBox("Home Joint Angles (degrees)")
        home_layout = QVBoxLayout(home_group)
        self.q_home_edit = QTextEdit()
        q_home = self.robot_config.get('q_home_deg', [])
        self.q_home_edit.setPlainText(', '.join([str(q) for q in q_home]))
        self.q_home_edit.setMaximumHeight(60)
        self.q_home_edit.setPlaceholderText("e.g., 0.0, -90.0, 90.0, -90.0, 0.0, 90.0, 0.0")
        home_layout.addWidget(self.q_home_edit)
        pose_layout.addWidget(home_group)
        
        # Default EE target
        ee_group = QGroupBox("Default End-Effector Target")
        ee_layout = QFormLayout(ee_group)
        
        # Position
        pos_row = QHBoxLayout()
        ee_target = self.robot_config.get('default_ee_target', {})
        pos = ee_target.get('position', [0.4, 0.0, 0.4])
        
        self.ee_pos_x = QDoubleSpinBox()
        self.ee_pos_x.setRange(-2.0, 2.0)
        self.ee_pos_x.setDecimals(4)
        self.ee_pos_x.setValue(pos[0])
        pos_row.addWidget(QLabel("X:"))
        pos_row.addWidget(self.ee_pos_x)
        
        self.ee_pos_y = QDoubleSpinBox()
        self.ee_pos_y.setRange(-2.0, 2.0)
        self.ee_pos_y.setDecimals(4)
        self.ee_pos_y.setValue(pos[1])
        pos_row.addWidget(QLabel("Y:"))
        pos_row.addWidget(self.ee_pos_y)
        
        self.ee_pos_z = QDoubleSpinBox()
        self.ee_pos_z.setRange(-2.0, 2.0)
        self.ee_pos_z.setDecimals(4)
        self.ee_pos_z.setValue(pos[2])
        pos_row.addWidget(QLabel("Z:"))
        pos_row.addWidget(self.ee_pos_z)
        pos_row.addStretch()
        ee_layout.addRow("Position (m):", pos_row)
        
        # Orientation (quaternion)
        orient_row = QHBoxLayout()
        orient = ee_target.get('orientation', [0.0, 0.0, 0.0, 1.0])
        
        self.ee_qx = QDoubleSpinBox()
        self.ee_qx.setRange(-1.0, 1.0)
        self.ee_qx.setDecimals(4)
        self.ee_qx.setValue(orient[0])
        orient_row.addWidget(QLabel("Qx:"))
        orient_row.addWidget(self.ee_qx)
        
        self.ee_qy = QDoubleSpinBox()
        self.ee_qy.setRange(-1.0, 1.0)
        self.ee_qy.setDecimals(4)
        self.ee_qy.setValue(orient[1])
        orient_row.addWidget(QLabel("Qy:"))
        orient_row.addWidget(self.ee_qy)
        
        self.ee_qz = QDoubleSpinBox()
        self.ee_qz.setRange(-1.0, 1.0)
        self.ee_qz.setDecimals(4)
        self.ee_qz.setValue(orient[2])
        orient_row.addWidget(QLabel("Qz:"))
        orient_row.addWidget(self.ee_qz)
        
        self.ee_qw = QDoubleSpinBox()
        self.ee_qw.setRange(-1.0, 1.0)
        self.ee_qw.setDecimals(4)
        self.ee_qw.setValue(orient[3])
        orient_row.addWidget(QLabel("Qw:"))
        orient_row.addWidget(self.ee_qw)
        orient_row.addStretch()
        ee_layout.addRow("Orientation (quat):", orient_row)
        
        pose_layout.addWidget(ee_group)
        pose_layout.addStretch()
        
        tabs.addTab(pose_tab, "Default Pose")
        
        # ── Joint Limits Tab ──
        limits_tab = QWidget()
        limits_layout = QVBoxLayout(limits_tab)
        
        limits_label = QLabel("Joint Limits (degrees) — format: lower, upper per line")
        limits_layout.addWidget(limits_label)
        
        self.joint_limits_edit = QTextEdit()
        limits = self.robot_config.get('joint_limits_deg', {})
        lower = limits.get('lower', [])
        upper = limits.get('upper', [])
        limits_text = ""
        for i in range(len(lower)):
            limits_text += f"{lower[i]}, {upper[i]}\n"
        self.joint_limits_edit.setPlainText(limits_text.strip())
        self.joint_limits_edit.setPlaceholderText("-180, 180\n-120, 120\n...")
        limits_layout.addWidget(self.joint_limits_edit)
        
        limits_note = QLabel("Enter one joint's limits per line: lower_limit, upper_limit\n"
                             "Number of lines should match number of joints.")
        theme.bind(limits_note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        limits_layout.addWidget(limits_note)
        
        # ── Joint Torque Limits ──
        torque_label = QLabel("Joint Torque Limits (Nm) — format: min, max per line\n"
                              "Hardware torque limits used by the autotuner as hard bounds.")
        theme.bind(
            torque_label,
            lambda t: "color: %s; font-size: 11px; margin-top: 12px;" % t["text_secondary"]
        )
        limits_layout.addWidget(torque_label)
        
        self.joint_torque_limits_edit = QTextEdit()
        torque_limits = self.robot_config.get('joint_torque_limits', {})
        torque_min = torque_limits.get('min', [])
        torque_max = torque_limits.get('max', [])
        torque_text = ""
        for i in range(max(len(torque_min), len(torque_max))):
            t_min = torque_min[i] if i < len(torque_min) else 0.0
            t_max = torque_max[i] if i < len(torque_max) else 50.0
            torque_text += f"{t_min}, {t_max}\n"
        self.joint_torque_limits_edit.setPlainText(torque_text.strip())
        self.joint_torque_limits_edit.setPlaceholderText("0, 39\n0, 39\n0, 9\n...")
        self.joint_torque_limits_edit.setMaximumHeight(120)
        limits_layout.addWidget(self.joint_torque_limits_edit)
        
        torque_note = QLabel("These are the physical actuator torque limits.\n"
                             "The autotuner will never set force_max beyond these values.")
        theme.bind(torque_note, lambda t: "color: %s; font-size: 10px;" % t["text_secondary"])
        limits_layout.addWidget(torque_note)
        
        tabs.addTab(limits_tab, "Joint Limits")
        
        # ── Tab 7: Safety Zones ──
        safety_tab = QWidget()
        safety_layout = QVBoxLayout(safety_tab)
        
        safety_desc = QLabel("Define safety zones to limit robot motion. These are enforced system-wide.")
        safety_desc.setWordWrap(True)
        theme.bind(safety_desc, lambda t: "color: %s; margin-bottom: 10px;" % t["text_secondary"])
        safety_layout.addWidget(safety_desc)
        
        # Load existing safety zones config
        safety_zones = self.robot_config.get('safety_zones', {})
        cart_safety = safety_zones.get('cartesian', {})
        joint_safety = safety_zones.get('joint_safety_deg', {})
        vel_safety = safety_zones.get('velocity', {})
        
        # Cartesian Safety Limits
        cart_grp = QGroupBox("Cartesian Safety Limits (meters)")
        cart_layout = QGridLayout(cart_grp)
        
        cart_layout.addWidget(QLabel(""), 0, 0)
        cart_layout.addWidget(QLabel("Min"), 0, 1)
        cart_layout.addWidget(QLabel("Max"), 0, 2)
        cart_layout.addWidget(QLabel("Enabled"), 0, 3)
        
        # X limits
        x_config = cart_safety.get('x', {})
        cart_layout.addWidget(QLabel("X:"), 1, 0)
        self.safety_x_min = QDoubleSpinBox()
        self.safety_x_min.setRange(-2.0, 2.0)
        self.safety_x_min.setDecimals(3)
        self.safety_x_min.setValue(x_config.get('min', -1.0))
        cart_layout.addWidget(self.safety_x_min, 1, 1)
        self.safety_x_max = QDoubleSpinBox()
        self.safety_x_max.setRange(-2.0, 2.0)
        self.safety_x_max.setDecimals(3)
        self.safety_x_max.setValue(x_config.get('max', 1.0))
        cart_layout.addWidget(self.safety_x_max, 1, 2)
        self.safety_x_enabled = QCheckBox()
        self.safety_x_enabled.setChecked(x_config.get('enabled', True))
        cart_layout.addWidget(self.safety_x_enabled, 1, 3)
        
        # Y limits
        y_config = cart_safety.get('y', {})
        cart_layout.addWidget(QLabel("Y:"), 2, 0)
        self.safety_y_min = QDoubleSpinBox()
        self.safety_y_min.setRange(-2.0, 2.0)
        self.safety_y_min.setDecimals(3)
        self.safety_y_min.setValue(y_config.get('min', -1.0))
        cart_layout.addWidget(self.safety_y_min, 2, 1)
        self.safety_y_max = QDoubleSpinBox()
        self.safety_y_max.setRange(-2.0, 2.0)
        self.safety_y_max.setDecimals(3)
        self.safety_y_max.setValue(y_config.get('max', 1.0))
        cart_layout.addWidget(self.safety_y_max, 2, 2)
        self.safety_y_enabled = QCheckBox()
        self.safety_y_enabled.setChecked(y_config.get('enabled', True))
        cart_layout.addWidget(self.safety_y_enabled, 2, 3)
        
        # Z limits
        z_config = cart_safety.get('z', {})
        cart_layout.addWidget(QLabel("Z:"), 3, 0)
        self.safety_z_min = QDoubleSpinBox()
        self.safety_z_min.setRange(-2.0, 2.0)
        self.safety_z_min.setDecimals(3)
        self.safety_z_min.setValue(z_config.get('min', 0.0))
        cart_layout.addWidget(self.safety_z_min, 3, 1)
        self.safety_z_max = QDoubleSpinBox()
        self.safety_z_max.setRange(-2.0, 2.0)
        self.safety_z_max.setDecimals(3)
        self.safety_z_max.setValue(z_config.get('max', 1.5))
        cart_layout.addWidget(self.safety_z_max, 3, 2)
        self.safety_z_enabled = QCheckBox()
        self.safety_z_enabled.setChecked(z_config.get('enabled', True))
        cart_layout.addWidget(self.safety_z_enabled, 3, 3)
        
        safety_layout.addWidget(cart_grp)
        
        # Joint Safety Limits (separate from mechanical limits)
        joint_safety_grp = QGroupBox("Joint Safety Limits (degrees)")
        joint_safety_layout = QVBoxLayout(joint_safety_grp)
        
        joint_safety_note = QLabel("Soft safety limits (within mechanical limits). Format: min, max per line.")
        theme.bind(
            joint_safety_note,
            lambda t: "color: %s; font-size: 10px;" % t["text_secondary"]
        )
        joint_safety_layout.addWidget(joint_safety_note)
        
        self.joint_safety_edit = QTextEdit()
        self.joint_safety_edit.setPlaceholderText("-160, 160\n-95, 95\n...")
        self.joint_safety_edit.setMaximumHeight(150)
        
        # Load existing joint safety limits
        joint_lower = joint_safety.get('lower', [])
        joint_upper = joint_safety.get('upper', [])
        if joint_lower and joint_upper:
            joint_safety_text = '\n'.join([f"{lo}, {hi}" for lo, hi in zip(joint_lower, joint_upper)])
            self.joint_safety_edit.setPlainText(joint_safety_text)
        
        joint_safety_layout.addWidget(self.joint_safety_edit)
        
        self.joint_safety_enabled = QCheckBox("Enable joint safety limits")
        self.joint_safety_enabled.setChecked(joint_safety.get('enabled', True))
        joint_safety_layout.addWidget(self.joint_safety_enabled)
        
        safety_layout.addWidget(joint_safety_grp)
        
        # Velocity limits
        vel_grp = QGroupBox("Velocity Limits")
        vel_layout = QGridLayout(vel_grp)
        
        vel_layout.addWidget(QLabel("Max Joint Velocity:"), 0, 0)
        self.safety_max_joint_vel = QDoubleSpinBox()
        self.safety_max_joint_vel.setRange(0.01, 5.0)
        self.safety_max_joint_vel.setDecimals(2)
        self.safety_max_joint_vel.setValue(vel_safety.get('max_joint_vel', 1.0))
        self.safety_max_joint_vel.setSuffix(" rad/s")
        vel_layout.addWidget(self.safety_max_joint_vel, 0, 1)
        
        vel_layout.addWidget(QLabel("Max Cartesian Velocity:"), 1, 0)
        self.safety_max_cart_vel = QDoubleSpinBox()
        self.safety_max_cart_vel.setRange(0.01, 2.0)
        self.safety_max_cart_vel.setDecimals(2)
        self.safety_max_cart_vel.setValue(vel_safety.get('max_cart_vel', 0.5))
        self.safety_max_cart_vel.setSuffix(" m/s")
        vel_layout.addWidget(self.safety_max_cart_vel, 1, 1)
        
        self.velocity_limits_enabled = QCheckBox("Enable velocity limits")
        self.velocity_limits_enabled.setChecked(vel_safety.get('enabled', True))
        vel_layout.addWidget(self.velocity_limits_enabled, 2, 0, 1, 2)
        
        safety_layout.addWidget(vel_grp)
        
        safety_layout.addStretch()
        tabs.addTab(safety_tab, "Safety Zones")
        
        layout.addWidget(tabs)
        
        # ── Buttons ──
        button_row = QHBoxLayout()
        button_row.addStretch()
        
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_row.addWidget(cancel_btn)
        
        save_btn = QPushButton("Save Robot")
        save_btn.setObjectName("BtnTeal")
        save_btn.clicked.connect(self._save_config)
        button_row.addWidget(save_btn)
        
        layout.addLayout(button_row)
    
    def _on_num_joints_changed(self, value):
        """Update joint-related fields when number of joints changes."""
        pass
    
    def _browse_file(self, line_edit: QLineEdit, file_filter: str):
        """Browse for a file and update the line edit."""
        filename, _ = QFileDialog.getOpenFileName(self, "Select File", self.ws_path, file_filter)
        if filename:
            # Convert to relative path if possible
            if "src/" in filename:
                filename = filename.split("src/", 1)[1]
            line_edit.setText(filename)
    
    def _browse_build_directory(self):
        """Browse for the build directory."""
        start_dir = self.build_dir_edit.text().strip()
        if not start_dir or '{' in start_dir:
            start_dir = _FIC_ROBOT_LIB

        directory = QFileDialog.getExistingDirectory(
            self, "Select Build Directory", start_dir
        )
        if directory:
            # Offer to use {robot_lib} placeholder if path is under robot_library
            robot_lib = _FIC_ROBOT_LIB
            if directory.startswith(robot_lib):
                relative = directory[len(robot_lib):]
                self.build_dir_edit.setText(f"{{robot_lib}}{relative}")
            else:
                self.build_dir_edit.setText(directory)
    
    def _save_config(self):
        """Validate and save the configuration."""
        import math
        
        # Validate robot ID
        robot_id = self.robot_id_edit.text().strip()
        if not robot_id:
            QMessageBox.warning(self, "Validation Error", "Robot ID is required.")
            return
        if ' ' in robot_id:
            QMessageBox.warning(self, "Validation Error", "Robot ID cannot contain spaces.")
            return
        
        # Parse joint names
        joint_names_text = self.joint_names_edit.toPlainText().strip()
        joint_names = [n.strip() for n in joint_names_text.split('\n') if n.strip()]
        
        # Parse q_home
        q_home_text = self.q_home_edit.toPlainText().strip()
        try:
            q_home_deg = [float(q.strip()) for q in q_home_text.split(',') if q.strip()]
            q_home_rad = [math.radians(q) for q in q_home_deg]
        except ValueError:
            QMessageBox.warning(self, "Validation Error", "Invalid home angles format. Use comma-separated numbers.")
            return
        
        # Parse joint limits
        limits_text = self.joint_limits_edit.toPlainText().strip()
        lower_limits = []
        upper_limits = []
        for line in limits_text.split('\n'):
            if line.strip():
                try:
                    parts = line.split(',')
                    lower_limits.append(float(parts[0].strip()))
                    upper_limits.append(float(parts[1].strip()))
                except (ValueError, IndexError):
                    QMessageBox.warning(self, "Validation Error", "Invalid joint limits format.")
                    return
        
        # Parse joint safety limits
        safety_limits_text = self.joint_safety_edit.toPlainText().strip()
        joint_safety_lower = []
        joint_safety_upper = []
        if safety_limits_text:
            for line in safety_limits_text.split('\n'):
                if line.strip():
                    try:
                        parts = line.split(',')
                        joint_safety_lower.append(float(parts[0].strip()))
                        joint_safety_upper.append(float(parts[1].strip()))
                    except (ValueError, IndexError):
                        QMessageBox.warning(self, "Validation Error", "Invalid joint safety limits format.")
                        return
        
        # Parse build command — join multi-line into single command with &&
        build_cmd_raw = self.build_cmd_edit.toPlainText().strip()
        build_cmd_lines = [l.strip() for l in build_cmd_raw.split('\n') if l.strip()]
        build_command = ' && '.join(build_cmd_lines)
        
        # Parse joint torque limits
        torque_min_list = []
        torque_max_list = []
        torque_text = self.joint_torque_limits_edit.toPlainText().strip()
        if torque_text:
            for line in torque_text.split('\n'):
                if line.strip():
                    try:
                        parts = line.split(',')
                        torque_min_list.append(float(parts[0].strip()))
                        torque_max_list.append(float(parts[1].strip()) if len(parts) > 1 else 50.0)
                    except (ValueError, IndexError):
                        QMessageBox.warning(self, "Validation Error", "Invalid joint torque limits format.")
                        return
        
        # Build config
        self.result_name = robot_id
        self.result_config = {
            'display_name': self.display_name_edit.text().strip() or robot_id,
            'num_joints': self.num_joints_spin.value(),
            'joint_names': joint_names,
            'urdf_path': self.urdf_path_edit.text().strip(),
            'mujoco_xml_path': self.mujoco_path_edit.text().strip(),
            'ee_frame_name': self.ee_frame_edit.text().strip(),
            'elbow_frame_name': self.elbow_frame_edit.text().strip(),
            'kinematics_solver': self.kin_solver_combo.currentText().strip() or 'pinocchio',
            'default_ee_target': {
                'position': [self.ee_pos_x.value(), self.ee_pos_y.value(), self.ee_pos_z.value()],
                'orientation': [self.ee_qx.value(), self.ee_qy.value(), self.ee_qz.value(), self.ee_qw.value()]
            },
            'q_home_deg': q_home_deg,
            'q_home_rad': q_home_rad,
            'joint_limits_deg': {
                'lower': lower_limits,
                'upper': upper_limits
            },
            'joint_torque_limits': {
                'min': torque_min_list,
                'max': torque_max_list
            },
            'torque_sign': 1 if self.torque_sign_combo.currentIndex() == 0 else -1,
            'gravity_compensated': self.gravity_compensated_check.isChecked(),
            'wrench_gravity_compensated': self.wrench_gravity_compensated_check.isChecked(),
            'wrench_from_api': self.wrench_from_api_check.isChecked(),
            'admittance_deadbands': {
                'joint_tau': self.db_joint_tau.value(),
                'cart_force': self.db_cart_force.value(),
                'cart_torque': self.db_cart_torque.value(),
            },
            'has_gripper': self.has_gripper_check.isChecked(),
            'gripper_range': [self.gripper_min.value(), self.gripper_max.value()],
            'is_supported': self.is_supported_check.isChecked(),
            'driver_command': self.driver_cmd_edit.text().strip(),
            'build_directory': self.build_dir_edit.text().strip(),
            'build_command': build_command,
            # Safety zones
            'safety_zones': {
                'cartesian': {
                    'x': {'min': self.safety_x_min.value(), 'max': self.safety_x_max.value(), 'enabled': self.safety_x_enabled.isChecked()},
                    'y': {'min': self.safety_y_min.value(), 'max': self.safety_y_max.value(), 'enabled': self.safety_y_enabled.isChecked()},
                    'z': {'min': self.safety_z_min.value(), 'max': self.safety_z_max.value(), 'enabled': self.safety_z_enabled.isChecked()}
                },
                'joint_safety_deg': {
                    'lower': joint_safety_lower,
                    'upper': joint_safety_upper,
                    'enabled': self.joint_safety_enabled.isChecked()
                },
                'velocity': {
                    'max_joint_vel': self.safety_max_joint_vel.value(),
                    'max_cart_vel': self.safety_max_cart_vel.value(),
                    'enabled': self.velocity_limits_enabled.isChecked()
                }
            }
        }
        
        self.accept()


# ═══════════════════════════════════════════════════════════════════════════════
#  ROBOT CONFIG MIXIN
# ═══════════════════════════════════════════════════════════════════════════════

class RobotConfigMixin:
    """Mixin for robot configuration management."""

    # Resolve the robots YAML in the SOURCE tree via config.py. Under a macOS/
    # RoboStack copy-install, __file__ points into fic_ws/install where the
    # YAML is not shipped (setup.py installs only .py files), so the package-
    # relative path silently falls back to DEFAULT_ROBOT_CONFIGS and newly
    # added robots vanish. A symlink-install (Ubuntu) hides the problem
    # because __file__ resolves back into the source tree. Reading and
    # writing the source YAML also keeps one file authoritative across
    # rebuilds instead of an install-dir copy that gets clobbered.
    try:
        from fic_studio.config import WS_PATH as _WS_PATH
        _SRC_YAML = os.path.join(
            _WS_PATH, "src", "fic_studio", "fic_studio", "fic_studio_robots.yaml")
    except Exception:
        _SRC_YAML = None
    ROBOT_CONFIG_FILE = (
        _SRC_YAML if _SRC_YAML and os.path.exists(_SRC_YAML)
        else os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "fic_studio_robots.yaml"))
    
    def _init_robot_config(self):
        """Initialize robot configuration system."""
        self.robot_configs = self._load_robot_configs()
        # Update self.robots dict for compatibility with other mixins
        self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
        # Note: _refresh_robots_dropdown() is called after init_ui() in main_window
    
    def _load_robot_configs(self) -> dict:
        """Load robot configurations from file."""
        if os.path.exists(self.ROBOT_CONFIG_FILE):
            try:
                with open(self.ROBOT_CONFIG_FILE, 'r') as f:
                    data = yaml.safe_load(f)
                    return data.get('robots', {})
            except Exception as e:
                print(f"Could not load robot configs: {e}")
        
        # Return default configs
        return DEFAULT_ROBOT_CONFIGS.copy()
    
    def _save_robot_configs(self):
        """Save robot configurations to file. Returns False if that failed."""
        try:
            data = {
                'version': '1.0',
                'robots': self.robot_configs
            }
            write_yaml_atomically(self.ROBOT_CONFIG_FILE, data)
            self.log_message("INFO", "Robot configurations saved", "Config")
            return True
        except Exception as e:
            self.log_message("ERROR", f"Could not save robot configs: {e}", "Config")
            QMessageBox.critical(self, 'Save failed',
                                 'Could not write %s:\n%s' % (self.ROBOT_CONFIG_FILE, e))
            return False
    
    def _refresh_robots_dropdown(self):
        """Refresh the robot dropdown with current configs."""
        if hasattr(self, 'robot_dropdown'):
            current = self.robot_dropdown.currentText()
            self.robot_dropdown.clear()
            for robot_id in self.robot_configs.keys():
                self.robot_dropdown.addItem(robot_id)
            
            # Restore selection if possible
            idx = self.robot_dropdown.findText(current)
            if idx >= 0:
                self.robot_dropdown.setCurrentIndex(idx)
            
            # Update self.robots dict for compatibility
            self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}

            # The config page shows the toolbar's robot unless it is mid-edit.
            if hasattr(self, '_rc_robot_list') and not getattr(self, '_rc_follows_toolbar', False):
                self._rc_follows_toolbar = True
                self.robot_dropdown.currentTextChanged.connect(self._rc_follow_toolbar)
                self._rc_follow_toolbar(self.robot_dropdown.currentText())

        self._refresh_other_robot_lists()

    def _rc_refresh_robot_lists(self):
        """Show the current set of robots in every robot list, selecting nothing.

        ``_refresh_robots_dropdown`` rebuilds the toolbar's selector by
        clearing and re-adding it, which walks it through the first robot and
        back: every slot on it runs twice, resetting the control panel's
        typed pose, re-publishing the tuning profile to a running controller
        and stopping a running locomotion controller. Fine once at start-up;
        not on every Save. Here its signals stay blocked, and fire only when
        the selected robot itself was deleted.
        """
        self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
        dropdown = getattr(self, 'robot_dropdown', None)
        if dropdown is not None:
            ids = list(self.robot_configs)
            if [dropdown.itemText(i) for i in range(dropdown.count())] != ids:
                current = dropdown.currentText()
                dropdown.blockSignals(True)
                dropdown.clear()
                dropdown.addItems(ids)
                idx = dropdown.findText(current)
                if idx >= 0:
                    dropdown.setCurrentIndex(idx)
                dropdown.blockSignals(False)
                if idx < 0 and ids:
                    dropdown.currentTextChanged.emit(dropdown.currentText())
        self._refresh_other_robot_lists()

    def _refresh_other_robot_lists(self):
        """Refresh the robot lists besides the toolbar's, keeping their selections."""
        # The extra robot cards (Robot 2-4) each have a Type list, filled
        # when the card was made; a robot added since must appear there too.
        fallback = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        for card in getattr(self, '_card_robot_types', {}):
            combo = getattr(self, '%s_type_dropdown' % card, None)
            if combo is None:
                continue
            kept = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(list(self.robot_configs))
            combo.setCurrentText(kept if kept in self.robot_configs else fallback)
            combo.blockSignals(False)

        # Robot -> Select Robot menu, built once at start-up from the same list.
        menu = getattr(self, '_robot_select_menu', None)
        group = getattr(self, 'robot_action_group', None)
        if menu is not None and group is not None and \
                [a.text() for a in group.actions()] != list(self.robot_configs):
            from PyQt5.QtWidgets import QAction
            for action in group.actions():
                group.removeAction(action)
                menu.removeAction(action)
                action.deleteLater()
            for robot_name in self.robot_configs:
                action = QAction(robot_name, self)
                action.setCheckable(True)
                action.triggered.connect(
                    lambda checked, r=robot_name: self._select_robot_from_menu(r))
                menu.addAction(action)
                group.addAction(action)
            if hasattr(self, '_sync_robot_menu_check'):
                self._sync_robot_menu_check()

        # Also refresh the settings default robot dropdown
        if hasattr(self, 'settings_default_robot'):
            current_setting = self.settings_default_robot.currentText()
            self.settings_default_robot.blockSignals(True)
            self.settings_default_robot.clear()
            self.settings_default_robot.addItems(list(self.robot_configs.keys()))
            idx = self.settings_default_robot.findText(current_setting)
            if idx >= 0:
                self.settings_default_robot.setCurrentIndex(idx)
            self.settings_default_robot.blockSignals(False)

    def get_robot_config(self, robot_name: str) -> dict:
        """Get configuration for a specific robot."""
        return self.robot_configs.get(robot_name, {})
    
    # -------------------------------------------------------------------------
    # Inline card helpers
    # -------------------------------------------------------------------------

    def _rc_warmth_card(self, title=None):
        """
        Create a Warmth styled card QFrame.

        Returns (QFrame, QVBoxLayout inside it).  Uses theme tokens so both
        light and dark themes are supported without any inline hex colours.
        """
        frame = QFrame()
        # Styled by name: a bare ``QFrame { ... }`` rule also matches every
        # QLabel inside (QLabel is a QFrame), which boxed each label in a
        # border of its own.
        frame.setObjectName('RcCard')
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)

        def _style(t):
            return (
                'QFrame#RcCard { background: %s; border: 1px solid %s;'
                ' border-radius: 10px; }' % (t['bg_raised'], t['border_subtle'])
            )

        frame.setStyleSheet(_style(theme.current()))
        theme.bind(frame, _style)

        if title:
            hdr = QLabel(title.upper())
            theme.bind(
                hdr,
                lambda t: (
                    'color: %s; font-size: 10px; font-weight: 700;'
                    % t['text_disabled']
                ),
            )
            layout.addWidget(hdr)

        return frame, layout

    def _rc_form_row(self, label_text, widget):
        """
        Return a QHBoxLayout with a fixed-width label and an expanding widget.

        The label uses ``text_secondary`` token; no inline colours.
        """
        row = QHBoxLayout()
        row.setSpacing(10)
        lbl = QLabel(label_text)
        lbl.setFixedWidth(180)
        theme.bind(lbl, lambda t: 'color: %s; font-size: 12px;' % t['text_secondary'])
        row.addWidget(lbl)
        if widget.maximumWidth() < 10000:     # a narrow field: keep it by its label
            row.addWidget(widget)
            row.addStretch(1)
        else:
            row.addWidget(widget, 1)
        return row

    # -------------------------------------------------------------------------
    # Page builder
    # -------------------------------------------------------------------------

    def _build_robot_config_page(self) -> QWidget:
        """
        Build the Robot Configuration page.

        Left: every robot, with a drop zone that adds one from its URDF.
        Right: the selected robot -- a header with its status and the one
        Save (which writes both robot YAMLs) over five tabs: Overview,
        Joints, Frames, Hardware and Safety.
        """
        from fic_studio.components.robot_drop_zone import RobotDropZone

        self._rc_dirty = False
        self._rc_out_of_sync = False
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(22, 20, 22, 20)
        outer.setSpacing(12)
        outer.addWidget(self._page_title('Robot Configuration'))

        body = QHBoxLayout()
        body.setSpacing(16)
        outer.addLayout(body, 1)

        # ── Left: the robots ───────────────────────────────────────────────
        left = QWidget()
        left.setFixedWidth(264)
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(8)
        lv.addWidget(self._rc_caption('Robots'))
        self._rc_robot_list = QListWidget()
        self._rc_robot_list.setSelectionMode(QListWidget.SingleSelection)
        self._rc_robot_list.setVerticalScrollMode(QListWidget.ScrollPerPixel)
        theme.bind(self._rc_robot_list, lambda t: (
            'QListWidget { background: %s; border: 1px solid %s; border-radius: 10px;'
            ' padding: 4px; outline: none; }'
            'QListWidget::item { border-radius: 6px; margin: 1px 0; }'
            'QListWidget::item:selected { background: %s; }'
            'QListWidget::item:hover:!selected { background: %s; }'
        ) % (t['bg_raised'], t['border_subtle'], t['bg_selected'], t['bg_hover']))
        self._rc_robot_list.currentRowChanged.connect(self._rc_on_list_changed)
        lv.addWidget(self._rc_robot_list, 1)

        self._rc_drop_zone = RobotDropZone()
        self._rc_drop_zone.dropped.connect(self._rc_start_import)
        self._rc_drop_zone.dropped_many.connect(self._rc_start_import)
        lv.addWidget(self._rc_drop_zone)
        blank_btn = self._btn('+ Blank robot')
        blank_btn.setToolTip('Add a robot by typing its settings in, without a URDF')
        blank_btn.clicked.connect(self._rc_add_robot)
        lv.addWidget(blank_btn)
        body.addWidget(left)

        # ── Right: the selected robot ──────────────────────────────────────
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(10)

        head = QHBoxLayout()
        head.setSpacing(8)
        names = QVBoxLayout()
        names.setSpacing(3)
        self._rc_title_lbl = QLabel('')
        theme.bind(self._rc_title_lbl, lambda t: (
            'color: %s; font-size: 18px; font-weight: 700;' % t['text_primary']))
        names.addWidget(self._rc_title_lbl)
        sub = QHBoxLayout()
        sub.setSpacing(6)
        self._rc_subtitle_lbl = QLabel('')
        theme.bind(self._rc_subtitle_lbl, lambda t: (
            'color: %s; font-size: 12px;' % t['text_secondary']))
        sub.addWidget(self._rc_subtitle_lbl)
        self._rc_badge_sim = self._rc_pill()
        self._rc_badge_hw = self._rc_pill()
        sub.addWidget(self._rc_badge_sim)
        sub.addWidget(self._rc_badge_hw)
        sub.addStretch()
        names.addLayout(sub)
        head.addLayout(names, 1)

        self._rc_sync_indicator = QLabel('')
        self._rc_sync_indicator.setToolTip(
            "Save writes both the Studio's robot list (fic_studio_robots.yaml) and "
            "the controllers' copy (fic_library/config/robots_config.yaml).")
        theme.bind(self._rc_sync_indicator, self._rc_indicator_style)
        head.addWidget(self._rc_sync_indicator)

        self._rc_save_btn = self._btn('Save', 'BtnTeal')
        self._rc_save_btn.setToolTip(
            "Save this robot. Updates the Studio's robot list and the controllers' "
            'robots_config.yaml together; running controllers pick up the '
            'gravity and wrench switches straight away.')
        self._rc_save_btn.clicked.connect(self._rc_save)
        head.addWidget(self._rc_save_btn)

        more = QToolButton()
        more.setText('⋯')
        more.setToolTip('More')
        more.setCursor(Qt.PointingHandCursor)
        more.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(more)
        menu.addAction('Duplicate', self._rc_duplicate_robot)
        menu.addAction('Delete…', self._rc_delete_robot)
        menu.addSeparator()
        menu.addAction('Import robots from YAML…', self._import_robot_config)
        menu.addAction('Export all robots to YAML…', self._export_robot_config)
        menu.addAction("View the controllers' YAML", self._open_robots_yaml_viewer)
        menu.addAction('Open model folder', self._rc_open_folder)
        more.setMenu(menu)
        theme.bind(more, lambda t: (
            'QToolButton { color: %s; background: %s; border: 1px solid %s;'
            ' border-radius: 6px; padding: 4px 12px; font-size: 14px; font-weight: 700; }'
            'QToolButton:hover { background: %s; }'
            'QToolButton::menu-indicator { image: none; }'
        ) % (t['text_primary'], t['bg_raised'], t['border_subtle'], t['bg_hover']))
        head.addWidget(more)
        rv.addLayout(head)

        self._rc_tabs = QTabWidget()
        self._rc_tabs.setDocumentMode(True)
        for title, build in (('Overview', self._rc_build_overview_tab),
                             ('Joints', self._rc_build_joints_tab),
                             ('Frames', self._rc_build_frames_tab),
                             ('Hardware', self._rc_build_hardware_tab),
                             ('Safety', self._rc_build_safety_tab)):
            self._rc_tabs.addTab(self._rc_scroll(build()), title)
        rv.addWidget(self._rc_tabs, 1)
        body.addWidget(right, 1)

        self._rc_connect_dirty()
        self._rc_rebuild_list()

        # Start on the robot the toolbar has selected (it follows later too;
        # see _rc_follow_toolbar).
        if self.robot_configs:
            current = getattr(self, 'robot_dropdown', None)
            initial_id = (
                current.currentText()
                if current and current.currentText() in self.robot_configs
                else next(iter(self.robot_configs))
            )
            self._rc_load_robot(initial_id)
        self._rc_check_sync_state()
        return page

    # -------------------------------------------------------------------------
    # Page pieces
    # -------------------------------------------------------------------------

    def _rc_caption(self, text):
        lbl = QLabel(text.upper())
        theme.bind(lbl, lambda t: (
            'color: %s; font-size: 10px; font-weight: 700;' % t['text_disabled']))
        return lbl

    def _rc_hint(self, text):
        lbl = QLabel(text)
        lbl.setWordWrap(True)
        theme.bind(lbl, lambda t: 'color: %s; font-size: 11px;' % t['text_secondary'])
        return lbl

    def _rc_pill(self):
        lbl = QLabel('')
        lbl.setProperty('tok', 'text_disabled')

        def style(t, lbl=lbl):
            tok = lbl.property('tok') or 'text_disabled'
            return ('color: %s; border: 1px solid %s; border-radius: 9px;'
                    ' padding: 1px 8px; font-size: 11px; font-weight: 600;'
                    % (t[tok], t[tok]))
        lbl._rc_style = style
        theme.bind(lbl, style)
        return lbl

    def _rc_set_pill(self, lbl, text, tok):
        lbl.setText(text)
        lbl.setProperty('tok', tok)
        lbl.setStyleSheet(lbl._rc_style(theme.current()))

    def _rc_scroll(self, widget):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(widget)
        return scroll

    def _rc_tab_page(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(2, 14, 8, 14)
        lay.setSpacing(12)
        lay.setAlignment(Qt.AlignTop)
        return w, lay

    def _rc_path_row(self, edit, file_filter):
        """``edit`` with a Browse button that fills in a path under fic_ws/src."""
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(6)
        h.addWidget(edit, 1)
        btn = self._btn('Browse…')
        btn.clicked.connect(lambda: self._rc_browse_path(edit, file_filter))
        h.addWidget(btn)
        return row

    def _rc_browse_path(self, edit, file_filter):
        ws_src = os.path.join(_FIC_WS, 'src')
        start = rc_check.resolve(ws_src, edit.text().strip()) or ws_src
        path, _ = QFileDialog.getOpenFileName(
            self, 'Choose file', os.path.dirname(start) if os.path.isfile(start) else ws_src,
            file_filter + ';;All files (*)')
        if path:
            rel = os.path.relpath(path, ws_src)
            edit.setText(path if rel.startswith('..') else rel)

    def _rc_frame_row(self, edit):
        """``edit`` with a ▾ button listing the URDF's links."""
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(6)
        h.addWidget(edit, 1)
        pick = QToolButton()
        pick.setText('▾')
        pick.setToolTip('Pick a link from the URDF')
        pick.setCursor(Qt.PointingHandCursor)
        pick.setPopupMode(QToolButton.InstantPopup)
        theme.bind(pick, lambda t: (
            'QToolButton { color: %s; background: %s; border: 1px solid %s;'
            ' border-radius: 5px; padding: 3px 9px; }'
            'QToolButton:hover { background: %s; }'
            'QToolButton::menu-indicator { image: none; }'
        ) % (t['text_primary'], t['bg_raised'], t['border_subtle'], t['bg_hover']))
        menu = QMenu(pick)
        menu.aboutToShow.connect(lambda: self._rc_fill_link_menu(menu, edit))
        pick.setMenu(menu)
        h.addWidget(pick)
        return row

    def _rc_urdf_links(self):
        """Link names of the URDF in the Files card, in file order ([] if unreadable)."""
        path = rc_check.resolve(os.path.join(_FIC_WS, 'src'), self._rc_urdf_path.text().strip())
        if not path or not os.path.isfile(path):
            return []
        try:
            root = ET.parse(path).getroot()
        except (ET.ParseError, OSError):
            return []
        return [link.get('name') for link in root.findall('link') if link.get('name')]

    def _rc_fill_link_menu(self, menu, edit):
        menu.clear()
        links = self._rc_urdf_links()
        if not links:
            menu.addAction('No readable URDF set (Overview → Files)').setEnabled(False)
            return
        for name in links:
            menu.addAction(name, lambda n=name: edit.setText(n))

    # ── Tabs ──────────────────────────────────────────────────────────────

    def _rc_build_overview_tab(self):
        w, lay = self._rc_tab_page()

        card, cl = self._rc_warmth_card('Ready to simulate?')
        self._rc_status_holder = QVBoxLayout()
        self._rc_status_holder.setSpacing(4)
        cl.addLayout(self._rc_status_holder)
        actions = QHBoxLayout()
        actions.setSpacing(6)
        self._rc_try_btn = self._btn('▶  Try in simulation', 'BtnTeal')
        self._rc_try_btn.setToolTip('Start the MuJoCo simulator and the FIC controller '
                                    'for this robot')
        self._rc_try_btn.clicked.connect(self._rc_try_in_sim)
        self._rc_viewer_btn = self._btn('Open in MuJoCo viewer')
        self._rc_viewer_btn.setToolTip('Look at the model on its own: no controller, '
                                       'nothing else started')
        self._rc_viewer_btn.clicked.connect(self._rc_open_viewer)
        self._rc_report_btn = self._btn('Import report')
        self._rc_report_btn.setToolTip('What the importer found and did for this robot')
        self._rc_report_btn.clicked.connect(self._rc_show_report)
        for b in (self._rc_try_btn, self._rc_viewer_btn, self._rc_report_btn):
            actions.addWidget(b)
        actions.addStretch()
        cl.addLayout(actions)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Identity')
        self._rc_display_name = QLineEdit()
        self._rc_display_name.setPlaceholderText('e.g., My Robot 7DOF')
        cl.addLayout(self._rc_form_row('Display name', self._rc_display_name))
        self._rc_robot_id_lbl = QLabel('')
        self._rc_robot_id_lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        theme.bind(self._rc_robot_id_lbl, lambda t: (
            'color: %s; font-size: 12px; font-family: monospace;' % t['text_primary']))
        cl.addLayout(self._rc_form_row('Robot ID', self._rc_robot_id_lbl))
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Files')
        cl.addWidget(self._rc_hint(
            'Paths are relative to fic_ws/src. The controllers read the URDF; the '
            'simulator loads the MuJoCo scene. A robot added from a URDF has both in '
            'menagerie/model/<Robot ID>/.'))
        self._rc_urdf_path = QLineEdit()
        self._rc_urdf_path.setPlaceholderText('menagerie/model/Robot/URDF/Robot.urdf')
        self._rc_mujoco_path = QLineEdit()
        self._rc_mujoco_path.setPlaceholderText('menagerie/model/Robot/xml/scene.xml')
        self._rc_urdf_path.textChanged.connect(
            lambda _: self._rc_update_path_indicator(self._rc_urdf_path))
        self._rc_mujoco_path.textChanged.connect(
            lambda _: self._rc_update_path_indicator(self._rc_mujoco_path))
        cl.addLayout(self._rc_form_row(
            'URDF', self._rc_path_row(self._rc_urdf_path, 'URDF (*.urdf)')))
        cl.addLayout(self._rc_form_row(
            'MuJoCo scene', self._rc_path_row(self._rc_mujoco_path, 'MuJoCo XML (*.xml)')))
        lay.addWidget(card)
        return w

    def _rc_build_joints_tab(self):
        w, lay = self._rc_tab_page()
        card, cl = self._rc_warmth_card('Joints')
        cl.addWidget(self._rc_hint(
            "One row per driven joint, in order: the simulator's motor torq_jk drives "
            'row k. Names must match the URDF and the MuJoCo model. Limits are in '
            "degrees; torque is the actuator's maximum."))
        self._rc_num_joints = QSpinBox()
        self._rc_num_joints.setRange(1, 12)
        self._rc_num_joints.setMaximumWidth(120)
        self._rc_num_joints.valueChanged.connect(self._rc_on_num_joints_changed)
        cl.addLayout(self._rc_form_row('Number of joints', self._rc_num_joints))
        self._rc_joint_table = JointTable(unit_deg='deg', unit_tau='Nm')
        cl.addWidget(self._rc_joint_table)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Home pose')
        cl.addWidget(self._rc_hint('Where the robot starts and returns to, in degrees.'))
        self._rc_home_container = QWidget()
        self._rc_home_layout = QGridLayout(self._rc_home_container)
        self._rc_home_layout.setContentsMargins(0, 0, 0, 0)
        self._rc_home_layout.setSpacing(6)
        self._rc_home_fields = []   # list[MetricField]
        cl.addWidget(self._rc_home_container)
        lay.addWidget(card)
        return w

    def _rc_build_frames_tab(self):
        w, lay = self._rc_tab_page()
        card, cl = self._rc_warmth_card('Frames')
        cl.addWidget(self._rc_hint(
            'Frames are link names from the URDF. The end effector is what Cartesian '
            "control moves; the elbow steers the arm's redundancy (leave it empty if "
            'unsure).'))
        self._rc_ee_frame = QLineEdit()
        self._rc_ee_frame.setPlaceholderText('e.g., ee_link, tool0')
        self._rc_elbow_frame = QLineEdit()
        self._rc_elbow_frame.setPlaceholderText('e.g., link4, forearm_link')
        # Kinematics solver backend: pinocchio (default), none (joint-space
        # only), or a custom name registered in fic/solvers/solver_factory.hpp
        self._rc_kin_solver = QComboBox()
        self._rc_kin_solver.setEditable(True)
        self._rc_kin_solver.addItems(['pinocchio', 'none'])
        self._rc_kin_solver.setToolTip(
            'Kinematics/dynamics solver for this robot:\n'
            '  pinocchio — built-in solver (default)\n'
            '  none — joint-space control only (Cartesian modes disabled)\n'
            '  <custom> — a solver registered in fic/solvers/solver_factory.hpp\n'
            'See docs/custom-kinematics-solver.md')
        cl.addLayout(self._rc_form_row('End effector', self._rc_frame_row(self._rc_ee_frame)))
        cl.addLayout(self._rc_form_row('Elbow', self._rc_frame_row(self._rc_elbow_frame)))
        cl.addLayout(self._rc_form_row('Kinematics solver', self._rc_kin_solver))
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Default Cartesian target')
        cl.addWidget(self._rc_hint(
            'Where Cartesian control aims when it starts, in the base frame. The end '
            "effector's pose at the home pose is a safe choice."))
        pos_row = QHBoxLayout()
        pos_row.setSpacing(6)
        self._rc_ee_pos_x = MetricField('X', 0.0, -2.0, 2.0, unit='m', decimals=4)
        self._rc_ee_pos_y = MetricField('Y', 0.0, -2.0, 2.0, unit='m', decimals=4)
        self._rc_ee_pos_z = MetricField('Z', 0.0, -2.0, 2.0, unit='m', decimals=4)
        for mf in (self._rc_ee_pos_x, self._rc_ee_pos_y, self._rc_ee_pos_z):
            pos_row.addWidget(mf, 1)
        cl.addLayout(pos_row)
        quat_row = QHBoxLayout()
        quat_row.setSpacing(6)
        self._rc_ee_qx = QDoubleSpinBox()
        self._rc_ee_qy = QDoubleSpinBox()
        self._rc_ee_qz = QDoubleSpinBox()
        self._rc_ee_qw = QDoubleSpinBox()
        self._rc_qnorm_lbl = QLabel('|q| = 0.0000')
        theme.bind(self._rc_qnorm_lbl, lambda t: 'color: %s; font-size: 11px;'
                   % t['text_secondary'])
        for sb, name in ((self._rc_ee_qx, 'qx'), (self._rc_ee_qy, 'qy'),
                         (self._rc_ee_qz, 'qz'), (self._rc_ee_qw, 'qw')):
            sb.setRange(-1.0, 1.0)
            sb.setDecimals(4)
            sb.setSingleStep(0.0001)
            sb.valueChanged.connect(self._rc_update_qnorm)
            quat_row.addWidget(self._rc_sec_lbl(name))
            quat_row.addWidget(sb, 1)
        quat_row.addWidget(self._rc_qnorm_lbl)
        cl.addWidget(self._rc_hint('Orientation as a quaternion [x, y, z, w].'))
        cl.addLayout(quat_row)
        from_home = QHBoxLayout()
        self._rc_from_home_btn = self._btn('Set from home pose')
        self._rc_from_home_btn.setToolTip(
            "Compute the end effector's pose at the home pose (Joints tab) from the "
            'URDF, the way the controllers do')
        self._rc_from_home_btn.clicked.connect(self._rc_target_from_home)
        from_home.addWidget(self._rc_from_home_btn)
        from_home.addStretch()
        cl.addLayout(from_home)
        lay.addWidget(card)
        return w

    def _rc_build_hardware_tab(self):
        from fic_studio.components.robot_drop_zone import LibraryDropZone
        w, lay = self._rc_tab_page()
        card, cl = self._rc_warmth_card('Real robot')
        self._rc_lib_zone = LibraryDropZone()
        self._rc_lib_zone.setToolTip(
            "The vendor's SDK with FIC's driver program in it (it talks to the robot "
            "and to FIC's shared memory). Copied into robot_library, built, and set as "
            "this robot's driver.")
        self._rc_lib_zone.dropped.connect(self._rc_start_library)
        cl.addWidget(self._rc_lib_zone)
        self._rc_is_supported = QCheckBox('This robot has a hardware driver')
        self._rc_is_supported.setToolTip('Off: the robot is offered for simulation only')
        cl.addWidget(self._rc_is_supported)
        self._rc_driver_cmd = QLineEdit()
        self._rc_driver_cmd.setPlaceholderText(
            "Command that starts the driver; {robot_lib} is fic_library's robot folder")
        self._rc_build_dir = QLineEdit()
        self._rc_build_dir.setPlaceholderText('{robot_lib}/libfranka/build')
        self._rc_build_cmd = QLineEdit()
        self._rc_build_cmd.setPlaceholderText(
            'cmake .. && make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)')
        for label, widget in (('Driver command', self._rc_driver_cmd),
                              ('Build directory', self._rc_build_dir),
                              ('Build command', self._rc_build_cmd)):
            cl.addLayout(self._rc_form_row(label, widget))
        row = QHBoxLayout()
        compile_btn = self._btn('Compile driver', 'BtnWarn')
        compile_btn.setToolTip('Run the build command in the build directory')
        compile_btn.clicked.connect(self._rc_compile_driver)
        row.addWidget(compile_btn)
        row.addStretch()
        cl.addLayout(row)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Inner-loop MCU')
        cl.addWidget(self._rc_hint(
            'With an adapter, this robot\'s joint layer (joint impedance / admittance, '
            'gravity, torque limits, watchdog) runs at 1 kHz on the FIC MCU whenever one '
            'answers — on Ethernet, or on this board\'s own Bridge — and the vendor '
            'driver is not started. Without one, or with no MCU found, the standard '
            'path is used. Decided again at every Start.'))
        self._rc_mcu_adapter = QComboBox()
        for a in MCU_ADAPTERS:
            self._rc_mcu_adapter.addItem(a or 'none (standard path only)', a)
        self._rc_mcu_adapter.setMaximumWidth(260)
        self._rc_mcu_adapter.setToolTip(
            'How the MCU talks to the arm. kortex: Kinova Gen3 (Ethernet shield); '
            'loopback: no arm, for tests. The MCU reports which ones its firmware has.')
        self._rc_mcu_arm_ip = QLineEdit()
        self._rc_mcu_arm_ip.setPlaceholderText('192.168.1.10')
        self._rc_mcu_setup = QLineEdit()
        self._rc_mcu_setup.setPlaceholderText(
            'Optional, run on Linux at Start before the MCU takes the arm (clear faults, home ...)')
        self._rc_mcu_hold = QDoubleSpinBox()
        self._rc_mcu_hold.setRange(0.01, 1.0)
        self._rc_mcu_hold.setDecimals(3)
        self._rc_mcu_hold.setSuffix(' s')
        self._rc_mcu_hold.setToolTip('No command from Linux for this long: the MCU holds the last pose.')
        self._rc_mcu_ramp = QDoubleSpinBox()
        self._rc_mcu_ramp.setRange(0.1, 30.0)
        self._rc_mcu_ramp.setDecimals(2)
        self._rc_mcu_ramp.setSuffix(' s')
        self._rc_mcu_ramp.setToolTip('No command for this long: ramp to gravity, then leave torque mode.')
        self._rc_mcu_routing = QComboBox()
        self._rc_mcu_routing.addItems(['haptic', 'direct'])
        self._rc_mcu_routing.setMaximumWidth(160)
        self._rc_mcu_routing.setToolTip(
            'haptic: Cartesian modes send tau_des = J^T w to the MCU\'s joint admittance '
            'haptic layer (as hpmp does). direct: as a feed-forward torque.')
        for label, widget in (('Adapter', self._rc_mcu_adapter),
                              ('Arm address', self._rc_mcu_arm_ip),
                              ('Setup command', self._rc_mcu_setup),
                              ('Hold after', self._rc_mcu_hold),
                              ('Ramp out after', self._rc_mcu_ramp),
                              ('Cartesian routing', self._rc_mcu_routing)):
            cl.addLayout(self._rc_form_row(label, widget))
        row = QHBoxLayout()
        probe_btn = self._btn('Check now', 'BtnGreen')
        probe_btn.setToolTip('Look for an MCU that can drive this robot (Ethernet, then the Bridge)')
        probe_btn.clicked.connect(self._rc_mcu_check)
        row.addWidget(probe_btn)
        self._rc_mcu_result = QLabel('')
        self._rc_mcu_result.setWordWrap(True)
        row.addWidget(self._rc_mcu_result, 1)
        cl.addLayout(row)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Torque and wrench')
        self._rc_torque_sign = QComboBox()
        self._rc_torque_sign.addItems(['Positive (+1)', 'Negative (-1)'])
        self._rc_torque_sign.setMaximumWidth(200)
        cl.addLayout(self._rc_form_row('Torque sign', self._rc_torque_sign))
        self._rc_gravity_comp = QCheckBox(
            'Measured joint torques are already gravity-free')
        self._rc_wrench_gravity_comp = QCheckBox(
            'Measured wrench is already gravity-free')
        self._rc_wrench_from_api = QCheckBox(
            "Use the driver's wrench (off: compute it from joint torques)")
        for cb in (self._rc_gravity_comp, self._rc_wrench_gravity_comp,
                   self._rc_wrench_from_api):
            cl.addWidget(cb)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Sensor deadbands')
        cl.addWidget(self._rc_hint('Admittance ignores torque and force readings below '
                                   'these, so sensor noise does not move the robot.'))
        db_row = QHBoxLayout()
        db_row.setSpacing(6)
        self._rc_db_joint_tau = MetricField('τ', 1.5, 0.0, 50.0, unit='Nm', decimals=2)
        self._rc_db_joint_tau.setToolTip('Joint admittance torque deadband (Nm).')
        self._rc_db_cart_force = MetricField('F', 4.0, 0.0, 50.0, unit='N', decimals=2)
        self._rc_db_cart_force.setToolTip('Cartesian translational force deadband (N).')
        self._rc_db_cart_torque = MetricField('M', 0.2, 0.0, 10.0, unit='Nm', decimals=3)
        self._rc_db_cart_torque.setToolTip('Cartesian rotational torque deadband (Nm).')
        for mf in (self._rc_db_joint_tau, self._rc_db_cart_force, self._rc_db_cart_torque):
            db_row.addWidget(mf, 1)
        cl.addLayout(db_row)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Gripper')
        self._rc_has_gripper = QCheckBox('Has a gripper')
        cl.addWidget(self._rc_has_gripper)
        grip_row = QHBoxLayout()
        grip_row.setSpacing(6)
        self._rc_gripper_closed = MetricField('closed', 0.0, 0.0, 0.2, unit='m', decimals=4)
        self._rc_gripper_open = MetricField('open', 0.08, 0.0, 0.2, unit='m', decimals=4)
        for mf in (self._rc_gripper_closed, self._rc_gripper_open):
            grip_row.addWidget(mf, 1)
        cl.addLayout(grip_row)
        lay.addWidget(card)
        return w

    def _rc_build_safety_tab(self):
        w, lay = self._rc_tab_page()
        card, cl = self._rc_warmth_card('Workspace box')
        cl.addWidget(self._rc_hint('The end effector is kept inside these limits (metres, '
                                   'base frame) on each enabled axis.'))
        cart_grid = QGridLayout()
        cart_grid.setSpacing(4)
        cart_grid.addWidget(self._rc_sec_lbl(''), 0, 0)
        cart_grid.addWidget(self._rc_sec_lbl('Min'), 0, 1)
        cart_grid.addWidget(self._rc_sec_lbl('Max'), 0, 2)
        cart_grid.addWidget(self._rc_sec_lbl('Enabled'), 0, 3)
        self._rc_sz_cart = {}
        for row_i, axis in enumerate(('X', 'Y', 'Z'), start=1):
            cart_grid.addWidget(self._rc_sec_lbl(axis), row_i, 0)
            sb_min = QDoubleSpinBox()
            sb_min.setRange(-2.0, 2.0)
            sb_min.setDecimals(3)
            sb_min.setValue(-1.0 if axis != 'Z' else 0.0)
            sb_max = QDoubleSpinBox()
            sb_max.setRange(-2.0, 2.0)
            sb_max.setDecimals(3)
            sb_max.setValue(1.0 if axis != 'Z' else 1.5)
            cb_en = QCheckBox()
            cb_en.setChecked(True)
            cart_grid.addWidget(sb_min, row_i, 1)
            cart_grid.addWidget(sb_max, row_i, 2)
            cart_grid.addWidget(cb_en, row_i, 3)
            self._rc_sz_cart[axis.lower()] = (sb_min, sb_max, cb_en)
        cart_w = QWidget()
        cart_w.setLayout(cart_grid)
        cl.addWidget(cart_w)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Joint safety limits')
        cl.addWidget(self._rc_hint('Tighter than the joint limits, in degrees: one '
                                   '"lower, upper" per line, joint 1 first.'))
        self._rc_joint_safety_edit = QTextEdit()
        self._rc_joint_safety_edit.setMaximumHeight(120)
        self._rc_joint_safety_edit.setPlaceholderText('-160, 160\n-95, 95\n...')
        cl.addWidget(self._rc_joint_safety_edit)
        self._rc_joint_safety_enabled = QCheckBox('Enable joint safety limits')
        self._rc_joint_safety_enabled.setChecked(True)
        cl.addWidget(self._rc_joint_safety_enabled)
        lay.addWidget(card)

        card, cl = self._rc_warmth_card('Speed limits')
        vel_row = QHBoxLayout()
        self._rc_max_joint_vel = QDoubleSpinBox()
        self._rc_max_joint_vel.setRange(0.01, 5.0)
        self._rc_max_joint_vel.setDecimals(2)
        self._rc_max_joint_vel.setValue(1.0)
        self._rc_max_joint_vel.setSuffix(' rad/s')
        self._rc_max_cart_vel = QDoubleSpinBox()
        self._rc_max_cart_vel.setRange(0.01, 2.0)
        self._rc_max_cart_vel.setDecimals(2)
        self._rc_max_cart_vel.setValue(0.5)
        self._rc_max_cart_vel.setSuffix(' m/s')
        self._rc_vel_enabled = QCheckBox('Enable speed limits')
        self._rc_vel_enabled.setChecked(True)
        for lbl_txt, sb in (('Joints', self._rc_max_joint_vel),
                            ('End effector', self._rc_max_cart_vel)):
            vel_row.addWidget(self._rc_sec_lbl(lbl_txt))
            vel_row.addWidget(sb)
        vel_row.addWidget(self._rc_vel_enabled)
        vel_row.addStretch()
        cl.addLayout(vel_row)
        lay.addWidget(card)
        return w

    def _rc_connect_dirty(self):
        """Every editable widget marks the robot as having unsaved edits."""
        for edit in (self._rc_display_name, self._rc_urdf_path, self._rc_mujoco_path,
                     self._rc_driver_cmd, self._rc_build_dir, self._rc_build_cmd,
                     self._rc_ee_frame, self._rc_elbow_frame):
            edit.textChanged.connect(self._rc_mark_dirty)
        for cb in (self._rc_is_supported, self._rc_gravity_comp, self._rc_wrench_gravity_comp,
                   self._rc_wrench_from_api, self._rc_has_gripper,
                   self._rc_joint_safety_enabled, self._rc_vel_enabled):
            cb.toggled.connect(self._rc_mark_dirty)
        for sb in (self._rc_num_joints, self._rc_ee_qx, self._rc_ee_qy, self._rc_ee_qz,
                   self._rc_ee_qw, self._rc_max_joint_vel, self._rc_max_cart_vel):
            sb.valueChanged.connect(self._rc_mark_dirty)
        for mf in (self._rc_ee_pos_x, self._rc_ee_pos_y, self._rc_ee_pos_z,
                   self._rc_db_joint_tau, self._rc_db_cart_force, self._rc_db_cart_torque,
                   self._rc_gripper_closed, self._rc_gripper_open):
            mf.committed.connect(self._rc_mark_dirty)
        self._rc_torque_sign.currentIndexChanged.connect(self._rc_mark_dirty)
        self._rc_kin_solver.currentTextChanged.connect(self._rc_mark_dirty)
        self._rc_joint_table.changed.connect(self._rc_mark_dirty)
        self._rc_joint_safety_edit.textChanged.connect(self._rc_mark_dirty)
        for sb_min, sb_max, cb_en in self._rc_sz_cart.values():
            sb_min.valueChanged.connect(self._rc_mark_dirty)
            sb_max.valueChanged.connect(self._rc_mark_dirty)
            cb_en.toggled.connect(self._rc_mark_dirty)

    # -------------------------------------------------------------------------
    # Inline page helpers
    # -------------------------------------------------------------------------

    def _rc_sec_lbl(self, text):
        """Return a small secondary-coloured QLabel for section sub-headers."""
        lbl = QLabel(text)
        theme.bind(lbl, lambda t: 'color: %s; font-size: 11px;' % t['text_secondary'])
        return lbl

    def _make_labeled_line(self, label_text, widget):
        """Return a QWidget row pairing a label with a line-edit widget."""
        row_w = QWidget()
        row_h = QHBoxLayout(row_w)
        row_h.setContentsMargins(0, 0, 0, 0)
        row_h.setSpacing(10)
        lbl = QLabel(label_text)
        lbl.setFixedWidth(160)
        theme.bind(lbl, lambda t: 'color: %s; font-size: 12px;' % t['text_secondary'])
        row_h.addWidget(lbl)
        row_h.addWidget(widget, 1)
        return row_w

    # -------------------------------------------------------------------------
    # The robot list
    # -------------------------------------------------------------------------

    def _rc_current_id(self):
        """ID of the robot the editor shows ('' before the page is built)."""
        lbl = getattr(self, '_rc_robot_id_lbl', None)
        return lbl.text().strip() if lbl is not None else ''

    def _rc_list_ids(self):
        lst = self._rc_robot_list
        return [lst.item(i).data(Qt.UserRole) for i in range(lst.count())]

    def _rc_select_in_list(self, robot_id):
        ids = self._rc_list_ids()
        self._rc_robot_list.blockSignals(True)
        self._rc_robot_list.setCurrentRow(ids.index(robot_id) if robot_id in ids else -1)
        self._rc_robot_list.blockSignals(False)

    def _rc_list_row(self, robot_id, cfg, ready):
        row = QWidget()
        row.setObjectName('RcListRow')
        row.setStyleSheet('QWidget#RcListRow, QWidget#RcListRow QLabel '
                          '{ background: transparent; border: none; }')
        h = QHBoxLayout(row)
        h.setContentsMargins(10, 6, 10, 6)
        h.setSpacing(9)
        dot = QLabel('●')
        tok = 'accent_green' if ready else 'accent_orange'
        theme.bind(dot, lambda t: 'color: %s; font-size: 10px;' % t[tok])
        dot.setToolTip('Ready to simulate' if ready else 'Needs attention: see Overview')
        h.addWidget(dot, 0, Qt.AlignTop)
        text = QVBoxLayout()
        text.setSpacing(1)
        name = QLabel(cfg.get('display_name') or robot_id)
        theme.bind(name, lambda t: 'color: %s; font-size: 13px; font-weight: 600;'
                   % t['text_primary'])
        n = cfg.get('num_joints', len(cfg.get('joint_names', [])))
        detail = QLabel('%s joint%s · %s' % (
            n, '' if n == 1 else 's',
            'hardware + simulation' if cfg.get('is_supported') else 'simulation'))
        theme.bind(detail, lambda t: 'color: %s; font-size: 11px;' % t['text_secondary'])
        text.addWidget(name)
        text.addWidget(detail)
        h.addLayout(text, 1)
        for w in (dot, name, detail):
            w.setAttribute(Qt.WA_TransparentForMouseEvents)
        row.setToolTip(robot_id)
        return row

    def _rc_rebuild_list(self):
        """Rebuild the robot list from ``self.robot_configs``, keeping the selection."""
        ws_src = os.path.join(_FIC_WS, 'src')
        current = self._rc_current_id()
        lst = self._rc_robot_list
        lst.blockSignals(True)
        lst.clear()
        for robot_id, cfg in self.robot_configs.items():
            ready = rc_check.sim_ready(rc_check.check_robot(cfg, ws_src))
            item = QListWidgetItem()
            item.setData(Qt.UserRole, robot_id)
            item.setToolTip(robot_id)
            row = self._rc_list_row(robot_id, cfg, ready)
            item.setSizeHint(QSize(0, 50))
            lst.addItem(item)
            lst.setItemWidget(item, row)
        lst.blockSignals(False)
        if current:
            self._rc_select_in_list(current)

    def _rc_on_list_changed(self, row):
        """Show the robot picked in the list, offering to save unsaved edits first."""
        if row < 0:
            return
        robot_id = self._rc_robot_list.item(row).data(Qt.UserRole)
        if robot_id == self._rc_current_id() or robot_id not in self.robot_configs:
            return
        if not self._rc_confirm_leave('switching to %s' % robot_id):
            self._rc_select_in_list(self._rc_current_id())
            return
        self._rc_load_robot(robot_id)

    def _rc_confirm_leave(self, doing):
        """Return True when it is fine to replace the editor's contents.

        With unsaved edits the user chooses: Save (then go on), Discard, or
        Cancel (stay).
        """
        if not getattr(self, '_rc_dirty', False):
            return True
        reply = QMessageBox.question(
            self, 'Unsaved changes',
            'You have unsaved changes to %s.\nSave them before %s?'
            % (self._rc_current_id() or 'this robot', doing),
            QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel,
        )
        if reply == QMessageBox.Save:
            self._rc_save()
            return not self._rc_dirty
        return reply == QMessageBox.Discard

    def _rc_follow_toolbar(self, robot_id):
        """Show the toolbar's robot, unless the editor holds unsaved edits."""
        if (robot_id in getattr(self, 'robot_configs', {})
                and not getattr(self, '_rc_dirty', False)
                and robot_id != self._rc_current_id()):
            self._rc_load_robot(robot_id)

    # -------------------------------------------------------------------------
    # Save state
    # -------------------------------------------------------------------------

    @staticmethod
    def _rc_fic_config_yaml_path():
        """
        Return the absolute path to fic_library/config/robots_config.yaml.

        Computed relative to this source file's location so it works from any
        working directory.
        """
        base = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
        )
        return os.path.join(base, 'fic_library', 'config', 'robots_config.yaml')

    def _rc_check_sync_state(self):
        """
        Compare mtime of fic_studio_robots.yaml vs the controllers' robots_config.yaml.

        Returns True when the Studio's YAML is strictly newer, i.e. the
        controllers' copy is out of date (edited by hand, or a save failed
        half way). Updates ``_rc_sync_indicator`` accordingly.
        """
        gui_yaml = self.ROBOT_CONFIG_FILE
        deploy_yaml = self._rc_fic_config_yaml_path()
        out_of_sync = False
        if os.path.exists(gui_yaml) and os.path.exists(deploy_yaml):
            out_of_sync = os.path.getmtime(gui_yaml) > os.path.getmtime(deploy_yaml)
        self._rc_update_sync_indicator(out_of_sync)
        return out_of_sync

    def _rc_indicator_state(self):
        if getattr(self, '_rc_dirty', False):
            return 'dirty'
        return 'stale' if getattr(self, '_rc_out_of_sync', False) else 'saved'

    def _rc_indicator_style(self, t):
        if self._rc_indicator_state() == 'saved':
            return 'color: %s; font-size: 11px;' % t['text_disabled']
        return 'color: %s; font-size: 11px; font-weight: 700;' % t['accent_orange']

    def _rc_update_sync_indicator(self, out_of_sync=None):
        """
        Show whether this robot is saved.

        Amber (``accent_orange``) for unsaved edits or an out-of-date
        controllers' copy; quiet (``text_disabled``) when everything is saved.
        """
        if out_of_sync is not None:
            self._rc_out_of_sync = out_of_sync
        ind = getattr(self, '_rc_sync_indicator', None)
        if ind is None:
            return
        state = self._rc_indicator_state()
        ind.setText({
            'dirty': '● Unsaved changes',
            'stale': "● Controllers' copy is older: Save to update",
            'saved': '✓ Saved',
        }[state])
        ind.setStyleSheet(self._rc_indicator_style(theme.current()))

    # -------------------------------------------------------------------------
    # Path-exists indicator
    # -------------------------------------------------------------------------

    def _rc_update_path_indicator(self, line_edit):
        """
        Apply a brick-red border to ``line_edit`` when the resolved path is missing.

        The path is resolved relative to the workspace src directory.  An empty
        path is treated as neutral (no warning border).  Uses ``theme.bind``
        tokens — ``accent_red`` for missing, ``border_subtle`` for present/empty.
        """
        text = line_edit.text().strip()
        ws_src = os.path.join(_FIC_WS, 'src')
        missing = bool(text) and not os.path.exists(
            text if os.path.isabs(text) else os.path.join(ws_src, text))
        line_edit.setProperty('rc_missing', missing)
        if not getattr(line_edit, '_rc_bound', False):
            line_edit._rc_bound = True
            theme.bind(line_edit, lambda t, e=line_edit: (
                'QLineEdit { border: 2px solid %s; border-radius: 4px; padding: 2px 4px; }'
                % t['accent_red'] if e.property('rc_missing') else
                'QLineEdit { border: 1px solid %s; border-radius: 4px; padding: 2px 4px; }'
                % t['border_subtle']))
        else:
            t = theme.current()
            line_edit.setStyleSheet(
                'QLineEdit { border: 2px solid %s; border-radius: 4px; padding: 2px 4px; }'
                % t['accent_red'] if missing else
                'QLineEdit { border: 1px solid %s; border-radius: 4px; padding: 2px 4px; }'
                % t['border_subtle'])

    # -------------------------------------------------------------------------
    # Dirty flag
    # -------------------------------------------------------------------------

    def _rc_mark_dirty(self, *_args):
        """Mark the current robot as having unsaved edits."""
        self._rc_dirty = True
        self._rc_update_sync_indicator()

    def _rc_clear_dirty(self):
        """Clear the dirty flag (call after save or load)."""
        self._rc_dirty = False
        self._rc_update_sync_indicator()

    def _rc_publish_hot_config(self, robot_id=None):
        """Push the hot-reloadable switches to controllers running this robot.

        Only stations whose robot is ``robot_id``: saving one robot must not
        flip another robot's gravity or wrench handling mid-run.
        """
        from std_msgs.msg import Bool
        robot_id = robot_id or self._rc_current_id()
        cfg = self.robot_configs.get(robot_id)
        if not cfg:
            return
        node = getattr(self, 'node', None)
        pubs = getattr(node, 'pubs', None) or {}
        for rid in self._rc_live_rids(robot_id, pubs):
            station = pubs[rid]
            if 'gravity_comp' in station:
                station['gravity_comp'].publish(
                    Bool(data=cfg.get('gravity_compensated', False)))
            if 'wrench_gravity_comp' in station:
                station['wrench_gravity_comp'].publish(
                    Bool(data=cfg.get('wrench_gravity_compensated', False)))
            if 'wrench_from_api' in station:
                station['wrench_from_api'].publish(
                    Bool(data=cfg.get('wrench_from_api', False)))

    def _rc_live_rids(self, robot_id, pubs):
        """Topic prefixes ('robotN') of the stations set to ``robot_id``."""
        node = getattr(self, 'node', None)
        dropdown = getattr(self, 'robot_dropdown', None)
        types = {'robot%s' % getattr(node, 'station_id', 1):
                 dropdown.currentText() if dropdown is not None else ''}
        if hasattr(self, '_card_topic_rid') and hasattr(self, 'card_robot_type'):
            for card in getattr(self, '_card_robot_types', {}):
                types[self._card_topic_rid(card)] = self.card_robot_type(card)
        return [rid for rid in pubs if types.get(rid) == robot_id]

    # -------------------------------------------------------------------------
    # Quaternion norm readout
    # -------------------------------------------------------------------------

    def _rc_update_qnorm(self):
        """Recalculate |q| and update the readout label."""
        qx = self._rc_ee_qx.value()
        qy = self._rc_ee_qy.value()
        qz = self._rc_ee_qz.value()
        qw = self._rc_ee_qw.value()
        norm = math.sqrt(qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2)
        self._rc_qnorm_lbl.setText('|q| = %.4f' % norm)

    def _rc_rebuild_home_rows(self, n):
        """
        Rebuild the per-joint home-angle MetricFields to exactly ``n``.

        Laid out four to a row. Existing values are preserved up to
        ``min(old_count, n)``; new fields start at 0.0.
        """
        old_vals = [mf.value() for mf in self._rc_home_fields]
        while self._rc_home_layout.count():
            item = self._rc_home_layout.takeAt(0)
            w = item.widget()
            if w:
                w.hide()
                w.deleteLater()
        self._rc_home_fields = []
        cols = 4
        for i in range(n):
            mf = MetricField('J%d' % (i + 1), old_vals[i] if i < len(old_vals) else 0.0,
                             -360.0, 360.0, unit='deg', decimals=2)
            mf.committed.connect(self._rc_mark_dirty)
            self._rc_home_fields.append(mf)
            self._rc_home_layout.addWidget(mf, i // cols, i % cols)
        for c in range(cols):
            self._rc_home_layout.setColumnStretch(c, 1)

    def _rc_on_num_joints_changed(self, value):
        """Resize joint table and home rows when the num_joints spinner changes."""
        self._rc_joint_table.set_count(value)
        self._rc_rebuild_home_rows(value)

    # -------------------------------------------------------------------------
    # Load / Collect
    # -------------------------------------------------------------------------

    def _rc_load_robot(self, robot_id):
        """
        Populate all editor widgets from ``self.robot_configs[robot_id]``.

        Drives JointTable and home rows from joint_names + limits + torque.
        Safe to call even if some optional keys are absent.
        """
        cfg = self.robot_configs.get(robot_id, {})

        # Identity
        self._rc_robot_id_lbl.setText(robot_id)
        self._rc_select_in_list(robot_id)
        self._rc_display_name.setText(cfg.get('display_name', robot_id))
        n = cfg.get('num_joints', 7)
        self._rc_num_joints.blockSignals(True)
        self._rc_num_joints.setValue(n)
        self._rc_num_joints.blockSignals(False)
        self._rc_is_supported.setChecked(cfg.get('is_supported', False))
        sign = cfg.get('torque_sign', 1)
        self._rc_torque_sign.setCurrentIndex(0 if sign >= 0 else 1)

        # Joint Definition table
        joint_names = cfg.get('joint_names', ['joint%d' % (i + 1) for i in range(n)])
        limits = cfg.get('joint_limits_deg', {})
        lower_lims = limits.get('lower', [0.0] * n)
        upper_lims = limits.get('upper', [0.0] * n)
        torque_lims = cfg.get('joint_torque_limits', {})
        torque_maxs = torque_lims.get('max', [50.0] * n)

        rows_data = []
        for i in range(n):
            rows_data.append({
                'name': joint_names[i] if i < len(joint_names) else '',
                'lower': lower_lims[i] if i < len(lower_lims) else 0.0,
                'upper': upper_lims[i] if i < len(upper_lims) else 0.0,
                'torque_max': torque_maxs[i] if i < len(torque_maxs) else 50.0,
            })
        self._rc_joint_table.set_rows(rows_data)

        # Home pose
        self._rc_rebuild_home_rows(n)
        q_home_deg = cfg.get('q_home_deg', [0.0] * n)
        for i, mf in enumerate(self._rc_home_fields):
            mf.set_value(q_home_deg[i] if i < len(q_home_deg) else 0.0)

        # Default Cartesian target
        ee_tgt = cfg.get('default_ee_target', {})
        pos = ee_tgt.get('position', [0.0, 0.0, 0.0])
        self._rc_ee_pos_x.set_value(pos[0] if len(pos) > 0 else 0.0)
        self._rc_ee_pos_y.set_value(pos[1] if len(pos) > 1 else 0.0)
        self._rc_ee_pos_z.set_value(pos[2] if len(pos) > 2 else 0.0)
        ori = ee_tgt.get('orientation', [0.0, 0.0, 0.0, 1.0])
        for i, sb in enumerate((self._rc_ee_qx, self._rc_ee_qy, self._rc_ee_qz,
                                self._rc_ee_qw)):
            sb.blockSignals(True)
            sb.setValue(ori[i] if len(ori) > i else (1.0 if i == 3 else 0.0))
            sb.blockSignals(False)
        self._rc_update_qnorm()

        # Admittance & Gravity
        db = cfg.get('admittance_deadbands', {})
        self._rc_db_joint_tau.set_value(db.get('joint_tau', 1.5))
        self._rc_db_cart_force.set_value(db.get('cart_force', 4.0))
        self._rc_db_cart_torque.set_value(db.get('cart_torque', 0.2))
        self._rc_gravity_comp.setChecked(cfg.get('gravity_compensated', False))
        self._rc_wrench_gravity_comp.setChecked(
            cfg.get('wrench_gravity_compensated', cfg.get('gravity_compensated', False))
        )
        self._rc_wrench_from_api.setChecked(cfg.get('wrench_from_api', False))

        # Gripper
        self._rc_has_gripper.setChecked(cfg.get('has_gripper', False))
        grip = cfg.get('gripper_range', [0.0, 0.08])
        self._rc_gripper_closed.set_value(grip[0] if len(grip) > 0 else 0.0)
        self._rc_gripper_open.set_value(grip[1] if len(grip) > 1 else 0.08)

        # Frame names
        self._rc_ee_frame.setText(cfg.get('ee_frame_name', 'ee_link'))
        self._rc_elbow_frame.setText(cfg.get('elbow_frame_name', ''))
        self._rc_kin_solver.setCurrentText(cfg.get('kinematics_solver', 'pinocchio'))

        # File paths & build
        self._rc_urdf_path.setText(cfg.get('urdf_path', ''))
        self._rc_mujoco_path.setText(cfg.get('mujoco_xml_path', ''))
        self._rc_driver_cmd.setText(cfg.get('driver_command', ''))
        self._rc_build_dir.setText(cfg.get('build_directory', ''))
        self._rc_build_cmd.setText(cfg.get('build_command', ''))
        self._rc_load_mcu(robot_id, cfg)

        # Safety zones
        sz = cfg.get('safety_zones', {})
        cart_sz = sz.get('cartesian', {})
        defaults = {'x': (-1.0, 1.0), 'y': (-1.0, 1.0), 'z': (0.0, 1.5)}
        for axis in ('x', 'y', 'z'):
            axis_cfg = cart_sz.get(axis, {})
            sb_min, sb_max, cb_en = self._rc_sz_cart[axis]
            sb_min.setValue(axis_cfg.get('min', defaults[axis][0]))
            sb_max.setValue(axis_cfg.get('max', defaults[axis][1]))
            cb_en.setChecked(axis_cfg.get('enabled', True))

        jsz = sz.get('joint_safety_deg', {})
        js_lower = jsz.get('lower', [])
        js_upper = jsz.get('upper', [])
        if js_lower and js_upper:
            self._rc_joint_safety_edit.setPlainText('\n'.join(
                '%g, %g' % (lo, hi) for lo, hi in zip(js_lower, js_upper)))
        else:
            self._rc_joint_safety_edit.clear()
        self._rc_joint_safety_enabled.setChecked(jsz.get('enabled', True))

        vel_sz = sz.get('velocity', {})
        self._rc_max_joint_vel.setValue(vel_sz.get('max_joint_vel', 1.0))
        self._rc_max_cart_vel.setValue(vel_sz.get('max_cart_vel', 0.5))
        self._rc_vel_enabled.setChecked(vel_sz.get('enabled', True))

        self._rc_update_path_indicator(self._rc_urdf_path)
        self._rc_update_path_indicator(self._rc_mujoco_path)
        self._rc_update_header()
        self._rc_refresh_status()

        # A fresh load, not a user edit
        self._rc_clear_dirty()

    def _rc_update_header(self):
        robot_id = self._rc_current_id()
        cfg = self.robot_configs.get(robot_id, {})
        self._rc_title_lbl.setText(cfg.get('display_name') or robot_id)
        n = cfg.get('num_joints', len(cfg.get('joint_names', [])))
        self._rc_subtitle_lbl.setText('%s · %s joint%s' % (robot_id, n, '' if n == 1 else 's'))
        if cfg.get('is_supported'):
            self._rc_set_pill(self._rc_badge_hw, 'Hardware driver', 'accent_blue')
        else:
            self._rc_set_pill(self._rc_badge_hw, 'Simulation only', 'text_disabled')

    def _rc_model_dir(self, cfg=None):
        """menagerie/model/<name> folder of a robot (from its MuJoCo path), or ''."""
        cfg = cfg if cfg is not None else self.robot_configs.get(self._rc_current_id(), {})
        rel = cfg.get('mujoco_xml_path') or cfg.get('urdf_path') or ''
        if not rel:
            return ''
        ws_src = os.path.join(_FIC_WS, 'src')
        parts = rel.replace('\\', '/').split('/')
        if 'model' in parts[:-1]:
            i = parts.index('model')
            return os.path.join(ws_src, *parts[:i + 2])
        return os.path.dirname(rc_check.resolve(ws_src, rel))

    def _rc_refresh_status(self):
        """Re-run the checks for the saved config and show them on Overview."""
        robot_id = self._rc_current_id()
        cfg = self.robot_configs.get(robot_id, {})
        ws_src = os.path.join(_FIC_WS, 'src')
        messages = rc_check.check_robot(cfg, ws_src) if cfg else []
        while self._rc_status_holder.count():
            w = self._rc_status_holder.takeAt(0).widget()
            if w:
                w.hide()
                w.deleteLater()
        icon = {rc_check.OK: '✓', rc_check.WARN: '!', rc_check.ERROR: '✕'}
        tok = {rc_check.OK: 'accent_green', rc_check.WARN: 'accent_orange',
               rc_check.ERROR: 'accent_red'}
        for level, text in messages:
            row = QLabel('%s  %s' % (icon[level], text))
            row.setWordWrap(True)
            row.setTextInteractionFlags(Qt.TextSelectableByMouse)
            theme.bind(row, lambda t, k=tok[level]: 'color: %s; font-size: 12px;' % t[k])
            self._rc_status_holder.addWidget(row)
        ready = rc_check.sim_ready(messages) if cfg else False
        self._rc_set_pill(self._rc_badge_sim,
                          'Simulation ready' if ready else 'Needs attention',
                          'accent_green' if ready else 'accent_orange')
        xml = rc_check.resolve(ws_src, cfg.get('mujoco_xml_path', ''))
        self._rc_try_btn.setEnabled(ready)
        self._rc_viewer_btn.setEnabled(bool(xml) and os.path.isfile(xml))
        model_dir = self._rc_model_dir(cfg)
        self._rc_report_btn.setVisible(
            bool(model_dir) and os.path.isfile(os.path.join(model_dir, 'import_report.md')))

    def _rc_collect_robot(self):
        """
        Read all editor widgets and return a complete robot config dict.

        Derives ``q_home_rad`` from ``q_home_deg``; assembles
        ``joint_limits_deg``, ``joint_torque_limits``, and ``safety_zones``
        from their respective widgets.  Raises ``ValueError`` on parse errors.
        """
        n = self._rc_num_joints.value()
        table_rows = self._rc_joint_table.rows()

        joint_names = [r['name'] for r in table_rows]
        lower_lims = [r['lower'] for r in table_rows]
        upper_lims = [r['upper'] for r in table_rows]
        torque_maxs = [r['torque_max'] for r in table_rows]
        # The editor has no field for the lower torque limit; keep what the
        # config had (KinovaGen3_CSPM's wrist inputs are symmetric, -0.235).
        old_min = list(self.robot_configs.get(self._rc_current_id(), {})
                       .get('joint_torque_limits', {}).get('min', []))
        torque_mins = (old_min + [0.0] * len(table_rows))[:len(table_rows)]

        q_home_deg = [mf.value() for mf in self._rc_home_fields]
        q_home_rad = [math.radians(d) for d in q_home_deg]

        # Parse joint safety limits from text
        js_text = self._rc_joint_safety_edit.toPlainText().strip()
        js_lower = []
        js_upper = []
        if js_text:
            for line in js_text.split('\n'):
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) < 2:
                    raise ValueError('Joint safety line must be "lower, upper": %r' % line)
                js_lower.append(float(parts[0].strip()))
                js_upper.append(float(parts[1].strip()))

        safety_zones = {
            'cartesian': {},
            'joint_safety_deg': {
                'lower': js_lower,
                'upper': js_upper,
                'enabled': self._rc_joint_safety_enabled.isChecked(),
            },
            'velocity': {
                'max_joint_vel': self._rc_max_joint_vel.value(),
                'max_cart_vel': self._rc_max_cart_vel.value(),
                'enabled': self._rc_vel_enabled.isChecked(),
            },
        }
        for axis in ('x', 'y', 'z'):
            sb_min, sb_max, cb_en = self._rc_sz_cart[axis]
            safety_zones['cartesian'][axis] = {
                'min': sb_min.value(),
                'max': sb_max.value(),
                'enabled': cb_en.isChecked(),
            }

        return {
            'display_name': (self._rc_display_name.text().strip()
                             or self._rc_robot_id_lbl.text()),
            'num_joints': n,
            'joint_names': joint_names,
            'urdf_path': self._rc_urdf_path.text().strip(),
            'mujoco_xml_path': self._rc_mujoco_path.text().strip(),
            'ee_frame_name': self._rc_ee_frame.text().strip() or 'ee_link',
            'elbow_frame_name': self._rc_elbow_frame.text().strip(),
            'kinematics_solver': self._rc_kin_solver.currentText().strip() or 'pinocchio',
            'default_ee_target': {
                'position': [
                    self._rc_ee_pos_x.value(),
                    self._rc_ee_pos_y.value(),
                    self._rc_ee_pos_z.value(),
                ],
                'orientation': [
                    self._rc_ee_qx.value(),
                    self._rc_ee_qy.value(),
                    self._rc_ee_qz.value(),
                    self._rc_ee_qw.value(),
                ],
            },
            'q_home_deg': q_home_deg,
            'q_home_rad': q_home_rad,
            'joint_limits_deg': {
                'lower': lower_lims,
                'upper': upper_lims,
            },
            'joint_torque_limits': {
                'min': torque_mins,
                'max': torque_maxs,
            },
            'torque_sign': 1 if self._rc_torque_sign.currentIndex() == 0 else -1,
            'gravity_compensated': self._rc_gravity_comp.isChecked(),
            'wrench_gravity_compensated': self._rc_wrench_gravity_comp.isChecked(),
            'wrench_from_api': self._rc_wrench_from_api.isChecked(),
            'admittance_deadbands': {
                'joint_tau': self._rc_db_joint_tau.value(),
                'cart_force': self._rc_db_cart_force.value(),
                'cart_torque': self._rc_db_cart_torque.value(),
            },
            'has_gripper': self._rc_has_gripper.isChecked(),
            'gripper_range': [
                self._rc_gripper_closed.value(),
                self._rc_gripper_open.value(),
            ],
            'is_supported': self._rc_is_supported.isChecked(),
            'driver_command': self._rc_driver_cmd.text().strip(),
            'build_directory': self._rc_build_dir.text().strip(),
            'build_command': self._rc_build_cmd.text().strip(),
            'safety_zones': safety_zones,
            'mcu': self._rc_collect_mcu(),
        }

    # -------------------------------------------------------------------------
    # Inner-loop MCU (Hardware tab)
    # -------------------------------------------------------------------------

    def _rc_mcu_settings(self, robot_id, cfg):
        """The robot's mcu block: the Studio's copy, else the controllers'."""
        from fic_studio.mcu_probe import controller_mcu_settings
        mcu = cfg.get('mcu')
        return dict(mcu) if isinstance(mcu, dict) else dict(controller_mcu_settings(robot_id))

    def _rc_load_mcu(self, robot_id, cfg):
        mcu = self._rc_mcu_settings(robot_id, cfg)
        self._rc_mcu_base = mcu                 # keys without a widget survive a Save
        adapter = mcu.get('adapter', '') or ''
        idx = self._rc_mcu_adapter.findData(adapter)
        if idx < 0:                              # an adapter this editor does not list
            self._rc_mcu_adapter.addItem(adapter, adapter)
            idx = self._rc_mcu_adapter.count() - 1
        self._rc_mcu_adapter.setCurrentIndex(idx)
        self._rc_mcu_arm_ip.setText(mcu.get('arm_ip', ''))
        self._rc_mcu_setup.setText(mcu.get('setup_command', ''))
        self._rc_mcu_hold.setValue(float(mcu.get('hold_after_s', 0.05)))
        self._rc_mcu_ramp.setValue(float(mcu.get('ramp_after_s', 2.0)))
        self._rc_mcu_routing.setCurrentText(mcu.get('cartesian_routing', 'haptic'))
        self._rc_mcu_result.setText('')

    def _rc_collect_mcu(self):
        adapter = self._rc_mcu_adapter.currentData() or ''
        if not adapter:
            return {}
        mcu = dict(getattr(self, '_rc_mcu_base', {}) or {})
        mcu.update({
            'adapter': adapter,
            'hold_after_s': round(self._rc_mcu_hold.value(), 3),
            'ramp_after_s': round(self._rc_mcu_ramp.value(), 2),
            'cartesian_routing': self._rc_mcu_routing.currentText(),
        })
        for key, edit in (('arm_ip', self._rc_mcu_arm_ip), ('setup_command', self._rc_mcu_setup)):
            text = edit.text().strip()
            if text:
                mcu[key] = text
            else:
                mcu.pop(key, None)
        return mcu

    def _rc_mcu_check(self):
        """Probe for an MCU with the adapter as edited (not yet saved)."""
        from fic_studio.mcu_probe import discover
        robot_id = self._rc_robot_id_lbl.text()
        self._rc_mcu_result.setText('looking ...')
        QApplication.processEvents()
        info = discover({'mcu': self._rc_collect_mcu() or {'adapter': ''}}, robot_name=None)
        mark = '✓ ' if info['use'] else ''
        self._rc_mcu_result.setText(mark + info['detail'])
        self.log_message('INFO', 'MCU check for %s: %s' % (robot_id, info['detail']), 'Config')
        if hasattr(self, 'il_refresh_probe'):
            self.il_refresh_probe()

    # -------------------------------------------------------------------------
    # Save
    # -------------------------------------------------------------------------

    def _rc_persist(self):
        """Write both robot YAMLs and refresh everything that lists robots.

        Returns False when a write failed (the user has been told).
        """
        ok = self._save_robot_configs() is not False
        ok = self._sync_to_fic_config() is not False and ok
        self._rc_refresh_robot_lists()
        self._rc_check_sync_state()
        return ok

    def _rc_save(self):
        """
        Save the robot: both YAMLs, then the running controllers' switches.

        Merges over the stored config, so keys the editor has no widget for
        (loop_closures, urdf_path_gripper, a deadband's per-joint list...)
        survive, however deeply nested.
        """
        robot_id = self._rc_current_id()
        if not robot_id:
            QMessageBox.warning(self, 'No Robot', 'No robot is currently selected.')
            return
        try:
            cfg = self._rc_collect_robot()
        except ValueError as exc:
            QMessageBox.warning(self, 'Validation Error', str(exc))
            return
        self.robot_configs[robot_id] = merge_robot_config(
            self.robot_configs.get(robot_id, {}), cfg)
        if not self._rc_persist():
            return
        self._rc_publish_hot_config(robot_id)
        self._rc_clear_dirty()
        self._rc_rebuild_list()
        self._rc_update_header()
        self._rc_refresh_status()
        self.log_message('INFO', 'Saved robot: %s' % robot_id, 'Config')

    # -------------------------------------------------------------------------
    # Robots: add, import, duplicate, delete
    # -------------------------------------------------------------------------

    def _rc_model_root(self):
        """Where robot models live: fic_ws/src/menagerie/model."""
        return os.path.join(_FIC_WS, 'src', 'menagerie', 'model')

    def _rc_add_robot(self):
        """Add a blank robot: prompt for an ID, insert the default template, select."""
        from PyQt5.QtWidgets import QInputDialog
        from fic_studio.robot_import.importer import valid_robot_id
        if not self._rc_confirm_leave('adding a robot'):
            return
        robot_id, ok = QInputDialog.getText(
            self, 'Add Robot', 'Robot ID (letters, digits and _):'
        )
        if not ok or not robot_id.strip():
            return
        robot_id = robot_id.strip()
        if not valid_robot_id(robot_id):
            QMessageBox.warning(self, 'Invalid ID',
                                'A robot ID starts with a letter and uses only letters, '
                                'digits and _.')
            return
        if robot_id in self.robot_configs:
            QMessageBox.warning(self, 'Duplicate', "Robot '%s' already exists." % robot_id)
            return
        import copy
        new_cfg = copy.deepcopy(DEFAULT_ROBOT_TEMPLATE)
        new_cfg['display_name'] = robot_id
        new_cfg['urdf_path'] = 'menagerie/model/%s/URDF/%s.urdf' % (robot_id, robot_id)
        new_cfg['mujoco_xml_path'] = 'menagerie/model/%s/xml/scene.xml' % robot_id
        self.robot_configs[robot_id] = new_cfg
        self._rc_persist()
        self._rc_rebuild_list()
        self._rc_load_robot(robot_id)
        self.log_message('INFO', 'Added robot: %s' % robot_id, 'Config')

    def _rc_start_import(self, path):
        """Import a robot from what was dropped.

        A URDF (or its folder / zip), and optionally the robot's own MuJoCo
        model dropped along with it.
        """
        from PyQt5.QtWidgets import QApplication
        from fic_studio.robot_import import importer, mujoco_model
        from fic_studio.components.robot_import_dialog import RobotImportDialog
        if not self._rc_confirm_leave('adding a robot'):
            return
        mjcf = None
        if isinstance(path, (list, tuple)):
            models = [p for p in path if mujoco_model.is_mjcf(p)]
            rest = [p for p in path if p not in models]
            if not rest:
                QMessageBox.warning(self, 'Cannot add this robot', importer.URDF_REQUIRED)
                return
            path, mjcf = rest[0], (models[0] if models else None)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            plan = importer.analyse(path, existing_ids=list(self.robot_configs), mjcf_path=mjcf)
        except importer.ImportError_ as exc:
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, 'Cannot add this robot', str(exc))
            return
        except Exception as exc:
            QApplication.restoreOverrideCursor()
            self.log_message('ERROR', 'Robot import failed: %r' % exc, 'Config')
            QMessageBox.critical(self, 'Cannot add this robot',
                                 'Reading %s failed: %s' % (os.path.basename(path), exc))
            return
        QApplication.restoreOverrideCursor()
        dlg = None
        try:
            dlg = RobotImportDialog(plan, list(self.robot_configs), self._rc_model_root(),
                                    register=self._rc_register_imported_robot, parent=self)
            dlg.exec_()
        finally:
            # Picking another file in the dialog hands the unpacked zip on
            # to a new plan, so clean up whichever one it ended with.
            (dlg.plan if dlg is not None else plan).cleanup()
        if dlg.next_action == 'simulate':
            self._rc_try_in_sim()
        elif dlg.next_action == 'viewer':
            self._rc_open_viewer()

    def _rc_register_imported_robot(self, result):
        """Add a robot the importer just built, and show it."""
        robot_id = result.robot_id
        self.robot_configs[robot_id] = result.config
        self._rc_persist()
        self._rc_dirty = False
        self._rc_rebuild_list()
        self._rc_load_robot(robot_id)
        self._rc_tabs.setCurrentIndex(0)
        self.log_message('INFO', 'Added robot %s from its URDF (%d joints, model in %s)'
                         % (robot_id, result.config['num_joints'], result.model_dir),
                         'Config')

    def _rc_duplicate_robot(self):
        """Duplicate the current robot with a '_copy' suffix."""
        import copy
        robot_id = self._rc_current_id()
        if not robot_id or robot_id not in self.robot_configs:
            QMessageBox.information(self, 'No Robot', 'Select a robot to duplicate.')
            return
        if not self._rc_confirm_leave('duplicating it'):
            return
        new_id = robot_id + '_copy'
        suffix = 1
        while new_id in self.robot_configs:
            new_id = '%s_copy%d' % (robot_id, suffix)
            suffix += 1
        new_cfg = copy.deepcopy(self.robot_configs[robot_id])
        new_cfg['display_name'] = new_cfg.get('display_name', robot_id) + ' (Copy)'
        self.robot_configs[new_id] = new_cfg
        self._rc_persist()
        self._rc_rebuild_list()
        self._rc_load_robot(new_id)
        self.log_message('INFO', 'Duplicated robot as: %s' % new_id, 'Config')

    def _rc_delete_robot(self):
        """Delete the current robot after confirmation.

        A robot the importer made also has a model folder; that is only
        removed when the user says so, in a second question.
        """
        robot_id = self._rc_current_id()
        if not robot_id or robot_id not in self.robot_configs:
            QMessageBox.information(self, 'No Robot', 'Select a robot to delete.')
            return
        reply = QMessageBox.question(
            self, 'Confirm Delete',
            "Delete '%s' from the robot list?  This cannot be undone." % robot_id,
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        model_dir = self._rc_model_dir(self.robot_configs[robot_id])
        imported = bool(model_dir) and os.path.isfile(
            os.path.join(model_dir, 'import_report.md'))
        del self.robot_configs[robot_id]
        if imported and QMessageBox.question(
                self, 'Delete its files too?',
                'Also delete the files the importer made for %s?\n%s' % (robot_id, model_dir),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            import shutil
            shutil.rmtree(model_dir, ignore_errors=True)
            self.log_message('WARN', 'Deleted model folder %s' % model_dir, 'Config')
        self._rc_persist()
        self._rc_dirty = False
        self._rc_robot_id_lbl.setText('')
        self._rc_rebuild_list()
        if self.robot_configs:
            self._rc_load_robot(next(iter(self.robot_configs)))
        self.log_message('WARN', 'Deleted robot: %s' % robot_id, 'Config')

    def _rc_compile_driver(self):
        """Compile the current robot's driver via ProcessMixin."""
        robot_id = self._rc_current_id()
        if not robot_id:
            QMessageBox.information(self, 'No Robot', 'Select a robot first.')
            return
        if hasattr(self, '_compile_robot_api'):
            self._compile_robot_api(robot_id)
        else:
            QMessageBox.warning(self, 'Not Available', 'Compile function not available.')

    # -------------------------------------------------------------------------
    # Driver library (Hardware tab)
    # -------------------------------------------------------------------------

    def _rc_library_root(self):
        """Where driver libraries live: FIC_Interface/robot_library ({robot_lib})."""
        return _FIC_ROBOT_LIB

    def _rc_start_library(self, path):
        """Set up a dropped driver library as the current robot's driver.

        Review it, copy it into robot_library, fill in the build and driver
        commands, then build it and point the driver command at the program
        the build made.
        """
        from PyQt5.QtWidgets import QApplication
        from fic_studio.robot_import import driver_lib
        from fic_studio.components.driver_library_dialog import DriverLibraryDialog
        robot_id = self._rc_current_id()
        if not robot_id or robot_id not in self.robot_configs:
            QMessageBox.information(self, 'No Robot', 'Select the robot this driver is for.')
            return
        lib_root = self._rc_library_root()
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            plan = driver_lib.analyse_library(path, lib_root=lib_root)
        except Exception as exc:          # every failure is shown, none is fatal
            QApplication.restoreOverrideCursor()
            known = isinstance(exc, driver_lib.LibraryError)
            if not known:
                self.log_message('ERROR', 'Reading driver library failed: %r' % exc, 'Config')
            QMessageBox.warning(self, 'Cannot use this library',
                                str(exc) if known else 'Reading %s failed: %s'
                                % (os.path.basename(path), exc))
            return
        QApplication.restoreOverrideCursor()
        try:
            cfg = self.robot_configs[robot_id]
            dlg = DriverLibraryDialog(
                plan, lib_root, robot_id, configs=self.robot_configs, parent=self,
                current={k: cfg.get(k, '') for k in
                         ('driver_command', 'build_directory', 'build_command')})
            if dlg.exec_() != QDialog.Accepted or not dlg.settings:
                return
            settings = dlg.settings
            plan.name = settings['name']
            try:
                lib_dir = self._rc_copy_library(plan, lib_root, settings['replace'])
            except driver_lib.Cancelled:
                self.log_message('INFO', 'Copying %s cancelled' % plan.name, 'Config')
                return
            except Exception as exc:
                QMessageBox.warning(self, 'Cannot copy the library', str(exc))
                return
        finally:
            plan.cleanup()
        if not self._rc_apply_driver(robot_id, settings):
            return
        self.log_message('INFO', 'Driver library %s set up for %s%s'
                         % (lib_dir, robot_id, '' if plan.in_place else ' (copied)'), 'Config')
        code, since = None, None
        if settings['compile_now'] and hasattr(self, '_compile_robot_api'):
            import time
            build_dir = driver_lib.expand(settings['build_directory'], lib_root, robot_id)
            os.makedirs(build_dir, exist_ok=True)   # a CMake build folder is made, not shipped
            since = time.time() - 1
            code = self._compile_robot_api(robot_id)
        self._rc_after_library(robot_id, lib_dir, plan.drivers, settings, code, since)

    def _rc_copy_library(self, plan, lib_root, replace):
        """Copy the library into robot_library, with a progress bar; return its folder."""
        import threading
        from PyQt5.QtWidgets import QApplication, QProgressDialog
        from fic_studio.robot_import import driver_lib
        if plan.in_place:
            return plan.root
        state = {'bytes': 0, 'cancel': False, 'result': None, 'error': None}

        def progress(n):
            state['bytes'] = n
            if state['cancel']:
                raise driver_lib.Cancelled('Copy cancelled.')

        def work():
            try:
                state['result'] = driver_lib.install_library(plan, lib_root, replace, progress)
            except Exception as exc:
                state['error'] = exc
        bar = QProgressDialog('Copying %s into robot_library…' % plan.name, 'Cancel',
                              0, 1000, self)
        bar.setWindowTitle('Driver library')
        bar.setWindowModality(Qt.WindowModal)
        bar.setMinimumDuration(400)
        bar.canceled.connect(lambda: state.__setitem__('cancel', True))
        thread = threading.Thread(target=work, name='fic-copy-driver-lib', daemon=True)
        thread.start()
        total = max(plan.size_bytes, 1)
        while thread.is_alive():
            thread.join(0.05)
            if not state['cancel']:
                bar.setValue(min(999, int(1000 * state['bytes'] / total)))
            QApplication.processEvents()
        bar.reset()
        if state['error'] is not None:
            raise state['error']
        return state['result']

    def _rc_apply_driver(self, robot_id, settings):
        """Store the driver settings for ``robot_id``, show them, and save them.

        Only these keys are written: other edits on the page stay unsaved,
        as the user left them. ``is_supported`` is never touched: the
        hardware node controls a robot only once the user has ticked it.
        """
        new = {k: settings[k] for k in ('driver_command', 'build_directory', 'build_command')}
        self.robot_configs[robot_id] = merge_robot_config(self.robot_configs[robot_id], new)
        was_dirty = getattr(self, '_rc_dirty', False)
        if self._rc_current_id() == robot_id:
            self._rc_driver_cmd.setText(new['driver_command'])
            self._rc_build_dir.setText(new['build_directory'])
            self._rc_build_cmd.setText(new['build_command'])
        ok = self._rc_persist()
        if ok and not was_dirty:
            self._rc_clear_dirty()
        self._rc_rebuild_list()
        self._rc_update_header()
        self._rc_refresh_status()
        return ok

    def _rc_after_library(self, robot_id, lib_dir, drivers, settings, code, since):
        """Point the driver command at the built program, and say how it went."""
        from fic_studio.robot_import import driver_lib
        program = settings['program']
        exists = bool(program) and os.path.isfile(os.path.join(lib_dir, program))
        if code == 0 and not exists:
            found = driver_lib.pick_program(lib_dir, drivers, since)
            if found:
                program, exists = found, True
                settings = dict(settings, program=found,
                                driver_command=driver_lib.driver_command(
                                    settings['name'], found, settings['args']))
        if exists:
            if settings['driver_command'] != self.robot_configs[robot_id]['driver_command']:
                self._rc_apply_driver(robot_id, settings)
            self.log_message('INFO', 'Driver for %s: %s' % (robot_id, settings['driver_command']),
                             'Config')
            ticked = self.robot_configs[robot_id].get('is_supported')
            QMessageBox.information(
                self, 'Driver ready',
                "%s's driver is set up. The API button in the toolbar runs:\n\n%s\n\n%s"
                % (robot_id, settings['driver_command'],
                   'Check the torque sign and limits before commanding it.' if ticked else
                   'The hardware node will not control %s until you tick "This robot has a '
                   'hardware driver" and Save: check the torque sign and limits on this tab '
                   'first.' % robot_id))
        elif code is None and not settings['compile_now']:
            QMessageBox.information(
                self, 'Library set up',
                'The build and driver commands are filled in. Build it with Compile driver; '
                'then check the driver command points at the program the build made.')
        elif code == 0:
            QMessageBox.warning(
                self, 'Driver program not found',
                'The build finished, but the driver program (%s) was not found. Set the '
                'driver command on the Hardware tab to the program it built.'
                % (program or 'none named'))
        else:
            QMessageBox.warning(
                self, 'The build did not finish',
                'The library is in robot_library and its commands are saved, but the build '
                '%s. Fix what its output shows, then use Compile driver on the Hardware tab.'
                % ('failed' if code not in (None, -1) else 'was stopped or closed'))

    # -------------------------------------------------------------------------
    # Overview actions
    # -------------------------------------------------------------------------

    def _rc_try_in_sim(self):
        """Start the simulator, then the FIC controller, for this robot."""
        robot_id = self._rc_current_id()
        if not robot_id:
            return
        if getattr(self, '_rc_dirty', False):
            if QMessageBox.question(
                    self, 'Save first?',
                    'Save your changes to %s and simulate it?' % robot_id,
                    QMessageBox.Save | QMessageBox.Cancel) != QMessageBox.Save:
                return
            self._rc_save()
            if self._rc_dirty:
                return
        if getattr(self, 'simulation_running', False) or getattr(self, 'fic_running', False):
            QMessageBox.information(
                self, 'Already running',
                'A simulation or controller is already running. Stop it with the SIM '
                'and FIC buttons in the toolbar, then try again.')
            return
        dropdown = getattr(self, 'robot_dropdown', None)
        if dropdown is None or not hasattr(self, '_start_simulation'):
            return
        if dropdown.findText(robot_id) < 0:
            self._rc_refresh_robot_lists()
        dropdown.setCurrentIndex(dropdown.findText(robot_id))
        self._start_simulation()
        if hasattr(self, '_update_status'):
            self._update_status()
        if getattr(self, 'simulation_running', False) and hasattr(self, '_start_fic_simulation'):
            # The controller subscribes to the simulator's topics; give it a
            # moment to come up rather than racing it.
            QTimer.singleShot(2000, lambda: self._rc_start_controller_for(robot_id))

    def _rc_start_controller_for(self, robot_id):
        dropdown = getattr(self, 'robot_dropdown', None)
        if (getattr(self, 'simulation_running', False) and not getattr(self, 'fic_running', False)
                and dropdown is not None and dropdown.currentText() == robot_id):
            self._start_fic_simulation()
            if hasattr(self, '_update_status'):
                self._update_status()

    def _rc_open_viewer(self):
        """Open the robot's MuJoCo scene in MuJoCo's own viewer."""
        import shutil
        import subprocess
        import sys
        cfg = self.robot_configs.get(self._rc_current_id(), {})
        xml = rc_check.resolve(os.path.join(_FIC_WS, 'src'), cfg.get('mujoco_xml_path', ''))
        if not xml or not os.path.isfile(xml):
            QMessageBox.warning(self, 'No MuJoCo model', 'This robot has no MuJoCo model yet.')
            return
        python = sys.executable
        if sys.platform == 'darwin' and shutil.which('mjpython'):
            python = shutil.which('mjpython')   # the viewer needs it on macOS
        try:
            subprocess.Popen([python, '-m', 'mujoco.viewer', '--mjcf=%s' % xml],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
        except OSError as exc:
            QMessageBox.warning(self, 'Cannot open the viewer', str(exc))
            return
        self.log_message('INFO', 'Opened %s in the MuJoCo viewer' % xml, 'Config')

    def _rc_open_folder(self):
        from PyQt5.QtCore import QUrl
        from PyQt5.QtGui import QDesktopServices
        folder = self._rc_model_dir()
        if not folder or not os.path.isdir(folder):
            QMessageBox.information(self, 'No model folder',
                                    'This robot has no model folder yet.')
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(folder))

    def _rc_show_report(self):
        from PyQt5.QtWidgets import QTextBrowser
        path = os.path.join(self._rc_model_dir(), 'import_report.md')
        try:
            with open(path) as fh:
                text = fh.read()
        except OSError as exc:
            QMessageBox.warning(self, 'No report', str(exc))
            return
        dlg = QDialog(self)
        dlg.setWindowTitle('Import report: %s' % self._rc_current_id())
        dlg.resize(760, 620)
        lay = QVBoxLayout(dlg)
        view = QTextBrowser()
        view.setOpenExternalLinks(True)
        view.setMarkdown(text)
        lay.addWidget(view)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dlg.reject)
        lay.addWidget(buttons)
        dlg.exec_()

    def _rc_target_from_home(self):
        """Set the default Cartesian target to the end effector's pose at home."""
        from fic_studio.robot_import import fk
        ws_src = os.path.join(_FIC_WS, 'src')
        urdf = rc_check.resolve(ws_src, self._rc_urdf_path.text().strip())
        xml = rc_check.resolve(ws_src, self._rc_mujoco_path.text().strip())
        names = [r['name'] for r in self._rc_joint_table.rows()]
        q = [math.radians(mf.value()) for mf in self._rc_home_fields]
        frame = self._rc_ee_frame.text().strip()
        try:
            pos, quat = fk.ee_pose(frame, names, q,
                                   urdf=urdf if urdf and os.path.isfile(urdf) else None,
                                   xml=xml if xml and os.path.isfile(xml) else None)
        except fk.FkError as exc:
            QMessageBox.warning(self, 'Cannot compute the pose', str(exc))
            return
        for mf, v in zip((self._rc_ee_pos_x, self._rc_ee_pos_y, self._rc_ee_pos_z), pos):
            mf.set_value(v)
        for sb, v in zip((self._rc_ee_qx, self._rc_ee_qy, self._rc_ee_qz, self._rc_ee_qw),
                         quat):
            sb.setValue(v)
        self._rc_mark_dirty()
        self.log_message('INFO', 'Default target set from the home pose: %s %s'
                         % (pos, quat), 'Config')

    def _refresh_robot_list(self):
        """Refresh the robot list widget."""
        if not hasattr(self, 'robot_list_widget'):
            return
        
        self.robot_list_widget.clear()
        for robot_id, config in self.robot_configs.items():
            display_name = config.get('display_name', robot_id)
            num_joints = config.get('num_joints', '?')
            supported = "✓" if config.get('is_supported', False) else "○"
            has_build = "(build)" if config.get('build_command', '') else ""
            item = QListWidgetItem(f"{supported}  {robot_id}  ({display_name}, {num_joints} joints) {has_build}")
            self.robot_list_widget.addItem(item)
    
    def _on_robot_list_selection_changed(self, row):
        """Update details when robot selection changes."""
        if row < 0:
            return
        
        robot_id = list(self.robot_configs.keys())[row]
        config = self.robot_configs[robot_id]
        
        for key, label in self.detail_labels.items():
            value = config.get(key, '-')
            if isinstance(value, bool):
                value = "Yes" if value else "No"
            elif isinstance(value, list):
                if len(value) > 4:
                    value = ', '.join([str(v) for v in value[:3]]) + f"... ({len(value)} total)"
                else:
                    value = ', '.join([str(v) for v in value])
            elif not value:
                value = '-'
            label.setText(str(value))
    
    def _add_new_robot(self):
        """Open dialog to add a new robot."""
        dialog = RobotEditorDialog(self, "", None, is_new=True)
        if dialog.exec_() == QDialog.Accepted and dialog.result_name:
            if dialog.result_name in self.robot_configs:
                QMessageBox.warning(self, "Duplicate", f"Robot '{dialog.result_name}' already exists.")
                return
            
            self.robot_configs[dialog.result_name] = dialog.result_config
            self._save_robot_configs()
            self._refresh_robot_list()
            self._refresh_robots_dropdown()
            self.log_message("INFO", f"Added new robot: {dialog.result_name}", "Config")
    
    def _edit_selected_robot(self):
        """Edit the selected robot configuration."""
        row = self.robot_list_widget.currentRow()
        if row < 0:
            QMessageBox.information(self, "No Selection", "Please select a robot to edit.")
            return
        
        robot_id = list(self.robot_configs.keys())[row]
        config = self.robot_configs[robot_id]
        
        dialog = RobotEditorDialog(self, robot_id, config, is_new=False)
        if dialog.exec_() == QDialog.Accepted:
            # Merge over the old config so keys the dialog doesn't expose
            # (e.g. kinematics_solver) survive an edit round-trip.
            self.robot_configs[robot_id] = {**config, **dialog.result_config}
            self._save_robot_configs()
            self._refresh_robot_list()
            self._refresh_robots_dropdown()
            self.log_message("INFO", f"Updated robot: {robot_id}", "Config")
    
    def _duplicate_selected_robot(self):
        """Duplicate the selected robot with a new name."""
        row = self.robot_list_widget.currentRow()
        if row < 0:
            QMessageBox.information(self, "No Selection", "Please select a robot to duplicate.")
            return
        
        robot_id = list(self.robot_configs.keys())[row]
        config = self.robot_configs[robot_id].copy()
        
        # Generate new name
        new_id = f"{robot_id}_copy"
        i = 1
        while new_id in self.robot_configs:
            new_id = f"{robot_id}_copy{i}"
            i += 1
        
        config['display_name'] = config.get('display_name', robot_id) + " (Copy)"
        
        dialog = RobotEditorDialog(self, new_id, config, is_new=True)
        if dialog.exec_() == QDialog.Accepted and dialog.result_name:
            # Merge over the duplicated source config so keys the dialog
            # doesn't expose (e.g. kinematics_solver) carry over to the copy.
            self.robot_configs[dialog.result_name] = {**config, **dialog.result_config}
            self._save_robot_configs()
            self._refresh_robot_list()
            self._refresh_robots_dropdown()
            self.log_message("INFO", f"Duplicated robot as: {dialog.result_name}", "Config")
    
    def _delete_selected_robot(self):
        """Delete the selected robot configuration."""
        row = self.robot_list_widget.currentRow()
        if row < 0:
            QMessageBox.information(self, "No Selection", "Please select a robot to delete.")
            return
        
        robot_id = list(self.robot_configs.keys())[row]
        
        reply = QMessageBox.question(
            self, "Confirm Delete",
            f"Are you sure you want to delete '{robot_id}'?\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            del self.robot_configs[robot_id]
            self._save_robot_configs()
            self._refresh_robot_list()
            self._refresh_robots_dropdown()
            self.log_message("WARN", f"Deleted robot: {robot_id}", "Config")
    
    def _compile_selected_robot_driver(self):
        """Compile the selected robot's driver using its build config."""
        row = self.robot_list_widget.currentRow()
        if row < 0:
            QMessageBox.information(self, "No Selection", "Please select a robot to compile.")
            return
        
        robot_id = list(self.robot_configs.keys())[row]
        
        # Delegate to ProcessMixin._compile_robot_api()
        if hasattr(self, '_compile_robot_api'):
            self._compile_robot_api(robot_id)
        else:
            QMessageBox.warning(self, "Not Available", "Compile function not available.")
    
    def _import_robot_config(self):
        """Import robot configuration from a YAML file."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Import Robot Configuration",
            os.path.expanduser("~"),
            "YAML Files (*.yaml *.yml)"
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    data = yaml.safe_load(f)
                
                imported = data.get('robots', data)
                count = 0
                for robot_id, config in imported.items():
                    if robot_id not in self.robot_configs:
                        self.robot_configs[robot_id] = config
                        count += 1
                    else:
                        reply = QMessageBox.question(
                            self, "Robot Exists",
                            f"Robot '{robot_id}' already exists. Overwrite?",
                            QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel
                        )
                        if reply == QMessageBox.Yes:
                            self.robot_configs[robot_id] = config
                            count += 1
                        elif reply == QMessageBox.Cancel:
                            break
                
                self._rc_persist()
                if hasattr(self, '_rc_robot_list'):
                    self._rc_rebuild_list()
                    if self._rc_current_id() in self.robot_configs:
                        self._rc_load_robot(self._rc_current_id())
                self.log_message("INFO", 'Imported %d robot(s)' % count, 'Config')
                QMessageBox.information(
                    self, 'Import Complete',
                    'Imported %d robot configuration(s).' % count,
                )
            except Exception as e:
                QMessageBox.critical(self, "Import Error", f"Failed to import: {e}")
    
    def _export_robot_config(self):
        """Export robot configurations to a YAML file."""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Robot Configuration",
            os.path.expanduser("~/robot_configs.yaml"),
            "YAML Files (*.yaml *.yml)"
        )
        if filename:
            try:
                data = {
                    'version': '1.0',
                    'robots': self.robot_configs
                }
                with open(filename, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
                
                self.log_message("INFO", f"Exported to {filename}", "Config")
                QMessageBox.information(self, "Export Complete", f"Exported to:\n{filename}")
            except Exception as e:
                QMessageBox.critical(self, "Export Error", f"Failed to export: {e}")
    
    def _sync_to_fic_config(self, show_result=False):
        """Write the controllers' copy of the robots (robots_config.yaml + config.xml).

        Called by every save, quietly; ``show_result`` reports what was done.
        Returns False, after telling the user, when the write failed. The
        C++ nodes read these files when they start, so nothing needs
        rebuilding.
        """
        # Resolve via config.py (robust for source and copy-install layouts).
        from fic_studio.config import FIC_CONFIG_DIR
        config_path = FIC_CONFIG_DIR

        if not os.path.exists(config_path):
            QMessageBox.warning(
                self, "Path Not Found",
                f"Config directory not found:\n{config_path}\n\n"
                "The controllers' copy of the robot list was not updated.")
            return False

        robot_yaml_path = os.path.join(config_path, "robots_config.yaml")
        config_xml_path = os.path.join(config_path, "config.xml")

        try:
            header = {'version': '1.0', 'description': 'Auto-generated by Robot Selector GUI'}
            existing = {}
            if os.path.exists(robot_yaml_path):
                with open(robot_yaml_path) as f:
                    old = yaml.safe_load(f) or {}
                existing = old.get('robots') or {}
                header.update({k: v for k, v in old.items() if k != 'robots'})
            config_data = dict(header)
            config_data['robots'] = controller_robots(self.robot_configs, existing)
            write_yaml_atomically(robot_yaml_path, config_data)

            # New robots get default tuning parameters in config.xml.
            xml_result = sync_robots_to_config_xml(config_xml_path, self.robot_configs)
        except Exception as e:
            self.log_message("ERROR", f"Could not update the controllers' robot list: {e}",
                             "Config")
            QMessageBox.critical(self, "Sync Error",
                                 f"Saved in the Studio, but the controllers' copy "
                                 f"({robot_yaml_path}) could not be updated:\n{e}")
            return False

        added = xml_result['added']
        self.log_message("INFO", f"Synced {len(self.robot_configs)} robot(s) to FIC config",
                         "Config")
        if added:
            self.log_message("INFO", f"Added {len(added)} robot(s) to config.xml with default "
                             f"tuning: {added}", "Config")
        if show_result:
            msg = [f"Robot configurations synced to:\n{robot_yaml_path}\n",
                   f"Synced {len(self.robot_configs)} robot(s)."]
            if added:
                msg.append("\nAdded to config.xml with default tuning:")
                msg.extend(f"  • {name}" for name in added)
            QMessageBox.information(self, "Sync Complete", "\n".join(msg))
        return True

    def _open_robots_yaml_viewer(self):
        """Open the raw robots_config.yaml viewer for the currently selected robot."""
        from fic_studio.mixins.config_viewer_dialog import ConfigViewerDialog

        fic_interface_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')
        )
        yaml_path = os.path.join(fic_interface_dir, 'fic_library', 'config', 'robots_config.yaml')

        # Prefer the robot on the config page; fall back to the toolbar's.
        if self._rc_current_id():
            robot_name = self._rc_current_id()
        elif hasattr(self, 'robot_dropdown'):
            robot_name = self.robot_dropdown.currentText()
        else:
            QMessageBox.warning(self, 'No Robot', 'Select a robot first.')
            return

        dlg = ConfigViewerDialog(
            parent=self,
            file_path=yaml_path,
            robot_name=robot_name,
            file_type='yaml',
        )
        dlg.exec_()

    def show_robot_config_panel(self):
        """Show the robot configuration panel."""
        self._set_active_sidebar_btn(self._side_buttons[10])
        self._show_page('robot_config')