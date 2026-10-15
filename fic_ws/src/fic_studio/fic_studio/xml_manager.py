"""
xml_manager.py - Mode-Separated FIC Parameter Management (v3.1)

Each control mode (1-8) has its own COMPLETE set of parameters stored in
separate <modeN> blocks. This allows independent tuning without interference.

Control Modes:
- Mode 1: Joint Impedance (NO damping)
- Mode 2: Joint Impedance With Planner (HAS damping_ratio)
- Mode 3: Joint Admittance (HAS Kv damping)
- Mode 4: Joint Admittance Haptic
- Mode 5: Cartesian Impedance (NO damping)
- Mode 6: Cartesian Planner (HAS damping_ratio)
- Mode 7: Cartesian Admittance (HAS Kv damping)
- Mode 8: Cartesian Admittance Haptic

v3.1 changes:
- Added ensure_robot() - auto-creates a full default robot entry if missing
- Added ensure_all_robots() - batch version for startup use
- load_joint_params / save_joint_params / save_cartesian_axis now use
  ensure_robot() instead of raising ValueError on missing robots
- Fixed load_cartesian_params / load_cartesian_axis to use the correct
  internal parsing path (_parse_cartesian_axis) instead of the old
  broken stub
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from pathlib import Path
import io
import logging
import os
import tempfile

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════════════════
#  CONTROL MODE DEFINITIONS
# ═══════════════════════════════════════════════════════════════════════════════

CONTROL_MODES = {
    1: {"name": "Joint Impedance",               "type": "joint",     "has_planner": False, "has_admittance": False, "has_damping": False},
    2: {"name": "Joint Impedance with Planner",  "type": "joint",     "has_planner": True,  "has_admittance": False, "has_damping": True},
    3: {"name": "Joint Admittance",              "type": "joint",     "has_planner": False, "has_admittance": True,  "has_damping": True},
    4: {"name": "Joint HAP-FIC",                 "type": "joint",     "has_planner": False, "has_admittance": True,  "has_damping": True},
    5: {"name": "Cartesian Impedance",           "type": "cartesian", "has_planner": False, "has_admittance": False, "has_damping": False},
    6: {"name": "Cartesian Impedance with Planner", "type": "cartesian", "has_planner": True,  "has_admittance": False, "has_damping": True},
    7: {"name": "Cartesian Admittance",          "type": "cartesian", "has_planner": True,  "has_admittance": True,  "has_damping": True},
    8: {"name": "Cartesian HAP-FIC",             "type": "cartesian", "has_planner": True,  "has_admittance": True,  "has_damping": True},
}

#: H-PMP's own tuning block, <mode id="100" name="H-PMP">: its joint layer in <joint> (FIC
#: controller, planner and admittance per joint) and its Cartesian layer in <cartesian> (FIC
#: controller and planner per axis). H-PMP is switched on in modes 5-8, never selected, so 100
#: is not in CONTROL_MODES. A robot with no H-PMP block runs H-PMP on mode 4's joints and mode
#: 6's Cartesian block, as the controller's loader does (fic_param_loader.hpp, loadTuningSet).
HPMP_MODE = 100
HPMP_FALLBACK_JOINT_MODE = 4
HPMP_FALLBACK_CART_MODE = 6
HPMP_INFO = {"name": "H-PMP", "type": "hpmp", "has_planner": True, "has_admittance": True, "has_damping": True}


def get_mode_info(mode: int) -> dict:
    if mode == HPMP_MODE:
        return HPMP_INFO
    return CONTROL_MODES.get(mode, CONTROL_MODES[1])

def is_hpmp_mode(mode: int) -> bool:
    return mode == HPMP_MODE

def mode_name(mode: int) -> str:
    return get_mode_info(mode)["name"]

def is_joint_mode(mode: int) -> bool:
    return get_mode_info(mode)["type"] == "joint"

def is_cartesian_mode(mode: int) -> bool:
    return get_mode_info(mode)["type"] == "cartesian"

def mode_has_damping(mode: int) -> bool:
    return get_mode_info(mode)["has_damping"]

def mode_uses_planner(mode: int) -> bool:
    return get_mode_info(mode)["has_planner"]

def mode_uses_admittance(mode: int) -> bool:
    return get_mode_info(mode)["has_admittance"]


def write_atomic(path: str, data) -> None:
    """Replace the file at `path` with `data` (str or bytes) in one step.

    It is written beside the file and renamed over it, so a reader sees the old file or the new
    one, never half of one. The controllers read config.xml itself (through the installed link,
    on macOS and with --symlink-install), and a half-written file would load as the loader's
    defaults. A symlinked `path` replaces the file it points to; the permissions are kept.
    """
    path = os.path.realpath(path)
    folder = os.path.dirname(path)
    mode = (os.stat(path).st_mode & 0o7777) if os.path.exists(path) else 0o644
    fd, tmp = tempfile.mkstemp(dir=folder, prefix="." + os.path.basename(path) + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data.encode("utf-8") if isinstance(data, str) else data)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, mode)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def write_tree_atomic(tree: ET.ElementTree, path: str, encoding: str = "utf-8") -> None:
    """ElementTree.write (with the XML declaration) through write_atomic."""
    buf = io.StringIO() if encoding == "unicode" else io.BytesIO()
    tree.write(buf, encoding=encoding, xml_declaration=True)
    write_atomic(path, buf.getvalue())


def find_mode_element(robot: ET.Element, mode: int, profile: str = "default") -> Optional[ET.Element]:
    """Find <mode id="N" profile="P"> under a robot, falling back to the default profile's block
    for that mode (as the controller's findModeElement does)."""
    fallback = None
    for m in robot.findall("mode"):
        try:
            if int(m.get("id", -1)) != mode:
                continue
        except ValueError:
            continue
        p = m.get("profile", "default") or "default"
        if p == profile:
            return m
        if p == "default":
            fallback = m
    return fallback


def build_hpmp_block(robot: ET.Element, profile: str = "default",
                       from_layers: bool = False) -> Optional[ET.Element]:
    """A whole <mode id="100" name="H-PMP" profile=...> with what H-PMP runs on for `profile`.

    That is the H-PMP block it falls back to (the default profile's), or, with none or with
    `from_layers`, mode 4's joints and mode 6's Cartesian block (each the profile's own or the
    default's). None if the robot has none of them. Always whole: the controller reads every joint
    and axis from the block, so one holding a single joint would leave the rest on the loader's
    defaults.
    """
    import copy
    src = None if from_layers else find_mode_element(robot, HPMP_MODE, profile)
    if src is not None:
        block = copy.deepcopy(src)
    else:
        joints = find_mode_element(robot, HPMP_FALLBACK_JOINT_MODE, profile)
        cart = find_mode_element(robot, HPMP_FALLBACK_CART_MODE, profile)
        cart = cart.find("cartesian") if cart is not None else None
        if joints is None and cart is None:
            return None
        block = ET.Element("mode")
        for j in (joints.findall("joint") if joints is not None else []):
            block.append(copy.deepcopy(j))
        if cart is not None:
            block.append(copy.deepcopy(cart))
    block.attrib.clear()
    block.set("id", str(HPMP_MODE))
    block.set("name", HPMP_INFO["name"])
    block.set("profile", profile)
    return block


# ═══════════════════════════════════════════════════════════════════════════════
#  DATA CLASSES
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class JointParams:
    """Complete parameters for a single joint in a specific mode."""
    # Controller params
    stiffness: float = 600.0
    diff_effort_max: float = 100.0
    error_max: List[float] = field(default_factory=lambda: [0.05, 0.1])
    force_max: List[float] = field(default_factory=lambda: [5.0, 39.0])
    saturation_ratios: List[float] = field(default_factory=lambda: [0.9])
    n_saturations: int = 2
    asymmetric: bool = False
    n_neg_sat: int = 1
    velocity: float = 1.0

    # Planner params (Mode 2 only)
    planner_stiffness: float = 15780.0
    planner_vel_desired: float = 0.4
    planner_acc_max: float = 5000.0
    planner_damping_ratio: float = 0.005
    planner_frequency: float = 150.0
    planner_wrapped: int = 0
    planner_pos_lower: float = -3.14159
    planner_pos_upper: float = 3.14159

    # Admittance params (Modes 3-4)
    admittance_inv_m: float = 0.000001
    admittance_damping: float = 9.0
    admittance_dflow: List[float] = field(default_factory=lambda: [-5.2, 5.2])
    admittance_flow: List[float] = field(default_factory=lambda: [-1.2, 1.2])
    admittance_pose: List[float] = field(default_factory=lambda: [-0.05, 0.05])


@dataclass
class CartesianAxisParams:
    """Complete parameters for a Cartesian axis in a specific mode."""
    # Controller params
    stiffness: float = 50.0
    diff_effort_max: float = 70.0
    error_max: List[float] = field(default_factory=lambda: [0.01, 0.05])
    force_max: List[float] = field(default_factory=lambda: [10.0, 25.0])
    saturation_ratios: List[float] = field(default_factory=lambda: [0.9])
    n_saturations: int = 2
    asymmetric: bool = False
    n_neg_sat: int = 1

    # Planner params (Mode 6)
    planner_stiffness_p: float = 15753.0
    planner_vel_desired: float = 0.1
    planner_acc_max: float = 5000.0
    planner_damping_ratio: float = 0.015
    planner_frequency: float = 150.0

    # Admittance params (Modes 7-8)
    admittance_inv_m: float = 1.0
    admittance_damping: float = 20.0
    admittance_dflow: List[float] = field(default_factory=lambda: [-5.2, 5.2])
    admittance_flow: List[float] = field(default_factory=lambda: [-0.5, 0.5])
    admittance_pose: List[float] = field(default_factory=lambda: [-0.3, 0.3])


@dataclass
class CartesianParams:
    """All Cartesian axis parameters for a mode."""
    x: CartesianAxisParams = field(default_factory=CartesianAxisParams)
    y: CartesianAxisParams = field(default_factory=CartesianAxisParams)
    z: CartesianAxisParams = field(default_factory=CartesianAxisParams)
    q: CartesianAxisParams = field(default_factory=CartesianAxisParams)


@dataclass
class PosturalParams:
    """Null-space postural task parameters. Robot-scoped (not per mode, not per
    profile): a single <postural> block under <robot> is shared by every
    Cartesian mode that invokes the postural controller."""
    stiffness: float = 1.0
    diff_effort_max: float = 10.0
    error_max: List[float] = field(default_factory=lambda: [0.001, 1.5])
    force_max: List[float] = field(default_factory=lambda: [5.0, 10.2])
    saturation_ratios: List[float] = field(default_factory=lambda: [0.9])
    n_saturations: int = 2
    asymmetric: bool = False
    n_neg_sat: int = 1
    velocity: float = 0.5

    # Planner block is written for completeness so existing configs round-trip,
    # but the runtime postural path only consumes controller params.
    planner_stiffness: float = 15753.0
    planner_vel_desired: float = 0.5
    planner_acc_max: float = 5000.0
    planner_damping_ratio: float = 0.05
    planner_frequency: float = 150.0


@dataclass
class ElbowParams:
    """Null-space elbow task parameters. Robot-scoped (not per mode, not per
    profile): a single <elbow> block under <robot> is shared by every
    Cartesian mode that invokes the elbow controller (channels 6-8)."""
    stiffness: float = 100.0
    diff_effort_max: float = 10.0
    error_max: List[float] = field(default_factory=lambda: [0.05, 0.1])
    force_max: List[float] = field(default_factory=lambda: [10.0, 20.0])
    saturation_ratios: List[float] = field(default_factory=lambda: [0.9])
    n_saturations: int = 2
    asymmetric: bool = False
    n_neg_sat: int = 1


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIG PATH FINDER
# ═══════════════════════════════════════════════════════════════════════════════

def find_config_path() -> str:
    """Find the config.xml file."""
    # Resolve via config.py, which locates the repo root robustly (works whether
    # fic_studio is run from the source tree or a colcon copy-install tree — a
    # fixed ../../../.. walk from __file__ breaks in the latter on macOS).
    try:
        from fic_studio.config import XML_FILE_PATH
        primary = str(XML_FILE_PATH)
    except Exception:
        primary = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "..",
            "fic_library", "config", "config.xml"))

    candidates = [
        primary,
        os.path.join(os.path.dirname(__file__), "..", "resources", "config.xml"),
        os.path.join(os.path.dirname(__file__), "resources", "config.xml"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]

# ═══════════════════════════════════════════════════════════════════════════════
#  DEFAULT CARTESIAN AXIS PARAMS PER AXIS
# ═══════════════════════════════════════════════════════════════════════════════

def _default_cartesian_axis(axis: str, mode: int) -> CartesianAxisParams:
    """
    Return sensible defaults for a Cartesian axis.
    q axis gets tighter limits to match existing robot configs.
    """
    params = CartesianAxisParams()
    if axis == "q":
        params.stiffness = 3.0 if mode == 5 else 5.0
        params.diff_effort_max = 10.0
        params.error_max = [0.05, 0.2]
        params.force_max = [1.0, 5.0]
        params.saturation_ratios = [0.9]
        params.planner_vel_desired = 0.3
        params.admittance_damping = 15.0
        params.admittance_flow = [-1.0, 1.0]
        params.admittance_pose = [-0.5, 0.5]
    return params


# ═══════════════════════════════════════════════════════════════════════════════
#  XML MANAGER CLASS
# ═══════════════════════════════════════════════════════════════════════════════

class XMLManager:
    """
    Manager for loading and saving mode-specific FIC parameters.

    Each control mode (1-8) has its own complete parameter set in <modeN> blocks.
    Missing robots are automatically created with safe default values.
    """

    def __init__(self, config_path: Optional[str] = None):
        self.config_path = Path(config_path) if config_path else Path(find_config_path())
        self._tree: Optional[ET.ElementTree] = None
        self._root: Optional[ET.Element] = None
        self._load()

    def _load(self):
        """Load the XML file."""
        if not self.config_path.exists():
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        self._tree = ET.parse(self.config_path)
        self._root = self._tree.getroot()

    def reload(self):
        """Reload config from disk."""
        self._load()

    # ─────────────────────────────────────────────────────────────────────────
    #  INTERNAL HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _find_robot(self, robot_name: str) -> Optional[ET.Element]:
        for robot in self._root.findall("robot"):
            if robot.get("name") == robot_name:
                return robot
        return None

    def _find_mode_element(self, robot: ET.Element, mode: int,
                            profile: str = "default") -> Optional[ET.Element]:
        """Find <mode id="N" profile="P">. Falls back to the default profile if
        the requested one does not exist for this mode."""
        return find_mode_element(robot, mode, profile)

    def ensure_hpmp_block(self, robot_name: str, profile: str = "default") -> bool:
        """Give `profile` a H-PMP block of its own, copied from what H-PMP runs on for it now
        (build_hpmp_block), unless it has one. Returns whether one was made."""
        robot = self.ensure_robot(robot_name)
        for m in robot.findall("mode"):
            if m.get("id") == str(HPMP_MODE) and (m.get("profile", "default") or "default") == profile:
                return False
        block = build_hpmp_block(robot, profile)
        if block is None:
            return False
        modes = [i for i, c in enumerate(robot) if c.tag == "mode"]
        robot.insert(modes[-1] + 1 if modes else len(robot), block)
        return True

    def _create_mode_element(self, robot: ET.Element, mode: int,
                              profile: str = "default") -> ET.Element:
        """Create a new <mode id=N name=... profile=P> sibling under robot."""
        mode_el = ET.SubElement(robot, "mode")
        mode_el.set("id", str(mode))
        mode_el.set("name", mode_name(mode))
        mode_el.set("profile", profile)
        return mode_el

    def _find_joint_in_mode(self, mode_el: ET.Element, joint_id: int) -> Optional[ET.Element]:
        for joint in mode_el.findall("joint"):
            if int(joint.get("id", -1)) == joint_id:
                return joint
        return None

    def _parse_list(self, text: str) -> List[float]:
        if not text or not text.strip():
            return []
        return [float(x.strip()) for x in text.split(",") if x.strip()]

    def _format_list(self, values: List[float]) -> str:
        return ",".join(f"{v:.6g}" for v in values)

    def _get_text(self, element: ET.Element, tag: str, default: str = "") -> str:
        child = element.find(tag)
        return child.text.strip() if child is not None and child.text else default

    def _get_float(self, element: ET.Element, tag: str, default: float = 0.0) -> float:
        text = self._get_text(element, tag)
        return float(text) if text else default

    def _get_int(self, element: ET.Element, tag: str, default: int = 0) -> int:
        text = self._get_text(element, tag)
        return int(float(text)) if text else default

    def _set_element_text(self, parent: ET.Element, tag: str, text: str):
        child = parent.find(tag)
        if child is None:
            child = ET.SubElement(parent, tag)
        child.text = text

    def _controller_read(self, parent: ET.Element) -> ET.Element:
        """Return the <controller> sub-element holding the scalar/list control
        params. Falls back to `parent` when the XML predates the controller
        nest so legacy configs still load."""
        ctrl = parent.find("controller")
        return ctrl if ctrl is not None else parent

    def _controller_write(self, parent: ET.Element) -> ET.Element:
        """Return the <controller> sub-element for writing, creating it as the
        first child so planner/admittance remain after it."""
        ctrl = parent.find("controller")
        if ctrl is None:
            ctrl = ET.Element("controller")
            parent.insert(0, ctrl)
        return ctrl

    # ─────────────────────────────────────────────────────────────────────────
    #  AUTO-CREATE MISSING ROBOTS  (v3.1)
    # ─────────────────────────────────────────────────────────────────────────

    def ensure_robot(self, robot_name: str, num_joints: int = 7) -> ET.Element:
        """
        Get or create a robot element with full default parameters for all 8 modes.

        If the robot already exists in config.xml it is returned unchanged.
        If it is missing, a complete default entry is written and saved to disk
        immediately so the file stays consistent even if the process crashes later.

        Args:
            robot_name:  Name that will appear in the <robot name="..."> attribute.
            num_joints:  Number of joints (default 7 for 7-DOF arms).

        Returns:
            The existing or newly created <robot> Element.
        """
        robot = self._find_robot(robot_name)
        if robot is not None:
            return robot

        logger.info(f"Robot '{robot_name}' not found in config — creating with defaults "
                    f"({num_joints} joints).")

        robot = ET.SubElement(self._root, "robot")
        robot.set("name", robot_name)

        # ── Joint modes 1-4 (default profile) ─────────────────────────────
        for mode in range(1, 5):
            mode_el = self._create_mode_element(robot, mode, "default")
            for j in range(num_joints):
                joint_el = ET.SubElement(mode_el, "joint")
                joint_el.set("id", str(j))
                self._write_joint_params(joint_el, JointParams(), mode)

        # ── Cartesian modes 5-8 (default profile) ─────────────────────────
        for mode in range(5, 9):
            mode_el = self._create_mode_element(robot, mode, "default")
            cart_el = ET.SubElement(mode_el, "cartesian")
            for axis in ("x", "y", "z", "q"):
                axis_el = ET.SubElement(cart_el, axis)
                self._write_cartesian_axis(axis_el, _default_cartesian_axis(axis, mode), mode)

        # ── Postural ──────────────────────────────────────────────────────
        postural = ET.SubElement(robot, "postural")
        ctrl = self._controller_write(postural)
        self._set_element_text(ctrl, "stiffness",         "1.0")
        self._set_element_text(ctrl, "diff_effort_max",   "10.0")
        self._set_element_text(ctrl, "error_max",         "0.001,1.5")
        self._set_element_text(ctrl, "force_max",         "5.0,10.2")
        self._set_element_text(ctrl, "saturation_ratios", "0.9")
        self._set_element_text(ctrl, "velocity",          "0.5")
        self._set_element_text(ctrl, "n_saturations",     "2")
        self._set_element_text(ctrl, "asymmetric",        "false")
        self._set_element_text(ctrl, "n_neg_sat",         "1")
        pl = ET.SubElement(postural, "planner")
        self._set_element_text(pl, "stiffness",    "15753.0")
        self._set_element_text(pl, "vel_desired",  "0.5")
        self._set_element_text(pl, "acc_max",      "5000.0")
        self._set_element_text(pl, "damping_ratio","0.05")
        self._set_element_text(pl, "frequency",    "150.0")

        # ── Elbow ─────────────────────────────────────────────────────────
        elbow_el = ET.SubElement(robot, "elbow")
        ctrl = self._controller_write(elbow_el)
        self._set_element_text(ctrl, "stiffness",         "100.0")
        self._set_element_text(ctrl, "diff_effort_max",   "10.0")
        self._set_element_text(ctrl, "error_max",         "0.05,0.1")
        self._set_element_text(ctrl, "force_max",         "10.0,20.0")
        self._set_element_text(ctrl, "saturation_ratios", "0.9")
        self._set_element_text(ctrl, "n_saturations",     "2")
        self._set_element_text(ctrl, "asymmetric",        "false")
        self._set_element_text(ctrl, "n_neg_sat",         "1")

        # Persist immediately
        self.save()
        logger.info(f"Robot '{robot_name}' created and saved to {self.config_path}.")
        return robot

    def ensure_all_robots(self, robot_map: Dict[str, int]):
        """
        Ensure every robot in robot_map exists in the config.

        Call once at application startup with self.robots  e.g.
            xml_mgr.ensure_all_robots({"UR5e": 6, "XArm7": 7})

        Robots that already exist are left completely untouched.
        Missing robots are auto-created with default values and saved.

        Args:
            robot_map: dict of {robot_name: num_joints}
        """
        for robot_name, num_joints in robot_map.items():
            self.ensure_robot(robot_name, num_joints)

    # ─────────────────────────────────────────────────────────────────────────
    #  LOADING - JOINT PARAMETERS
    # ─────────────────────────────────────────────────────────────────────────

    def load_joint_params(self, robot_name: str, joint_id: int, mode: int,
                          profile: str = "default") -> JointParams:
        """Load joint parameters. Auto-creates the robot entry if missing.
        Falls back to the default profile if the requested one does not exist."""
        if not (is_joint_mode(mode) or is_hpmp_mode(mode)):
            raise ValueError(f"Mode {mode} is not a joint mode (must be 1-4, or H-PMP's {HPMP_MODE})")

        robot = self.ensure_robot(robot_name)
        if is_hpmp_mode(mode) and self._find_mode_element(robot, mode, profile) is None:
            mode = HPMP_FALLBACK_JOINT_MODE          # no H-PMP block: it runs on mode 4's joints

        mode_el = self._find_mode_element(robot, mode, profile)
        if mode_el is None:
            logger.warning(f"Mode {mode}/{profile} not found for '{robot_name}', returning defaults")
            return JointParams()

        joint_el = self._find_joint_in_mode(mode_el, joint_id)
        if joint_el is None:
            logger.warning(f"Joint {joint_id} not found in mode {mode}/{profile} for '{robot_name}', "
                           f"returning defaults")
            return JointParams()

        return self._parse_joint_element(joint_el)

    def _parse_joint_element(self, joint_el: ET.Element) -> JointParams:
        params = JointParams()
        ctrl = self._controller_read(joint_el)

        params.stiffness      = self._get_float(ctrl, "stiffness",      params.stiffness)
        params.diff_effort_max= self._get_float(ctrl, "diff_effort_max", params.diff_effort_max)
        params.velocity       = self._get_float(ctrl, "velocity",        params.velocity)

        if (text := self._get_text(ctrl, "error_max")):
            params.error_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "force_max")):
            params.force_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "saturation_ratios")):
            params.saturation_ratios = self._parse_list(text)

        params.n_saturations = self._get_int(ctrl, "n_saturations", params.n_saturations)
        if (text := self._get_text(ctrl, "asymmetric")):
            params.asymmetric = text.strip().lower() in ("true", "1", "yes")
        params.n_neg_sat = self._get_int(ctrl, "n_neg_sat", params.n_neg_sat)

        planner = joint_el.find("planner")
        if planner is not None:
            params.planner_stiffness    = self._get_float(planner, "stiffness",    params.planner_stiffness)
            params.planner_vel_desired  = self._get_float(planner, "vel_desired",  params.planner_vel_desired)
            params.planner_acc_max      = self._get_float(planner, "acc_max",      params.planner_acc_max)
            params.planner_damping_ratio= self._get_float(planner, "damping_ratio",params.planner_damping_ratio)
            params.planner_frequency    = self._get_float(planner, "frequency",    params.planner_frequency)
            params.planner_wrapped      = self._get_int(  planner, "wrapped",      params.planner_wrapped)
            pos_limits = planner.find("pos_limits")
            if pos_limits is not None:
                params.planner_pos_lower = float(pos_limits.get("lower", params.planner_pos_lower))
                params.planner_pos_upper = float(pos_limits.get("upper", params.planner_pos_upper))

        adm = joint_el.find("admittance")
        if adm is not None:
            params.admittance_inv_m  = self._get_float(adm, "InvM",    params.admittance_inv_m)
            params.admittance_damping= self._get_float(adm, "Damping", params.admittance_damping)
            if (text := self._get_text(adm, "dFlow")):
                params.admittance_dflow = self._parse_list(text)
            if (text := self._get_text(adm, "Flow")):
                params.admittance_flow = self._parse_list(text)
            if (text := self._get_text(adm, "Pose")):
                params.admittance_pose = self._parse_list(text)

        return params

    def load_all_joint_params(self, robot_name: str, num_joints: int, mode: int,
                              profile: str = "default") -> List[JointParams]:
        return [self.load_joint_params(robot_name, j, mode, profile) for j in range(num_joints)]

    # ─────────────────────────────────────────────────────────────────────────
    #  LOADING - CARTESIAN PARAMETERS
    # ─────────────────────────────────────────────────────────────────────────

    def load_cartesian_params(self, robot_name: str, mode: int,
                              profile: str = "default") -> CartesianParams:
        """
        Load all four Cartesian axes for a given mode and profile.
        Auto-creates the robot entry if missing.
        Returns a CartesianParams with defaults for any axis not found.
        """
        if not (is_cartesian_mode(mode) or is_hpmp_mode(mode)):
            raise ValueError(f"Mode {mode} is not a Cartesian mode (must be 5-8, or H-PMP's {HPMP_MODE})")

        robot = self.ensure_robot(robot_name)
        if is_hpmp_mode(mode) and self._find_mode_element(robot, mode, profile) is None:
            mode = HPMP_FALLBACK_CART_MODE           # no H-PMP block: it runs on mode 6's Cartesian block

        result = CartesianParams()

        mode_el = self._find_mode_element(robot, mode, profile)
        if mode_el is None:
            logger.warning(f"Mode {mode}/{profile} not found for '{robot_name}', returning defaults")
            return result

        cart_el = mode_el.find("cartesian")
        if cart_el is None:
            logger.warning(f"No <cartesian> block in mode {mode}/{profile} for '{robot_name}', "
                           f"returning defaults")
            return result

        for axis_name in ("x", "y", "z", "q"):
            axis_el = cart_el.find(axis_name)
            axis_params = _default_cartesian_axis(axis_name, mode)
            if axis_el is not None:
                self._parse_cartesian_axis(axis_el, axis_params)
            setattr(result, axis_name, axis_params)

        return result

    def load_cartesian_axis(self, robot_name: str, mode: int, axis: str,
                            profile: str = "default") -> CartesianAxisParams:
        """Load a single Cartesian axis. Auto-creates robot if missing."""
        params = self.load_cartesian_params(robot_name, mode, profile)
        return getattr(params, axis, _default_cartesian_axis(axis, mode))

    def _parse_cartesian_axis(self, axis_el: ET.Element, params: CartesianAxisParams):
        """Parse an <x>/<y>/<z>/<q> element into a CartesianAxisParams (in-place)."""
        ctrl = self._controller_read(axis_el)

        params.stiffness       = self._get_float(ctrl, "stiffness",       params.stiffness)
        params.diff_effort_max = self._get_float(ctrl, "diff_effort_max",  params.diff_effort_max)

        if (text := self._get_text(ctrl, "error_max")):
            params.error_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "force_max")):
            params.force_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "saturation_ratios")):
            params.saturation_ratios = self._parse_list(text)

        params.n_saturations = self._get_int(ctrl, "n_saturations", params.n_saturations)
        if (text := self._get_text(ctrl, "asymmetric")):
            params.asymmetric = text.strip().lower() in ("true", "1", "yes")
        params.n_neg_sat = self._get_int(ctrl, "n_neg_sat", params.n_neg_sat)

        planner = axis_el.find("planner")
        if planner is not None:
            params.planner_stiffness_p  = self._get_float(planner, "stiffness_P",  params.planner_stiffness_p)
            params.planner_vel_desired  = self._get_float(planner, "vel_desired",   params.planner_vel_desired)
            params.planner_acc_max      = self._get_float(planner, "acc_max",       params.planner_acc_max)
            params.planner_damping_ratio= self._get_float(planner, "damping_ratio", params.planner_damping_ratio)
            params.planner_frequency    = self._get_float(planner, "frequency",     params.planner_frequency)

        adm = axis_el.find("admittance")
        if adm is not None:
            params.admittance_inv_m  = self._get_float(adm, "InvM",    params.admittance_inv_m)
            params.admittance_damping= self._get_float(adm, "Damping", params.admittance_damping)
            if (text := self._get_text(adm, "dFlow")):
                params.admittance_dflow = self._parse_list(text)
            if (text := self._get_text(adm, "Flow")):
                params.admittance_flow = self._parse_list(text)
            if (text := self._get_text(adm, "Pose")):
                params.admittance_pose = self._parse_list(text)

    # ─────────────────────────────────────────────────────────────────────────
    #  SAVING - JOINT PARAMETERS
    # ─────────────────────────────────────────────────────────────────────────

    def save_joint_params(self, robot_name: str, joint_id: int, mode: int, params: JointParams,
                          profile: str = "default"):
        """Save joint parameters. Auto-creates the robot entry and the
        <mode id=N profile=P> block if either is missing."""
        if not (is_joint_mode(mode) or is_hpmp_mode(mode)):
            raise ValueError(f"Mode {mode} is not a joint mode")

        robot = self.ensure_robot(robot_name)
        if is_hpmp_mode(mode):
            self.ensure_hpmp_block(robot_name, profile)

        # Exact match (don't fall back to default — we want to write into the
        # requested profile, creating it if needed).
        mode_el = None
        for m in robot.findall("mode"):
            try:
                if int(m.get("id", -1)) == mode and (m.get("profile", "default") or "default") == profile:
                    mode_el = m
                    break
            except ValueError:
                continue
        if mode_el is None:
            mode_el = self._create_mode_element(robot, mode, profile)

        joint_el = self._find_joint_in_mode(mode_el, joint_id)
        if joint_el is None:
            joint_el = ET.SubElement(mode_el, "joint")
            joint_el.set("id", str(joint_id))

        self._write_joint_params(joint_el, params, mode)

    def _write_joint_params(self, joint_el: ET.Element, params: JointParams, mode: int):
        ctrl = self._controller_write(joint_el)
        self._set_element_text(ctrl, "stiffness",       f"{params.stiffness:.3f}")
        self._set_element_text(ctrl, "diff_effort_max", f"{params.diff_effort_max:.1f}")
        self._set_element_text(ctrl, "error_max",       self._format_list(params.error_max))
        self._set_element_text(ctrl, "force_max",       self._format_list(params.force_max))
        self._set_element_text(ctrl, "saturation_ratios", self._format_list(params.saturation_ratios))
        if mode_uses_planner(mode):
            self._set_element_text(ctrl, "velocity", f"{params.velocity:.4f}")
        self._set_element_text(ctrl, "n_saturations", str(params.n_saturations))
        self._set_element_text(ctrl, "asymmetric", str(params.asymmetric).lower())
        self._set_element_text(ctrl, "n_neg_sat", str(params.n_neg_sat))

        if mode_uses_planner(mode):
            planner = joint_el.find("planner")
            if planner is None:
                planner = ET.SubElement(joint_el, "planner")
            self._set_element_text(planner, "stiffness",    f"{params.planner_stiffness:.1f}")
            self._set_element_text(planner, "vel_desired",  f"{params.planner_vel_desired:.4f}")
            self._set_element_text(planner, "acc_max",      f"{params.planner_acc_max:.1f}")
            self._set_element_text(planner, "damping_ratio",f"{params.planner_damping_ratio:.6f}")
            self._set_element_text(planner, "frequency",    f"{params.planner_frequency:.1f}")
            self._set_element_text(planner, "wrapped",      str(params.planner_wrapped))

        if mode_uses_admittance(mode):
            adm = joint_el.find("admittance")
            if adm is None:
                adm = ET.SubElement(joint_el, "admittance")
            self._set_element_text(adm, "InvM",    f"{params.admittance_inv_m:.6f}")
            self._set_element_text(adm, "Damping", f"{params.admittance_damping:.1f}")
            self._set_element_text(adm, "dFlow",   self._format_list(params.admittance_dflow))
            self._set_element_text(adm, "Flow",    self._format_list(params.admittance_flow))
            self._set_element_text(adm, "Pose",    self._format_list(params.admittance_pose))

    # ─────────────────────────────────────────────────────────────────────────
    #  SAVING - CARTESIAN PARAMETERS
    # ─────────────────────────────────────────────────────────────────────────

    def save_cartesian_axis(self, robot_name: str, mode: int, axis: str, params: CartesianAxisParams,
                             profile: str = "default"):
        """Save a single Cartesian axis. Auto-creates the robot entry and
        <mode id=N profile=P> block if missing."""
        if not (is_cartesian_mode(mode) or is_hpmp_mode(mode)):
            raise ValueError(f"Mode {mode} is not a Cartesian mode")

        robot = self.ensure_robot(robot_name)
        if is_hpmp_mode(mode):
            self.ensure_hpmp_block(robot_name, profile)

        mode_el = None
        for m in robot.findall("mode"):
            try:
                if int(m.get("id", -1)) == mode and (m.get("profile", "default") or "default") == profile:
                    mode_el = m
                    break
            except ValueError:
                continue
        if mode_el is None:
            mode_el = self._create_mode_element(robot, mode, profile)

        cart_el = mode_el.find("cartesian")
        if cart_el is None:
            cart_el = ET.SubElement(mode_el, "cartesian")

        axis_el = cart_el.find(axis)
        if axis_el is None:
            axis_el = ET.SubElement(cart_el, axis)

        self._write_cartesian_axis(axis_el, params, mode)

    def _write_cartesian_axis(self, axis_el: ET.Element, params: CartesianAxisParams, mode: int):
        ctrl = self._controller_write(axis_el)
        self._set_element_text(ctrl, "stiffness",       f"{params.stiffness:.3f}")
        self._set_element_text(ctrl, "diff_effort_max", f"{params.diff_effort_max:.1f}")
        self._set_element_text(ctrl, "error_max",       self._format_list(params.error_max))
        self._set_element_text(ctrl, "force_max",       self._format_list(params.force_max))
        self._set_element_text(ctrl, "saturation_ratios", self._format_list(params.saturation_ratios))
        self._set_element_text(ctrl, "n_saturations", str(params.n_saturations))
        self._set_element_text(ctrl, "asymmetric", str(params.asymmetric).lower())
        self._set_element_text(ctrl, "n_neg_sat", str(params.n_neg_sat))

        if mode_uses_planner(mode):
            planner = axis_el.find("planner")
            if planner is None:
                planner = ET.SubElement(axis_el, "planner")
            self._set_element_text(planner, "stiffness_P",  f"{params.planner_stiffness_p:.1f}")
            self._set_element_text(planner, "vel_desired",  f"{params.planner_vel_desired:.4f}")
            self._set_element_text(planner, "acc_max",      f"{params.planner_acc_max:.1f}")
            self._set_element_text(planner, "damping_ratio",f"{params.planner_damping_ratio:.6f}")
            self._set_element_text(planner, "frequency",    f"{params.planner_frequency:.1f}")

        if mode_uses_admittance(mode) and not is_hpmp_mode(mode):   # H-PMP's Cartesian layer has none
            adm = axis_el.find("admittance")
            if adm is None:
                adm = ET.SubElement(axis_el, "admittance")
            self._set_element_text(adm, "InvM",    f"{params.admittance_inv_m:.6f}")
            self._set_element_text(adm, "Damping", f"{params.admittance_damping:.1f}")
            self._set_element_text(adm, "dFlow",   self._format_list(params.admittance_dflow))
            self._set_element_text(adm, "Flow",    self._format_list(params.admittance_flow))
            self._set_element_text(adm, "Pose",    self._format_list(params.admittance_pose))

    # ─────────────────────────────────────────────────────────────────────────
    #  POSTURAL PARAMETERS  (robot-scoped, shared by all Cartesian modes)
    # ─────────────────────────────────────────────────────────────────────────

    def _find_postural(self, robot: ET.Element) -> Optional[ET.Element]:
        # The C++ loader checks both "postural" and "Postural" — match that.
        return robot.find("postural") or robot.find("Postural")

    def load_postural_params(self, robot_name: str) -> PosturalParams:
        """Load the null-space postural parameters for a robot. Shared by
        every Cartesian mode at runtime (not per-mode, not per-profile)."""
        robot = self.ensure_robot(robot_name)
        el = self._find_postural(robot)
        params = PosturalParams()
        if el is None:
            return params

        ctrl = self._controller_read(el)

        params.stiffness       = self._get_float(ctrl, "stiffness",       params.stiffness)
        params.diff_effort_max = self._get_float(ctrl, "diff_effort_max", params.diff_effort_max)
        params.velocity        = self._get_float(ctrl, "velocity",        params.velocity)

        if (text := self._get_text(ctrl, "error_max")):
            params.error_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "force_max")):
            params.force_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "saturation_ratios")):
            params.saturation_ratios = self._parse_list(text)

        params.n_saturations = self._get_int(ctrl, "n_saturations", params.n_saturations)
        if (text := self._get_text(ctrl, "asymmetric")):
            params.asymmetric = text.strip().lower() in ("true", "1", "yes")
        params.n_neg_sat = self._get_int(ctrl, "n_neg_sat", params.n_neg_sat)

        planner = el.find("planner")
        if planner is not None:
            params.planner_stiffness     = self._get_float(planner, "stiffness",     params.planner_stiffness)
            params.planner_vel_desired   = self._get_float(planner, "vel_desired",   params.planner_vel_desired)
            params.planner_acc_max       = self._get_float(planner, "acc_max",       params.planner_acc_max)
            params.planner_damping_ratio = self._get_float(planner, "damping_ratio", params.planner_damping_ratio)
            params.planner_frequency     = self._get_float(planner, "frequency",     params.planner_frequency)

        return params

    def save_postural_params(self, robot_name: str, params: PosturalParams):
        """Write the robot's <postural> block. Creates it if missing."""
        robot = self.ensure_robot(robot_name)
        el = self._find_postural(robot)
        if el is None:
            el = ET.SubElement(robot, "postural")

        ctrl = self._controller_write(el)
        self._set_element_text(ctrl, "stiffness",         f"{params.stiffness:.3f}")
        self._set_element_text(ctrl, "diff_effort_max",   f"{params.diff_effort_max:.1f}")
        self._set_element_text(ctrl, "error_max",         self._format_list(params.error_max))
        self._set_element_text(ctrl, "force_max",         self._format_list(params.force_max))
        self._set_element_text(ctrl, "saturation_ratios", self._format_list(params.saturation_ratios))
        self._set_element_text(ctrl, "velocity",          f"{params.velocity:.4f}")
        self._set_element_text(ctrl, "n_saturations",     str(params.n_saturations))
        self._set_element_text(ctrl, "asymmetric",        str(params.asymmetric).lower())
        self._set_element_text(ctrl, "n_neg_sat",         str(params.n_neg_sat))

        planner = el.find("planner")
        if planner is None:
            planner = ET.SubElement(el, "planner")
        self._set_element_text(planner, "stiffness",     f"{params.planner_stiffness:.1f}")
        self._set_element_text(planner, "vel_desired",   f"{params.planner_vel_desired:.4f}")
        self._set_element_text(planner, "acc_max",       f"{params.planner_acc_max:.1f}")
        self._set_element_text(planner, "damping_ratio", f"{params.planner_damping_ratio:.6f}")
        self._set_element_text(planner, "frequency",     f"{params.planner_frequency:.1f}")

    # ─────────────────────────────────────────────────────────────────────────
    #  ELBOW PARAMETERS  (robot-scoped, shared by all Cartesian modes)
    # ─────────────────────────────────────────────────────────────────────────

    def _find_elbow(self, robot: ET.Element) -> Optional[ET.Element]:
        return robot.find("elbow") or robot.find("Elbow")

    def load_elbow_params(self, robot_name: str) -> ElbowParams:
        """Load the null-space elbow parameters for a robot. Shared by
        every Cartesian mode at runtime (not per-mode, not per-profile)."""
        robot = self.ensure_robot(robot_name)
        el = self._find_elbow(robot)
        params = ElbowParams()
        if el is None:
            return params

        ctrl = self._controller_read(el)

        params.stiffness       = self._get_float(ctrl, "stiffness",       params.stiffness)
        params.diff_effort_max = self._get_float(ctrl, "diff_effort_max", params.diff_effort_max)

        if (text := self._get_text(ctrl, "error_max")):
            params.error_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "force_max")):
            params.force_max = self._parse_list(text)
        if (text := self._get_text(ctrl, "saturation_ratios")):
            params.saturation_ratios = self._parse_list(text)

        params.n_saturations = self._get_int(ctrl, "n_saturations", params.n_saturations)
        if (text := self._get_text(ctrl, "asymmetric")):
            params.asymmetric = text.strip().lower() in ("true", "1", "yes")
        params.n_neg_sat = self._get_int(ctrl, "n_neg_sat", params.n_neg_sat)

        return params

    def save_elbow_params(self, robot_name: str, params: ElbowParams):
        """Write the robot's <elbow> block. Creates it if missing."""
        robot = self.ensure_robot(robot_name)
        el = self._find_elbow(robot)
        if el is None:
            el = ET.SubElement(robot, "elbow")

        ctrl = self._controller_write(el)
        self._set_element_text(ctrl, "stiffness",         f"{params.stiffness:.3f}")
        self._set_element_text(ctrl, "diff_effort_max",   f"{params.diff_effort_max:.1f}")
        self._set_element_text(ctrl, "error_max",         self._format_list(params.error_max))
        self._set_element_text(ctrl, "force_max",         self._format_list(params.force_max))
        self._set_element_text(ctrl, "saturation_ratios", self._format_list(params.saturation_ratios))
        self._set_element_text(ctrl, "n_saturations",     str(params.n_saturations))
        self._set_element_text(ctrl, "asymmetric",        str(params.asymmetric).lower())
        self._set_element_text(ctrl, "n_neg_sat",         str(params.n_neg_sat))

    # ─────────────────────────────────────────────────────────────────────────
    #  SAVE TO DISK
    # ─────────────────────────────────────────────────────────────────────────

    def save(self):
        """Write the in-memory XML tree back to disk with pretty-printing."""
        self._indent(self._root)
        write_tree_atomic(self._tree, self.config_path)
        logger.info(f"Saved config to {self.config_path}")

    def _indent(self, elem: ET.Element, level: int = 0):
        """Recursively add indentation whitespace for human-readable output."""
        i = "\n" + "    " * level
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "    "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for child in elem:
                self._indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    # ─────────────────────────────────────────────────────────────────────────
    #  INTROSPECTION HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def get_available_robots(self) -> List[str]:
        return [r.get("name") for r in self._root.findall("robot") if r.get("name")]

    def get_available_modes(self, robot_name: str) -> List[int]:
        """Unique control-mode ids that have at least one <mode> entry."""
        robot = self._find_robot(robot_name)
        if robot is None:
            return []
        ids = set()
        for m in robot.findall("mode"):
            try:
                ids.add(int(m.get("id", -1)))
            except ValueError:
                continue
        return sorted(i for i in ids if 1 <= i <= 8)

    def get_all_profiles(self, robot_name: str) -> List[str]:
        """Union of profile names across every mode for this robot.

        A profile is conceptually robot-scoped: the same name (e.g. 'Stiff')
        may have concrete blocks for some modes and fall back to 'default' in
        others at runtime (see the loader's findModeElement). This method is
        the right source for UI selectors that let the user pick a tuning
        set for the whole robot.
        """
        robot = self._find_robot(robot_name)
        if robot is None:
            return []
        seen = []
        for m in robot.findall("mode"):
            p = m.get("profile", "default") or "default"
            if p not in seen:
                seen.append(p)
        seen.sort(key=lambda p: (p != "default", p.lower()))
        return seen

    def delete_profile_everywhere(self, robot_name: str, profile: str) -> int:
        """Delete a profile from every mode that has it. Returns the number of
        blocks removed. Refuses to delete the 'default' profile."""
        if profile == "default":
            raise ValueError("Cannot delete the 'default' profile")
        robot = self._find_robot(robot_name)
        if robot is None:
            return 0
        victims = []
        for m in robot.findall("mode"):
            if (m.get("profile", "default") or "default") == profile:
                victims.append(m)
        for v in victims:
            robot.remove(v)
        return len(victims)

    def get_available_profiles(self, robot_name: str, mode: int) -> List[str]:
        """Profile names defined for (robot, mode). 'default' always first if present."""
        robot = self._find_robot(robot_name)
        if robot is None:
            return []
        profiles = []
        for m in robot.findall("mode"):
            try:
                if int(m.get("id", -1)) != mode:
                    continue
            except ValueError:
                continue
            p = m.get("profile", "default") or "default"
            if p not in profiles:
                profiles.append(p)
        profiles.sort(key=lambda p: (p != "default", p.lower()))
        return profiles

    def clone_profile(self, robot_name: str, mode: int, src_profile: str, dst_profile: str):
        """Copy all parameters from (mode, src_profile) into a new (mode, dst_profile)
        block. Raises if dst already exists or src is missing."""
        if not dst_profile or dst_profile.strip() != dst_profile:
            raise ValueError("Profile name must be non-empty and not pad-whitespace")
        robot = self.ensure_robot(robot_name)
        src = self._find_mode_element(robot, mode, src_profile)
        if src is None:
            raise ValueError(f"Source profile '{src_profile}' not found for mode {mode}")
        for m in robot.findall("mode"):
            try:
                if int(m.get("id", -1)) == mode and (m.get("profile", "default") or "default") == dst_profile:
                    raise ValueError(f"Profile '{dst_profile}' already exists for mode {mode}")
            except ValueError as e:
                if "already exists" in str(e):
                    raise
                continue
        import copy as _copy
        dst = _copy.deepcopy(src)
        dst.set("profile", dst_profile)
        robot.append(dst)
        return dst

    def copy_profile_blocks(self, robot_name: str, src_profile: str, dst_profile: str) -> int:
        """Copy every <mode> block of src_profile onto dst_profile.

        A block dst already has for that mode is replaced where it stands; a
        missing one is added. dst's blocks for modes src has none of are left
        alone. Returns the number of blocks copied.
        """
        if not dst_profile or dst_profile.strip() != dst_profile or dst_profile == src_profile:
            raise ValueError("Copy needs two different, non-empty profile names")
        robot = self._find_robot(robot_name)
        if robot is None:
            return 0
        import copy as _copy
        src, dst = {}, {}
        for m in robot.findall("mode"):
            try:
                mid = int(m.get("id", -1))
            except ValueError:
                continue
            p = m.get("profile", "default") or "default"
            if p == src_profile:
                src.setdefault(mid, m)
            elif p == dst_profile:
                dst.setdefault(mid, m)
        for mid, m in src.items():
            new = _copy.deepcopy(m)
            new.set("profile", dst_profile)
            old = dst.get(mid)
            if old is None:
                robot.append(new)
            else:
                robot.insert(list(robot).index(old), new)
                robot.remove(old)
        return len(src)

    def delete_profile(self, robot_name: str, mode: int, profile: str):
        """Remove a <mode id=N profile=P> block. Refuses to delete the last
        remaining block for a given mode."""
        robot = self._find_robot(robot_name)
        if robot is None:
            return False
        matching = []
        target = None
        for m in robot.findall("mode"):
            try:
                if int(m.get("id", -1)) != mode:
                    continue
            except ValueError:
                continue
            matching.append(m)
            if (m.get("profile", "default") or "default") == profile:
                target = m
        if target is None:
            return False
        if len(matching) <= 1:
            raise ValueError(f"Cannot delete the only profile for mode {mode}")
        robot.remove(target)
        return True

    def get_num_joints(self, robot_name: str, mode: int = 1, profile: str = "default") -> int:
        robot = self._find_robot(robot_name)
        if robot is None:
            return 0
        mode_el = self._find_mode_element(robot, mode, profile)
        if mode_el is None:
            return 7
        return len(mode_el.findall("joint"))


# ═══════════════════════════════════════════════════════════════════════════════
#  CONVENIENCE FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════════════

def load_robot_parameters(robot_name: str, param_type: str = "Joint", mode: int = 1,
                          profile: str = "default"):
    mgr = XMLManager()
    if param_type == "Joint":
        return [mgr.load_joint_params(robot_name, j, mode, profile) for j in range(7)]
    elif param_type == "Cartesian":
        return mgr.load_cartesian_params(robot_name, mode, profile)
    return None


def save_parameters_to_xml(robot_name: str, joint_id: int, params: dict, mode: int = 2,
                           profile: str = "default"):
    mgr = XMLManager()
    jp = JointParams()
    jp.stiffness       = float(params.get('stiffness', jp.stiffness))
    jp.diff_effort_max = float(params.get('diff_effort_max', jp.diff_effort_max))
    if 'error_max' in params:
        jp.error_max = [float(x) for x in str(params['error_max']).split(',')]
    if 'force_max' in params:
        jp.force_max = [float(x) for x in str(params['force_max']).split(',')]
    if 'planner' in params:
        pl = params['planner']
        jp.planner_stiffness    = float(pl.get('stiffness',    jp.planner_stiffness))
        jp.planner_vel_desired  = float(pl.get('vel_desired',  jp.planner_vel_desired))
        jp.planner_damping_ratio= float(pl.get('damping_ratio',jp.planner_damping_ratio))
    if 'admittance' in params:
        ad = params['admittance']
        jp.admittance_inv_m  = float(ad.get('InvM',    jp.admittance_inv_m))
        jp.admittance_damping= float(ad.get('Damping', jp.admittance_damping))
    mgr.save_joint_params(robot_name, joint_id, mode, jp, profile)
    mgr.save()


def save_cartesian_parameters_to_xml(robot_name: str, axis: str, params: dict, mode: int = 6,
                                     profile: str = "default"):
    mgr = XMLManager()
    cp = CartesianAxisParams()
    cp.stiffness = float(params.get('stiffness', cp.stiffness))
    if 'planner' in params:
        pl = params['planner']
        cp.planner_stiffness_p  = float(pl.get('stiffness_P',  cp.planner_stiffness_p))
        cp.planner_vel_desired  = float(pl.get('vel_desired',   cp.planner_vel_desired))
        cp.planner_damping_ratio= float(pl.get('damping_ratio', cp.planner_damping_ratio))
    if 'admittance' in params:
        ad = params['admittance']
        cp.admittance_inv_m  = float(ad.get('InvM',    cp.admittance_inv_m))
        cp.admittance_damping= float(ad.get('Damping', cp.admittance_damping))
    mgr.save_cartesian_axis(robot_name, mode, axis, cp, profile)
    mgr.save()