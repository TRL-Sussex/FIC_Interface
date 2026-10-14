"""
Tests for SafetyZonePanel: config round-trip, master-disable, live readout.

WS3b — the reworked joint/Cartesian safety GUI. The master toggle gates all
zones; to_config/from_config match the safety_mixin schema; set_live flags
out-of-zone values.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.components.safety_widgets import SafetyZonePanel  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


def test_to_from_config_roundtrip():
    p = SafetyZonePanel(joint_count=7)
    p.from_config({
        'cartesian': {'z': {'min': 0.0, 'max': 0.5, 'enabled': True},
                      'x': {'min': -0.6, 'max': 0.6, 'enabled': False},
                      'y': {'min': -0.6, 'max': 0.6, 'enabled': False}},
        'joint_safety_deg': {'lower': [-160] * 7, 'upper': [160] * 7,
                             'enabled': True},
    })
    out = p.to_config()
    assert out['cartesian']['z']['max'] == 0.5
    assert out['cartesian']['z']['enabled'] is True
    assert out['joint_safety_deg']['enabled'] is True
    assert out['joint_safety_deg']['lower'][0] == -160


def test_master_off_forces_all_disabled():
    p = SafetyZonePanel(joint_count=7)
    p.from_config({'cartesian': {'z': {'min': 0.0, 'max': 0.5, 'enabled': True}},
                   'joint_safety_deg': {'enabled': True}})
    p._master.setChecked(False)
    out = p.to_config()
    assert out['cartesian']['z']['enabled'] is False
    assert out['joint_safety_deg']['enabled'] is False


def test_set_live_marks_out_of_zone():
    p = SafetyZonePanel(joint_count=7)
    p.from_config({'cartesian': {'z': {'min': 0.0, 'max': 0.5, 'enabled': True}}})
    p.set_live(ee_pos=(0.1, 0.0, 0.9), joint_pos_deg=[0.0] * 7)
    assert '0.900' in p._cart['z'].live.text()


def test_set_joint_count_rebuilds_rows():
    p = SafetyZonePanel(joint_count=7)
    assert len(p._joints) == 7
    assert p.set_joint_count(6) is True       # changed -> rebuilt
    assert len(p._joints) == 6
    assert p.set_joint_count(6) is False      # unchanged -> no-op
    # config round-trips at the new count
    p.from_config({'joint_safety_deg': {'lower': [-90] * 6, 'upper': [90] * 6,
                                        'enabled': True}})
    out = p.to_config()
    assert len(out['joint_safety_deg']['lower']) == 6
    assert out['joint_safety_deg']['enabled'] is True
    # growing back works too
    assert p.set_joint_count(7) is True
    assert len(p._joints) == 7


def test_set_joint_count_preserves_enforce_flag():
    p = SafetyZonePanel(joint_count=7)
    p._master.setChecked(True)
    p._joint_en.setChecked(True)
    p.set_joint_count(6)
    assert p._joint_en.isChecked() is True
    assert len(p._joints) == 6


# ── mechanical-limit seeding (quadruped safety-panel-defaults fix) ──────────
# Spot's joint_safety_deg is {lower: [], upper: [], enabled: false}, so
# from_config never touches the rows -- they must be seeded from the robot's
# own joint_limits_deg (mechanical stops), not the manipulator-arm +/-170 deg
# default, or "enforce joint bounds" clamps every leg to a range it never
# occupies.
SPOT_LIMITS = {
    'lower': [-45.0, -51.5, -160.0, -45.0],
    'upper': [45.0, 131.5, -14.6, 45.0],
}


def test_known_joint_limits_seed_the_rows_not_the_arm_default():
    p = SafetyZonePanel(joint_count=4, joint_limits_deg=SPOT_LIMITS)
    mins = [row.min.value() for row in p._joints]
    maxs = [row.max.value() for row in p._joints]
    assert mins == SPOT_LIMITS['lower']
    assert maxs == SPOT_LIMITS['upper']
    # the knee (index 2) never goes positive -- the old hardcoded default
    # would have silently allowed the full +/-170 deg arm range instead.
    assert p._joints[2].max.value() < 0


def test_no_configured_limits_falls_back_to_the_arm_default():
    # explicit joint_limits_deg=None (or empty lists): a robot that genuinely
    # has no mechanical limits configured still gets the +/-170 deg fallback.
    p = SafetyZonePanel(joint_count=4, joint_limits_deg=None)
    assert [row.min.value() for row in p._joints] == [-170.0] * 4
    assert [row.max.value() for row in p._joints] == [170.0] * 4
    q = SafetyZonePanel(joint_count=4, joint_limits_deg={'lower': [], 'upper': []})
    assert [row.min.value() for row in q._joints] == [-170.0] * 4
    assert [row.max.value() for row in q._joints] == [170.0] * 4


def test_explicit_joint_safety_deg_still_wins_over_seeded_defaults():
    p = SafetyZonePanel(joint_count=4, joint_limits_deg=SPOT_LIMITS)
    p.from_config({
        'joint_safety_deg': {'lower': [-30.0] * 4, 'upper': [30.0] * 4,
                             'enabled': True},
    })
    assert [row.min.value() for row in p._joints] == [-30.0] * 4
    assert [row.max.value() for row in p._joints] == [30.0] * 4
    # an empty saved config (Spot's actual on-disk shape) must NOT overwrite
    # the mechanical-limit seed with the arm default.
    q = SafetyZonePanel(joint_count=4, joint_limits_deg=SPOT_LIMITS)
    q.from_config({'joint_safety_deg': {'lower': [], 'upper': [], 'enabled': False}})
    assert [row.min.value() for row in q._joints] == SPOT_LIMITS['lower']
    assert [row.max.value() for row in q._joints] == SPOT_LIMITS['upper']


def test_set_joint_count_reseeds_when_limits_change_at_same_count():
    """A same-joint-count robot swap (e.g. two 7-DoF arms) must reseed too."""
    p = SafetyZonePanel(joint_count=4, joint_limits_deg=SPOT_LIMITS)
    other_limits = {'lower': [-10.0] * 4, 'upper': [10.0] * 4}
    rebuilt = p.set_joint_count(4, joint_limits_deg=other_limits)
    assert rebuilt is True
    assert [row.min.value() for row in p._joints] == [-10.0] * 4
    assert [row.max.value() for row in p._joints] == [10.0] * 4
