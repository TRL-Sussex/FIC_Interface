"""Tests for the contact-localization telemetry accessors on JointStateMixin.

joint_torque_for, contact_estimate_for and admittance_offset_for wire
{rid}_joint_torques, {rid}_contact_estimate and {rid}_admittance_offset
into the Studio. The accessors must never confuse "no message has ever
arrived" with a real zero-valued reading.
"""
import types

from fic_studio.mixins.joint_state_mixin import JointStateMixin


def _host():
    """Return a JointStateMixin host wired to an empty-robot stub node.

    active_robots=[] means _init_joint_state() skips
    _add_robot_subscriptions (and so the deferred `communication` import
    it needs, which is only on sys.path once the ROS workspace has been
    sourced) while still exercising the real dict setup and the two
    global-fallback subscriptions.
    """
    node = types.SimpleNamespace()
    node.active_robots = []
    node.station_robot_id = 'robot1'
    node.create_subscription = lambda *a, **kw: object()

    host = JointStateMixin()
    host.node = node
    host._init_joint_state()
    return host


class TestJointTorqueFor:
    """joint_torque_for reads {rid}_joint_torques — MEASURED torque."""

    def test_empty_before_first_message(self):
        """No message yet means [], not an exception."""
        assert _host().joint_torque_for('robot1') == []

    def test_returns_the_latest_measured_torque(self):
        """A message populates the per-robot cache."""
        host = _host()
        host._on_joint_torques('robot1', types.SimpleNamespace(data=[1.0, 2.0, 3.0]))
        assert host.joint_torque_for('robot1') == [1.0, 2.0, 3.0]

    def test_rid_none_defaults_to_the_station_robot_id(self):
        """Matches the sibling accessors' rid=None convention."""
        host = _host()
        host._on_joint_torques('robot1', types.SimpleNamespace(data=[4.0]))
        assert host.joint_torque_for() == [4.0]

    def test_an_empty_message_does_not_clobber_the_last_good_reading(self):
        """Mirrors _on_joint_velocities' guard against overwriting with []."""
        host = _host()
        host._on_joint_torques('robot1', types.SimpleNamespace(data=[1.0]))
        host._on_joint_torques('robot1', types.SimpleNamespace(data=[]))
        assert host.joint_torque_for('robot1') == [1.0]


class TestContactEstimateFor:
    """contact_estimate_for must distinguish all four readout states."""

    def test_none_before_first_message(self):
        """Absence of a message is its own state, distinct from link 0."""
        assert _host().contact_estimate_for('robot1') is None

    def test_uncalibrated_is_link_index_minus_one(self):
        """-1: not calibrated / gated off, no statement possible."""
        host = _host()
        msg = types.SimpleNamespace(
            link_index=-1, link_name='', confidence=0.0, calibrated=False)
        host._on_contact_estimate('robot1', msg)
        est = host.contact_estimate_for('robot1')
        assert est['link_index'] == -1

    def test_no_contact_is_link_index_zero_and_is_not_none(self):
        """0 is a genuine 'nothing is touching the arm' measurement."""
        host = _host()
        msg = types.SimpleNamespace(
            link_index=0, link_name='', confidence=0.9, calibrated=True)
        host._on_contact_estimate('robot1', msg)
        est = host.contact_estimate_for('robot1')
        assert est is not None
        assert est['link_index'] == 0

    def test_a_contacted_link_reports_its_name_and_confidence(self):
        """1..n carries the link name and confidence through untouched."""
        host = _host()
        msg = types.SimpleNamespace(
            link_index=3, link_name='ForeArm_Link', confidence=0.87,
            calibrated=True)
        host._on_contact_estimate('robot1', msg)
        assert host.contact_estimate_for('robot1') == {
            'link_index': 3, 'link_name': 'ForeArm_Link',
            'confidence': 0.87, 'calibrated': True,
        }


class TestAdmittanceOffsetFor:
    """admittance_offset_for reads {rid}_admittance_offset."""

    def test_empty_before_first_message(self):
        """Absent in every mode but 3/4 — [] rather than an exception."""
        assert _host().admittance_offset_for('robot1') == []

    def test_returns_the_latest_offset(self):
        """A message populates the per-robot cache."""
        host = _host()
        host._on_admittance_offset('robot1', types.SimpleNamespace(data=[0.1, 0.2]))
        assert host.admittance_offset_for('robot1') == [0.1, 0.2]

    def test_an_empty_message_does_not_clobber_the_last_good_reading(self):
        """Same overwrite guard as the torque and velocity callbacks."""
        host = _host()
        host._on_admittance_offset('robot1', types.SimpleNamespace(data=[0.3]))
        host._on_admittance_offset('robot1', types.SimpleNamespace(data=[]))
        assert host.admittance_offset_for('robot1') == [0.3]
