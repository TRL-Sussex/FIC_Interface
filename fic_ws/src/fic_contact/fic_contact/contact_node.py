"""ROS node publishing a link-level contact estimate from joint torque.

Runs alongside the hardware stack so contact events are estimated and
recordable whether or not the twin viewer is open.
"""

import os
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from communication.msg import ContactEstimate
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32
from std_srvs.srv import Trigger

from fic_contact.admittance import admitting_link, combine_link_estimates
from fic_contact.autocal import (
    AutoCalibrator, calibration_disturbed, choose_reference_pose)
from fic_contact.deflection import (
    KINOVA_STIFFNESS_NM_PER_RAD, deflection_sigma, fuse_inverse_variance,
    tau_ext_from_deflection)
from fic_contact.gate import ContactGate
from fic_contact.geometry import load_candidates
from fic_contact.hypotheses import (
    LinkDebouncer, derate_confidence_for_motion, derate_confidence_for_pose_drift,
    fit_contact, support_link)
from fic_contact.recorder import Recorder
from fic_contact.residual import (
    ArmModel, ResidualEstimator, angular_difference,
    link_inertials_from_mjcf, tool_payload_from_mjcf)


def _resolve_repo_root():
    """Locate the repo root from this file's own path, not the cwd.

    Mirrors ``fic_studio.config._resolve_repo_root``: this file lives at
    ``<repo>/fic_ws/src/fic_contact/fic_contact/contact_node.py`` whether
    imported from the source tree or from a colcon copy-install tree
    (``fic_ws/install/fic_contact/lib/pythonX/site-packages/fic_contact/
    contact_node.py``), and both layouts still carry the literal
    ``fic_ws`` path segment -- the repo root is everything before it. A
    fixed ``parents[N]`` count would silently break if this file ever
    moved or were installed elsewhere; walking for the marker does not.
    Falls back to the fixed source-layout walk only if ``fic_ws`` is
    missing from the path entirely. Honours ``FIC_ROOT``, exactly as
    ``fic_studio.config`` does, for consistency between the two packages.
    """
    here = Path(__file__).resolve()
    parts = here.parts
    if 'fic_ws' in parts:
        idx = parts.index('fic_ws')  # first occurrence = the real workspace
        default_root = Path(*parts[:idx]) if idx > 0 else here.parents[4]
    else:
        default_root = here.parents[4]  # fallback: original source-layout walk
    return Path(os.environ.get('FIC_ROOT', str(default_root))).resolve()


#: This module's own repo root -- independent of cwd and of the caller's
#: `workspace` parameter -- computed once at import time.
_REPO_ROOT = _resolve_repo_root()

#: Default for the `workspace` parameter when the caller does not pass one
#: explicitly (see `ContactNode.__init__`). An explicit `workspace` param
#: still overrides this, exactly as before -- only the *default* changed,
#: from `os.getcwd()` (the bug) to this module-derived path.
_DEFAULT_WORKSPACE = str(_REPO_ROOT / 'fic_ws')


def _first_existing(candidates, description):
    """Return the first existing path among ``candidates``, or raise loudly.

    Tried in order, so an explicit ``workspace`` override is honoured
    first; a second, module-derived candidate (independent of
    ``workspace`` entirely -- see ``_resolve_repo_root``) follows as a
    fallback, so a wrong override still has a chance of finding the real
    files. Duplicate candidates (the common case: no override was given,
    so both guesses are the same path) are only listed once. If nothing
    exists, the ``FileNotFoundError`` names every path that was tried --
    not just one wrong guess, which is what made the original bug hard
    to diagnose.
    """
    tried = []
    for candidate in candidates:
        path = os.path.abspath(str(candidate))
        if path not in tried:
            tried.append(path)
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        'could not find %s; tried:\n%s'
        % (description, '\n'.join('  - %s' % p for p in tried)))


class ContactNode(Node):
    """Publish ``{rid}_contact_estimate`` at 100 Hz.

    Runs in all eight control modes — see ``ContactGate``. Modes 1, 2, 5
    and 6 (impedance) use the joint-torque residual alone, exactly as
    before. Modes 3 and 4 (joint admittance) additionally subscribe to
    ``{rid}_admittance_offset`` — the controller's own accumulated
    per-joint "human push offset" (``fic::ficControl::humanOffset``,
    ``fic_control.h``'s ``offest`` accumulator) — and combine it with
    the residual. This is a live controller state readout, not derived
    from ``{rid}_trajectory_data``: an earlier version of this node
    reconstructed a similar-looking quantity as
    ``block5 - block3`` of that topic, which turned out to be the
    trajectory planner's own tracking lag, not the human's push — see
    ``fic_contact.admittance``'s module docstring for the full story
    (``parse_trajectory_blocks`` / ``admitting_joints`` there are kept
    for anyone parsing raw trajectory_data offline, but are no longer
    part of this node's live decision path). Modes 7 and 8 (Cartesian
    admittance) have no equivalent per-joint offset topic at all, so
    they fall back to the residual alone — a single-observable, and
    therefore weaker, estimate in those two modes only.

    Modes 1 and 5 hold the arm still, so the estimate is at its most
    accurate there. Every other mode moves the arm — under a planner
    (2, 6) or under admittance (3, 4, 7, 8) — so the residual uses the
    velocity-aware model term (see ``ResidualEstimator.residual``), the
    calibration pose tolerance is relaxed (see ``MOVING_POSE_TOLERANCE``),
    and both facts are reflected in a derated ``confidence`` rather than
    hidden — see ``_confidence``. Neither the residual nor the offset
    ever models the inertial term ``M(q)*ddq``: no measured joint
    acceleration exists anywhere in this project to feed it.
    """

    #: All eight control modes. Admittance modes 3, 4, 7 and 8 stopped
    #: being refused once the admittance-offset observable (see
    #: ``fic_contact.admittance``) gave them a meaningful estimate.
    ALLOWED_MODES = frozenset({1, 2, 3, 4, 5, 6, 7, 8})

    #: Joint admittance — the only modes where fic_hardware/fic_simulation
    #: publish `{rid}_admittance_offset` at all (see the publisher's own
    #: comment in fic_hardware.cpp/fic_simulation.cpp: absent, not zero,
    #: in every other mode). Cartesian admittance (7, 8) has no equivalent
    #: topic, so it is deliberately excluded here.
    JOINT_ADMITTANCE_MODES = frozenset({3, 4})

    #: rad. Floor below which an admittance offset is treated as noise
    #: rather than a genuine push. ``admitting_link`` applies this itself
    #: (via ``hypotheses.support_link``'s per-joint threshold test), so
    #: no separate pre-flooring step is needed. Chosen well below the
    #: offset a deliberate push leaves once it clears the controller's
    #: own 2.0 Nm torque deadband.
    ADMITTANCE_OFFSET_THRESHOLD_RAD = 0.01

    RATE_HZ = 100.0

    #: rad/s. Below this, every joint is treated as at rest — the residual
    #: uses the gravity-only model term and the calibration pose tolerance
    #: is not relaxed. Chosen well above typical joint-velocity sensor
    #: jitter at a genuinely stationary pose.
    MOVING_DQ_THRESHOLD = 0.02

    #: rad. Pose tolerance used for the staleness check once the arm is
    #: moving (see ``MOVING_DQ_THRESHOLD``). The estimator's own default
    #: (0.05 rad) would go stale within a fraction of a second of a
    #: planner starting to move, making the feature look broken in modes
    #: 2 and 6. This wider tolerance keeps it usable; the cost is that the
    #: bias is only approximate over that wider range of poses, which is
    #: why moving-mode confidence is derated for pose drift as well as
    #: for velocity — see ``_confidence``.
    MOVING_POSE_TOLERANCE = 0.5

    def __init__(self):
        """Load the robot model and wire up the ROS interface."""
        super().__init__('contact_localization')
        self.declare_parameter('station_id', 1)
        self.declare_parameter('robot_name', 'KinovaGen3_7DOF')
        self.declare_parameter('workspace', _DEFAULT_WORKSPACE)
        self.declare_parameter('sigma_k', 6.0)
        # Escape hatch: a robot whose URDF already carries its tool, or
        # one where the MJCF and URDF disagree about the wrist, should not
        # have the payload added twice.
        # Multiple of the mean detection threshold within which two link
        # hypotheses count as explaining the residual equally well, so the
        # more proximal one wins. fit_contact selects on minimum residual,
        # and link k+1's Jacobian spans link k's columns plus one more, so
        # an adjacent distal link fits a contact almost as well as the true
        # one (measured 0.0000 Nm vs 0.0628 Nm) and noise hands it the
        # answer. Measured against the MuJoCo rig, 0.25 lifts published
        # link accuracy from 32.9% to 54.1%; 0 restores the old behaviour.
        # See fic_contact.evaluate.
        # Second observable of the same contact, from the controller's
        # tracking error rather than the torque sensors:
        # K*(q_current - q_target). Measured against the MuJoCo rig at a
        # stale calibration, fusing it lifted link accuracy from 49.8% to
        # 58.2% -- and to the SAME 58.2% with a deliberately wrong gravity
        # model, because the two channels carry the model error with
        # opposite signs and equal weights cancel it. Default OFF: it is
        # unvalidated on hardware and changes the estimator's core, so it
        # is a flag to A/B rather than a silent change.
        # This arm powers up in Joint Impedance, and {rid}_control_mode is
        # VOLATILE and sent only on change -- so a node started with the
        # arm waits forever for a message that never comes, and sits
        # blocked until an operator touches the mode selector. Assume the
        # boot mode; a real message still overrides it immediately.
        # Temporal hysteresis on the reported link, in 100 Hz frames.
        # Entry is cheap so first detection stays responsive; release is
        # dearer so a momentary dropout does not blank the highlight; and
        # SWITCHING an established link is dearest of all, because a hand
        # resting in one place is the common case and the estimate
        # wandering to a neighbouring link is the common failure. Observed
        # on hardware: with entry and switch equal, the highlight jumped
        # between links while the hand stayed still.
        self.declare_parameter('debounce_enter_frames', 5)
        self.declare_parameter('debounce_exit_frames', 15)
        self.declare_parameter('debounce_switch_frames', 40)
        self.declare_parameter('default_control_mode', 1)
        self.declare_parameter('fuse_deflection', False)
        # 'equal' cancels the shared gravity model error and wins above
        # ~10 N; 'inverse-variance' weights by noise and wins below it
        # (15.5% vs 10.5% at 5 N). The crossover is where bias overtakes
        # noise. See fic_contact.evaluate.
        self.declare_parameter('fusion_mode', 'equal')
        self.declare_parameter('encoder_sigma_rad', 1e-4)
        self.declare_parameter(
            'joint_stiffness', list(KINOVA_STIFFNESS_NM_PER_RAD))
        self.declare_parameter('parsimony_k', 0.25)
        # Rebuild every link's inertia from the MJCF, not just the tool.
        # Exact where the tool payload was merely close: worst gravity
        # error against MuJoCo falls to 0.00 Nm on the Kinova (from 0.85)
        # and 0.00 Nm on the Franka (from 49.36, whose URDF carries no
        # <inertial> elements at all and is otherwise unusable).
        # Supersedes tool_payload_from_mjcf, which it already includes --
        # enabling both would count the tool twice.
        self.declare_parameter('link_inertials_from_mjcf', True)
        self.declare_parameter('tool_payload_from_mjcf', True)
        self.declare_parameter('calibration_seconds', 1.0)
        # How many recent calibration windows the noise estimate is
        # pooled over (the bias is never pooled -- it is the thing
        # that must stay fresh). Per-second sigma varies ~3x on a
        # still arm here, so a single window makes the thresholds a
        # lottery and a quiet one leaves the next noisy second
        # reading as contact. 8 windows ~= 8 s of pooled samples.
        # 1 restores the original single-window behaviour.
        self.declare_parameter('sigma_pool', 8)
        # Discard a calibration window whose noise on any joint is over
        # this multiple of the recent windows' median (plus 0.05 Nm): it
        # was disturbed. Second line of defence behind the per-tick
        # disturbance check in _tick, and the one that matters most
        # because the noise is pooled -- one bad window otherwise sets the
        # thresholds for the next eight. Per-second noise varies ~3x on a
        # still arm, so 5 leaves room for that. 0 accepts every window.
        self.declare_parameter('calibration_outlier_ratio', 5.0)
        # rad. How far a STILL arm may leave the pose its bias was measured
        # at before the estimate is refused as stale. Read every tick, so
        # a client may widen it while it needs estimates from an arm held
        # deflected: the Following extension raises it while it samples a
        # push and restores it after, because a firm push moves the arm
        # past 0.05 rad and blocked most of such a sample on hardware.
        # While the arm moves, MOVING_POSE_TOLERANCE applies instead.
        self.declare_parameter('stale_pose_tolerance', 0.05)
        # Leave joints gravity does not depend on -- the base joint of an
        # arm on a vertical axis -- out of the staleness test. Turning the
        # base swings the arm round without changing any joint's load, so
        # the bias stays valid; counting it blocked the estimate after
        # every turn of the Following page until a recalibration finished.
        self.declare_parameter('stale_ignores_base_turn', True)
        # Auto-recalibration: re-measure the bias whenever the arm is
        # demonstrably untouched, so it survives node restarts and being
        # moved around without an operator pressing calibrate. See
        # fic_contact.autocal for why this is restricted to stiff modes.
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('auto_calibrate_tolerance', 0.02)
        # s. How long the steady-state tracking offset must hold
        # steady before it is adopted as the auto-calibration
        # baseline. Measured on this arm, the offset's peak-to-peak
        # over 10 s of a still, untouched arm is 0.00000 rad on
        # every joint -- so 'the offset stopped changing' is a far
        # sharper signal than 'the offset is small', and unlike a
        # frozen baseline it survives the arm moving to a new pose,
        # where gravity_error/K gives a different equilibrium.
        # This is also the push-absorption window: a push held
        # perfectly still for longer than this re-baselines and is
        # absorbed into the bias. Manual calibrate stays the
        # authoritative reset.
        self.declare_parameter('auto_calibrate_baseline_stable_seconds', 2.0)
        # rad. Movement below this counts as 'the offset did not
        # change'. Ten times the measured 0.0000 rad noise floor.
        self.declare_parameter('auto_calibrate_baseline_epsilon', 0.002)
        self.declare_parameter('auto_calibrate_dwell_seconds', 0.5)
        # Re-measure the bias this often while the arm sits settled and
        # untouched, even when it has not moved. Drift that is not pose
        # drift -- thermal, sensor bias -- is invisible to the staleness
        # test: measured on hardware at 0.53 Nm against 0.10 Nm
        # thresholds, giving 100% false positives on an untouched arm.
        # 0 disables the refresh.
        #
        # 2.0 s, not the original 8.0: the arm was spending most of a
        # session on 'calibration stale'. The full refresh cycle is
        # max_age + dwell + calibration_seconds -- 2.0 + 0.5 + 1.0 =
        # 3.5 s between calibration starts, because _age does not
        # advance while a window is open (_tick returns early) and the
        # dwell has to rebuild afterwards. This used to cost a blind
        # window -- _tick returned early while calibrating, publishing
        # nothing for 1.0 s in every 3.5 s -- but the window now keeps
        # publishing on the previous bias, so the refresh rate no longer
        # trades against availability.
        self.declare_parameter('auto_calibrate_max_age_seconds', 2.0)
        # s. Hard ceiling on how long a 'the arm is loaded' reading
        # may hold off recalibration. Without a ceiling a bad bias
        # is self-sustaining: it makes the residual report contact,
        # that report vetoes recalibration, and nothing can ever
        # replace the bias that is generating it -- measured on this
        # arm at 75 s stuck and 99.7% false positives, completely
        # untouched. Staleness cannot break it: that test is
        # pose-based and the arm had not moved. 3x max_age is long
        # enough to protect a normal push and short enough that a
        # bad bias always clears. 0 disables it (deadlock returns).
        self.declare_parameter('auto_calibrate_touch_veto_seconds', 6.0)
        # Default OFF, on measured evidence. The veto's purpose is to
        # stop a push held perfectly still from being absorbed into
        # the bias, which no pose-based test can see. Measured on
        # this arm it costs far more than it buys: a transient bad
        # bias makes the residual read contact, the veto then blocks
        # recalibration for 6-12 s, and a blip that used to clear
        # itself in 2.5 s persists instead. Like-for-like on an
        # untouched arm, single node: 0.9% false positives without
        # it, 24.5% with it. The push it protects against is
        # hypothetical here; the false positives are not. Turn it on
        # to A/B once absorbed pushes are shown to be a real problem.
        self.declare_parameter('auto_calibrate_touch_veto', False)
        #: 'v2' (default) fits a push-constrained contact point/force with
        #: fit_contact, gated on support_link already reporting a contact.
        #: 'v1' keeps the pre-existing link-only behaviour, leaving
        #: ContactEstimate.force/point/residual_norm empty.
        self.declare_parameter('estimator', 'v2')
        #: Operator-declared ground truth for the run being recorded, as a
        #: link name or 'none' -- see set_link_truth and _on_set_parameters.
        #: Set remotely via the standard ROS set_parameters service (the
        #: fic_studio ground-truth combo uses this), never by another
        #: process reaching into this node's Python object directly.
        self.declare_parameter('link_truth', 'none')
        #: geometry.load_candidates's own default (200) sweeps ~1400
        #: candidates total and measured ~32 ms per fit_contact call on
        #: this machine -- more than 3x the 10 ms/tick budget at 100 Hz,
        #: and sustained for as long as a human keeps pushing (fit_contact
        #: runs every tick support_link reports a contact, not just once).
        #: 20 measured at ~3 ms/call, leaving headroom for the rest of
        #: _tick; logged below rather than silently capped, since a silent
        #: cap would read as full 200-point coverage when it is not.
        self.declare_parameter('n_per_link', 20)

        sid = self.get_parameter('station_id').value
        self._rid = 'robot%s' % sid
        robot_name = self.get_parameter('robot_name').value
        ws = self.get_parameter('workspace').value

        cfg = self._load_robot_config(ws, robot_name)
        self._joint_names = cfg['joint_names']
        self._n = int(cfg['num_joints'])
        self._joint_torque_limits = cfg.get('joint_torque_limits', {})
        self._workspace = ws
        self._robot_name = robot_name

        urdf_path = _first_existing(
            [os.path.join(ws, 'src', cfg['urdf_path']),
             os.path.join(_REPO_ROOT, 'fic_ws', 'src', cfg['urdf_path'])],
            'URDF %r' % cfg['urdf_path'])
        mjcf_path = _first_existing(
            [os.path.join(ws, 'src', cfg['mujoco_xml_path']),
             os.path.join(_REPO_ROOT, 'fic_ws', 'src', cfg['mujoco_xml_path'])],
            'MJCF %r' % cfg['mujoco_xml_path'])

        # The URDF under-describes what the arm carries: for the Kinova it
        # ends at ee_link with mass="0" and has no Robotiq gripper, leaving
        # g(q) wrong by up to 9.4 Nm and -- worse, because a bias
        # calibration cannot absorb it -- varying by 25 Nm across the
        # workspace. The MJCF loaded just above for the candidate geometry
        # describes the same arm complete with its tool, so the missing
        # mass is derived from it rather than hand-entered.
        payload, inertials = None, None
        if self.get_parameter('link_inertials_from_mjcf').value:
            inertials = link_inertials_from_mjcf(
                mjcf_path, self._joint_names)
            self.get_logger().info(
                'link inertials rebuilt from MJCF: %.3f kg total across %d '
                'links. This replaces the URDF mass distribution outright '
                'and already contains the tool, so tool_payload is skipped.'
                % (sum(m for m, _, _ in inertials), len(inertials)))
        elif self.get_parameter('tool_payload_from_mjcf').value:
            mass, com = tool_payload_from_mjcf(mjcf_path, self._joint_names)
            if mass > 0.0:
                payload = (mass, com)
                self.get_logger().info(
                    'tool payload from MJCF: %.3f kg at %s in the %s frame '
                    '(the URDF omits this; without it g(q) drifts by tens '
                    'of Nm across the workspace)'
                    % (mass, np.round(com, 4).tolist(), self._joint_names[-1]))
        self._model = ArmModel(urdf_path, self._joint_names,
                               tool_payload=payload,
                               link_inertials=inertials)
        if self._model.gravity_is_degenerate():
            self.get_logger().error(
                'gravity torque is identically zero for %s: the URDF has no '
                'usable inertials and the MJCF could not supply them. Every '
                'residual would be meaningless, so contact estimation will '
                'report no statement.' % robot_name)
        self._est = ResidualEstimator(
            self._model,
            torque_sign=1.0,  # already applied by fic_hardware — see the spec
            gravity_compensated=bool(cfg.get('gravity_compensated', False)),
            sigma_k=float(self.get_parameter('sigma_k').value),
            sigma_pool=int(self.get_parameter('sigma_pool').value),
            outlier_ratio=float(
                self.get_parameter('calibration_outlier_ratio').value),
            ignore_joints=(self._model.gravity_free_joints()
                           if self.get_parameter(
                               'stale_ignores_base_turn').value else ()))
        if self.get_parameter('stale_ignores_base_turn').value:
            self.get_logger().info(
                'staleness ignores joints %s: gravity does not depend on '
                'them, so turning them leaves the bias valid'
                % [j + 1 for j in self._model.gravity_free_joints()])
        n_per_link = int(self.get_parameter('n_per_link').value)
        self._candidates = load_candidates(
            mjcf_path, self._model.link_names, n_per_link=n_per_link)
        self.get_logger().info(
            'v2 fit: %d candidates/link (geometry.load_candidates default '
            'is 200; reduced here to keep fit_contact inside the 100 Hz '
            'tick budget -- see the n_per_link parameter comment)'
            % n_per_link)

        self._gate = ContactGate(allowed_mode=self.ALLOWED_MODES)
        self._q_desired = None
        self._q_target = None
        self._q_target_active = None
        self._defl_bias = None
        self._defl_samples = []
        stiffness = list(self.get_parameter('joint_stiffness').value)
        if len(stiffness) < self._n:
            stiffness = list(KINOVA_STIFFNESS_NM_PER_RAD)
        self._stiffness = np.asarray(stiffness[:self._n], dtype=float)
        self._defl_sigma = deflection_sigma(
            self._stiffness,
            float(self.get_parameter('encoder_sigma_rad').value))
        self._autocal = AutoCalibrator(
            tolerance=float(
                self.get_parameter('auto_calibrate_tolerance').value),
            dwell_ticks=int(
                self.RATE_HZ * float(self.get_parameter(
                    'auto_calibrate_dwell_seconds').value)),
            max_age_ticks=int(
                self.RATE_HZ * float(self.get_parameter(
                    'auto_calibrate_max_age_seconds').value)),
            veto_max_age_ticks=int(
                self.RATE_HZ * float(self.get_parameter(
                    'auto_calibrate_touch_veto_seconds').value)))
        self._q = None
        self._tau = None
        self._dq = None
        #: Rolling tracker for the auto-calibration baseline: the
        #: last offset seen and how many ticks it has held steady.
        self._offset_prev = None
        self._offset_stable = 0
        self._admittance_offset = None
        self._mode = int(self.get_parameter('default_control_mode').value)
        self._gate.set_mode(self._mode)
        self._last_tau_ext = None
        self._calibrating = False
        self._cal_ticks = 0
        #: The pose an open calibration window started at, and whether the
        #: arm was moving then -- what calibration_disturbed judges against.
        self._cal_q0 = None
        self._cal_started_moving = False

        #: Operator-declared ground-truth link for the run being recorded,
        #: set from outside via ``set_link_truth`` (the GUI supplies this
        #: later — see fic_docs). Defaults to None, an absent value never
        #: a guess, so "no ground truth declared" is never confused with
        #: "the operator declared no contact" (link 0).
        self._link_truth = None
        #: Temporal hysteresis on the reported link: entry is
        #: faster than exit, so a momentary dropout does not flicker the
        #: highlight. The estimate that fed it is still logged unfiltered
        #: as link_est_raw, so the debounced false-positive rate stays
        #: measurable rather than hidden behind the filter.
        self._debounce = LinkDebouncer(
            enter_frames=int(self.get_parameter(
                'debounce_enter_frames').value),
            exit_frames=int(self.get_parameter(
                'debounce_exit_frames').value),
            switch_frames=int(self.get_parameter(
                'debounce_switch_frames').value))
        #: None while not recording; a Recorder while a run is open.
        self._recorder = None
        self._record_dir = None
        self._modes_seen = set()

        self.create_subscription(
            Float64MultiArray, '%s_joint_torques' % self._rid,
            self._on_torque, qos_profile_sensor_data)
        self.create_subscription(
            Float64MultiArray, '%s_q_current' % self._rid,
            self._on_q, qos_profile_sensor_data)
        self.create_subscription(
            Float64MultiArray, '%s_joint_velocities' % self._rid,
            self._on_dq, qos_profile_sensor_data)
        self.create_subscription(
            Float64MultiArray, '%s_admittance_offset' % self._rid,
            self._on_admittance_offset, qos_profile_sensor_data)
        self.create_subscription(
            Int32, '%s_control_mode' % self._rid, self._on_mode, 10)
        # Float32, not Float64: FIC Studio and fic_hardware both publish
        # this topic as Float32MultiArray, and a mismatched subscription
        # never matches the publisher -- silently, with _q_desired stuck
        # at None and auto-calibration never firing.
        self.create_subscription(
            Float32MultiArray, '%s_desired_joint_angles' % self._rid,
            self._on_q_desired, 10)
        self.create_subscription(
            Float64MultiArray, '%s_q_target' % self._rid,
            self._on_q_target, qos_profile_sensor_data)
        self.create_subscription(
            Float64MultiArray, '%s_q_target_active' % self._rid,
            self._on_q_target_active, qos_profile_sensor_data)
        self._pub = self.create_publisher(
            ContactEstimate, '%s_contact_estimate' % self._rid, 10)
        self.create_service(
            Trigger, '%s/contact/calibrate' % self._rid, self._on_calibrate)
        self.create_service(
            Trigger, '%s/contact/record' % self._rid, self._on_record)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(1.0 / self.RATE_HZ, self._tick)

        self.get_logger().info(
            'contact localization ready for %s (%d joints, modes %s)'
            % (robot_name, self._n, sorted(self.ALLOWED_MODES)))

    @staticmethod
    def _load_robot_config(ws, robot_name):
        """Read one robot's block from robots_config.yaml.

        ``ws`` is either the caller's explicit ``workspace`` parameter
        override or this module's own resolved default (``fic_ws`` under
        ``_resolve_repo_root()``) -- see ``ContactNode.__init__`` and
        ``_first_existing`` for how a wrong or missing ``ws`` still gets
        a fallback attempt, and how a genuine failure is reported.
        """
        path = _first_existing(
            [os.path.join(ws, '..', 'fic_library', 'config',
                          'robots_config.yaml'),
             os.path.join(_REPO_ROOT, 'fic_library', 'config',
                          'robots_config.yaml')],
            'robots_config.yaml')
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
        robots = data.get('robots', data)
        if robot_name not in robots:
            raise ValueError('robot %r not in robots_config.yaml' % robot_name)
        return robots[robot_name]

    def _on_torque(self, msg):
        """Cache the latest measured joint torque."""
        if len(msg.data) >= self._n:
            self._tau = np.asarray(msg.data[:self._n], dtype=float)

    def _on_q(self, msg):
        """Cache the latest joint configuration."""
        if len(msg.data) >= self._n:
            self._q = np.asarray(msg.data[:self._n], dtype=float)

    def _on_dq(self, msg):
        """Cache the latest joint velocity.

        Published by ``fic_hardware`` in every control mode, so this is
        available whether the arm is holding still or being moved by a
        planner.
        """
        if len(msg.data) >= self._n:
            self._dq = np.asarray(msg.data[:self._n], dtype=float)

    def _on_admittance_offset(self, msg):
        """Cache the latest `{rid}_admittance_offset` array.

        Only ever published while joint admittance is active (modes 3,
        4 — see the publisher in fic_hardware.cpp/fic_simulation.cpp),
        so a stale cached value from a previous admittance session can
        briefly survive a mode switch; ``_estimate`` only reads this
        while ``self._mode`` is itself in ``JOINT_ADMITTANCE_MODES``, so
        that staleness is bounded to the first tick or two after
        re-entering an admittance mode, exactly as the old
        `{rid}_trajectory_data`-derived path was.
        """
        if len(msg.data) >= self._n:
            self._admittance_offset = np.asarray(msg.data[:self._n], dtype=float)

    def _max_abs_dq(self):
        """Max |dq| across joints, rad/s, or 0.0 with no velocity data yet."""
        if self._dq is None:
            return 0.0
        return float(np.max(np.abs(self._dq)))

    def _is_moving(self):
        """True once any joint's measured speed clears MOVING_DQ_THRESHOLD."""
        return self._max_abs_dq() > self.MOVING_DQ_THRESHOLD

    def _on_q_target_active(self, msg):
        """Cache the target the live control mode actually tracks."""
        self._q_target_active = list(msg.data)

    def _on_q_target(self, msg):
        """Cache the target the impedance controller is tracking.

        Distinct from `_q_desired`: in the Cartesian modes fic_hardware
        derives q_target from its own IK, so the command stream cannot be
        used to reconstruct the tracking error.
        """
        self._q_target = list(msg.data)

    def _deflection_tau(self):
        """Return the deflection channel's tau_ext estimate, or None."""
        if self._q is None or self._q_target is None:
            return None
        if len(self._q_target) < self._n:
            return None
        return tau_ext_from_deflection(
            np.asarray(self._q_target[:self._n], dtype=float),
            np.asarray(self._q[:self._n], dtype=float),
            self._stiffness)

    def _on_q_desired(self, msg):
        """Cache the commanded joint pose, for the auto-calibrate proxy."""
        self._q_desired = list(msg.data)

    def _on_mode(self, msg):
        """Track the control mode for the gate and for `_estimate`."""
        self._mode = int(msg.data)
        self._gate.set_mode(self._mode)

    def _on_calibrate(self, _request, response):
        """Start a no-contact calibration window at the current pose."""
        if self._q is None or self._tau is None:
            response.success = False
            response.message = 'no joint feedback yet'
            return response
        self._begin_calibration()
        response.success = True
        response.message = 'calibrating — do not touch the arm'
        return response

    def _begin_calibration(self):
        """Open a no-contact calibration window at the current pose.

        The single entry point for both the operator's calibrate service
        and the automatic trigger, so the two can never drift apart.
        """
        self._est.start_calibration()
        self._calibrating = True
        self._cal_ticks = int(
            self.RATE_HZ * float(
                self.get_parameter('calibration_seconds').value))
        self._cal_q0 = None if self._q is None else np.array(self._q)
        self._cal_started_moving = self._is_moving()
        self._defl_samples = []
        # Re-baseline the steady-state tracking offset that
        # auto-calibration measures its tolerance against. An impedance
        # arm settles a constant distance from its commanded pose (0.034
        # rad on this Kinova, against a 0.02 rad tolerance), so comparing
        # the absolute distance blocked auto-calibration outright and let
        # the bias go stale enough to report contact on a still arm.
        # Doing this here, rather than only on the automatic path, means
        # the operator's own calibrate button re-baselines too.
        ref = choose_reference_pose(
            self._q_target, self._q_desired, expected=self._n,
            active=self._q_target_active)
        if self._q is not None and ref is not None and len(ref) == self._n:
            self._autocal.set_offset_reference(angular_difference(
                self._q, np.asarray(ref[:self._n], dtype=float)))
        self._autocal.reset()
        # Drop the latch too: a new bias means the evidence the current
        # highlight rests on no longer applies. This is also what clears a
        # highlight left standing after the operator let go -- once the arm
        # settles back on its target, auto-calibration fires and releases
        # it, rather than the estimate having to fall below threshold on
        # its own.
        self._debounce.reset()

    def _finish_calibration(self):
        """Close the open window: adopt it, or keep the old bias if it was bad."""
        cal = self._est.finish_calibration()
        self._calibrating = False
        if cal is None:
            self._defl_samples = []
            self.get_logger().warn(
                'calibration discarded (%s -- touched?); keeping the '
                'previous bias.' % self._est.last_discard)
            return
        self._defl_bias = (
            np.mean(self._defl_samples, axis=0)
            if self._defl_samples else None)
        self._defl_samples = []
        self.get_logger().info(
            'calibrated: thresholds %s Nm'
            % np.round(cal.thresholds, 3).tolist())

    def _abort_calibration(self, why):
        """Drop a disturbed window; the previous bias stays in force.

        The dwell and bias age restart, so auto-calibration waits for the
        arm to settle again before it opens the next window.
        """
        self._est.abort_calibration()
        self._calibrating = False
        self._defl_samples = []
        self._autocal.reset()
        self.get_logger().warn(
            'calibration discarded (%s); keeping the previous bias.' % why,
            throttle_duration_sec=2.0)

    def set_link_truth(self, link_index):
        """Set the operator-declared ground-truth link for this run.

        ``link_index`` follows the ``link_index`` regimes used everywhere
        else in this module (0 = operator says no contact, 1..n = the
        link, ``None`` to clear it back to "no ground truth declared").
        Called from outside — the GUI will supply this later; nothing in
        this package calls it yet. Kept separate from ``link_est`` so a
        run's CSV can measure the estimator against a declared truth
        without the truth ever being confused with "no contact".
        """
        self._link_truth = link_index

    def _on_set_parameters(self, params):
        """Apply a live `link_truth` parameter change from outside.

        Runs inside the node's standard ROS `set_parameters` service, so
        the fic_studio ground-truth combo (or `ros2 param set`) can set
        this without any process reaching into this node's Python object
        directly. `'none'` clears the declared truth back to "no ground
        truth declared" -- see `set_link_truth` for why that must stay
        distinct from the operator declaring "no contact" (link 0, not
        exposed by this parameter since the combo never offers it).
        Every other declared parameter passes through unchanged.
        """
        for param in params:
            if param.name != 'link_truth':
                continue
            value = param.value
            if value == 'none':
                self.set_link_truth(None)
            elif value in self._model.link_names:
                self.set_link_truth(self._model.link_names.index(value) + 1)
            else:
                return SetParametersResult(
                    successful=False,
                    reason='link_truth %r is not a known link name' % value)
        return SetParametersResult(successful=True)

    def _record_columns(self):
        """Build the data.csv column list, sized to this robot's joint count.

        ``t, q1..qN, tau_meas1..N, tau_ext1..N, link_est, confidence,
        link_truth`` plus ``dq1..dqN``, ``adm_offset1..N`` (admittance
        modes only — often absent), ``link_est_raw`` (the estimate before
        the ``LinkDebouncer``'s hysteresis — differs from ``link_est``
        during an entry/exit window, and is what keeps the false-positive
        rate measurable rather than hidden behind the filter) and ``mode``.
        """
        idx = range(1, self._n + 1)
        cols = ['t']
        cols += ['q%d' % i for i in idx]
        cols += ['tau_meas%d' % i for i in idx]
        cols += ['tau_ext%d' % i for i in idx]
        cols += ['link_est', 'confidence', 'link_truth']
        cols += ['dq%d' % i for i in idx]
        cols += ['adm_offset%d' % i for i in idx]
        cols += ['link_est_raw', 'mode']
        return cols

    def _start_recording(self):
        """Open a fresh run folder under experiments/contact/ and buffer."""
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._record_dir = os.path.join(
            self._workspace, 'experiments', 'contact',
            '%s_%s' % (stamp, self._robot_name))
        os.makedirs(self._record_dir, exist_ok=True)
        self._recorder = Recorder(self._record_columns())
        self._modes_seen = set()

    def _record_meta(self):
        """Assemble everything needed to interpret data.csv, months later."""
        return {
            'robot': self._robot_name,
            'joint_names': list(self._joint_names),
            'link_names': list(self._model.link_names),
            'control_modes_seen': sorted(
                m for m in self._modes_seen if m is not None),
            'sample_rate_hz': self.RATE_HZ,
            'calibration': (self._est.calibration.as_dict()
                            if self._est.calibrated else None),
            'joint_torque_limits': self._joint_torque_limits,
            'estimator_params': {
                'estimator': self.get_parameter('estimator').value,
                'n_per_link': int(self.get_parameter('n_per_link').value),
                'sigma_k': float(self.get_parameter('sigma_k').value),
                # No public accessor exists for this constructor default;
                # read the estimator's own instance rather than duplicate
                # ResidualEstimator's default and risk it drifting stale.
                'min_threshold_nm': self._est._min_threshold_nm,
                'pose_tolerance_rad': self._est.pose_tolerance,
            },
            'units': {
                'time': 's',
                'q': 'rad',
                'dq': 'rad/s',
                'tau_meas': 'Nm',
                'tau_ext': 'Nm',
                'adm_offset': 'rad',
                'confidence': 'dimensionless, 0..1',
                'link_est': (
                    'link index: -1 no statement, 0 no contact, '
                    '1..n link number (proximal to distal)'),
                'link_est_raw': 'same regime as link_est, pre-debounce',
                'link_truth': 'same regime as link_est, operator-declared',
                'mode': 'control mode 1..8, see ContactGate',
            },
        }

    def _stop_recording(self):
        """Flush the buffered run to disk and clear recording state."""
        csv_path = os.path.join(self._record_dir, 'data.csv')
        meta_path = os.path.join(self._record_dir, 'meta.json')
        self._recorder.write_csv(csv_path)
        self._recorder.write_meta(meta_path, self._record_meta())
        path = self._record_dir
        self._recorder = None
        self._record_dir = None
        return path

    def _on_record(self, _request, response):
        """Toggle run recording, writing data.csv + meta.json on stop."""
        if self._recorder is None:
            self._start_recording()
            response.success = True
            response.message = 'recording started: %s' % self._record_dir
            return response
        path = self._stop_recording()
        response.success = True
        response.message = 'recording stopped: %s' % path
        return response

    def _add_record_row(self, link_est=None, confidence=None, link_est_raw=None):
        """Append one sample to the active run; absent channels stay absent.

        Called at most once per tick, from both the blocked-tick path
        (``link_est``/``confidence``/``link_est_raw`` stay None — a tick
        where no estimate was computed is genuinely absent, never a
        guessed 0) and the full-estimate path. ``tau_ext`` and the
        admittance offset are likewise only ever taken from the current
        tick's fresh values, never a stale carry-over from an earlier
        tick that happened to compute one.
        """
        if self._recorder is None:
            return
        row = {'t': self.get_clock().now().nanoseconds / 1e9}
        if self._q is not None:
            for i, v in enumerate(self._q, start=1):
                row['q%d' % i] = float(v)
        if self._dq is not None:
            for i, v in enumerate(self._dq, start=1):
                row['dq%d' % i] = float(v)
        if self._tau is not None:
            for i, v in enumerate(self._tau, start=1):
                row['tau_meas%d' % i] = float(v)
        # tau_ext is only ever this tick's fresh residual: link_est is
        # None on the blocked-tick path, where _estimate() was never
        # called and self._last_tau_ext (if set at all) is a stale
        # carry-over from an earlier tick that must not be recorded here.
        if link_est is not None and self._last_tau_ext is not None:
            for i, v in enumerate(self._last_tau_ext, start=1):
                row['tau_ext%d' % i] = float(v)
        if (self._mode in self.JOINT_ADMITTANCE_MODES
                and self._admittance_offset is not None):
            for i, v in enumerate(self._admittance_offset, start=1):
                row['adm_offset%d' % i] = float(v)
        if link_est is not None:
            row['link_est'] = link_est
        if confidence is not None:
            row['confidence'] = confidence
        if link_est_raw is not None:
            row['link_est_raw'] = link_est_raw
        if self._link_truth is not None:
            row['link_truth'] = self._link_truth
        if self._mode is not None:
            row['mode'] = self._mode
            self._modes_seen.add(self._mode)
        self._recorder.add(row)

    def _touched(self):
        """True when the existing bias's own residual says a joint is loaded.

        The pose-based trigger cannot see a push that is held perfectly
        still: once the transient is over the arm is stationary again,
        the offset has stopped changing, and every proxy reads
        'untouched' while somebody is leaning on it. The torque the push
        is applying does not go away, though, so the residual against the
        CURRENT bias catches exactly the case the proxy misses.

        Cheap enough to run every tick: one gravity evaluation and a
        threshold compare, no candidate sweep. Returns False when there
        is no bias to judge against, leaving the first calibration to the
        pose proxy alone.
        """
        if not self._est.calibrated or self._q is None or self._tau is None:
            return False
        dq = self._dq if self._is_moving() else None
        tau_ext = self._est.residual(self._q, self._tau, dq=dq)
        link, _ = support_link(tau_ext, self._est.calibration.thresholds)
        return link > 0

    def _update_baseline(self):
        """Adopt the steady-state tracking offset once it stops changing.

        An impedance arm never reaches its commanded pose: it settles
        where ``K*(q_target - q_current)`` balances the gravity-model and
        friction torque. That offset is large against the trigger's
        tolerance (measured up to 0.070 rad here, against 0.02) and it is
        pose-dependent, so neither an absolute test nor a frozen baseline
        works. What is reliable is that on a still, untouched arm the
        offset does not move at all -- 0.00000 rad peak-to-peak over 10 s,
        every joint -- while a push changes it. So the offset is adopted
        as the baseline only once it has held steady.
        """
        if self._q is None or self._dq is None:
            return          # never before real velocity data; see above
        ref = choose_reference_pose(
            self._q_target, self._q_desired, expected=self._n,
            active=self._q_target_active)
        if ref is None or len(ref) != self._n:
            return
        offset = angular_difference(
            self._q, np.asarray(ref[:self._n], dtype=float))
        eps = float(self.get_parameter(
            'auto_calibrate_baseline_epsilon').value)
        if (self._offset_prev is not None
                and np.max(np.abs(angular_difference(
                    offset, self._offset_prev))) <= eps):
            self._offset_stable += 1
        else:
            self._offset_stable = 0
        self._offset_prev = offset
        need = int(self.RATE_HZ * float(self.get_parameter(
            'auto_calibrate_baseline_stable_seconds').value))
        if self._offset_stable >= need:
            self._offset_stable = 0
            self._autocal.set_offset_reference(offset)
            self.get_logger().info(
                'auto-calibration baseline: steady-state tracking offset '
                '%s rad (max %.4f) held steady, adopted.'
                % (np.round(offset, 4).tolist(),
                   float(np.max(np.abs(offset)))),
                throttle_duration_sec=10.0)

    def _confidence(self, confidence):
        """Derate a raw ``support_link`` confidence for motion and drift.

        Two independent, multiplicative penalties, each a no-op at rest
        with a fresh calibration:

        - velocity, via ``derate_confidence_for_motion`` — the residual
          never models the inertial term ``M(q)*ddq`` (no measured joint
          acceleration exists anywhere in this project), so a moving-mode
          estimate is always missing a term a stationary one is not.
        - pose drift, via ``derate_confidence_for_pose_drift`` — once the
          arm has moved far enough that a stationary-mode gate would
          already call the calibration stale, but not so far that this
          node's relaxed ``MOVING_POSE_TOLERANCE`` refuses it outright,
          the bias is running approximate rather than fresh.

        This is how the estimate carries the fact that it is less
        trustworthy while moving, instead of reporting a number a
        downstream consumer would read as equally reliable at rest.
        """
        confidence = derate_confidence_for_motion(
            confidence, self._max_abs_dq(), self.MOVING_DQ_THRESHOLD)
        confidence = derate_confidence_for_pose_drift(
            confidence, self._est.pose_drift(self._q),
            self._est.pose_tolerance, self.MOVING_POSE_TOLERANCE)
        return confidence

    def _estimate(self):
        """Return ``(link_index, confidence)`` for the current sample.

        Assumes the gate has already been checked; callers that skip the
        gate check will get a link/confidence pair for whatever ``_q`` and
        ``_tau`` currently hold, calibrated or not. Caches the residual
        used to reach that answer in ``self._last_tau_ext`` so ``_tick``
        need not recompute it for the published message. Below
        ``MOVING_DQ_THRESHOLD`` the arm is treated as stationary and the
        residual uses its gravity-only path unchanged; above it, the
        measured ``dq`` is passed through so the residual accounts for
        Coriolis/centrifugal torque instead of attributing it to contact.

        The torque residual is computed the same way in every mode,
        including admittance — see ``fic_contact.admittance`` for why
        this deliberately does NOT reuse ``{rid}_trajectory_data``'s
        desired/measured-effort blocks for this part. In joint
        admittance modes (3, 4) this is then combined with the
        ``{rid}_admittance_offset`` observable via
        ``combine_link_estimates``; in every other mode (including
        Cartesian admittance, 7/8, which has no equivalent offset topic)
        the residual alone is returned.
        """
        dq = self._dq if self._is_moving() else None
        self._last_tau_ext = self._est.residual(self._q, self._tau, dq=dq)
        if self.get_parameter('fuse_deflection').value:
            self._last_tau_ext = self._fuse_deflection(self._last_tau_ext)
        link_b, confidence_b = support_link(
            self._last_tau_ext, self._est.calibration.thresholds)
        confidence_b = self._confidence(confidence_b)

        if (self._mode not in self.JOINT_ADMITTANCE_MODES
                or self._admittance_offset is None):
            return link_b, confidence_b

        link_a, confidence_a = admitting_link(
            self._admittance_offset, self.ADMITTANCE_OFFSET_THRESHOLD_RAD)
        confidence_a = self._confidence(confidence_a)

        return combine_link_estimates(link_a, confidence_a, link_b, confidence_b)

    def _fuse_deflection(self, tau_torque):
        """Blend the torque residual with the deflection channel.

        Returns ``tau_torque`` unchanged whenever the second channel is
        unavailable or uncalibrated -- degrading to the single-channel
        estimator rather than to a worse two-channel one.
        """
        defl = self._deflection_tau()
        if defl is None or self._defl_bias is None:
            return tau_torque
        defl = defl - self._defl_bias
        if self.get_parameter('fusion_mode').value == 'equal':
            return 0.5 * (np.asarray(tau_torque, dtype=float) + defl)
        sigma_torque = float(np.mean(self._est.calibration.thresholds)) / 6.0
        fused, _ = fuse_inverse_variance(
            [tau_torque, defl], [max(sigma_torque, 1e-6), self._defl_sigma])
        return tau_torque if fused is None else fused

    def _fit_v2(self, link):
        """Refine a reported contact into a push-constrained point/force fit.

        Only ever called once ``_estimate`` has already reported a
        contact (``link > 0``) — this is the cheap-gate-first ordering
        the ``estimator`` parameter's 'v2' mode requires: ``fit_contact``
        sweeps every link's ~200 decimated surface candidates, and paying
        that cost on every quiet 100 Hz tick would waste the budget for
        nothing. Returns ``(link, force, point, residual_norm)``, where
        ``link`` may differ from the input — ``fit_contact`` can name a
        more distal link than the joint-support heuristic alone would —
        or ``(link, None, None, None)`` unchanged if no push-consistent
        fit exists (the residual clears ``support_link``'s per-joint
        threshold but is still below ``fit_contact``'s own zero-residual
        floor, or every candidate would require pulling on the arm).

        Frame note, since getting this wrong would silently mislocate the
        twin's marker: ``fit_contact``'s force is already expressed in
        the same ``LOCAL_WORLD_ALIGNED`` axes ``ArmModel.link_jacobians``
        uses, and this arm's fixed base makes those axes the base frame
        too, so the force needs no rotation. Its point is in the
        contacted link's own BODY frame (see
        ``geometry.load_candidates``) and is carried into the base frame
        explicitly here, via that link's current pose from
        ``ArmModel.link_poses``.

        LIMIT this does not remove: with ``n`` torque sensors a contact's
        point is only determined up to the force's line of action, so
        ``point`` is a candidate on that line, not a uniquely located
        contact — see ``fit_contact``'s own docstring.
        """
        jac_q = self._model.link_jacobians(self._q)
        # Scaled by the calibration's own thresholds, so the rule tightens
        # to nothing on clean data and only bites where noise really can
        # swap adjacent links.
        tol_nm = float(self.get_parameter('parsimony_k').value) * float(
            np.mean(self._est.calibration.thresholds))
        hyp = fit_contact(self._last_tau_ext, jac_q, self._candidates,
                          parsimony_tol_nm=tol_nm)
        if hyp is None:
            return link, None, None, None
        position, rotation = self._model.link_poses(self._q)[hyp.link]
        point_base = rotation @ hyp.point + position
        return hyp.link, hyp.force, point_base, hyp.residual

    def _tick(self):
        """Estimate and publish once."""
        have = self._q is not None and self._tau is not None
        self._gate.set_feedback(have)

        if self._calibrating and have:
            if calibration_disturbed(
                    self._q, self._cal_q0, moving=self._is_moving(),
                    started_moving=self._cal_started_moving,
                    tolerance=float(self.get_parameter(
                        'auto_calibrate_tolerance').value)):
                self._abort_calibration(
                    'the arm moved during the window -- touched?')
            else:
                # Match the dq regime residual() will later be called
                # under (see _estimate), so a calibration taken while
                # moving removes exactly the model term the moving-mode
                # residual adds back.
                dq = self._dq if self._is_moving() else None
                self._est.accumulate(self._q, self._tau, dq=dq)
                # The deflection channel needs its own bias, measured in
                # the same contact-free window: what it captures there is
                # the negated gravity model error at this pose.
                defl = self._deflection_tau()
                if defl is not None:
                    self._defl_samples.append(defl)
                self._cal_ticks -= 1
                if self._cal_ticks <= 0:
                    self._finish_calibration()
            # Deliberately NO early return. start_calibration() does not
            # clear the existing Calibration -- only finish_calibration()
            # replaces it -- so the PREVIOUS bias stays valid for the
            # whole window and there is no reason to go silent. Returning
            # here left the estimator publishing nothing for 1.0 s out of
            # every 3.49 s: 30% blind, and a 1.02 s hole in the twin.
            # Falling through publishes on the previous bias instead; the
            # very first calibration has no predecessor, and there the
            # gate below correctly reports 'not calibrated' (-1, no
            # statement) rather than guessing.

        self._gate.set_calibrated(self._est.calibrated)
        if have and self._est.calibrated:
            # Relax the staleness tolerance while moving (modes 2, 6): the
            # arm leaves the calibration pose almost immediately there, and
            # the estimator's default tolerance would refuse constantly.
            # The tradeoff this accepts is carried in the published
            # confidence instead — see _confidence. At rest the tolerance
            # is the stale_pose_tolerance parameter, read every tick.
            tol = (self.MOVING_POSE_TOLERANCE if self._is_moving()
                   else float(self.get_parameter(
                       'stale_pose_tolerance').value))
            self._gate.set_stale(self._est.is_stale(self._q, pose_tolerance=tol))
            stale = self._est.is_stale(self._q, pose_tolerance=tol)
        else:
            stale = False

        # Maintain the auto-calibration baseline: the steady-state
        # tracking offset an impedance arm settles at, which the trigger
        # measures CHANGES against. Re-seeded whenever the offset holds
        # steady, not captured once: a one-shot seed froze a transient
        # (dq is None at startup, so the arm reads as 'still' before any
        # velocity has arrived) and, being pose-dependent, went wrong
        # again at every new pose. See the parameter comments.
        self._update_baseline()

        # Auto-recalibrate before the gate is read, so a bias measured on
        # this tick is usable on the next one rather than a tick later.
        # Placed after the _calibrating block above, which returns early:
        # a window in progress is never re-armed.
        if self.get_parameter('auto_calibrate').value and have:
            if self._autocal.update(
                    mode=self._mode,
                    q=list(self._q),
                    q_desired=choose_reference_pose(
                        self._q_target, self._q_desired, expected=self._n,
                        active=self._q_target_active),
                    moving=self._is_moving(),
                    calibrated=self._est.calibrated,
                    stale=stale,
                    calibrating=self._calibrating,
                    touched=(self._touched()
                             if self.get_parameter(
                                 'auto_calibrate_touch_veto').value
                             else False)):
                self._begin_calibration()
                self.get_logger().info(
                    'auto-calibrating: arm settled on its commanded pose '
                    'in mode %s -- do not touch the arm' % self._mode)
                self._debounce.reset()
                return

        msg = ContactEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.calibrated = self._est.calibrated

        reason = self._gate.blocked_reason()
        if reason is not None:
            # A blocked tick is indistinguishable from a quiet one on the
            # wire -- both publish link_index -1 -- so without this the
            # only symptom of an ungated cause (uncalibrated, no mode
            # yet) is a twin that never highlights and says nothing.
            # Throttled: this runs at 100 Hz.
            self.get_logger().warn(
                'contact estimation blocked: %s' % reason,
                throttle_duration_sec=5.0)
            msg.link_index = -1
            msg.link_name = ''
            self._pub.publish(msg)
            self._add_record_row()
            return

        link, confidence = self._estimate()

        # Cheap gate first: _estimate() above is exactly support_link (plus,
        # in joint admittance modes, the admittance-offset cross-check) —
        # fit_contact only ever runs once that has already reported a
        # contact, never on a quiet tick. 'v1' skips this block entirely,
        # leaving force/point/residual_norm at the message's empty default.
        force = point = residual_norm = None
        if link > 0 and self.get_parameter('estimator').value == 'v2':
            link, force, point, residual_norm = self._fit_v2(link)

        # Debounce the reported link: fit_contact above always
        # runs on the raw, undebounced estimate -- refining an active
        # contact must not wait on hysteresis -- but what gets published
        # and highlighted is the debounced value, so a momentary dropout
        # does not flicker. raw_link is kept for link_est_raw below.
        raw_link = link
        link = self._debounce.update(link)

        msg.tau_ext = self._last_tau_ext.tolist()
        msg.link_index = link
        msg.link_name = self._model.link_names[link - 1] if link > 0 else ''
        msg.confidence = confidence
        if force is not None:
            msg.force = force.tolist()
        if point is not None:
            msg.point = point.tolist()
        if residual_norm is not None:
            msg.residual_norm = residual_norm
        self._pub.publish(msg)
        # link_est is the debounced, published estimate; link_est_raw is
        # the pre-debounce value, logged alongside it so the false-positive
        # rate stays measurable rather than hidden behind the debouncer.
        self._add_record_row(
            link_est=link, confidence=confidence, link_est_raw=raw_link)


def main(args=None):
    """Spin the contact localization node."""
    rclpy.init(args=args)
    node = ContactNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
