"""
Leg kinematics for the locomotion stack, one LegModel per leg design.

Pure `math` only — no ROS, no numpy — so these run in a bare pytest and are
cheap enough for a 200 Hz control loop.

Geometry is transcribed from the MuJoCo Menagerie Spot MJCF
(`menagerie/model/Spot/xml/spot.xml`); the front-left leg reads::

    <body name="fl_hip" pos="0.29785 0.055 0">      hip origin
      <joint name="fl_hx" axis="1 0 0"/>            roll
      <body name="fl_uleg" pos="0 0.1108 0">
        <joint name="fl_hy" axis="0 1 0"/>          thigh pitch
        <body name="fl_lleg" pos="0.025 0 -0.32">
          <joint name="fl_kn" axis="0 1 0"/>        knee
          <geom name="FL" class="foot" pos="0 0 -0.3365"/>

All foot positions are in the trunk frame: +x forward, +y left, +z up.

A `LegModel` bundles everything the gait stack needs to know about one leg
design: the four legs' mounting offsets, the joint-name suffixes, the limits,
the stand and sit poses, and the forward/inverse kinematics themselves. `SPOT`
is the serial hip-roll / hip-pitch / knee design this module started as, and
the module-level `JOINT_NAMES`, `forward_kinematics` and friends are aliases
onto it, so every existing caller keeps working unchanged. A robot with
different kinematics supplies its own `LegModel` instead of editing this file.
"""

import math
from dataclasses import dataclass
from typing import Callable, Sequence, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class LegGeometry:
    """Constant link geometry for a single leg."""

    name: str
    hip_offset: Vec3
    side: float  # +1 left, -1 right
    #: Per-leg stand/sit override, for designs whose legs are not all mounted
    #: the same way round. None means use the model's shared pose.
    stand_q: object = None
    sit_q: object = None
    y_offset: float = 0.1108
    thigh_x: float = 0.025
    thigh_z: float = 0.32
    shank: float = 0.3365
    #: Hip-centre to foot vector in the hip frame, for fixed-length legs that
    #: have no knee. Unused by the serial model.
    femur: Vec3 = (0.0, 0.0, 0.0)

    @property
    def thigh(self) -> float:
        """Straight-line hip-pitch to knee distance."""
        return math.hypot(self.thigh_x, self.thigh_z)

    @property
    def thigh_skew(self) -> float:
        """Angle the thigh link is offset from the leg's own -z axis."""
        return math.atan2(self.thigh_x, self.thigh_z)


SPOT_LEGS = (
    LegGeometry('fl', (0.29785, 0.055, 0.0), 1.0),
    LegGeometry('fr', (0.29785, -0.055, 0.0), -1.0),
    LegGeometry('hl', (-0.29785, 0.055, 0.0), 1.0),
    LegGeometry('hr', (-0.29785, -0.055, 0.0), -1.0),
)

JOINT_NAMES = tuple(
    '%s_%s' % (leg.name, joint)
    for leg in SPOT_LEGS
    for joint in ('hx', 'hy', 'kn'))

# Per-joint (lower, upper) in radians, verbatim from the MJCF joint ranges.
# The knee and thigh upper bounds differ very slightly between legs upstream.
JOINT_LIMITS = (
    (-0.785398, 0.785398), (-0.898845, 2.29511), (-2.7929, -0.254402),
    (-0.785398, 0.785398), (-0.898845, 2.24363), (-2.7929, -0.255648),
    (-0.785398, 0.785398), (-0.898845, 2.29511), (-2.7929, -0.247067),
    (-0.785398, 0.785398), (-0.898845, 2.29511), (-2.7929, -0.248282),
)

# The MJCF `home` keyframe pose, per leg.
STAND_Q = (0.0, 1.04, -1.8)
# Tucked pose, inside every leg's range including the tighter front-right one.
SIT_Q = (0.0, 2.2, -2.65)


def serial_forward_kinematics(leg: LegGeometry, q: Sequence[float]) -> Vec3:
    """Foot-centre position in the trunk frame for joint angles `q`."""
    q1, q2, q3 = q
    # Solve in the leg's sagittal plane first, i.e. with hip roll undone.
    vx = (leg.thigh_x * math.cos(q2) - leg.thigh_z * math.sin(q2)
          - leg.shank * math.sin(q2 + q3))
    vy = leg.side * leg.y_offset
    vz = (-leg.thigh_x * math.sin(q2) - leg.thigh_z * math.cos(q2)
          - leg.shank * math.cos(q2 + q3))
    # Roll the sagittal solution back out about the leg's +x axis.
    cos_r, sin_r = math.cos(q1), math.sin(q1)
    hip_x, hip_y, hip_z = leg.hip_offset
    return (hip_x + vx,
            hip_y + vy * cos_r - vz * sin_r,
            hip_z + vy * sin_r + vz * cos_r)


def serial_inverse_kinematics(leg: LegGeometry, foot: Vec3) -> Tuple[float, float, float]:
    """
    Joint angles placing the foot centre at `foot` in the trunk frame.

    Targets outside the reachable envelope are projected onto it rather than
    raising: a gait controller must always produce some command, and a clamped
    target is far safer than an exception inside a 200 Hz loop.

    The returned angles are NOT checked against `JOINT_LIMITS`. A target close
    enough to the hip (inside `|thigh - shank|`) solves to a geometrically
    valid knee bend that lies well outside the physical joint range. Callers
    must run the result through `clamp_to_limits` (as part of the full
    12-joint vector) before sending it to any actuator.

    Only the below-hip knee-bend branch is solved: whenever a foot position is
    reachable via two joint-angle solutions (a below-hip combination and its
    above-hip mirror), the below-hip one is always returned, which is the
    branch a standing robot wants. Feeding a `forward_kinematics` output whose
    source angles used the above-hip branch back into this function therefore
    reproduces the same foot position through a different — but equally
    valid — set of angles, not the original ones.
    """
    px = foot[0] - leg.hip_offset[0]
    py = foot[1] - leg.hip_offset[1]
    pz = foot[2] - leg.hip_offset[2]

    # --- Hip roll ------------------------------------------------------
    # The foot is rigidly offset `side * y_offset` from the roll axis, so the
    # y-z projection of any reachable target lies on a cylinder of that radius.
    offset = leg.side * leg.y_offset
    radius_sq = py * py + pz * pz
    lateral = math.sqrt(max(radius_sq - offset * offset, 0.0))
    q1 = _wrap(math.atan2(pz, py) - math.atan2(-lateral, offset))

    # --- Thigh and knee, in the sagittal plane -------------------------
    # After the roll is removed the foot sits at (px, -lateral): always below
    # the hip, which is the branch a standing robot wants.
    vx, vz = px, -lateral
    thigh, shank = leg.thigh, leg.shank
    reach_sq = vx * vx + vz * vz
    cos_knee = (reach_sq - thigh * thigh - shank * shank) / (2.0 * thigh * shank)
    knee = math.acos(max(-1.0, min(1.0, cos_knee)))

    q3 = -knee - leg.thigh_skew
    bearing = math.atan2(vx, -vz)
    interior = math.atan2(shank * math.sin(knee), thigh + shank * math.cos(knee))
    q2 = leg.thigh_skew - (bearing - interior)
    return (q1, q2, q3)


@dataclass(frozen=True)
class LegModel:
    """
    One complete leg design: geometry, naming, limits, poses and kinematics.

    Everything the gait stack needs that differs between robots lives here, so
    supporting a new machine means constructing one of these rather than
    editing the controller. `joint_limits` is the FULL flat vector, not
    per-leg: Spot's knee bounds differ slightly between legs upstream.
    """

    name: str
    legs: Tuple[LegGeometry, ...]
    joint_suffixes: Tuple[str, ...]
    joint_limits: Tuple[Tuple[float, float], ...]
    stand_q: Tuple[float, ...]
    sit_q: Tuple[float, ...]
    forward_kinematics: Callable[[LegGeometry, Sequence[float]], Vec3]
    inverse_kinematics: Callable[[LegGeometry, Vec3], Tuple[float, ...]]

    @property
    def joints_per_leg(self) -> int:
        """Return how many joints each leg carries."""
        return len(self.joint_suffixes)

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Return every joint name, leg-major, in controller order."""
        return tuple('%s_%s' % (leg.name, j)
                     for leg in self.legs for j in self.joint_suffixes)

    def stand_for(self, leg: LegGeometry) -> Sequence[float]:
        """Return the standing joint vector for one leg."""
        return leg.stand_q if leg.stand_q is not None else self.stand_q

    def sit_for(self, leg: LegGeometry) -> Sequence[float]:
        """Return the tucked joint vector for one leg."""
        return leg.sit_q if leg.sit_q is not None else self.sit_q

    def nominal_stance(self, leg: LegGeometry) -> Vec3:
        """Return the default standing foot position for one leg."""
        return self.forward_kinematics(leg, self.stand_for(leg))

    def clamp_to_limits(self, targets: Sequence[float]) -> list:
        """Clamp a full joint vector to this model's joint ranges."""
        return [max(lo, min(hi, q))
                for q, (lo, hi) in zip(targets, self.joint_limits)]

    def pose(self, per_leg_q: Sequence[float]) -> list:
        """Repeat a single leg's joint vector across every leg."""
        return list(per_leg_q) * len(self.legs)

    def stand_pose(self) -> list:
        """Return the full joint vector that stands every leg."""
        return [q for leg in self.legs for q in self.stand_for(leg)]

    def sit_pose(self) -> list:
        """Return the full joint vector that tucks every leg."""
        return [q for leg in self.legs for q in self.sit_for(leg)]


# ── A rigid leg on a 3-DoF spherical hip ─────────────────────────────────────
# leg_spherical has no knee: the hip carries three coincident orthogonal hinges
# and the femur is rigid, so the foot rides a sphere of fixed radius about the
# hip centre. Three joints aim a fixed-length vector, which is two degrees of
# freedom of foot POSITION plus one redundant twist about the hip-to-foot axis.
#
# Verified against the MJCF over 40 random poses, max error 0.0000 mm: the hip
# composes as Rx(q1) @ Ry(q2) @ Rz(q3) acting on the femur vector.
#
# The practical consequence for a gait: leg length cannot change, so stance
# height is set by how far out the foot is planted rather than commanded
# independently. Targets off the sphere are projected onto it.


def _rot(axis: str, angle: float):
    """Return a 3x3 rotation about one principal axis, as nested tuples."""
    c, s = math.cos(angle), math.sin(angle)
    if axis == 'x':
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == 'y':
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _matmul(a, b):
    """Multiply two 3x3 matrices."""
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def _apply(m, v: Vec3) -> Vec3:
    """Apply a 3x3 matrix to a 3-vector."""
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def spherical_forward_kinematics(leg: LegGeometry, q: Sequence[float]) -> Vec3:
    """Foot-centre position in the trunk frame for hip angles `q`."""
    rot = _matmul(_matmul(_rot('x', q[0]), _rot('y', q[1])), _rot('z', q[2]))
    v = _apply(rot, leg.femur)
    return tuple(leg.hip_offset[i] + v[i] for i in range(3))


def spherical_inverse_kinematics(leg: LegGeometry, foot: Vec3) -> Tuple[float, float, float]:
    """
    Hip angles aiming the rigid femur at `foot`.

    The leg cannot change length, so only the DIRECTION from hip to target is
    achievable: anything nearer or further is projected onto the sphere. Of the
    one-parameter family of rotations that aim the femur correctly, the minimal
    one is chosen (zero twist about the hip-to-foot axis), which keeps the hip
    away from its limits and makes the solution unique.
    """
    target = tuple(foot[i] - leg.hip_offset[i] for i in range(3))
    v0 = leg.femur
    n0 = math.sqrt(sum(c * c for c in v0))
    nt = math.sqrt(sum(c * c for c in target))
    if n0 < 1e-12 or nt < 1e-12:
        return (0.0, 0.0, 0.0)
    a = tuple(c / n0 for c in v0)
    b = tuple(c / nt for c in target)

    # Minimal rotation carrying `a` onto `b`, via Rodrigues.
    axis = (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])
    sin_t = math.sqrt(sum(c * c for c in axis))
    cos_t = max(-1.0, min(1.0, sum(a[i] * b[i] for i in range(3))))
    if sin_t < 1e-12:
        if cos_t > 0.0:
            return (0.0, 0.0, 0.0)
        # Antipodal: any perpendicular axis turns it through pi.
        perp = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
        axis = (a[1] * perp[2] - a[2] * perp[1],
                a[2] * perp[0] - a[0] * perp[2],
                a[0] * perp[1] - a[1] * perp[0])
        sin_t = math.sqrt(sum(c * c for c in axis))
        angle = math.pi
    else:
        angle = math.atan2(sin_t, cos_t)
    k = tuple(c / sin_t for c in axis)
    ct, st, vt = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    rot = (
        (ct + k[0] * k[0] * vt, k[0] * k[1] * vt - k[2] * st, k[0] * k[2] * vt + k[1] * st),
        (k[1] * k[0] * vt + k[2] * st, ct + k[1] * k[1] * vt, k[1] * k[2] * vt - k[0] * st),
        (k[2] * k[0] * vt - k[1] * st, k[2] * k[1] * vt + k[0] * st, ct + k[2] * k[2] * vt),
    )
    # Decompose that rotation into the hip's own Rx @ Ry @ Rz order.
    q2 = math.asin(max(-1.0, min(1.0, rot[0][2])))
    if abs(rot[0][2]) < 1.0 - 1e-9:
        q1 = math.atan2(-rot[1][2], rot[2][2])
        q3 = math.atan2(-rot[0][1], rot[0][0])
    else:                                   # gimbal lock: fold the twist into q1
        q1 = math.atan2(rot[2][1], rot[1][1])
        q3 = 0.0
    return (q1, q2, q3)


#: The serial roll/pitch/knee design this module was written for.
SPOT = LegModel(
    name='spot',
    legs=SPOT_LEGS,
    joint_suffixes=('hx', 'hy', 'kn'),
    joint_limits=JOINT_LIMITS,
    stand_q=STAND_Q,
    sit_q=SIT_Q,
    forward_kinematics=serial_forward_kinematics,
    inverse_kinematics=serial_inverse_kinematics,
)

#: leg_spherical's rigid femur, hip centre to foot, measured from the MJCF.
SPHERICAL_FEMUR = (-0.12186, 0.00065, -0.38107)

#: Four leg_spherical legs on a trunk. The femur mirrors in y for the right
#: side; its x component is the CAD leg's own backward rake, kept as built.
#: The same femur on a leg turned 180 deg about z, which is how build_quad.py
#: mounts the hind pair so they rake forward rather than aft. Rz(180) negates x
#: and y: verified against the generated MJCF at 0.0000 mm over 200 poses, where
#: the unturned vector is out by up to 243.72 mm.
SPHERICAL_FEMUR_TURNED = (-SPHERICAL_FEMUR[0], -SPHERICAL_FEMUR[1], SPHERICAL_FEMUR[2])

#: Four leg_spherical legs on a trunk. All four copies are IDENTICAL - the
#: generator mirrors nothing left-to-right - so fl and fr share one femur. The
#: hind pair is mounted turned, so it carries the turned femur and the mirrored
#: stand and sit poses.
_STAND_F = (-0.001706, -0.309506, -0.000266)
_SIT_F = (-0.002371, -0.746132, -0.001720)
_STAND_H = (-_STAND_F[0], -_STAND_F[1], _STAND_F[2])
_SIT_H = (-_SIT_F[0], -_SIT_F[1], _SIT_F[2])

SPHERICAL_QUAD_LEGS = (
    LegGeometry('fl', (0.30, 0.15, 0.0), 1.0, femur=SPHERICAL_FEMUR,
                stand_q=_STAND_F, sit_q=_SIT_F),
    LegGeometry('fr', (0.30, -0.15, 0.0), -1.0, femur=SPHERICAL_FEMUR,
                stand_q=_STAND_F, sit_q=_SIT_F),
    LegGeometry('hl', (-0.30, 0.15, 0.0), 1.0, femur=SPHERICAL_FEMUR_TURNED,
                stand_q=_STAND_H, sit_q=_SIT_H),
    LegGeometry('hr', (-0.30, -0.15, 0.0), -1.0, femur=SPHERICAL_FEMUR_TURNED,
                stand_q=_STAND_H, sit_q=_SIT_H),
)

#: Hip ranges verbatim from the leg_spherical MJCF: +-90 deg on x and y, and
#: +-180 on the twist axis.
SPHERICAL_LIMITS = ((-1.570796, 1.570796), (-1.570796, 1.570796),
                    (-3.141593, 3.141593)) * 4

#: Foot straight below the hip - 400.1 mm, the full femur length, because a
#: rigid leg has only one stand height per foot direction.
SPHERICAL_STAND_Q = (-0.001706, -0.309506, -0.000266)
#: Foot drawn forward and up, 362.5 mm below the hip.
SPHERICAL_SIT_Q = (-0.002371, -0.746132, -0.001720)

#: Four leg_spherical legs driven as a quadruped.
SPHERICAL_QUAD = LegModel(
    name='leg_spherical_quad',
    legs=SPHERICAL_QUAD_LEGS,
    joint_suffixes=('hx', 'hy', 'hz'),
    joint_limits=SPHERICAL_LIMITS,
    stand_q=SPHERICAL_STAND_Q,
    sit_q=SPHERICAL_SIT_Q,
    forward_kinematics=spherical_forward_kinematics,
    inverse_kinematics=spherical_inverse_kinematics,
)

#: Every leg design the locomotion stack can drive, by name.
MODELS = {SPOT.name: SPOT, SPHERICAL_QUAD.name: SPHERICAL_QUAD}


def model_by_name(name: str) -> LegModel:
    """Look up a leg design by name, defaulting to Spot."""
    return MODELS.get(name, SPOT)


#: Default model, and the module-level aliases every existing caller imports.
DEFAULT_MODEL = SPOT
forward_kinematics = serial_forward_kinematics
inverse_kinematics = serial_inverse_kinematics


def nominal_stance(leg: LegGeometry) -> Vec3:
    """Return the default standing foot position, the MJCF `home` keyframe pose."""
    return SPOT.nominal_stance(leg)


def clamp_to_limits(targets: Sequence[float]) -> list:
    """Clamp twelve joint targets to the model's joint ranges."""
    return SPOT.clamp_to_limits(targets)


def _wrap(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
