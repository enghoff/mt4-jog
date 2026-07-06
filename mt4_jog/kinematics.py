"""MT4 forward kinematics and Cartesian jog mixing (matches jog firmware geometry)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Factory EEPROM geometry (mm). The MT4 is a parallel-link (palletizing) arm:
# J2 sets the upper-arm absolute angle, J3 sets the forearm absolute angle via
# the link rods (independent of J2), and the head platform stays level.
LINKAGE1 = 130.0  # shoulder -> elbow
LINKAGE2 = 150.0  # elbow -> wrist pivot
CENCER_OFFSET = 45.0  # J1 axis -> shoulder, horizontal
CENCER_HEIGHT = 140.0  # shoulder pivot height
HEAD_OFFSET = 35.0  # wrist pivot -> TCP, horizontal (head stays level)
HEAD_HEIGHT = 14.43  # TCP below wrist pivot

# Model angles at the homed pose (firmware step counters = 0): upper arm
# vertical, forearm horizontal — factory firmware reports TCP (230, 0, 255.57).
HOME_J1_DEG = 0.0
HOME_J2_DEG = 90.0
HOME_J3_DEG = 0.0
HOME_J4_DEG = 0.0

STEPS_PER_DEG: tuple[float, float, float, float] = (44.001, 35.556, 35.556, 852.0)

# +1 if positive step count increases firmware joint angle, -1 if driver is inverted.
# J3 confirmed inverted 2026-07-06: a +299-step probe raised the forearm tip
# instead of lowering it as the old -1.0 sign predicted (photo-confirmed).
J_STEP_SIGN: tuple[float, float, float, float] = (1.0, -1.0, 1.0, 1.0)

DLS_LAMBDA = 0.05

# Positive joint angle => DIR pin low (matches jog_keyboard.py "q" direction).
DIR_POS_HIGH: tuple[bool, bool, bool, bool] = (False, False, False, False)


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def normalized(self) -> Vec3:
        n = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        if n < 1e-9:
            return Vec3(0.0, 0.0, 0.0)
        return Vec3(self.x / n, self.y / n, self.z / n)


@dataclass(frozen=True)
class JointAnglesDeg:
    j1: float
    j2: float
    j3: float
    j4: float = 0.0

    @classmethod
    def from_steps(cls, steps: tuple[int, int, int, int]) -> JointAnglesDeg:
        spd = STEPS_PER_DEG
        s = J_STEP_SIGN
        return cls(
            HOME_J1_DEG + s[0] * steps[0] / spd[0],
            HOME_J2_DEG + s[1] * steps[1] / spd[1],
            HOME_J3_DEG + s[2] * steps[2] / spd[2],
            HOME_J4_DEG + s[3] * steps[3] / spd[3],
        )


def fk_tcp(q: JointAnglesDeg) -> Vec3:
    q1, q2, q3 = map(math.radians, (q.j1, q.j2, q.j3))
    radial = CENCER_OFFSET + LINKAGE1 * math.cos(q2) + LINKAGE2 * math.cos(q3) + HEAD_OFFSET
    return Vec3(
        radial * math.cos(q1),
        radial * math.sin(q1),
        CENCER_HEIGHT + LINKAGE1 * math.sin(q2) + LINKAGE2 * math.sin(q3) - HEAD_HEIGHT,
    )


def jacobian_mm_per_deg(q: JointAnglesDeg) -> np.ndarray:
    """3x4 matrix: d(TCP mm)/d(joint deg)."""
    j = np.zeros((3, 4))
    base = np.array([q.j1, q.j2, q.j3, q.j4], dtype=float)
    tcp0 = fk_tcp(q)
    p0 = np.array([tcp0.x, tcp0.y, tcp0.z])
    for i in range(4):
        trial = base.copy()
        trial[i] += 0.1
        q_t = JointAnglesDeg(*trial)
        p1 = np.array([fk_tcp(q_t).x, fk_tcp(q_t).y, fk_tcp(q_t).z])
        j[:, i] = (p1 - p0) / 0.1
    return j


DEFAULT_ORIENT_GAIN = 0.82  # empirical; tune to match the real J1/J4 coupling


def cartesian_joint_rates_deg(
    q: JointAnglesDeg,
    direction: Vec3,
    *,
    hold_orientation: bool = True,
    orient_gain: float = DEFAULT_ORIENT_GAIN,
) -> tuple[float, float, float, float] | None:
    """Model-space joint deg rates for unit world velocity along direction.

    orient_gain scales the J4 wrist-unwind counter-rotation against J1 yaw;
    it's empirical (real axis alignment / mechanical coupling aren't modeled)
    and matches the firmware's `orient <gain>` serial command default.
    Driver step signs are applied in cartesian_step_rates().
    """
    v = direction.normalized()
    if abs(v.x) + abs(v.y) + abs(v.z) < 1e-9:
        return None

    j_pos = jacobian_mm_per_deg(q)[:, :3]
    jjt = j_pos @ j_pos.T + (DLS_LAMBDA**2) * np.eye(3)
    try:
        y = np.linalg.solve(jjt, np.array([v.x, v.y, v.z]))
        dq123 = j_pos.T @ y
    except np.linalg.LinAlgError:
        return None

    dq4 = 0.0
    if hold_orientation and abs(dq123[0]) > 1e-6:
        dq4 = -orient_gain * dq123[0]

    return float(dq123[0]), float(dq123[1]), float(dq123[2]), dq4


def cartesian_step_rates(
    q: JointAnglesDeg,
    direction: Vec3,
    *,
    hold_orientation: bool = True,
    master_scale: int = 10_000,
) -> tuple[int, int, int, int, int] | None:
    """Bresenham integer rates (j1..j4, master) for firmware cj command."""
    rates = cartesian_joint_rates_deg(q, direction, hold_orientation=hold_orientation)
    if rates is None:
        return None

    steps = [J_STEP_SIGN[i] * rates[i] * STEPS_PER_DEG[i] for i in range(4)]

    # Peak/master-scale is based on the POSITION joints (J1-J3) only. J4's
    # wrist-unwind is a small angular correction, but its steps/deg (852) is
    # ~19x J1's (44), so including it in the peak lets a modest orientation
    # hold dominate the DDA timing budget and throttle the primary motion.
    # J4 is scaled the same as the position joints and clamped to the
    # achievable +/-1-step-per-tick range (best effort).
    peak = max(abs(s) for s in steps[:3])
    if peak < 1e-9:
        return None

    scale = master_scale / peak
    ints = [int(round(s * scale)) for s in steps]
    ints[3] = max(-master_scale, min(master_scale, ints[3]))
    return ints[0], ints[1], ints[2], ints[3], master_scale


def steps_to_dir_high(joint_index: int, rate: int) -> bool:
    positive = rate > 0
    return DIR_POS_HIGH[joint_index] if positive else not DIR_POS_HIGH[joint_index]


# ---------------------------------------------------------------------------
# Position-level IK (closed form) -- for bounded point-to-point moves via the
# firmware `m` command, as opposed to the resolved-rate jog IK above.
# ---------------------------------------------------------------------------


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def ik_q2_q3(
    radial: float, z: float, near_q2: float, near_q3: float
) -> tuple[float, float] | None:
    """Closed-form two-link solve in the arm's vertical plane:

        LINKAGE1*(cos q2, sin q2) + LINKAGE2*(cos q3, sin q3) = target

    (absolute joint angles; circle-circle intersection). Returns the branch
    nearest (near_q2, near_q3), or None if the target is out of reach.
    """
    tx = radial - CENCER_OFFSET - HEAD_OFFSET
    ty = z - CENCER_HEIGHT + HEAD_HEIGHT
    d = math.hypot(tx, ty)
    if (
        d < 1e-6
        or d > LINKAGE1 + LINKAGE2
        or d < abs(LINKAGE1 - LINKAGE2)
    ):
        return None

    cos_alpha = (LINKAGE1**2 + d * d - LINKAGE2**2) / (2 * LINKAGE1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    beta = math.atan2(ty, tx)

    best: tuple[float, float, float] | None = None
    for sign in (1.0, -1.0):
        q2 = math.degrees(beta + sign * alpha)
        p1x = LINKAGE1 * math.cos(math.radians(q2))
        p1y = LINKAGE1 * math.sin(math.radians(q2))
        q3 = math.degrees(math.atan2(ty - p1y, tx - p1x))
        dist = abs(_wrap_deg(q2 - near_q2)) + abs(_wrap_deg(q3 - near_q3))
        if best is None or dist < best[0]:
            best = (dist, q2, q3)
    assert best is not None
    return best[1], best[2]


def ik_position(
    x: float,
    y: float,
    z: float,
    *,
    near: JointAnglesDeg,
    orient_gain: float = DEFAULT_ORIENT_GAIN,
) -> JointAnglesDeg | None:
    """Full position IK: TCP (x, y, z) mm -> joint angles, elbow branch and
    J1 wrap chosen nearest `near`. J4 gets the empirical wrist-unwind
    counter-rotation against the J1 change (same convention as the jog IK);
    pass orient_gain=0 to leave J4 untouched.
    """
    q1 = math.degrees(math.atan2(y, x))
    q1 = near.j1 + _wrap_deg(q1 - near.j1)
    sol = ik_q2_q3(math.hypot(x, y), z, near.j2, near.j3)
    if sol is None:
        return None
    q4 = near.j4 - orient_gain * (q1 - near.j1)
    return JointAnglesDeg(q1, sol[0], sol[1], q4)


def steps_from_angles(q: JointAnglesDeg) -> tuple[int, int, int, int]:
    """Inverse of JointAnglesDeg.from_steps(): absolute step counters."""
    home = (HOME_J1_DEG, HOME_J2_DEG, HOME_J3_DEG, HOME_J4_DEG)
    vals = (q.j1, q.j2, q.j3, q.j4)
    return tuple(
        int(round((vals[i] - home[i]) * STEPS_PER_DEG[i] * J_STEP_SIGN[i]))
        for i in range(4)
    )  # type: ignore[return-value]
