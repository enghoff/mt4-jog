"""Envelope bound helpers (no hardware)."""

from __future__ import annotations

import unittest

from mt4_jog.joints import (
    GROUND_Z_MM,
    J2_J3_SUM_MAX_STEPS,
    J2_J3_SUM_MIN_STEPS,
    JOINT_SOFT_MAX_STEPS,
    JOINT_SOFT_MIN_STEPS,
)
from mt4_vision.workspace import (
    MAX_REACH_MM,
    is_within_envelope,
    joints_within_soft_limits,
)


class EnvelopeBoundTests(unittest.TestCase):
    def test_ground_and_reach(self) -> None:
        self.assertTrue(is_within_envelope(250.0, 0.0, GROUND_Z_MM))
        self.assertFalse(is_within_envelope(250.0, 0.0, GROUND_Z_MM - 1.0))
        self.assertFalse(is_within_envelope(MAX_REACH_MM + 5.0, 0.0, 180.0))
        self.assertFalse(is_within_envelope(100.0, 0.0, 180.0))  # keep-out

    def test_joint_soft_limits(self) -> None:
        mid = tuple(
            (lo + hi) // 2
            for lo, hi in zip(JOINT_SOFT_MIN_STEPS, JOINT_SOFT_MAX_STEPS)
        )
        self.assertTrue(joints_within_soft_limits(mid))
        lo = list(JOINT_SOFT_MIN_STEPS)
        lo[0] -= 1
        self.assertFalse(joints_within_soft_limits(lo))
        hi = list(JOINT_SOFT_MAX_STEPS)
        hi[2] += 1
        self.assertFalse(joints_within_soft_limits(hi))

    def test_j2_j3_sum_extension(self) -> None:
        # Full-stretch in-sample #11 (park-zero j2=2922 j3=-12 sum=2910),
        # shifted +1000/+500 into the limit-referenced frame — allowed.
        self.assertTrue(joints_within_soft_limits((0, 3922, 488, 0)))
        # Over-extension out #12: old sum=3108 → 4608 — rejected by sum.
        self.assertFalse(joints_within_soft_limits((0, 4031, 577, 0)))
        # Out #3: j2/j3 each inside box, old sum=3085 → 4585 — sum catches it.
        self.assertTrue(
            JOINT_SOFT_MIN_STEPS[1] <= 3376 <= JOINT_SOFT_MAX_STEPS[1]
        )
        self.assertTrue(
            JOINT_SOFT_MIN_STEPS[2] <= 1209 <= JOINT_SOFT_MAX_STEPS[2]
        )
        self.assertFalse(joints_within_soft_limits((0, 3376, 1209, 0)))

    def test_measured_constants(self) -> None:
        self.assertAlmostEqual(GROUND_Z_MM, 115.0)
        self.assertAlmostEqual(MAX_REACH_MM, 350.0)
        self.assertEqual(JOINT_SOFT_MIN_STEPS[1], 0)
        self.assertEqual(JOINT_SOFT_MAX_STEPS[1], 3950)
        self.assertEqual(JOINT_SOFT_MAX_STEPS[0], 4580)
        self.assertEqual(JOINT_SOFT_MIN_STEPS[2], -1550)
        self.assertEqual(J2_J3_SUM_MAX_STEPS, 4410)
        self.assertEqual(J2_J3_SUM_MIN_STEPS, -200)


if __name__ == "__main__":
    unittest.main()
