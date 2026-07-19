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
        # Full-stretch in-sample #11: j2=2922 j3=-12 sum=2910 — allowed.
        self.assertTrue(joints_within_soft_limits((0, 2922, -12, 0)))
        # Over-extension out #12: sum=3108 — rejected by sum even if box ok.
        self.assertFalse(joints_within_soft_limits((0, 3031, 77, 0)))
        # Out #3: j2/j3 each inside box, sum=3085 — sum catches it.
        self.assertTrue(
            JOINT_SOFT_MIN_STEPS[1] <= 2376 <= JOINT_SOFT_MAX_STEPS[1]
        )
        self.assertTrue(
            JOINT_SOFT_MIN_STEPS[2] <= 709 <= JOINT_SOFT_MAX_STEPS[2]
        )
        self.assertFalse(joints_within_soft_limits((0, 2376, 709, 0)))

    def test_measured_constants(self) -> None:
        self.assertAlmostEqual(GROUND_Z_MM, 136.0)
        self.assertAlmostEqual(MAX_REACH_MM, 350.0)
        self.assertEqual(JOINT_SOFT_MIN_STEPS[1], -1000)
        self.assertEqual(JOINT_SOFT_MAX_STEPS[1], 2950)
        self.assertEqual(JOINT_SOFT_MAX_STEPS[0], 4580)
        self.assertEqual(JOINT_SOFT_MIN_STEPS[2], -2050)
        self.assertEqual(J2_J3_SUM_MAX_STEPS, 2910)
        self.assertEqual(J2_J3_SUM_MIN_STEPS, -1700)


if __name__ == "__main__":
    unittest.main()
