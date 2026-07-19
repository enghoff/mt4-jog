"""Unit tests for gamepad stick → jog speed mapping (no hardware)."""

from __future__ import annotations

import unittest

from jog import SPEED_MAX_US, SPEED_MIN_US, speed_us_from_stick_factor
from mt4_jog.gamepad import THUMB_AXIS_MAX, stick_speed_factor


class StickSpeedFactorTests(unittest.TestCase):
    def test_both_in_deadzone(self) -> None:
        self.assertIsNone(stick_speed_factor(0, 0, 0, 0, deadzone=9000))
        self.assertIsNone(stick_speed_factor(8000, 0, -5000, 0, deadzone=9000))

    def test_single_stick_full_throw(self) -> None:
        self.assertAlmostEqual(
            stick_speed_factor(THUMB_AXIS_MAX, 0, 0, 0, deadzone=9000),
            1.0,
        )
        self.assertAlmostEqual(
            stick_speed_factor(0, 0, 0, -THUMB_AXIS_MAX, deadzone=9000),
            1.0,
        )

    def test_ignores_idle_stick(self) -> None:
        # Right stick idle (deadzone) must not affect the factor.
        factor = stick_speed_factor(THUMB_AXIS_MAX, 0, 0, 0, deadzone=9000)
        self.assertAlmostEqual(factor, 1.0)

    def test_max_of_two_active_sticks(self) -> None:
        dz = 9000
        # Left at full, right halfway past deadzone → max is the left stick.
        half = dz + (THUMB_AXIS_MAX - dz) // 2
        factor = stick_speed_factor(THUMB_AXIS_MAX, 0, half, 0, deadzone=dz)
        self.assertAlmostEqual(factor, 1.0)

    def test_just_past_deadzone_is_near_zero(self) -> None:
        dz = 9000
        factor = stick_speed_factor(dz + 1, 0, 0, 0, deadzone=dz)
        self.assertIsNotNone(factor)
        assert factor is not None
        self.assertLess(factor, 0.01)


class SpeedUsFromFactorTests(unittest.TestCase):
    def test_endpoints(self) -> None:
        self.assertEqual(speed_us_from_stick_factor(0.0), SPEED_MAX_US)
        self.assertEqual(speed_us_from_stick_factor(1.0), SPEED_MIN_US)

    def test_midpoint(self) -> None:
        mid = speed_us_from_stick_factor(0.5)
        self.assertEqual(mid, (SPEED_MIN_US + SPEED_MAX_US) // 2)


if __name__ == "__main__":
    unittest.main()
