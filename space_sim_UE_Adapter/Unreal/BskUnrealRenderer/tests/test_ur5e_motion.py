"""Deterministic motion contract for the UR5e visualization demo."""

from __future__ import annotations

import importlib.util
import math
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "examples" / "scenario_ur5e_unreal.py"
SPEC = importlib.util.spec_from_file_location("scenario_ur5e_unreal_test", SCRIPT_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {SCRIPT_PATH}")
SCENARIO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO)


class Ur5eSequentialMotionTests(unittest.TestCase):
    def test_each_joint_moves_once_by_the_requested_angle(self) -> None:
        angle = math.radians(20.0)
        move_seconds = 2.0
        hold_seconds = 0.75

        self.assertEqual(
            SCENARIO.sequential_joint_targets(0.5, angle, move_seconds, hold_seconds),
            SCENARIO.HOME,
        )

        first_midpoint = SCENARIO.sequential_joint_targets(2.0, angle, move_seconds, hold_seconds)
        self.assertAlmostEqual(first_midpoint[0], SCENARIO.HOME[0] + 0.5 * angle)
        self.assertEqual(first_midpoint[1:], SCENARIO.HOME[1:])

        second_start = SCENARIO.INITIAL_HOLD_SECONDS + move_seconds + hold_seconds
        after_first = SCENARIO.sequential_joint_targets(
            second_start,
            angle,
            move_seconds,
            hold_seconds,
        )
        self.assertAlmostEqual(after_first[0], SCENARIO.HOME[0] + angle)
        self.assertAlmostEqual(after_first[1], SCENARIO.HOME[1])

        finished_time = SCENARIO.INITIAL_HOLD_SECONDS + len(SCENARIO.HOME) * (
            move_seconds + hold_seconds
        )
        finished = SCENARIO.sequential_joint_targets(
            finished_time,
            angle,
            move_seconds,
            hold_seconds,
        )
        for actual, home in zip(finished, SCENARIO.HOME):
            self.assertAlmostEqual(actual, home + angle)

    def test_rejects_unsafe_or_invalid_parameters(self) -> None:
        with self.assertRaises(ValueError):
            SCENARIO.sequential_joint_targets(0.0, math.radians(46.0), 2.0, 0.5)
        with self.assertRaises(ValueError):
            SCENARIO.sequential_joint_targets(0.0, 0.1, 0.0, 0.5)
        with self.assertRaises(ValueError):
            SCENARIO.sequential_joint_targets(0.0, 0.1, 2.0, -0.1)


if __name__ == "__main__":
    unittest.main()
