"""Contract tests for Demo 8 live wall-clock pacing."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path
from unittest import mock


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "examples" / "scenario_mjscene_unreal.py"
SPEC = importlib.util.spec_from_file_location("scenario_mjscene_unreal_test", SCRIPT_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {SCRIPT_PATH}")
SCENARIO = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCENARIO)


class _FakeTotalSimulation:
    def __init__(self) -> None:
        self.NextTaskTime = 0


class _FakeSimulation:
    def __init__(self) -> None:
        self.TotalSim = _FakeTotalSimulation()
        self.StopTime = 8_000_000_000
        self.showProgressBar = True
        self.executed_stops: list[int] = []

    def ConfigureStopTime(self, stop_time: int) -> None:
        self.StopTime = int(stop_time)

    def ExecuteSimulation(self) -> None:
        self.executed_stops.append(self.StopTime)
        self.TotalSim.NextTaskTime += 4_000_000_000


class Demo8LivePacingTests(unittest.TestCase):
    def test_paces_each_existing_task_tick_without_adding_a_model(self) -> None:
        simulation = _FakeSimulation()
        with mock.patch.object(SCENARIO.time, "monotonic", return_value=100.0), mock.patch.object(
            SCENARIO.time, "sleep"
        ) as sleep:
            SCENARIO.install_wall_clock_pacing(simulation, 120.0)
            simulation.ExecuteSimulation()

        self.assertEqual(simulation.executed_stops, [0, 4_000_000_000, 8_000_000_000])
        self.assertEqual(simulation.StopTime, 8_000_000_000)
        self.assertTrue(simulation.showProgressBar)
        self.assertNotIn("ExecuteSimulation", simulation.__dict__)
        self.assertEqual(sleep.call_count, 3)
        self.assertAlmostEqual(sleep.call_args_list[0].args[0], 0.0)
        self.assertAlmostEqual(sleep.call_args_list[1].args[0], 4.0 / 120.0)
        self.assertAlmostEqual(sleep.call_args_list[2].args[0], 8.0 / 120.0)

    def test_rejects_non_positive_rate(self) -> None:
        with self.assertRaises(ValueError):
            SCENARIO.install_wall_clock_pacing(_FakeSimulation(), 0.0)


if __name__ == "__main__":
    unittest.main()
