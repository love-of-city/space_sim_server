"""Real Basilisk task scheduler, not a mocked time loop. No MuJoCo import."""
import pytest
pytest.importorskip("Basilisk")
from Basilisk.architecture import sysModel
from Basilisk.utilities import SimulationBaseClass
from simulation.physics_clock import RationalPhysicsClock
from simulation.teleop_grasp_unreal import CartesianIkControlModel
from space_arm_platform.sampling import tick_time_ns


@pytest.mark.parametrize("seconds", [1, 60])
def test_real_scheduler_and_ik_are_integer_divided_without_drift(seconds):
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    task = simulation.CreateNewTask("physics", tick_time_ns(1, 240))
    process.addTask(task)
    clock = RationalPhysicsClock(task)
    physics_stamps, ik_stamps = [], []

    class Target:
        def reset(self, time):
            ik_stamps.clear()
        def update(self, time):
            ik_stamps.append(round(time * 1_000_000_000))

    class Probe(sysModel.SysModel):
        def UpdateState(self, now):
            physics_stamps.append(int(now))

    ik = CartesianIkControlModel(Target(), clock=clock, stride=2)
    probe = Probe()
    simulation.AddModelToTask("physics", clock, 20000)
    simulation.AddModelToTask("physics", ik, 10000)
    simulation.AddModelToTask("physics", probe, 0)
    simulation.InitializeSimulation()
    # Segmented outer calls must not reset the rational clock's phase.
    for end in (1_000_000_000 // 2, seconds * 1_000_000_000):
        simulation.ConfigureStopTime(end)
        simulation.ExecuteSimulation()
    assert physics_stamps == [tick_time_ns(n, 240) for n in range(seconds * 240 + 1)]
    assert ik_stamps == physics_stamps[::2]
    assert len(ik_stamps) == seconds * 120 + 1
    assert physics_stamps[-1] == seconds * 1_000_000_000
    # Same simulation scheduler can also be reinitialized without retaining phase.
    simulation.InitializeSimulation()
    physics_stamps.clear()
    simulation.ConfigureStopTime(100_000_000)
    simulation.ExecuteSimulation()
    assert physics_stamps == [tick_time_ns(n, 240) for n in range(25)]
    assert ik_stamps == physics_stamps[::2]
