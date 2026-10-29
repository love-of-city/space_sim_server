"""Actual Basilisk messages/scheduler; run in the cloud simulation environment."""
from types import SimpleNamespace

import pytest
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import SimulationBaseClass

from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.components.clock import ClockComponent
from simulation.components.gyroscope import GyroscopeComponent, GyroSampler
from simulation.gyroscope import Gyroscope


def test_after_physics_sampler_preserves_source_timestamp_and_reset():
    class Source(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'gyroTestTruth'
            self.message = messaging.SCStatesMsg()
            self.writes = []

        def UpdateState(self, stamp):
            if int(stamp) % 20_000_000 == 0:
                rate = [int(stamp) * 1e-9, -.02, .03]
                self.message.write(messaging.SCStatesMsgPayload(omega_BN_B=rate), stamp, self.moduleID)
                self.writes.append((int(stamp), rate))

    source = Source()
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess('graspProcess')
    process.addTask(sim.CreateNewTask('graspTask', 2_500_000))
    ports = SimpleNamespace(body_state=lambda name: source.message)
    ctx = AssemblyContext(sim, None, config=SessionConfig(publish_stride=8), physics_ports=ports)
    ctx.install([ClockComponent()])
    ctx.add(source, Slot.PHYSICS)
    gyro = GyroscopeComponent(dict(white_noise_std_rad_s=0., initial_bias_std_rad_s=0.,
                                   bias_random_walk_rad_s_sqrt_s=0.), seed=2)
    ctx.install([gyro])
    sim.InitializeSimulation()
    sim.ConfigureStopTime(100_000_000)
    sim.ExecuteSimulation()
    assert [stamp for stamp, _ in source.writes] == list(range(0, 100_000_001, 20_000_000))
    assert gyro.observation()['sample_time_ns'] == '100000000'
    assert gyro.observation()['angular_velocity_body_rad_s'] == [.1, -.02, .03]
    assert ctx.extra['service_gyro'] is gyro.sensor
    entries = [r for r in ctx.describe() if r['component'] == 'service_gyro']
    assert len(entries) == 1 and entries[0]['rate_hz'] == 50
    # Reading the sensor never writes the source SCStatesMsg.
    assert source.message.read().omega_BN_B == [.1, -.02, .03]


def test_stale_or_unwritten_truth_is_never_relabelled():
    message = messaging.SCStatesMsg()
    sampler = GyroSampler(message.addSubscriber(), Gyroscope())
    with pytest.raises(RuntimeError, match='fresh authoritative'):
        sampler.UpdateState(0)
    message.write(messaging.SCStatesMsgPayload(omega_BN_B=[0., 0., 0.]), 0)
    sampler.UpdateState(0)
    with pytest.raises(RuntimeError, match='fresh authoritative'):
        sampler.UpdateState(20_000_000)
    with pytest.raises(ValueError, match='time zero'):
        sampler.Reset(20_000_000)
    sampler.Reset(0)
    assert sampler.sensor.latest is None


@pytest.mark.parametrize('epoch_ns', [2_500_000, 7_500_000, 20_000_000, 37_500_000])
def test_restart_off_gyro_grid_keeps_absolute_epochs_and_rng(epoch_ns):
    from simulation.gyroscope import GyroConfig

    config = GyroConfig(white_noise_std_rad_s=.001, initial_bias_std_rad_s=.0001,
                        bias_random_walk_rad_s_sqrt_s=.00001)
    original = Gyroscope(config, seed=2)
    for stamp in range(0, epoch_ns+1, 20_000_000):
        original.sample(stamp, [.01, -.02, .03])

    class Bulk(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'gyroBulkTruth'
            self.message = messaging.MJSceneStateMsg()

        def UpdateState(self, nanos):
            self.message.write(messaging.MJSceneStateMsgPayload(
                qpos=[0.]*7, qvel=[0., 0., 0., .01, -.02, .03]), nanos, self.moduleID)

    source = Bulk()
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess('graspProcess')
    process.addTask(sim.CreateNewTask('graspTask', 2_500_000))
    ports = SimpleNamespace(stepper=object(), scene_state=lambda: source.message,
                            free_body_layout=lambda name: (0, 0, [0, 0, 0]))
    ctx = AssemblyContext(sim, None, config=SessionConfig(publish_stride=4), physics_ports=ports)
    ctx.install([ClockComponent()])
    ctx.add(source, Slot.PHYSICS)
    gyro = GyroscopeComponent(vars(config), seed=2, epoch_ns=epoch_ns, restart=original.snapshot())
    ctx.install([gyro])
    sim.InitializeSimulation()
    end_ns = 100_000_000
    sim.ConfigureStopTime(end_ns-epoch_ns)
    sim.ExecuteSimulation()
    for stamp in range((epoch_ns//20_000_000+1)*20_000_000, end_ns+1, 20_000_000):
        original.sample(stamp, [.01, -.02, .03])
    assert gyro.sensor.snapshot() == original.snapshot()
    assert next(r for r in ctx.describe() if r['component'] == 'service_gyro')['rate_hz'] == 50
