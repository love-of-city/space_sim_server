import json
from types import SimpleNamespace
import pytest
from space_arm_platform.models import WrenchObservation, SimulationObservation
from space_arm_platform.wrench_recording import WrenchStreamWriter
from test_wrist_wrench_recording import wrench
from test_lerobot_recording import observation


def sample(end):
    data = wrench()
    data.update(sample_time_ns=str(end-2_500_000), interval_end_time_ns=str(end))
    return WrenchObservation(**data)


def packet(stamp, ends):
    return SimpleNamespace(sim_time_ns=str(stamp), wrist_wrench_rate_hz=200,
                           wrist_wrench_samples=[sample(end) for end in ends])


def test_sensor_stream_trims_preroll_and_keeps_four_samples_between_frames(tmp_path):
    writer = WrenchStreamWriter(tmp_path)
    writer.append(packet(20_000_000, [5_000_000, 10_000_000, 15_000_000, 20_000_000]))
    writer.append(packet(40_000_000, [25_000_000, 30_000_000, 35_000_000, 40_000_000]))
    rows = [json.loads(line) for line in (tmp_path/writer.relative_path).read_text().splitlines()]
    assert len(rows)==5
    assert [row['timestamp'] for row in rows] == pytest.approx([0, .005, .01, .015, .02])
    assert writer.metadata()['samples']==5
    assert all(int(row['sim_time_ns'])-int(row['sample_time_ns'])==2_500_000 for row in rows)


@pytest.mark.parametrize('ends', [[30_000_000,35_000_000,40_000_000], [20_000_000,25_000_000,30_000_000,35_000_000,40_000_000], [25_000_000,30_000_000,35_000_000]])
def test_stream_rejects_gap_duplicate_or_missing_tail(tmp_path, ends):
    writer=WrenchStreamWriter(tmp_path)
    writer.append(packet(20_000_000,[20_000_000]))
    with pytest.raises(ValueError): writer.append(packet(40_000_000,ends))
    assert writer.count==1


def test_protocol_rejects_future_and_discontinuous_force_samples():
    data=observation(1).model_dump()
    data['wrist_wrench_rate_hz']=200
    data['wrist_wrench_samples']=[sample(95_000_000).model_dump(),sample(105_000_000).model_dump()]
    with pytest.raises(ValueError): SimulationObservation.model_validate(data)


def test_real_scheduler_samples_native_each_second_physics_step():
    from Basilisk.architecture import sysModel
    from Basilisk.utilities import SimulationBaseClass
    from simulation.physics_clock import RationalPhysicsClock
    from simulation.wrench_sampling import WrenchSampler
    import numpy as np
    sim=SimulationBaseClass.SimBaseClass()
    proc=sim.CreateNewProcess('p'); task=sim.CreateNewTask('physics',2_500_000); proc.addTask(task)
    clock=RationalPhysicsClock(task)
    class Native:
        now=0
        def read_wrench(self,*args):
            return np.full(6,self.now,dtype=float), self.now-2_500_000 if self.now else None, self.now
    native=Native()
    class Physics(sysModel.SysModel):
        def UpdateState(self, now): native.now=int(now)
    physics=Physics(); sampler=WrenchSampler(native,clock)
    sim.AddModelToTask('physics',clock,20000); sim.AddModelToTask('physics',physics,1000); sim.AddModelToTask('physics',sampler,999)
    sim.InitializeSimulation(); sim.ConfigureStopTime(20_000_000); sim.ExecuteSimulation()
    rows=sampler.drain(20_000_000)
    assert [int(row['interval_end_time_ns']) for row in rows]==[5_000_000,10_000_000,15_000_000,20_000_000]
    assert [row['force_n'][0] for row in rows]==[5_000_000,10_000_000,15_000_000,20_000_000]
    assert sampler.drain(20_000_000)==[]
