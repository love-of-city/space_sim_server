"""Sample native wrist readings after physics, independent of RGB publication."""
from collections import deque
from Basilisk.architecture import sysModel
from space_arm_platform.sampling import DYNAMICS_HZ, WRENCH_HZ, sample_tick
from simulation.assembly import Slot
from simulation.wrench_sensors import WRIST_SENSOR


class WrenchSampler(sysModel.SysModel):
    def __init__(self, stepper, clock, transform=None):
        super().__init__()
        self.ModelTag = "wristWrench200Hz"
        self.stepper, self.clock = stepper, clock
        self.pending = deque()
        self.transform = transform

    def Reset(self, CurrentSimNanos):
        self.pending.clear()

    def UpdateState(self, CurrentSimNanos):
        if self.clock.step_index % (DYNAMICS_HZ // WRENCH_HZ):
            return
        reading = WRIST_SENSOR.observation(self.stepper, int(CurrentSimNanos))
        if reading is None or not reading["valid"]:
            return
        if len(self.pending) >= 1024:
            raise RuntimeError("wrist sample queue overflow; refusing silent data loss")
        self.pending.append(self.transform(reading) if self.transform else reading)

    def drain(self, stamp, capture_hz=50):
        if sample_tick(int(stamp), int(capture_hz)) is None:
            return []
        rows = []
        while self.pending and int(self.pending[0]["interval_end_time_ns"]) <= stamp:
            rows.append(self.pending.popleft())
        return rows


def install_wrench_sampler(ctx, transform=None):
    model = WrenchSampler(ctx.simulation.local_stepper, ctx.clock, transform)
    # Reads native sensordata, not decimated BSK body/site messages. Self-divides
    # after physics so body publication stride never limits sensor sampling.
    ctx.add(model, Slot.SENSOR, every=1, component="wrist_wrench_sampling")
    return model
