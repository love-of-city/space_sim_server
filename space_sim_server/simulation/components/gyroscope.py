"""Sample the published service-bus state after physics, before output."""
from Basilisk.architecture import sysModel

from space_arm_platform.sampling import DYNAMICS_HZ
from simulation.assembly import Slot
from simulation.gyroscope import GYRO_HZ, GyroConfig, Gyroscope


class GyroSampler(sysModel.SysModel):
    def __init__(self, reader, sensor):
        super().__init__()
        self.ModelTag = "serviceGyroscope50Hz"
        self.reader, self.sensor = reader, sensor

    def Reset(self, CurrentSimNanos):
        if int(CurrentSimNanos) != 0:
            raise ValueError("gyro reset requires a fresh simulation at time zero")
        self.sensor.reset()

    def UpdateState(self, CurrentSimNanos):
        # Never relabel a held state as a new measurement. The sensor's divider
        # is aligned to native body publication, not wall time or render reads.
        now = int(CurrentSimNanos)
        if not self.reader.isWritten() or int(self.reader.timeWritten()) != now:
            raise RuntimeError("service gyro requires a fresh authoritative body state")
        self.sensor.sample(now, self.reader().omega_BN_B)


class GyroscopeComponent:
    name = "service_gyro"

    def __init__(self, config=None, *, seed=0):
        self.sensor = Gyroscope(GyroConfig.from_mapping({} if config is None else config), seed=seed)

    def install(self, ctx):
        if not self.sensor.config.enabled:
            return
        model = GyroSampler(ctx.ports.body_state("cubesat_bus").addSubscriber(), self.sensor)
        ctx.add(model, Slot.SENSOR, every=DYNAMICS_HZ // GYRO_HZ)
        ctx.extra["service_gyro"] = self.sensor

    def observation(self):
        return self.sensor.latest
