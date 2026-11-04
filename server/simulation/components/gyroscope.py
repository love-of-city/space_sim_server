"""Sample the published service-bus state after physics, before output."""
from Basilisk.architecture import sysModel
import numpy as np

from space_arm_platform.sampling import DYNAMICS_HZ
from simulation.assembly import Slot
from simulation.gyroscope import GYRO_HZ, GYRO_PERIOD_NS, GyroConfig, Gyroscope


class GyroSampler(sysModel.SysModel):
    def __init__(self, reader, sensor, *, epoch_ns=0, restart=None, angular_dof=None):
        super().__init__()
        self.ModelTag = sensor.sensor_name + "50Hz"
        self.reader, self.sensor = reader, sensor
        self.epoch_ns, self.restart = int(epoch_ns), restart
        self.angular_dof = angular_dof

    def Reset(self, CurrentSimNanos):
        if int(CurrentSimNanos) != 0:
            raise ValueError("gyro reset requires a fresh simulation at time zero")
        if self.restart is None:
            self.sensor.reset()
        else:
            self.sensor.restore(self.restart)

    def UpdateState(self, CurrentSimNanos):
        # Never relabel a held state as a new measurement. The sensor's divider
        # is aligned to native body publication, not wall time or render reads.
        now = int(CurrentSimNanos)
        if not self.reader.isWritten() or int(self.reader.timeWritten()) != now:
            raise RuntimeError("service gyro requires a fresh authoritative body state")
        payload = self.reader()
        # The native bulk message is fresh at every physics tick, even when
        # body/site publication is thinned. MuJoCo free-joint angular velocity
        # is in the body frame, matching SCStates. This also permits a handoff
        # at a 400 Hz epoch that is not on the 50 Hz sampling grid.
        rate = (payload.omega_BN_B if self.angular_dof is None else
                np.asarray(payload.qvel, dtype=float).reshape(-1)[self.angular_dof:self.angular_dof+3])
        self.sensor.sample(now + self.epoch_ns, rate)


class GyroscopeComponent:
    name = "service_gyro"

    def __init__(self, config=None, *, seed=0, body_name="cubesat_bus", name="service_gyro",
                 epoch_ns=0, restart=None):
        self.name, self.body_name = name, body_name
        self.epoch_ns, self.restart = epoch_ns, restart
        self.sensor = Gyroscope(GyroConfig.from_mapping({} if config is None else config), seed=seed,
                                body_name=body_name, sensor_name=name)

    def install(self, ctx):
        if not self.sensor.config.enabled:
            return
        step_ns = round(1e9 / ctx.config.clock_rate_hz)
        if (ctx.config.clock_rate_hz != DYNAMICS_HZ or type(self.epoch_ns) is not int
                or self.epoch_ns < 0 or self.epoch_ns % step_ns):
            raise ValueError('gyro handoff epoch must lie on the 400 Hz physical grid')
        if self.epoch_ns and self.restart is None:
            raise ValueError('a continuing gyro requires its saved internal state')
        if self.restart is not None:
            last = self.restart.get('last')
            if last is None or not 0 <= self.epoch_ns-int(last['sample_time_ns']) < GYRO_PERIOD_NS:
                raise ValueError('gyro restart must contain the most recent sampling epoch')
        dof = None
        if getattr(ctx.ports, 'stepper', None) is not None:
            _, dof, _ = ctx.ports.free_body_layout(self.body_name)
            dof += 3
            reader = ctx.ports.scene_state().addSubscriber()
        else:
            if self.epoch_ns % GYRO_PERIOD_NS:
                raise ValueError('off-grid gyro restart requires fresh native bulk states')
            reader = ctx.ports.body_state(self.body_name).addSubscriber()
        model = GyroSampler(reader, self.sensor, epoch_ns=self.epoch_ns,
                            restart=self.restart, angular_dof=dof)
        phase = ((-self.epoch_ns) % GYRO_PERIOD_NS) // step_ns
        ctx.add(model, Slot.SENSOR, every=DYNAMICS_HZ // GYRO_HZ, phase=phase)
        ctx.extra[self.name] = self.sensor

    def observation(self):
        return self.sensor.latest
