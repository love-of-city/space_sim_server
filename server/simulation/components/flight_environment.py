"""Shared local-backend drag, eclipsed SRP and unmodelled orbital acceleration.

Forces enter PhysicsPorts; this module never writes propagated state. The seeded
Gauss-Markov acceleration is a truth disturbance, not filter process covariance.
Areas and coefficients are scenario assumptions, not qualified hardware data.
"""
import math
import numpy as np
from Basilisk.architecture import messaging, sysModel
from simulation.assembly import Slot
from simulation.components.drag import atmosphere_velocity_n, planet_atmosphere_velocity_n, drag_force_n

EARTH_RADIUS_M = 6378136.6
AU_M = 149597870700.0
SOLAR_PRESSURE_N_M2 = 1361.0 / 299792458.0


def solar_force_n(position, sun_position, area, coefficient, illumination):
    away = np.asarray(position, dtype=float) - np.asarray(sun_position, dtype=float)
    radius = np.linalg.norm(away)
    if radius < 1 or not np.isfinite(radius):
        raise ValueError("invalid Sun-spacecraft distance")
    return SOLAR_PRESSURE_N_M2 * area * coefficient * illumination * (AU_M / radius)**2 * away / radius


class FlightEnvironment(sysModel.SysModel):
    def __init__(self, state, sun, density, shadow, *, mass, area, options, seed, planet=None):
        super().__init__()
        self.ModelTag = "flightEnvironment"
        self.state = state.addSubscriber()
        self.sun = sun.addSubscriber()
        self.density = density.addSubscriber()
        self.shadow = shadow.addSubscriber()
        self.planet = planet.addSubscriber() if planet is not None else None
        self.mass, self.area, self.options = mass, area, options
        self.rng = np.random.default_rng(seed)
        self.acceleration = np.zeros(3)
        self.forceOutMsg = messaging.CmdForceInertialMsg()
        self.last_time = None
        self.last = {"drag_n": [0., 0., 0.], "srp_n": [0., 0., 0.], "disturbance_n": [0., 0., 0.], "density_kg_m3": 0., "illumination": 1.}

    def UpdateState(self, nanos):
        state, sun, opt = self.state(), self.sun(), self.options
        density = float(self.density().neutralDensity)
        shadow = self.shadow()
        light = float(shadow.illuminationFactor if hasattr(shadow, "illuminationFactor") else shadow.shadowFactor)
        if not math.isfinite(density) or density < 0 or not 0 <= light <= 1:
            raise ValueError("invalid atmospheric density or eclipse factor")
        if self.last_time is None:
            self.acceleration = self.rng.normal(0, opt.orbit_acceleration_sigma_m_s2, 3)
        else:
            dt = (nanos - self.last_time) * 1e-9
            phi = math.exp(-dt / opt.orbit_correlation_s)
            if dt > 0:
                self.acceleration = phi * self.acceleration + math.sqrt(max(0., 1-phi*phi)) * self.rng.normal(0, opt.orbit_acceleration_sigma_m_s2, 3)
        self.last_time = nanos
        atmosphere_velocity = (planet_atmosphere_velocity_n(state.r_BN_N, self.planet())
                               if self.planet is not None else atmosphere_velocity_n(state.r_BN_N))
        relative = np.asarray(state.v_BN_N) - atmosphere_velocity
        drag = drag_force_n(density, relative, self.area, opt.drag_coefficient) if opt.drag_enabled else np.zeros(3)
        srp = solar_force_n(state.r_BN_N, sun.PositionVector, self.area, opt.reflection_coefficient, light) if opt.srp_enabled else np.zeros(3)
        disturbance = self.mass * self.acceleration
        self.last = {"drag_n": drag.tolist(), "srp_n": srp.tolist(), "disturbance_n": disturbance.tolist(), "density_kg_m3": density, "illumination": light}
        total = drag + srp + disturbance
        if not np.isfinite(total).all():
            raise ValueError("non-finite environment force")
        self.forceOutMsg.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=total.tolist()), nanos, self.moduleID)

    def snapshot(self, *, epoch_ns=0):
        return {"acceleration_m_s2": self.acceleration.tolist(), "rng_state": self.rng.bit_generator.state,
                "last_time_ns": None if self.last_time is None else epoch_ns+self.last_time}

    def restore(self, state, *, epoch_ns=0):
        value = np.asarray(state["acceleration_m_s2"], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("invalid handoff disturbance state")
        self.acceleration = value.copy()
        self.rng.bit_generator.state = state["rng_state"]
        self.last_time = (state['last_time_ns']-epoch_ns
                          if epoch_ns and state.get('last_time_ns') is not None else 0)


class FlightEnvironmentComponent:
    """Reusable on either far-field flights or the platform's local SARM session."""
    name = "flight_environment"

    def __init__(self, options, bodies):
        # (body name, optional assembly COM site, effective mass, effective area)
        self.options, self.bodies = options, bodies
        self.models = []

    def install(self, ctx):
        from Basilisk.simulation import exponentialAtmosphere, eclipse
        if ctx.ports.stepper is None:
            raise RuntimeError("flight environment requires the platform local dynamics core")
        seeds = np.random.SeedSequence([self.options.seed, 2]).spawn(len(self.bodies))
        for (body, site, mass, area), seed in zip(self.bodies, seeds, strict=True):
            state = ctx.ports.site_state(site) if site else ctx.ports.body_com_state(body)
            atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
            atmosphere.ModelTag = f"flightAtmosphere/{body}"
            atmosphere.planetRadius = EARTH_RADIUS_M
            atmosphere.scaleHeight = self.options.scale_height_m
            atmosphere.baseDensity = self.options.density_300km_kg_m3 * math.exp(300000 / self.options.scale_height_m)
            atmosphere.planetPosInMsg.subscribeTo(ctx.ports.planet_state("earth"))
            atmosphere.addSpacecraftToModel(state)
            shadow = eclipse.Eclipse()
            shadow.ModelTag = f"flightEclipse/{body}"
            shadow.sunInMsg.subscribeTo(ctx.ports.planet_state("sun"))
            shadow.addPlanetToModel(ctx.ports.planet_state("earth"))
            shadow.addSpacecraftToModel(state)
            force = FlightEnvironment(state, ctx.ports.planet_state("sun"), atmosphere.envOutMsgs[0], shadow.eclipseOutMsgs[0],
                                      mass=mass, area=area, options=self.options, seed=seed, planet=ctx.ports.planet_state("earth"))
            force.ModelTag += "/" + body
            ctx.ports.add_body_wrench(body, source=f"{self.name}/{body}", force_inertial=force.forceOutMsg)
            for module in (atmosphere, shadow, force):
                ctx.add(module, Slot.ENVIRONMENT)
            ctx.keep_alive(atmosphere, shadow, force)
            self.models.append(force)

    def telemetry(self):
        return [model.last for model in self.models]
