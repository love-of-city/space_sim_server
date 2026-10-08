"""Install native BSK gravity models on the local multibody integration boundary."""
from simulation.assembly import Slot
from simulation.orbital_environment import BskOrbitalEnvironment, orbital_mode


class OrbitalGravityComponent:
    name = "orbital_gravity"

    def __init__(self, mode=None, *, ephemeris_sampler=None):
        self.mode = orbital_mode(mode)
        self.ephemeris_sampler = ephemeris_sampler
        self.environment = None

    def install(self, ctx):
        if ctx.config.dynamics_backend != "local" or self.mode == "linear_tidal":
            return
        reference, factory = ctx.ports.orbital_providers()
        environment = BskOrbitalEnvironment(reference, ephemeris_sampler=self.ephemeris_sampler)
        for name, body in factory.gravBodies.items():
            # The factory subscribes these readers to the common SPICE interface.
            environment.add_gravity_source(name, body, ctx.ports.planet_state(name))
        ctx.ports.set_orbital_environment(environment)
        self.environment = environment
        # NBodyGravity runs at each local physics substep through its adapter,
        # NOT on the publish-only MJScene task or a separate competing integrator.
        ctx.note_external(environment.gravity, component=self.name,
                          task="localMujocoSubsteps", priority=int(Slot.ENVIRONMENT), every=0)
        ctx.keep_alive(environment)

class BskGravitySourceComponent:
    """Add a native BSK GravBodyData + ephemeris to BOTH orbital paths.

    Install after OrbitalGravityComponent and before InitializeSimulation. The
    source message uses the existing J2000 ephemeris origin (Earth in this app).
    An additional SPICE/ephemeris module, if required, is scheduled by the caller
    in Slot.ORBIT. No local force formula or stepper change is needed.
    """

    def __init__(self, source_name, body, state_message):
        self.source_name = source_name
        self.name = f"gravity_source_{source_name}"
        self.body = body
        self.state_message = state_message

    def install(self, ctx):
        ctx.ports.orbital_environment().add_gravity_source(
            self.source_name, self.body, self.state_message)
        ctx.keep_alive(self.body, self.state_message)
