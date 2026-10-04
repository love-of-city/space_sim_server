"""Native BSK orbital adapter: field equivalence, extension and timing contracts."""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("Basilisk")
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody

from simulation.orbital_environment import (
    BskOrbitalEnvironment, LinearEphemerisSampler, interpolate_origin, orbital_mode,
)


R = np.array([6_878_137., 100_000., -80_000.])


def source_message(position=(0., 0., 0.), velocity=(0., 0., 0.), nanos=0):
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.PositionVector = list(position)
    payload.VelocityVector = list(velocity)
    payload.J20002Pfix = np.eye(3).tolist()
    return messaging.SpicePlanetStateMsg().write(payload, nanos)


def environment(sun=False):
    reference = spacecraft.Spacecraft()
    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth()
    earth.isCentralBody = True
    env = BskOrbitalEnvironment(reference)
    earth_message = source_message()
    env.add_gravity_source("earth", earth, earth_message)
    keep = [factory, earth_message]
    if sun:
        body = factory.createSun()
        msg = source_message((1.3e11, -6e10, 2e10))
        env.add_gravity_source("sun", body, msg)
        keep.append(msg)
    env.begin_interval()
    return env, keep


def direct_acceleration(position, sources):
    result = np.zeros(3)
    for source in sources.values():
        delta = position - np.asarray(source["reader"]().PositionVector)
        result -= source["body"].mu * delta / np.linalg.norm(delta)**3
    return result


@pytest.mark.parametrize("sun", [False, True])
@pytest.mark.parametrize("distance", [1., 100., 100_000.])
def test_per_body_full_differential_gravity_matches_point_mass_reference(sun, distance):
    env, keep = environment(sun)
    positions = np.array([[0., 0., 0.], [distance, 0., 0.], [0., -distance, distance/2]])
    masses = np.array([0., 2., 5.])
    force = env.forces(positions, masses, R, 0)
    common = direct_acceleration(R, env.sources)
    for i in (1, 2):
        expected = masses[i]*(direct_acceleration(R + positions[i], env.sources) - common)
        np.testing.assert_allclose(force[i], expected, atol=2e-14, rtol=2e-8)
    np.testing.assert_array_equal(force[0], 0)
    assert env.telemetry()["bodies"] == 2


def test_common_acceleration_is_removed_and_world_massless_bodies_are_not_forced():
    env, keep = environment(True)
    forces = env.forces(np.zeros((4, 3)), np.array([0., 3., 0., 9.]), R, 0)
    np.testing.assert_array_equal(forces, 0)


def test_add_native_bsk_moon_component_without_changing_physics_core():
    env, keep = environment(True)
    factory = keep[0]
    moon = factory.createMoon()
    moon_message = source_message((3.7e8, 2e7, -3e7))
    env.add_gravity_source("moon", moon, moon_message)
    env.begin_interval()
    positions = np.array([[0., 0., 0.], [30000., 2000., 0.]])
    force = env.forces(positions, np.array([0., 4.]), R, 0)
    expected = 4*(direct_acceleration(R + positions[1], env.sources) - direct_acceleration(R, env.sources))
    np.testing.assert_allclose(force[1], expected, atol=2e-14, rtol=1e-9)
    assert len(env.sources) == 3
    # Shared BSK object: reference propagation and local field use the same model.
    assert int(env.gravity.getGravitySource("moon").model.this) != 0
    assert [s["name"] for s in env.telemetry()["sources"]] == ["earth", "sun", "moon"]


def test_ephemeris_translation_does_not_change_relative_forces():
    first, keep1 = environment(True)
    second, keep2 = environment(True)
    translation = np.array([1e10, -2e10, 3e10])
    translated_messages = []
    for name, source in second.sources.items():
        original = first.sources[name]["reader"]()
        msg = source_message(np.asarray(original.PositionVector) + translation)
        source["reader"] = msg.addSubscriber()
        translated_messages.append(msg)
    second.begin_interval()
    x = np.array([[0., 0., 0.], [100., -50., 30.]])
    masses = np.array([0., 2.])
    np.testing.assert_allclose(first.forces(x, masses, R, 0), second.forces(x, masses, R, 0), atol=2e-13)


def test_ephemeris_is_sampled_at_requested_substep_time():
    env, keep = environment(True)
    source = env.sources["sun"]
    msg = source_message((1e9, 2e9, 3e9), (1000., -3000., 500.), 1_000_000_000)
    source["reader"] = msg.addSubscriber()
    env.begin_interval()
    env.forces(np.zeros((2, 3)), np.array([0., 1.]), R, 500_000_000)
    actual = source["message"].read()
    np.testing.assert_allclose(actual.PositionVector, [1e9-500, 2e9+1500, 3e9-250])
    assert source["message"].addSubscriber().timeWritten() == 500_000_000


def test_hermite_origin_has_correct_endpoints_and_constant_acceleration():
    r0 = np.array([1., 2., 3.]); v0 = np.array([2., 3., 4.]); a = np.array([.2, -.4, .1]); dt = .01
    r1 = r0 + v0*dt + .5*a*dt*dt; v1 = v0 + a*dt
    for u in (0., .25, .5, .75, 1.):
        np.testing.assert_allclose(interpolate_origin(r0, v0, r1, v1, dt, u),
                                   r0 + v0*dt*u + .5*a*(dt*u)**2, atol=1e-14)


def test_source_registration_and_runtime_mutation_fail_closed():
    env, keep = environment()
    earth = env.sources["earth"]["body"]
    with pytest.raises(ValueError, match="repeated"):
        env.add_gravity_source("earth", earth, keep[1])
    with pytest.raises(ValueError, match="same GravBodyData"):
        env.add_gravity_source("copy", earth, keep[1])
    other = simIncludeGravBody.gravBodyFactory().createEarth(); other.isCentralBody = True
    with pytest.raises(ValueError, match="one central"):
        env.add_gravity_source("other", other, keep[1])
    x = np.zeros((2, 3)); masses = np.array([0., 1.])
    env.forces(x, masses, R, 10)
    with pytest.raises(RuntimeError, match="fixed for a session"):
        env.add_gravity_source("late", other, keep[1])
    with pytest.raises(ValueError, match="backwards"):
        env.forces(x, masses, R, 9)
    with pytest.raises(ValueError, match="mass changes"):
        env.forces(x, np.array([0., 2.]), R, 11)
    with pytest.raises(ValueError, match="invalid COM"):
        env.forces(x*np.nan, masses, R, 11)


def test_mode_selection(monkeypatch):
    monkeypatch.delenv("SPACE_SIM_ORBITAL_MODE", raising=False)
    assert orbital_mode() == "bsk"
    monkeypatch.setenv("SPACE_SIM_ORBITAL_MODE", "linear_tidal")
    assert orbital_mode() == "linear_tidal"
    assert orbital_mode("bsk") == "bsk"
    with pytest.raises(ValueError):
        orbital_mode("typo")

def test_reset_restarts_gravity_clock_without_duplicate_sources():
    env, keep = environment()
    x = np.zeros((2, 3)); masses = np.array([0., 1.])
    env.forces(x, masses, R, 100)
    env.reset()
    env.begin_interval()
    env.forces(x, masses, R, 0)
    assert env.evaluations == 1
    assert len(env.sources) == 1
    assert env.telemetry()["bodies"] == 1


def test_gravity_source_component_registers_through_physics_ports():
    from types import SimpleNamespace
    from simulation.components.orbital_gravity import BskGravitySourceComponent
    env, keep = environment()
    moon = keep[0].createMoon()
    message = source_message((3.8e8, 0., 0.))
    retained = []
    ctx = SimpleNamespace(ports=SimpleNamespace(orbital_environment=lambda: env),
                          keep_alive=lambda *objects: retained.extend(objects))
    component = BskGravitySourceComponent("moon", moon, message)
    component.install(ctx)
    assert "moon" in env.sources
    assert retained == [moon, message]


def propagate_pair(tmp_path, *, substeps=1, publish_every=2, seconds=2., rebase=50., sun=False, j2=False):
    """Compare two MuJoCo free bodies with two native BSK point spacecraft.

    Native BSK supplies BOTH the orbit carrier and field models. No analytical
    orbit or separate production gravity implementation is used by the adapter.
    """
    import math
    from Basilisk.simulation import mujoco
    from Basilisk.utilities import SimulationBaseClass, macros
    from simulation.local_mujoco_stepper import LocalMujocoStepper, SpacecraftOrbitOrigin

    path = tmp_path / "orbital_pair.xml"
    path.write_text('''<mujoco><option gravity="0 0 0"/><worldbody>
      <body name="a" pos="0 0 0"><freejoint name="a_free"/>
        <geom type="sphere" size="0.1" mass="2" contype="0" conaffinity="0"/>
      </body><body name="b" pos="1000 0 0"><freejoint name="b_free"/>
        <geom type="sphere" size="0.1" mass="3" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>''', encoding="utf-8")
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("p")
    process.addTask(simulation.CreateNewTask("t", macros.sec2nano(1/240)))
    process.addTask(simulation.CreateNewTask("scene_init", 10**18))
    scene = mujoco.MJScene.fromFile(str(path))
    simulation.AddModelToTask("scene_init", scene)
    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth(); earth.isCentralBody = True
    if j2:
        from Basilisk.simulation import sphericalHarmonicsGravityModel
        field = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
        field.maxDeg = 2
        field.cBar = [[1.], [0., 0.], [-1.08263e-3 / np.sqrt(5.), 0., 0.]]
        field.sBar = [[0.], [0., 0.], [0., 0., 0.]]
        field.radEquator, field.muBody = earth.radEquator, earth.mu
        earth.gravityModel = field
    origin = spacecraft.Spacecraft(); origin.ModelTag = "referenceOrbit"; origin.hub.mHub = 1.
    message = source_message()
    # This fixture has a deliberately static central body, not a live SPICE
    # publisher. Explicitly allow its constant message for the whole experiment.
    env = BskOrbitalEnvironment(origin, ephemeris_sampler=LinearEphemerisSampler(seconds + 1))
    env.add_gravity_source("earth", earth, message)
    if sun:
        sun_body = factory.createSun()
        sun_message = source_message((1.3e11, -6e10, 2e10))
        env.add_gravity_source("sun", sun_body, sun_message)
    stepper = LocalMujocoStepper(scene, path, [], origin=SpacecraftOrbitOrigin(origin, earth.mu),
                                 substeps=substeps, publish_every=publish_every, rebase_distance_m=rebase)
    stepper.set_orbital_environment(env)
    simulation.AddModelToTask("t", origin, 19000)
    simulation.AddModelToTask("t", stepper, 1000)
    r0 = np.array([6_878_137., 0., 0.]); velocity = np.array([0., math.sqrt(earth.mu/r0[0]), 0.])
    origin.hub.r_CN_NInit = r0; origin.hub.v_CN_NInit = velocity
    truth = []
    for i, dx in enumerate((0., 1000.)):
        craft = spacecraft.Spacecraft(); craft.ModelTag = f"truth{i}"; craft.hub.mHub = 1.
        factory.addBodiesTo(craft)
        craft.hub.r_CN_NInit = r0 + [dx, 0., 0.]
        craft.hub.v_CN_NInit = velocity
        simulation.AddModelToTask("t", craft, 18000-i)
        truth.append(craft)
    simulation.InitializeSimulation()
    for name, dx in (("a", 0.), ("b", 1000.)):
        body = scene.getBody(name)
        body.setPosition(r0 + [dx, 0., 0.]); body.setVelocity(velocity)
    simulation.ConfigureStopTime(macros.sec2nano(seconds))
    simulation.ExecuteSimulation()
    actual = []
    expected = []
    for name, craft in zip(("a", "b"), truth):
        # Compare authoritative bulk state, not a possibly decimated body message.
        address = scene.getBody(name).getFreeJoint().getQposAdr()
        actual.append(np.asarray(scene.assembleFullQpos()).reshape(-1)[address:address+3])
        expected.append(np.asarray(craft.dynManager.getStateObject(craft.hub.nameOfHubPosition).getState()).reshape(3))
    result = np.array(actual), np.array(expected), stepper.telemetry(), stepper.qpos.copy()
    stepper.close()
    return result


def test_local_pair_orbits_track_native_bsk_spacecraft_and_converge(tmp_path):
    coarse, expected, telemetry, _ = propagate_pair(tmp_path, substeps=1, seconds=5.)
    fine, expected_fine, fine_telemetry, _ = propagate_pair(tmp_path, substeps=4, seconds=5.)
    coarse_error = np.linalg.norm((coarse[1]-coarse[0]) - (expected[1]-expected[0]))
    fine_error = np.linalg.norm((fine[1]-fine[0]) - (expected_fine[1]-expected_fine[0]))
    assert coarse_error < 1e-3, coarse_error  # test fixture tolerance, not a mission claim
    assert fine_error < coarse_error * .7, (coarse_error, fine_error)
    assert telemetry["tidal_mu_m3_s2"] == 0
    assert fine_telemetry["orbital_environment"]["evaluations"] == fine_telemetry["steps"]*4
    assert telemetry["orbital_environment"]["bodies"] == 2


def test_bsk_gravity_does_not_depend_on_mjscene_publication_stride(tmp_path):
    _, _, _, first = propagate_pair(tmp_path, publish_every=1)
    _, _, _, second = propagate_pair(tmp_path, publish_every=8)
    np.testing.assert_allclose(first, second, atol=1e-12, rtol=0)


@pytest.mark.parametrize("models", [{"sun": True}, {"j2": True}])
def test_extended_fields_track_independently_integrated_bsk_spacecraft(tmp_path, models):
    actual, truth, telemetry, _ = propagate_pair(tmp_path, substeps=4, seconds=3., **models)
    # Compare absolute as well as relative states: the same BSK field must reach O.
    np.testing.assert_allclose(actual, truth, atol=5e-4, rtol=0)
    assert telemetry["orbital_environment"]["sources"][0]["central"]


def test_bsk_orbital_rebase_preserves_inertial_trajectory(tmp_path):
    first, _, a, _ = propagate_pair(tmp_path, rebase=50.)
    second, _, b, _ = propagate_pair(tmp_path, rebase=1e9)
    assert a["rebases"] > 0 and b["rebases"] == 0
    np.testing.assert_allclose(first, second, atol=1e-5, rtol=0)

def test_native_bsk_spherical_harmonic_model_and_body_orientation_are_reused():
    from Basilisk.simulation import sphericalHarmonicsGravityModel
    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth(); earth.isCentralBody = True
    model = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
    model.maxDeg = 2
    model.cBar = [[1.], [0., 0.], [-1.08263e-3 / np.sqrt(5.), 0., 0.]]
    model.sBar = [[0.], [0., 0.], [0., 0., 0.]]
    model.radEquator = earth.radEquator; model.muBody = earth.mu
    earth.gravityModel = model
    angle = .6
    rotation = np.array([[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                         [-np.sin(angle), 0., np.cos(angle)]])
    payload = messaging.SpicePlanetStateMsgPayload(); payload.J20002Pfix = rotation.tolist()
    message = messaging.SpicePlanetStateMsg().write(payload)
    env = BskOrbitalEnvironment(spacecraft.Spacecraft())
    env.add_gravity_source("earth", earth, message)
    env.begin_interval()
    x = np.array([[0., 0., 0.], [10000., -3000., 2000.]])
    forces = env.forces(x, np.array([0., 2.]), R, 0)
    common = np.asarray(model.computeField(rotation @ R)).reshape(3)
    displaced = np.asarray(model.computeField(rotation @ (R+x[1]))).reshape(3)
    np.testing.assert_allclose(forces[1], 2*rotation.T @ (displaced-common), atol=2e-14, rtol=1e-10)
    point_mass = 2*(-earth.mu*(R+x[1])/np.linalg.norm(R+x[1])**3 + earth.mu*R/np.linalg.norm(R)**3)
    assert np.linalg.norm(forces[1] - point_mass) > 1e-6
    assert env.telemetry()["sources"][0]["model"] == "SphericalHarmonicsGravityModel"


def test_orbital_ports_reject_missing_or_duplicate_providers():
    from types import SimpleNamespace
    from simulation.physics_ports import PhysicsPorts, PortError
    ports = PhysicsPorts(SimpleNamespace())
    with pytest.raises(PortError):
        ports.orbital_providers()
    with pytest.raises(PortError):
        ports.orbital_environment()
    with pytest.raises(PortError):
        ports.set_orbital_environment(object())
    factory = simIncludeGravBody.gravBodyFactory()
    reference = spacecraft.Spacecraft()
    ports.set_gravity_factory(factory)
    ports.set_orbit_reference(reference)
    assert ports.orbital_providers() == (reference, factory)
    with pytest.raises(PortError):
        ports.set_gravity_factory(factory)
    recorded = []
    ports.set_stepper(SimpleNamespace(set_orbital_environment=recorded.append))
    env = BskOrbitalEnvironment(reference)
    ports.set_orbital_environment(env)
    assert ports.orbital_environment() is env
    assert recorded == [env]
    with pytest.raises(PortError):
        ports.set_orbital_environment(env)


def test_ephemeris_sampler_rejects_stale_nonfinite_and_missing_orientation():
    sampler = LinearEphemerisSampler()
    payload = messaging.SpicePlanetStateMsgPayload()
    with pytest.raises(ValueError, match="orientation"):
        sampler.sample(payload, 0, 0, point_mass=False)
    np.testing.assert_array_equal(sampler.sample(payload, 0, 0, point_mass=True).J20002Pfix, np.eye(3))
    with pytest.raises(ValueError, match="too far"):
        sampler.sample(payload, 0, 2_000_000_000, point_mass=True)
    payload.VelocityVector = [float("nan"), 0., 0.]
    with pytest.raises(ValueError, match="non-finite"):
        sampler.sample(payload, 0, 0, point_mass=True)


def test_rotating_source_sampler_preserves_a_proper_dcm():
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.J20002Pfix = np.eye(3).tolist()
    payload.J20002Pfix_dot = [[0., -.01, 0.], [.01, 0., 0.], [0., 0., 0.]]
    sample = LinearEphemerisSampler().sample(payload, 0, 500_000_000, point_mass=False)
    rotation = np.asarray(sample.J20002Pfix)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-14)
    assert np.linalg.det(rotation) == pytest.approx(1.)
    assert rotation[1, 0] > 0  # do not silently discard orientation evolution


def test_source_configuration_mutation_is_rejected_before_force_evaluation():
    env, keep = environment()
    env.sources["earth"]["body"].mu *= 1.01
    with pytest.raises(ValueError, match="configuration changed"):
        env.begin_interval()


def test_sampler_can_be_replaced_without_modifying_the_stepper():
    calls = []

    class Sampler:
        name = "test_sampler"

        def sample(self, payload, epoch_ns, nanos, *, point_mass):
            calls.append((epoch_ns, nanos, point_mass))
            return payload

    env, keep = environment()
    env.ephemeris_sampler = Sampler()
    env.forces(np.zeros((2, 3)), np.array([0., 2.]), R, 12)
    assert calls == [(0, 12, True)]
    assert env.telemetry()["ephemeris_sampling"] == "test_sampler"


def test_missing_or_multiple_central_bodies_are_not_silently_assumed():
    env = BskOrbitalEnvironment(spacecraft.Spacecraft())
    moon = simIncludeGravBody.gravBodyFactory().createMoon()
    msg = source_message((3.8e8, 0., 0.))
    env.add_gravity_source("moon", moon, msg)
    env.begin_interval()
    with pytest.raises(ValueError, match="exactly one central"):
        env.forces(np.zeros((2, 3)), np.array([0., 2.]), R, 0)


def test_force_provider_substeps_are_fresh_additive_and_integer_timed(tmp_path):
    from Basilisk.simulation import mujoco
    from Basilisk.utilities import SimulationBaseClass
    from simulation.local_mujoco_stepper import LocalMujocoStepper

    class Provider:
        def reset(self):
            self.times, self.positions = [], []

        def begin_interval(self):
            pass

        def forces(self, positions, masses, origin, nanos):
            self.times.append(nanos)
            self.positions.append(positions[1].copy())
            result = np.zeros_like(positions)
            result[1, 0] = 2.0
            return result

        def telemetry(self):
            return {"mode": "fixture"}

    path = tmp_path / "orbital_wrench.xml"
    path.write_text('<mujoco><option gravity="0 0 0"/><worldbody>'
                    '<body name="body"><freejoint/><geom type="sphere" size=".1" mass="2"/>'
                    '</body></worldbody></mujoco>', encoding="utf-8")
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("p")
    process.addTask(sim.CreateNewTask("t", 1_000_000))
    process.addTask(sim.CreateNewTask("init_scene", 10**18))
    scene = mujoco.MJScene.fromFile(str(path))
    sim.AddModelToTask("init_scene", scene)
    stepper = LocalMujocoStepper(scene, path, [], substeps=3, publish_every=8)
    provider = Provider()
    stepper.set_orbital_environment(provider)
    force = messaging.CmdForceInertialMsg().write(
        messaging.CmdForceInertialMsgPayload(forceRequestInertial=[4., 0., 0.]))
    stepper.add_body_wrench_input("body", force, source="test_thrust")
    sim.AddModelToTask("t", stepper)
    try:
        sim.InitializeSimulation()
        sim.ConfigureStopTime(10_000_000)
        sim.ExecuteSimulation()
        assert stepper.qvel[0] == pytest.approx(3. * .01, abs=1e-13)  # (2 N + 4 N)/2 kg
        assert provider.times == [k*1_000_000 + j*1_000_000//3 for k in range(10) for j in range(3)]
        assert np.all(np.diff(np.asarray(provider.positions)[:, 0]) > 0)
        assert stepper.telemetry()["tidal_mu_m3_s2"] == 0
        # Native layer rejects simultaneous analytic tide and provider gravity.
        ctrl, origin, qpos, qvel, qacc, effort, stats = stepper._pointers
        code = stepper._lib.lms_step(stepper._handle, ctrl, .001, 1, origin,
                                     3.986e14, qpos, qvel, qacc, effort, stats)
        assert code < 0
    finally:
        stepper.close()
