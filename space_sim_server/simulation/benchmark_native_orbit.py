"""Native single-spacecraft orbit dynamics; private evidence and public GNSS."""
from __future__ import annotations

from pathlib import Path
import uuid
import numpy as np
from Basilisk.architecture import sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk, SimulationBaseClass

from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.benchmark_native_flight import _InertialForceActuator
from simulation.benchmark_orbit_model import prepare_orbit_model
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.navigation_gravity import BskNavigationGravity
from simulation.orbital_rendezvous.navigation import OrbitFilter
from space_arm_platform.benchmark_orbit import (
    EARTH_MU_M3_S2 as MU, EARTH_RADIUS_M as RADIUS, OrbitContext, OrbitObservation, OrbitTruthSample,
)
from space_arm_platform.benchmark_orbit_score import OrbitScorer
from space_arm_platform.orbital_physics import OrbitalPhysics
from space_arm_platform.rendezvous_config import FlightOptions


class _OrbitEvidence(sysModel.SysModel):
    def __init__(self, environment):
        super().__init__()
        self.ModelTag = "benchmarkPrivateOrbitEvidence"
        self.env = environment

    def UpdateState(self, nanos):
        env = self.env
        state = env._physical_state(nanos)
        if int(env._body_reader.timeWritten()) != nanos:
            raise RuntimeError("stale attitude cannot rotate native motor evidence")
        sample = OrbitTruthSample(time_ns=int(nanos), chaser_com_j2000=tuple(state),
            orientation_body_to_j2000_wxyz=tuple(rbk.MRP2EP(env._body_reader().sigma_BN)),
            applied_motor_force_body_n=tuple(float(env._stepper.actuator_force[i]) for i in env._force_indices),
            external_state_writes=env._stepper.external_writes)
        env._scorer.add(sample)
        env._truth_stream.write(sample.model_dump_json()+"\n")


class NativeOrbitEnvironment:
    context_type = OrbitContext
    scorer_type = OrbitScorer

    def __init__(self, task, output_directory):
        self.task, self.output_directory = task, output_directory.resolve()
        self.repository = Path(__file__).resolve().parents[1]
        self._simulation = self._stepper = self._truth_stream = self._observation = None

    def reset(self, seed):
        self.task.seeds.split_for(seed)
        self.close()
        self.episode_id = uuid.uuid4().hex
        self.episode_directory = self.output_directory / "episodes" / self.episode_id
        self.episode_directory.mkdir(parents=True, exist_ok=False)
        model, meshes, self.model_manifest = prepare_orbit_model(self.repository, self.episode_directory / "model",
                                                                force_limit_n=self.task.safety.maximum_force_n)
        self.context = self.context_type(task=self.task, chaser_mass_kg=self.model_manifest["bodies"]["cubesat_bus"]["mass_kg"])
        sim = self._simulation = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("autonomyOrbitBenchmark")
        process.addTask(sim.CreateNewTask("benchmarkPhysics", self.task.physics_period_ns), 30)
        process.addTask(sim.CreateNewTask("benchmarkPublisherInit", self.task.physics_period_ns))
        scene = self._scene = mujoco.MJScene.fromFile(str(model), files=meshes)
        scene.ModelTag = "benchmarkSingleOrbitMJScene"
        scene.extraEoMCall = True
        sim.AddModelToTask("benchmarkPublisherInit", scene)
        ctx = self._assembly = AssemblyContext(sim, scene, config=SessionConfig(model_path=model,
            dynamics_backend="local", publish_stride=1, clock_rate_hz=50, physics_task="benchmarkPhysics"))
        from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC
        options = FlightOptions(seed=seed, orbital_physics=OrbitalPhysics(earth_gravity=self.task.earth_gravity).model_dump(mode="json"),
                                drag_enabled=self.task.drag_enabled, srp_enabled=self.task.srp_enabled,
                                orbit_acceleration_sigma_m_s2=self.task.disturbance_sigma_m_s2)
        orbit = self._orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=options.physics)
        ctx.install([orbit])
        self._stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin,
                                         substeps=self.task.physics_substeps, publish_every=1)
        ctx.ports.set_stepper(self._stepper)
        ctx.add(self._stepper, Slot.PHYSICS, component="dynamics_core")
        ctx.install([OrbitalGravityComponent(force_every=1), FlightEnvironmentComponent(options, [
            ("cubesat_bus", "orbital_thrust_center", self.context.chaser_mass_kg, options.chaser_area_m2)])])
        self._actuator = _InertialForceActuator(ctx.ports)
        ctx.add(self._actuator, Slot.ACTUATOR, component="benchmark_actuator")
        self._state_reader = ctx.ports.site_state("orbital_thrust_center").addSubscriber()
        self._body_reader = ctx.ports.body_state("cubesat_bus").addSubscriber()
        self._force_indices = [self._stepper.actuator_names.index(f"orbital_force_{i}") for i in range(3)]
        self._setup_observation(ctx)
        self._sensor_rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))
        self._scorer = self.scorer_type(self.task, self.context.chaser_mass_kg)
        self._truth_stream = (self.episode_directory / "physics.jsonl").open("w", encoding="utf-8", newline="\n")
        self._recorder = _OrbitEvidence(self)
        ctx.add(self._recorder, Slot.RECORD, component="benchmark_evidence")
        sim.InitializeSimulation()
        sim.disableTask("benchmarkPublisherInit")
        settings = self.task.initial
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        radius = RADIUS+settings.altitude_m+rng.uniform(-settings.altitude_half_width_m, settings.altitude_half_width_m)
        theta = np.deg2rad(settings.true_anomaly_deg+rng.uniform(-settings.anomaly_half_width_deg, settings.anomaly_half_width_deg))
        inc = np.deg2rad(settings.inclination_deg)
        radial = np.array([np.cos(theta), np.sin(theta)*np.cos(inc), np.sin(theta)*np.sin(inc)])
        tangent = np.array([-np.sin(theta), np.cos(theta)*np.cos(inc), np.cos(theta)*np.sin(inc)])
        state = np.r_[radius*radial, np.sqrt(MU/radius)*tangent]
        body = scene.getBody("cubesat_bus")
        center = np.asarray(self.model_manifest["bodies"]["cubesat_bus"]["center_body_m"])
        body.setPosition((state[:3]-center).tolist())
        body.setVelocity(state[3:].tolist())
        body.setAttitude([0., 0., 0.])
        body.setAttitudeRate([0., 0., 0.])
        orbit.origin.set_state(state[:3], state[3:])
        self._time_ns = 0
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        self._observation = self._measure(0)
        self.initial_states_j2000 = [state.tolist()]
        self.environment_options = options.to_dict()
        return self._observation

    def _setup_observation(self, ctx):
        self._filter = OrbitFilter(self.task.observation.position_sigma_m, self.task.observation.velocity_sigma_m_s,
                                   gravity_model=BskNavigationGravity(ctx.ports.orbital_environment()))

    def _physical_state(self, nanos):
        if int(self._state_reader.timeWritten()) != nanos:
            raise RuntimeError("stale native COM publication cannot be benchmark evidence")
        state = self._state_reader()
        return np.r_[state.r_BN_N, state.v_BN_N]

    def _measure(self, sequence):
        physical = self._physical_state(self._time_ns)
        noise = np.array([self.task.observation.position_sigma_m]*3+[self.task.observation.velocity_sigma_m_s]*3)
        measurement = physical+self._sensor_rng.normal(size=6)*noise
        dt = self.task.decision_period_ns*1e-9 if sequence else 0.
        estimate = self._filter.update(measurement, dt, self._actuator.command/self.context.chaser_mass_kg,
                                       time_s=self._time_ns*1e-9)
        return OrbitObservation(episode_id=self.episode_id, sequence=sequence, sample_time_ns=self._time_ns,
                                delivery_time_ns=self._time_ns, chaser_estimate_j2000=tuple(estimate))

    def step(self, action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError("reset must precede step")
        if self.evaluation()["outcome"] != "incomplete":
            raise RuntimeError("episode has terminated; reset before another step")
        action.validate_for(self._observation, self.task.safety.maximum_force_n)
        self._actuator.command = np.asarray(action.force_j2000_n)
        self._time_ns += self.task.decision_period_ns
        self._simulation.ConfigureStopTime(self._time_ns)
        self._simulation.ExecuteSimulation()
        self._observation = self._measure(self._observation.sequence+1)
        return self._observation

    def evaluation(self):
        return self._scorer.result()

    def provenance(self):
        from simulation.mujoco_runtime import basilisk_mujoco_runtime
        from space_arm_platform.benchmark_execution import sha256
        return {"model": self.model_manifest, "initial_states_j2000": self.initial_states_j2000,
                "environment_options": self.environment_options, "schedule": self._assembly.summary(),
                "native_dynamics": self._stepper.telemetry(), "propagation_earth_mu_m3_s2": float(self._orbit.earth.mu),
                "native_build_manifest_sha256": sha256(self.repository / "run/native_acceleration/local_mujoco_stepper.json"),
                "basilisk_mujoco_runtime_sha256": sha256(basilisk_mujoco_runtime()),
                "observation_model": "single noisy GNSS solution and orbit filter; no target or attitude telemetry",
                "actuation_model": "bounded ideal J2000 force through native COM motors; passive attitude"}

    def close(self):
        if getattr(self, "_actuator", None) is not None:
            self._actuator.command = np.zeros(3)
        if self._truth_stream is not None:
            self._truth_stream.close()
            self._truth_stream = None
        if self._stepper is not None:
            self._stepper.close()
            self._stepper = None
        self._simulation = self._observation = None
