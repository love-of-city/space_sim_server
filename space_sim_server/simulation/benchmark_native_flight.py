"""Native SpaceSim environment for the versioned cooperative waypoint task.

Run in a dedicated process. The method receives immutable navigation messages,
while a separate recorder grades raw physical COM states at 50 Hz. The existing
local integrator advances eight substeps per tick (400 Hz), evaluates BSK gravity
at each substep and never accepts post-initialization state assignments.
"""
from __future__ import annotations

from pathlib import Path
import uuid

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass

from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.navigation_gravity import BskNavigationGravity
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.model import prepare_model
from simulation.orbital_rendezvous.navigation import OrbitFilter
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_contract import (
    FlightTruthSample, ForceAction, MethodContext, NavigationObservation, WaypointTask,
)
from space_arm_platform.benchmark_scoring import WaypointScorer
from space_arm_platform.orbital_physics import OrbitalPhysics
from space_arm_platform.rendezvous_config import FlightOptions


class _InertialForceActuator(sysModel.SysModel):
    """Ideal inertial-vector actuation, implemented by actual native motors.

The ideal actuator uses physical orientation internally to realize its declared
J2000 command. Orientation is not an observation and this is not attitude GNC.
There is no hidden truth-based attitude-hold controller.
"""
    def __init__(self, ports):
        super().__init__()
        self.ModelTag = "benchmarkIdealInertialForce"
        self.command = np.zeros(3)
        self.body_reader = ports.body_state("cubesat_bus").addSubscriber()
        self.messages = [messaging.SingleActuatorMsg() for _ in range(3)]
        for index, message in enumerate(self.messages):
            message.write(messaging.SingleActuatorMsgPayload(input=0.0))
            ports.drive_actuator(f"orbital_force_{index}", message, owner="benchmark_method")
        # The stowed model retains physical reaction wheels. Passive means an
        # explicit zero motor command, not an unconnected native message port.
        self.passive_command = messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=0.0))
        for name in ports.stepper.actuator_names:
            if name.startswith("orbital_force_"):
                continue
            if not name.startswith("rw_"):
                raise ValueError(f"unexpected non-flight actuator in the stowed model: {name}")
            ports.drive_actuator(name, self.passive_command, owner="benchmark_passive_wheels")

    def UpdateState(self, nanos):
        body_force = rbk.MRP2C(self.body_reader().sigma_BN) @ self.command
        for message, value in zip(self.messages, body_force, strict=True):
            message.write(messaging.SingleActuatorMsgPayload(input=float(value)), nanos)


class _PhysicsEvidence(sysModel.SysModel):
    def __init__(self, environment):
        super().__init__()
        self.ModelTag = "benchmarkPrivatePhysicsEvidence"
        self.environment = environment

    def UpdateState(self, nanos):
        env = self.environment
        states = env._read_physical_states(nanos)
        sample = FlightTruthSample(
            time_ns=int(nanos), target_com_j2000=tuple(states[0]), chaser_com_j2000=tuple(states[1]),
            applied_motor_force_body_n=tuple(float(env._stepper.actuator_force[i]) for i in env._force_indices),
            external_state_writes=env._stepper.external_writes,
        )
        env._scorer.add(sample)
        env._truth_stream.write(sample.model_dump_json() + "\n")


class NativeWaypointEnvironment:
    def __init__(self, task: WaypointTask, output_directory: Path):
        self.task = task
        self.output_directory = output_directory.resolve()
        self.repository = Path(__file__).resolve().parents[1]
        self._simulation = None
        self._stepper = None
        self._truth_stream = None
        self._observation = None

    def reset(self, seed: int) -> NavigationObservation:
        self.task.seeds.split_for(seed)
        self.close()
        self.episode_id = uuid.uuid4().hex
        self.episode_directory = self.output_directory / "episodes" / self.episode_id
        self.episode_directory.mkdir(parents=True, exist_ok=False)
        model, meshes, self.model_manifest = prepare_model(
            self.repository, self.episode_directory / "model", force_limit_n=self.task.safety.maximum_force_n)
        self.context = MethodContext(task=self.task, chaser_mass_kg=self.model_manifest["bodies"]["cubesat_bus"]["mass_kg"])
        sim = self._simulation = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("autonomyBenchmark")
        process.addTask(sim.CreateNewTask("benchmarkPhysics", self.task.physics_period_ns), 30)
        process.addTask(sim.CreateNewTask("benchmarkPublisherInit", self.task.physics_period_ns))
        scene = self._scene = mujoco.MJScene.fromFile(str(model), files=meshes)
        scene.ModelTag = "benchmarkTeamMJScene"
        scene.extraEoMCall = True
        sim.AddModelToTask("benchmarkPublisherInit", scene)
        ctx = self._assembly = AssemblyContext(sim, scene, config=SessionConfig(
            model_path=model, dynamics_backend="local", publish_stride=1,
            clock_rate_hz=50, physics_task="benchmarkPhysics"))
        from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC
        options = FlightOptions(seed=seed, orbital_physics=OrbitalPhysics(earth_gravity=self.task.earth_gravity).model_dump(mode="json"),
                                drag_enabled=self.task.drag_enabled, srp_enabled=self.task.srp_enabled,
                                orbit_acceleration_sigma_m_s2=self.task.disturbance_sigma_m_s2)
        orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=options.physics)
        ctx.install([orbit])
        self._stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin,
                                         substeps=self.task.physics_substeps, publish_every=1)
        ctx.ports.set_stepper(self._stepper)
        ctx.add(self._stepper, Slot.PHYSICS, component="dynamics_core")
        ctx.install([OrbitalGravityComponent(force_every=1)])
        self._environment = FlightEnvironmentComponent(options, [
            ("capture_target", "target_flight_center", self.model_manifest["bodies"]["capture_target"]["mass_kg"], options.target_area_m2),
            ("cubesat_bus", "orbital_thrust_center", self.context.chaser_mass_kg, options.chaser_area_m2),
        ])
        ctx.install([self._environment])
        self._actuator = _InertialForceActuator(ctx.ports)
        ctx.add(self._actuator, Slot.ACTUATOR, component="benchmark_actuator")
        self._state_readers = [ctx.ports.site_state(name).addSubscriber()
                               for name in ("target_flight_center", "orbital_thrust_center")]
        self._force_indices = [self._stepper.actuator_names.index(f"orbital_force_{i}") for i in range(3)]
        gravity = BskNavigationGravity(ctx.ports.orbital_environment())
        self._filters = [OrbitFilter(self.task.observation.position_sigma_m,
                                     self.task.observation.velocity_sigma_m_s, gravity_model=gravity) for _ in range(2)]
        self._sensor_rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))
        self._scorer = WaypointScorer(self.task, self.context.chaser_mass_kg)
        self._truth_stream = (self.episode_directory / "physics.jsonl").open("w", encoding="utf-8", newline="\n")
        self._recorder = _PhysicsEvidence(self)
        ctx.add(self._recorder, Slot.RECORD, component="benchmark_evidence")
        sim.InitializeSimulation()
        sim.disableTask("benchmarkPublisherInit")
        states = self._initial_states(seed)
        for index, name in enumerate(("capture_target", "cubesat_bus")):
            center = np.asarray(self.model_manifest["bodies"][name]["center_body_m"])
            body = scene.getBody(name)
            body.setPosition((states[index, :3] - center).tolist())
            body.setVelocity(states[index, 3:].tolist())
            body.setAttitude([0.0, 0.0, 0.0])
            body.setAttitudeRate([0.0, 0.0, 0.0])
        orbit.origin.set_state(states[0, :3], states[0, 3:])
        self._time_ns = 0
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        self._observation = self._measure(sequence=0)
        self.initial_states_j2000 = states.tolist()
        self.environment_options = options.to_dict()
        return self._observation

    def _initial_states(self, seed):
        settings = self.task.initial
        radius = EARTH_RADIUS + settings.target_altitude_m
        theta, inclination = np.deg2rad([settings.true_anomaly_deg, settings.inclination_deg])
        radial = np.array([np.cos(theta), np.sin(theta) * np.cos(inclination), np.sin(theta) * np.sin(inclination)])
        tangent = np.array([-np.sin(theta), np.cos(theta) * np.cos(inclination), np.cos(theta) * np.sin(inclination)])
        target = np.r_[radius * radial, np.sqrt(MU / radius) * tangent]
        frame, omega = vbar_frame(target)
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        position = np.asarray(settings.relative_position_tnr_m) + rng.uniform(-settings.position_half_width_m, settings.position_half_width_m, 3)
        velocity = np.asarray(settings.relative_velocity_tnr_m_s) + rng.uniform(-settings.velocity_half_width_m_s, settings.velocity_half_width_m_s, 3)
        dr = frame @ position
        dv = frame @ velocity + np.cross(omega, dr)
        return np.array([target, target + np.r_[dr, dv]])

    def _read_physical_states(self, nanos):
        if any(int(reader.timeWritten()) != nanos for reader in self._state_readers):
            raise RuntimeError("stale native COM publication cannot be benchmark evidence")
        return np.asarray([np.r_[p.r_BN_N, p.v_BN_N] for p in (reader() for reader in self._state_readers)])

    def _measure(self, sequence):
        physical = self._read_physical_states(self._time_ns)
        sigma = np.array([self.task.observation.position_sigma_m] * 3 + [self.task.observation.velocity_sigma_m_s] * 3)
        measurements = physical + self._sensor_rng.normal(size=(2, 6)) * sigma
        dt = self.task.decision_period_ns * 1e-9 if sequence else 0.0
        estimates = [f.update(measurement, dt, self._actuator.command / self.context.chaser_mass_kg if i else np.zeros(3),
                              time_s=self._time_ns * 1e-9)
                     for i, (f, measurement) in enumerate(zip(self._filters, measurements, strict=True))]
        return NavigationObservation(episode_id=self.episode_id, sequence=sequence,
                                     sample_time_ns=self._time_ns, delivery_time_ns=self._time_ns,
                                     target_estimate_j2000=tuple(estimates[0]), chaser_estimate_j2000=tuple(estimates[1]))

    def step(self, action: ForceAction) -> NavigationObservation:
        if self._simulation is None or self._observation is None:
            raise RuntimeError("reset must precede step")
        if self.evaluation()["outcome"] != "incomplete":
            raise RuntimeError("episode has terminated; reset before another step")
        action.validate_for(self._observation, self.task.safety.maximum_force_n)
        self._actuator.command = np.asarray(action.force_j2000_n)
        self._time_ns += self.task.decision_period_ns
        self._simulation.ConfigureStopTime(self._time_ns)
        self._simulation.ExecuteSimulation()
        self._observation = self._measure(self._observation.sequence + 1)
        return self._observation

    def evaluation(self):
        return self._scorer.result()

    def provenance(self):
        from simulation.mujoco_runtime import basilisk_mujoco_runtime
        from space_arm_platform.benchmark_execution import sha256
        native = self.repository / "run/native_acceleration/local_mujoco_stepper.json"
        return {"model": self.model_manifest, "initial_states_j2000": self.initial_states_j2000,
                "environment_options": self.environment_options,
                "schedule": self._assembly.summary(), "native_dynamics": self._stepper.telemetry(),
                "native_build_manifest_sha256": sha256(native),
                "basilisk_mujoco_runtime_sha256": sha256(basilisk_mujoco_runtime()),
                "observation_model": "zero-latency independent GNSS solutions; filtered target shared over cooperative link",
                "actuation_model": "bounded ideal J2000 force; native body-axis motors at assembly COM; passive attitude"}

    def close(self):
        if getattr(self, "_actuator", None) is not None:
            self._actuator.command = np.zeros(3)
        if self._truth_stream is not None:
            self._truth_stream.close()
            self._truth_stream = None
        if self._stepper is not None:
            self._stepper.close()
            self._stepper = None
        self._simulation = None
        self._observation = None
