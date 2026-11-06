"""Native single-spacecraft attitude environment using original wheel hardware."""
from __future__ import annotations

from pathlib import Path
from space_arm_platform.benchmark_paths import evidence_path
from types import SimpleNamespace
import uuid

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass

from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.attitude_control import WheelDrive, WheelDriveGroup, load_hardware
from simulation.benchmark_attitude_model import prepare_attitude_model
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.gyroscope import GyroConfig, Gyroscope
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_attitude import (
    AttitudeContext, AttitudeObservation, AttitudeTask, AttitudeTruthSample, DetumbleInitialState, WheelTorqueAction,
)
from space_arm_platform.benchmark_attitude_score import AttitudeScorer
from space_arm_platform.orbital_physics import OrbitalPhysics
from space_arm_platform.rendezvous_config import FlightOptions


def rotation_quaternion(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-15:
        return np.array([1., 0., 0., 0.])
    return np.r_[np.cos(angle/2), np.asarray(vector) * (np.sin(angle/2) / angle)]


class _AttitudeEvidence(sysModel.SysModel):
    def __init__(self, environment):
        super().__init__()
        self.ModelTag = "benchmarkPrivateAttitudeEvidence"
        self.environment = environment

    def UpdateState(self, nanos):
        env = self.environment
        if int(env._body_reader.timeWritten()) != nanos:
            raise RuntimeError("stale native attitude publication")
        sample = env._truth_sample(nanos)
        env._scorer.add(sample)
        env._truth_stream.write(sample.model_dump_json() + "\n")


class NativeAttitudeEnvironment:
    context_type = AttitudeContext
    scorer_type = AttitudeScorer

    def __init__(self, task: AttitudeTask, output_directory: Path):
        self.task, self.output_directory = task, evidence_path(output_directory)
        self.repository = Path(__file__).resolve().parents[1]
        self._simulation = self._stepper = self._truth_stream = self._observation = None

    def reset(self, seed):
        self.task.seeds.split_for(seed)
        self.close()
        self.episode_id = uuid.uuid4().hex
        self.episode_directory = self.output_directory / "episodes" / self.episode_id
        self.episode_directory.mkdir(parents=True, exist_ok=False)
        model, meshes, self.model_manifest = prepare_attitude_model(self.repository, self.episode_directory / "model")
        wheels, _ = load_hardware(model)
        if (self.task.safety.maximum_motor_torque_nm > min(w.max_torque for w in wheels)
                or self.task.safety.maximum_wheel_speed_rad_s > min(w.max_speed for w in wheels)):
            raise ValueError("task limits exceed the original wheel hardware limits")
        body_properties = self.model_manifest["bodies"]["cubesat_bus"]
        self.context = self.context_type(task=self.task, chaser_mass_kg=body_properties["mass_kg"],
            inertia_com_body_kg_m2=body_properties["inertia_com_body_kg_m2"],
            wheel_spin_inertia_kg_m2=tuple(w.spin_inertia for w in wheels),
            hardware_torque_limits_nm=tuple(w.max_torque for w in wheels),
            hardware_speed_limits_rad_s=tuple(w.max_speed for w in wheels))
        sim = self._simulation = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess("autonomyAttitudeBenchmark")
        process.addTask(sim.CreateNewTask("benchmarkPhysics", self.task.physics_period_ns), 30)
        process.addTask(sim.CreateNewTask("benchmarkPublisherInit", self.task.physics_period_ns))
        scene = self._scene = mujoco.MJScene.fromFile(str(model), files=meshes)
        scene.ModelTag, scene.extraEoMCall = "benchmarkSingleSARM", True
        sim.AddModelToTask("benchmarkPublisherInit", scene)
        ctx = self._assembly = AssemblyContext(sim, scene, config=SessionConfig(
            model_path=model, dynamics_backend="local", publish_stride=1,
            clock_rate_hz=50, physics_task="benchmarkPhysics"))
        from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC
        options = FlightOptions(seed=seed, orbital_physics=OrbitalPhysics(earth_gravity=self.task.earth_gravity).model_dump(mode="json"),
                                drag_enabled=self.task.drag_enabled, srp_enabled=self.task.srp_enabled,
                                orbit_acceleration_sigma_m_s2=self.task.disturbance_sigma_m_s2)
        orbit = self._orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=options.physics)
        ctx.install([orbit])
        self._stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin,
                                         substeps=self.task.physics_substeps, publish_every=1, model_from_memory=True)
        ctx.ports.set_stepper(self._stepper)
        ctx.add(self._stepper, Slot.PHYSICS, component="dynamics_core")
        ctx.install([OrbitalGravityComponent(force_every=1)])
        self._environment = FlightEnvironmentComponent(options, [
            ("cubesat_bus", "orbital_thrust_center", self.context.chaser_mass_kg, options.chaser_area_m2)])
        ctx.install([self._environment])
        joints = [scene.getBody(wheel.body).getScalarJoint(wheel.joint) for wheel in wheels]
        self._wheel_owner = SimpleNamespace(wheels=wheels, joints=joints, enabled=True,
            cached_speed_reads=True, settings={"speed_guard_fraction": 0.98})
        self._motor_messages = [messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=0.)) for _ in wheels]
        self._drives = [self._make_wheel_drive(i, message) for i, message in enumerate(self._motor_messages)]
        for wheel, drive in zip(wheels, self._drives, strict=True):
            ctx.ports.drive_actuator(wheel.motor, drive.actuatorOutMsg, owner="benchmark_wheel_method")
        self._drive_group = WheelDriveGroup(self._drives)
        ctx.add(self._drive_group, Slot.ACTUATOR, component="benchmark_wheel_drives")
        self._body_reader = ctx.ports.body_state("cubesat_bus").addSubscriber()
        self._wheel_readers = [joint.stateDotOutMsg.addSubscriber() for joint in joints]
        self._motor_indices = [self._stepper.actuator_names.index(w.motor) for w in wheels]
        self._sensor_rng = np.random.default_rng(np.random.SeedSequence([seed, 3]))
        obs = self.task.observation
        self._gyro = Gyroscope(GyroConfig(white_noise_std_rad_s=obs.gyro_white_noise_std_rad_s,
            initial_bias_std_rad_s=obs.gyro_initial_bias_std_rad_s,
            bias_random_walk_rad_s_sqrt_s=obs.gyro_bias_random_walk_rad_s_sqrt_s,
            range_rad_s=obs.gyro_range_rad_s), seed=seed)
        self._setup_additional_observation(ctx, seed)
        self._scorer = self.scorer_type(self.task)
        self._truth_stream = (self.episode_directory / "physics.jsonl").open("w", encoding="utf-8", newline="\n")
        self._recorder = _AttitudeEvidence(self)
        ctx.add(self._recorder, Slot.RECORD, component="benchmark_evidence")
        sim.InitializeSimulation()
        sim.disableTask("benchmarkPublisherInit")
        state, quaternion, omega = self._initial_state(seed)
        rotation = rbk.EP2C(quaternion).T
        center = np.asarray(body_properties["center_body_m"])
        body = scene.getBody("cubesat_bus")
        body.setPosition((state[:3] - rotation @ center).tolist())
        body.setVelocity((state[3:] - rotation @ np.cross(omega, center)).tolist())
        body.setAttitude(rbk.EP2MRP(quaternion).tolist())
        body.setAttitudeRate(omega.tolist())
        for joint, speed in zip(joints, self.task.initial.wheel_speed_rad_s, strict=True):
            joint.setVelocity(speed)
        orbit.origin.set_state(state[:3], state[3:])
        self._time_ns = 0
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        self._observation = self._measure(0)
        self.initial_states_j2000 = [state.tolist()]
        self.initial_attitude = {"orientation_body_to_j2000_wxyz": quaternion.tolist(),
                                 "angular_velocity_body_rad_s": omega.tolist()}
        self.environment_options = options.to_dict()
        return self._observation

    def _setup_additional_observation(self, ctx, seed):
        pass

    def _make_wheel_drive(self, index, message):
        return WheelDrive(self._wheel_owner, index, message)

    def _truth_sample(self, nanos):
        state = self._body_reader()
        return AttitudeTruthSample(time_ns=int(nanos),
            orientation_body_to_j2000_wxyz=tuple(rbk.MRP2EP(state.sigma_BN)),
            angular_velocity_body_rad_s=tuple(state.omega_BN_B),
            wheel_speed_rad_s=tuple(float(read().state) for read in self._wheel_readers),
            applied_motor_torque_xyz_nm=tuple(float(self._stepper.actuator_force[i]) for i in self._motor_indices),
            external_state_writes=self._stepper.external_writes)

    def _initial_state(self, seed):
        initial = self.task.initial
        radius = EARTH_RADIUS + initial.altitude_m
        theta, inc = np.deg2rad([initial.true_anomaly_deg, initial.inclination_deg])
        radial = np.array([np.cos(theta), np.sin(theta)*np.cos(inc), np.sin(theta)*np.sin(inc)])
        tangent = np.array([-np.sin(theta), np.cos(theta)*np.cos(inc), np.cos(theta)*np.sin(inc)])
        state = np.r_[radius*radial, np.sqrt(MU/radius)*tangent]
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        delta = rotation_quaternion(axis * rng.uniform(initial.rotation_min_rad, initial.rotation_max_rad))
        quaternion = rbk.C2EP(rbk.EP2C(delta) @ rbk.EP2C(self.task.objective.orientation_body_to_j2000_wxyz))
        quaternion /= np.linalg.norm(quaternion)
        omega = rng.uniform(-initial.body_rate_half_width_rad_s, initial.body_rate_half_width_rad_s, 3)
        if isinstance(initial, DetumbleInitialState):
            # The contract limits the norm floor to the cube half-width, so
            # rejection sampling accepts at least 1-pi/6 of candidates.
            while np.linalg.norm(omega) < initial.body_rate_min_norm_rad_s:
                omega = rng.uniform(-initial.body_rate_half_width_rad_s, initial.body_rate_half_width_rad_s, 3)
        return state, quaternion, omega

    def _measure(self, sequence):
        if int(self._body_reader.timeWritten()) != self._time_ns:
            raise RuntimeError("stale native attitude sensor sample")
        physical = self._body_reader()
        noise = rotation_quaternion(self._sensor_rng.normal(0., self.task.observation.orientation_noise_std_rad, 3))
        measured = rbk.C2EP(rbk.EP2C(noise) @ rbk.MRP2C(physical.sigma_BN))
        measured /= np.linalg.norm(measured)
        gyro = self._gyro.sample(self._time_ns, physical.omega_BN_B)
        return AttitudeObservation(episode_id=self.episode_id, sequence=sequence, sample_time_ns=self._time_ns,
            delivery_time_ns=self._time_ns, orientation_body_to_j2000_wxyz=tuple(measured),
            angular_velocity_body_rad_s=tuple(gyro["angular_velocity_body_rad_s"]), gyro_valid=gyro["valid"],
            wheel_speed_rad_s=tuple(float(read().state) for read in self._wheel_readers))

    def step(self, action: WheelTorqueAction):
        if self._simulation is None or self._observation is None:
            raise RuntimeError("reset must precede step")
        if self.evaluation()["outcome"] != "incomplete":
            raise RuntimeError("episode has terminated; reset before another step")
        action.validate_for(self._observation, self.task.safety.maximum_motor_torque_nm)
        for message, torque in zip(self._motor_messages, action.motor_torque_xyz_nm, strict=True):
            message.write(messaging.SingleActuatorMsgPayload(input=torque), self._time_ns)
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
        return {"model": self.model_manifest, "initial_states_j2000": self.initial_states_j2000,
                "initial_attitude": self.initial_attitude, "environment_options": self.environment_options,
                "schedule": self._assembly.summary(), "native_dynamics": self._stepper.telemetry(),
                "native_build_manifest_sha256": sha256(self.repository / "run/native_acceleration/local_mujoco_stepper.json"),
                "basilisk_mujoco_runtime_sha256": sha256(basilisk_mujoco_runtime()),
                "observation_model": "noisy absolute attitude telemetry; existing stochastic 50 Hz gyroscope; ideal wheel encoders; no images",
                "actuation_model": "original native reaction-wheel joint motors, existing torque/speed protection; no body torque injection"}

    def close(self):
        if self._truth_stream is not None:
            self._truth_stream.close()
            self._truth_stream = None
        if self._stepper is not None:
            self._stepper.close()
            self._stepper = None
        self._simulation = self._observation = None
