"""Native cooperative formation with a distinct force actuator and sensor on each SARM."""
from pathlib import Path
import uuid

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk, SimulationBaseClass

from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.benchmark_formation_model import prepare_formation_model
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.navigation_gravity import BskNavigationGravity
from simulation.near_field.orbital_guidance import vbar_frame
from simulation.orbital_rendezvous.navigation import OrbitFilter
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, MU
from space_arm_platform.benchmark_formation import FormationContext, FormationObservation, FormationTruthSample
from space_arm_platform.benchmark_formation_score import FormationScorer
from space_arm_platform.benchmark_paths import evidence_path
from space_arm_platform.orbital_physics import OrbitalPhysics
from space_arm_platform.rendezvous_config import FlightOptions


class _FormationActuators(sysModel.SysModel):
    def __init__(self, ports, crafts):
        super().__init__()
        self.ModelTag = 'benchmarkIndividualFormationForces'
        self.commands = np.zeros((len(crafts), 3))
        self.readers, self.messages = [], []
        self.passive = messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=0.))
        for craft in crafts:
            self.readers.append(ports.body_state(craft['body']).addSubscriber())
            messages = []
            for actuator in craft['force_actuators']:
                message = messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=0.))
                ports.drive_actuator(actuator, message, owner='formation_method')
                messages.append(message)
            self.messages.append(messages)
            for actuator in craft['wheel_actuators']:
                ports.drive_actuator(actuator, self.passive, owner='formation_passive_wheels')

    def UpdateState(self, nanos):
        for command, reader, messages in zip(self.commands, self.readers, self.messages, strict=True):
            force_body = rbk.MRP2C(reader().sigma_BN) @ command
            for value, message in zip(force_body, messages, strict=True):
                message.write(messaging.SingleActuatorMsgPayload(input=float(value)), nanos)


class _FormationEvidence(sysModel.SysModel):
    def __init__(self, environment):
        super().__init__()
        self.ModelTag = 'benchmarkPrivateFormationEvidence'
        self.environment = environment

    def UpdateState(self, nanos):
        env = self.environment
        sample = FormationTruthSample(time_ns=int(nanos),
            com_states_j2000=tuple(tuple(state) for state in env._read_physical_states(nanos)),
            applied_motor_forces_body_n=tuple(tuple(float(env._stepper.actuator_force[index]) for index in indices)
                                            for indices in env._force_indices),
            external_state_writes=env._stepper.external_writes)
        env._scorer.add(sample)
        env._truth_stream.write(sample.model_dump_json()+'\n')


class NativeFormationEnvironment:
    def __init__(self, task, output_directory):
        self.task = task
        self.output_directory = evidence_path(output_directory)
        self.repository = Path(__file__).resolve().parents[1]
        self._simulation = self._stepper = self._truth_stream = self._observation = None

    def reset(self, seed):
        self.task.seeds.split_for(seed)
        self.close()
        self.episode_id = uuid.uuid4().hex
        self.episode_directory = self.output_directory/'episodes'/self.episode_id
        self.episode_directory.mkdir(parents=True, exist_ok=False)
        model, meshes, self.model_manifest = prepare_formation_model(self.repository, self.episode_directory/'model',
            spacecraft_count=self.task.spacecraft_count, force_limit_n=self.task.safety.maximum_force_n)
        crafts = self.model_manifest['crafts']
        self.context = FormationContext(task=self.task,
            spacecraft_masses_kg=tuple(self.model_manifest['bodies'][craft['body']]['mass_kg'] for craft in crafts))
        sim = self._simulation = SimulationBaseClass.SimBaseClass()
        process = sim.CreateNewProcess('formationBenchmark')
        process.addTask(sim.CreateNewTask('formationPhysics', self.task.physics_period_ns), 30)
        process.addTask(sim.CreateNewTask('formationPublisherInit', self.task.physics_period_ns))
        scene = self._scene = mujoco.MJScene.fromFile(str(model), files=meshes)
        scene.ModelTag, scene.extraEoMCall = 'benchmarkOriginalSARMFormation', True
        sim.AddModelToTask('formationPublisherInit', scene)
        ctx = self._assembly = AssemblyContext(sim, scene, config=SessionConfig(
            model_path=model, dynamics_backend='local', publish_stride=1, clock_rate_hz=50, physics_task='formationPhysics'))
        from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC
        options = FlightOptions(seed=seed, orbital_physics=OrbitalPhysics(earth_gravity=self.task.earth_gravity).model_dump(mode='json'),
            drag_enabled=self.task.drag_enabled, srp_enabled=self.task.srp_enabled,
            orbit_acceleration_sigma_m_s2=self.task.disturbance_sigma_m_s2)
        orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=options.physics)
        ctx.install([orbit])
        self._stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin,
            substeps=self.task.physics_substeps, publish_every=1, model_from_memory=True)
        ctx.ports.set_stepper(self._stepper)
        ctx.add(self._stepper, Slot.PHYSICS, component='dynamics_core')
        ctx.install([OrbitalGravityComponent(force_every=1)])
        self._environment = FlightEnvironmentComponent(options, [
            (craft['body'], craft['center_site'], mass, options.chaser_area_m2)
            for craft, mass in zip(crafts, self.context.spacecraft_masses_kg, strict=True)])
        ctx.install([self._environment])
        self._actuator = _FormationActuators(ctx.ports, crafts)
        ctx.add(self._actuator, Slot.ACTUATOR, component='formation_actuators')
        self._state_readers = [ctx.ports.site_state(craft['center_site']).addSubscriber() for craft in crafts]
        self._force_indices = [[self._stepper.actuator_names.index(name) for name in craft['force_actuators']] for craft in crafts]
        gravity = BskNavigationGravity(ctx.ports.orbital_environment())
        self._filters = [OrbitFilter(self.task.observation.position_sigma_m, self.task.observation.velocity_sigma_m_s,
                                    gravity_model=gravity) for _ in crafts]
        self._sensor_rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))
        self._scorer = FormationScorer(self.task, self.context.spacecraft_masses_kg)
        self._truth_stream = (self.episode_directory/'physics.jsonl').open('w', encoding='utf-8', newline='\n')
        self._recorder = _FormationEvidence(self)
        ctx.add(self._recorder, Slot.RECORD, component='formation_evidence')
        sim.InitializeSimulation()
        sim.disableTask('formationPublisherInit')
        states = self._initial_states(seed)
        for craft, state in zip(crafts, states, strict=True):
            center = np.asarray(self.model_manifest['bodies'][craft['body']]['center_body_m'])
            body = scene.getBody(craft['body'])
            body.setPosition((state[:3]-center).tolist())
            body.setVelocity(state[3:].tolist())
            body.setAttitude([0., 0., 0.])
            body.setAttitudeRate([0., 0., 0.])
        orbit.origin.set_state(states[0, :3], states[0, 3:])
        self._time_ns = 0
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        self._observation = self._measure(0)
        self.initial_states_j2000 = states.tolist()
        self.environment_options = options.to_dict()
        return self._observation

    def _initial_states(self, seed):
        initial = self.task.initial
        radius = EARTH_RADIUS+initial.reference_altitude_m
        phase, inc = np.deg2rad([initial.true_anomaly_deg, initial.inclination_deg])
        radial = np.array([np.cos(phase), np.sin(phase)*np.cos(inc), np.sin(phase)*np.sin(inc)])
        tangent = np.array([-np.sin(phase), np.cos(phase)*np.cos(inc), np.cos(phase)*np.sin(inc)])
        reference = np.r_[radius*radial, np.sqrt(MU/radius)*tangent]
        frame, omega = vbar_frame(reference)
        rng = np.random.default_rng(np.random.SeedSequence([seed, 1]))
        positions = np.array(initial.positions_tnr_m)
        velocities = np.array(initial.velocities_tnr_m_s)
        positions[1:] += rng.uniform(-initial.position_half_width_m, initial.position_half_width_m, positions[1:].shape)
        velocities[1:] += rng.uniform(-initial.velocity_half_width_m_s, initial.velocity_half_width_m_s, velocities[1:].shape)
        displacement = positions @ frame.T
        delta_v = velocities @ frame.T+np.cross(omega, displacement)
        return np.column_stack((displacement, delta_v))+reference

    def _read_physical_states(self, nanos):
        if any(int(reader.timeWritten()) != nanos for reader in self._state_readers):
            raise RuntimeError('stale spacecraft COM publication')
        return np.array([np.r_[sample.r_BN_N, sample.v_BN_N] for sample in (reader() for reader in self._state_readers)])

    def _measure(self, sequence):
        physical = self._read_physical_states(self._time_ns)
        sigma = np.array([self.task.observation.position_sigma_m]*3+[self.task.observation.velocity_sigma_m_s]*3)
        measurements = physical+self._sensor_rng.normal(size=physical.shape)*sigma
        dt = self.task.decision_period_ns*1e-9 if sequence else 0.
        estimates = [estimator.update(measurement, dt, command/mass, time_s=self._time_ns*1e-9)
            for estimator, measurement, command, mass in zip(self._filters, measurements, self._actuator.commands,
                                                            self.context.spacecraft_masses_kg, strict=True)]
        return FormationObservation(episode_id=self.episode_id, sequence=sequence, sample_time_ns=self._time_ns,
                                    delivery_time_ns=self._time_ns, estimates_j2000=tuple(tuple(state) for state in estimates))

    def step(self, action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError('reset must precede step')
        if self.evaluation()['outcome'] != 'incomplete':
            raise RuntimeError('formation episode has terminated')
        action.validate_for(self._observation, self.task)
        self._actuator.commands = np.asarray(action.forces_j2000_n)
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
        return dict(model=self.model_manifest, initial_states_j2000=self.initial_states_j2000,
            environment_options=self.environment_options, schedule=self._assembly.summary(),
            native_dynamics=self._stepper.telemetry(),
            native_build_manifest_sha256=sha256(self.repository/'run/native_acceleration/local_mujoco_stepper.json'),
            basilisk_mujoco_runtime_sha256=sha256(basilisk_mujoco_runtime()),
            observation_model='independent zero-latency GNSS solutions and filters; all cooperative states shared',
            actuation_model='distinct bounded native COM motors for every original SARM; passive original wheels')

    def close(self):
        if getattr(self, '_actuator', None) is not None:
            self._actuator.commands.fill(0.)
        if self._truth_stream is not None:
            self._truth_stream.close()
            self._truth_stream = None
        if self._stepper is not None:
            self._stepper.close()
            self._stepper = None
        self._simulation = self._observation = None
