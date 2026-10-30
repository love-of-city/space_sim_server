"""Shared 400 Hz capture26 runtime for diagnostics and mission stages.

Uses the authored model, both arm servos, plug retention, BSK orbital gravity,
GNSS and two gyros. Optional UE capture uses the production image receiver and
isolated stereo worker. Image-navigation runs retain strict fixed delivery;
the explicit motion-demo profile uses authoritative feedback and diagnostic
sensor delivery. Run in a fresh Basilisk Python process.
"""
from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]


def demo_telemetry(guidance):
    keys = ('acceptance_profile', 'feedback_source', 'reason', 'node', 'position_error_m', 'velocity_error_m_s',
            'position_qualified', 'velocity_qualified', 'gap_qualified', 'at_node', 'dwell_s',
            'dwell_remaining_s', 'completed_nodes', 'parking_records', 'working_reference_sha256', 'tcp_gap_m',
            'center_distance_m')
    return {key: guidance[key] for key in keys if key in guidance}


def run(args):
    import numpy as np
    from simulation.capture26_mission import MODEL_PATH, vision_path, desired_geometry, translation_reference, load_vision, source_identity
    from simulation.pose_navigation import PoseNavigation, exp, log
    from space_arm_platform.gnss_config import GNSSConfig
    from space_arm_platform.visual_navigation import calibration_id, VisualReadiness

    simulation_rate = float(getattr(args, 'simulation_rate', 1.))
    if not np.isfinite(simulation_rate) or not 0 < simulation_rate <= 100:
        raise ValueError('simulation_rate must be finite and in (0, 100]')
    output = args.output.resolve()
    mission = getattr(args, 'mission_stage', None)
    operations = None
    preparation = None
    if getattr(args, 'preparation', None):
        if mission or getattr(args, 'restart', None):
            raise ValueError('an unvalidated preparation path is restricted to a fresh diagnostic')
        from simulation.capture26_preparation import JointPreparation
        from simulation.capture26_mission import MODEL_PATH
        preparation = JointPreparation(json.loads(args.preparation.read_text('utf-8')), ROOT/MODEL_PATH)
    from simulation.capture26_mission import load_mission
    from simulation.platform_stability import PlatformStability
    from simulation.sensor_delivery import FixedStereoDelivery
    mission_config = getattr(args, 'mission_config', None) or load_mission(ROOT)
    near23 = mission_config.is_near23
    camera_config_path = vision_path(mission_config)
    vision_config = load_vision(ROOT, mission_config)
    near23_route = None
    own_motion = None
    auto_capture = None
    body_image_bridge = None
    handoff_continuity = None
    demo = mission_config.is_demo and bool(mission or getattr(args, 'approach', False))
    if demo or near23:
        # Basilisk invokes Python models on its simulation thread while the UE
        # publisher runs on another thread. The default 5 ms GIL interval can
        # stall each native callback; this changes host scheduling only.
        sys.setswitchinterval(min(sys.getswitchinterval(), .0001))
    from simulation.capture26_demo_geometry import load_geometry, working_reference, camera_depth, axial_gap
    operating_geometry = load_geometry(ROOT, mission_config) if demo else None
    delivery = FixedStereoDelivery(round(mission_config.stereo_delay_s*1e9), mission_config.vision_task_timeout_wall_s)
    operation_stability = PlatformStability(mission_config.dwell_s)
    output.mkdir(parents=True, exist_ok=bool(mission))
    emergency_request = output/'emergency-stop.request'
    if (output/'summary.json').exists():
        raise ValueError('refusing to overwrite an existing capture26 run')
    incoming = None
    epoch_ns = 0
    if getattr(args, 'handoff', None):
        from simulation.capture26_handoff import load, SCHEMA, DEMO_SCHEMA, NEAR23_SCHEMA
        incoming = load(args.handoff, ROOT)
        if incoming['schema'] != (NEAR23_SCHEMA if near23 else DEMO_SCHEMA if demo else SCHEMA) or incoming.get('mission_configuration') != mission_config.to_dict():
            raise ValueError('capture26 frozen mission configuration missing or changed across handoff')
        if mission == 'operations' and incoming['phase'] != 'NearParked':
            raise ValueError('operations require an accepted NearParked handoff')
        epoch_ns = incoming['time_ns']
        (output/'input_handoff.json').write_text(json.dumps(incoming,indent=2)+'\n',encoding='utf-8')
    elif getattr(args, 'restart', None):
        saved = json.loads(args.restart.read_text('utf-8-sig'))
        if saved.get('schema') != 'space-sim-capture26-probe-checkpoint/2':
            raise ValueError('private restart requires the complete version 2 checkpoint')
        if saved.get('mission_stage') is not None or saved.get('environment'):
            raise ValueError('formal capture26 stages require accepted handoffs, not private probe restarts')
        incoming = saved
        if incoming.get('mission_configuration') != mission_config.to_dict():
            raise ValueError('capture26 frozen mission configuration changed across private restart')
        epoch_ns = saved['physical']['time_ns']
        if epoch_ns % 100_000_000:
            raise ValueError('runtime restart requires a common 10 Hz sensor/control epoch')
        if saved['gyros'][0]['seed'] != args.seed or saved['gyros'][1]['seed'] != args.seed+1:
            raise ValueError('capture26 sensor seed changed across restart')
        args.depth = saved['control']['depth_m']
        args.pointing_error_deg = saved['control']['pointing_error_deg']
    if getattr(args, 'reference_depth', None) is not None:
        if mission or not getattr(args, 'restart', None):
            raise ValueError('diagnostic reference changes require a private probe checkpoint')
        args.depth = args.reference_depth
    if incoming and incoming.get('sensor_delivery'):
        delivery.restore(incoming['sensor_delivery'])
    if near23:
        from simulation.capture26_near23 import Near23Route
        from simulation.own_motion import OwnMotionFeedback
        near23_route = Near23Route(ROOT, vision_config, epoch_ns=epoch_ns)
        preparation = near23_route.preparation
        own_motion = OwnMotionFeedback()
        if incoming:
            preparation.restore(incoming['control']['preparation'])
            own_motion.restore(incoming['control']['own_motion'])
    elif mission and not demo:
        from simulation.capture26_mission import load_certified_preparation
        preparation = load_certified_preparation(ROOT, mission_config)
        if preparation and incoming:
            preparation.restore(incoming['control']['preparation'])
            if not preparation.complete or preparation.failed:
                raise ValueError('formal handoff requires the certified joint preparation to be complete')
    provenance = source_identity(ROOT, args.adapter)
    provenance['python_switch_interval_s'] = sys.getswitchinterval()
    provenance['basilisk_scheduler'] = 'single_step_processes' if demo or near23 else 'execute_simulation'
    provenance['numeric_thread_environment'] = {key: os.environ.get(key) for key in
        ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')}
    native_manifest = ROOT/'run/native_acceleration/local_mujoco_stepper.json'
    if native_manifest.is_file():
        provenance['native_stepper_build'] = json.loads(native_manifest.read_text('utf-8'))
    if incoming and incoming.get('source_sha256', incoming.get('model', {}).get('source_sha256')) != provenance['source_sha256']:
        raise ValueError('capture26 restart source model mismatch')
    (output/'source.json').write_text(json.dumps(provenance, indent=2)+'\n', encoding='utf-8')
    adapter = args.adapter.resolve()
    sys.path[:0] = [str(adapter/'Adapters'), str(adapter/'Unreal/BskUnrealRenderer/examples')]
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    native = load_native_grasp_module(ROOT/'model/SARM/platform')
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.utilities import macros, RigidBodyKinematics as rbk
    from simulation.assembly import AssemblyContext, SessionConfig, Slot
    from simulation.components.clock import ClockComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.components.attitude import AttitudeComponent
    from simulation.components.capture26 import Capture26Component
    from simulation.components.plug_retention import install_configured_plug_retention
    from simulation.components.gyroscope import GyroscopeComponent
    from simulation.components.render import RenderComponent
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from simulation.sarm_model_profile import load_model_profile
    from simulation.orbital_rendezvous.reference import FlightProfile
    from simulation.navigation_gravity import BskNavigationGravity
    from space_arm_platform.orbital_physics import OrbitalPhysics
    from simulation.differential_gnss import DifferentialGNSS
    flight_options = getattr(args, 'flight_options', None)

    native.MODEL_PATH = ROOT/MODEL_PATH
    native.TIME_STEP = .0025
    profile = load_model_profile(native.MODEL_PATH)
    native.JOINTS, native.ACTUATORS = profile.joints, profile.arm_actuators
    native.KP, native.KD, native.TORQUE_LIMITS = np.full(6, 500.), np.full(6, 50.), np.full(6, 100.)
    # No inverse kinematics or truth-driven capture script runs in this probe.
    operating_reference = np.array(incoming['control']['operating_reference'] if incoming else [0.]*8)
    arm_hold = HeldJointReferencePublisher(lambda t: operations.target.cached_reference(t) if operations else
                                          (operating_reference.copy(), np.zeros(8)), 8,
                                          publish_on_change=True)
    sim, scene, models, recorders = native._build_simulation(external_reference=arm_hold,
        record_history=False, dynamics_backend='local', local_substeps=1, defer_attitude_control=True)
    stepper = sim.local_stepper
    # Body/site messages at 50 Hz; bulk/joint state remains fresh at 400 Hz.
    # Estimated attitude feedback at 100 Hz does not read thinned truth poses.
    if stepper.publish_every != 8:
        raise ValueError('set SPACE_SIM_LOCAL_PUBLISH_STRIDE=8 before loading the native runtime')
    ctx = AssemblyContext(sim, scene, config=SessionConfig(model_path=native.MODEL_PATH,
        dynamics_backend='local', publish_stride=8))
    physics = flight_options.physics if flight_options is not None else OrbitalPhysics.model_validate({'earth_gravity': 'j2'})
    ephemeris_epoch = '2026 SEPTEMBER 02 00:00:00.000'
    if incoming:
        from datetime import datetime, timedelta
        ephemeris_epoch = (datetime.strptime(incoming['ephemeris_epoch_utc'], '%Y %B %d %H:%M:%S.%f')
                           + timedelta(seconds=epoch_ns*1e-9)).strftime('%Y %B %d %H:%M:%S.%f')
    orbit = OrbitComponent(epoch_utc=ephemeris_epoch, physics=physics)
    ctx.install([ClockComponent(), orbit, DynamicsCoreComponent(native.MODEL_PATH), OrbitalGravityComponent()])

    class IdleClient:
        def latest_action(self):
            return {}, True

    if getattr(args, 'operation_client', None) is not None:
        from simulation.capture26_operations import OperationsControlBridge
        operations = OperationsControlBridge(args, native.MODEL_PATH, profile, operating_reference)
        # Install IK above the reference publisher in the COMMAND segment.
        operations.install(ctx, profile)
    ctx.add(arm_hold, Slot.COMMAND, component='operating_arm_hold')
    if near23 and mission == 'near':
        from simulation.capture26_near23 import Near23CaptureClient
        auto_capture = Near23CaptureClient(output,ROOT/camera_config_path,session_id='pending',epoch_ns=epoch_ns,
            own_motion=own_motion,gyro_provider=lambda: gyros[0].observation(),com_provider=lambda: com[0])
    capture = Capture26Component(native.MODEL_PATH, auto_capture or (operations.arm_client if operations else IdleClient()),
                                 passive_hold=operations is None and auto_capture is None)
    ctx.install([capture])
    if auto_capture:
        auto_capture.component = capture
    retention = install_configured_plug_retention(ctx)
    environment = None
    if flight_options is not None:
        from simulation.components.flight_environment import FlightEnvironmentComponent
        tree = ET.parse(native.MODEL_PATH)
        effective_masses = {name: sum(stepper.body_inertial(b.get('name'))[0] for b in
            tree.find(f"./worldbody/body[@name='{name}']").iter('body')) for name in ('cubesat_bus', 'capture_target')}
        environment = FlightEnvironmentComponent(flight_options, [
            (name, None, effective_masses[name], area) for name, area in
            (('cubesat_bus', flight_options.chaser_area_m2), ('capture_target', flight_options.target_area_m2))])
        ctx.install([environment])
    # Set authored scalar initial coordinates before measuring assembled COM.
    # Native scene initialization itself occurs below; the closure keeps the
    # filter explicit prior separate from the physical perturbations.
    pointing_error = exp(np.deg2rad(getattr(args, 'pointing_error_deg', [0., 0., 0.])))
    reference_vision = near23_route.reference_vision() if near23 else vision_config
    cam = reference_vision['body_from_optical']['left']
    gnss_config = (GNSSConfig.from_dict(flight_options.differential_gnss)
                   if flight_options is not None and flight_options.differential_gnss else GNSSConfig(delay_s=args.gnss_delay))
    receiver = DifferentialGNSS(gnss_config, args.seed)
    gyros = [GyroscopeComponent(seed=args.seed, epoch_ns=epoch_ns,
                               restart=incoming['gyros'][0] if incoming else None),
             GyroscopeComponent(seed=args.seed+1, body_name='capture_target', name='target_gyro', epoch_ns=epoch_ns,
                               restart=incoming['gyros'][1] if incoming else None)]
    ctx.install(gyros)
    bulk_truth = ctx.ports.scene_state().addSubscriber()
    free_layouts = [ctx.ports.free_body_layout(n) for n in ('cubesat_bus', 'capture_target')]
    bus_com = np.asarray(stepper.body_inertial('cubesat_bus')[1:4])
    pending = []
    packet_log = score = platform_log = None
    sensor_messages = [messaging.NavAttMsg() for _ in range(2)]
    force = messaging.CmdForceInertialMsg()
    torque = messaging.CmdTorqueBodyMsg()
    ctx.ports.add_body_wrench('cubesat_bus', source='capture26_translation', force_inertial=force, torque_body=torque)
    target_torque = messaging.CmdTorqueBodyMsg()
    if near23:
        ctx.ports.add_body_wrench('capture_target',source='near23_target_pointing',torque_body=target_torque)
        from simulation.near23_navigation import Near23NavigationBridge
        body_image_bridge = Near23NavigationBridge(native.MODEL_PATH,vision_config,
            joint_covariance_rad2=np.zeros((6,6)),mount_covariance=np.zeros((6,6)),
            uncertainty_source='ideal simulation encoders and fixed nominal mount; image covariance retained')
    field = BskNavigationGravity(ctx.ports.orbital_environment(), epoch_s=epoch_ns*1e-9)
    nav = None
    approach = None
    flight = None
    if mission == 'far':
        from simulation.capture26_flight import Capture26Flight
        flight = Capture26Flight(args.scenario, handoff_distance_m=flight_options.handoff_distance_m, config=mission_config)
    masses = None
    com = None
    com_rates = np.zeros((2, 3))
    geometry_model = None
    geometry_readers = {}
    current_geometry = {}

    def authoritative_feedback(nanos):
        """Read the current integrator publication; never assign physical state."""
        from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
        if not bulk_truth.isWritten() or int(bulk_truth.timeWritten()) != int(nanos):
            raise RuntimeError('ideal feedback requires the current physical epoch')
        physical = bulk_truth()
        q, v = (np.asarray(a, float).reshape(-1) for a in (physical.qpos, physical.qvel))
        if not np.isfinite(np.r_[q, v]).all():
            raise RuntimeError('nonfinite authoritative dynamics')
        rotations = [quaternion_wxyz_to_matrix(q[qa+3:qa+7]) for qa, _, _ in free_layouts]
        p, vel, rates = [], [], []
        for i, ((qa, va, _), r) in enumerate(zip(free_layouts, rotations)):
            p.append(q[qa:qa+3]+r@com[i])
            vel.append(v[va:va+3]+r@(np.cross(v[va+3:va+6], com[i])+com_rates[i]))
            rates.append(v[va+3:va+6])
        return dict(time_ns=epoch_ns+int(nanos), p=np.array(p), v=np.array(vel), r=np.array(rotations),
            rates=np.array(rates), b=np.zeros((2, 3)), com_velocity_body=com_rates.copy(),
            tcp_body_m=np.array(current_geometry['cubesat_bus']['sites']['grip_frame']['position_body_m']))

    def refresh_geometry(now):
        nonlocal com, com_rates, current_geometry
        if current_geometry and current_geometry['cubesat_bus']['time_ns'] == now:
            return
        if any(not p.isWritten() or not v.isWritten() or int(p.timeWritten()) != now-epoch_ns
               or int(v.timeWritten()) != now-epoch_ns for p, v in geometry_readers.values()):
            raise RuntimeError('assembly geometry requires same-epoch joint encoders')
        encoders = {name: [p().state, v().state] for name, (p, v) in geometry_readers.items()}
        current_geometry = geometry_model.evaluate(now, encoders)
        com = [np.array(current_geometry[name]['com_body_m']) for name in ('cubesat_bus', 'capture_target')]
        com_rates = np.array([current_geometry[name]['com_velocity_body_m_s'] for name in ('cubesat_bus', 'capture_target')])
    result_receiver = None
    mission_id = getattr(args, 'mission_id', None) or (incoming['mission_id'] if incoming else output.name)
    import uuid
    session_id = uuid.uuid4().hex if mission or incoming else mission_id
    if incoming and (incoming['mission_id'] != mission_id or incoming['calibration_id'] != calibration_id(vision_config)):
        raise ValueError('capture26 task or calibration changed across handoff')
    from space_arm_platform.visual_navigation import NEAR23_BODY_PROFILE, FIXED_STEREO_PROFILE
    visual_profile = NEAR23_BODY_PROFILE if near23 else FIXED_STEREO_PROFILE
    gate = VisualReadiness(mission_id, session_id, calibration_id(vision_config),visual_profile=visual_profile)
    if auto_capture:
        auto_capture.servo.session_id = session_id
    gate_audit_cursor = 0
    if operations and incoming and incoming['control'].get('platform_stability'):
        operation_stability.restore(incoming['control']['platform_stability'])
    fault_start_ns = None

    def fault_active(kind, now_ns):
        nonlocal fault_start_ns
        if getattr(args, 'fault', '') != kind:
            return False
        if fault_start_ns is None and now_ns-epoch_ns >= round(getattr(args, 'fault_after', 2.)*1e9):
            ready = not getattr(args, 'fault_after_ready', False) or gate.ready
            released = not getattr(args, 'fault_after_release', False) or getattr(control, 'guidance', {}).get('released', False)
            if ready and released:
                fault_start_ns = now_ns
        duration = getattr(args, 'fault_duration', None)
        return fault_start_ns is not None and (duration is None or now_ns < fault_start_ns+round(duration*1e9))

    class Sensors(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'capture26PairedGNSS10Hz'

        def UpdateState(self, nanos):
            if nav is None:
                return
            refresh_geometry(int(nanos)+epoch_ns)
            if fault_active('gnss_loss', int(nanos)+epoch_ns):
                return
            if receiver.last_time is not None and int(nanos)+epoch_ns == round(receiver.last_time*1e9):
                return  # The preceding scene already sampled this physical epoch.
            if not bulk_truth.isWritten() or int(bulk_truth.timeWritten()) != int(nanos):
                raise RuntimeError('GNSS requires fresh physical bulk states')
            from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
            physical = bulk_truth()
            q, v = (np.asarray(a, float).reshape(-1) for a in (physical.qpos, physical.qvel))
            truth = []
            for i, ((qa, va, _), center) in enumerate(zip(free_layouts, com)):
                r = quaternion_wxyz_to_matrix(q[qa+3:qa+7])
                truth.append(np.r_[q[qa:qa+3]+r@center,
                    v[va:va+3]+r@(np.cross(v[va+3:va+6], center)+com_rates[i])])
            packet = receiver.measure(truth, (int(nanos)+epoch_ns)*1e-9)
            pending.append(packet)
            packet_log.append(dict(exposure_ns=int(nanos)+epoch_ns, sequence=packet['sequence'], due=packet['due'], truth=np.array(truth).tolist(),
                                   observations=packet['observations'].tolist(), covariance=packet['covariance'].tolist()))

    class Control(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'capture26PoseNav100HzTranslation20Hz'
            self.force_n = np.zeros(3)
            self.last_gyro = [None, None]
            self.last_gnss_ns = None
            self.accepted_visual = 0
            self.invalid_visual = 0
            self.operation_dwell_ns = None
            self.operation_acquired = False
            self.operating_reference = operating_reference
            self.sensor_delivery = delivery
            self.joint_preparation = preparation
            self.gyro_pending = [] if not incoming else list(incoming['control'].get('gyro_pending', []))
            self.last_cooperative_ns = None if not incoming else incoming['control'].get('last_cooperative_ns')
            if incoming:
                self.last_gyro = [g['last']['sample_time_ns'] for g in incoming['gyros']]

        def UpdateState(self, nanos):
            if nav is None:
                return
            now = int(nanos)+epoch_ns
            if (demo or near23) and now % 50_000_000 == 0 and emergency_request.exists():
                if auto_capture:
                    auto_capture.failure = 'operator_emergency_stop'
                if approach:
                    approach._hold('operator emergency stop')
                if flight:
                    flight.hold('operator emergency stop')
                if operations:
                    operations.fail('operator emergency stop')
            refresh_geometry(now)
            if preparation is not None and mission != 'operations' and not (auto_capture and auto_capture.active):
                samples = [geometry_readers[name] for name in preparation.names]
                reference = preparation.update(now, [p().state for p, _ in samples], [v().state for _, v in samples])
                operating_reference[:6], capture.target.position[:6] = reference[:6], reference[6:]
            nav.advance(now)
            if near23:
                qcap = np.array([geometry_readers[f'bottom_joint{i}'][0]().state for i in range(1,7)])
                body_image_bridge.remember_encoders(now,qcap)
                reading = gyros[0].observation()
                if reading:
                    own_motion.update(now,reading,com[0],epoch=(mission_id,'continuous'))
            if not (incoming and now == epoch_ns):
                nav.encoder_geometry(now, np.array(cam['position_body_m'])-com[0], com[1], com_rates)
            if getattr(sim, 'attitude_control', None) is not None:
                tensor = current_geometry['cubesat_bus']['inertia_com_body_kg_m2']
                sim.attitude_control.vehicle_config.write(messaging.VehicleConfigMsgPayload(
                    ISCPntB_B=np.array(tensor).reshape(-1).tolist()), int(nanos), self.moduleID)
                # mrpFeedback caches this tensor in Reset; update its exposed
                # tensor directly, preserving integrator and wheel states.
                sim.attitude_control.controller.ISCPntB_B = np.array(tensor).reshape(-1).tolist()
            for i, gyro in enumerate(gyros):
                observation = gyro.observation()
                if observation and observation['sample_time_ns'] != self.last_gyro[i]:
                    if fault_active(('service_gyro_loss', 'target_gyro_loss')[i], now):
                        observation.update(valid=False)
                    if fault_active('gyro_saturation', now) and i == 0:
                        observation.update(valid=False, saturated_axes=[True, False, False])
                    self.last_gyro[i] = observation['sample_time_ns']
                    if i == 1 and fault_active('communication_loss', now):
                        continue
                    delay = mission_config.gyro_delay_s+(mission_config.cooperative_delay_s if i == 1 else 0.)
                    self.gyro_pending.append(dict(body=i, observation=observation,
                        due_ns=int(observation['sample_time_ns'])+round(delay*1e9)))
            due = sorted([p for p in self.gyro_pending if p['due_ns'] <= now], key=lambda p: (p['due_ns'], p['body']))
            self.gyro_pending = [p for p in self.gyro_pending if p['due_ns'] > now]
            for packet in due:
                i, observation = packet['body'], packet['observation']
                accepted = nav.gyro(i, observation)
                if i == 1:
                    self.last_cooperative_ns = now
                packet_log.append(dict(kind='gyro', body=i, control_time_ns=now, due_ns=packet['due_ns'],
                    accepted=accepted, observation=observation, navigation_reason=nav.last_reason))
            while pending and pending[0]['due'] <= now*1e-9:
                packet = pending.pop(0)
                accepted = nav.gnss(packet['observations'], packet['covariance'], round(packet['stamp']*1e9), packet['sequence'])
                packet_log.append(dict(kind='gnss_update', control_time_ns=now,
                                       exposure_ns=round(packet['stamp']*1e9), sequence=packet['sequence'],
                                       accepted=accepted, navigation_reason=nav.last_reason))
                if accepted:
                    self.last_gnss_ns = now
            results = delivery.take_due(now)
            if fault_active('image_loss', now):
                results = [dict(r, status='rejected', reason='injected image loss') for r in results]
            if fault_active('worker_exit', now):
                (output/'vision-worker-stop').touch()
            for result in results:
                received_wall_ns = time.time_ns()
                if near23:
                    from space_arm_platform.stereo_capture_packet import capture_packet, publish_packet
                    # Publish the actual current image, including invalid frames.
                    if all(key in result for key in ('mission_id','session_id','calibration_id','source_frame_id','sim_time_ns')):
                        publish_packet(output/'capture_measurement.json',capture_packet(result,vision_config,
                            session_id=session_id,source_frame_id=result['source_frame_id'],sim_time_ns=int(result['sim_time_ns'])))
                        result = body_image_bridge.image_result(result)
                accepted = nav.visual(result, received_wall_time_ns=received_wall_ns)
                self.accepted_visual += int(accepted)
                self.invalid_visual += int(not accepted)
                pc, ac = nav.relative_covariances()
                gate.observe(result, now, accepted=accepted,
                    relative_position_covariance=pc, relative_attitude_covariance=ac,
                    received_wall_time_ns=received_wall_ns, navigation_reason=nav.last_reason)
                packet_log.append(dict(kind='visual', control_time_ns=now, accepted=accepted,
                                       receive_wall_time_ns=received_wall_ns, navigation_reason=nav.last_reason, result=result))
            if not delivery.healthy:
                gate.revoke('vision process fault: '+delivery.failure, now_ns=now, category='process_fault')
            gate.reconcile(nav.accepted, now)
            gate.tick(now)
            pc, ac = nav.relative_covariances()
            gate.check_uncertainty(pc, ac, now)
            nonlocal gate_audit_cursor
            for reset in gate.revocations[gate_audit_cursor:]:
                packet_log.append(dict(kind='visual_revocation', **reset))
            gate_audit_cursor = len(gate.revocations)
            s = authoritative_feedback(nanos) if demo else nav.state
            for i in range(2):
                sensor_messages[i].write(messaging.NavAttMsgPayload(sigma_BN=rbk.C2MRP(s['r'][i].T).tolist(),
                    omega_BN_B=(s['rates'][i]-s['b'][i]).tolist()), now, self.moduleID)
            # Start with the configured optical reference; a mission stage
            # below replaces it using the selected feedback source.
            offset, reference_velocity, _ = translation_reference(args.depth, s, com[0], com[1], reference_vision)
            p_goal = s['p'][1]+offset
            v_goal = s['v'][1]+reference_velocity
            service_rate = float(np.linalg.norm(s['rates'][0]-s['b'][0]))
            rate_stable = service_rate <= mission_config.service_angular_rate_limit_rad_s
            feedforward = np.zeros(3)
            acquisition_expired = now-epoch_ns > round((.35+gnss_config.delay_s)*1e9)
            if operations is not None:
                from space_arm_platform.visual_navigation import principal_sigma
                gnss_healthy = self.last_gnss_ns is not None and now-self.last_gnss_ns <= 350_000_000
                cooperative_healthy = self.last_cooperative_ns is not None and now-self.last_cooperative_ns <= 60_000_000
                healthy = (all(nav.gyro_health(now)) and gnss_healthy and delivery.healthy and cooperative_healthy
                           and (preparation is None or preparation.complete and not preparation.failed))
                if not demo and not cooperative_healthy and acquisition_expired:
                    operations.fail('cooperative communication lost')
                if not operations.client.connected:
                    operations.fail('operator communication lost')
                reference = ['optical_depth_m', args.depth]
                if demo:
                    offset, reference_velocity, feedforward = working_reference(s, com[0], com[1], operating_geometry)
                    p_goal, v_goal = s['p'][1]+offset, s['v'][1]+reference_velocity
                    position_error = float(np.linalg.norm(p_goal-s['p'][0]))
                    speed_error = float(np.linalg.norm(v_goal-s['v'][0]))
                    parked = incoming['control']['parking_completion']
                    enabled = position_error <= .05 and speed_error <= .01 and not operations.failed
                    operations.arm_client.enabled = enabled
                    self.guidance = dict(phase='SafeHold' if operations.failed else 'OperationsReady' if enabled else 'OperationsHold',
                        reason=operations.failed or 'working position hold; inherited completed five-second dwell',
                        acceptance_profile=mission_config.acceptance_profile, feedback_source=mission_config.feedback_source,
                        node='PlatformStandoff', position_error_m=position_error, velocity_error_m_s=speed_error,
                        position_qualified=position_error <= .05, velocity_qualified=speed_error <= .01,
                        at_node=True, released=bool(enabled), dwell_s=5., dwell_remaining_s=0.,
                        completed_nodes=[r['node'] for r in parked['records']], parking_records=parked['records'],
                        working_reference_sha256=mission_config.operating_geometry_sha256,
                        center_distance_m=float(np.linalg.norm(s['p'][0]-s['p'][1])),
                        tcp_gap_m=axial_gap(s, com[0], com[1], operating_geometry, tcp_body_m=s['tcp_body_m']),
                        optical_depth_m=camera_depth(s, com[0], com[1], vision_config),
                        platform_stable=True, platform_stable_duration_s=5., platform_failures=[],
                        gnss_healthy=gnss_healthy, process_health=delivery.status(),
                        service_angular_rate_rad_s=service_rate, service_angular_rate_stable=rate_stable,
                        position_goal=p_goal, velocity_goal=v_goal)
                else:
                    platform = operation_stability.update(now, reference, dict(
                    position=float(np.linalg.norm(p_goal-s['p'][0])) <= .006,
                    velocity=float(np.linalg.norm(s['v'][0]-v_goal)) <= .002,
                    service_angular_rate=rate_stable,
                    position_uncertainty=principal_sigma(pc) < .04,
                        health=healthy, safety=not operations.failed))
                    enabled = platform['platform_stable'] and gate.ready and not operations.failed
                    operations.arm_client.enabled = enabled
                    self.guidance = dict(phase='SafeHold' if operations.failed else 'OperationsReady' if enabled else 'OperationsHold',
                    reason=operations.failed or (gate.reason if not gate.ready else 'platform stability pending'),
                    gnss_healthy=gnss_healthy, process_health=delivery.status(),
                    released=bool(enabled), dwell_s=platform['platform_stable_duration_s'],
                    service_angular_rate_rad_s=service_rate, service_angular_rate_stable=rate_stable,
                        **platform, position_goal=p_goal, velocity_goal=v_goal)
            if flight is not None:
                if now % 50_000_000 == 0:
                    healthy = (all(nav.gyro_health(now)) and self.last_gnss_ns is not None
                               and now-self.last_gnss_ns <= 350_000_000
                               and self.last_cooperative_ns is not None and now-self.last_cooperative_ns <= 60_000_000)
                    # Preparation runs by ordinary servos during distant flight;
                    # completion is checked for handoff, while a failed path is a fault.
                    healthy = healthy and not (preparation and preparation.failed)
                    # Initial GNSS transport acquisition is not a sensor failure.
                    if demo or self.last_gnss_ns is not None or acquisition_expired:
                        self.guidance = flight.step(now, s, healthy=not cross_contacts if demo else healthy,
                            preparation_ready=preparation is None or preparation.complete)
                if hasattr(self, 'guidance'):
                    if demo:
                        self.guidance['optical_depth_m'] = camera_depth(s, com[0], com[1], vision_config)
                        self.guidance['center_distance_m'] = float(np.linalg.norm(s['p'][0]-s['p'][1]))
                    p_goal, v_goal = self.guidance['position_goal'], self.guidance['velocity_goal']
                    feedforward = self.guidance['differential_acceleration']
            if near23 and mission == 'near' and now % 50_000_000 == 0:
                from space_arm_platform.visual_navigation import principal_sigma
                from simulation.near23_navigation import relative_navigation_output
                relative_cov = np.array(relative_navigation_output(nav)['position_velocity_covariance'])
                qdot = np.array([geometry_readers[f'bottom_joint{i}'][1]().state for i in range(1,7)])
                healthy = (all(nav.gyro_health(now)) and self.last_gnss_ns is not None and now-self.last_gnss_ns<=350_000_000
                    and self.last_cooperative_ns is not None and now-self.last_cooperative_ns<=60_000_000
                    and preparation.complete and not preparation.failed and not cross_contacts)
                self.guidance = near23_route.step(now,s,service_com=com[0],target_com=com[1],q=qcap,qdot=qdot,
                    covariance=relative_cov,sigma=principal_sigma(pc),last_gnss_ns=self.last_gnss_ns,
                    last_cooperative_ns=self.last_cooperative_ns,healthy=healthy,visual_ready=gate.ready,
                    visual_stamp_ns=gate.last_exposure_ns,mass=masses[0],grasp_complete=auto_capture.complete,grasp_failure=auto_capture.failure)
                if near23_route.safe_hold and auto_capture.active and auto_capture.failure is None:
                    auto_capture.failure = near23_route.sequence.failure or 'platform_safe_hold'
                if near23_route.sequence.phase=='Grasp' and not auto_capture.active:
                    from simulation.capture_audit import CaptureAudit
                    auto_capture.audit = CaptureAudit(native.MODEL_PATH,stepper,output_path=output/'contacts_400hz.jsonl')
                    auto_capture.active = True
            if near23:
                if mission=='far':
                    from simulation.cooperative_pointing import bounded_pointing_torque
                    from simulation.near_field.orbital_guidance import vbar_frame
                    frame,omega = vbar_frame(np.r_[s['p'][1],s['v'][1]])
                    near23_route.target_torque = bounded_pointing_torque(target_rotation_n_from_body=s['r'][1],
                        reference_rotation_n_from_body=frame@near23_route.target_mount,
                        target_angular_rate_body_rad_s=s['rates'][1]-s['b'][1],reference_angular_rate_n_rad_s=omega)
                target_torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=near23_route.target_torque.tolist()),int(nanos),self.moduleID)
                if hasattr(self,'guidance'):
                    p_goal,v_goal = self.guidance['position_goal'],self.guidance['velocity_goal']
                    feedforward = self.guidance.get('differential_acceleration',feedforward)
                    self.guidance.update(acceptance_profile=mission_config.acceptance_profile,feedback_source=mission_config.feedback_source)
            if approach is not None and not near23:
                approach.service_com, approach.target_com = com
                if demo or self.last_gnss_ns is not None or acquisition_expired:
                    pc, ac = nav.relative_covariances()
                    from space_arm_platform.visual_navigation import principal_sigma
                    # Availability is an accepted and fresh image; readiness
                    # additionally requires the complete quality streak.
                    age = None if gate.last_exposure_ns is None else (now-gate.last_exposure_ns)*1e-9
                    self.guidance = approach.step(now, s, position_sigma_m=principal_sigma(pc),
                        visual_ready=gate.ready, visual_available=gate.count > 0 and age is not None and age <= .35,
                        gyro_healthy=nav.gyro_health(now),
                        gnss_healthy=self.last_gnss_ns is not None and now-self.last_gnss_ns <= 350_000_000,
                        communication_healthy=self.last_cooperative_ns is not None and now-self.last_cooperative_ns <= 60_000_000,
                        observation_age_s=.35 if age is None else age, mass_kg=masses[0],
                        **(dict(safety_reason='unplanned spacecraft contact' if cross_contacts else '') if demo else {}),
                        process_healthy=delivery.healthy and
                        (preparation is None or preparation.complete and not preparation.failed))
                if hasattr(self, 'guidance'):
                    p_goal, v_goal = self.guidance['position_goal'], self.guidance['velocity_goal']
                    feedforward = self.guidance.get('differential_acceleration', feedforward)
            if platform_log is not None and hasattr(self, 'guidance'):
                platform_log.append(dict(time_ns=now, stage=mission,
                    depth_m=approach.depth if approach else args.depth,
                    reference_com_distance_m=self.guidance.get('reference_com_distance_m'),
                    node=self.guidance.get('node', 'PlatformStandoff'),
                    position_n_m=s['p'].tolist(), velocity_n_m_s=s['v'].tolist(),
                    rotation_n_from_body=s['r'].tolist(), rates_body_rad_s=(s['rates']-s['b']).tolist(),
                    com_body_m=[p.tolist() for p in com], com_velocity_body_m_s=com_rates.tolist(),
                    tcp_body_m=s.get('tcp_body_m', np.zeros(3)).tolist(),
                    reference_position_n_m=p_goal.tolist(), reference_velocity_n_m_s=v_goal.tolist(),
                    acceptance_profile=mission_config.acceptance_profile, feedback_source=mission_config.feedback_source,
                    at_node=self.guidance.get('at_node', False),
                    position_covariance_m2=pc.tolist(), attitude_covariance_rad2=ac.tolist(),
                    gyro_health=nav.gyro_health(now), last_gnss_ns=self.last_gnss_ns,
                    last_cooperative_ns=self.last_cooperative_ns, process_healthy=delivery.healthy,
                    safe_hold=bool(approach.safe_hold if approach else operations.failed),
                    visual_ready=gate.ready, visual_count=gate.count, released=self.guidance.get('released', False),
                    reported_platform_stable=self.guidance.get('platform_stable', False)))
            # The translation controller shares the specified 20 Hz guidance
            # grid. Forces are held between commands; gyro/nav/attitude feedback
            # above continues at 100 Hz. Replay records actual command changes,
            # rather than 100 redundant commanded-acceleration events per second.
            if now % 50_000_000:
                return
            a = field.acceleration(s['p'][1], now*1e-9)-field.acceleration(s['p'][0], now*1e-9)+feedforward
            kp, kd = (.0004, .04) if flight is not None else (.04, .4)
            a += kp*(p_goal-s['p'][0])+kd*(v_goal-s['v'][0])
            if self.last_gnss_ns is None and mission and not demo:
                a[:] = 0.
            requested = a*masses[0]
            self.force_n = requested*min(1., 25./max(1e-12, np.linalg.norm(requested)))
            force.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=self.force_n.tolist()), now, self.moduleID)
            # The force acts at bus COM; cancel its moment about assembly COM
            # using the estimated attitude and the fixed physical lever arm.
            moment = np.cross(com[0]-bus_com, s['r'][0].T@self.force_n)
            torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=moment.tolist()), now, self.moduleID)
            nav.command(np.array([self.force_n/masses[0], np.zeros(3)]))

    control = Control()
    # Consume new physical sensor epochs after SENSOR. The attitude chain on
    # the next physics tick sees this posterior; no held gyro sample is relabelled
    # and the normal ordered path does not need to rewind its own prediction.
    ctx.add(control, Slot.RECORD, every=4, component='pose_navigation', reads_kinematics=False)
    ctx.add(Sensors(), Slot.SENSOR, every=40, phase=((-epoch_ns) % 100_000_000)//2_500_000,
            component='paired_gnss', reads_kinematics=False)
    reference_rotation = None

    # A nominal target attitude is an initial prior. It is not a truth sensor.
    r0, v0, _ = (flight.profile if flight else FlightProfile()).target(0.)
    target_rotation = exp([0., 0., np.pi/2])
    if demo and getattr(args, 'approach', False) and not incoming:
        # A standalone diagnostic starts behind the target on COM V-bar. Formal
        # near flight always restores the actual accepted distant checkpoint.
        from simulation.near_field.orbital_guidance import vbar_frame
        frame, _ = vbar_frame(np.r_[r0, v0])
        target_rotation = np.column_stack((np.cross(-frame[:, 0], frame[:, 1]), -frame[:, 0], frame[:, 1]))
    if flight:
        end_r, end_v, _ = flight.profile.target(flight.profile.duration_s)
        y = -end_v/np.linalg.norm(end_v)
        z = np.cross(end_r, end_v)
        z /= np.linalg.norm(z)
        target_rotation = np.column_stack((np.cross(y, z), y, z))
    _, reference_rotation = desired_geometry(args.depth, target_rotation, np.zeros(3), np.zeros(3), reference_vision)
    if near23:
        from simulation.near_field.orbital_guidance import vbar_frame
        frame,_ = vbar_frame(np.r_[r0,v0])
        reference_rotation = frame@near23_route.service_mount
        target_rotation = frame@near23_route.target_mount
    reference_rotation = reference_rotation@pointing_error
    if incoming:
        reference_rotation = np.array(incoming['navigation']['state']['r'][0])
    attitude = AttitudeComponent(native.MODEL_PATH, estimated_navigation=sensor_messages[0],
        initial_attitude_prior=rbk.C2MRP(reference_rotation.T), control_rate_hz=mission_config.attitude_hz,
        control_mode=mission_config.attitude_control_mode,
        reference_provider=(lambda t: near23_route.attitude_reference(nav.state) if nav is not None else
            (reference_rotation,np.zeros(3),np.zeros(3))) if near23 else None)
    ctx.install([attitude])
    bridge = None
    vision = None
    if args.ue:
        from space_arm_platform.vision_channel import VisionResultReceiver
        import subprocess
        if not args.catalog or not args.catalog.is_file():
            raise ValueError('UE probe requires a prepared capture26 catalog')
        render_profile = json.loads((ROOT/'config/scene_render_profiles.json').read_text())['sarm-dual-capture26']
        render = RenderComponent(model_path=native.MODEL_PATH, catalog=args.catalog.resolve(),
            host='127.0.0.1', port=args.render_port, render_rate_hz=args.render_rate, capture_rate_hz=10,
            capture_state_provider=operations.client.capture_state if operations else lambda: {},
            visual_attachments=([dict(parent_id='teleop/capture_target',
                geometry_file='config/visual_attachments/capture26_near23/geometry_metres.json',
                sha256=hashlib.sha256((ROOT/'config/visual_attachments/capture26_near23/geometry_metres.json').read_bytes()).hexdigest())]
                if near23 else render_profile['visual_attachments']),
            vision_capture=dict(enabled=mission != 'far', config_file=camera_config_path.as_posix(),
                diagnostic_only=demo,preserve_original_lighting=near23))
        ctx.extra['template_id'] = 'sarm-dual-capture26'
        ctx.install([render])
        bridge = render.bridge
        if demo:
            from bsk_render_adapter.descriptors import RelativeView
            bridge._settings.relative_view = RelativeView(
                primary_object_id='teleop/cubesat_bus', secondary_object_id='teleop/capture_target',
                primary_center_body_m=com[0].tolist() if com else (0., 0., 0.),
                secondary_center_body_m=com[1].tolist() if com else (0., 0., 0.),
                primary_label='Capture26 service', secondary_label='Passive target')
            bridge._settings.orbit_lines = bridge._settings.trajectory_history = False
            # Main-viewport evidence must show both spacecraft. Camera streams
            # remain available in the workbench's camera/diagnostic panels.
            for camera in bridge._cameras:
                camera.picture_in_picture = False
        bridge.session_id = session_id
        bridge.epoch_offset_ns = epoch_ns
        bridge.vision_delivery_delay_ns = round(mission_config.stereo_delay_s*1e9)
        bridge.vision_exposure_after_ns = delivery.last_exposure
        bridge.frame_id_from_simulation_epoch = True
        result_receiver = VisionResultReceiver(mission_id, session_id, calibration_id(vision_config))
        external_vision = getattr(args, 'external_vision', False)
        log_file = None if external_vision else (output/'vision-host.log').open('wb')
        command = [sys.executable, str(ROOT/'tools/run_capture26_vision.py'),
            '--config', str(ROOT/camera_config_path), '--mission', mission_id, '--session', session_id,
            '--result-port', str(result_receiver.port), '--capture-port', str(args.capture_port),
            '--output', str(output)]
        if getattr(args, 'save_all_images', False):
            command.append('--save-all-images')
        vision_process = None if external_vision else subprocess.Popen(command, cwd=ROOT, stdout=log_file, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        if external_vision:
            route = dict(schema='space-sim-vision-route/1', mission_id=mission_id, session_id=session_id,
                         calibration_id=calibration_id(vision_config), result_port=result_receiver.port,
                         config_file=camera_config_path.as_posix())
            temporary = output/'vision-route.tmp'
            temporary.write_text(json.dumps(route), encoding='utf-8')
            temporary.replace(output/'vision-route.json')

        class VisionHost:
            def status(self):
                try:
                    return json.loads((output/'vision-status.json').read_text('utf-8'))
                except (OSError, ValueError):
                    return dict(last_error='vision host has not published status')

            def close(self):
                if vision_process is None:
                    return
                (output/'vision-stop').touch()
                try:
                    vision_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    import psutil
                    with contextlib.suppress(psutil.NoSuchProcess):
                        parent = psutil.Process(vision_process.pid)
                        for child in reversed(parent.children(recursive=True)):
                            with contextlib.suppress(psutil.NoSuchProcess):
                                child.kill()
                        parent.kill()
                    vision_process.wait(timeout=5)
                log_file.close()

        vision = VisionHost()
        deadline = time.monotonic()+20
        while not (output/'vision-ready.json').exists():
            if (vision_process is not None and vision_process.poll() is not None) or time.monotonic() >= deadline:
                if demo:
                    delivery.fail('diagnostic image ingress failed to warm up', epoch_ns)
                    break
                vision.close()
                result_receiver.close()
                raise RuntimeError('image ingress failed to warm up; see vision-host.log')
            time.sleep(.05)

    cross_contacts = []
    geom_roots = {}
    geom_bodies = {}
    for body in ET.parse(native.MODEL_PATH).findall('./worldbody/body'):
        for geom in body.iter('geom'):
            if geom.get('name'):
                geom_roots[geom.get('name')] = body.get('name')
        for child in body.iter('body'):
            for geom in child.findall('geom'):
                if geom.get('name'):
                    geom_bodies[geom.get('name')] = child.get('name')

    class ContactMonitor(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'capture26NoContactMonitor400Hz'

        def UpdateState(self, nanos):
            if not stepper.state_initialized:
                return
            for row in stepper.contact_detail():
                roots = {geom_roots.get(stepper._name(5, int(i))) for i in row[:2]}
                names = [stepper._name(5,int(i)) for i in row[:2]]
                planned = (auto_capture is not None and auto_capture.active and roots=={'cubesat_bus','capture_target'}
                    and any(geom_bodies.get(name,'').startswith(('jaw_','axial_carriage','cam','gripper')) for name in names))
                if planned:
                    continue
                if row[2] < -1e-5 and (roots == {'cubesat_bus', 'capture_target'} or roots == {'cubesat_bus'}):
                    if len(cross_contacts) < 100:
                        cross_contacts.append(dict(time_ns=epoch_ns+int(nanos), distance_m=float(row[2]),
                            kind='self_contact' if len(roots) == 1 else 'spacecraft_contact',
                            geoms=[stepper._name(5, int(i)) for i in row[:2]]))
                    if preparation:
                        preparation.failed = 'unplanned contact during joint preparation'
                    if approach:
                        approach._hold('unplanned spacecraft contact')
                    if flight:
                        flight.hold('unplanned spacecraft contact')
                    if operations:
                        operations.fail('unplanned spacecraft contact')

    # This monitor reads native coordinates directly; it never writes them.
    ctx.add(ContactMonitor(), Slot.SENSOR, component='no_contact_monitor', reads_kinematics=False)

    sim.InitializeSimulation()
    from simulation.capture_arm_target import INITIAL_JOINTS
    for body, joint, value in INITIAL_JOINTS:
        scene.getBody(body).getScalarJoint(joint).setPosition(value)
    properties = stepper.initial_assembly_properties(native.MODEL_PATH, ('cubesat_bus', 'capture_target'))
    com = [np.array(properties[name]['com_body_m']) for name in ('cubesat_bus', 'capture_target')]
    masses = [properties[name]['mass_kg'] for name in ('cubesat_bus', 'capture_target')]
    from simulation.assembly_geometry import AssemblyGeometry
    geometry_model = AssemblyGeometry(native.MODEL_PATH, stepper.body_inertial)
    geometry_readers = {name: (ctx.ports.joint_state(name).addSubscriber(), ctx.ports.joint_rate(name).addSubscriber())
                        for name in geometry_model.joints}

    offset, _ = desired_geometry(args.depth, target_rotation, com[0], com[1], reference_vision,
                                 service_rotation=reference_rotation)
    nominal_positions = np.array([r0+offset, r0])
    nominal_velocities = np.array([v0, v0])
    if flight:
        nominal_positions[0], nominal_velocities[0], _ = flight.profile.chaser(0.)
    rotations = [reference_rotation, target_rotation]
    # Perturb physical state independently; navigation starts from explicit
    # nominal priors with matching uncertainty rather than reading truth.
    rng = np.random.default_rng(args.seed)
    assigned_initial = []
    assigned_velocities = []
    position_sigma = flight_options.initial_position_sigma_m if flight else .002
    velocity_sigma = flight_options.initial_velocity_sigma_m_s if flight else .002
    for i, name in enumerate(('cubesat_bus', 'capture_target')):
        body = scene.getBody(name)
        rotation_true = exp(rng.normal(0, np.deg2rad(.05), 3))@rotations[i]
        position_true = nominal_positions[i]-rotation_true@com[i]+rng.normal(0, position_sigma, 3)
        assigned_initial.append((position_true, rotation_true))
        body.setPosition(position_true.tolist())
        assigned_velocities.append(nominal_velocities[i]+rng.normal(0, velocity_sigma, 3))
        body.setVelocity(assigned_velocities[-1].tolist())
        body.setAttitude(rbk.C2MRP(rotation_true.T).tolist())
        body.setAttitudeRate([0., 0., 0.])
    # Free tools retain their actual relative authored positions/orientations
    # when placing the service craft in orbit; their states remain independent.
    # setPosition setters do not publish until the first native tick. Use the
    # exact already assigned initialization, never the stale XML body message.
    bus_position, bus_rot = assigned_initial[0]
    root = ET.parse(native.MODEL_PATH).getroot()
    for body in root.findall('./worldbody/body'):
        name = body.get('name')
        if name in ('cubesat_bus', 'capture_target'):
            continue
        q = np.fromstring(body.get('quat', '1 0 0 0'), sep=' ')
        from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
        item = scene.getBody(name)
        item.setPosition((bus_position+bus_rot@np.fromstring(body.get('pos', '0 0 0'), sep=' ')).tolist())
        item.setVelocity(assigned_velocities[0].tolist())
        item.setAttitude(rbk.C2MRP((bus_rot@quaternion_wxyz_to_matrix(q)).T).tolist())
        item.setAttitudeRate([0., 0., 0.])
    orbit.origin.set_state(r0, v0)
    if incoming:
        # Restoration occurs once, before any propagation. This includes free
        # tools, both complete arms and all wheel coordinates, not just two roots.
        stepper.restore_physical_snapshot(incoming['physical'])
        receiver.restore(incoming['gnss_receiver'])
        pending.extend(incoming['gnss_pending'])
        if retention:
            retention.restore(incoming['retention'])
        capture.target.position[:] = incoming['control']['capture_reference']
        if environment:
            for module, saved in zip(environment.models, incoming['environment'], strict=True):
                module.restore(saved, epoch_ns=epoch_ns)
    prior = np.diag(([max(position_sigma, .002)**2]*3+[max(velocity_sigma, .002)**2]*3
                     +[np.deg2rad(.05)**2]*3+[1e-5**2]*3)*2)
    nav = PoseNavigation(nominal_positions, nominal_velocities, rotations, prior, gravity=field,
        camera_rotation=cam['R_body_from_optical'], camera_position=cam['position_body_m'],
        service_com=com[0], target_com=com[1],
        time_ns=epoch_ns,
        vision_identity=(tuple(incoming['navigation']['configuration']['vision_identity']) if incoming else
                         (mission_id, session_id, calibration_id(vision_config))),visual_profile=visual_profile)
    if incoming:
        nav.restore(incoming['navigation'])
        inherited_images = [packet for _, packet in delivery.staged]+list(delivery.completed.values())
        nav.bind_vision_session(mission_id, session_id, calibration_id(vision_config),
                               pending_visual_results=inherited_images)
        control.last_gnss_ns = incoming['control']['last_gnss_ns']
        control.force_n = np.array(incoming['control']['force_n'])
    if near23 and mission == 'near':
        approach = near23_route
    elif getattr(args, 'approach', False) or mission == 'near':
        from simulation.capture26_approach import Capture26Approach, Capture26DemoApproach
        entry_depth = args.depth
        if incoming:
            zero_offset, _ = desired_geometry(0., nav.state['r'][1], com[0], com[1], vision_config,
                                              service_rotation=nav.state['r'][0])
            approach_axis = -nav.state['r'][1][:, 1]
            entry_depth = float((zero_offset-(nav.state['p'][0]-nav.state['p'][1]))@approach_axis)
        approach = (Capture26DemoApproach(com[0], com[1], vision_config, config=mission_config, geometry=operating_geometry)
                    if demo else Capture26Approach(com[0], com[1], vision_config, initial_depth_m=entry_depth,
                                                   config=mission_config))
    for i in range(2):
        sensor_messages[i].write(messaging.NavAttMsgPayload(sigma_BN=rbk.C2MRP(nav.state['r'][i].T).tolist(),
            omega_BN_B=(nav.state['rates'][i]-nav.state['b'][i]).tolist()))
    channel = None
    if getattr(args, 'state_port', 0):
        import socket
        channel = socket.create_connection(('127.0.0.1', args.state_port), timeout=2)
        channel.settimeout(.02)
    fingerprint = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    from simulation.capture26_evidence import EvidenceRows
    packet_log = EvidenceRows(output/'measurements.jsonl')
    if approach is not None or operations is not None:
        platform_log = EvidenceRows(output/'platform-estimates.jsonl', retain=1)
    # Enough for the complete 120 s distant hold and the near dwell. Every
    # earlier record is retained on disk, including when a later stage fails.
    score = EvidenceRows(output/'score.jsonl', retain=2601)
    maximum_relative_error = 0.
    started, compute, windows = time.perf_counter(), 0., []
    gravity_cache_error = None
    gravity_acceleration_error = None
    profiler = None
    if args.profile:
        import cProfile
        profiler = cProfile.Profile()
        profiler.enable()
    try:
        first_release_elapsed = None
        braking_stable_start_ns = None
        paced_wall_target = time.perf_counter()
        stage_snapshots = set()
        for tick in range(0, round(args.duration*20)+1):
            elapsed = tick*.05
            if result_receiver is not None and mission != 'far':
                next_ns = epoch_ns+round(elapsed*1e9)
                if fault_active('worker_exit', next_ns):
                    (output/'vision-worker-stop').touch()
                    delivery.fail('injected vision process exit', next_ns)
                def poll_images():
                    valid = result_receiver.poll()
                    invalid = result_receiver.drain_invalid()
                    for event in invalid:
                        event.setdefault('sim_time_ns', str(next_ns))
                        event.setdefault('source_frame_id', 'invalid-transport')
                    return valid+invalid
                prepare_images = delivery.prepare_diagnostic if demo else delivery.prepare
                prepare_images(next_ns, poll_images,
                    alive=lambda: vision_process is None or vision_process.poll() is None)
            before = time.perf_counter()
            sim.ConfigureStopTime(macros.sec2nano(elapsed))
            if demo or near23:
                # This runner schedules every sensor/control event as a BSK
                # model. The public single-step API executes the same process
                # priorities on this thread, avoiding Python GIL handoffs on
                # every 400 Hz callback. No physical tick is skipped or merged.
                if any(sim.activeEvents()) or sim.TotalSim.getThreadCount() != 1:
                    raise RuntimeError('capture26 direct scheduling requires one BSK thread and model-scheduled events')
                while sim.TotalSim.NextTaskTime <= sim.StopTime:
                    if sim.terminate:
                        raise RuntimeError('authoritative dynamics requested termination')
                    sim.TotalSim.SingleStepProcesses(-1)
            else:
                sim.ExecuteSimulation()
            compute += time.perf_counter()-before
            now = epoch_ns+round(elapsed*1e9)
            if tick == 0:
                if incoming:
                    # Record what the receiving native scene actually publishes,
                    # after its first tick, for an independent continuity audit.
                    restored = dict(physical=stepper.physical_snapshot(epoch_ns=epoch_ns),
                        navigation=nav.snapshot(), gyros=[g.sensor.snapshot() for g in gyros],
                        gnss_receiver=receiver.snapshot(), gnss_pending=pending,
                        retention=retention.snapshot() if retention else None,
                        control=dict(operating_reference=operating_reference.tolist(),
                                     capture_reference=capture.target.position.tolist(),
                                     gyro_pending=control.gyro_pending,
                                     last_cooperative_ns=control.last_cooperative_ns,
                                     platform_stability=operation_stability.snapshot() if operations else approach.stability.snapshot() if approach and not near23 else None,
                                     parking_completion=incoming['control'].get('parking_completion') if operations else None,
                                     preparation=preparation.snapshot() if preparation else None,
                                     own_motion=own_motion.snapshot() if own_motion else None),
                        mission_configuration=mission_config.to_dict(), sensor_delivery=delivery.snapshot(),
                        environment=[m.snapshot(epoch_ns=epoch_ns) for m in environment.models] if environment else [])
                    (output/'restored-state.json').write_text(json.dumps(restored,
                        default=lambda a: a.tolist())+'\n', encoding='utf-8')
                    if near23:
                        from simulation.capture26_near23 import verify_handoff
                        handoff_continuity = verify_handoff(args.handoff,output,near23_route,auto_capture)
                # Independent native-field check of the bounded EKF local
                # linearization; the truth dynamics use the full native field.
                native_gradient = field.gradient(nav.state['p'][0], epoch_ns*1e-9)
                native_acceleration = field.acceleration(nav.state['p'][0], epoch_ns*1e-9)
                gravity_acceleration_error = max(float(np.linalg.norm(
                    native_acceleration+native_gradient@(sign*.25*axis)-field.acceleration(
                        nav.state['p'][0]+sign*.25*axis, epoch_ns*1e-9)))
                    for axis in np.eye(3) for sign in (-1, 1))
                gravity_cache_error = max(float(np.linalg.norm(native_gradient-field.gradient(
                    nav.state['p'][0]+sign*.25*axis, epoch_ns*1e-9)))
                    for axis in np.eye(3) for sign in (-1, 1))
                if gravity_cache_error > 1e-12:
                    raise RuntimeError('local navigation Jacobian reuse exceeds native field error bound')
                if gravity_acceleration_error > 1e-11:
                    raise RuntimeError('local navigation acceleration reuse exceeds native field error bound')
            if bridge and bridge.last_vision_exposure is not None:
                exposure, frame_id = bridge.last_vision_exposure
                if delivery.last_exposure is None or exposure > delivery.last_exposure:
                    delivery.expect(exposure, frame_id)
            # Independent scoring reads the fresh bulk physical coordinates.
            # Body messages at 50 Hz need not coincide with this 20 Hz log.
            if not bulk_truth.isWritten() or int(bulk_truth.timeWritten()) != round(elapsed*1e9):
                raise RuntimeError('independent score requires fresh physical bulk states')
            state_truth = bulk_truth()
            q_truth = np.asarray(state_truth.qpos, float).reshape(-1)
            v_truth = np.asarray(state_truth.qvel, float).reshape(-1)
            from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
            rotations_truth = [quaternion_wxyz_to_matrix(q_truth[qa+3:qa+7]) for qa, _, _ in free_layouts]
            assembly_truth = stepper.assembly_truth(native.MODEL_PATH, ('cubesat_bus', 'capture_target'))
            actual = [assembly_truth[name]['position_n_m'] for name in ('cubesat_bus', 'capture_target')]
            truth_com_body = [r.T@(p-q_truth[qa:qa+3]) for (qa, _, _), r, p in zip(free_layouts, rotations_truth, actual)]
            geometry_error = max(float(np.linalg.norm(a-b)) for a, b in zip(truth_com_body, com))
            if geometry_error > 1e-7:
                raise RuntimeError(f'encoder geometry disagrees with native assembly COM: {geometry_error}')
            truth_com_rates = [r.T@(assembly_truth[name]['velocity_n_m_s']-v_truth[va:va+3])
                -np.cross(v_truth[va+3:va+6], centre)
                for name, (_, va, _), r, centre in zip(('cubesat_bus', 'capture_target'), free_layouts, rotations_truth, truth_com_body)]
            geometry_rate_error = float(np.max(np.abs(np.array(truth_com_rates)-com_rates)))
            inertia_error = max(float(np.max(np.abs(assembly_truth[name]['inertia_com_body_kg_m2']
                -current_geometry[name]['inertia_com_body_kg_m2']))) for name in ('cubesat_bus', 'capture_target'))
            if geometry_rate_error > 1e-6 or inertia_error > 1e-6:
                raise RuntimeError(f'encoder geometry disagrees with native assembly velocity/inertia: {geometry_rate_error}, {inertia_error}')
            estimated = nav.state['p'][1]-nav.state['p'][0]
            actual_relative = actual[1]-actual[0]
            maximum_relative_error = max(maximum_relative_error, float(np.linalg.norm(estimated-actual_relative)))
            score.append(dict(time_s=now*1e-9, estimated_relative_position_m=estimated.tolist(),
                truth_relative_position_m=actual_relative.tolist(),
                relative_error_m=(estimated-actual_relative).tolist(),
                attitude_error_rad=[log(nav.state['r'][i]@rotations_truth[i].T).tolist() for i in range(2)],
                truth_angular_velocity_body_rad_s=[v_truth[va+3:va+6].tolist() for _, va, _ in free_layouts],
                service_angular_rate_rad_s=float(np.linalg.norm(nav.state['rates'][0]-nav.state['b'][0])),
                truth_com_position_n_m=[p.tolist() for p in actual],
                truth_com_velocity_n_m_s=[assembly_truth[name]['velocity_n_m_s'].tolist() for name in ('cubesat_bus', 'capture_target')],
                truth_com_body_m=[p.tolist() for p in truth_com_body],
                truth_tcp_body_m=current_geometry['cubesat_bus']['sites']['grip_frame']['position_body_m'],
                encoder_geometry=current_geometry, encoder_native_com_error_m=geometry_error,
                encoder_native_com_velocity_error_m_s=geometry_rate_error,
                encoder_native_inertia_error_kg_m2=inertia_error,
                preparation=preparation.status() if preparation else None,
                capture=(dict(phase=auto_capture.servo.sequence.phase,completed=auto_capture.complete,
                    force_hold_s=auto_capture.hold_s,feedback=auto_capture.audit.feedback if auto_capture.audit else {})
                    if auto_capture else None),
                truth_rotation_n_from_body=[r.tolist() for r in rotations_truth],
                estimated_rotation_n_from_body=nav.state['r'].tolist(),
                gyro_health=nav.gyro_health(now), visual=gate.status(now, time.time_ns() if args.ue else None), force_n=control.force_n.tolist(),
                gnss_healthy=control.last_gnss_ns is not None and now-control.last_gnss_ns <= 350_000_000,
                fault_start_ns=fault_start_ns,
                guidance=getattr(control, 'guidance', None)))
            if bridge and tick % 10 == 0:
                from bsk_render_adapter.near_field import publish_capture26_status
                guidance = getattr(control, 'guidance', {})
                publish_capture26_status(bridge, dict(phase=guidance.get('phase', 'NavigationAcquire') if mission else 'SENSOR PROBE / NOT MISSION ACCEPTANCE',
                    visual=gate.status(now, time.time_ns()), gyro_health=nav.gyro_health(now), optical_depth_m=guidance.get('optical_depth_m', args.depth),
                    platform_stable_duration_s=getattr(control, 'guidance', {}).get('platform_stable_duration_s', 0.),
                    platform_stable=getattr(control, 'guidance', {}).get('platform_stable', False),
                    service_angular_rate_rad_s=float(np.linalg.norm(nav.state['rates'][0]-nav.state['b'][0])),
                    service_angular_rate_stable=float(np.linalg.norm(nav.state['rates'][0]-nav.state['b'][0])) <= mission_config.service_angular_rate_limit_rad_s,
                    released=getattr(control, 'guidance', {}).get('released', False), process_health=delivery.status(),
                    **demo_telemetry(guidance),
                    processing_real_time_factor=elapsed/max(compute, 1e-9)))
                if demo:
                    key = (('Entry300km' if args.scenario == 'orbit-raise' else 'Entry7km') if flight else
                        'OperationsEntry' if operations else 'NearEntry') if tick == 0 else (
                        ('FarHandoffComplete' if flight else 'OperationsHold' if operations else 'WorkingPositionHold') if guidance.get('released')
                        else guidance.get('node', 'Flight'))
                    if (key not in stage_snapshots and (guidance.get('dwell_s', 0.) >= 2. or guidance.get('released')
                            or tick == 0)):
                        bridge.publish_event('capture26_stage_snapshot', dict(filename=key+'.png',
                            message='Physical state at '+key, sim_time_ns=str(now)))
                        stage_snapshots.add(key)
            if operations and bridge and tick % 2 == 0:
                operations.publish(stepper, nav, gyros, capture, gate, now, bridge,
                    getattr(control, 'guidance', dict(phase='OperationsHold', reason='sensor acquisition')),
                    elapsed/max(compute, 1e-9))
            if channel and tick % 2 == 0:
                import struct
                from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
                bodies = []
                for i, ((qa, va, _), rotation_true) in enumerate(zip(free_layouts, rotations_truth)):
                    bodies.append(dict(body_id='rendezvous/'+('chaser' if i == 0 else 'target'),
                        position_m=q_truth[qa:qa+3].tolist(), velocity_m_s=v_truth[va:va+3].tolist(),
                        attitude_wxyz=matrix_to_quaternion_wxyz(rotation_true).tolist(),
                        angular_velocity_rad_s=v_truth[va+3:va+6].tolist()))
                guidance = getattr(control, 'guidance', {})
                metadata = dict(run_id=output.name, phase=guidance.get('phase', 'NavigationAcquire'),
                    model='capture26', center_distance_m=float(np.linalg.norm(actual_relative)),
                    navigation_source='GNSS + UE stereo pose + dual gyro ESKF',
                    navigation_valid=all(nav.gyro_health(now)), visual_ready=gate.ready,
                    sensor_health=dict(gyros=nav.gyro_health(now), gnss=score[-1]['gnss_healthy']),
                    visual=score[-1]['visual'], optical_depth_m=guidance.get('optical_depth_m'),
                    platform_stable_duration_s=guidance.get('platform_stable_duration_s', 0.),
                    platform_stable=guidance.get('platform_stable', False), released=guidance.get('released', False),
                    service_angular_rate_rad_s=float(np.linalg.norm(nav.state['rates'][0]-nav.state['b'][0])),
                    service_angular_rate_stable=float(np.linalg.norm(nav.state['rates'][0]-nav.state['b'][0])) <= mission_config.service_angular_rate_limit_rad_s,
                    process_health=delivery.status(), cooperative_healthy=control.last_cooperative_ns is not None and now-control.last_cooperative_ns <= 60_000_000,
                    processing_real_time_factor=elapsed/max(compute, 1e-9),
                    mission_options=flight_options.to_dict() if flight_options else {})
                if demo:
                    metadata.update(demo_telemetry(guidance),
                        navigation_source='authoritative simulation state feedback; sensors diagnostic',
                        navigation_valid=True)
                if near23:
                    metadata.update(demo_telemetry(guidance),near_guidance=guidance.get('vbar'),
                        acceptance_profile=mission_config.acceptance_profile,
                        capture_state=auto_capture.servo.sequence.phase if auto_capture and auto_capture.active else 'waiting_platform',
                        capture_force_hold_s=auto_capture.hold_s if auto_capture else 0.,
                        capture_feedback=auto_capture.audit.feedback if auto_capture and auto_capture.audit else None)
                data = json.dumps(dict(protocol='space-sim-state/1', type='state', sim_time_s=now*1e-9,
                                       bodies=bodies, metadata=metadata)).encode()
                try:
                    channel.sendall(struct.pack('!I', len(data))+data)
                except OSError:
                    channel.close()
                    channel = None
                    if approach:
                        approach._hold('platform telemetry communication lost')
                    if flight:
                        flight.hold('platform telemetry communication lost')
            windows.append(dict(sim_time_s=elapsed, wall_s=time.perf_counter()-started, compute_s=compute))
            if args.realtime:
                guidance = getattr(control, 'guidance', {})
                pacing_rate = min(1., simulation_rate) if demo and (guidance.get('at_node') or operations) else simulation_rate
                if tick:
                    # Slow initialization/transit earns no credit for playing a
                    # later dwell faster than real time. Anchor each interval
                    # before integrating it, keeping all clocks in simulation ns.
                    paced_wall_target = max(paced_wall_target, before)+.05/pacing_rate
                delay = paced_wall_target-time.perf_counter()
                while delay > 0:
                    time.sleep(min(delay, .05))
                    delay = paced_wall_target-time.perf_counter()
                windows[-1].update(pacing_rate=pacing_rate, interval_wall_s=time.perf_counter()-before)
            if mission in ('near', 'far') and tick % 2 == 0:
                guidance = getattr(control, 'guidance', {})
                if guidance.get('released') and first_release_elapsed is None:
                    first_release_elapsed = elapsed
                continuation = getattr(args, 'post_release_duration', 0.)
                finished = first_release_elapsed is not None and elapsed-first_release_elapsed >= continuation
                safe_stop = guidance.get('phase') == 'SafeHold' and elapsed > 5. and not continuation
                if safe_stop and (demo or near23):
                    # Keep integrating the commanded brake. Stopping the native
                    # process at the first SafeHold publication would only
                    # freeze the view while the spacecraft was still closing.
                    velocity_error = np.linalg.norm(assembly_truth['cubesat_bus']['velocity_n_m_s']
                        -np.asarray(guidance['velocity_goal']))
                    if velocity_error > mission_config.velocity_tolerance_m_s:
                        braking_stable_start_ns = None
                    elif braking_stable_start_ns is None:
                        braking_stable_start_ns = now
                    safe_stop = (braking_stable_start_ns is not None
                                 and now-braking_stable_start_ns >= 5_000_000_000)
                if finished or safe_stop:
                    break
        simulated_s = elapsed
        if result_receiver is not None and delivery.pending and not demo:
            # Complete old-session computation before closing its image host.
            # Results keep their future simulation delivery epochs in handoff.
            delivery.prepare(delivery.pending[-1]['due_ns'], result_receiver.poll,
                alive=lambda: vision_process is None or vision_process.poll() is None)
        wall = time.perf_counter()-started
        summary = dict(scope='native sensor/control probe; does not certify far/near mission or scene handoff',
            **provenance, probe_sha256=fingerprint, settings=vars(args) | {'output': str(output), 'adapter': str(adapter), 'catalog': str(args.catalog)},
            calibration_id=calibration_id(vision_config), gyro_configuration=[asdict(g.sensor.config) for g in gyros],
            body_properties=properties, dynamics=stepper.telemetry(), schedule=ctx.summary(),
            simulated_s=simulated_s, epoch_ns=epoch_ns, wall_s=wall, compute_s=compute, compute_real_time_factor=simulated_s/compute,
            realtime_paced=args.realtime, lag_s=wall-simulated_s if args.realtime else None,
            requested_simulation_rate=simulation_rate,
            pacing_backlog_s=wall-simulated_s/simulation_rate if args.realtime else None,
            accepted_visual=control.accepted_visual, invalid_visual=control.invalid_visual,
            fault_start_ns=fault_start_ns, fault_exercised=fault_start_ns is not None,
            vision=vision.status() if vision else None,
            sensor_delivery=delivery.status(), mission_configuration=mission_config.to_dict(),
            preparation=preparation.status() if preparation else None, unplanned_contacts=cross_contacts,
            visual_readiness=gate.status(now, time.time_ns() if args.ue else None),
            maximum_relative_position_error_m=maximum_relative_error,
            gyro_final_health=nav.gyro_health(now),
            maximum_checked_gravity_jacobian_difference_s2=gravity_cache_error,
            maximum_checked_gravity_acceleration_remainder_m_s2=gravity_acceleration_error,
            model_assets_modified=False, integration_complete=False)
        if demo:
            summary.update(acceptance_profile=mission_config.acceptance_profile, feedback_source=mission_config.feedback_source,
                operating_geometry=operating_geometry,
                validation_claim='simulation state feedback motion demonstration only')
        if near23:
            control.own_motion = own_motion
            near23_route.capture_feedback = auto_capture.audit.feedback if auto_capture and auto_capture.audit else None
            summary.update(acceptance_profile=mission_config.acceptance_profile,feedback_source=mission_config.feedback_source,
                handoff_continuity=handoff_continuity,
                near23_capture=dict(complete=auto_capture.complete,failure=auto_capture.failure,hold_s=auto_capture.hold_s)
                    if auto_capture else None)
        if mission:
            summary['scope'] = 'capture26 mission stage; integration acceptance remains separate'
        (output/'summary.json').write_text(json.dumps(summary, indent=2, default=str)+'\n', encoding='utf-8')
        # Private diagnostic checkpoint. It is deliberately not an accepted
        # mission handoff; physical/scene continuity must be validated separately.
        (output/'restart.json').write_text(json.dumps(dict(schema='space-sim-capture26-probe-checkpoint/2',
            model=provenance, mission_stage=mission, mission_id=mission_id, ephemeris_epoch_utc='2026 SEPTEMBER 02 00:00:00.000',
            calibration_id=calibration_id(vision_config), physical=stepper.physical_snapshot(epoch_ns=epoch_ns),
            navigation=nav.snapshot(), mission_configuration=mission_config.to_dict(),
            sensor_delivery=delivery.snapshot(),
            gyros=[g.sensor.snapshot() for g in gyros], gnss_receiver=receiver.snapshot(), gnss_pending=pending,
            retention=retention.snapshot() if retention else None,
            control=dict(depth_m=args.depth, pointing_error_deg=args.pointing_error_deg,
                         capture_reference=capture.target.position.tolist(),
                         operating_reference=(operations.target.position if operations else operating_reference).tolist(),
                         force_n=control.force_n.tolist(), last_gnss_ns=control.last_gnss_ns,
                         gyro_pending=control.gyro_pending, last_cooperative_ns=control.last_cooperative_ns,
                         platform_stability=operation_stability.snapshot() if operations else approach.stability.snapshot() if approach and not near23 else None,
                         preparation=preparation.snapshot() if preparation else None,
                         own_motion=own_motion.snapshot() if own_motion else None),
            environment=[m.snapshot(epoch_ns=epoch_ns) for m in environment.models] if environment else []),
            default=lambda a: a.tolist())+'\n', encoding='utf-8')
        (output/'delivery.json').write_text(json.dumps(delivery.events)+'\n', encoding='utf-8')
        (output/'performance.json').write_text(json.dumps(windows)+'\n', encoding='utf-8')
        if mission in ('near', 'far'):
            score.file.flush()
            if platform_log is not None:
                platform_log.file.flush()
            if auto_capture and auto_capture.audit:
                auto_capture.audit.file.flush()
                auto_capture.audit.force_file.flush()
            from simulation.capture26_results import write_stage_result
            write_stage_result(output, args, summary, score, windows, nav, gyros, receiver, pending,
                retention, capture, stepper, control, approach, cross_contacts,
                environment, mission_id, epoch_ns, ephemeris_epoch)
        try:
            print(json.dumps({key: summary[key] for key in ('compute_real_time_factor', 'accepted_visual', 'maximum_relative_position_error_m', 'integration_complete')}), flush=True)
        except OSError as error:
            # Detached Windows consoles can disappear after the physical result
            # was persisted. Preserve the completed result and record the I/O fault.
            (output/'console-warning.json').write_text(json.dumps(dict(error=str(error)))+'\n', encoding='utf-8')
        return summary
    except Exception as exc:
        (output/'failure.json').write_text(json.dumps(dict(
            **provenance, probe_sha256=fingerprint, integration_complete=False,
            error=str(exc), simulated_s=(stepper._last_nanos or 0)*1e-9,
            dynamics=stepper.telemetry(), calibration_id=calibration_id(vision_config)), indent=2)+'\n', encoding='utf-8')
        raise
    finally:
        score.close()
        packet_log.close()
        if platform_log is not None:
            platform_log.close()
        if channel:
            channel.close()
        if profiler:
            profiler.disable()
            profiler.dump_stats(output/'compute.pstats')
        if auto_capture and auto_capture.audit:
            auto_capture.audit.file.close()
            if auto_capture.audit.force_file:
                auto_capture.audit.force_file.close()
        if vision:
            vision.close()
        if result_receiver:
            result_receiver.close()
        if bridge:
            bridge.close()
        ctx.ports.close()
        orbit.gravity_factory.unloadSpiceKernels()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--adapter', type=Path, default=ROOT.parent/'space_sim_UE_Adapter')
    p.add_argument('--depth', type=float, default=2.)
    p.add_argument('--duration', type=float, default=12.)
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--gnss-delay', type=float, choices=(0., .1, .25), default=0.)
    p.add_argument('--ue', action='store_true')
    p.add_argument('--catalog', type=Path)
    p.add_argument('--render-port', type=int, default=5558)
    p.add_argument('--render-rate', type=int, choices=(10, 20, 50), default=10)
    p.add_argument('--capture-port', type=int, default=8767)
    p.add_argument('--realtime', action='store_true', help='pace the physical clock at --simulation-rate')
    p.add_argument('--simulation-rate', type=float, default=1.,
                   help='requested wall-clock pacing multiplier; physics/sensor rates remain unchanged')
    p.add_argument('--profile', action='store_true')
    p.add_argument('--restart', type=Path, help='private complete checkpoint; does not grant mission acceptance')
    p.add_argument('--preparation', type=Path, help='diagnose a model-bound candidate with normal joint servos')
    p.add_argument('--reference-depth', type=float, help='change only the controlled optical reference after private restart')
    p.add_argument('--save-all-images', action='store_true', help='retain every processed stereo pair for prevalidation')
    p.add_argument('--pointing-error-deg', type=float, nargs=3, default=[0., 0., 0.],
                   metavar=('X', 'Y', 'Z'), help='initial camera pointing offset; subsequent attitude evolves with body-rate damping')
    p.add_argument('--approach', action='store_true', help='exercise the configured near approach; diagnostic runs do not grant stage handoffs')
    p.add_argument('--fault', choices=('', 'image_loss', 'worker_exit', 'gnss_loss', 'communication_loss', 'service_gyro_loss',
                                      'target_gyro_loss', 'gyro_saturation'), default='')
    p.add_argument('--fault-after', type=float, default=2.)
    p.add_argument('--fault-after-ready', action='store_true', help='inject only after a current visual-ready gate, or report unexercised')
    p.add_argument('--fault-after-release', action='store_true', help='inject only after actual platform release')
    p.add_argument('--fault-duration', type=float, help='simulation duration of a recoverable injected outage')
    args = p.parse_args()
    if not 0 < args.duration <= 900 or not .25 <= args.depth <= 110:
        p.error('invalid probe duration or optical depth')
    if args.approach and args.depth < 51:
        p.error('approach entry must start outside the 50 m COM node')
    if args.reference_depth is not None and not .25 <= args.reference_depth <= 110:
        p.error('invalid diagnostic reference depth')
    if any(not abs(x) <= 10 for x in args.pointing_error_deg):
        p.error('prevalidation pointing offset must be finite within +/-10 degrees')
    if not 0 < args.fault_after < args.duration and args.fault:
        p.error('fault epoch must fall inside the probe')
    os.environ['SPACE_SIM_LOCAL_PUBLISH_STRIDE'] = '8'
    run(args)


if __name__ == '__main__':
    main()
