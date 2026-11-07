"""Capture task runtime dispatched by the standard run_scripted_task entry.

Uses the same 400 Hz native physics, 200 Hz task and 50 Hz authoritative
recording transport; this task does not load rendezvous/visual estimators.
"""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def run(args, *, graph_cache=None):
    # Separate graph: no task-box graph can be reused for the capture12 robot.
    os.environ.setdefault('SPACE_SIM_LOCAL_PUBLISH_STRIDE', '8')
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    native = load_native_grasp_module(ROOT/'model/SARM/platform')
    from simulation.sarm_model_profile import load_model_profile
    from simulation.assembly import AssemblyContext, SessionConfig, Slot
    from simulation.components.clock import ClockComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.components.arm_reference import JointReferenceComponent
    from simulation.components.attitude import AttitudeComponent
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from simulation.capture_arm_target import INITIAL_JOINTS
    from simulation.scripted_tasks.capture_contact_audit import CaptureAudit
    from simulation.scripted_tasks.capture_grasp import CaptureGraspTask, INTERFACE_CENTER, TARGET_FROM_INTERFACE
    from simulation.scripted_tasks.capture_initialization import CaptureInitialState, camera_view
    from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
    from simulation.teleop_scene import DEFAULT_EPHEMERIS_EPOCH_UTC
    from simulation.orbital_configuration import resolve_orbital_physics, environment_metadata
    from simulation.scripted_tasks.capture_platform import install_platform_environment, platform_contract
    from space_arm_platform.sampling import tick_time_ns, capture_preroll_ns
    from Basilisk.utilities import RigidBodyKinematics as rbk

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    from collection.capture_initial_pool import source_scene
    scene_instance = json.loads(args.scene.read_text()) if args.scene else source_scene()
    if 'environment' not in scene_instance:
        raise ValueError('capture v2 requires a complete platform scene; recreate the initial pool')
    if scene_instance.get('render_environment') != 'platform_main':
        raise ValueError('capture requires the original platform_main environment; simplified scenes are historical only')
    orbital_physics = resolve_orbital_physics(scene_instance)
    environment = scene_instance['environment']
    epoch_utc = environment.get('ephemeris_epoch_utc', DEFAULT_EPHEMERIS_EPOCH_UTC)
    initial = CaptureInitialState(**scene_instance.get('capture_initial_state', {}))
    source_model = (args.model or ROOT/'model/SARM/platform/sarm_dual_capture26.xml').resolve()
    from simulation.model_preparation.capture_grasp_model import prepare_capture_model
    model = prepare_capture_model(source_model, output/'model.xml').resolve()
    native.MODEL_PATH, native.TIME_STEP = model, .0025
    profile = load_model_profile(model)
    native.JOINTS, native.ACTUATORS = profile.joints, profile.arm_actuators
    native.KP, native.KD, native.TORQUE_LIMITS = np.full(6,500.), np.full(6,50.), np.full(6,100.)
    task = CaptureGraspTask(model, initial_joints=initial.arm_joint_position_rad)
    command, velocity = task.reference.copy(), np.zeros(12)
    operating = np.r_[np.deg2rad([90.,-60.,60.,0.,-90.,0.]), 0., 0.]
    saved_scene = source_model.with_name('initial-scene.json')
    if saved_scene.is_file():
        saved = json.loads(saved_scene.read_text())
        operating = np.asarray(saved['randomization']['arm_joint_position_rad'],dtype=float)
    if operating.shape != (8,) or not np.isfinite(operating).all():
        raise ValueError('capture scene needs eight finite operating-arm hold coordinates')
    scene_instance.setdefault('randomization',{})['arm_joint_position_rad'] = operating.tolist()
    op_publisher = HeldJointReferencePublisher(lambda t: (operating, np.zeros(8)), 8)
    publisher = HeldJointReferencePublisher(lambda t: (command, velocity), 12)
    sim = stepper = orbit = audit = transport = None
    report = dict(task_name=task.name, result='failed', reason='initialization_incomplete',
                  model=str(model), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  source_model=str(source_model),source_model_sha256=hashlib.sha256(source_model.read_bytes()).hexdigest(),
                  operating_hold_joint_position=operating.tolist(),
                  initial_state=asdict(initial), physics_hz=400, task_hz=200, capture_hz=50,
                  expert_inputs='simulator pose + encoders + contact forces; no RGB estimator',
                  policy_inputs='RGB + capture12 proprioception; no target truth',
                  framework_commit='0b55742fc5b8f4710ab1abfe2ae2c6d723d6dbb7',
                  capture_platform_contract=platform_contract(),
                  dynamics_graph_reused=False)
    started = time.monotonic()
    try:
        sim, scene, models, recorders = native._build_simulation(
            external_reference=op_publisher, dynamics_backend='local', local_substeps=args.substeps,
            record_history=False, defer_attitude_control=True)
        stepper = sim.local_stepper
        ctx = AssemblyContext(sim, scene, config=SessionConfig(model_path=model,
            dynamics_backend='local', publish_stride=stepper.publish_every))
        ctx.extra['template_id'] = 'sarm-dual-capture26'
        orbit = OrbitComponent(epoch_utc=epoch_utc, physics=orbital_physics)
        actuators = tuple([f'bottom_joint{i}_act' for i in range(1,7)] + ['axial_act','cam_act'] + [f'jaw_act_{i}' for i in range(4)])
        ctx.install([ClockComponent(), orbit, DynamicsCoreComponent(model), OrbitalGravityComponent(mode='bsk'),
            JointReferenceComponent(publisher, joints=tuple(j for _,j,_ in INITIAL_JOINTS), actuators=actuators,
                kp=[1500.]*6+[10000.,10.]+[8000.]*4, kd=[80.]*6+[80.,1.]+[40.]*4,
                limits=[80.]*6+[15.,1.]+[5.]*4, ik_rate_hz=200), AttitudeComponent(model)])
        modules = install_platform_environment(ctx, scene_instance, orbit)
        ctx.extra['capture_environment'] = modules
        report['orbital_physics'] = environment_metadata(orbital_physics, orbit.earth, epoch_utc=epoch_utc)
        report['platform_modules'] = dict(
            celestial_registered='celestial' in ctx.extra,
            configured_drag=bool(modules['drag']),
            handoff_flight_environment=modules['flight'] is not None,
            plug_retention=modules['retention'] is not None,
            service_gyro=asdict(modules['gyro'].sensor.config),
            attitude_control=sim.attitude_control.telemetry())
        # The scenario owns operating-arm servo wiring; schedule its reference.
        ctx.add(op_publisher, Slot.COMMAND, component='operating_hold')
        ctx.keep_alive(op_publisher)
        if args.capture_control_port:
            from simulation.scripted_tasks.capture_grasp_transport import CaptureGraspTransport
            transport = CaptureGraspTransport(ctx, task, command, report['model_sha256'], scene_instance,
                control_port=args.capture_control_port, render_port=args.capture_render_port, catalog=args.catalog)
            report['recording_cameras'] = [camera.to_payload() for camera in transport.render.bridge._cameras]
            report['original_cameras'] = transport.original_cameras
            report['scene_lights'] = [visual.to_payload() for visual in transport.render.bridge._visuals
                                     if visual.kind == 'light']
        audit = CaptureAudit(model, stepper, output_path=output/'contacts_400hz.jsonl')
        from Basilisk.architecture import sysModel
        unplanned = []
        qa = []
        class AuditSampler(sysModel.SysModel):
            def UpdateState(self, stamp):
                nonlocal unplanned
                if qa and stepper.state_initialized:
                    unplanned = audit.record(stamp*1e-9, task.stage, stepper.qpos[qa].copy())
        sampler = AuditSampler(); sampler.ModelTag = 'captureGraspContactAudit400Hz'
        ctx.add(sampler, Slot.SENSOR, component='capture_grasp_audit')
        ctx.keep_alive(sampler)
        report['module_schedule'] = ctx.summary()
        sim.InitializeSimulation()
        for (body, joint), value in zip(profile.joints, operating):
            scene.getBody(body).getScalarJoint(joint).setPosition(float(value))
        for (body,joint,_), value in zip(INITIAL_JOINTS, command):
            scene.getBody(body).getScalarJoint(joint).setPosition(float(value))
        gp, gr = task.chain.forward(command[:6])
        target_p, target_r = initial.geometry(gp, gr)
        view = camera_view(gp, gr, target_p, target_r)
        report['camera_projection'] = view
        if not view['projected_visible']:
            report.update(reason='initial_camera_projection_rejected', failure_kind='initialization_rejected')
            return 1
        # Common inertial velocity, zero relative velocity/angular rate at reset.
        from Basilisk.utilities import orbitalMotion
        elements = orbitalMotion.ClassicElements()
        config = environment['orbit']
        elements.a = float(orbit.earth.radEquator)+config['altitude_m']
        elements.e = config['eccentricity']
        for field, key in [('i','inclination_deg'),('Omega','raan_deg'),
                           ('omega','argument_of_periapsis_deg'),('f','true_anomaly_deg')]:
            setattr(elements, field, np.deg2rad(config[key]))
        r0, orbital_velocity = map(np.asarray, orbitalMotion.elem2rv(float(orbit.earth.mu), elements))
        v0 = orbital_velocity+np.asarray(native.COMMON_VELOCITY)
        report['initial_orbit'] = dict(elements=config, position_inertial_m=r0.tolist(),
                                      velocity_inertial_m_s=v0.tolist(), relative_velocity_m_s=[0.,0.,0.])
        report['initial_geometry'] = dict(tcp_position_service_m=gp.tolist(),
            target_origin_service_m=target_p.tolist(), spacecraft_origin_separation_m=float(np.linalg.norm(target_p)),
            tcp_interface_distance_m=float(np.linalg.norm(target_p+target_r@INTERFACE_CENTER-gp)),
            axial_gap_m=initial.gap_m)
        assignments = {'cubesat_bus': (np.zeros(3),np.eye(3)), 'capture_target': (target_p,target_r)}
        root = ET.parse(model).getroot()
        from simulation.scripted_tasks.capture_payload import initial_payload_poses
        payload_poses = initial_payload_poses(root,target_p,target_r)
        assignments.update(payload_poses)
        report['target_owned_free_parts'] = list(payload_poses)
        report['active_capture_weld'] = any(e.get('active','true')=='true'
            for e in root.findall('./equality/weld') if e.get('name')=='preheld_capture_lock')
        for body in root.findall('./worldbody/body'):
            name = body.get('name')
            pos, rot = assignments.get(name, (np.fromstring(body.get('pos','0 0 0'),sep=' '),
                quaternion_wxyz_to_matrix(np.fromstring(body.get('quat','1 0 0 0'),sep=' '))))
            item = scene.getBody(name)
            item.setPosition((r0+pos).tolist()); item.setVelocity(v0.tolist())
            item.setAttitude(rbk.C2MRP(rot.T).tolist()); item.setAttitudeRate([0.,0.,0.])
        orbit.origin.set_state(r0, orbital_velocity)
        joints = [scene.getBody(b).getScalarJoint(j) for b,j,_ in INITIAL_JOINTS]
        qa = [j.getQposAdr() for j in joints]; va = [j.getQvelAdr() for j in joints]
        from simulation.scripted_tasks.native_state import CaptureState
        state_reader = CaptureState(scene, stepper)
        sim.ConfigureStopTime(0); sim.ExecuteSimulation()
        from simulation.scripted_tasks.capture_preflight import validate_initial_and_path
        preflight = validate_initial_and_path(model,stepper,task,initial,view)
        report['preflight'] = preflight
        if not preflight['accepted']:
            report.update(reason=preflight['reason'],failure_kind='initialization_rejected')
            return 1
        report['initial_qpos'] = stepper.qpos.tolist()
        report['initial_qvel'] = stepper.qvel.tolist()
        from collection.runtime_provenance import source_identity, native_identity, environment_identity, loaded_native_provenance
        report['source_identity'] = source_identity(ROOT,ROOT.parent/'ue-adapter')
        report['native_identity'] = native_identity(ROOT)
        report['dynamics_environment'] = environment_identity()
        report['loaded_native'] = loaded_native_provenance(ROOT)
        report['dynamics_python'] = sys.executable
        (output/'initialization.json').write_text(json.dumps(report,indent=2))
        epoch = None if transport else int(args.warmup_sim_seconds*1e9)
        frame = 0
        last_stage = None
        with (output/'trace.jsonl').open('w') as trace:
            while True:
                if transport:
                    transport.flush()
                    if args.capture_stop_file and args.capture_stop_file.exists(): break
                    if (task.result is not None and not transport.enabled and
                            not transport.render.bridge.last_capture_state.get('capture_episode_id')):
                        # Same OFF publication barrier as the original runner.
                        time.sleep(.01); continue
                    if not transport.can_advance(): time.sleep(.002); continue
                stamp = tick_time_ns(frame, 200)
                if transport and epoch is None and stamp >= capture_preroll_ns(args.warmup_sim_seconds) and not transport.enabled:
                    time.sleep(.01); continue
                if transport and epoch is None and transport.enabled:
                    epoch = stamp; transport.task_start_ns = stamp
                elapsed = 0. if epoch is None else (stamp-epoch)*1e-9
                q, v = stepper.qpos[qa].copy(), stepper.qvel[va].copy()
                pi, ri, interface_velocity, interface_omega = state_reader.interface()
                sample = dict(t=elapsed,q=q,v=v,interface_position=pi,interface_rotation=ri,
                    interface_linear_velocity=interface_velocity,
                    interface_angular_velocity=interface_omega,
                    native_qpos=stepper.qpos.copy(),
                    feedback=dict(audit.feedback),unplanned_contacts=unplanned)
                if epoch is not None and elapsed >= 0. and task.result is None:
                    if elapsed >= args.max_sim_seconds: task.fail('total_simulation_timeout',sample)
                    command[:], velocity[:] = task.step(sample)
                if transport:
                    transport.sequence += 1; transport.task_elapsed_s = elapsed
                    transport.feedback = dict(audit.feedback)
                if frame % 4 == 0:
                    trace.write(json.dumps(dict(sim_time_ns=str(stamp),**task.metrics(),q=q.tolist(),
                        qvel=v.tolist(),reference=command.tolist(),feedback=audit.feedback,
                        interface_position_body_m=pi.tolist(),interface_rotation_body=ri.tolist()))+'\n')
                if task.stage != last_stage or frame % 1000 == 0:
                    print(json.dumps(dict(type='capture_grasp_progress',t=elapsed,**task.metrics(),feedback=audit.feedback)),flush=True)
                    last_stage = task.stage
                if task.result is not None and not transport: break
                frame += 1
                sim.ConfigureStopTime(tick_time_ns(frame,200)); sim.ExecuteSimulation()
        report.update(result=task.result or 'failed',reason=task.reason,metrics=task.metrics(),
                      carry_plan=task.carry_plan,
                      final_feedback=audit.feedback,simulation_seconds=frame/200.,
                      final_service_gyro=modules['gyro'].observation(),
                      final_atmospheric_drag=[c.telemetry() for c in modules['drag']],
                      final_orbital_environment=stepper.telemetry()['orbital_environment'])
        return 0 if task.result == 'success' else 1
    finally:
        report['wall_seconds'] = time.monotonic()-started
        (output/'summary.json').write_text(json.dumps(report,indent=2))
        if transport: transport.close()
        if audit:
            audit.file.close()
            if audit.force_file: audit.force_file.close()
        if sim:
            sim.TotalSim.clearProcsFromThreads()
            sim.TotalSim.deleteThreads()
        if stepper: stepper.close()
        if orbit and orbit.gravity_factory: orbit.gravity_factory.unloadSpiceKernels()
