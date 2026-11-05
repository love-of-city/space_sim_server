"""Saved dual-arm camera scenes on the production BSK/MuJoCo/UE pipeline.

The approved static workspace pose is the initial condition. Original position
actuators hold it while the free satellite propagates under Earth J2, Sun and
Moon gravity. This preview does not advertise unimplemented dual-arm teleop.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]


def run(args):
    from Basilisk.architecture import messaging
    from Basilisk.simulation import mujoco
    from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
    from simulation.assembly import AssemblyContext, SessionConfig, Slot
    from simulation.components.orbit import OrbitComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.local_mujoco_stepper import LocalMujocoStepper
    from space_arm_platform.orbital_physics import OrbitalPhysics
    from simulation.control_client import SimulationControlClient
    from space_arm_platform.protocol import CONTROL_PROTOCOL
    from bsk_render_adapter import BasiliskRenderBridge, SceneSettings, GeometryVisual
    from simulation.teleop_grasp_unreal import _register_celestial_bodies
    from simulation.camera_lighting import overview_camera_lamp

    instance=json.loads(args.scene_instance.read_text(encoding='utf-8'))
    version=int(instance['template_id'].rsplit('-',1)[1])
    model=ROOT/f'model/dual_stowed_service/camera_version_{version}/dual_stowed_service.xml'
    config=json.loads((model.parent/'design.json').read_text())
    checksum=hashlib.sha256(model.read_bytes()).hexdigest()
    if config['model_sha256']!=checksum:raise ValueError('Saved model hash changed')
    pose=json.loads((ROOT/'model/dual_stowed_service/workspace_pose.json').read_text())
    root=ET.parse(model).getroot()
    args.output.mkdir(parents=True,exist_ok=True)
    sim=SimulationBaseClass.SimBaseClass()
    process=sim.CreateNewProcess('dualService')
    process.addTask(sim.CreateNewTask('graspTask',macros.sec2nano(.0025)),30)
    process.addTask(sim.CreateNewTask('sceneInit',macros.sec2nano(.0025)))
    from copy import deepcopy
    from simulation.publication_scene import prepare_publication_scene
    publication=deepcopy(root);prepare_publication_scene(publication)
    files=sorted({str((model.parent/node.get('file')).resolve()) for node in publication.findall('./asset/mesh[@file]')})
    scene=mujoco.MJScene(ET.tostring(publication,encoding='unicode'),files)
    sim.AddModelToTask('sceneInit',scene)
    ctx=AssemblyContext(sim,scene,config=SessionConfig(model_path=model,publish_stride=8))
    env=instance['environment']
    orbit=OrbitComponent(epoch_utc=env['ephemeris_epoch_utc'],
                         physics=OrbitalPhysics.model_validate(env['physics']),include_moon=True)
    ctx.install([orbit])
    stepper=LocalMujocoStepper(scene,model,[],origin=orbit.origin,substeps=1,publish_every=8)
    ctx.ports.set_stepper(stepper);ctx.add(stepper,Slot.PHYSICS,component='dynamics_core')
    ctx.install([OrbitalGravityComponent(force_every=8)])
    held=[]
    for node in root.findall('./actuator/*'):
        message=messaging.SingleActuatorMsg()
        scene.getSingleActuator(node.get('name')).actuatorInMsg.subscribeTo(message)
        held.append((node,message))
    client=None
    if args.control_port:
        client=SimulationControlClient('127.0.0.1',args.control_port,'dual-service-preview',
            allow_scene_reset=False,capabilities=['reliable_observations_v1','joint_observation','dual_service_preview'])
        client.start();client.wait_until_ready(30.)
    bridge=None
    preview_capture={'capture_episode_id':'','capture_request_id':''}
    if not args.no_render:
        bridge=BasiliskRenderBridge(host='127.0.0.1',port=args.render_port,origin_object='teleop/cubesat_bus',
            frame_rate_hz=50,capture_state_provider=lambda:preview_capture.copy(),
            recording_path=args.output/'scene.bskrec')
        bridge.add_mj_scene(scene,namespace='teleop',source_path=model,mesh_asset_catalog=args.catalog,
            camera_picture_in_picture=True,camera_capture_rate_hz=5.,camera_pip_resolution=(1024,1024),
            camera_capture_products=('rgb',),camera_display_names={
                'spacecraft_overview':f'Global V{version}','right_sarm_wrist_cam':'Left wrist','left_sarm_wrist_cam':'Right wrist'})
        order=['teleop/camera/spacecraft_overview','teleop/camera/right_sarm_wrist_cam','teleop/camera/left_sarm_wrist_cam']
        bridge._cameras.sort(key=lambda c:order.index(c.camera_id))
        for side in ('left_', 'right_'):
            for link in ('link1','link2','link3','link4','link5','wrist_ft_tool'):
                binding=next(item for item in bridge._objects if item.object_id==f'teleop/{side}{link}')
                if not binding.geometries:
                    raise ValueError(f'Missing UE visual geometry: {binding.object_id}')
        # Original platform MLI is a separate UV-mapped visual skin. The new
        # recessed hull has a different asset path, so the legacy path-based
        # overlay mapping cannot attach it automatically. Apply precisely the
        # hull's approved similarity transform: 2 * (source - [.2,.265,0]).
        hull_scale=float(config['hull_uniform_scale'])
        bus_visual=next(item for item in bridge._objects if item.object_id=='teleop/cubesat_bus')
        bus_visual.geometries.append(GeometryVisual(
            geometry_id='teleop/cubesat_bus/visual/mli',shape='mesh',dimensions_m=(1.,1.,1.),
            position_body_m=(-.2*hull_scale,-.265*hull_scale,0.),
            asset_type='static_mesh',
            asset_path='/Game/Generated/DualServiceMLI/SM_DualServiceMLI.SM_DualServiceMLI',
            scale=(hull_scale,)*3,use_asset_materials=True,render_role='visual'))
        for slot,camera in enumerate(bridge._cameras,1):camera.picture_in_picture_slot=slot
        bridge.add_lights(overview_camera_lamp('teleop/camera/spacecraft_overview'))
        for light in bridge._visuals:
            if light.kind == 'light' and light.visual_id.endswith('_wrist_lamp'):
                light.range_m = 1.
                light.properties = {**light.properties, 'intensity': 500.}
        _register_celestial_bodies(bridge,orbit.earth,orbit.sun,orbit.moon)
        bridge.set_scene_settings(SceneSettings(origin_object_id='teleop/cubesat_bus',
            default_camera_target='teleop/cubesat_bus',default_camera_distance_m=4.,
            use_scene_lighting=False,
            sunlight_intensity_scale=env['lighting']['sunlight_intensity_scale'],
            interpolation_delay_ms=15.,max_extrapolation_ms=50.,orbit_lines=False,trajectory_history=False))
        ctx.add(bridge,Slot.OUTPUT,component='render')
    sim.InitializeSimulation();sim.disableTask('sceneInit')
    oe=orbitalMotion.ClassicElements();saved=env['orbit']
    oe.a=orbit.earth.radEquator+saved['altitude_m'];oe.e=saved['eccentricity']
    oe.i=math.radians(saved['inclination_deg']);oe.Omega=math.radians(saved['raan_deg'])
    oe.omega=math.radians(saved['argument_of_periapsis_deg']);oe.f=math.radians(saved['true_anomaly_deg'])
    r,v=orbitalMotion.elem2rv(orbit.earth.mu,oe)
    orbit.origin.set_state(np.array(r),np.array(v))
    q=np.array(pose['qpos'],dtype=float);q[:3]+=r
    dq=np.zeros(stepper.nv);dq[:3]=v
    scene.dynManager.getStateObject('mujocoQpos').setState(q.reshape(-1,1))
    scene.dynManager.getStateObject('mujocoQvel').setState(dq.reshape(-1,1))
    scene.postIntegration(0);scene.UpdateState(0)
    joints={j.get('name'):(body.get('name'),j.get('name')) for body in root.iter('body') for j in body.findall('joint')}
    addresses={name:scene.getBody(body).getScalarJoint(name).getQposAdr() for name,(body,_) in joints.items()}
    for node,message in held:
        payload=messaging.SingleActuatorMsgPayload()
        payload.input=float(q[addresses[node.get('joint')]]) if node.get('joint') else 0.
        message.write(payload,0)
    indices=[addresses[f'right_joint{i}'] for i in range(1,7)]
    velocity_indices=[scene.getBody(joints[f'right_joint{i}'][0]).getScalarJoint(f'right_joint{i}').getQvelAdr() for i in range(1,7)]
    reference=q[indices].copy()
    report={'schema':'dual-service-full-system/1','version':version,'model_sha256':checksum,
        'scene_instance':instance,'celestial_bodies':['earth','sun','moon'],
        'physics':'production LocalMujocoStepper + native BSK gravity + SPICE; Earth J2, Sun and Moon',
        'initial_pose':'saved static workspace pose; original position actuators hold it; no teleop/grasp acceptance',
        'schedule':ctx.summary(),'render_session_id':bridge.session_id if bridge else None,
        'max_penetration_m':0.,'sampled_contact_count':0,'steps':0}
    start=time.monotonic();frame=0
    preview_start=10.;preview_end=12.;preview_stamp='';preview_pending=True
    refresh_path=args.output/'refresh_preview.request';refresh_mtime=0
    lighting_path=args.output/'preview_lighting.json';lighting_mtime=0
    try:
        while args.duration<=0 or frame*.02<args.duration:
            if client and not client.has_observation_capacity():time.sleep(.01);continue
            stop_ns=round(frame*.02*1e9)
            if bridge and frame%50==0 and lighting_path.exists():
                modified=lighting_path.stat().st_mtime_ns
                if modified!=lighting_mtime:
                    lighting=json.loads(lighting_path.read_text(encoding='utf-8-sig'))
                    intensity=float(lighting.get('wrist_intensity',500.))
                    if not math.isfinite(intensity) or not 0 <= intensity <= 8000:
                        raise ValueError('Preview wrist intensity outside [0,8000]')
                    for light in bridge._visuals:
                        if light.kind=='light' and light.visual_id.endswith('_wrist_lamp'):
                            light.properties={**light.properties,'intensity':intensity}
                    bridge._manifest_revision+=1
                    bridge._record_and_retain_static()
                    (args.output/'live_manifest.json').write_text(json.dumps(bridge._manifest_message(),indent=2))
                    for key,camera in [('global_enabled','spacecraft_overview'),
                                       ('left_enabled','right_sarm_wrist_cam'),
                                       ('right_enabled','left_sarm_wrist_cam')]:
                        if key in lighting:
                            if not isinstance(lighting[key],bool):raise ValueError(f'{key} must be boolean')
                            bridge.publish_event('camera_work_light',{'camera_id':f'teleop/camera/{camera}', 'enabled':lighting[key]})
                    lighting_mtime=modified
                    preview_start=frame*.02+3.;preview_end=preview_start+2.
                    preview_stamp=f'_light_{modified}';preview_pending=True
            if frame%50==0 and refresh_path.exists():
                modified=refresh_path.stat().st_mtime_ns
                if modified!=refresh_mtime:
                    refresh_mtime=modified;preview_start=frame*.02+2.;preview_end=preview_start+2.
                    preview_stamp=f'_{modified}';preview_pending=True
            preview_capture['capture_episode_id']='visual-preview-only' if preview_start <= frame*.02 <= preview_end else ''
            sim.ConfigureStopTime(stop_ns);sim.ExecuteSimulation()
            if bridge and preview_pending and frame*.02>=preview_end:
                bridge.publish_event('capture26_stage_snapshot',{'filename':f'full_system_v{version}{preview_stamp}.png',
                    'message':f'Dual service V{version}: Earth, Sun, Moon; saved workspace pose'})
                preview_pending=False
            if bridge and frame==750 and bridge.recorder:
                bridge.recorder.close()
                bridge.recorder=None
            contacts=stepper.contact_snapshot()
            if len(contacts):
                report['max_penetration_m']=max(report['max_penetration_m'],float(max(0,-contacts[:,4].min())))
                report['sampled_contact_count']+=len(contacts)
            if client:
                pos=np.asarray(scene.assembleFullQpos()).reshape(-1)
                msg={'protocol':CONTROL_PROTOCOL,'type':'observation','simulation_id':'dual-service-preview',
                    'scene_instance_id':instance['instance_id'],'scene_template_id':instance['template_id'],
                    'render_session_id':bridge.session_id if bridge else '',
                    'step_id':str(frame+1),'render_frame_id':str(frame),
                    'sim_time_ns':str(stop_ns),'wall_time_ns':str(time.time_ns()),'applied_action_sequence':'0',
                    'joint_position_rad':pos[indices].tolist(),'joint_velocity_rad_s':np.asarray(scene.assembleFullQvel()).reshape(-1)[velocity_indices].tolist(),
                    'target_joint_position_rad':reference.tolist(),'model_sha256':checksum,
                    'ik_mode':'saved_pose_hold','ik_status':'preview_only','celestial_bodies':['earth','sun','moon'],
                    'orbital_dynamics':stepper.telemetry()['orbital_environment'],
                    'dual_arm_joint_position_rad':{p:[float(pos[addresses[f'{p}joint{i}']]) for i in range(1,7)] for p in ('left_','right_')}}
                client.send_observation(msg)
            if frame%50==0:
                report.update(sim_time_s=frame*.02,steps=stepper.step_count,wall_time_s=time.monotonic()-start,
                    telemetry=stepper.telemetry(),celestial_states={name:{'position_m':list(ctx.ports.planet_state(name).read().PositionVector),
                        'velocity_m_s':list(ctx.ports.planet_state(name).read().VelocityVector)} for name in ('earth','sun','moon')})
                temp=args.output/'runtime.json.tmp';temp.write_text(json.dumps(report,indent=2),encoding='utf-8');temp.replace(args.output/'runtime.json')
                print(json.dumps({'sim_time_s':frame*.02,'version':version,'steps':stepper.step_count,'max_penetration_m':report['max_penetration_m']}),flush=True)
            frame+=1
            if args.realtime:
                delay=start+frame*.02/instance['runtime']['simulation_rate']-time.monotonic()
                if delay>0:time.sleep(min(delay,.02))
    finally:
        if bridge:bridge.close()
        if client:client.close()
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene-instance',type=Path,required=True);p.add_argument('--catalog',type=Path)
    p.add_argument('--render-port',type=int,default=15558);p.add_argument('--control-port',type=int,default=0)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=0.)
    p.add_argument('--no-render',action='store_true');p.add_argument('--realtime',action='store_true')
    run(p.parse_args())
