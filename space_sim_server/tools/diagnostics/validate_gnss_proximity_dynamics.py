"""GNSS and optional real-image diagnostics on the articulated dual-arm model.

Run in a separate process with an explicit compatible Basilisk installation.
The guarded case starts at 17.4 m (not a far-flight handoff). Hold cases start
at their declared optical depth; they never demonstrate boundary crossing.
Estimator/control translation comes from GNSS, optional real image corners,
commanded forces and model kinematics. Truth is used by simulated receivers,
ideal cooperative attitude telemetry, rendering and separate scoring only.
Image trials spawn private UE processes; original lighting and live scene remain.
"""
import argparse
import ast
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--basilisk-root',type=Path,required=True)
parser.add_argument('--source-server',type=Path,required=True)
parser.add_argument('--scene-model',type=Path,required=True)
parser.add_argument('--camera-config',type=Path,required=True)
parser.add_argument('--source-scene',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--mode',choices=['guarded_approach','hold','gnss_leg','pointing_handoff','continuous_original','vbar_original'],required=True)
parser.add_argument('--depth-m',type=float,default=17.4)
parser.add_argument('--end-depth-m',type=float,default=3.36)
parser.add_argument('--duration-s',type=float,default=180.)
parser.add_argument('--seed',type=int,default=7)
parser.add_argument('--tolerance-mm',type=float,default=3.)
parser.add_argument('--pointing-depth-m',type=float,default=17.4)
parser.add_argument('--initial-target-angle-deg',type=float,default=90.)
parser.add_argument('--initial-target-axis',type=float,nargs=3,default=[1.,2.,3.])
parser.add_argument('--pointing-gnss-continuation',action='store_true',
                    help='Diagnostic only: bypass visual wait after real target alignment')
parser.add_argument('--image-channel',choices=['wide'],help='Original stereo only; extra cameras withdrawn')
parser.add_argument('--long-focus-fov-deg',type=float,default=12.)
parser.add_argument('--image-position-sigma-m',type=float,
                    help='Explicit total per-axis position uncertainty assumption for this image experiment')
parser.add_argument('--cooperative-attitude-images',action='store_true',
                    help='Explicit position-only image route aided by ideal same-epoch cooperative attitude')
parser.add_argument('--images-after-prealignment',action='store_true',
                    help='Start image sampling at VisualWait; preview still follows actual prealignment dynamics')
parser.add_argument('--terminal-hold-s',type=float,default=30.)
parser.add_argument('--vision-threads',type=int,default=4)
parser.add_argument('--station-tolerance-m',type=float,default=.05)
parser.add_argument('--visual-dwell-s',type=float,default=0.)
parser.add_argument('--near-image-position-sigma-m',type=float,default=.006,
                    help='Dual-only: total per-axis near-image uncertainty assumption')
parser.add_argument('--kinematic-origin-prediction',action='store_true',
                    help='Model rotating body-origin lever arms using ideal gyro/encoder telemetry')
parser.add_argument('--tool-gap-m',type=float,
                    help='Hold-only: place mechanism center on gripper -Z at this signed axial gap')
parser.add_argument('--gnss-bridge-diagnostic',action='store_true',
                    help='Explicit trial policy: GNSS via 5.5 m to original-stereo acquisition; production rules unchanged')
parser.add_argument('--near-checkpoint-depth-m',type=float,default=1.5)
parser.add_argument('--platform-gap-m',type=float,default=.5)
parser.add_argument('--with-grasp',action='store_true',help='Continue the SAME dynamics into image/contact-driven grasp')
args = parser.parse_args()
continuous_mode = args.mode in {'continuous_original', 'vbar_original'}
if continuous_mode:
    if args.pointing_gnss_continuation or args.cooperative_attitude_images:
        parser.error('continuous original-camera trial uses full image PnP; legacy bypass/aided image routes are not used')
    if not 0<args.platform_gap_m<args.near_checkpoint_depth_m<5.5<args.pointing_depth_m<args.depth_m:
        parser.error('continuous checkpoints require platform gap < near checkpoint < 5.5 < pointing depth < initial depth')
    if args.with_grasp and not args.image_channel:
        parser.error('grasp requires actual original-stereo images')
elif args.gnss_bridge_diagnostic or args.with_grasp:
    parser.error('bridge and grasp flags require --mode continuous_original')
if args.image_channel and (args.image_position_sigma_m is None or args.image_position_sigma_m<=0):
    parser.error('image experiments require --image-position-sigma-m > 0')
if args.image_channel and args.pointing_gnss_continuation:
    parser.error('real image handoff cannot be combined with diagnostic visual bypass')
if args.cooperative_attitude_images and not args.image_channel:
    parser.error('--cooperative-attitude-images requires an actual image channel')
if args.image_channel=='dual' and (not args.cooperative_attitude_images or args.near_image_position_sigma_m<=0):
    parser.error('dual-range trial requires cooperative far attitude and positive near uncertainty')
if args.tool_gap_m is not None and (args.mode!='hold' or args.tool_gap_m<=0):
    parser.error('--tool-gap-m is positive and hold-only')
if args.vision_threads<1 or args.terminal_hold_s<0:
    parser.error('positive vision threads and nonnegative terminal hold required')
SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(args.basilisk_root.resolve()),str(SERVER),str(SERVER/'backend')]

import numpy as np
import Basilisk
from Basilisk.architecture import messaging,sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass,RigidBodyKinematics as rbk
from simulation.assembly import AssemblyContext,SessionConfig,Slot
from simulation.attitude_control import AttitudeControl
from simulation.components.clock import ClockComponent
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.components.plug_retention import PlugRetentionComponent
from simulation.plug_retention import load_retention_parameters
from simulation.cooperative_pointing import bounded_pointing_torque
from simulation.differential_gnss import DifferentialGNSS
from simulation.end_stereo_pointing import EndStereoGeometry,PrealignmentController
from simulation.pointing_handoff import PointingHandoff,PointingHandoffConfig
from simulation.original_stereo_sequence import OriginalStereoSequence
from simulation.origin_lever_arm import OriginLeverArm
from simulation.dual_range_vision import AcceptedImageGate
from simulation.image_relative_navigation import ImageRelativeNavigation
from simulation.joint_reference_publisher import HeldJointReferencePublisher
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.mujoco_plugins import validate_plugin_manifest
from simulation.navigation_gravity import BskNavigationGravity
from simulation.near_field.center_kinematics import CenterKinematics,center_relative
from simulation.near_field.navigation import RelativeEKF
from simulation.near_field.orbital_guidance import vbar_frame,rotating_acceleration
from simulation.near_field.parameters import PARAMETERS as P
from simulation.near_field.reference import RestLeg,StopReference
from simulation.near_field.vbar_approach import VbarApproach
from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.orbital_configuration import resolve_orbital_physics
from space_arm_platform.gnss_config import GNSSConfig


def native_runtime():
    path = Path(Basilisk.__file__).resolve().parent/'mujoco.dll'
    runtime = ctypes.CDLL(str(path))
    runtime.mj_version.restype = ctypes.c_int
    if runtime.mj_version()!=3007000:
        raise RuntimeError('compatible MuJoCo 3.7.0 runtime required; no ABI bypass')
    manifest = args.source_server/'run/native_acceleration/sarm_thread.json'
    document = json.loads(manifest.read_text(encoding='utf-8-sig'))
    plugin = args.source_server/document['library']
    validate_plugin_manifest(plugin,manifest,runtime.mj_version())
    if hashlib.sha256((args.source_server/'native/sarm_thread_sdf.cpp').read_bytes()).hexdigest()!=document['source_sha256']:
        raise RuntimeError('source SDF plugin is stale')
    runtime.mj_loadPluginLibrary.argtypes = [ctypes.c_char_p]
    runtime.mj_loadPluginLibrary.restype = None
    runtime.mj_loadPluginLibrary(str(plugin.resolve()).encode('utf-8'))
    return runtime,path


def prepare(output):
    original = ET.parse(args.scene_model).getroot()
    root = ET.fromstring(ET.tostring(original))
    compiler = root.find('compiler')
    meshdir = args.scene_model.parent/compiler.get('meshdir','')
    files = []
    for selector in ('./asset/mesh[@file]','.//flexcomp[@file]','./asset/texture[@file]'):
        for element in root.findall(selector):
            path = (meshdir/element.get('file')).resolve()
            if not path.is_file() or path.read_bytes().startswith(b'version https://git-lfs'):
                raise ValueError(f'missing model asset: {path}')
            element.set('file',path.as_posix())
            files.append(str(path))
    compiler.set('meshdir','.')
    compiler.set('fusestatic','false')
    # Only the publication-only copy omits constraints/rigid collision flexes,
    # exactly as the production local-backend scenario loader does.
    published = ET.fromstring(ET.tostring(root))
    flag = published.find('./option/flag')
    if flag is None:
        flag = ET.SubElement(published.find('option'),'flag')
    flag.set('contact','disable');flag.set('constraint','disable')
    for parent in published.iter():
        for child in list(parent):
            if child.tag=='flexcomp' and child.get('rigid','false')=='true':
                parent.remove(child)
    path = output/'model.xml'
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    settings=json.loads(args.scene_model.with_name('attitude_control.json').read_text(encoding='utf-8-sig'))
    (output/'attitude_control.json').write_text(json.dumps(settings,indent=2),encoding='utf-8')
    return original,root,path,published,files


def statistics(rows):
    result = dict(samples=len(rows))
    for key in ('raw_position_error_mm','filtered_position_error_mm','mechanism_position_error_mm',
                'true_tracking_error_mm','estimated_tracking_error_mm'):
        values=np.array([r[key] for r in rows if r.get(key) is not None])
        result[key]=(dict(rmse=float(np.sqrt(np.mean(values**2))),p95=float(np.percentile(values,95)),
                          max=float(values.max())) if len(values) else None)
    vectors=np.array([r['filtered_position_error_xyz_mm'] for r in rows if 'filtered_position_error_xyz_mm' in r])
    if len(vectors):
        result['filtered_position_xyz_mm']=dict(bias=vectors.mean(axis=0).tolist(),
            rmse=np.sqrt(np.mean(vectors**2,axis=0)).tolist(),max_abs=np.abs(vectors).max(axis=0).tolist())
    return result


def main():
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    sources=output/'diagnostic_sources';sources.mkdir()
    for source in (Path(__file__),SERVER/'simulation/pointing_handoff.py',
                   SERVER/'simulation/end_stereo_pointing.py',SERVER/'simulation/cooperative_pointing.py',
                   SERVER/'simulation/near_field/navigation.py',SERVER/'simulation/image_relative_navigation.py',
                   SERVER/'simulation/dual_range_vision.py',SERVER/'tools/diagnostics/native_image_dynamics_fixture.py',
                   SERVER/'simulation/origin_lever_arm.py',
                   SERVER/'simulation/original_stereo_sequence.py',
                   SERVER/'simulation/vbar_stereo_sequence.py',SERVER/'simulation/near_field/vbar_approach.py',
                   SERVER/'simulation/near23_navigation.py',SERVER/'simulation/stereo_predictive_capture.py',
                   SERVER/'simulation/visual_predictive_capture.py',SERVER/'simulation/own_motion.py',
                   SERVER/'simulation/body_stereo_capture.py',SERVER/'simulation/capture_arm_target.py',
                   SERVER/'simulation/capture_sequence.py',SERVER/'simulation/capture_audit.py',
                   SERVER/'backend/space_arm_platform/cooperative_stereo_translation.py',
                   SERVER/'backend/space_arm_platform/multicamera_pose.py'):
        (sources/source.name).write_bytes(source.read_bytes())
    runtime,runtime_path=native_runtime()
    original,root,model_path,published,files=prepare(output)
    source_scene=json.loads(args.source_scene.read_text(encoding='utf-8-sig'))
    entry_source=args.source_server/'simulation/teleop_grasp_unreal.py'
    entry_tree=ast.parse(entry_source.read_text(encoding='utf-8-sig'))
    gains={}
    for node in entry_tree.body:
        if isinstance(node,ast.Assign) and isinstance(node.value,ast.Call):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in ('TELEOP_ARM_KP','TELEOP_ARM_KD','TELEOP_ARM_TORQUE_LIMIT'):
                    gains[target.id]=ast.literal_eval(node.value.args[0])
    if len(gains)!=3:
        raise ValueError('could not resolve current operating-arm servo constants')
    qop=np.asarray(source_scene['randomization']['arm_joint_position_rad'][:6])
    qcap=np.asarray(source_scene['randomization']['capture_arm_joint_position_rad'])
    geometry=EndStereoGeometry.from_files(args.scene_model,args.camera_config)
    cp,cr=geometry.cameras_service(qcap)['left']
    camera_orbital=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    service_mount=camera_orbital@cr.T
    target_mount=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    initial_target_mount=target_mount.copy()
    if args.mode in {'pointing_handoff','continuous_original','vbar_original'}:
        axis=np.asarray(args.initial_target_axis,float)
        if not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-12:
            raise ValueError('finite nonzero target tilt axis required')
        axis/=np.linalg.norm(axis)
        x,y,z=axis
        skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        angle=np.deg2rad(args.initial_target_angle_deg)
        initial_target_mount=target_mount@(np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*skew@skew)
    sim=SimulationBaseClass.SimBaseClass();process=sim.CreateNewProcess('gnssDiagnostic')
    process.addTask(sim.CreateNewTask('gnssPhysics',2_500_000),30)
    process.addTask(sim.CreateNewTask('gnssInit',10**18),40)
    scene=mujoco.MJScene(ET.tostring(published,encoding='unicode'),files)
    sim.AddModelToTask('gnssInit',scene)
    ctx=AssemblyContext(sim,scene,config=SessionConfig(model_path=model_path,dynamics_backend='local',
        publish_stride=1,clock_rate_hz=400,physics_task='gnssPhysics'))
    physics=resolve_orbital_physics(source_scene)
    orbit=OrbitComponent(epoch_utc=source_scene['environment']['ephemeris_epoch_utc'],physics=physics)
    ctx.install([ClockComponent('gnssPhysics'),orbit])
    stepper=LocalMujocoStepper(scene,model_path,[],origin=orbit.origin,substeps=1,publish_every=1)
    sim.local_stepper=stepper
    ctx.ports.set_stepper(stepper);ctx.add(stepper,Slot.PHYSICS,component='local_dynamics')
    ctx.install([OrbitalGravityComponent(force_every=1)])
    retention_path=args.source_server/'config/plug_retention.json'
    retention=load_retention_parameters(retention_path) if retention_path.is_file() else None
    if retention is not None:
        ctx.install([PlugRetentionComponent(model_path=model_path,parameters=retention)])
    navigation_gravity=BskNavigationGravity(ctx.ports.orbital_environment())
    joints={j.get('name'):(body.get('name'),j) for body in original.iter('body') for j in body.findall('joint')
            if j.get('type')!='free'}
    references={f'joint{i}':float(qop[i-1]) for i in range(1,7)}
    references.update({f'bottom_joint{i}':float(qcap[i-1]) for i in range(1,7)})
    references.update(axial=.02,cam_angle=np.pi/3)
    references.update({f'jaw_slide_{i}':-.012 for i in range(4)})
    messages=[]
    for actuator in root.findall('./actuator/position'):
        name,joint=actuator.get('name'),actuator.get('joint')
        msg=messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=references.get(joint,0.)))
        scene.getSingleActuator(name).actuatorInMsg.subscribeTo(msg);messages.append(msg)
    controlled=[(f'joint{i}_position',f'joint{i}',kp,kd,limit) for i,kp,kd,limit in
                zip(range(1,7),gains['TELEOP_ARM_KP'],gains['TELEOP_ARM_KD'],gains['TELEOP_ARM_TORQUE_LIMIT'])]
    controlled += [(f'bottom_joint{i}_act',f'bottom_joint{i}',1500.,80.,80.) for i in range(1,7)]
    controlled += [('axial_act','axial',10000.,80.,15.),('cam_act','cam_angle',10.,1.,1.)]
    controlled += [(f'jaw_act_{i}',f'jaw_slide_{i}',8000.,40.,5.) for i in range(4)]
    refs=np.array([references.get(c[1],0.) for c in controlled])
    capture_state=dict(enabled=False,start_s=None,phase=None,failure=None,completed=False,
                       force_since=None,longest_force_s=0.,last_contact_log=None)
    capture_target=capture_servo=capture_audit=capture_log=gyro_log=None
    def held_or_capture_reference(t):
        if not capture_state['enabled']:
            return refs.copy(),np.zeros(len(refs))
        measured=np.array([float(joint_readers[j][0]().state) for _,j,_,_,_ in controlled[6:]])
        hits=capture_audit.record(t,capture_state['phase'] or 'waiting',measured)
        if hits:
            capture_state['failure']='service_self_contact'
        feedback=capture_audit.feedback
        if feedback['peak_target_contact_force_n']>10.:
            capture_state['failure']='target_contact_force_over_10n'
        elif feedback['pad_compression_m']>.002:
            capture_state['failure']='pad_compression_limit'
        elif feedback['jaw_force_n']>20.:
            capture_state['failure']='contact_force_limit'
        pg,rg=capture_target.kinematics.forward(measured[:6])
        if args.mode=='vbar_original':
            try:
                own_feedback=own_motion.update(round(t*1e9),ctx.extra['service_gyro'].latest,
                    own_mass.com({name:float(reader().state) for name,reader in own_readers.items()}),
                    epoch=(image_fixture.session,'continuous_vbar'))
                command,phase=capture_servo.command(t,measured,(pg,rg),capture_audit.feedback,
                                                   reference_from_base=own_feedback)
            except ValueError as error:
                command,phase=capture_servo.gyro_unavailable(str(error),t,measured,capture_audit.feedback)
        else:
            command,phase=capture_servo.command(t,measured,rg,capture_audit.feedback,grip_position_body_m=pg)
        capture_state['phase']=phase
        if capture_servo.sequence.phase=='abort':capture_state['failure']=capture_servo.sequence.failure_reason or 'controller_abort'
        if capture_state['failure']:command['deadman']=False
        positions,velocities=capture_target.step(.0025,command,False,measured)
        feedback=capture_audit.feedback
        supported=bool(capture_state['failure'] is None and capture_servo.sequence.phase=='hold_force' and feedback.get('jaw_support_count',0)>=4
                       and 4.5<=feedback.get('pad_force_n',0)<=5.5)
        if not supported:capture_state['force_since']=None
        elif capture_state['force_since'] is None:capture_state['force_since']=t
        duration=0. if capture_state['force_since'] is None else t-capture_state['force_since']
        capture_state['longest_force_s']=max(capture_state['longest_force_s'],duration)
        capture_state['completed']=capture_state['longest_force_s']>=(20. if args.mode=='vbar_original' else 10.)
        if capture_state['last_contact_log'] is None or t-capture_state['last_contact_log']>=.009999:
            capture_log.write(json.dumps(dict(time_s=t,phase=phase,feedback=feedback,
                source_frame_id=(capture_servo.last_packet or {}).get('source_frame_id'),
                failure=capture_state['failure'],four_jaw_force_streak_s=duration,
                prediction=(capture_servo.diagnostics() if args.mode=='vbar_original' else None),
                own_motion=(own_motion.diagnostics() if args.mode=='vbar_original' else None)))+'\n')
            capture_state['last_contact_log']=t
        return np.r_[refs[:6],positions],np.r_[np.zeros(6),velocities]
    publisher=HeldJointReferencePublisher(held_or_capture_reference,len(refs))
    for i,(act,joint,kp,kd,limit) in enumerate(controlled):
        ctx.ports.drive_servo(act,publisher.positionOutMsgs[i],publisher.velocityOutMsgs[i],owner='held_arms',
                             joint=joint,kp=kp,kd=kd,limit=limit)
    ctx.add(publisher,Slot.COMMAND,component='held_arms')
    # Resolve geom-derived inertias from the unchanged compiled model, only
    # in the analytic kinematics copy. The actual physics model stays intact.
    analytic=ET.fromstring(ET.tostring(root))
    for body in analytic.iter('body'):
        if body.find('inertial') is None:
            values=stepper.body_inertial(body.get('name'))
            ET.SubElement(body,'inertial',mass=str(values[0]),pos=' '.join(map(str,values[1:4])))
    centers=[CenterKinematics(analytic.find(f'./worldbody/body[@name="{n}"]')) for n in ('capture_target','cubesat_bus')]
    readers=[scene.getBody(n).getOrigin().stateOutMsg.addSubscriber() for n in ('capture_target','cubesat_bus')]
    joint_readers={name:(scene.getBody(body).getScalarJoint(name).stateOutMsg.addSubscriber(),
                         scene.getBody(body).getScalarJoint(name).stateDotOutMsg.addSubscriber())
                   for name,(body,j) in joints.items() if not name.startswith('rw_')}
    if args.mode=='vbar_original':
        from simulation.components.gyroscope import GyroscopeComponent
        from simulation.own_motion import EncoderMassModel,OwnMotionFeedback
        ctx.install([GyroscopeComponent(seed=args.seed)])
        own_mass=EncoderMassModel(model_path,stepper.body_inertial)
        own_motion=OwnMotionFeedback()
        gyro_log=(output/'service_gyro.jsonl').open('w',encoding='utf-8',buffering=1)
        own_readers={name:scene.getBody(body).getScalarJoint(name).stateOutMsg.addSubscriber()
                     for body,name in own_mass.joints}

        class ContinuousOwnMotion(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.ModelTag='continuousCaptureOwnMotion'

            def UpdateState(self,nanos):
                # Start at mission zero and consume every real 50 Hz sample.
                # Late grasp activation never resets/relabels the gyro stream.
                own_motion.update(int(nanos),ctx.extra['service_gyro'].latest,
                    own_mass.com({name:float(reader().state) for name,reader in own_readers.items()}),
                    epoch=(image_fixture.session if image_fixture else output.name,'continuous_vbar'))
                gyro_log.write(json.dumps(ctx.extra['service_gyro'].latest)+'\n')

        ctx.add(ContinuousOwnMotion(),Slot.RECORD,every=8,phase=0,component='continuous_own_motion')
    ct=centers[0].evaluate(references)[0];cb=centers[1].evaluate(references)[0]
    optical_offset=float((-target_mount@ct+service_mount@cb+target_mount@geometry.center-service_mount@cp)[0])
    optical_s=lambda depth:depth-optical_offset
    mass=sum(float(stepper.body_mass(b.get('name'))) for b in root.find('./worldbody/body[@name="cubesat_bus"]').iter('body'))
    nav=None;target_orbit=None;applied_force=np.zeros(3)
    force_msg=messaging.CmdForceInertialMsg().write(messaging.CmdForceInertialMsgPayload())
    force_torque_msg=messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    torque_msg=messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    ctx.ports.add_body_wrench('cubesat_bus',source='gnss_translation',force_inertial=force_msg,torque_body=force_torque_msg)
    ctx.ports.add_body_wrench('capture_target',source='cooperative_target',torque_body=torque_msg)
    panel_torque=messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    panel_reaction=messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    ctx.ports.add_body_wrench('satellite_outer_panel',source='hinge_hold',torque_body=panel_torque)
    ctx.ports.add_body_wrench('capture_target',source='hinge_reaction',torque_body=panel_reaction)
    def attitude_reference(t):
        Q,w=vbar_frame(target_orbit)
        return Q@service_mount,w,np.zeros(3)
    attitude=AttitudeControl(sim,process,scene,model_path,drive_task=('gnssPhysics',2000),reference_provider=attitude_reference)
    image_fixture=None;image_converter=None;image_estimator=None;image_gate=AcceptedImageGate()
    image_count=0;last_image_stamp=None;image_last=None;platform_since=None;platform_entered=None;platform_completed=None
    image_channels={};channel_last={};range_router=None;range_events=[];range_status=None
    platform_failure=None
    if args.image_channel:
        from tools.diagnostics.native_image_dynamics_fixture import NativeImageDynamicsFixture
        from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator
        near_config=json.loads(args.camera_config.read_text(encoding='utf-8-sig'))
        image_fixture=NativeImageDynamicsFixture(args.source_server,output,scene,orbit,near_config,
            long_focus_fov=args.long_focus_fov_deg if args.image_channel in {'long','dual'} else None,
            include_near=args.image_channel=='dual')
        image_geometry=EndStereoGeometry(args.scene_model,image_fixture.config)
        image_converter=ImageRelativeNavigation(image_geometry,image_fixture.session,
            allow_cooperative_attitude=args.cooperative_attitude_images)
        if args.cooperative_attitude_images:
            from space_arm_platform.cooperative_stereo_translation import CooperativeStereoTranslation
            image_estimator=CooperativeStereoTranslation(image_fixture.config['calibration'],image_fixture.config['layout'])
        else:
            image_estimator=MultiCameraPoseEstimator(image_fixture.config['calibration'],image_fixture.config['layout'])
        if args.mode=='vbar_original':
            from simulation.near23_navigation import Near23NavigationBridge
            body_image_bridge=Near23NavigationBridge(args.scene_model,image_fixture.config,
                joint_covariance_rad2=np.zeros((6,6)),mount_covariance=np.zeros((6,6)),
                uncertainty_source='declared ideal simulation encoders and mount; additional translation floor explicit')
        image_channels['far']=dict(geometry=image_geometry,converter=image_converter,estimator=image_estimator,
            gate=image_gate,sigma=args.image_position_sigma_m,aided=args.cooperative_attitude_images,last_stamp=None)
        if args.image_channel=='dual':
            from simulation.dual_range_vision import RangeChannelHandoff
            near_geometry=EndStereoGeometry(args.scene_model,near_config)
            image_channels['near']=dict(geometry=near_geometry,
                converter=ImageRelativeNavigation(near_geometry,image_fixture.session),
                estimator=MultiCameraPoseEstimator(near_config['calibration'],near_config['layout']),
                gate=AcceptedImageGate(),sigma=args.near_image_position_sigma_m,aided=False,last_stamp=None)
            range_router=RangeChannelHandoff()
        import cv2
        cv2.setNumThreads(args.vision_threads)
    sim.InitializeSimulation();sim.disableTask('gnssInit')
    rtarget,vtarget,_=FlightProfile().target(0.)
    target_orbit=np.r_[rtarget,vtarget]
    Q,omega=vbar_frame(target_orbit)
    initial_com=np.array([optical_s(args.depth_m),0.,0.])
    gp,gr=geometry.chain.forward(qcap)
    platform_com=(service_mount@(gp+gr@np.array([0.,0.,-args.platform_gap_m]))
                  -target_mount@geometry.center+target_mount@ct-service_mount@cb)
    if args.tool_gap_m is not None:
        gp,gr=geometry.chain.forward(qcap)
        initial_com=(service_mount@(gp+gr@np.array([0.,0.,-args.tool_gap_m]))
                     -target_mount@geometry.center+target_mount@ct-service_mount@cb)
    relative=Q@(initial_com-target_mount@ct+service_mount@cb)
    rservice=rtarget-relative;vservice=vtarget-np.cross(omega,relative)
    # Tilt about the assembled target COM, preserving the intended COM station.
    # This is initialization only; subsequent motion is torque/force integrated.
    target_origin_shift=Q@(target_mount-initial_target_mount)@ct
    initial_target_position=rtarget+target_origin_shift
    initial_target_velocity=vtarget+np.cross(omega,target_origin_shift)
    for body in root.findall('./worldbody/body'):
        name=body.get('name');obj=scene.getBody(name)
        if not obj.isFree():
            continue
        if name=='capture_target':
            p,v,R=initial_target_position,initial_target_velocity,Q@initial_target_mount
        elif name=='cubesat_bus':
            p,v,R=rservice,vservice,Q@service_mount
        else:
            offset=np.fromstring(body.get('pos','0 0 0'),sep=' ')
            R=Q@service_mount
            p=rservice+R@offset;v=vservice+np.cross(omega,R@offset)
        obj.setPosition(p.tolist());obj.setVelocity(v.tolist());obj.setAttitude(rbk.C2MRP(R.T).tolist())
        obj.setAttitudeRate((R.T@omega).tolist())
    for name,value in references.items():
        if name in joints:
            body,j=joints[name];scene.getBody(body).getScalarJoint(name).setPosition(float(value))
            scene.getBody(body).getScalarJoint(name).setVelocity(0.)
    orbit.origin.set_state(rtarget,vtarget)
    scene.postIntegration(0);scene.UpdateState(0)
    sensor=DifferentialGNSS(GNSSConfig(),seed=args.seed)
    vbar=None
    if args.mode=='guarded_approach':
        # Historical boundary-only diagnostic starts after the far nodes.
        # The vbar_original route creates its own completely uncompleted gates.
        vbar=VbarApproach()
        vbar.initial=np.r_[initial_com,np.zeros(3)];vbar.acquire.completed_at_s=0.
        vbar.gates['Vbar50m'].completed_at_s=0.;vbar.index=1
        vbar.targets=[np.array([d,0.,0.]) for d in (50.,optical_s(17.4),optical_s(5.5),optical_s(.5))]
        vbar.next_start=initial_com[0];vbar.offset_end_s=optical_s(2.)
    diagnostic_leg=RestLeg(initial_com[0],optical_s(args.end_depth_m),P.close_speed_m_s)
    diagnostic_stop=None;diagnostic_stop_start=None;diagnostic_failure=None
    handoff=(PointingHandoff(initial_com,[optical_s(args.pointing_depth_m),0.,0.],
        PointingHandoffConfig(arrival_tolerance_m=args.station_tolerance_m,visual_dwell_s=args.visual_dwell_s),
        end_com=[optical_s(args.end_depth_m),0.,0.],
        gnss_diagnostic_continuation=args.pointing_gnss_continuation)
        if args.mode=='pointing_handoff' else None)
    if continuous_mode:
        if args.mode=='vbar_original':
            from simulation.vbar_stereo_sequence import VbarStereoSequence
        sequence_type=VbarStereoSequence if args.mode=='vbar_original' else OriginalStereoSequence
        handoff=sequence_type(initial_com,[optical_s(args.pointing_depth_m),0.,0.],
            [optical_s(5.5),0.,0.],[optical_s(args.near_checkpoint_depth_m),0.,0.],platform_com,
            gnss_bridge_diagnostic=args.gnss_bridge_diagnostic,tool_gap_m=args.platform_gap_m,
            with_grasp=args.with_grasp)
    capture_target_mount=visual_hold_mount=None
    prealignment=PrealignmentController(geometry)
    alignment_result=None;small_motion_since=None;safe_target_mount=None
    target_reference=Q@initial_target_mount;target_reference_rate=omega
    navigation_process_sigma=.003
    origin_prediction=OriginLeverArm() if args.kinematic_origin_prediction else None
    parent={b.get('name'):p.get('name') for p in root.iter('body') for b in p.findall('body')}
    def family(name):
        while name in parent:
            name=parent[name]
        return name
    families={stepper._lib.lms_name2id(stepper._handle,1,b.get('name').encode()):family(b.get('name')) for b in root.iter('body')}
    rows=[];last_raw=None;last_gnss=0.;first_fault=None;start=time.monotonic()
    trace=(output/'trajectory.jsonl').open('w',encoding='utf-8',buffering=1)
    try:
        if image_fixture is not None:
            image_fixture.start(image_fixture.render_frame(0))
            image_fixture.send(image_fixture.render_frame(0))  # discard renderer settling capture
        for k in range(round(args.duration_s*20)+1):
            now=k*.05;nanos=k*50_000_000
            sim.ConfigureStopTime(nanos);sim.ExecuteSimulation()
            if any(r.timeWritten()!=nanos for r in readers):
                raise RuntimeError('physical-state messages are not at scoring epoch')
            truth=np.array([np.r_[p.r_BN_N,p.v_BN_N] for p in [r() for r in readers]])
            states=[r() for r in readers]
            rt,rb=[rbk.MRP2C(s.sigma_BN).T for s in states]
            pos={j:float(r[0]().state) for j,r in joint_readers.items()}
            rate={j:float(r[1]().state) for j,r in joint_readers.items()}
            ctr, cbr=centers[0].evaluate(pos,rate),centers[1].evaluate(pos,rate)
            origin_acceleration=np.zeros(3)
            if origin_prediction is not None:
                origin_acceleration=origin_prediction.update(now,navigation_gravity.gradient(target_orbit[:3],max(0.,now-.05)),
                    target_rotation=rt,service_rotation=rb,target_com=ctr[0],service_com=cbr[0],
                    target_com_rate=ctr[1],service_com_rate=cbr[1],
                    target_omega=states[0].omega_BN_B,service_omega=states[1].omega_BN_B)
            if nav is not None:
                G=navigation_gravity.gradient(target_orbit[:3],now-.05)
                nav.predict(.05,-applied_force/mass+origin_acceleration,G)
                acc=navigation_gravity.acceleration(target_orbit[:3],now-.05)
                target_orbit[:3]+=.05*target_orbit[3:]+.5*.05**2*acc
                target_orbit[3:]+=.05*acc
            if k%2==0:
                packet=sensor.measure(truth,now)
                last_raw=packet['relative'][:3]-(truth[0,:3]-truth[1,:3])
                if nav is None:
                    nav=RelativeEKF(packet['relative'],packet['relative_covariance'],acceleration_sigma_m_s2=.003)
                    target_orbit=packet['observations'][0].copy();last_gnss=now
                elif nav.update(dict(source='gnss',stamp=now,z=packet['relative'],R=packet['relative_covariance']),now):
                    last_gnss=now;target_orbit=packet['observations'][0].copy()
            Q,w=vbar_frame(target_orbit)
            com_state,com_cov=center_relative(nav.x,nav.P,Q,w,rt,rb,states[0].omega_BN_B,states[1].omega_BN_B,ctr,cbr)
            measured_q=np.array([pos[f'bottom_joint{i}'] for i in range(1,7)])
            measured_qdot=np.array([rate[f'bottom_joint{i}'] for i in range(1,7)])
            image_update=None
            sample_images=(not args.images_after_prealignment or handoff is None
                           or handoff.phase in {'VisualWait','VisualContinue','TerminalHold'})
            if continuous_mode:
                # Far snapshots document the actual lack of readable pixels.
                # Only the near stream can establish a 10 Hz readiness streak.
                if capture_state['enabled'] or (handoff.image_release_s is not None):
                    sample_images=True
                else:
                    near_depth=float((geometry.cameras_service(measured_q)['left'][1].T@rb.T@
                        (nav.x[:3]+rt@geometry.center-rb@geometry.cameras_service(measured_q)['left'][0]))[2])
                    sample_images=(near_depth<=1.65 or (near_depth<=3.5 and k%20==0) or k%200==0)
            if image_fixture is not None and k>0 and k%2==0 and not sample_images:
                image_fixture.send(image_fixture.render_frame(nanos),capture=False)
            if image_fixture is not None and k>0 and k%2==0 and sample_images:
                packet,blobs=image_fixture.send(image_fixture.render_frame(nanos))
                captures={'far':(packet,blobs)}
                if range_router is not None:captures['near']=image_fixture.last_near_capture
                updates={}
                for name,(packet,blobs) in captures.items():
                    channel=image_channels[name]
                    if channel['aided']:
                        _,camera_rotation=channel['geometry'].cameras_service(measured_q)['left']
                        image_result=channel['estimator'].estimate_aided(blobs['left'],blobs['right'],
                            rotation_target_to_left=camera_rotation.T@rb.T@rt,
                            attitude_stamp_ns=nanos,image_stamp_ns=nanos)
                        chosen=image_result['measurement']
                    else:
                        image_result=channel['estimator'].estimate(blobs['left'],blobs['right'])
                        chosen=image_result['methods']['joint_multicamera']
                    packet['measurement']=chosen
                    if continuous_mode:
                        (output/'measurement.json').write_text(json.dumps(packet),encoding='utf-8')
                    body_packet=None
                    if args.mode=='vbar_original':
                        from space_arm_platform.stereo_capture_packet import capture_packet,publish_packet
                        capture_observation=capture_packet(chosen,near_config,session_id=image_fixture.session,
                            source_frame_id=packet['source_frame_id'],sim_time_ns=nanos)
                        publish_packet(output/'capture_measurement.json',capture_observation)
                        body_packet=body_image_bridge.packet(chosen,mission_id=output.name,
                            session_id=image_fixture.session,source_frame_id=packet['source_frame_id'],
                            sim_time_ns=nanos,encoder_time_ns=nanos,joint_position_rad=measured_q)
                    used_edges={}
                    for eye in ('left','right'):
                        ids=chosen.get('used_ids',{}).get(eye,[])
                        corners=image_result['detected_corners_px'][eye]
                        edges=[float(np.min(np.linalg.norm(np.array(corners[str(i)])-
                                   np.roll(np.array(corners[str(i)]),1,axis=0),axis=1))) for i in ids]
                        used_edges[eye]=max(edges) if edges else 0.
                    accepted=False;update=None;conversion_error=None
                    if chosen['status']=='ok':
                        try:
                            if args.mode=='vbar_original':
                                if body_packet['status']!='ok':raise ValueError(body_packet['reason'])
                                # The legacy six-state diagnostic explicitly uses
                                # ideal cooperative attitude telemetry. The body
                                # packet itself is independent of this attitude.
                                body_cov=np.array(body_packet['pose_covariance']['matrix'])[:3,:3]
                                update=dict(source='stereo',stamp=now,
                                    z=rb@np.array(body_packet['pose_target_to_service']['t_m']),
                                    R=rb@body_cov@rb.T+np.eye(3)*channel['sigma']**2)
                            else:
                                update=channel['converter'].convert(packet,now_ns=nanos,joint_position_rad=measured_q,
                                    kinematics_stamp_ns=nanos,service_rotation_n_from_body=rb,service_attitude_stamp_ns=nanos,
                                    position_covariance_n_m2=np.eye(3)*channel['sigma']**2)
                            # Both pairs update during qualification. Once switched,
                            # the long-focus pair is monitored but never fused again.
                            if range_router is None or range_router.active=='far' or name=='near':
                                accepted=nav.update(update,now)
                        except ValueError as error:
                            conversion_error=str(error)
                    image_count+=1;updates[name]=update
                    depth_sigma=channel['sigma']
                    if args.mode=='vbar_original' and body_packet.get('status')=='ok':
                        depth_sigma=float(np.hypot(depth_sigma,body_packet['optical_depth_sigma_m']))
                    channel['gate'].observe(now_s=now,stamp_s=now,frame_id=packet['source_frame_id'],
                        image_accepted=chosen['status']=='ok',ekf_accepted=accepted,used_marker_edges_px=used_edges,
                        depth_sigma_m=depth_sigma,navigation_sigma_m=nav.sigma_m)
                    if accepted:channel['last_stamp']=now
                    channel_last[name]=dict(frame_id=packet['source_frame_id'],time_s=now,status=chosen['status'],route=chosen['route'],
                        reason=chosen['reason'],ekf_accepted=accepted,conversion_error=conversion_error,
                        used_marker_edges_px=used_edges,used_ids=chosen.get('used_ids'),
                        reprojection_error_px=chosen.get('reprojection_error_px'),ready_count=channel['gate'].count,
                        body_navigation=body_packet,
                        position_error_mm=(None if update is None else float(np.linalg.norm(update['z']-(truth[0,:3]-truth[1,:3]))*1000)))
                if range_router is not None:
                    previous=range_router.active
                    range_status=range_router.update(**{n+'_ready':c['gate'].current(now,nav.sigma_m) for n,c in image_channels.items()})
                    if range_router.active!=previous:
                        range_events.append(dict(time_s=now,active=range_router.active))
                        print(json.dumps(dict(time_s=now,range_channel=range_router.active)),flush=True)
                active=range_router.active if range_router else 'far'
                image_last=channel_last[active];image_update=updates[active]
                last_image_stamp=image_channels[active]['last_stamp']
                (output/'image_progress.json').write_text(json.dumps(dict(time_s=now,last=image_last,
                    navigation_accepted=nav.accepted,channels=channel_last,range_status=range_status),indent=2),encoding='utf-8')
                com_state,com_cov=center_relative(nav.x,nav.P,Q,w,rt,rb,states[0].omega_BN_B,
                                                states[1].omega_BN_B,ctr,cbr)
            visual_ready=image_fixture is not None and image_channels[range_router.active if range_router else 'far']['gate'].current(now,nav.sigma_m)
            cam_p,cam_r=geometry.cameras_service(measured_q)['left']
            mechanism_n=nav.x[:3]+rt@geometry.center
            depth=float((cam_r.T@rb.T@(mechanism_n-rb@cam_p))[2])
            if handoff is not None:
                if visual_hold_mount is None:
                    target_reference,target_reference_rate=geometry.reference_motion(
                        relative_position_n_m=nav.x[:3],relative_velocity_n_m_s=nav.x[3:],
                        service_rotation_n_from_body=rb,service_angular_rate_body_rad_s=states[1].omega_BN_B,
                        joint_position_rad=measured_q,joint_velocity_rad_s=measured_qdot)
                else:
                    target_reference,target_reference_rate=Q@visual_hold_mount,w
                alignment_result=prealignment.update(now_s=now,
                    input_stamps_s=dict(navigation=last_gnss,service_attitude=now,
                                        target_telemetry=now,camera_kinematics=now),
                    relative_position_n_m=nav.x[:3],navigation_position_sigma_m=nav.sigma_m,
                    service_rotation_n_from_body=rb,target_rotation_n_from_body=rt,
                    target_angular_rate_body_rad_s=states[0].omega_BN_B,
                    reference_angular_rate_n_rad_s=target_reference_rate,joint_position_rad=measured_q,
                    enabled=handoff.target_pointing_enabled,
                    reference_rotation_n_from_body=(target_reference if visual_hold_mount is not None else None))
                sigma_perp=np.sqrt(max(0.,np.linalg.eigvalsh(com_cov[1:3,1:3]).max()))
                width=min(P.corridor_max_m,max(P.corridor_min_m,P.corridor_slope*com_state[0]))
                corridor_margin=width-np.linalg.norm(com_state[1:3])-3*sigma_perp
                service_error=float(np.arccos(np.clip((np.trace(rb.T@Q@service_mount)-1)/2,-1,1)))
                previous_phase=handoff.phase
                extra={}
                if continuous_mode:
                    pg,rg=geometry.chain.forward(measured_q)
                    extra=dict(estimated_tool_center_gripper_m=rg.T@(rb.T@mechanism_n-pg),
                               grasp_complete=capture_state['completed'],grasp_failure=capture_state['failure'])
                if args.mode=='vbar_original':
                    extra.update(covariance=com_cov,optical_depth_m=depth,
                        available_acceleration_m_s2=min(.005,25./mass),observation_age_s=now-last_gnss)
                ref,refv,refa=handoff.update(now,com_state,position_sigma_m=nav.sigma_m,
                    inputs_valid=now-last_gnss<=.35,service_ready=service_error<np.deg2rad(1.),
                    corridor_ready=corridor_margin>0,prealigned=alignment_result.aligned,
                    visual_ready=visual_ready,visual_stamp_s=last_image_stamp,**extra)
                if continuous_mode and handoff.image_release_s is not None and visual_hold_mount is None:
                    # The far pre-alignment solver deliberately supports >=1m
                    # origin-to-camera separation. After stationary real-image
                    # admission, torque-hold its established orbital attitude.
                    # Keep checking actual view and images during translation.
                    visual_hold_mount=Q.T@target_reference
                if continuous_mode and handoff.phase=='Grasp' and not capture_state['enabled']:
                    from simulation.body_stereo_capture import BodyStereoCaptureServo
                    from simulation.capture_arm_target import CaptureArmTarget
                    from simulation.capture_audit import CaptureAudit
                    capture_target=CaptureArmTarget(model_path)
                    capture_target.position=np.array([pos[j] for _,j,_,_,_ in controlled[6:]])
                    if args.mode=='vbar_original':
                        from simulation.stereo_predictive_capture import StereoPredictiveCapture
                        capture_servo=StereoPredictiveCapture(output/'capture_measurement.json',image_fixture.session,
                                                              config_file=args.camera_config)
                    else:
                        capture_servo=BodyStereoCaptureServo(output/'measurement.json',image_fixture.session,near_config)
                    capture_audit=CaptureAudit(model_path,stepper,output_path=output/'contact_audit.jsonl')
                    capture_log=(output/'grasp_feedback.jsonl').open('w',encoding='utf-8',buffering=1)
                    capture_target_mount=visual_hold_mount.copy()
                    capture_state.update(enabled=True,start_s=now)
                policy=dict(handoff.telemetry,corridor_margin_m=float(corridor_margin))
                if capture_state['enabled'] and handoff.failure and capture_state['failure'] is None:
                    capture_state['failure']='platform:'+handoff.failure
                if previous_phase!=handoff.phase:
                    print(json.dumps(dict(time_s=now,handoff_event=handoff.phase)),flush=True)
                small_motion=(alignment_result.attitude_error_deg is not None
                    and alignment_result.attitude_error_deg<1.
                    and alignment_result.rate_error_rad_s<.001)
                if not small_motion:
                    small_motion_since=None
                elif small_motion_since is None:
                    small_motion_since=now
                # A slewing body origin accelerates about its COM. Inflate the
                # unmodeled-acceleration process noise, without resetting x/P.
                navigation_process_sigma=(.05 if origin_prediction is None and handoff.target_pointing_enabled
                    and (small_motion_since is None or now-small_motion_since<2.) else .003)
                nav.acceleration_sigma_m_s2=navigation_process_sigma
            elif args.mode=='guarded_approach':
                ref,refv,refa=vbar.update(now,com_state,com_cov,common=True,sigma=nav.sigma_m,
                    visual_valid=False,visual_ready=False,optical_depth=depth,optical_s=optical_s,
                    hold=np.array([optical_s(.5),0,0]),offset_end_s=optical_s(2.),geometry_ready=True,
                    available_acceleration=.005,observation_age=now-last_gnss)
                policy=vbar.telemetry
            elif args.mode=='hold':
                ref=initial_com.copy();refv=np.zeros(3);refa=np.zeros(3)
                policy=dict(subsegment='initialized_gnss_hold',failure_reason=None)
            else:
                # Explicit user-authorized counterfactual diagnostic only.
                # Formal mission VbarApproach/parameters are never edited.
                progress=diagnostic_leg.sample(max(0.,now-20.))
                ref=np.array([progress[0],0.,0.]);refv=np.array([progress[1],0.,0.]);refa=np.array([progress[2],0.,0.])
                sigma_perp=np.sqrt(max(0.,np.linalg.eigvalsh(com_cov[1:3,1:3]).max()))
                width=min(P.corridor_max_m,max(P.corridor_min_m,P.corridor_slope*com_state[0]))
                corridor_margin=width-np.linalg.norm(com_state[1:3])-3*sigma_perp
                pointing_error=float(np.arccos(np.clip((np.trace(rb.T@Q@service_mount)-1)/2,-1,1)))
                if diagnostic_failure is None and (corridor_margin<=0 or now-last_gnss>.5 or pointing_error>np.deg2rad(1.)):
                    diagnostic_failure='diagnostic_corridor_link_or_pointing'
                    diagnostic_stop=StopReference(com_state);diagnostic_stop_start=now
                if diagnostic_failure:
                    ref,refv,refa=diagnostic_stop.sample(now-diagnostic_stop_start)
                policy=dict(subsegment='diagnostic_gnss_leg' if now<20+diagnostic_leg.duration else 'diagnostic_gnss_terminal_hold',
                    failure_reason=diagnostic_failure,formal_visual_policy_enabled=False,
                    corridor_margin_m=float(corridor_margin),end_optical_depth_m=args.end_depth_m)
            command=refa-P.kp_s2*(com_state[:3]-ref)-P.kd_s*(com_state[3:]-refv)
            com_delta_n=Q@com_state[:3];com_v_n=Q@com_state[3:]
            G=navigation_gravity.gradient(target_orbit[:3],now)
            omega_dot=-2*(target_orbit[:3]@target_orbit[3:])/(target_orbit[:3]@target_orbit[:3])*w
            acceleration=G@com_delta_n-Q@command-rotating_acceleration(w,com_delta_n,com_v_n)-np.cross(omega_dot,com_delta_n)
            applied_force=mass*acceleration
            applied_force*=min(1.,P.force_limit_n/max(np.linalg.norm(applied_force),1e-15))
            force_msg.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=applied_force.tolist()),nanos)
            # Equivalent application at the whole assembly COM, as in the
            # production near-field force-site model, instead of bus-body COM.
            moment=np.cross(cbr[0]-stepper.body_inertial('cubesat_bus')[1:4],rb.T@applied_force)
            force_torque_msg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=moment.tolist()),nanos)
            hinge_torque=float(np.clip(-.1*pos['outer_panel_hinge']-.1*rate['outer_panel_hinge'],-.02,.02))
            panel_torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0.,0.,hinge_torque]),nanos)
            panel_reaction.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0.,0.,-hinge_torque]),nanos)
            if capture_state['enabled']:
                target_reference=Q@capture_target_mount;target_reference_rate=w
                torque=bounded_pointing_torque(target_rotation_n_from_body=rt,reference_rotation_n_from_body=target_reference,
                    target_angular_rate_body_rad_s=states[0].omega_BN_B,reference_angular_rate_n_rad_s=target_reference_rate)
            elif handoff is not None and handoff.target_pointing_enabled:
                torque=alignment_result.torque_body_nm
            else:
                if handoff is not None and handoff.phase=='SafeHold' and safe_target_mount is None:
                    safe_target_mount=Q.T@rt
                held_mount=(safe_target_mount if safe_target_mount is not None else initial_target_mount)
                target_reference=Q@held_mount;target_reference_rate=w
                torque=bounded_pointing_torque(target_rotation_n_from_body=rt,reference_rotation_n_from_body=target_reference,
                    target_angular_rate_body_rad_s=states[0].omega_BN_B,reference_angular_rate_n_rad_s=target_reference_rate)
            torque_msg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=torque.tolist()),nanos)
            # Separate truth-only scoring below. It never changes nav or control.
            delta=truth[0,:3]-truth[1,:3]
            true_com=Q.T@(delta+rt@ctr[0]-rb@cbr[0])
            true_com_v=Q.T@(truth[0,3:]-truth[1,3:]
                +rt@(np.cross(states[0].omega_BN_B,ctr[0])+ctr[1])
                -rb@(np.cross(states[1].omega_BN_B,cbr[0])+cbr[1])
                -np.cross(w,Q@true_com))
            true_depth=float((cam_r.T@rb.T@(delta+rt@geometry.center-rb@cam_p))[2])
            contacts=[c.tolist() for c in stepper.contact_snapshot()
                      if {families.get(int(c[2])),families.get(int(c[3]))}=={'cubesat_bus','capture_target'}]
            pg,rg=geometry.chain.forward(measured_q)
            true_center_grip=rg.T@(rb.T@(delta+rt@geometry.center)-pg)
            estimated_center_grip=rg.T@(rb.T@mechanism_n-pg)
            platform_ok=(args.tool_gap_m is not None and visual_ready
                and np.linalg.norm(estimated_center_grip-[0,0,-args.tool_gap_m])<.006
                and abs(-estimated_center_grip[2]-args.tool_gap_m)<.006
                and np.linalg.norm(com_state[3:])<.002 and nav.sigma_m<.04
                and np.rad2deg(np.arccos(np.clip((np.trace(rb.T@Q@service_mount)-1)/2,-1,1)))<1.)
            if not platform_ok:
                platform_since=None
                if platform_entered is not None and platform_failure is None:
                    platform_failure=dict(time_s=now,reason='platform_quality_lost_after_entry')
            elif platform_since is None:
                platform_since=now
            if platform_ok and platform_failure is None and now-platform_since>=5.:
                if platform_entered is None:platform_entered=now;platform_since=now
                elif platform_completed is None:platform_completed=now
            row=dict(time_s=now,estimated_optical_depth_m=depth,true_optical_depth_m=true_depth,
                truth_relative_origin_eci_m=delta.tolist(),estimated_relative_origin_eci_m=nav.x[:3].tolist(),
                filtered_position_error_xyz_mm=((nav.x[:3]-delta)*1000).tolist(),
                raw_position_error_xyz_mm=(last_raw*1000).tolist() if k%2==0 else None,
                true_body_origin_distance_m=float(np.linalg.norm(delta)),true_com_distance_m=float(np.linalg.norm(true_com)),
                true_gripper_to_mechanism_m=float(np.linalg.norm(true_center_grip)),
                true_center_gripper_m=true_center_grip.tolist(),estimated_center_gripper_m=estimated_center_grip.tolist(),
                true_signed_tool_gap_m=float(-true_center_grip[2]),
                platform_position_error_mm=(None if args.tool_gap_m is None else
                    float(np.linalg.norm(true_center_grip-[0,0,-args.tool_gap_m])*1000)),
                platform_gap_error_mm=(None if args.tool_gap_m is None else
                    float(abs(-true_center_grip[2]-args.tool_gap_m)*1000)),
                visual_ready=visual_ready,image=image_last,image_channels=channel_last.copy(),
                active_image_channel=range_router.active if range_router else None,
                range_status=range_status,
                image_position_error_mm=(None if image_update is None else
                    float(np.linalg.norm(image_update['z']-delta)*1000)),
                platform_estimated_qualified=bool(platform_ok),platform_entered_s=platform_entered,
                platform_completed_s=platform_completed,
                platform_failure=platform_failure,
                filtered_position_error_mm=float(np.linalg.norm(nav.x[:3]-delta)*1000),
                mechanism_position_error_mm=float(np.linalg.norm(estimated_center_grip-true_center_grip)*1000),
                raw_position_error_mm=float(np.linalg.norm(last_raw)*1000) if k%2==0 else None,
                true_tracking_error_mm=float(np.linalg.norm(true_com-ref)*1000),
                estimated_tracking_error_mm=float(np.linalg.norm(com_state[:3]-ref)*1000),
                nav_position_sigma_m=nav.sigma_m,nav_rejected=nav.rejected,
                estimated_rotating_speed_m_s=float(np.linalg.norm(com_state[3:])),
                reference_com_m=ref.tolist(),applied_force_n=applied_force.tolist(),
                estimated_com_state=com_state.tolist(),true_com_position_m=true_com.tolist(),
                true_com_velocity_m_s=true_com_v.tolist(),
                target_torque_body_nm=torque.tolist(),nav_acceleration_sigma_m_s2=navigation_process_sigma,
                kinematic_origin_acceleration_n_m_s2=origin_acceleration.tolist(),
                prealignment=(None if alignment_result is None else dict(aligned=alignment_result.aligned,
                    reason=alignment_result.reason,dwell_s=alignment_result.dwell_s,
                    attitude_error_deg=alignment_result.attitude_error_deg,
                    rate_error_rad_s=alignment_result.rate_error_rad_s,view=alignment_result.view)),
                true_target_rotation_n_from_body=rt.tolist(),reference_target_rotation_n_from_body=target_reference.tolist(),
                true_target_reference_error_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(rt.T@target_reference)-1)/2,-1,1)))),
                true_target_reference_rate_error_rad_s=float(np.linalg.norm(rt@states[0].omega_BN_B-target_reference_rate)),
                target_contact_pairs=contacts,policy=policy,
                capture=(dict(capture_state,feedback=capture_audit.feedback) if capture_state['enabled'] else None),
                target_attitude_error_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(rt.T@Q@target_mount)-1)/2,-1,1)))),
                service_attitude_error_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(rb.T@Q@service_mount)-1)/2,-1,1)))))
            trace.write(json.dumps(row)+'\n');rows.append(row)
            if (output/'stop_requested').exists():
                break
            if k%200==0:
                print(json.dumps({key:row[key] for key in ('time_s','true_optical_depth_m','filtered_position_error_mm','service_attitude_error_deg')}),flush=True)
            if contacts and not capture_state['enabled']:
                break
            if continuous_mode and handoff.phase=='Complete':
                break
            if capture_state['enabled'] and now-capture_state['start_s']>300.:
                capture_state['failure']='grasp_trial_timeout'
            if policy.get('failure_reason') and first_fault is None:
                first_fault=now
            if first_fault is not None and now-first_fault>=20.:
                break
            if args.mode=='gnss_leg' and now>=20+diagnostic_leg.duration+30.:
                break
            if handoff is not None and handoff.phase=='TerminalHold' and now-handoff.phase_start>=args.terminal_hold_s:
                break
    except Exception as error:
        (output/'failure.json').write_text(json.dumps(dict(exception_type=type(error).__name__,
            reason=str(error),last_completed_sample_s=(rows[-1]['time_s'] if rows else None),
            last_phase=(rows[-1]['policy']['phase'] if rows else None),
            requested_stage_completed=False),indent=2),encoding='utf-8')
        raise
    finally:
        trace.close();sim.TotalSim.deleteThreads()
        if image_fixture is not None:image_fixture.close()
        if capture_log is not None:capture_log.close()
        if gyro_log is not None:gyro_log.close()
        if capture_audit is not None:capture_audit.file.close()
        if capture_audit is not None and capture_audit.force_file is not None:capture_audit.force_file.close()
    warm=[r for r in rows if r['time_s']>=20.]
    physical_valid=bool(rows) and (args.with_grasp or not any(r['target_contact_pairs'] for r in rows)) \
        and stepper.external_writes==0 and max(r['service_attitude_error_deg'] for r in rows)<1.
    summary=dict(scope=__doc__,mode=args.mode,initial_optical_depth_m=args.depth_m,seed=args.seed,
        requested_terminal_optical_depth_m=args.end_depth_m if args.mode in {'gnss_leg','pointing_handoff'} else None,
        formal_visual_boundary_policy_applied=args.mode=='guarded_approach',
        source_model=str(args.scene_model.resolve()),source_model_sha256=hashlib.sha256(args.scene_model.read_bytes()).hexdigest(),
        source_scene=str(args.source_scene.resolve()),camera_config_sha256=hashlib.sha256(args.camera_config.read_bytes()).hexdigest(),
        entry_source_sha256=hashlib.sha256(entry_source.read_bytes()).hexdigest(),operating_arm_servo=gains,
        basilisk_path=str(Path(Basilisk.__file__).resolve()),mujoco_runtime=str(runtime_path),mujoco_version=runtime.mj_version(),
        physics_hz=400,control_hz=20,gnss=GNSSConfig().to_dict(),vision_updates=nav.accepted['stereo'],
        image_channel=args.image_channel,image_position_sigma_assumption_m=args.image_position_sigma_m,
        image_uses_cooperative_attitude=args.cooperative_attitude_images,
        navigation_origin_rotation_compensated=args.kinematic_origin_prediction,
        range_transfer=(None if range_router is None else dict(events=range_events,final_channel=range_router.active,
            near_position_sigma_assumption_m=args.near_image_position_sigma_m,
            independent_image_noise_during_overlap_assumed=True,
            far_observations_fused_after_switch=False)),
        images_acquired=image_count,tool_gap_reference_m=args.tool_gap_m,
        platform_entered_s=platform_entered,platform_completed_s=platform_completed,
        platform_failure=platform_failure,
        image_capture_scope='held physics for images; realtime latency not validated' if image_fixture else None,
        orbital_physics=physics.model_dump(mode='json'),mass_kg=mass,
        fixed_operating_arm_rad=qop.tolist(),fixed_capture_arm_rad=qcap.tolist(),
        assumptions=['current saved arm positions held by production servo gains',
            'service initial attitude chosen so actual left optical axis lies along V-bar',
            ('target starts tilted; torque slew enabled only after stationary arrival' if handoff else
             'target marker face initially aligned'),
            'ideal cooperative target attitude/rate, service attitude/rate and exact encoders',
            'current saved scene point-mass Earth and Sun; no added drag/SRP/random disturbances',
            '25 N equivalent translational force, not nozzle hardware allocation',
            'no continuation of accepted 110 m far-flight handoff',
            'diagnostic admission uses saved nonzero arm pose; not original mission authored-zero gate',
            'initialized hold does not execute the original visual boundary policy'],
        source_retention_config_present=retention_path.is_file(),
        gnss_accepted=nav.accepted,gnss_rejection_reasons=nav.rejection_reasons,
        elapsed_sim_s=rows[-1]['time_s'],wall_s=time.monotonic()-start,
        all_samples=statistics(rows),after20s=statistics(warm),
        minimum_true_optical_depth_m=min(r['true_optical_depth_m'] for r in rows),
        terminal=rows[-1],stepper=stepper.telemetry(),
        physical_run_valid=physical_valid,
        position_accuracy=dict(tolerance_mm=args.tolerance_mm,
            all_samples_below_tolerance=all(r['filtered_position_error_mm']<args.tolerance_mm for r in rows),
            after20s_below_tolerance=bool(warm) and all(r['filtered_position_error_mm']<args.tolerance_mm for r in warm)),
        closest_safe_distance_established=False,complete_mission_validated=False)
    if continuous_mode:
        summary['continuous_original_stereo']=dict(
            diagnostic_gnss_bridge=args.gnss_bridge_diagnostic,
            changed_trial_requirements=['GNSS may cross original 12m visual boundary',
                '5.5m trial dwell is navigation-only; real visual readiness required at near checkpoint'],
            production_policy_unchanged=True,one_physics_and_navigation_history=True,
            initialized_at_optical_depth_m=args.depth_m,far_110m_handoff_inherited=False,
            near_checkpoint_depth_m=args.near_checkpoint_depth_m,platform_gap_m=args.platform_gap_m,
            target_orbital_attitude_held_after_visual_release=True,
            platform_com_reference_m=platform_com.tolist(),
            image_release_s=handoff.image_release_s,platform_entered_s=handoff.platform_entered_s,
            platform_completed_s=handoff.platform_completed_s,capture=capture_state)
    if args.mode=='vbar_original':
        summary['vbar_integration']=dict(reference_owner=handoff.owner,
            route='vbar_owns_all_platform_references_v2',
            station17_completed_s=handoff.handoff_s,station17_evidence=handoff.handoff_evidence,
            vbar_state=handoff.vbar.telemetry,
            own_motion=own_motion.diagnostics(),
            capture_controller='near23 stereo + shared gyro/encoders' if args.with_grasp else None,
            scope='actual V-bar from initialized near entry; not an inherited far-flight handoff')
    if handoff is not None:
        pointing_rows=[r for r in rows if r['policy']['phase']=='TargetPointing']
        aligned_rows=[r for r in rows if r['prealignment']['aligned']]
        summary['pointing_handoff']=dict(events=handoff.events,final_phase=handoff.phase,
            failure_reason=handoff.failure,station_nominal_optical_depth_m=args.pointing_depth_m,
            initial_target_tilt_deg=args.initial_target_angle_deg,initial_target_tilt_axis=args.initial_target_axis,
            station_com_reference_m=handoff.station.tolist(),
            gnss_diagnostic_continuation=args.pointing_gnss_continuation,
            arrival_tolerance_m=args.station_tolerance_m,visual_dwell_s=args.visual_dwell_s,
            formal_visual_release=any(r['policy']['formal_visual_release'] for r in rows),
            first_prealigned=next(iter(aligned_rows),None),
            during_target_pointing=statistics(pointing_rows),
            maximum_target_torque_nm=max(np.linalg.norm(r['target_torque_body_nm']) for r in rows),
            alignment_achieved=bool(aligned_rows),
            sequence_awaited_vision=any(e['phase']=='VisualWait' for e in handoff.events))
        if continuous_mode:
            summary['requested_stage_completed']=bool(physical_valid and handoff.failure is None
                and handoff.platform_completed_s is not None
                and (handoff.phase=='Complete' if args.with_grasp else handoff.phase=='TerminalHold'))
        else:
            summary['requested_stage_completed']=bool(physical_valid and handoff.phase=='TerminalHold'
                and handoff.failure is None and (not args.image_channel or summary['pointing_handoff']['formal_visual_release']))
        if range_router is not None:
            summary['requested_stage_completed']=summary['requested_stage_completed'] and range_router.active=='near'
    elif args.tool_gap_m is not None:
        summary['requested_stage_completed']=bool(physical_valid and platform_completed is not None and platform_failure is None)
    # The diagnostic CLI exit code reports physical execution validity only.
    # Stage acceptance is separate: a safely stopped/unfinished run can have
    # valid physics while requested_stage_completed is false.
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ('mode','elapsed_sim_s','wall_s','minimum_true_optical_depth_m','after20s','physical_run_valid','position_accuracy')},indent=2),flush=True)
    return 0 if summary['physical_run_valid'] else 1


if __name__=='__main__':
    raise SystemExit(main())
