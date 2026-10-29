"""Validate actual grasp-end installation in a reduced native dynamics fixture.

No renderer, lighting or scene files are changed. Arm poses in this tool are
fixed encoder inputs; this is not arm deployment or collision verification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SERVER), str(SERVER/'backend')]

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros
from simulation.differential_gnss import DifferentialGNSS
from simulation.end_stereo_pointing import EndStereoGeometry, PrealignmentController
from simulation.native_integration import configure_scene_integrator
from simulation.near_field.navigation import RelativeEKF
from simulation.orbital_rendezvous.model import assembly_properties
from space_arm_platform.gnss_config import GNSSConfig
from tools.diagnostics.validate_cooperative_pointing import fixture, sample_gnss, PHYSICS_PERIOD_NS, CONTROL_PERIOD_NS


def verify_fk(source, geometry, output):
    """Independent MJScene FK with authored transforms, stripped of assets.

    State assignment here is an explicitly static kinematic comparison, never
    part of the torque-controlled trajectories below.
    """
    original = ET.parse(source).getroot()
    parents = {child: parent for parent in original.iter() for child in parent}
    node = original.find('.//body[@name="gripper"]')
    chain = []
    while node.get('name') != 'cubesat_bus':
        chain.append(node)
        node = parents[node]
    root = ET.Element('mujoco')
    ET.SubElement(root, 'option', gravity='0 0 0')
    world = ET.SubElement(root, 'worldbody')
    body = ET.SubElement(world, 'body', name='cubesat_bus')
    ET.SubElement(body, 'freejoint', name='service_free')
    ET.SubElement(body, 'inertial', mass='1', pos='0 0 0', diaginertia='.1 .1 .1')
    for original_body in reversed(chain):
        body = ET.SubElement(body, 'body', **{k:v for k,v in original_body.attrib.items()
                                             if k in ('name','pos','quat')})
        ET.SubElement(body, 'inertial', mass='1', pos='0 0 0', diaginertia='.1 .1 .1')
        for joint in original_body.findall('joint'):
            if joint.get('name') in geometry.chain.joint_names:
                ET.SubElement(body, 'joint', type='hinge', **{k:v for k,v in joint.attrib.items()
                    if k in ('name','pos','axis')})
    ET.SubElement(body, 'site', name='grip_frame')
    path = output/'arm_kinematic_fixture.xml'
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess('fk')
    process.addTask(sim.CreateNewTask('fk_task', PHYSICS_PERIOD_NS))
    scene = mujoco.MJScene.fromFile(str(path))
    sim.AddModelToTask('fk_task', scene)
    sim.InitializeSimulation()
    records = []
    try:
        for q in (np.zeros(6), np.array([-np.pi/2,0,0,0,0,0]), np.array([.2,-.3,.4,-.2,.15,.1])):
            for i, value in enumerate(q, 1):
                scene.getBody(f'bottom_link{i}').getScalarJoint(f'bottom_joint{i}').setPosition(float(value))
            scene.postIntegration(0)
            scene.UpdateState(0)
            state = scene.getSite('grip_frame').stateOutMsg.read()
            p, r = geometry.chain.forward(q)
            position_error = float(np.linalg.norm(np.array(state.r_BN_N)-p))
            rotation_error = float(np.linalg.norm(rbk.MRP2C(state.sigma_BN).T-r))
            records.append(dict(joints_rad=q.tolist(),position_error_m=position_error,
                                rotation_matrix_error=rotation_error))
        if any(r['position_error_m']>1e-8 or r['rotation_matrix_error']>1e-8 for r in records):
            raise AssertionError('FK differs from native MJScene')
        return records
    finally:
        sim.TotalSim.deleteThreads()


def run_case(model, geometry, output, *, label, depth, angle, q, expect_alignment, duration=100.,
             initial_axis=(1.,2.,3.), target_spin=(0.,0.,0.), service_spin=(0.,0.,0.),
             target_com_velocity=(0.,0.,0.), motion_reference=False, tracking_hold_s=0.):
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess('pointing')
    process.addTask(sim.CreateNewTask('physics',PHYSICS_PERIOD_NS))
    scene = mujoco.MJScene.fromFile(str(model))
    integration = configure_scene_integrator(scene)
    scene.addTorqueActuator('pointing','target_origin')
    msg = messaging.TorqueAtSiteMsg()
    msg.write(messaging.TorqueAtSiteMsgPayload())
    scene.getTorqueActuator('pointing').torqueInMsg.subscribeTo(msg)
    sim.AddModelToTask('physics',scene)
    sim.InitializeSimulation()
    try:
        relative = np.array([depth,0.,0.])
        # Optical checkpoints are not body-origin separations. Place the
        # *aligned reference* mechanism center at the requested optical depth.
        # The misaligned target may initially have a different optical depth.
        if expect_alignment:
            camera_p, camera_r = geometry.cameras_service(q)['left']
            for _ in range(12):
                target_r = geometry.reference(relative,np.eye(3),q)
                z = (camera_r.T@(relative+target_r@geometry.center-camera_p))[2]
                relative[0] += (depth-z)/camera_r[0,2]
            if abs(z-depth)>1e-7:
                raise AssertionError('optical checkpoint initialization did not converge')
        axis = np.asarray(initial_axis,float)
        axis = axis/np.linalg.norm(axis)
        ep = np.r_[np.cos(np.deg2rad(angle)/2),axis*np.sin(np.deg2rad(angle)/2)]
        initial = geometry.reference(relative,np.eye(3),q)@rbk.EP2C(ep).T
        body = scene.getBody('target')
        # Only physical initialization; the loop writes torque, never state.
        body.setPosition(relative.tolist())
        body.setAttitude(rbk.C2MRP(initial.T).tolist())
        body.setAttitudeRate(list(target_spin))
        center = np.fromstring(ET.parse(model).find('./worldbody/body[@name="target"]/inertial').get('pos'),sep=' ')
        body.setVelocity((np.asarray(target_com_velocity)-initial@np.cross(target_spin,center)).tolist())
        scene.getBody('service').setAttitudeRate(list(service_spin))
        scene.postIntegration(0)
        scene.UpdateState(0)
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        readers = [scene.getBody(name).getOrigin().stateOutMsg.addSubscriber()
                   for name in ('target','service')]
        sensor = DifferentialGNSS(GNSSConfig())
        controller = PrealignmentController(geometry)
        nav = None
        stamp = 0.
        rows = []
        completed = None
        tracking_since = None
        max_torque = 0.
        for k in range(int(duration/.05)+1):
            nanos = k*CONTROL_PERIOD_NS
            now = nanos*1e-9
            if k:
                sim.ConfigureStopTime(nanos)
                sim.ExecuteSimulation()
                nav.predict(CONTROL_PERIOD_NS*1e-9,np.zeros(3),np.zeros((3,3)))
            if k%2 == 0:
                packet = sample_gnss(sensor,readers,now)
                if nav is None:
                    nav = RelativeEKF(packet['z'],packet['R'],acceleration_sigma_m_s2=.05)
                    stamp = now
                elif nav.update(packet,now):
                    stamp = now
            telemetry = readers[0]()
            service_telemetry = readers[1]()
            service_r = rbk.MRP2C(service_telemetry.sigma_BN).T
            reference_rate = np.zeros(3)
            if motion_reference:
                _, reference_rate = geometry.reference_motion(relative_position_n_m=nav.x[:3],
                    relative_velocity_n_m_s=nav.x[3:],service_rotation_n_from_body=service_r,
                    service_angular_rate_body_rad_s=service_telemetry.omega_BN_B,
                    joint_position_rad=q,joint_velocity_rad_s=np.zeros(6))
            arguments = dict(now_s=now,input_stamps_s=dict(navigation=stamp,service_attitude=now,
                target_telemetry=now,camera_kinematics=now),relative_position_n_m=nav.x[:3],
                navigation_position_sigma_m=nav.sigma_m,service_rotation_n_from_body=service_r,
                target_rotation_n_from_body=rbk.MRP2C(telemetry.sigma_BN).T,
                target_angular_rate_body_rad_s=np.array(telemetry.omega_BN_B),
                reference_angular_rate_n_rad_s=reference_rate,joint_position_rad=q)
            result = controller.update(**arguments)
            payload = messaging.TorqueAtSiteMsgPayload()
            payload.torque_S = result.torque_body_nm.tolist()
            msg.write(payload,nanos)
            max_torque = max(max_torque,float(np.linalg.norm(result.torque_body_nm)))
            rows.append(dict(time_s=now,aligned=result.aligned,reason=result.reason,dwell_s=result.dwell_s,
                attitude_error_deg=result.attitude_error_deg,rate_error_rad_s=result.rate_error_rad_s,
                torque_body_nm=result.torque_body_nm.tolist(),view=result.view,
                reference_angular_rate_n_rad_s=reference_rate.tolist()))
            if result.aligned:
                if tracking_since is None:
                    tracking_since = now
                if now-tracking_since >= tracking_hold_s-1e-9:
                    completed = now
                    break
            else:
                tracking_since = None
        stale_args = dict(arguments)
        stale_args['now_s'] = now+.05
        stale_args['input_stamps_s'] = dict(arguments['input_stamps_s'],navigation=now-1.)
        stale = controller.update(**stale_args)
        stale_ok = not stale.aligned and np.linalg.norm(stale.torque_body_nm)==0 and stale.reason=='input_stale'
        success = bool(completed is not None) if expect_alignment else (
            completed is None and result.view is not None and not result.view['geometry_ok']
            and result.attitude_error_deg is not None and result.attitude_error_deg<1.)
        record = dict(label=label,body_origin_separation_m=float(relative[0]),initial_error_deg=angle,
            aligned_reference_optical_depth_m=depth if expect_alignment else None,
            fixed_encoder_joints_rad=q.tolist(),expect_alignment=expect_alignment,
            initial_axis_body=axis.tolist(),initial_target_rate_body_rad_s=list(target_spin),
            initial_service_rate_body_rad_s=list(service_spin),initial_target_com_velocity_n_m_s=list(target_com_velocity),
            reference_rate_from_estimated_motion=motion_reference,required_tracking_after_alignment_s=tracking_hold_s,
            achieved_tracking_after_alignment_s=0. if tracking_since is None else now-tracking_since,
            completed_s=completed,final_error_deg=result.attitude_error_deg,
            final_rate_rad_s=result.rate_error_rad_s,final_reason=result.reason,view=result.view,
            max_torque_nm=max_torque,gnss_rejected=nav.rejected,stale_input_stops_torque=stale_ok,
            passed=success and stale_ok and max_torque<=.02+1e-12,integration=integration)
        (output/(label+'.jsonl')).write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
        return record
    finally:
        sim.TotalSim.deleteThreads()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-model',type=Path,required=True)
    parser.add_argument('--camera-config',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=SERVER/'run/end_stereo_pointing')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    geometry = EndStereoGeometry.from_files(args.scene_model,args.camera_config)
    fk = verify_fk(args.scene_model,geometry,output)
    properties = assembly_properties(ET.parse(args.scene_model).find('./worldbody/body[@name="capture_target"]'))
    model = output/'target_rigid_fixture.xml'
    fixture(model,properties)
    q_aimed = np.array([-np.pi/2,0.,0.,0.,0.,0.])
    cases = [dict(label=f'aimed_17m4_{angle:g}deg',depth=17.4,angle=angle,q=q_aimed,
                  expect_alignment=True) for angle in (0.,30.,90.,180.)]
    cases += [dict(label=f'aimed_{depth:g}m',depth=depth,angle=90.,q=q_aimed,
                   expect_alignment=True) for depth in (50.,5.5)]
    cases += [dict(label='folded_vbar_camera_outside',depth=17.4,angle=0.,q=np.zeros(6),
                   expect_alignment=False,duration=10.)]
    results = []
    for case in cases:
        result = run_case(model,geometry,output,**case)
        results.append(result)
        print(json.dumps(result),flush=True)
    summary = dict(scope='actual installation and FK, reduced native target dynamics; not full mission or image validation',
        sources={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in (args.scene_model,args.camera_config)},fk_checks=fk,
        target_properties=properties,controller_hz=20,physics_hz=240,
        assumptions=['fixed measured arm posture and service attitude','ideal cooperative attitude/rate telemetry',
                     'GNSS solution-level sensor noise','zero-gravity rigid target; locked-panel inertia',
                     'candidate arm prepoint is not deployment/collision validation',
                     'pinhole projection is not image detection or occlusion evidence'],
        results=results,passed=all(r['passed'] for r in results))
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=summary['passed'],cases=len(results))))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
