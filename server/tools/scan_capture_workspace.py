"""Map initial-position coverage with real camera rays, IK paths and collisions.

This geometric scan is not a successful dynamics trial or training recording.
Full dynamics verification is subsequently run on separated representative poses.
"""
from dataclasses import asdict
import argparse
import itertools
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gap-m',type=float,action='append',help='Repeat to scan several TCP gaps')
    parser.add_argument('--coarse',action='store_true')
    parser.add_argument('--model',type=Path,default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--sampling-config',type=Path,
                        help='Scan continuous random candidates, including arm postures')
    parser.add_argument('--candidates',type=Path,help='Explicit diagnostic initial states')
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    from simulation.model_preparation.capture_grasp_model import prepare_capture_model
    from simulation.scripted_tasks.capture_grasp import CaptureGraspTask,PREPARED_ARM,INTERFACE_CENTER
    from simulation.scripted_tasks.capture_initialization import CaptureInitialState,camera_view
    from simulation.scripted_tasks.capture_preflight import validate_initial_and_path
    from simulation.scripted_tasks.native_geometry_probe import NativePoseProbe
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
    from simulation.scripted_tasks.capture_platform import platform_contract
    model=prepare_capture_model(args.model,out/'model.xml')
    probe=NativePoseProbe(model)
    base=probe.q.copy()
    for i,value in enumerate(np.deg2rad([90.,-60.,60.,0.,-90.,0.]),1):
        base[probe.address(f'joint{i}')]=value
    snapshot=args.model.resolve().with_name('initial-scene.json')
    if snapshot.is_file():
        joints=json.loads(snapshot.read_text())['randomization']['arm_joint_position_rad']
        for name,value in zip([f'joint{i}' for i in range(1,7)]+['ee_link11_joint','ee_link12_joint'],joints):
            base[probe.address(name)]=value
    capture_names=[f'bottom_joint{i}' for i in range(1,7)]+['axial','cam_angle']+[f'jaw_slide_{i}' for i in range(4)]
    addresses=[probe.address(name) for name in capture_names]
    target_address=probe.address('capture_target_free')
    gaps=args.gap_m or [.3,.5]
    if args.sampling_config and args.candidates:
        parser.error('choose sampling-config or candidates')
    if args.candidates:
        if args.gap_m or args.coarse:parser.error('candidates cannot be combined with grid options')
        parameters=json.loads(args.candidates.read_text())
        cases=[CaptureInitialState(**state) for state in parameters]
        sampling='explicit diagnostic poses; not random success-rate validation'
    elif args.sampling_config:
        if args.gap_m or args.coarse:
            parser.error('--sampling-config cannot be combined with grid options')
        from tools.prepare_capture_initial_pool import sample_candidates
        config=json.loads(args.sampling_config.read_text())
        cases=[CaptureInitialState(**state) for state in sample_candidates(config)]
        parameters=config
        sampling='seeded continuous joint-space and local target perturbations; not uniform Cartesian hemisphere'
    else:
        cases=[]
        for yaw,shoulder,elbow,gap in itertools.product(
                ([-90.,0.,90.] if args.coarse else [-90.,-60.,-30.,0.,30.,60.,90.]),
                ([-40.,0.,40.] if args.coarse else [-40.,-20.,0.,20.,40.]),[-20.,0.,20.],gaps):
            q=PREPARED_ARM.copy();q[:3]+=np.deg2rad([yaw,shoulder,elbow])
            cases.append(CaptureInitialState(gap_m=gap,arm_joint_position_rad=tuple(q)))
        parameters=dict(arm_joint_baseline_rad=PREPARED_ARM.tolist(),
                        arm_joint_offset_deg=[[-90.,90.],[-40.,40.],[-20.,20.]],
                        tcp_gap_m=gaps,target_local_tilt_deg=0.)
        sampling='discrete joint-space grid, not uniform Cartesian hemisphere'
    report=dict(schema='capture-workspace-scan/1',capture_platform_contract=platform_contract(),
        scope='initial view + sampled nominal IK/collision path; NOT physical success',
        parameters=parameters, search_sampling=sampling,total=len(cases),points=[])
    start=time.monotonic()
    def save():
        temporary=out/'scan.tmp';temporary.write_text(json.dumps(report,indent=2));temporary.replace(out/'scan.json')
    save()
    try:
        for index,initial in enumerate(cases):
            q=np.asarray(initial.arm_joint_position_rad if initial.arm_joint_position_rad is not None
                         else PREPARED_ARM)
            offset=np.rad2deg(q-PREPARED_ARM).tolist()
            try:
                task=CaptureGraspTask(model,initial_joints=q)
                gp,gr=task.chain.forward(q);tp,tr=initial.geometry(gp,gr)
                interface=tp+tr@INTERFACE_CENTER
                mount=task.chain._geometry(q)[1][0]
                view=camera_view(gp,gr,tp,tr)
                native_q=base.copy();native_q[addresses]=task.reference
                native_q[target_address:target_address+3]=tp
                native_q[target_address+3:target_address+7]=matrix_to_quaternion_wxyz(tr)
                from simulation.scripted_tasks.capture_payload import initial_payload_poses,payload_free_roots
                import xml.etree.ElementTree as ET
                root=ET.parse(model).getroot()
                parts=initial_payload_poses(root,tp,tr)
                for name,joint in payload_free_roots(root):
                    a=probe.address(joint);pp,pr=parts[name]
                    native_q[a:a+7]=np.r_[pp,matrix_to_quaternion_wxyz(pr)]
                check=(validate_initial_and_path(model,SimpleNamespace(qpos=native_q),task,initial,view)
                       if view['projected_visible'] else dict(accepted=False,reason='initial_camera_projection_rejected'))
                point=dict(id=index,initial_state=asdict(initial),offset_deg=offset[:3],
                    tcp_service_m=gp.tolist(),interface_service_m=interface.tolist(),
                    target_origin_service_m=tp.tolist(),mount_service_m=mount.tolist(),
                    radius_from_mount_m=float(np.linalg.norm(interface-mount)),
                    distance_from_satellite_reference_m=float(np.linalg.norm(interface)),
                    accepted=check['accepted'],reason=check['reason'],path_samples=check.get('path_samples'),
                    minimum_joint_margin_rad=check.get('minimum_joint_margin_rad'),
                    nominal_goal_arm_joint_position_rad=check.get('nominal_goal_arm_joint_position_rad'))
            except ValueError as error:
                point=dict(id=index,initial_state=asdict(initial),offset_deg=offset[:3],
                    accepted=False,reason=str(error))
            report['points'].append(point)
            if index%10==0:
                save();print(json.dumps(dict(completed=index+1,total=len(cases),accepted=sum(p['accepted'] for p in report['points']))),flush=True)
        report.update(status='complete',wall_seconds=time.monotonic()-start,
                      accepted=sum(p['accepted'] for p in report['points']))
        save();print(json.dumps({k:v for k,v in report.items() if k!='points'}),flush=True)
    finally:probe.close()


if __name__=='__main__':main()
