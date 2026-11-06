"""Joint scene preflight: left carry first, then right with the left object held."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from .randomization import generator
import numpy as np
from simulation.sarm_model_profile import DUAL_ARMS
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz
from .floating_assets import FLOATING_TASKS
from .floating_capture import FloatingCaptureTask, FloatingCaptureConfig
from .floating_grasps import evaluate_grasps, requested_candidates, GraspCandidate
from .drawer_initialization import NativePoseProbe


def make_arm_task(model, scene, side):
    record=scene['collection_config']['dual_floating']
    obj=next(i for i in record['instances'] if i['arm']==side)
    asset=replace(FLOATING_TASKS[obj['type_task']],body=obj['instance_id'])
    cfg=FloatingCaptureConfig(task=obj['type_task'],
        retract_position_body_m=tuple(record['arms'][side]['retract_position_body_m']))
    task=FloatingCaptureTask(model,cfg,arm=DUAL_ARMS[side],target_asset=asset)
    task.object_instances=record['instances']
    grasp=record['arms'][side].get('grasp')
    if grasp: task.grasp=GraspCandidate(**grasp['candidate'])
    return task


def select_dual_grasps(model, scene):
    record=scene['collection_config']['dual_floating']
    held_q=None
    reports={}
    for side,arm in DUAL_ARMS.items():
        task=make_arm_task(model,scene,side)
        probe=NativePoseProbe(model,arm=arm)
        try:
            for name,values in record['initial_joint_positions'].items():
                address=probe.address(name);probe.home[address:address+len(values)]=values
            if held_q is not None:
                probe.home[:]=held_q
                probe.held_finger_contacts={name:'object_left' for name in DUAL_ARMS['left'].finger_bodies}
            initial=np.array(record['arms'][side]['initial_joint_position_rad'])
            obj=next(i for i in record['instances'] if i['arm']==side)
            position=np.array(obj['position_body_m']);rotation=quaternion_wxyz_to_matrix(obj['orientation_wxyz'])
            options,candidates=requested_candidates(obj['type_task'],scene)
            feasible,report=evaluate_grasps(probe,task,initial,position,rotation,record['instances'],{},candidates)
            reports[side]=report
            if not feasible:return dict(accepted=False,arm=side,reports=reports)
            candidate,row,goal,pre,carry=choose_grasp(feasible,options,scene['seed'],f'dual-grasp-{side}-v1')
            record['arms'][side]['grasp']=dict(candidate=candidate.snapshot(),grasp_joint_position_rad=goal.tolist(),
                pregrasp_joint_position_rad=pre.tolist(),retract_joint_position_rad=carry.tolist(),
                contact_opening_rad=row['contact_opening_rad'],candidates=report['candidates'])
            # Only the private native probe assumes retention to screen right-arm geometry.
            # Runtime retention remains real contact, with continuous left-arm monitoring.
            point,desired=candidate.pose(position,rotation)
            local_p=desired.T@(position-point);local_r=desired.T@rotation
            final_p=np.array(task.config.retract_position_body_m)+desired@local_p
            final_r=desired@local_r
            probe.q[:]=probe.home
            address=probe.address(obj['free_joint'])
            probe.q[address:address+7]=[*final_p,*matrix_to_quaternion_wxyz(final_r)]
            opening=row['contact_opening_rad']
            probe.contacts(np.r_[carry,opening,-opening])
            held_q=probe.q.copy()
        finally:probe.close()
    record['preflight']=dict(accepted=True,reports=reports)
    record['selection_sha256']=selection_digest(scene)
    return record['preflight']


def choose_grasp(feasible, options, seed, stream):
    if options.selection == 'random':
        return feasible[int(generator(seed, stream).integers(len(feasible)))]
    return max(feasible,key=lambda x:(not x[1]['contact_asymmetry_warning'],x[0].id=='baseline',x[1]['score']))


def selection_digest(scene):
    record={k:v for k,v in scene['collection_config']['dual_floating'].items() if k!='selection_sha256'}
    value=dict(record=record,seed=scene['seed'],options=scene.get('floating_grasp',{}))
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_dual_selection(model, scene):
    record=scene['collection_config']['dual_floating']
    if (not record.get('preflight',{}).get('accepted')
            or hashlib.sha256(Path(model).read_bytes()).hexdigest()!=record['model_sha256']
            or record.get('selection_sha256')!=selection_digest(scene)):
        raise ValueError('Dual preflight does not match model, scene or selected grasps')
    for side in DUAL_ARMS:
        if not record['arms'][side].get('grasp'):
            raise ValueError('Both arms require an accepted grasp')
