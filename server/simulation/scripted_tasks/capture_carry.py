"""Plan capture-arm transport with the whole spacecraft as a carried payload.

Only this private native geometry instance uses a rigid grasp transform while
checking routes. The executing simulation retains its free body and contacts.
Search and spline timing are the same modules used by the operating-arm tasks.
"""
import xml.etree.ElementTree as ET
from dataclasses import replace
import numpy as np

from simulation.serial_chain_kinematics import (
    quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz, rotation_matrix_to_vector)
from .capture_contact_audit import unplanned_capture_contact
from .capture_grasp import INTERFACE_CENTER, TARGET_FROM_INTERFACE
from .motion import solve_pose
from .native_geometry_probe import NativePoseProbe
from .obstacle_planner import JointRouteSearch, SearchOptions, endpoint_seed, PlanningFailure
from .motion_timing import time_joint_path

# wenquandan777, aacce0d963aa0dcf0b3d655b0a4e2260d25f1cfd:
# model/scene_library/original-root-v2/initial-scene.json in master.
# Use the saved target POSE: its arm joint angles belong to a different mount.
HANDOFF_POSITION = np.array([.792849999999958, .010000000000000009, .311035])
HANDOFF_ROTATION = quaternion_wxyz_to_matrix([.5, .5, .5, -.5])
HANDOFF_SOURCE = 'aacce0d963aa0dcf0b3d655b0a4e2260d25f1cfd:space_sim_server/model/scene_library/original-root-v2/initial-scene.json'
# IK hint from the positive-wrist branch of completed physical deliveries.
# The target pose, collision checks and real contact-supported execution still
# determine validity; this is never commanded directly as a joint target.
HANDOFF_IK_SEED = np.array([1.743, -1.998, 1.928, -3.072, 1.571, 1.741])


def target_pose(sample):
    rotation = sample['interface_rotation'] @ TARGET_FROM_INTERFACE.T
    return sample['interface_position'] - rotation @ INTERFACE_CENTER, rotation


def handoff_error(sample):
    p, r = target_pose(sample)
    return float(np.linalg.norm(p-HANDOFF_POSITION)), float(np.linalg.norm(
        rotation_matrix_to_vector(HANDOFF_ROTATION @ r.T)))


def plan_capture_carry(model, task, sample):
    probe = NativePoseProbe(model)
    try:
        root = ET.parse(model).getroot()
        from .capture_payload import payload_free_roots
        payload = payload_free_roots(root)
        payload_names = {name for name,_ in payload}
        geom_body, geom_root, parents = {}, {}, {}
        def walk(body, owner, parent=None):
            name = body.get('name'); parents[name] = parent
            for geom in body.findall('geom'):
                geom_body[geom.get('name')] = name
                geom_root[geom.get('name')] = owner
            for child in body.findall('body'): walk(child, owner, name)
        for body in root.findall('./worldbody/body'):
            walk(body,'capture_target' if body.get('name') in payload_names else body.get('name'))
        q = np.asarray(sample['native_qpos']).copy()
        a = probe.address('cubesat_free')
        bp, br = q[a:a+3].copy(), quaternion_wxyz_to_matrix(q[a+3:a+7])
        # Express every free root in the service frame, preserving scalar joints.
        for body in root.findall('./worldbody/body'):
            joint = body.find('joint[@type="free"]')
            if joint is None: joint = body.find('freejoint')
            if joint is None: continue
            j = probe.address(joint.get('name'))
            q[j:j+3] = br.T @ (q[j:j+3]-bp)
            q[j+3:j+7] = matrix_to_quaternion_wxyz(br.T @ quaternion_wxyz_to_matrix(q[j+3:j+7]))
        addresses = [probe.address(f'bottom_joint{i}') for i in range(1,7)]
        target_address = probe.address('capture_target_free')
        start = np.asarray(sample['q'][:6]).copy()
        p, r = task.chain.forward(start)
        tp, tr = target_pose(sample)
        relative_p, relative_r = r.T @ (tp-p), r.T @ tr
        free_parts = []
        for name,joint in payload:
            address = probe.address(joint)
            free_parts.append((address,r.T@(q[address:address+3]-p),
                r.T@quaternion_wxyz_to_matrix(q[address+3:address+7])))
        goal_r = HANDOFF_ROTATION @ relative_r.T
        goal_p = HANDOFF_POSITION - goal_r @ relative_p
        def collision(arm):
            cp, cr = task.chain.forward(arm)
            candidate = q.copy(); candidate[addresses] = arm
            candidate[target_address:target_address+3] = cp + cr @ relative_p
            candidate[target_address+3:target_address+7] = matrix_to_quaternion_wxyz(cr @ relative_r)
            for address,rp,rr in free_parts:
                candidate[address:address+3] = cp+cr@rp
                candidate[address+3:address+7] = matrix_to_quaternion_wxyz(cr@rr)
            probe.check(probe.lib.lms_probe_set_state(probe.handle, probe.ptr(candidate), probe.ptr(probe.v)))
            count = probe.lib.lms_contacts(probe.handle, None, 0); probe.check(count)
            rows = np.empty((count,5))
            probe.check(probe.lib.lms_contacts(probe.handle, probe.ptr(rows), count))
            for row in rows:
                if row[4] >= -.0001: continue
                names = [probe.geom_name(row[0]), probe.geom_name(row[1])]
                bodies = [geom_body.get(n,'') for n in names]
                owners = [geom_root.get(n) for n in names]
                if unplanned_capture_contact(names,bodies,owners,parents,row[4]):
                    return dict(geoms=names,bodies=bodies,separation_m=float(row[4]))
            return None
        search = JointRouteSearch(task.lower[:6],task.upper[:6],collision,
            seed=endpoint_seed(start,goal_p),options=SearchOptions(iterations=2400,query_budget=90000))
        rng = np.random.default_rng(endpoint_seed(start,goal_p))
        failures = []
        seeds = [HANDOFF_IK_SEED.copy(), start,
                 np.array([.55,-2.93,2.56,-1.21,-1.02,-3.14])]
        seeds += [rng.uniform(task.lower[:6],task.upper[:6]) for _ in range(24)]
        for seed_index, seed in enumerate(seeds):
            try:
                goal = solve_pose(task.chain,goal_p,goal_r,seed,task.lower[:6],task.upper[:6])
                if not search.valid(goal):
                    failures.append('goal_in_collision'); continue
                route = search.search(start,goal)
                knots = search.smooth(route)
                break
            except (ValueError, PlanningFailure) as error:
                failures.append(str(error))
                if str(error) in ('start_in_collision','collision_query_budget_exhausted'): raise
        else:
            raise PlanningFailure('capture_handoff_no_route', {**search.report,'candidate_failures':failures})
        poses = [task.chain.forward(joints) for joints in knots]
        distance = sum(np.linalg.norm(b[0]-a[0]) for a,b in zip(poses,poses[1:]))
        angle = sum(np.linalg.norm(rotation_matrix_to_vector(b[1]@a[1].T)) for a,b in zip(poses,poses[1:]))
        timing = time_joint_path(knots,float(distance),float(angle),
            joint_speed=.08,linear_speed=.020,angular_speed=.05,
            linear_acceleration=.01,angular_acceleration=.025,joint_acceleration=.05,
            linear_jerk=.025,angular_jerk=.08,joint_jerk=.15,
            slow_distance=.02,slow_speed=.004)
        # Arc-length timing alone does not bound local Cartesian derivatives.
        values = []
        for u in np.linspace(0.,1.,501):
            joints,dj,_,_ = timing.path.state(u)
            values.append(task.chain.jacobian(joints) @ dj)
        values = np.asarray(values)
        for component,limits in ((values[:,:3],(.020,.01,.025)),(values[:,3:],(.05,.025,.08))):
            first = np.gradient(component,np.linspace(0.,1.,501),axis=0,edge_order=2)
            second = np.gradient(first,np.linspace(0.,1.,501),axis=0,edge_order=2)
            bounds = 1.1*np.array([np.linalg.norm(v,axis=1).max() for v in (component,first,second)])
            timing = timing.limit_derivatives(bounds,*limits)
        # The target's offset workbench produces an appreciable contact moment.
        # Dilate the complete C2 law: speed /2, acceleration /4, jerk /8.
        # Contact support and final-pose acceptance remain unchanged.
        timing = replace(timing, times=timing.times*2., velocity=timing.velocity/2.)
        report = {**search.report,'candidate_failures':failures,'source':HANDOFF_SOURCE,
            'goal_seed_index':seed_index,
            'goal_seed_kind':('validated_positive_wrist' if seed_index==0 else
                              'measured_start' if seed_index==1 else 'fallback'),
            'target_goal_position_service_m':HANDOFF_POSITION.tolist(),
            'target_goal_orientation_wxyz':[.5,.5,.5,-.5],
            'arm_goal_rad':goal.tolist(),'duration_s':timing.duration,
            'tcp_path_length_m':float(distance),'path_waypoints':knots.tolist(),
            'collision_scope':'sampled complete carried-spacecraft and arm geometry; private probe only',
            'physics':'live free body, servo torques and native contacts; no weld or teleport'}
        return timing, report
    finally:
        probe.close()
