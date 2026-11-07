"""Private full-model IK/path/contact and native visual-ray preflight."""
import numpy as np
from .native_geometry_probe import NativePoseProbe
from simulation.local_mujoco_bindings import double_pointer
from simulation.serial_chain_kinematics import rotation_matrix_to_vector
from .capture_contact_audit import unplanned_capture_contact


def validate_initial_and_path(model, stepper, task, initial, projection):
    probe = NativePoseProbe(model)
    try:
        q = stepper.qpos.copy()
        # Remove the common orbital offset for robust local geometry queries.
        # Body_detail is local, qpos is inertial. Use the bus free coordinate.
        bus_address = probe.address('cubesat_free')
        common = q[bus_address:bus_address+3].copy()
        import xml.etree.ElementTree as ET
        root=ET.parse(model).getroot()
        from .capture_payload import payload_free_roots
        payload = {name for name,_ in payload_free_roots(root)}
        geom_root,geom_body,parents={},{},{}
        for body in root.findall('./worldbody/body'):
            for child in body.iter('body'):
                for descendant in child.findall('body'):
                    parents[descendant.get('name')]=child.get('name')
                for geom in child.findall('geom'):
                    geom_root[geom.get('name')]='capture_target' if body.get('name') in payload else body.get('name')
                    geom_body[geom.get('name')]=child.get('name')
            free = body.find('joint[@type="free"]')
            if free is None: free = body.find('freejoint')
            if free is not None:
                a = probe.address(free.get('name'));q[a:a+3] -= common
        probe.q[:] = q
        probe.check(probe.lib.lms_probe_set_state(probe.handle,probe.ptr(q),probe.ptr(probe.v)))
        def collisions():
            count=probe.lib.lms_contacts(probe.handle,None,0);probe.check(count)
            contacts=np.empty((count,5))
            probe.check(probe.lib.lms_contacts(probe.handle,probe.ptr(contacts),count))
            hits=[]
            for row in contacts:
                # Native rows: geom1, geom2, body1, body2, signed separation [m].
                if row[4]>=-.0001:continue
                names=[probe.geom_name(row[0]),probe.geom_name(row[1])]
                roots=[geom_root.get(n) for n in names]
                bodies=[geom_body.get(n,'') for n in names]
                if unplanned_capture_contact(names,bodies,roots,parents,row[4]):
                    hits.append(dict(geoms=names,distance_m=float(row[4])))
            return hits
        hits=collisions()
        if hits:return dict(accepted=False,reason='initial_collision',contacts=hits)
        camera = np.ascontiguousarray(projection['camera_position_service_m'])
        rays=[]
        for point in projection['region_points_service_m']:
            direction = np.ascontiguousarray(np.asarray(point)-camera)
            hit=np.empty(2)
            probe.check(probe.lib.lms_visual_ray(probe.handle,double_pointer(camera),double_pointer(direction),double_pointer(hit)))
            # A near-surface hit belongs to the visible interface, rather than an occluder.
            geom=probe.geom_name(int(hit[1])) if hit[1]>=0 else None
            interface_hit=geom=='target_instance_0030_part_015_color_00'
            occluded=bool(not interface_hit and 0. <= hit[0] < np.linalg.norm(direction)-.008)
            rays.append(dict(distance_m=float(hit[0]),geom=geom,interface_hit=interface_hit,occluded=occluded))
        visible = sum(not ray['occluded'] for ray in rays)
        # All sampled features of the reference square must remain unoccluded.
        # The interface has an open centre. A central ray can legitimately
        # reach the backplate through that opening; require visible frame
        # samples and no foreground obstruction instead of a solid centre.
        if visible < len(rays) or sum(r['interface_hit'] for r in rays)<3:
            return dict(accepted=False,reason='capture_camera_occluded',rays=rays)
        capture_addresses=[probe.address(name) for name in task.chain.joint_names]
        posture=task.reference[:6].copy()
        gp,gr=task.chain.forward(posture)
        tp,tr=initial.geometry(gp,gr)
        from .capture_grasp import INTERFACE_CENTER,TARGET_FROM_INTERFACE
        pi=tp+tr@INTERFACE_CENTER;ri=tr@TARGET_FROM_INTERFACE
        checked=0
        minimum_margin=float(np.min(np.minimum(posture-task.lower[:6],task.upper[:6]-posture)))
        for gap in (task.config.clearance_m,task.config.seated_gap_m):
            goal=pi+ri@[0.,0.,gap]
            for iteration in range(1000):
                p,r=task.chain.forward(posture)
                error=np.r_[goal-p,rotation_matrix_to_vector(ri@r.T)]
                if np.linalg.norm(error[:3])<.0003 and np.linalg.norm(error[3:])<.005:break
                dq=np.linalg.lstsq(task.chain.jacobian(posture),error,rcond=1e-4)[0]
                dq*=min(1.,.02/max(np.max(np.abs(dq)),1e-12))
                posture=np.clip(posture+dq,task.lower[:6],task.upper[:6])
                minimum_margin=min(minimum_margin,float(np.min(np.minimum(posture-task.lower[:6],task.upper[:6]-posture))))
                probe.q[capture_addresses]=posture
                probe.check(probe.lib.lms_probe_set_state(probe.handle,probe.ptr(probe.q),probe.ptr(probe.v)))
                hits=collisions()
                if hits:
                    return dict(accepted=False,reason='approach_path_collision',contacts=hits,rays=rays,path_samples=checked)
                checked+=1
            else:return dict(accepted=False,reason='approach_ik_unreachable',rays=rays,path_samples=checked)
        return dict(accepted=True,reason='geometric_preflight_passed',rays=rays,path_samples=checked,
                    nominal_goal_arm_joint_position_rad=posture.tolist(),minimum_joint_margin_rad=minimum_margin,
                    scope='finite sampled nominal path; actual free-body dynamics must still pass')
    finally:probe.close()
