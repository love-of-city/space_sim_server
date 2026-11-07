"""Keep target-owned free connectors in their saved seats at capture reset.

These are reset/private-planning transforms only. Runtime retention and thread
contacts remain the scene's ordinary physical components.
"""
import numpy as np
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from simulation.scene_library.frames import site_attachment


def payload_free_roots(root):
    seats = [('aviation_plug','aviation_pickup'),('guide_plug','guide_pickup'),
             ('cylindrical_plug','cylindrical_pickup'),('bolt','bolt_storage_hole')]
    result = []
    for body_name, site in seats:
        if root.find(f'.//site[@name="{site}"]') is None: continue
        owner,_,_ = site_attachment(root,site)
        if owner != 'capture_target': continue
        body = root.find(f'./worldbody/body[@name="{body_name}"]')
        if body is None: raise ValueError('Target-owned part missing: '+body_name)
        joint = body.find('freejoint')
        if joint is None: joint = body.find('joint[@type="free"]')
        if joint is None: raise ValueError('Target-owned part is not free: '+body_name)
        result.append((body_name,joint.get('name')))
    return result


def initial_payload_poses(root,target_position,target_rotation):
    target = root.find('./worldbody/body[@name="capture_target"]')
    p = np.fromstring(target.get('pos','0 0 0'),sep=' ')
    r = quaternion_wxyz_to_matrix(np.fromstring(target.get('quat','1 0 0 0'),sep=' '))
    delta = target_rotation @ r.T
    assignments = {}
    for name,_ in payload_free_roots(root):
        body = root.find(f'./worldbody/body[@name="{name}"]')
        bp = np.fromstring(body.get('pos','0 0 0'),sep=' ')
        br = quaternion_wxyz_to_matrix(np.fromstring(body.get('quat','1 0 0 0'),sep=' '))
        assignments[name] = (target_position+delta@(bp-p),delta@br)
    return assignments
