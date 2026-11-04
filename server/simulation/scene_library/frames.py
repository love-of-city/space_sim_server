"""Rigid mounting transforms; movable ancestry must use measured state."""
import numpy as np
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


def site_attachment(root, name):
    """Return a rigid site's coordinates in its actual free-body owner."""
    node = root.find(f'.//site[@name="{name}"]')
    if node is None:
        raise ValueError(f'Missing site {name}')
    parents = {child: parent for parent in root.iter() for child in parent}
    position, rotation = np.zeros(3), np.eye(3)
    while node.tag != 'worldbody':
        free = node.find('freejoint') is not None or node.find('joint[@type="free"]') is not None
        if node.tag == 'body' and free:
            return node.get('name'), position, rotation
        if node.find('joint') is not None:
            raise ValueError('Site on articulated ancestry requires a measured site transform')
        if any(k in node.attrib for k in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
            raise ValueError('Attachment requires explicit quaternions')
        r = quaternion_wxyz_to_matrix(np.fromstring(node.get('quat', '1 0 0 0'), sep=' '))
        position = np.fromstring(node.get('pos', '0 0 0'), sep=' ') + r @ position
        rotation = r @ rotation
        node = parents[node]
    raise ValueError('Site needs a free-body owner')


def mounted_body_pose(root, name, base="cubesat_bus", *, allow_preheld=False):
    body = root.find(f'.//body[@name="{name}"]')
    if body is None:
        raise ValueError(f"Missing mounted body {name}")
    parents = {child: parent for parent in root.iter() for child in parent}
    position, rotation = np.zeros(3), np.eye(3)
    node = body
    while node.get("name") != base:
        lock = root.find('./equality/weld[@name="preheld_capture_lock"]')
        if (allow_preheld and node.get('name') == 'capture_target' and lock is not None
                and lock.get('body1') == 'gripper' and lock.get('body2') == 'capture_target'
                and lock.get('active', 'true') == 'true'):
            r = quaternion_wxyz_to_matrix(np.fromstring(node.get('quat', '1 0 0 0'), sep=' '))
            p = np.fromstring(node.get('pos', '0 0 0'), sep=' ')
            bus = root.find(f'./worldbody/body[@name="{base}"]')
            br = quaternion_wxyz_to_matrix(np.fromstring(bus.get('quat', '1 0 0 0'), sep=' '))
            bp = np.fromstring(bus.get('pos', '0 0 0'), sep=' ')
            return br.T @ (p + r @ position - bp), br.T @ r @ rotation
        if node.tag != "body" or node is not body and (
            node.find("joint") is not None or node.find("freejoint") is not None
        ):
            raise ValueError("Movable mounting ancestry requires measured-state transforms")
        if any(k in node.attrib for k in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Mounted body transforms require explicit quaternions")
        q = np.fromstring(node.get("quat", "1 0 0 0"), sep=" ")
        p = np.fromstring(node.get("pos", "0 0 0"), sep=" ")
        if q.shape != (4,) or p.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(p).all() or np.linalg.norm(q) == 0:
            raise ValueError("Invalid mounted body pose")
        r = quaternion_wxyz_to_matrix(q / np.linalg.norm(q))
        position, rotation = p + r @ position, r @ rotation
        node = parents[node]
    return position, rotation
