"""Mass-weighted articulated COM lever arms from the unchanged authored MJCF."""
import numpy as np

from simulation.orbital_rendezvous.model import rotation


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class CenterKinematics:
    def __init__(self, root):
        def parse(body, top=False):
            inertia = body.find('inertial')
            joints = []
            for joint in body.findall('joint'):
                if joint.get('type') == 'free' or joint.get('name', '').startswith('rw_'):
                    continue  # wheel mass centers coincide with their spin axes
                axis = np.fromstring(joint.get('axis', '0 0 1'), sep=' ')
                axis /= np.linalg.norm(axis)
                joints.append((joint.get('name'), joint.get('type', 'hinge'),
                               np.fromstring(joint.get('pos', '0 0 0'), sep=' '), axis))
            return (np.zeros(3) if top else np.fromstring(body.get('pos', '0 0 0'), sep=' '),
                    np.eye(3) if top else rotation(body), float(inertia.get('mass')),
                    np.fromstring(inertia.get('pos', '0 0 0'), sep=' '), joints,
                    [parse(child) for child in body.findall('body')])
        self.tree = parse(root, True)
        self.cache = None

    def evaluate(self, positions, rates=None):
        rates = rates or {}
        key = (tuple(sorted(positions.items())), tuple(sorted(rates.items())))
        if self.cache is not None and key == self.cache[0]:
            return self.cache[1].copy(), self.cache[2].copy()

        def walk(node, p, R, v, w):
            offset, fixed_R, mass, local_com, joints, children = node
            offset = R @ offset
            p, v, R = p+offset, v+np.cross(w, offset), R@fixed_R
            for name, kind, anchor, axis in joints:
                q, dq = positions.get(name, 0.), rates.get(name, 0.)
                axis_n = R@axis
                if kind == 'slide':
                    shift = axis_n*q
                    p, v = p+shift, v+np.cross(w, shift)+axis_n*dq
                elif kind == 'hinge':
                    K = skew(axis)
                    turn = np.eye(3)+np.sin(q)*K+(1-np.cos(q))*(K@K)
                    shift = R@(anchor-turn@anchor)
                    w_new = w+axis_n*dq
                    v = v+np.cross(w, shift)-np.cross(axis_n*dq, R@turn@anchor)
                    p, R, w = p+shift, R@turn, w_new
                else:
                    raise ValueError(f'unsupported COM joint {kind}')
            lever = R@local_com
            total = np.r_[mass, mass*(p+lever), mass*(v+np.cross(w, lever))]
            for child in children:
                total += walk(child, p, R, v, w)
            return total

        total = walk(self.tree, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3))
        center, velocity = total[1:4]/total[0], total[4:]/total[0]
        self.cache = key, center, velocity
        return center.copy(), velocity.copy()


def center_relative(origin_relative, covariance, frame, omega, target_rotation, chaser_rotation,
                    target_rate, chaser_rate, target_center, chaser_center):
    """Exact attitude/encoders are deterministic offsets; transform inherited P."""
    ct, dct = target_center
    cb, dcb = chaser_center
    lever = target_rotation@ct-chaser_rotation@cb
    lever_velocity = (target_rotation@(np.cross(target_rate, ct)+dct)
                      - chaser_rotation@(np.cross(chaser_rate, cb)+dcb))
    p = origin_relative[:3]+lever
    v = origin_relative[3:]+lever_velocity-np.cross(omega, p)
    transform = np.block([[frame.T, np.zeros((3, 3))], [-frame.T@skew(omega), frame.T]])
    return np.r_[frame.T@p, frame.T@v], transform@covariance@transform.T
