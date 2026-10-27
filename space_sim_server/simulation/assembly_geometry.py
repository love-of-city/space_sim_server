"""Encoder-only assembly geometry using compiled, unchanged mass properties."""
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


def vector(element, name, default):
    return np.fromstring(element.get(name, default), sep=' ')


def rotation_vector(axis, angle):
    axis = axis/np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)


class AssemblyGeometry:
    def __init__(self, model_path, inertial_provider, roots=('cubesat_bus', 'capture_target')):
        self.tree = ET.parse(model_path)
        self.roots = [self.tree.find(f"./worldbody/body[@name='{name}']") for name in roots]
        self.inertials = {}
        self.joints = []
        for root in self.roots:
            if root is None:
                raise ValueError('assembly root missing')
            for body in self._attached(root):
                self.inertials[body.get('name')] = np.array(inertial_provider(body.get('name')), float)
                self.joints += [j.get('name') for j in body.findall('joint') if j.get('type') != 'free']

    @staticmethod
    def _attached(root):
        yield root
        for child in root.findall('body'):
            if child.find('freejoint') is None and child.find('joint[@type="free"]') is None:
                yield from AssemblyGeometry._attached(child)

    def evaluate(self, time_ns, encoders):
        if any(name not in encoders or not np.isfinite(encoders[name]).all() for name in self.joints):
            raise ValueError('complete finite timestamped joint encoders required')
        result = {}
        for root in self.roots:
            pieces, sites = [], {}
            def walk(body, p, r, velocity, omega, is_root=False):
                if not is_root:
                    delta = r@vector(body, 'pos', '0 0 0')
                    p, velocity = p+delta, velocity+np.cross(omega, delta)
                    r = r@quaternion_wxyz_to_matrix(vector(body, 'quat', '1 0 0 0'))
                    for joint in body.findall('joint'):
                        q, dq = encoders[joint.get('name')]
                        q -= float(joint.get('ref', 0.))
                        axis = vector(joint, 'axis', '0 0 1')
                        axis /= np.linalg.norm(axis)
                        axis_world = r@axis
                        if joint.get('type', 'hinge') == 'slide':
                            delta = axis_world*q
                            p, velocity = p+delta, velocity+np.cross(omega, delta)+axis_world*dq
                        else:
                            jpos = vector(joint, 'pos', '0 0 0')
                            rot = rotation_vector(axis, q)
                            delta = r@(jpos-rot@jpos)
                            velocity = velocity+np.cross(omega, delta)+np.cross(axis_world*dq, -r@rot@jpos)
                            p, r, omega = p+delta, r@rot, omega+axis_world*dq
                props = self.inertials[body.get('name')]
                if props[0] > 0:
                    delta = r@props[1:4]
                    ir = r@quaternion_wxyz_to_matrix(props[4:8])
                    pieces.append((props[0], p+delta, velocity+np.cross(omega, delta), ir@np.diag(props[8:11])@ir.T))
                for site in body.findall('site'):
                    sites[site.get('name')] = dict(position_body_m=(p+r@vector(site, 'pos', '0 0 0')).tolist(),
                        rotation_body_from_site=(r@quaternion_wxyz_to_matrix(vector(site, 'quat', '1 0 0 0'))).tolist())
                for child in body.findall('body'):
                    if child.find('freejoint') is None and child.find('joint[@type="free"]') is None:
                        walk(child, p.copy(), r.copy(), velocity.copy(), omega.copy())
            walk(root, np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), True)
            mass = sum(m for m, _, _, _ in pieces)
            centre = sum(m*p for m, p, _, _ in pieces)/mass
            centre_rate = sum(m*v for m, _, v, _ in pieces)/mass
            inertia = np.zeros((3, 3))
            for m, p, _, local_inertia in pieces:
                d = p-centre
                inertia += local_inertia+m*(np.dot(d, d)*np.eye(3)-np.outer(d, d))
            result[root.get('name')] = dict(time_ns=int(time_ns), mass_kg=float(mass),
                com_body_m=centre.tolist(), com_velocity_body_m_s=centre_rate.tolist(),
                inertia_com_body_kg_m2=inertia.tolist(), sites=sites)
        return result
