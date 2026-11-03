"""Bounded unloading through capture26's existing ideal body-torque channel.

Inputs are wheel encoder speeds. The attitude controller counteracts this known
external torque with the wheels, transferring their stored angular momentum out
of the spacecraft. This models an ideal torque actuator, not thruster geometry.
"""
import numpy as np


class WheelUnloading:
    schema = 'space-sim-wheel-unloading/1'
    start_fraction = .70
    stop_fraction = .50
    max_torque_nm = .02
    gain_per_s = .02

    def __init__(self):
        self.active = np.zeros(3, dtype=bool)
        self.torque = np.zeros(3)
        self.speed_fraction = np.zeros(3)

    def update(self, speeds, inertias, limits):
        speeds, inertias, limits = (np.asarray(v, float) for v in (speeds, inertias, limits))
        if (any(v.shape != (3,) or not np.isfinite(v).all() for v in (speeds, inertias, limits))
                or np.any(inertias <= 0) or np.any(limits <= 0)):
            raise ValueError('unloading requires three finite wheel encoder speeds and positive hardware limits')
        self.speed_fraction = np.abs(speeds)/limits
        self.active = np.where(self.active, self.speed_fraction > self.stop_fraction,
                               self.speed_fraction >= self.start_fraction)
        request = -self.gain_per_s*inertias*speeds*self.active
        self.torque = request*min(1., self.max_torque_nm/max(np.linalg.norm(request), 1e-12))
        return self.torque.copy()

    def snapshot(self):
        return dict(schema=self.schema, active=self.active.tolist(), torque_body_nm=self.torque.tolist(),
                    speed_fraction=self.speed_fraction.tolist(), start_fraction=self.start_fraction,
                    stop_fraction=self.stop_fraction, max_torque_nm=self.max_torque_nm,
                    gain_per_s=self.gain_per_s, actuator_model='existing ideal capture26 body-torque channel')

    def restore(self, saved):
        if saved['schema'] != self.schema or any(saved[k] != getattr(self, k) for k in (
                'start_fraction', 'stop_fraction', 'max_torque_nm', 'gain_per_s')):
            raise ValueError('wheel unloading policy changed across handoff')
        active = np.asarray(saved['active'])
        torque, fraction = (np.asarray(saved[k], float) for k in ('torque_body_nm', 'speed_fraction'))
        if (active.shape != (3,) or active.dtype.kind != 'b'
                or any(v.shape != (3,) or not np.isfinite(v).all() for v in (torque, fraction))
                or np.any(fraction < 0) or np.linalg.norm(torque) > self.max_torque_nm+1e-12):
            raise ValueError('invalid wheel unloading history')
        self.active = active.copy()
        self.torque, self.speed_fraction = torque.copy(), fraction.copy()
