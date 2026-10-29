"""Body-origin prediction from cooperative attitude and encoder kinematics.

The relative filter tracks body origins, while translational force acts at the
assembly COM. Rotating lever arms therefore add acceleration at the origins.
Only installed geometry, attitudes, angular rates and encoder kinematics enter
this correction; there is no measured/true target translation input.
"""
import numpy as np

from simulation.cooperative_pointing import _rotation, _vector


class OriginLeverArm:
    def __init__(self, max_gap_s=.15):
        if not np.isfinite(max_gap_s) or max_gap_s<=0:
            raise ValueError('positive finite telemetry gap required')
        self.max_gap=max_gap_s
        self.previous_time=None
        self.previous_velocity=None

    def update(self, now_s, gradient_n_s2, *, target_rotation, service_rotation,
               target_com, service_com, target_com_rate, service_com_rate,
               target_omega, service_omega):
        """Return the interval-mean correction for the current prediction step.

        COM vectors and their joint-induced derivatives are in each body frame;
        omega is measured in that body frame. Rotations map into J2000. Current
        gyro/encoder samples must belong to now_s. The first call initializes
        history before any prediction; subsequent stale/gapped calls reject.
        Ideal telemetry is assumed by the caller's current experiments. Noisy
        gyro differentiation requires additional covariance/modelling work.
        """
        if not np.isfinite(now_s):raise ValueError('finite telemetry epoch required')
        gradient=np.asarray(gradient_n_s2,float)
        if gradient.shape!=(3,3) or not np.isfinite(gradient).all():
            raise ValueError('finite gravity gradient required')
        rt,rb=_rotation(target_rotation),_rotation(service_rotation)
        ct,cb=_vector(target_com,'target COM'),_vector(service_com,'service COM')
        vt,vb=_vector(target_com_rate,'target COM rate'),_vector(service_com_rate,'service COM rate')
        wt,wb=_vector(target_omega,'target gyro'),_vector(service_omega,'service gyro')
        offset=rt@ct-rb@cb
        velocity=rt@(np.cross(wt,ct)+vt)-rb@(np.cross(wb,cb)+vb)
        correction=gradient@offset
        if self.previous_time is not None:
            dt=now_s-self.previous_time
            if not 0<dt<=self.max_gap+1e-9:
                raise ValueError('nonincreasing or interrupted telemetry')
            correction=correction-(velocity-self.previous_velocity)/dt
        self.previous_time=float(now_s)
        self.previous_velocity=velocity
        return correction
