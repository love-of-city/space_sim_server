"""Optional stationary evidence window with unchanged raw-wrench protection."""
import numpy as np


class TerminalObservation:
    def __init__(self,duration,force_limit,torque_limit,dwell):
        self.duration=duration;self.force_limit=force_limit;self.torque_limit=torque_limit;self.dwell=dwell
        self.start=None;self.overload=None;self.failed=False;self.complete=False

    def update(self,time_s,wrench):
        if np.shape(wrench)!=(6,) or not np.isfinite(wrench).all():
            self.failed=True;return False
        if self.start is None:self.start=time_s
        if np.linalg.norm(wrench[:3])>self.force_limit or np.linalg.norm(wrench[3:])>self.torque_limit:
            if self.overload is None:self.overload=time_s
            if time_s-self.overload>=self.dwell-1e-9:self.failed=True
        else:self.overload=None
        self.complete=bool(time_s-self.start>=self.duration-1e-9 and not self.failed)
        return self.complete
