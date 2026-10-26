"""Analytic jerk-bounded rest-to-rest progress and C2 grasp-offset path."""
import numpy as np


class RestLeg:
    def __init__(self, start, end, vmax, amax=.002, jmax=.002):
        self.start, self.end = float(start), float(end)
        self.sign = np.sign(end-start)
        distance = abs(end-start)

        def ramps(v):
            tj = min(amax/jmax, np.sqrt(v/jmax))
            ta = max(0., v/(jmax*max(tj, 1e-15))-tj)
            return tj, ta, 2*tj+ta

        lo, hi = 0., vmax
        for _ in range(50):
            mid = (lo+hi)/2
            if mid*ramps(mid)[2] > distance:
                hi = mid
            else:
                lo = mid
        peak = lo
        tj, ta, ramp = ramps(peak)
        cruise = max(0., distance/max(peak, 1e-15)-ramp)
        self.phases = [(tj, jmax), (ta, 0.), (tj, -jmax), (cruise, 0.),
                       (tj, -jmax), (ta, 0.), (tj, jmax)]
        self.duration = sum(t for t, _ in self.phases)
        self.braking_start = self.duration-ramp

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return np.array([self.end, 0., 0.])
        t, p, v, a = max(0., elapsed), 0., 0., 0.
        for duration, jerk in self.phases:
            h = min(t, duration)
            p += v*h+.5*a*h*h+jerk*h**3/6
            v += a*h+.5*jerk*h*h
            a += jerk*h
            t -= h
            if t <= 1e-12:
                break
        return np.array([self.start+self.sign*p, self.sign*v, self.sign*a])


def offset_path(progress, start_s, end_s, transverse):
    s, ds, dds = progress
    span = start_s-end_s
    u = np.clip((start_s-s)/span, 0., 1.)
    f = 10*u**3-15*u**4+6*u**5
    df = (30*u**2-60*u**3+30*u**4)/(-span)
    ddf = (60*u-180*u**2+120*u**3)/span**2
    if s >= start_s or s <= end_s:
        df = ddf = 0.
    return (np.r_[s, f*transverse], np.r_[ds, df*ds*transverse],
            np.r_[dds, (ddf*ds**2+df*dds)*transverse], float(u))


class StopReference:
    """One latched jerk-bounded braking trajectory to a reachable stop."""
    def __init__(self, state, amax=.005, jmax=.002):
        self.position = np.array(state[:3], dtype=float)
        self.speed = float(np.linalg.norm(state[3:]))
        self.direction = np.array(state[3:])/max(self.speed, 1e-15)
        tj = min(amax/jmax, np.sqrt(self.speed/jmax))
        ta = max(0., self.speed/(jmax*max(tj, 1e-15))-tj)
        self.phases = [(tj, -jmax), (ta, 0.), (tj, jmax)]
        self.duration = 2*tj+ta
        self.endpoint = self.position+.5*self.speed*self.duration*self.direction

    def sample(self, elapsed):
        if elapsed >= self.duration:
            return self.endpoint.copy(), np.zeros(3), np.zeros(3)
        t, p, v, a = max(0., elapsed), 0., self.speed, 0.
        for duration, jerk in self.phases:
            h = min(t, duration)
            p += v*h+.5*a*h*h+jerk*h**3/6
            v += a*h+.5*jerk*h*h
            a += jerk*h
            t -= h
            if t <= 1e-12:
                break
        return self.position+p*self.direction, v*self.direction, a*self.direction
