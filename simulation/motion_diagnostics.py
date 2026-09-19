"""Measured, directional Cartesian speed monitoring. Simulation-time debounce.

The denominator is the smoothed operator command BEFORE IK/constraint scaling.
Linear and angular channels never mix units. Contact/torque are not inferred
from speed alone: absent instrumented evidence, the cause stays unknown.
"""
import numpy as np


class MotionSpeedMonitor:
    def __init__(self, threshold=.8, recovery=.85, dwell=.3, recovery_dwell=.2, grace=.35):
        self.threshold, self.recovery = threshold, recovery
        self.dwell, self.recovery_dwell, self.grace = dwell, recovery_dwell, grace
        self.reset()

    def reset(self):
        self.last_time = None
        self.channels = {key: dict(direction=None, age=0., low=0., high=0., warning=False)
                         for key in ("linear", "angular")}
        self.latest = {"frame": "spacecraft_body", "control_point": "sarm_ee", "threshold": self.threshold,
                       "enabled": False, "command_stale": False, "linear": None, "angular": None}

    def update(self, now, expected, measured, predicted, *, enabled, stale=False,
               solver_reasons=(), measured_joints=None, target_joints=None,
               measured_joint_velocity=None, target_joint_velocity=None, actuator_evidence=None):
        if self.last_time is not None and now < self.last_time:
            self.reset()
        dt = 0. if self.last_time is None else min(.1, max(0., now-self.last_time))
        self.last_time = now
        expected, measured, predicted = [np.asarray(x, float) for x in (expected, measured, predicted)]
        valid = all(x.shape == (6,) and np.all(np.isfinite(x)) for x in (expected, measured, predicted))
        result = {"frame": "spacecraft_body", "control_point": "sarm_ee", "threshold": self.threshold,
                  "enabled": bool(enabled), "command_stale": bool(stale), "measurement_valid": bool(valid)}
        for name, sl, epsilon in (("linear", slice(0,3), .001), ("angular", slice(3,6), .005)):
            state = self.channels[name]
            cmd = expected[sl] if valid else np.zeros(3)
            speed = float(np.linalg.norm(cmd))
            active = enabled and not stale and valid and speed >= epsilon
            if not active:
                state.update(direction=None, age=0., low=0., high=0., warning=False)
                result[name] = {"active": False, "warning": False, "expected_speed": speed,
                                "actual_speed_along_command": 0., "ratio": None,
                                "reasons": [], "state": "idle" if valid else "measurement_unavailable"}
                continue
            direction = cmd/speed
            if state["direction"] is None or np.dot(direction, state["direction"]) < .95:
                state.update(age=0., low=0., high=0., warning=False)
            state["direction"] = direction
            state["age"] += dt
            actual = float(np.dot(measured[sl], direction))
            achievable = float(np.dot(predicted[sl], direction))
            ratio = actual/speed
            if state["age"] >= self.grace:
                state["low"] = state["low"]+dt if ratio < self.threshold-1e-12 else 0.
                state["high"] = state["high"]+dt if ratio >= self.recovery-1e-12 else 0.
                if state["low"]+1e-9 >= self.dwell:
                    state["warning"] = True
                if state["high"]+1e-9 >= self.recovery_dwell:
                    state["warning"] = False
            reasons = []
            if state["warning"]:
                # These are evidenced active solver constraints, not proof that
                # a nearby singularity/contact alone caused plant under-speed.
                if achievable/speed < self.threshold:
                    reasons.extend(dict(reason) for reason in solver_reasons)
                if measured_joints is not None and target_joints is not None:
                    err = np.abs(np.asarray(target_joints)-np.asarray(measured_joints))
                    lag = err > .01
                    if measured_joint_velocity is not None and target_joint_velocity is not None:
                        wanted = np.asarray(target_joint_velocity)
                        lag |= np.abs(np.asarray(measured_joint_velocity)-wanted) > np.maximum(.03, .3*np.abs(wanted))
                    if np.any(lag) and actual < achievable - .1*speed:
                        reasons.append({"code": "joint_tracking_error", "joints": (np.flatnonzero(lag)+1).tolist()})
                if actuator_evidence is not None and actual < achievable - .1*speed:
                    requested = np.asarray(actuator_evidence.get("requested_torque_nm", []), float)
                    applied = np.asarray(actuator_evidence.get("applied_torque_nm", []), float)
                    limits = np.asarray(actuator_evidence.get("torque_limits_nm", []), float)
                    if all(v.shape == (6,) and np.all(np.isfinite(v)) for v in (requested, applied, limits)) and np.all(limits > 0):
                        saturated = (np.abs(requested) > limits*1.001) & (np.abs(applied) >= limits*.98)
                        if np.any(saturated):
                            reasons.append({"code": "torque_saturation", "joints": (np.flatnonzero(saturated)+1).tolist()})
                if not reasons:
                    reasons.append({"code": "cause_unconfirmed", "joints": []})
            result[name] = {"active": True, "warning": bool(state["warning"]),
                            "expected_speed": speed, "actual_speed_along_command": actual,
                            "predicted_speed_along_command": achievable,
                            "lateral_speed": float(np.linalg.norm(measured[sl]-actual*direction)),
                            "ratio": ratio, "reasons": reasons,
                            "state": "low_speed" if state["warning"] else ("settling" if state["age"] < self.grace else "tracking")}
        self.latest = result
        return result
