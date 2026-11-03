"""Reference points and continuous dwell gates for the original SARM model.

The 0.5 m engineering standoff is the stowed TCP's axial distance to the
grasp plane, not the distance between spacecraft origins or assembly COMs.
"""
from dataclasses import dataclass

import numpy as np
from .parameters import PARAMETERS


def approach_plan(chain):
    stowed, rotation = chain.forward(np.zeros(6))
    axis = rotation[:, 2]
    if axis[0] < .99:
        raise ValueError("near-field plan requires the authored SARM +X grasp axis")
    # Keep the target above the bus envelope and within both camera images.
    nominal = np.array([0., .10, .30])
    nominal[0] = (.5 + axis @ stowed - axis @ nominal) / axis[0]
    return {"schema": "spacesim-near-approach-plan/1",
            "standoff_reference": "stowed TCP to grasp plane along grasp normal",
            "stowed_tcp_bus_m": stowed.tolist(), "grasp_axis_bus": axis.tolist(),
            "nominal_grasp_bus_m": nominal.tolist(), "stowed_tcp_gap_m": .5,
            "optical_checkpoint_depth_m": 17.4,
            "optical_checkpoint_reference": "rectified stereo optical depth; camera Z=-0.10 m in tool frame",
            "checkpoint_position_tolerance_m": .05, "checkpoint_speed_limit_m_s": .002,
            "checkpoint_dwell_s": 5., "platform_position_tolerance_m": .006,
            "platform_dwell_s": 5., "pregrasp_gap_m": .15, "capture_gap_m": .018,
            "near_parameters": PARAMETERS.metadata(),
            "nodes": [
                {"id": "Vbar50m", "reference": "assembled COM target minus chaser, TNR along-track", "target_m": 50.},
                {"id": "Optical17p4m", "reference": "rectified stereo optical depth on COM V-bar", "target_m": 17.4},
                {"id": "VisualReady5p5m", "reference": "rectified stereo optical depth on COM V-bar", "target_m": 5.5},
                {"id": "PlatformStandoff", "reference": "stowed TCP axial gap to grasp plane", "target_m": .5}]}


@dataclass
class DwellGate:
    """Total capture/wait deadlines never reset when a joint condition fails."""
    timeout_s: float = 300.
    dwell_required_s: float = 5.
    capture_started_s: float | None = None
    waiting_since_s: float | None = None
    stable_since_s: float | None = None
    completed_at_s: float | None = None
    dwell_s: float = 0.
    samples: int = 0
    last_time_s: float | None = None

    def update(self, now, *, holding, qualified, braking=False):
        if self.completed_at_s is not None:
            return True
        if self.last_time_s is not None and now < self.last_time_s:
            raise ValueError('dwell clock moved backwards')
        new_sample = self.last_time_s is None or now > self.last_time_s
        self.last_time_s = now
        if braking and self.capture_started_s is None:
            self.capture_started_s = now
        if holding and self.waiting_since_s is None:
            self.waiting_since_s = now
        if not holding or not qualified:
            self.stable_since_s, self.samples, self.dwell_s = None, 0, 0.
        elif new_sample:
            if self.stable_since_s is None:
                self.stable_since_s = now
            self.samples += 1
            self.dwell_s = now-self.stable_since_s
            if self.dwell_s >= self.dwell_required_s-1e-9 and self.samples >= 101:
                self.completed_at_s = now
        return self.completed_at_s is not None

    def failure(self, now):
        if self.completed_at_s is not None:
            return None
        if self.waiting_since_s is not None and now-self.waiting_since_s >= self.timeout_s:
            return 'node_wait_timeout'
        if (self.waiting_since_s is None and self.capture_started_s is not None
                and now-self.capture_started_s >= PARAMETERS.capture_timeout_s):
            return 'node_capture_timeout'
        return None

    def telemetry(self, now):
        end = self.completed_at_s if self.completed_at_s is not None else now
        return {"dwell_s": self.dwell_s, "samples": self.samples,
                "wait_s": 0. if self.waiting_since_s is None else end-self.waiting_since_s,
                "capture_s": 0. if self.capture_started_s is None else end-self.capture_started_s,
                "completed_at_s": self.completed_at_s, "completed": self.completed_at_s is not None}


@dataclass
class OpticalCheckpoint:
    """Require an uninterrupted stationary dwell; completion is latched."""
    dwell_s: float = 0.
    completed_at_s: float | None = None
    stable_since_s: float | None = None

    def update(self, *, error_m, speed_m_s, ready, now):
        if self.completed_at_s is None:
            stationary = ready and np.linalg.norm(error_m) < .05 and speed_m_s < .002
            if not stationary:
                self.stable_since_s = None
            elif self.stable_since_s is None:
                self.stable_since_s = now
            self.dwell_s = 0. if self.stable_since_s is None else now-self.stable_since_s
            if self.dwell_s >= 5.:
                self.completed_at_s = now
        return self.completed_at_s is not None
