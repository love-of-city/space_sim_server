"""Reference points and continuous dwell gates for the original SARM model.

The 0.5 m engineering standoff is the stowed TCP's axial distance to the
grasp plane, not the distance between spacecraft origins or assembly COMs.
"""
from dataclasses import dataclass

import numpy as np


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
            "platform_dwell_s": 5., "pregrasp_gap_m": .15, "capture_gap_m": .018}


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
