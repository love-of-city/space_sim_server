"""Physical full-window hold and approach corridor grading, independent of guidance."""
import math

import numpy as np

from .benchmark_contract import FlightTruthSample
from .benchmark_navigation_score import initial_orbit_errors
from .benchmark_relative_flight import AxisApproachCorridor
from .benchmark_scoring import WaypointScorer, relative_truth


class RelativeFlightScorer(WaypointScorer):
    def __init__(self, task, chaser_mass_kg):
        super().__init__(task, chaser_mass_kg)
        self.initialization_errors = set()
        self.initial_axis_distance = None
        self.approach_progress = None
        self.maximum_cross_track = None
        self.maximum_hold_error = 0.
        self.first_hold_violation_ns = None

    def add(self, raw):
        try:
            sample = FlightTruthSample.model_validate(raw)
            position, velocity = relative_truth(sample)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add("invalid_physical_sample")
            raise ValueError("invalid relative-flight physical evidence") from error
        initial, goal, path = self.task.initial, self.task.objective, self.task.path_requirement
        if self.last_time_ns is None:
            if initial_orbit_errors(sample.target_com_j2000, initial.target_altitude_m, 0.,
                                     initial.inclination_deg, initial.true_anomaly_deg, 0.):
                self.initialization_errors.add("initial_target_orbit_outside_declared_distribution")
            if (np.any(abs(position-initial.relative_position_tnr_m) > initial.position_half_width_m+1e-5)
                    or np.any(abs(velocity-initial.relative_velocity_tnr_m_s) > initial.velocity_half_width_m_s+1e-8)):
                self.initialization_errors.add("initial_relative_state_outside_declared_distribution")
        error = float(np.linalg.norm(position-goal.position_tnr_m))
        self.maximum_hold_error = max(self.maximum_hold_error, error)
        if isinstance(path, AxisApproachCorridor):
            index = 0 if path.axis == "T" else 2
            cross = math.sqrt(sum(float(position[i])**2 for i in range(3) if i != index))
            self.maximum_cross_track = max(self.maximum_cross_track or 0., cross)
            if cross > path.maximum_cross_track_m:
                self.safety_violations.add("approach_corridor")
            if position[index] >= 0:
                self.safety_violations.add("wrong_approach_side")
            if self.initial_axis_distance is None:
                self.initial_axis_distance = -float(position[index])
            self.approach_progress = self.initial_axis_distance+float(position[index])
        elif (error > goal.position_tolerance_m or np.linalg.norm(velocity) > goal.speed_tolerance_m_s):
            if self.first_hold_violation_ns is None:
                self.first_hold_violation_ns = sample.time_ns
        super().add(sample)

    def result(self):
        score = super().result()
        path = self.task.path_requirement
        outcome = score['outcome']
        quality = []
        if self.first_hold_violation_ns is not None:
            quality.append('continuous_hold_envelope')
        if self.initialization_errors:
            outcome = 'invalid_initialization'
        elif (outcome == 'time_limit' and quality):
            outcome = 'task_failure'
        elif (outcome == 'success' and isinstance(path, AxisApproachCorridor)
              and self.approach_progress < path.minimum_progress_m):
            outcome = 'task_failure'
            quality.append('approach_progress')
        score.update(schema_version='spacesim-autonomy-relative-flight-score/1', outcome=outcome,
                     success=outcome == 'success', initialization_errors=sorted(self.initialization_errors),
                     quality_violations=quality, path_requirement=path.kind,
                     maximum_position_error_m=self.maximum_hold_error if self.count else None,
                     first_hold_violation_s=None if self.first_hold_violation_ns is None else self.first_hold_violation_ns*1e-9,
                     maximum_cross_track_m=self.maximum_cross_track, approach_progress_m=self.approach_progress,
                     corridor_margin_m=(path.maximum_cross_track_m-self.maximum_cross_track
                                        if isinstance(path, AxisApproachCorridor) and self.maximum_cross_track is not None else None))
        if outcome != 'success':
            score['completion_time_s'] = None
        return score
