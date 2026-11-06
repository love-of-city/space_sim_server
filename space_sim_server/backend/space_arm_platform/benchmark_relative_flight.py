"""Distinct stationkeeping, axis approach and proximity-abort task semantics."""
import math
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .benchmark_contract import Contract, Positive, WaypointTask


class FullDurationHold(Contract):
    kind: Literal["full_duration_relative_hold"]


class AxisApproachCorridor(Contract):
    kind: Literal["axis_approach_corridor"]
    axis: Literal["T", "R"]
    side: Literal["negative"]
    maximum_cross_track_m: Positive
    minimum_progress_m: Positive


class AbortRetreat(Contract):
    kind: Literal["abort_and_retreat"]
    prediction_horizon_s: Positive
    minimum_initial_closing_speed_m_s: Positive
    minimum_outward_speed_m_s: Positive
    reversal_confirmation_s: Positive
    reversal_deadline_s: Positive
    minimum_retreat_distance_m: Positive


class RelativeFlightTask(WaypointTask):
    schema_version: Literal["spacesim-autonomy-relative-flight/1"]
    configuration_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    requirement_id: Literal["ssb.relative-motion.hold", "ssb.approach.vbar", "ssb.approach.rbar", "ssb.safety.proximity-abort"]
    source_id: Literal["D12", "D29", "D30", "D40"]
    backend: Literal["native_team_relative_flight_v1"]
    path_requirement: Annotated[FullDurationHold | AxisApproachCorridor | AbortRetreat, Field(discriminator="kind")]

    @model_validator(mode="after")
    def distinct_flight_semantics(self):
        expected = {"D12": "ssb.relative-motion.hold", "D29": "ssb.approach.vbar", "D30": "ssb.approach.rbar",
                    "D40": "ssb.safety.proximity-abort"}
        if self.requirement_id != expected[self.source_id]:
            raise ValueError("relative-flight source and stable requirement must agree")
        initial, goal, path = self.initial, self.objective, self.path_requirement
        if self.source_id == "D12":
            if not isinstance(path, FullDurationHold) or goal.dwell_s != self.duration_s:
                raise ValueError("stationkeeping requires uninterrupted hold over the full episode")
            initial_error = math.dist(initial.relative_position_tnr_m, goal.position_tnr_m)
            initial_speed = math.dist(initial.relative_velocity_tnr_m_s, (0, 0, 0))
            if (initial_error+math.sqrt(3)*initial.position_half_width_m >= goal.position_tolerance_m
                    or initial_speed+math.sqrt(3)*initial.velocity_half_width_m_s >= goal.speed_tolerance_m_s):
                raise ValueError("the complete stationkeeping initial distribution must be inside the hold envelope")
        elif self.source_id == "D40":
            if not isinstance(path, AbortRetreat):
                raise ValueError("proximity abort requires a timely physical reversal and retreat")
            position, velocity = initial.relative_position_tnr_m, initial.relative_velocity_tnr_m_s
            dp, dv = initial.position_half_width_m, initial.velocity_half_width_m_s
            maximum_initial_range = math.dist(position, (0, 0, 0))+math.sqrt(3)*dp
            # Interval bounds cover the complete initial box, not just its mean
            # or sampled corners. Prediction is a declared linear risk model.
            maximum_dot = sum(p*v+abs(p)*dv+abs(v)*dp+dp*dv for p, v in zip(position, velocity, strict=True))
            if maximum_dot >= -path.minimum_initial_closing_speed_m_s*maximum_initial_range:
                raise ValueError("every initial state must be approaching with the declared minimum closing speed")
            projected = tuple(p+path.prediction_horizon_s*v for p, v in zip(position, velocity, strict=True))
            projected_bound = math.dist(projected, (0, 0, 0))+math.sqrt(3)*(dp+path.prediction_horizon_s*dv)
            if projected_bound >= self.safety.minimum_range_m:
                raise ValueError("the complete initial distribution must predict entry into the safety sphere")
            guaranteed_retreat = math.dist(goal.position_tnr_m, (0, 0, 0))-goal.position_tolerance_m-maximum_initial_range
            if path.minimum_retreat_distance_m <= 2*goal.position_tolerance_m or guaranteed_retreat < path.minimum_retreat_distance_m:
                raise ValueError("the endpoint must guarantee nontrivial retreat beyond the initial range")
            if not (path.reversal_confirmation_s < path.reversal_deadline_s < path.prediction_horizon_s
                    and path.reversal_deadline_s+goal.dwell_s < self.duration_s):
                raise ValueError("reversal confirmation and deadline must leave time for retreat and dwell")
            if path.minimum_outward_speed_m_s >= self.safety.maximum_relative_speed_m_s:
                raise ValueError("reversal speed must lie below the hard speed bound")
            for seconds, grid in ((path.reversal_confirmation_s, self.physics_period_ns),
                                  (path.reversal_deadline_s, self.decision_period_ns)):
                if not math.isclose(seconds*1e9/grid, round(seconds*1e9/grid), rel_tol=0, abs_tol=1e-8):
                    raise ValueError("reversal times must lie on their declared grids")
        else:
            axis = "T" if self.source_id == "D29" else "R"
            if not isinstance(path, AxisApproachCorridor) or path.axis != axis:
                raise ValueError("V-bar uses the T axis and R-bar the R axis")
            index = 0 if axis == "T" else 2
            cross = [i for i in range(3) if i != index]
            if any(goal.position_tnr_m[i] != 0 for i in cross) or goal.position_tnr_m[index] >= 0:
                raise ValueError("the approach endpoint must lie on the declared negative axis")
            if goal.position_tolerance_m >= path.maximum_cross_track_m:
                raise ValueError("the arrival region must lie inside the approach corridor")
            cross_extent = math.sqrt(sum((abs(initial.relative_position_tnr_m[i])+initial.position_half_width_m)**2 for i in cross))
            if cross_extent >= path.maximum_cross_track_m:
                raise ValueError("every initial position must lie inside the corridor")
            guaranteed_progress = (goal.position_tnr_m[index]-initial.relative_position_tnr_m[index]
                                   -initial.position_half_width_m-goal.position_tolerance_m)
            if guaranteed_progress < path.minimum_progress_m:
                raise ValueError("initial separation and arrival tolerance must guarantee the required approach progress")
            if goal.dwell_s >= self.duration_s:
                raise ValueError("approach must leave time for translation before arrival dwell")
        return self


class RelativeFlightContext(Contract):
    task: RelativeFlightTask
    chaser_mass_kg: Positive
