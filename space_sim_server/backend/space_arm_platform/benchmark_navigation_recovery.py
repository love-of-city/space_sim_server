"""S23: complete GNSS solution outage, propagated estimates, and reacquisition."""
import math
from typing import Literal

from pydantic import Field, StrictBool, model_validator

from .benchmark_contract import Contract, Count, Positive, State6
from .benchmark_navigation import (
    AbsoluteNavigationSafety, GNSSMeasurements, NavigationAccuracy, NavigationContext,
    NavigationTask, validate_navigation_physics,
)
from .benchmark_orbit import OrbitInitialState, OrbitTruthSample


class GNSSOutageDistribution(Contract):
    kind: Literal['temporary_complete_receiver_solution_outage']
    earliest_onset_s: Positive = Field(ge=5)
    latest_onset_s: Positive
    minimum_duration_s: Positive = Field(ge=5)
    maximum_duration_s: Positive


class NavigationRecoveryObjective(NavigationAccuracy):
    kind: Literal['navigation_outage_recovery']
    outage_maximum_position_error_m: Positive
    outage_maximum_velocity_error_m_s: Positive
    recovered_position_tolerance_m: Positive
    recovered_velocity_tolerance_m_s: Positive
    recovered_dwell_s: Positive
    recovery_deadline_after_return_s: Positive


class NavigationRecoveryTask(NavigationTask):
    schema_version: Literal['spacesim-autonomy-navigation-recovery/1']
    source_id: Literal['S23']
    requirement_id: Literal['ssb.recovery.navigation-failure']
    backend: Literal['native_team_gnss_recovery_v1']
    spacecraft_count: Literal[1]
    target_cooperation: Literal['none']
    reference_frame: Literal['Earth_centered_J2000']
    initial: OrbitInitialState
    safety: AbsoluteNavigationSafety
    objective: NavigationRecoveryObjective
    fault: GNSSOutageDistribution

    @model_validator(mode='after')
    def semantic_contract(self):
        validate_navigation_physics(self)
        fault, goal = self.fault, self.objective
        if self.observation.target_link != 'none':
            raise ValueError('single-receiver recovery has no cooperating target telemetry')
        if fault.earliest_onset_s > fault.latest_onset_s or fault.minimum_duration_s > fault.maximum_duration_s:
            raise ValueError('outage distributions must be ordered')
        if not goal.recovered_dwell_s < goal.recovery_deadline_after_return_s:
            raise ValueError('recovery must allow a continuous accuracy confirmation')
        if fault.latest_onset_s+fault.maximum_duration_s+goal.recovery_deadline_after_return_s >= self.duration_s:
            raise ValueError('outage and confirmed reacquisition must finish before episode end')
        if (goal.outage_maximum_position_error_m > goal.maximum_position_error_m
                or goal.outage_maximum_velocity_error_m_s > goal.maximum_velocity_error_m_s
                or goal.recovered_position_tolerance_m > goal.outage_maximum_position_error_m
                or goal.recovered_velocity_tolerance_m_s > goal.outage_maximum_velocity_error_m_s):
            raise ValueError('recovered and outage limits must fit the complete navigation envelope')
        times = (fault.earliest_onset_s, fault.latest_onset_s, fault.minimum_duration_s,
                 fault.maximum_duration_s, goal.recovered_dwell_s, goal.recovery_deadline_after_return_s)
        for seconds in times:
            ticks = seconds*1e9/self.decision_period_ns
            if not math.isclose(ticks, round(ticks), rel_tol=0, abs_tol=1e-8):
                raise ValueError('receiver transitions and confirmations must lie on the sensor grid')
        return self


class NavigationRecoveryObservation(GNSSMeasurements):
    schema_version: Literal['spacesim-autonomy-gnss-recovery-observation/1'] = 'spacesim-autonomy-gnss-recovery-observation/1'
    chaser_measurement_j2000: State6 | None
    target_measurement_j2000: None
    receiver_available: StrictBool

    @model_validator(mode='after')
    def no_solution_during_failure(self):
        if self.receiver_available != (self.chaser_measurement_j2000 is not None):
            raise ValueError('an unavailable receiver must deliver no solution')
        if self.delivery_time_ns != self.sample_time_ns:
            raise ValueError('this receiver declares zero delivery latency')
        return self


class GNSSOutageRealization(Contract):
    onset_time_ns: Count
    return_time_ns: Count

    def validate_for(self, task):
        fault = task.fault
        duration = self.return_time_ns-self.onset_time_ns
        if (self.onset_time_ns % task.decision_period_ns or self.return_time_ns % task.decision_period_ns
                or not round(fault.earliest_onset_s*1e9) <= self.onset_time_ns <= round(fault.latest_onset_s*1e9)
                or not round(fault.minimum_duration_s*1e9) <= duration <= round(fault.maximum_duration_s*1e9)):
            raise ValueError('private outage realization lies outside its declared distribution')

    def available(self, nanos):
        return not self.onset_time_ns <= nanos < self.return_time_ns


class NavigationRecoveryTruthSample(OrbitTruthSample):
    receiver_available: StrictBool
    outage: GNSSOutageRealization


class NavigationRecoveryDelivery(Contract):
    record_type: Literal['gnss_receiver_delivery'] = 'gnss_receiver_delivery'
    sequence: Count
    sample_time_ns: Count
    delivery_time_ns: Count
    receiver_available: StrictBool
    measurement_j2000: State6 | None

    @model_validator(mode='after')
    def actual_delivery(self):
        if self.receiver_available != (self.measurement_j2000 is not None):
            raise ValueError('receiver availability must match the actual delivered solution')
        if self.delivery_time_ns != self.sample_time_ns:
            raise ValueError('this receiver declares zero delivery latency')
        return self


class NavigationRecoveryContext(NavigationContext):
    task: NavigationRecoveryTask
