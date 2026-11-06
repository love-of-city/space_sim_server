"""Independent outage delivery coverage, coasting accuracy and confirmed recovery."""
from .benchmark_contract import Contract
from .benchmark_navigation_recovery import NavigationRecoveryDelivery, NavigationRecoveryTruthSample
from .benchmark_navigation_score import NavigationScorer


class NavigationRecoveryScorer(NavigationScorer):
    def __init__(self, task, chaser_mass_kg):
        super().__init__(task, chaser_mass_kg)
        self.outage = None
        self.delivery_count = self.missing_measurements = self.outage_estimate_count = 0
        self.last_delivery_time_ns = None
        self.peak_outage_position_error = self.peak_outage_velocity_error = 0.
        self.recovery_hold_start_ns = self.recovery_confirmed_ns = None
        self.longest_recovered_hold_ns = 0

    def add(self, raw):
        document = raw.model_dump() if isinstance(raw, Contract) else raw
        if isinstance(document, dict) and document.get('record_type') == 'navigation_estimate':
            super().add(document)
            return
        if isinstance(document, dict) and document.get('record_type') == 'gnss_receiver_delivery':
            self._delivery(document)
            return
        try:
            sample = NavigationRecoveryTruthSample.model_validate(document)
            sample.outage.validate_for(self.task)
            if self.outage is None:
                self.outage = sample.outage
            if sample.outage != self.outage:
                self.evidence_errors.add('receiver_outage_realization_changed')
            if sample.receiver_available != self.outage.available(sample.time_ns):
                self.evidence_errors.add('receiver_availability_does_not_match_fault')
            super().add(sample.model_dump(exclude={'outage', 'receiver_available'}))
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_receiver_physical_evidence')
            raise ValueError('invalid receiver recovery physical evidence') from error

    def _delivery(self, document):
        try:
            delivery = NavigationRecoveryDelivery.model_validate(document)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_receiver_delivery')
            raise ValueError('invalid receiver delivery') from error
        expected_time = self.delivery_count*self.task.decision_period_ns
        if (self.outage is None or delivery.sequence != self.delivery_count
                or delivery.sample_time_ns != expected_time or self.last_time_ns != expected_time
                or self.delivery_count > self.task.expected_estimates):
            self.evidence_errors.add('missing_duplicate_or_noncontemporaneous_receiver_delivery')
            return
        if delivery.receiver_available != self.outage.available(expected_time):
            self.evidence_errors.add('solution_delivered_during_receiver_outage_or_missing_after_return')
        self.missing_measurements += int(not delivery.receiver_available)
        self.last_delivery_time_ns = expected_time
        self.delivery_count += 1

    def _estimate(self, raw):
        previous_count = self.estimate_count
        if self.last_delivery_time_ns != self.last_time_ns:
            self.evidence_errors.add('estimate_without_current_receiver_delivery')
        super()._estimate(raw)
        if self.estimate_count == previous_count or self.outage is None:
            return
        now = self.last_time_ns
        if not self.outage.available(now):
            self.outage_estimate_count += 1
            self.peak_outage_position_error = max(self.peak_outage_position_error, self.final_position_error)
            self.peak_outage_velocity_error = max(self.peak_outage_velocity_error, self.final_velocity_error)
        goal = self.task.objective
        if now >= self.outage.return_time_ns:
            valid = (not self.evidence_errors and self.final_position_error <= goal.recovered_position_tolerance_m
                     and self.final_velocity_error <= goal.recovered_velocity_tolerance_m_s)
            if valid:
                if self.recovery_hold_start_ns is None:
                    self.recovery_hold_start_ns = now
                held = now-self.recovery_hold_start_ns
                self.longest_recovered_hold_ns = max(self.longest_recovered_hold_ns, held)
                if held >= round(goal.recovered_dwell_s*1e9) and self.recovery_confirmed_ns is None:
                    self.recovery_confirmed_ns = now
            else:
                self.recovery_hold_start_ns = None

    def result(self):
        score = super().result()
        goal = self.task.objective
        quality = set(score['quality_violations'])
        evidence = set(score['evidence_errors'])
        if self.outage_estimate_count:
            if self.peak_outage_position_error > goal.outage_maximum_position_error_m:
                quality.add('outage_position_error')
            if self.peak_outage_velocity_error > goal.outage_maximum_velocity_error_m_s:
                quality.add('outage_velocity_error')
        complete = self.last_time_ns == round(self.task.duration_s*1e9)
        latency = (None if self.recovery_confirmed_ns is None else
                   (self.recovery_confirmed_ns-self.outage.return_time_ns)*1e-9)
        expected_missing = (None if self.outage is None else
            (self.outage.return_time_ns-self.outage.onset_time_ns)//self.task.decision_period_ns)
        if complete:
            if (self.outage is None or self.delivery_count != self.task.expected_estimates+1
                    or self.missing_measurements != expected_missing or self.outage_estimate_count != expected_missing):
                evidence.add('incomplete_receiver_outage_delivery_or_estimate_coverage')
            if latency is None or latency > goal.recovery_deadline_after_return_s:
                quality.add('navigation_recovery_deadline')
        if evidence:
            score['outcome'] = 'invalid_evidence'
        elif score['outcome'] == 'success' and quality:
            score['outcome'] = 'task_failure'
        score.update(schema_version='spacesim-autonomy-navigation-recovery-score/1',
            success=score['outcome'] == 'success', quality_violations=sorted(quality), evidence_errors=sorted(evidence),
            completion_time_s=self.task.duration_s if score['outcome'] == 'success' else None,
            receiver_delivery_count=self.delivery_count, missing_measurement_count=self.missing_measurements,
            expected_missing_measurements=expected_missing, outage_estimate_count=self.outage_estimate_count,
            maximum_outage_position_error_m=self.peak_outage_position_error if self.outage_estimate_count else None,
            maximum_outage_velocity_error_m_s=self.peak_outage_velocity_error if self.outage_estimate_count else None,
            outage_onset_s=None if self.outage is None else self.outage.onset_time_ns*1e-9,
            receiver_return_s=None if self.outage is None else self.outage.return_time_ns*1e-9,
            recovery_confirmation_latency_s=latency, longest_recovered_dwell_s=self.longest_recovered_hold_ns*1e-9,
            evaluation_scope='full-duration estimation across actual withheld GNSS solutions and confirmed reacquisition; no control authority')
        return score
