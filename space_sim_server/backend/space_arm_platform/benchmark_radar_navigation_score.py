"""Full target-orbit accuracy plus contemporaneous noncooperative sensor-delivery coverage."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

from .benchmark_contract import Contract
from .benchmark_navigation_score import NavigationScorer, initial_orbit_errors
from .benchmark_orbit_score import orbital_elements
from .benchmark_radar_navigation import RadarDelivery


class RadarNavigationScorer(NavigationScorer):
    def __init__(self, task, chaser_mass_kg):
        super().__init__(task, chaser_mass_kg)
        self.delivery_count = 0
        self.last_delivery_time_ns = None
        self.last_episode_id = None
        self.physical_pair = None
        self.range_squared_error = self.bearing_squared_error = self.range_rate_squared_error = 0.

    def _relative_estimate_truth(self, sample):
        return np.asarray(sample.target_com_j2000)

    def _initial_target_orbit_invalid(self, sample):
        initial = self.task.initial
        orbit = orbital_elements(sample.target_com_j2000)
        inclination = math.degrees(math.acos(float(np.clip(orbit['normal'][2], -1., 1.))))
        if abs(inclination-initial.inclination_deg) > initial.target_inclination_half_width_deg+1e-8:
            return True
        return initial_orbit_errors(sample.target_com_j2000, initial.target_altitude_m,
            initial.target_altitude_half_width_m, inclination, initial.true_anomaly_deg, initial.target_anomaly_half_width_deg)

    def _physical(self, sample):
        super()._physical(sample)
        self.physical_pair = np.array([sample.target_com_j2000, sample.chaser_com_j2000])

    def add(self, raw):
        document = raw.model_dump() if isinstance(raw, Contract) else raw
        if isinstance(document, dict) and document.get('record_type') == 'radar_delivery':
            try:
                observation = RadarDelivery.model_validate(document).observation
            except (ValueError, TypeError) as error:
                self.evidence_errors.add('invalid_radar_delivery')
                raise ValueError('invalid radar delivery evidence') from error
            expected = self.delivery_count*self.task.decision_period_ns
            if (self.physical_pair is None or observation.sequence != self.delivery_count
                    or observation.sample_time_ns != expected or self.last_time_ns != expected
                    or self.delivery_count > self.task.expected_estimates):
                self.evidence_errors.add('missing_duplicate_or_noncontemporaneous_radar_delivery')
                return
            if self.last_episode_id is not None and observation.episode_id != self.last_episode_id:
                self.evidence_errors.add('radar_delivery_from_another_episode')
            self.last_episode_id = observation.episode_id
            target, observer = self.physical_pair
            difference = target[:3]-observer[:3]
            distance = float(np.linalg.norm(difference))
            sensor = self.task.observation
            if not sensor.minimum_range_m <= distance <= sensor.maximum_range_m:
                self.evidence_errors.add('radar_delivery_outside_declared_sensor_envelope')
            direction = difference/distance
            measured = Rotation.from_quat(observation.orientation_body_to_j2000_wxyz, scalar_first=True).apply(observation.direction_body_unit)
            self.range_squared_error += (observation.range_m-distance)**2
            self.bearing_squared_error += math.atan2(float(np.linalg.norm(np.cross(measured, direction))), float(measured@direction))**2
            self.range_rate_squared_error += (observation.range_rate_m_s-float(direction@(target[3:]-observer[3:])))**2
            self.delivery_count += 1
            self.last_delivery_time_ns = expected
            return
        super().add(document)

    def _estimate(self, raw):
        if self.last_delivery_time_ns != self.last_time_ns:
            self.evidence_errors.add('estimate_without_current_radar_delivery')
        super()._estimate(raw)

    def result(self):
        score = super().result()
        evidence = set(score['evidence_errors'])
        if self.last_time_ns == round(self.task.duration_s*1e9) and self.delivery_count != self.task.expected_estimates+1:
            evidence.add('incomplete_radar_delivery_coverage')
        if evidence and score['outcome'] != 'invalid_initialization':
            score['outcome'] = 'invalid_evidence'
        score.update(schema_version='spacesim-autonomy-radar-navigation-score/1', success=score['outcome'] == 'success',
            completion_time_s=self.task.duration_s if score['outcome'] == 'success' else None,
            evidence_errors=sorted(evidence), radar_delivery_count=self.delivery_count,
            target_telemetry_available=False,
            range_measurement_rmse_m=math.sqrt(self.range_squared_error/self.delivery_count) if self.delivery_count else None,
            inertial_bearing_measurement_rmse_rad=math.sqrt(self.bearing_squared_error/self.delivery_count) if self.delivery_count else None,
            range_rate_measurement_rmse_m_s=math.sqrt(self.range_rate_squared_error/self.delivery_count) if self.delivery_count else None,
            evaluation_scope='full-duration absolute target orbit estimation; idealized active range/bearing/Doppler; no target telemetry or control authority')
        return score
