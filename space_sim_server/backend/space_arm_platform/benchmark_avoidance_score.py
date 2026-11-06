"""Independent full-horizon conjunction grading, without method risk flags."""
import math
import numpy as np

from .benchmark_avoidance import AvoidanceTruthSample
from .benchmark_orbit_score import orbital_elements, rotate_body_force


def tnr_frame(state):
    p, v = np.asarray(state[:3]), np.asarray(state[3:])
    radial = p/np.linalg.norm(p)
    normal = np.cross(p, v)
    normal /= np.linalg.norm(normal)
    tangent = np.cross(normal, radial)
    return np.column_stack((tangent, normal, radial)), np.cross(p,v)/float(p@p)


class AvoidanceScorer:
    def __init__(self, task, chaser_mass_kg):
        if not math.isfinite(chaser_mass_kg) or chaser_mass_kg <= 0:
            raise ValueError('positive model mass required')
        self.task, self.mass = task, chaser_mass_kg
        self.count = 0
        self.last_time_ns = self.outward_start = self.coast_start = None
        self.last_relative = None
        self.minimum_range = math.inf
        self.closest_time_ns = None
        self.maximum_speed = self.impulse = self.outward_dwell = self.coast_dwell = 0.
        self.final_range = self.final_rate = self.initial_predicted_miss = None
        self.evidence_errors, self.safety_violations = set(), set()

    def _initial(self, sample, elements, relative, relative_velocity):
        task, initial, encounter = self.task, self.task.initial, self.task.encounter
        inc = math.radians(initial.inclination_deg)
        normal = np.array([0., -math.sin(inc), math.cos(inc)])
        p = np.asarray(sample.chaser_com_j2000[:3])
        phase = math.degrees(math.atan2(p[1]*math.cos(inc)+p[2]*math.sin(inc), p[0]))
        if (abs(elements['altitude_m']-initial.altitude_m) > initial.altitude_half_width_m+1e-5
                or elements['eccentricity'] > 1e-8 or np.linalg.norm(elements['normal']-normal) > 1e-8
                or abs((phase-initial.true_anomaly_deg+180)%360-180) > initial.anomaly_half_width_deg+1e-8):
            self.evidence_errors.add('initial_spacecraft_orbit_outside_distribution')
        basis, omega = tnr_frame(sample.chaser_com_j2000)
        p_tnr = basis.T@relative
        v_tnr = basis.T@(relative_velocity-np.cross(omega,relative))
        if (np.max(np.abs(p_tnr-encounter.relative_position_tnr_m)) > encounter.position_half_width_m+1e-5
                or np.max(np.abs(v_tnr-encounter.relative_velocity_tnr_m_s)) > encounter.velocity_half_width_m_s+1e-7):
            self.evidence_errors.add('initial_debris_state_outside_distribution')
        vv = float(v_tnr@v_tnr)
        tca = -float(p_tnr@v_tnr)/vv if vv > 0 else -1.
        self.initial_predicted_miss = float(np.linalg.norm(p_tnr+max(tca,0.)*v_tnr))
        if not (task.objective.encounter_window_start_s < tca < task.objective.encounter_window_end_s
                and self.initial_predicted_miss < task.objective.initial_predicted_miss_max_m):
            self.evidence_errors.add('initial_encounter_is_not_the_declared_screening_threat')

    def add(self, raw):
        try:
            sample = AvoidanceTruthSample.model_validate(raw)
            elements = orbital_elements(sample.chaser_com_j2000)
        except (ValueError, TypeError) as error:
            self.evidence_errors.add('invalid_physical_sample')
            raise ValueError('invalid avoidance evidence') from error
        expected = 0 if self.last_time_ns is None else self.last_time_ns+self.task.physics_period_ns
        valid_tick = sample.time_ns == expected
        if not valid_tick:
            self.evidence_errors.add('missing_duplicate_or_reordered_physics_sample')
            self.outward_start = self.coast_start = None
        if sample.time_ns > round(self.task.duration_s*1e9):
            self.evidence_errors.add('evidence_beyond_episode_time_limit')
        if sample.external_state_writes:
            self.evidence_errors.add('physical_state_overwritten_after_initialization')
        relative = np.asarray(sample.debris_com_j2000[:3])-sample.chaser_com_j2000[:3]
        velocity = np.asarray(sample.debris_com_j2000[3:])-sample.chaser_com_j2000[3:]
        distance, speed = float(np.linalg.norm(relative)), float(np.linalg.norm(velocity))
        rate = float(relative@velocity)/max(distance,1e-15)
        if self.count == 0:
            self._initial(sample, elements, relative, velocity)
        force = float(np.linalg.norm(rotate_body_force(sample.orientation_body_to_j2000_wxyz,
                                                     sample.applied_motor_force_body_n)))
        if sample.time_ns and valid_tick:
            self.impulse += force*self.task.physics_period_ns*1e-9
        # Check the straight chord between 50 Hz samples as well as its endpoints.
        # This is not a proof of continuous curved-trajectory or contact clearance.
        segment_min = distance
        if self.last_relative is not None and valid_tick:
            delta = relative-self.last_relative
            norm2 = float(delta@delta)
            u = float(np.clip(-self.last_relative@delta/norm2,0.,1.)) if norm2 else 0.
            segment_min = min(distance,float(np.linalg.norm(self.last_relative+u*delta)))
        if segment_min < self.minimum_range:
            self.minimum_range, self.closest_time_ns = segment_min, sample.time_ns
        limits, goal = self.task.safety, self.task.objective
        failures = {
            'minimum_range': segment_min < limits.minimum_range_m,
            'maximum_range': distance > limits.maximum_range_m,
            'relative_speed': speed > limits.maximum_relative_speed_m_s,
            'minimum_altitude': elements['altitude_m'] < limits.minimum_altitude_m,
            'maximum_altitude': elements['altitude_m'] > limits.maximum_altitude_m,
            'orbital_eccentricity_or_escape': elements['eccentricity'] > limits.maximum_eccentricity or elements['semimajor_axis_m'] is None,
            'applied_force': force > limits.maximum_force_n+1e-8,
            'delta_v_budget': self.impulse/self.mass > limits.maximum_delta_v_m_s+1e-9,
        }
        self.safety_violations.update(name for name, failed in failures.items() if failed)
        eligible = valid_tick and not self.evidence_errors
        if eligible and sample.time_ns >= round(goal.encounter_window_start_s*1e9) and rate >= goal.outward_speed_min_m_s:
            if self.outward_start is None:
                self.outward_start = sample.time_ns
            self.outward_dwell = (sample.time_ns-self.outward_start)*1e-9
        else:
            self.outward_start, self.outward_dwell = None, 0.
        if eligible and force <= goal.coast_force_tolerance_n:
            if self.coast_start is None:
                self.coast_start = sample.time_ns
            self.coast_dwell = (sample.time_ns-self.coast_start)*1e-9
        else:
            self.coast_start, self.coast_dwell = None, 0.
        self.final_range, self.final_rate = distance, rate
        self.maximum_speed = max(self.maximum_speed, speed)
        self.last_time_ns, self.last_relative = sample.time_ns, relative
        self.count += 1

    def result(self):
        goal = self.task.objective
        quality = []
        ended = self.last_time_ns == round(self.task.duration_s*1e9)
        if ended:
            if not round(goal.encounter_window_start_s*1e9) <= (self.closest_time_ns or 0) <= round(goal.encounter_window_end_s*1e9):
                quality.append('closest_approach_outside_encounter_window')
            if self.final_range < goal.final_range_min_m or self.outward_dwell+1e-9 < goal.outward_dwell_s:
                quality.append('outward_pass_not_confirmed')
            if self.coast_dwell+1e-9 < goal.final_coast_s:
                quality.append('final_unpowered_coast_not_confirmed')
            if self.impulse/self.mass < goal.minimum_maneuver_delta_v_m_s:
                quality.append('no_actual_avoidance_maneuver')
        outcome = ('invalid_evidence' if not self.count or self.evidence_errors else
            'safety_violation' if self.safety_violations else 'task_failure' if ended and quality else
            'success' if ended else 'incomplete')
        return dict(schema_version='spacesim-autonomy-debris-avoidance-score/1', task_fingerprint=self.task.fingerprint(),
            outcome=outcome, success=outcome == 'success', sample_count=self.count,
            final_time_s=self.last_time_ns*1e-9 if self.count else None,
            completion_time_s=self.task.duration_s if outcome == 'success' else None,
            minimum_range_m=self.minimum_range if self.count else None,
            minimum_range_margin_m=self.minimum_range-self.task.safety.minimum_range_m if self.count else None,
            closest_approach_time_s=self.closest_time_ns*1e-9 if self.closest_time_ns is not None else None,
            initial_screening_miss_distance_m=self.initial_predicted_miss,
            maximum_relative_speed_m_s=self.maximum_speed, final_range_m=self.final_range,
            final_range_rate_m_s=self.final_rate, outward_dwell_s=self.outward_dwell,
            final_coast_s=self.coast_dwell, thrust_impulse_n_s=self.impulse,
            thrust_delta_v_m_s=self.impulse/self.mass, fuel_kg=None, energy_j=None,
            resource_model='constant mass; native thrust integral; no propellant or electrical energy model',
            safety_sampling='50 Hz physical COM separation and segment chord checks; 400 Hz native integration',
            collision_probability=None, evaluation_scope='deterministic separation-constrained single conjunction; not operational Pc certification',
            evidence_errors=sorted(self.evidence_errors), safety_violations=sorted(self.safety_violations), quality_violations=quality)
