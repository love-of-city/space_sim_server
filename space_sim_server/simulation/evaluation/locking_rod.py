"""Independent measured-state rod cycle scoring; no controller flags or stages."""
import math

import numpy as np


class LockingRodScorer:
    version = 'space-arm-locking-rod-score/1'

    def __init__(self, task='locking_rod_cycle'):
        if task != 'locking_rod_cycle':
            raise ValueError('Unsupported rod scoring task')
        self.last = self.hold = self.overload = None
        self.samples, self.progress = 0, 0
        self.error = self.safety_failure = None
        self.success = False
        self.max_force = self.max_torque = 0.
        self.milestone_times = []
        self.final_metrics = {}

    def update(self, row):
        if self.error is not None:
            return
        try:
            t = float(row['time_s'])
            q, wrench, contacts = [np.asarray(row[key], dtype=float) for key in
                                  ('joint_position', 'wrench', 'contacts')]
            scalar = [float(row[key]) for key in ('rod_angle_rad', 'rod_velocity_rad_s',
                'fastener_slide_m', 'fastener_spin_rad', 'fastener_spin_velocity_rad_s',
                'hook_normal_force_n', 'tool_fastener_distance_m')]
            angle, rate, slide, spin, spin_rate, normal, distance = scalar
            if q.shape != (8,) or wrench.shape != (6,) or contacts.shape != (2,):
                raise ValueError('Invalid rod evidence dimensions')
            if not math.isfinite(t) or t < 0 or not np.isfinite(np.r_[q, wrench, contacts, scalar]).all():
                raise ValueError('Non-finite rod observation')
            if np.any(contacts < 0) or normal < 0 or distance < 0:
                raise ValueError('Invalid measured contacts or distance')
            bootstrap = self.samples == 0 and t == 0 and row['wrench_valid'] is False and np.all(wrench == 0)
            if row['wrench_valid'] is not True and not bootstrap:
                raise ValueError('Missing wrench measurement')
            if self.last is None:
                if t > .000001 or abs(angle) > .02 or abs(slide) > .0001:
                    raise ValueError('Missing initially restored and loose rod evidence')
            elif not 0 < t-self.last <= .005001:
                raise ValueError('Missing or nonmonotonic 200Hz rod evidence')
            force, torque = np.linalg.norm(wrench[:3]), np.linalg.norm(wrench[3:])
            self.max_force, self.max_torque = max(self.max_force, force), max(self.max_torque, torque)
            if force > 40. or torque > 4.:
                if self.overload is None:
                    self.overload = t
                if t-self.overload >= .1-1e-9:
                    self.safety_failure = 'wrist_overload'
            else:
                self.overload = None
            locked = abs(angle) < .025 and .004 < slide < .007 and normal > 1. and abs(spin_rate) < .01
            helix_ok = abs(slide-spin*.016/(2*np.pi)) < .00015
            grasped = np.all(contacts > 0) and q[6] > .05
            predicates = [grasped, grasped and angle < -.45 and abs(rate) < .01,
                grasped and abs(angle) < .02 and abs(rate) < .01,
                grasped and locked and helix_ok,
                locked and helix_ok and q[6] > .95 and np.all(contacts == 0) and distance > .12]
            ready = predicates[min(self.progress, 4)]
            if ready:
                if self.hold is None:
                    self.hold = t
            else:
                self.hold = None
            duration = .15 if self.progress == 0 else .3 if self.progress < 4 else 2.
            if self.hold is not None and t-self.hold >= duration-1e-9 and self.progress < 4:
                self.milestone_times.append(t)
                self.progress += 1
                self.hold = None
            self.success = bool(self.progress == 4 and self.hold is not None
                                and t-self.hold >= 2.-1e-9 and not self.safety_failure)
            self.last, self.samples = t, self.samples+1
            self.final_metrics = dict(rod_angle_rad=angle, fastener_slide_m=slide, hook_normal_force_n=normal,
                tool_fastener_distance_m=distance, helix_error_m=slide-spin*.016/(2*np.pi))
        except (KeyError, ValueError, TypeError, IndexError) as error:
            self.error, self.success = str(error), False

    def report(self):
        valid = self.samples > 1 and self.error is None
        return dict(version=self.version, success=bool(valid and self.success),
            status='unscorable' if not valid else 'success' if self.success else 'task_failure',
            reason=self.error or self.safety_failure or ('complete' if self.success else 'cycle_incomplete'),
            samples=self.samples, measured_milestones=self.progress,
            milestone_times_s=self.milestone_times, final_metrics=self.final_metrics,
            max_force_n=float(self.max_force), max_torque_nm=float(self.max_torque),
            terminal_hold_s=0. if self.hold is None or self.last is None else self.last-self.hold,
            scope='simulated_arm_driven_fold_restore_lock_release_not_hardware_validation')


def physical_row(sample, task):
    p, _ = task.chain.forward(sample.joints[:6])
    return dict(time_s=float(sample.time_s), joint_position=sample.joints.tolist(),
        wrench=sample.wrench.tolist(), wrench_valid=bool(sample.wrench_valid), contacts=list(sample.contacts),
        rod_angle_rad=sample.rod_angle_rad, rod_velocity_rad_s=sample.rod_velocity_rad_s,
        fastener_slide_m=sample.fastener_slide_m, fastener_spin_rad=sample.fastener_spin_rad,
        fastener_spin_velocity_rad_s=sample.fastener_spin_velocity_rad_s,
        hook_normal_force_n=sample.hook_normal_force_n,
        tool_fastener_distance_m=float(np.linalg.norm(p-sample.fastener_position_body_m)))
