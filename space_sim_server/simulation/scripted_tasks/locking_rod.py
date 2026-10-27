"""Contact-driven loosen/fold/restore/tighten cycle for the captive locking rod.

The rod keeps its delivered bottom hinge and the nut stays captive.  This task
only publishes the eight existing arm/finger references; scalar mechanism
coordinates and contact evidence are measurements, never control ports.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simulation.serial_chain_kinematics import axis_angle_to_matrix
from .pose_tracking import PoseTargetFilter
from .threaded_motion import (
    ThreadedMotionConfig, ThreadedSample, ThreadedTaskBase, model_body_pose,
)


@dataclass(frozen=True)
class LockingRodConfig(ThreadedMotionConfig):
    part_body: str = "fastener"
    grasp_offset_m: tuple[float, float, float] = (0., 0., -.005)
    fold_grasp_offset_m: tuple[float, float, float] = (0., 0., .010)
    folded_hinge_rad: float = -1.53
    hinge_tolerance_rad: float = .025
    returned_hinge_tolerance_rad: float = .025
    loose_slide_tolerance_m: float = .00035
    tight_slide_min_m: float = .0044
    tight_slide_max_m: float = .0065
    helix_tolerance_m: float = .00010
    minimum_hook_force_n: float = 1.
    maximum_seat_backoff_m: float = .00035
    release_clearance_rad: float = .08
    retreat_distance_m: float = .065
    regrasp_retreat_m: float = .14
    fold_approach_clearance_m: float = .010
    compact_regrasp_distance_m: float = .065
    clip_regrasp_distance_m: float = .030

    def __post_init__(self):
        # The signed hinge endpoint is a coordinate, not a positive speed.
        signed = self.folded_hinge_rad
        object.__setattr__(self, "folded_hinge_rad", abs(signed))
        try:
            super().__post_init__()
        finally:
            object.__setattr__(self, "folded_hinge_rad", signed)
        if self.part_body != "fastener":
            raise ValueError("The locking task must grasp the captive fastener")
        if not math.isfinite(signed) or signed >= -.8:
            raise ValueError("The folded endpoint must reach the physical limit clip")
        if self.tight_slide_max_m <= self.tight_slide_min_m:
            raise ValueError("Invalid fastener seating interval")
        if min(self.compact_regrasp_distance_m, self.clip_regrasp_distance_m,
               self.pregrasp_distance_m) <= self.fold_approach_clearance_m:
            raise ValueError("Regrasp withdrawal must stay outside the axial contact zone")
        if np.shape(self.grasp_offset_m) != (3,) or not np.isfinite(self.grasp_offset_m).all():
            raise ValueError("Invalid fastener grasp offset")
        if np.shape(self.fold_grasp_offset_m) != (3,) or not np.isfinite(self.fold_grasp_offset_m).all():
            raise ValueError("Invalid fold grasp offset")


class LockingRodTask(ThreadedTaskBase):
    name = "locking_rod_cycle"
    version = "locking-rod-cycle/2-coaxial-latch"
    stages = (
        "settle", "approach", "grip", "loosen", "fold_release", "fold_clear",
        "fold_approach", "fold_grip", "lower", "release_down", "clear_down",
        "reapproach", "regrip", "raise", "turn_release", "turn_clear",
        "turn_approach", "turn_grip", "tighten", "release", "retract", "verify",
    )
    _required_scalars = ("locking_rod_hinge", "fastener_slide", "fastener_spin")
    _closing_local = np.array([0., 1., 0.])
    _approach_local = np.array([0., 0., 1.])
    _fold_closing_local = np.array([-math.sqrt(3)/2, .5, 0.])
    _fold_approach_local = np.array([math.sqrt(3)/4, .75, .5])

    @classmethod
    def recorded_stages(cls, version=None):
        if version not in (None, cls.version):
            raise ValueError(f"Unsupported locking rod task version: {version}")
        return cls.stages

    @classmethod
    def validate_recorded_stages(cls, stages, version=None):
        """Require every physical operation, allowing repeated sampled labels."""
        expected = cls.recorded_stages(version)
        compact = tuple(name for i, name in enumerate(stages) if i == 0 or name != stages[i-1])
        return compact == expected

    def __init__(self, model_path, config=None):
        super().__init__(model_path, config or LockingRodConfig())
        rod = self.root.find('.//body[@name="locking_rod"]')
        nut = self.root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
        hinge = None if rod is None else rod.find('joint[@name="locking_rod_hinge"]')
        if nut is None or hinge is None or hinge.get("type", "hinge") != "hinge":
            raise ValueError("Task requires the original hinged rod and captive fastener")
        spin = nut.find('joint[@name="fastener_spin"]')
        slide = nut.find('joint[@name="fastener_slide"]')
        coupling = self.root.find('./equality/joint[@name="fastener_helix"]')
        if spin is None or slide is None or coupling is None:
            raise ValueError("Task requires measured helical spin/slide coordinates")
        coefficients = np.fromstring(coupling.get("polycoef", ""), sep=" ")
        if (coupling.get("joint1") != "fastener_spin" or coupling.get("joint2") != "fastener_slide"
                or len(coefficients) < 2 or coefficients[1] <= 0
                or not np.allclose(coefficients[2:], 0.)):
            raise ValueError("Unsupported fastener helix coupling")
        self.lead_m = 2*math.pi/coefficients[1]
        self.helix_offset_rad = float(coefficients[0])
        self.hinge_ref = float(hinge.get("ref", "0"))
        self.spin_ref = float(spin.get("ref", "0"))
        self.slide_ref = float(slide.get("ref", "0"))
        self.hinge_range = np.fromstring(hinge.get("range", ""), sep=" ")
        if (self.hinge_range.shape != (2,) or not self.hinge_range[0] <= self.config.folded_hinge_rad <= self.hinge_range[1]):
            raise ValueError("Locking rod hinge cannot reach its physical limit clip")
        self.rod_position, self.rod_rotation = model_body_pose(self.root, "locking_rod")
        local_pivot = np.fromstring(hinge.get("pos", "0 0 0"), sep=" ")
        local_axis = np.fromstring(hinge.get("axis", "0 0 1"), sep=" ")
        self.pivot = self.rod_position + self.rod_rotation @ local_pivot
        self.hinge_axis = self.rod_rotation @ (local_axis/np.linalg.norm(local_axis))
        self.part_reference_position, self.part_reference_rotation = model_body_pose(self.root, "fastener")
        # Fail early if an unrelated model drops either physical mating part.
        model_body_pose(self.root, "hook")
        model_body_pose(self.root, "limit_clip")
        self._initial_spin = self._initial_slide = None
        self._spin_command = self._hinge_command = 0.
        self._speed = self._acceleration = 0.
        self._release_initial_opening = 0.
        self._released_stable = 0.
        self._tighten_peak_hook_force_n = 0.
        self._tighten_seated_coordinates = None
        self._seating_reference = None
        self._grasp_mode = "turn_latch"
        self._grip_hold = self._grip_filter = None
        self.loosen_verified = self.folded_verified = False
        self.returned_verified = self.tightened_verified = False

    def mechanism_pose(self, hinge, slide, spin):
        """Exact reference kinematics, including task-specific nonzero joint refs."""
        rh = axis_angle_to_matrix(self.hinge_axis, hinge-self.hinge_ref)
        axis = self.part_reference_rotation[:, 2]
        position = self.pivot + rh @ (self.part_reference_position-self.pivot + axis*(slide-self.slide_ref))
        rotation = rh @ self.part_reference_rotation @ axis_angle_to_matrix(np.array([0., 0., 1.]), spin-self.spin_ref)
        return position, rotation

    def _grasp_pose(self, sample, *, offset_local=None):
        folding = self._grasp_mode == "fold_support"
        return super()._grasp_pose(
            sample, closing_local=self._fold_closing_local if folding else self._closing_local,
            approach_local=self._fold_approach_local if folding else self._approach_local,
            offset_local=(self.config.fold_grasp_offset_m if folding else self.config.grasp_offset_m)
            if offset_local is None else offset_local,
        )

    def _coordinates(self, sample):
        return tuple(float(sample.scalar_joints[name]) for name in self._required_scalars)

    def _helix_error(self, sample):
        _, slide, spin = self._coordinates(sample)
        return abs(slide-(spin-self.helix_offset_rad)*self.lead_m/(2*math.pi))

    def _at_clip(self, sample):
        hinge, _, _ = self._coordinates(sample)
        contacts = sum(sample.scalar_joints.get(key, 0.) for key in (
            "locking_rod_clip_contacts", "fastener_clip_contacts"))
        return bool(abs(hinge-self.config.folded_hinge_rad) <= self.config.hinge_tolerance_rad and contacts > 0)

    def _closed_coordinates(self, sample):
        hinge, slide, _ = self._coordinates(sample)
        return bool(abs(hinge) <= self.config.returned_hinge_tolerance_rad
                    and self.config.tight_slide_min_m <= slide <= self.config.tight_slide_max_m
                    and self._helix_error(sample) <= self.config.helix_tolerance_m)

    def _at_hook(self, sample):
        return bool(self._closed_coordinates(sample)
                    and sample.scalar_joints.get("fastener_hook_contacts", 0.) > 0
                    and sample.scalar_joints.get("fastener_hook_normal_force_n", 0.) >= self.config.minimum_hook_force_n)

    def _released_locked(self, sample):
        """Retained geometry after a measured, contact-confirmed tightening.

        In orbit the unloaded hook may carry zero reaction.  This is valid
        only after actual tightening and only while the thread retains the
        achieved seating coordinates within a small measured backoff.
        """
        if (not self.tightened_verified or self._tighten_seated_coordinates is None
                or self._tighten_peak_hook_force_n < self.config.minimum_hook_force_n):
            return False
        _, slide, spin = self._coordinates(sample)
        seated_slide, seated_spin = self._tighten_seated_coordinates
        backoff = self.config.maximum_seat_backoff_m
        return bool(self._closed_coordinates(sample)
                    and slide >= seated_slide-backoff
                    and spin >= seated_spin-backoff*2*math.pi/self.lead_m)

    def _initially_locked(self, sample):
        """A seated unloaded latch need not have a permanent contact preload.

        The episode-only model must encode the matching threaded closed rest
        transform.  Raw measured coordinates and body pose must still match it;
        Final tightening requires a measured reaction before release; unloaded
        final verification checks retained geometry and the achieved thread seat.
        """
        hinge, slide, spin = self._coordinates(sample)
        marker = self.root.find('./custom/numeric[@name="scripted_locking_cycle_initial_state"]')
        expected, _ = self.mechanism_pose(hinge, slide, spin)
        return bool(marker is not None
                    and self.config.tight_slide_min_m <= self.slide_ref <= self.config.tight_slide_max_m
                    and abs(hinge) <= self.config.returned_hinge_tolerance_rad
                    and abs(slide-self.slide_ref) <= .00015
                    and abs(spin-self.spin_ref) <= .06
                    and self._helix_error(sample) <= self.config.helix_tolerance_m
                    and np.linalg.norm(sample.part_position_body_m-expected) <= .001
                    and self._stationary(sample, .10))

    def _stationary(self, sample, angular=.05):
        return bool(np.linalg.norm(sample.part_velocity_body_m_s) < .008
                    and np.linalg.norm(sample.part_angular_velocity_body_rad_s) < angular
                    and abs(sample.scalar_joints.get("locking_rod_hinge_velocity", math.inf)) < angular
                    and abs(sample.scalar_joints.get("fastener_spin_velocity", math.inf)) < angular)

    def metrics(self, sample):
        values = super().metrics(sample)
        _, tool_rotation = self.chain.forward(sample.joints[:6])
        closing = sample.part_rotation_body @ (self._fold_closing_local if self._grasp_mode == "fold_support" else self._closing_local)
        heading = float(np.arccos(np.clip(abs(tool_rotation[:, 0] @ closing), 0., 1.)))
        available = all(key in sample.scalar_joints for key in self._required_scalars)
        hinge, slide, spin = self._coordinates(sample) if available else (None, None, None)
        values.update(
            hinge_rad=hinge, fastener_slide_m=slide, fastener_spin_rad=spin,
            fastener_thread_lead_m=self.lead_m,
            fastener_helix_error_m=self._helix_error(sample) if available else None,
            folded_target_hinge_rad=self.config.folded_hinge_rad,
            loosen_verified=self.loosen_verified, folded_verified=self.folded_verified,
            returned_verified=self.returned_verified, tightened_verified=self.tightened_verified,
            released_stable_seconds=self._released_stable,
            fastener_hook_contacts=sample.scalar_joints.get("fastener_hook_contacts"),
            fastener_hook_normal_force_n=sample.scalar_joints.get("fastener_hook_normal_force_n"),
            locking_rod_clip_contacts=sample.scalar_joints.get("locking_rod_clip_contacts"),
            fastener_clip_contacts=sample.scalar_joints.get("fastener_clip_contacts"),
            tighten_peak_hook_force_n=self._tighten_peak_hook_force_n,
            tighten_seated_slide_m=(None if self._tighten_seated_coordinates is None
                                    else self._tighten_seated_coordinates[0]),
            tighten_seated_spin_rad=(None if self._tighten_seated_coordinates is None
                                     else self._tighten_seated_coordinates[1]),
            grasp_heading_error_rad=heading,
            grasp_mode=self._grasp_mode,
        )
        return values

    def _approach_clearance(self, stage):
        if stage == "fold_approach" or (self.config.speed_profile == "fast_collection"
                                         and stage in ("reapproach", "turn_approach")):
            return self.config.fold_approach_clearance_m
        return None

    def _clear_pose(self, stage, sample):
        """Use short, geometry-checked withdrawals between real grasps.

        Changing from axial turning to inclined support still needs an
        axial withdrawal for arm reach.  Its former 14 cm detour is reduced
        to 6.5 cm; the next approach rotates while moving.  The opposite
        change can lift and rotate inward together.  Actual release is
        confirmed by the preceding stage before any withdrawal starts.
        """
        p, r = self.chain.forward(sample.joints[:6])
        compact = self.config.speed_profile == "fast_collection"
        if stage == "turn_clear":
            self._grasp_mode = "turn_latch"
            self._grasp_sign = None
            gp, gr = self._grasp_pose(sample)
            distance = self.config.pregrasp_distance_m if compact else self.config.retreat_distance_m
            return gp-distance*gr[:, 2], gr
        if stage == "fold_clear":
            distance = self.config.compact_regrasp_distance_m if compact else self.config.regrasp_retreat_m
        elif compact and stage == "clear_down":
            distance = self.config.clip_regrasp_distance_m
        else:
            distance = self.config.retreat_distance_m
        return p-distance*r[:, 2], r

    def _enter(self, stage, sample):
        if stage in ("approach", "reapproach", "fold_approach", "turn_approach"):
            self._grasp_mode = "fold_support" if stage in ("fold_approach", "reapproach") else "turn_latch"
        elif stage == "turn_clear":
            self._grasp_mode = "turn_latch"
        self._set_stage(stage, sample)
        self._speed = self._acceleration = 0.
        hinge, _, spin = self._coordinates(sample)
        self._hinge_command, self._spin_command = hinge, spin
        if stage in ("approach", "reapproach", "fold_approach", "turn_approach"):
            self._grasp_sign = None
            p, r = self._grasp_pose(sample)
            self._plan_pose(p, r, opening=self.config.approach_open_rad, approach=True,
                            approach_clearance=self._approach_clearance(stage))
        elif stage in ("grip", "regrip", "fold_grip", "turn_grip"):
            self._grip_hold = None
            self._grip_filter = PoseTargetFilter(*self._grasp_pose(sample))
            self._start_opening(0.)
        elif stage in ("lower", "raise"):
            self._plan_swing(sample, self.config.folded_hinge_rad if stage == "lower" else 0.)
        elif stage == "tighten":
            self._seating_reference = None
        elif stage in ("release_down", "release", "fold_release", "turn_release"):
            self._release_initial_opening = float(max(sample.joints[6], -sample.joints[7]))
            p, r = self.chain.forward(sample.joints[:6])
            self._plan_pose(p, r)
            self._start_opening(self.config.release_open_rad, sample, unload=True)
        elif stage in ("clear_down", "retract", "fold_clear", "turn_clear"):
            self._plan_pose(*self._clear_pose(stage, sample))

    def _smooth_rate(self, target, dt):
        if dt <= 0:
            return self._speed
        acceleration = np.clip((target-self._speed)/dt,
                               -self.config.rotation_acceleration_rad_s2, self.config.rotation_acceleration_rad_s2)
        self._acceleration += float(np.clip(acceleration-self._acceleration,
                                           -self.config.rotation_jerk_rad_s3*dt, self.config.rotation_jerk_rad_s3*dt))
        previous = self._speed
        self._speed += self._acceleration*dt
        if (target-previous)*(target-self._speed) < 0:
            self._speed, self._acceleration = target, 0.
        return self._speed

    def _rotate_fastener(self, sample, dt, goal):
        _, _, spin = self._coordinates(sample)
        error = goal-spin
        target = float(np.clip(2.*error, -self.config.rotation_speed_rad_s, self.config.rotation_speed_rad_s))
        speed = self._smooth_rate(target, dt)
        self._spin_command = float(np.clip(self._spin_command+speed*dt, spin-.14, spin+.14))
        self._spin_command = min(goal, self._spin_command) if goal >= spin else max(goal, self._spin_command)
        desired_slide = (self._spin_command-self.helix_offset_rad)*self.lead_m/(2*math.pi)
        # Keep the rod at its mating hook while turning. Following its measured
        # hinge here would absorb a contact-induced swing into the new target.
        p, r = self.mechanism_pose(0., desired_slide, self._spin_command)
        axis = r[:, 2]
        self._track_part(p, r, sample, dt, axis*speed*self.lead_m/(2*math.pi), axis*speed)

    def _plan_swing(self, sample, goal):
        hinge, slide, spin = self._coordinates(sample)
        offset, relative = self._grasp_transform
        def tool_pose(u):
            # The captured nut spin/slide remain held during the swing. Do not
            # chase nut rolling/slip by redefining the desired grip each frame.
            position, rotation = self.mechanism_pose(hinge+(goal-hinge)*u, slide, spin)
            tool_rotation = rotation @ relative.T
            return position-tool_rotation @ offset, tool_rotation
        position, rotation = tool_pose(1.)
        radius = np.linalg.norm(np.cross(self.hinge_axis, sample.part_position_body_m-self.pivot))
        self._plan_pose(position, rotation, path_fn=tool_pose,
                        distance=radius*abs(goal-hinge), angle=abs(goal-hinge))

    def step(self, sample):
        dt, old = self._step_start(sample)
        if self.result is not None:
            return self._step_finish(sample, old, dt)
        if (any(key not in sample.scalar_joints for key in self._required_scalars)
                or not np.isfinite([sample.scalar_joints.get(k, math.nan) for k in self._required_scalars]).all()):
            self.fail("mechanism_state_unavailable", sample)
            return self._step_finish(sample, old, dt)
        if self.stage == "pending":
            self._initial_slide, self._initial_spin = self._coordinates(sample)[1:]
            self._enter("settle", sample)
        hinge, slide, spin = self._coordinates(sample)
        if self.stage == "settle":
            if sample.time_s-self._entered >= self.config.initial_settle_s:
                if not self._initially_locked(sample):
                    self.fail("initial_fastener_not_physically_tightened", sample)
                else:
                    self._enter("approach", sample)
        elif self.stage in ("approach", "reapproach", "fold_approach", "turn_approach"):
            if self._path_step(sample, dt):
                self._enter({"approach": "grip", "reapproach": "regrip",
                             "fold_approach": "fold_grip", "turn_approach": "turn_grip"}[self.stage], sample)
        elif self.stage in ("grip", "regrip", "fold_grip", "turn_grip"):
            p, r = self._grasp_pose(sample)
            p, r = self._grip_filter.update(p, r, dt)
            if self._grip_hold is not None:
                p, r = self._grip_hold
            self._follow_pose(p, r, sample, dt)
            if self._grip_hold is None:
                self._advance_opening(dt)
            if self._grasp_mode == "turn_latch":
                bilateral = self._latch_engaged(sample)
            else:
                actual_p, actual_r = self.chain.forward(sample.joints[:6])
                bilateral = (sample.finger_part_contacts is not None and all(sample.finger_part_contacts)
                             and np.linalg.norm(actual_p-p) < .003
                             and abs(actual_r[:, 0]@r[:, 0]) > math.cos(.05))
            self._stable = self._stable+dt if bilateral and self._stationary(sample, .10) else 0.
            if self._stable >= self.config.grip_confirm_s:
                if self._grip_hold is None:
                    # Stop adding closing preload after confirmed engagement.
                    # Brake the arm reference smoothly before handing it to a
                    # zero-speed swing path; actual clip contact alone did not
                    # prove that the pose-feedback reference had stopped.
                    self._grip_hold = self.chain.forward(self.reference[:6])
                elif not self._motion_stopped(sample):
                    pass
                elif self.stage == "regrip" and not self._at_clip(sample):
                    self.fail("rod_left_clip_before_regrasp", sample)
                else:
                    self._capture_grasp(sample)
                    self._enter({"grip": "loosen", "regrip": "raise", "fold_grip": "lower",
                                 "turn_grip": "tighten"}[self.stage], sample)
        elif self.stage == "loosen":
            self._rotate_fastener(sample, dt, self.helix_offset_rad)
            loose = (slide <= self.config.loose_slide_tolerance_m
                     and abs(spin-self.helix_offset_rad) <= .015
                     and self._initial_spin-spin > .9
                     and self._helix_error(sample) <= self.config.helix_tolerance_m
                     and sample.scalar_joints.get("fastener_hook_normal_force_n", math.inf) < self.config.minimum_hook_force_n
                     and self._motion_stopped(sample))
            if loose:
                self.loosen_verified = True
                self._enter("fold_release", sample)
        elif self.stage == "lower":
            done = self._path_step(sample, dt)
            self._stable = self._stable+dt if self._at_clip(sample) and self._stationary(sample) else 0.
            if done and self._stable >= self.config.terminal_confirm_s and self._motion_stopped(sample):
                self.folded_verified = True
                self._enter("release_down", sample)
        elif self.stage in ("release_down", "release", "fold_release", "turn_release"):
            unloaded = self._path_step(sample, dt)
            self._advance_opening(dt)
            contacts_clear = sample.finger_part_contacts is not None and sum(sample.finger_part_contacts) == 0
            opened = max(sample.joints[6], -sample.joints[7]) >= self._release_initial_opening+self.config.release_clearance_rad
            if contacts_clear and opened and unloaded and self._motion_stopped(sample):
                self._enter({"release_down": "clear_down", "release": "retract",
                             "fold_release": "fold_clear", "turn_release": "turn_clear"}[self.stage], sample)
        elif self.stage in ("clear_down", "retract", "fold_clear", "turn_clear"):
            done = self._path_step(sample, dt)
            self._advance_opening(dt)
            if self.stage == "clear_down" and not self._at_clip(sample):
                self.fail("released_rod_not_retained_by_clip", sample)
            elif done:
                self._enter({"clear_down": "reapproach", "retract": "verify",
                             "fold_clear": "fold_approach", "turn_clear": "turn_approach"}[self.stage], sample)
        elif self.stage == "raise":
            done = self._path_step(sample, dt)
            if done and abs(hinge) <= self.config.returned_hinge_tolerance_rad and self._stationary(sample):
                self.returned_verified = True
                self._enter("turn_release", sample)
        elif self.stage == "tighten":
            # The actual mating hook, rather than an artificial travel stop,
            # provides the seating reaction.  Modest reference preload remains
            # an arm servo target; no mechanism force or state is written.
            self._tighten_peak_hook_force_n = max(self._tighten_peak_hook_force_n,
                                                sample.scalar_joints.get("fastener_hook_normal_force_n", 0.))
            if self._seating_reference is None:
                self._rotate_fastener(sample, dt, self._initial_spin+.12)
                self._stable = self._stable+dt if self._at_hook(sample) and self._stationary(sample) else 0.
                if self._stable >= self.config.terminal_confirm_s:
                    # A first touch can be a transient collision. Confirm a
                    # stationary seat under real load before holding its
                    # reference; stopping at first touch unloaded the hook.
                    self._seating_reference = self.mechanism_pose(
                        0., (self._spin_command-self.helix_offset_rad)*self.lead_m/(2*math.pi),
                        self._spin_command)
                    self._tighten_seated_coordinates = (slide, spin)
                    self.tightened_verified = True
            else:
                self._track_part(*self._seating_reference, sample, dt)
                if self._released_locked(sample) and self._stationary(sample) and self._motion_stopped(sample):
                    self._enter("release", sample)
        elif self.stage == "verify":
            self._advance_opening(dt)
            clear = sample.finger_part_contacts is not None and sum(sample.finger_part_contacts) == 0
            completed = all((self.loosen_verified, self.folded_verified,
                             self.returned_verified, self.tightened_verified))
            valid = completed and clear and self._released_locked(sample) and self._stationary(sample)
            self._released_stable = self._released_stable+dt if valid else 0.
            if self._released_stable >= self.config.verify_hold_s:
                self._succeed(sample)
        return self._step_finish(sample, old, dt)

    def preflight_paths(self, initial_q):
        """Nominal arm paths; physics preflight must also replay mechanism q.

        The task never uses these artificial samples at runtime.  They only
        describe candidate arm references to the private collision probe.
        """
        preview = type(self)(self.model_path, self.config)
        preview.reference[:] = np.asarray(initial_q, dtype=float)
        if preview.reference.shape != (8,):
            raise ValueError("Preflight requires eight initial joint coordinates")
        values = []
        mechanism_state = (0., self.slide_ref, self.spin_ref)
        def record(stage, p, r, *, opening=None, path_fn=None, distance=None, angle=None,
                   approach=False, mechanism_fn=None):
            nonlocal mechanism_state
            preview.stage = stage
            preview._plan_pose(p, r, opening=opening, path_fn=path_fn, distance=distance, angle=angle, approach=approach,
                               approach_clearance=self._approach_clearance(stage))
            times = np.linspace(0., preview._duration, 161)
            references = np.array([preview.planned_path_reference(t) for t in times])
            states = np.array([mechanism_state if mechanism_fn is None else mechanism_fn(float(preview._timing.at(t)))
                               for t in times])
            overrides = {key: states[:, i] for i, key in enumerate(self._required_scalars)}
            values.append((stage, references, ("fastener",), overrides))
            preview.reference[:] = references[-1]
            mechanism_state = tuple(states[-1])
        def sample(hinge, slide, spin):
            p, r = self.mechanism_pose(hinge, slide, spin)
            return ThreadedSample(0., preview.reference.copy(), np.zeros(8), p, r,
                                  np.zeros(3), np.zeros(3), np.zeros(6), True,
                                  scalar_joints=dict(zip(self._required_scalars, (hinge, slide, spin))))
        start = sample(0., self.slide_ref, self.spin_ref)
        grasp, orientation = preview._grasp_pose(start, offset_local=self.config.grasp_offset_m)
        record("approach", grasp, orientation, approach=True)
        held_opening = self._contact_opening(preview.reference, mechanism_state)
        position, rotation = preview.chain.forward(preview.reference[:6])
        record("grip", position, rotation, opening=held_opening)
        grasp_sample = sample(0., self.slide_ref, self.spin_ref)
        preview._capture_grasp(grasp_sample)
        offset, relative = preview._grasp_transform
        def tool_pose(h, z, theta):
            p, r = self.mechanism_pose(h, z, theta)
            tool_r = r @ relative.T
            return p-tool_r @ offset, tool_r
        zero_spin = self.helix_offset_rad
        p, r = tool_pose(0., 0., zero_spin)
        record("loosen", p, r, path_fn=lambda u: tool_pose(0., self.slide_ref*(1-u), self.spin_ref*(1-u)+zero_spin*u),
               distance=self.slide_ref, angle=abs(self.spin_ref-zero_spin),
               mechanism_fn=lambda u: (0., self.slide_ref*(1-u), self.spin_ref*(1-u)+zero_spin*u))
        def regrasp(prefix, mode, grasp_stage):
            """An open, clear, approach and close sequence with no held pose cheat."""
            position, rotation = preview.chain.forward(preview.reference[:6])
            record(prefix+"_release", position, rotation, opening=self.config.release_open_rad)
            record(prefix+"_clear", *preview._clear_pose(prefix+"_clear", sample(*mechanism_state)))
            preview._grasp_mode, preview._grasp_sign = mode, None
            gp, gr = preview._grasp_pose(sample(*mechanism_state))
            record(prefix+"_approach", gp, gr, approach=True)
            opening = self._contact_opening(preview.reference, mechanism_state)
            record(grasp_stage, gp, gr, opening=opening)
            preview._capture_grasp(sample(*mechanism_state))
            return preview._grasp_transform
        offset, relative = regrasp("fold", "fold_support", "fold_grip")
        p, r = tool_pose(self.config.folded_hinge_rad, 0., zero_spin)
        radius = np.linalg.norm(self.part_reference_position-self.pivot)
        record("lower", p, r, path_fn=lambda u: tool_pose(u*self.config.folded_hinge_rad, 0., zero_spin),
               distance=radius*abs(self.config.folded_hinge_rad), angle=abs(self.config.folded_hinge_rad),
               mechanism_fn=lambda u: (u*self.config.folded_hinge_rad, 0., zero_spin))
        record("release_down", p, r, opening=self.config.release_open_rad)
        record("clear_down", *preview._clear_pose("clear_down", sample(*mechanism_state)))
        down = sample(self.config.folded_hinge_rad, 0., zero_spin)
        preview._grasp_sign = None
        gp, gr = preview._grasp_pose(down)
        record("reapproach", gp, gr, approach=True)
        held_opening = self._contact_opening(preview.reference, mechanism_state)
        position, rotation = preview.chain.forward(preview.reference[:6])
        record("regrip", position, rotation, opening=held_opening)
        preview._capture_grasp(sample(self.config.folded_hinge_rad, 0., zero_spin))
        offset, relative = preview._grasp_transform
        p, r = tool_pose(0., 0., zero_spin)
        record("raise", p, r, path_fn=lambda u: tool_pose((1-u)*self.config.folded_hinge_rad, 0., zero_spin),
               distance=radius*abs(self.config.folded_hinge_rad), angle=abs(self.config.folded_hinge_rad),
               mechanism_fn=lambda u: ((1-u)*self.config.folded_hinge_rad, 0., zero_spin))
        offset, relative = regrasp("turn", "turn_latch", "turn_grip")
        p, r = tool_pose(0., self.slide_ref, self.spin_ref)
        record("tighten", p, r, path_fn=lambda u: tool_pose(0., self.slide_ref*u, zero_spin*(1-u)+self.spin_ref*u),
               distance=self.slide_ref, angle=abs(self.spin_ref-zero_spin),
               mechanism_fn=lambda u: (0., self.slide_ref*u, zero_spin*(1-u)+self.spin_ref*u))
        record("release", p, r, opening=self.config.release_open_rad)
        record("retract", p-self.config.retreat_distance_m*r[:, 2], r)
        return values

    def _contact_opening(self, reference, mechanism_state):
        """Use physical jaw contact, not a fully closed preload, in preflight."""
        from .drawer_initialization import NativePoseProbe

        probe = NativePoseProbe(self.model_path)
        try:
            for name, value in zip(self._required_scalars, mechanism_state):
                probe.q[probe.address(name)] = value
            return probe.grasp_opening(reference, self.config.part_body)
        finally:
            probe.close()
