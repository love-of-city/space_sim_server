"""Reproducible reset configuration and read-only native collision preflight."""

from copy import deepcopy
import ctypes
import math
import os
import numpy as np
from .drawer_cycle import DrawerCycleTask
from .drawer_lift import DrawerSample
from .plug_insert import PlugInsertTask
from .plug_profiles import is_plug_task, plug_profile
from .threaded_tasks import THREADED_TASKS, create_threaded_task, threaded_task_class

NOISE_DISABLED = {
    name: False
    for name in (
        "joint_observation_noise",
        "joint_bias",
        "action_noise",
        "observation_delay",
        "wrist_wrench_noise",
        "external_disturbance",
    )
}


def randomized_scene(source, seed, half_range_deg=10.0, task_name="drawer_cycle"):
    if not math.isfinite(half_range_deg) or not 0 <= half_range_deg <= 10:
        raise ValueError("initialization range must be finite and in [0,10] degrees")
    if not source.get("initialization_enabled", True):
        half_range_deg = 0.0
    scene = deepcopy(source)
    baseline = np.asarray(
        source["randomization"]["arm_joint_position_rad"][:6], dtype=float
    )
    if baseline.shape != (6,) or not np.isfinite(baseline).all():
        raise ValueError("source scene must supply six finite baseline joint angles")
    rng = np.random.Generator(np.random.PCG64(seed))
    offset = rng.uniform(-half_range_deg, half_range_deg, 6)
    scene["seed"] = int(seed)
    from space_arm_platform.overview_camera import initialize_overview_camera
    camera = initialize_overview_camera(scene)
    # Re-sample at the episode boundary; a copied scene otherwise retains the
    # same saved orbital phase for every attempt. Replay never samples again.
    orbit_randomized = scene.get("randomize_orbit_phase", False)
    if not isinstance(orbit_randomized, bool):
        raise ValueError("randomize_orbit_phase must be a boolean")
    if orbit_randomized:
        from space_arm_platform.orbit_randomization import sample_orbit_phase
        orbit = scene.setdefault("environment", {}).setdefault("orbit", {})
        orbit["true_anomaly_deg"] = sample_orbit_phase(int(seed))
    scene["randomization"]["arm_joint_position_rad"] = [
        *(baseline + np.deg2rad(offset)),
        0.0,
        0.0,
    ]
    scene["arm_preparation_required"] = False
    scene["collection_config"] = dict(
        schema="drawer-collection/1"
        if task_name == "drawer_cycle"
        else "scripted-collection/2",
        task=task_name,
        task_version=threaded_task_class(task_name).version
        if task_name in THREADED_TASKS
        else DrawerCycleTask.version
        if task_name == "drawer_cycle"
        else (PlugInsertTask.task_version(task_name)
              if is_plug_task(task_name)
              else PlugInsertTask.version),
        seed=int(seed),
        generator="numpy.PCG64",
        initialization=dict(
            distribution="independent_uniform",
            half_range_deg=half_range_deg,
            baseline_arm_joint_deg=np.rad2deg(baseline).tolist(),
            offset_deg=offset.tolist(),
            initial_joint_velocity_rad_s=[0.0] * 8,
            initial_gripper_opening=0.0,
        ),
        randomization_options=scene.get("collection_randomization", {}),
        orbital_phase_randomization=(dict(
            enabled=True, distribution="uniform_true_anomaly_deg", range_deg=[0.0, 360.0],
            stream="space-arm-orbit-phase-v1", seed=int(seed),
            true_anomaly_deg=scene["environment"]["orbit"]["true_anomaly_deg"],
        ) if orbit_randomized else dict(enabled=False)),
        disturbances=NOISE_DISABLED.copy(),
        lighting_randomized=False,
        dynamics_randomized=False,
    )
    if camera is not None:
        scene["collection_config"]["overview_camera_randomization"] = camera
    from .randomization import prepare_episode
    return prepare_episode(scene)


class NativePoseProbe:
    """Separate model/data with the *same* Basilisk runtime and original geometry.

    Only this private preflight state is changed; no production simulation state
    is touched, and the Python mujoco package is never imported.
    """

    def __init__(self, model_path, *, clearance_m=0.):
        if not np.isfinite(clearance_m) or not 0. <= clearance_m <= .02:
            raise ValueError("Planning clearance must be between 0 and 0.02 m")
        from .gripper_preflight import ParallelogramGripper
        self.gripper_kinematics = ParallelogramGripper(model_path)
        from simulation.mujoco_plugins import ensure_model_plugins
        from simulation.local_mujoco_stepper import (
            _library,
            _basilisk_mujoco_runtime,
            _pointer,
            _INT64_P,
        )

        ensure_model_plugins(model_path)
        self.lib, self.ptr, self.i64 = _library(), _pointer, _INT64_P
        error = ctypes.create_string_buffer(1024)
        runtime = str(_basilisk_mujoco_runtime())
        handle = self.lib.lms_create(
            runtime if os.name == "nt" else os.fsencode(runtime),
            str(model_path.resolve()).encode(),
            error,
            len(error),
        )
        if not handle:
            raise RuntimeError(error.value.decode(errors="replace"))
        self.handle = ctypes.c_void_p(handle)
        if clearance_m:
            self.check(self.lib.lms_probe_clearance(self.handle, float(clearance_m)))
        dims = np.zeros(6, dtype=np.int64)
        self.check(self.lib.lms_dims(self.handle, dims.ctypes.data_as(self.i64)))
        self.q = np.zeros(int(dims[0]))
        self.v = np.zeros(int(dims[1]))
        self.check(
            self.lib.lms_get_state(self.handle, self.ptr(self.q), self.ptr(self.v))
        )
        self.v.fill(0.0)
        self.home = self.q.copy()
        self.addresses = [
            self.address(name)
            for name in [
                *(f"joint{i}" for i in range(1, 7)),
                "ee_link11_joint",
                "ee_link12_joint",
            ]
        ]
        self.passive_addresses = {
            name: self.address(name)
            for name in self.gripper_kinematics.passive_positions([0.0, 0.0])
        }
        self.names = {}
        for body in range(int(dims[3])):
            buf = ctypes.create_string_buffer(256)
            self.lib.lms_id2name(self.handle, 1, body, buf, len(buf))
            self.names[body] = buf.value.decode()
        self.geom_names = {}
        self.passive_position_offsets = {}

    def geom_name(self, geom):
        """Resolve only encountered geoms; the native dimension API omits ngeom."""
        geom = int(geom)
        if geom not in self.geom_names:
            buf = ctypes.create_string_buffer(256)
            self.lib.lms_id2name(self.handle, 5, geom, buf, len(buf))
            self.geom_names[geom] = buf.value.decode()
        return self.geom_names[geom]

    @staticmethod
    def check(status):
        if status < 0:
            raise RuntimeError(f"native preflight error {status}")

    def address(self, name):
        joint = self.lib.lms_name2id(self.handle, 3, name.encode())
        if joint < 0:
            raise ValueError(f"missing joint {name}")
        a = np.zeros(3, dtype=np.int64)
        self.check(self.lib.lms_joint(self.handle, joint, a.ctypes.data_as(self.i64)))
        return int(a[0])

    def contacts(self, q):
        self.q[self.addresses] = q
        for name, value in self.gripper_kinematics.passive_positions(q[6:8]).items():
            self.q[self.passive_addresses[name]] = value + self.passive_position_offsets.get(name, 0.)
        self.check(
            self.lib.lms_set_state(self.handle, self.ptr(self.q), self.ptr(self.v))
        )
        count = self.lib.lms_contacts(self.handle, None, 0)
        self.check(count)
        rows = np.empty((count, 5))
        self.check(self.lib.lms_contacts(self.handle, self.ptr(rows), count))
        return rows

    def use_measured_qpos(self, qpos, joints):
        """Restore measured coordinates in this private geometry instance only.

        Preserve each passive four-bar's measured deviation from the analytic
        linkage while the candidate jaws open. The first candidate therefore
        reproduces the actual articulated gripper, including contact preload.
        """
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != self.home.shape or not np.isfinite(qpos).all():
            raise ValueError("Measured native qpos does not match this model")
        self.home[:] = qpos
        self.q[:] = qpos
        self.passive_position_offsets = {
            name: float(qpos[self.passive_addresses[name]] - value)
            for name, value in self.gripper_kinematics.passive_positions(joints[6:8]).items()
        }

    def body_pose(self, name):
        body = self.lib.lms_name2id(self.handle, 1, name.encode())
        values = np.zeros(12)
        self.check(self.lib.lms_body_geometry_pose(self.handle, body, self.ptr(values)))
        return values[:3].copy(), values[3:].reshape(3, 3).copy()

    def grasp_opening(self, candidate, part_body):
        """Actual parallel-jaw surface contact in this private native model.

        A closed-loop pose sweep locates the first bilateral contact instead
        of using a commanded preload angle as the fingers' physical position.
        The caller owns this separate probe; no runtime state is involved.
        """
        candidate = np.asarray(candidate, dtype=float).copy()
        if candidate.shape != (8,) or not np.isfinite(candidate).all():
            raise ValueError("Grasp opening requires eight finite joint coordinates")

        def bilateral(opening):
            candidate[6:] = [opening, -opening]
            fingers = set()
            for _, _, first, second, distance in self.contacts(candidate):
                names = {self.names[int(first)], self.names[int(second)]}
                if part_body in names and distance <= 0.0:
                    fingers.update(names.intersection({"ee_finger1", "ee_finger2"}))
            return len(fingers) == 2

        if bilateral(1.0):
            raise ValueError("Fully open fingers intersect the threaded part")
        previous = 1.0
        for opening in np.linspace(1.0, 0.0, 101)[1:]:
            if bilateral(float(opening)):
                closed, clear = float(opening), previous
                for _ in range(18):
                    middle = .5 * (closed + clear)
                    if bilateral(middle):
                        closed = middle
                    else:
                        clear = middle
                return closed
            previous = float(opening)
        raise ValueError("No bilateral finger/part contact along the closing sweep")

    def pose_collision(self, candidate, task_name, allowed_contact_bodies=(),
                       joint_values=None, baseline=None, arm_penetration_m=.0005,
                       clearance_m=0., mechanism_clearance_m=None, axial_socket=None):
        """The same private contact gate for reset preflight and carry branches."""
        moving_mechanism = ({"locking_rod", "fastener"} if task_name == "locking_rod_cycle"
                            else {"bolt"} if task_name == "aviation_bolt_insert"
                            else {task_name.removesuffix("_insert")} if task_name in ("aviation_plug_insert","guide_plug_insert","cylindrical_plug_insert")
                            else {"cylindrical_plug"} if task_name == "plug_insert"
                            else {"drawer_base", "drawer_buckle"} if task_name == "drawer_cycle" else set())
        if task_name not in THREADED_TASKS and not allowed_contact_bodies and axial_socket is None:
            # A released plug or drawer is static scene geometry. Its own seat
            # contact must not reject an unrelated empty-gripper withdrawal.
            moving_mechanism = set()
        own_rod_contacts = {
            frozenset(pair) for pair in (
                ("locking_rod", "fastener"), ("fastener", "hook"),
                ("locking_rod", "hook"), ("locking_rod", "limit_clip"),
                ("fastener", "limit_clip"),
            )
        }
        bolt_mating_geoms = {
            frozenset(("bolt_thread_sdf", "board_thread_sdf")),
            frozenset(("bolt_thread_sdf", "bolt_storage_thread_sdf")),
        }

        self.q[:] = self.home if baseline is None else baseline
        for name, values in (joint_values or {}).items():
            values = np.asarray(values, dtype=float).reshape(-1)
            if len(values) not in (1, 7) or not np.isfinite(values).all():
                raise ValueError(f"Invalid private preflight state for {name}")
            address = self.address(name)
            self.q[address:address + len(values)] = values
        for geom1, geom2, first, second, distance in self.contacts(candidate):
            a, b = self.names[int(first)], self.names[int(second)]
            is_arm = lambda name: (
                name.startswith(("ee", "wrist_ft")) or name in {f"link{i}" for i in range(1, 6)}
            )
            # Internal closed four-bar contacts belong to the gripper itself.
            if a.startswith("ee") and b.startswith("ee"):
                continue
            # Planned grasp contact permits the fingers only. A declared
            # grasped body never exempts a wrist or arm-link collision.
            if ((a in {"ee_finger1", "ee_finger2"} and b in allowed_contact_bodies)
                    or (b in {"ee_finger1", "ee_finger2"} and a in allowed_contact_bodies)):
                continue
            arm_threshold = clearance_m if clearance_m else -arm_penetration_m
            if (is_arm(a) or is_arm(b)) and distance < arm_threshold:
                return dict(body1=a, body2=b, penetration_m=float(-distance), separation_m=float(distance))
            mechanism_threshold = (mechanism_clearance_m if mechanism_clearance_m is not None
                                   else clearance_m if clearance_m else -0.0002)
            if moving_mechanism.intersection((a, b)) and distance < mechanism_threshold:
                if task_name == "drawer_cycle" and {a,b} <= {"drawer_base", "drawer_buckle"}:
                    continue
                if axial_socket is not None:
                    # Only the active connector's own mating wall is intentional
                    # during its constrained axial stroke; never exempt an arm.
                    prefix, pickup = axial_socket
                    other = b if a == prefix + "_plug" else a
                    geom = self.geom_name(geom2 if a == prefix + "_plug" else geom1)
                    if ((pickup and other == "asm_board2") or
                            (not pickup and geom.startswith(prefix + "_bore_"))):
                        continue
                if task_name == "locking_rod_cycle" and frozenset((a, b)) in own_rod_contacts:
                    continue
                if task_name == "aviation_bolt_insert":
                    geoms = (self.geom_name(geom1), self.geom_name(geom2))
                    if frozenset(geoms) in bolt_mating_geoms:
                        continue
                else:
                    geoms = None
                contact = dict(body1=a, body2=b, penetration_m=float(-distance),
                               collision_kind="mechanism_environment_collision")
                if geoms is not None:
                    contact.update(geom1=geoms[0], geom2=geoms[1])
                return contact
        return None

    def check_threaded_paths(self, task_name, paths, strict_stages=None):
        """Read-only geometry check for an explicitly planned mechanism branch."""
        if strict_stages is None:
            strict_stages = (("target_release", "retract")
                             if task_name == "aviation_bolt_insert" else ())
        checked = 0
        stages = []
        for stage, candidates, allowed, overrides in paths:
            stages.append(stage)
            for index, candidate in enumerate(candidates):
                checked += 1
                hit = self.pose_collision(candidate, task_name, allowed,
                    {joint: values[index] for joint, values in overrides.items()},
                    arm_penetration_m=0. if stage in strict_stages else .0005)
                if hit:
                    return dict(accepted=False, reason=hit.get("collision_kind", "approach_collision"),
                                stage=stage, contact=hit, checked_poses=checked)
        return dict(accepted=True, checked_poses=checked, checked_stages=stages)

    def validate(self, model, scene, task_name="drawer_cycle", *, task=None):
        self.q[:] = self.home
        target = scene.get("randomization", {})
        if "target_position_m" in target:
            address = self.address("capture_target_free")
            self.q[address : address + 3] = target["target_position_m"]
            self.q[address + 3 : address + 7] = target["target_orientation_wxyz"]
        q = np.array(scene["randomization"]["arm_joint_position_rad"])
        from .plug_profiles import is_plug_task, plug_profile

        is_plug = is_plug_task(task_name)
        if task_name in THREADED_TASKS:
            options = {}
            profile = scene.get("collection_config", {}).get("threaded_speed_profile")
            if profile is not None:
                options["speed_profile"] = profile
            task = task or create_threaded_task(task_name, model, **options)
            if hasattr(self, "handle"):
                task.enable_environment_planning()
                task.chain.enable_native_geometry()
        elif is_plug:
            from .plug_insert import PlugInsertTask, PlugInsertConfig, PlugSample
            from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
            import xml.etree.ElementTree as ET

            profile = plug_profile(task_name)
            task = type(task)(model,task.config) if task is not None else PlugInsertTask(model, PlugInsertConfig(plug_body=profile.body))
        else:
            task = type(task)(model,task.config) if task is not None else DrawerCycleTask(model)
        if hasattr(self,"handle") and task_name not in THREADED_TASKS:
            from .environment_motion import enable
            enable(task)
        if np.any(q[:6] < task.lower) or np.any(q[:6] > task.upper):
            return dict(accepted=False, reason="joint_limit")
        checked = 0
        probe_initial = self.q.copy()
        def collision(candidate, allowed_contact_bodies=(), joint_values=None,
                      arm_penetration_m=.0005):
            nonlocal checked
            checked += 1
            return self.pose_collision(candidate, task_name, allowed_contact_bodies,
                                        joint_values, probe_initial, arm_penetration_m)

        hit = collision(q)
        if hit:
            return dict(
                accepted=False,
                reason=hit.get("collision_kind", "initial_collision"),
                contact=hit,
                checked_poses=checked,
            )
        if task_name in THREADED_TASKS:
            try:
                paths = task.preflight_paths(q.copy())
                if not paths:
                    raise ValueError("Threaded task supplied no approach/target preflight paths")
                stages = []
                for planned in paths:
                    stage, candidates, allowed_bodies = planned[:3]
                    overrides = planned[3] if len(planned) == 4 else {}
                    if len(planned) not in (3, 4):
                        raise ValueError("Invalid threaded preflight path record")
                    stages.append(stage)
                    candidates = np.asarray(candidates, dtype=float)
                    if (candidates.ndim != 2 or candidates.shape[1] != 8
                            or len(candidates) < 2 or not np.isfinite(candidates).all()
                            or np.any(candidates[:, :6] < task.lower)
                            or np.any(candidates[:, :6] > task.upper)):
                        raise ValueError(f"Invalid native preflight path for {stage}")
                    for values in overrides.values():
                        if len(values) != len(candidates):
                            raise ValueError(f"Private preflight state count differs for {stage}")
                    for index, candidate in enumerate(candidates):
                        hit = collision(candidate, allowed_bodies,
                                        {key: values[index] for key, values in overrides.items()},
                                        0. if task_name == "aviation_bolt_insert"
                                        and stage in ("target_release", "retract") else .0005)
                        if hit:
                            return dict(accepted=False, reason=hit.get("collision_kind", "approach_collision"),
                                        stage=stage, contact=hit, checked_poses=checked)
            except (ValueError, RuntimeError, NotImplementedError) as exc:
                return dict(accepted=False, reason="planning_failed", detail=str(exc),
                            carry_branches=getattr(task, "preflight_carry_branches", []),
                            motion_plans=getattr(task, "preflight_motion_plans", []),
                            planning_failure=getattr(exc,"report",None),
                            checked_poses=checked)
            return dict(
                accepted=True, checked_poses=checked, checked_stages=stages,
                carry_branches=getattr(task, "preflight_carry_branches", []),
                carry_symmetry_rad=getattr(task, "preflight_carry_symmetry_rad", None),
                motion_plans=getattr(task, "preflight_motion_plans", []),
                collision_scope=("initial and controller-sampled approach/target paths; "
                                 "original native geometry; arm penetration threshold 0.5 mm; "
                                 "moving mechanism/environment threshold 0.2 mm; "
                                 "own rod joints/clip and exact thread mating contacts permitted; "
                                 "internal gripper and explicitly planned finger/part contacts excluded"),
            )
        if is_plug:
            element = (
                ET.parse(model)
                .getroot()
                .find(f'./worldbody/body[@name="{profile.body}"]')
            )
            sample = PlugSample(
                0.0,
                q,
                np.zeros(8),
                np.fromstring(element.get("pos"), sep=" "),
                quaternion_wxyz_to_matrix(np.fromstring(element.get("quat"), sep=" ")),
                np.zeros(3),
                np.zeros(3),
                np.zeros(6),
                False,
            )
        else:
            sample = DrawerSample(
                0.0, q, np.zeros(8), 0.0, 0.0, 0.0, np.zeros(6), False
            )
        task.step(sample)
        try:
            for stage in (
                (("approach",) if task.coordinated_grasp
                 else ("raise", "above_pickup", "align"))
                if is_plug
                else task.approach_stages
            ):
                task._enter(stage, sample)
                path = task._path
                if path is None:
                    continue
                if getattr(task, "_timing", None) is not None:
                    # Validate the executed spline, including former segment
                    # midpoints, rather than checking only its IK knots.
                    path = [
                        task._timing.path.at(u)
                        for u in np.linspace(0, 1, 2 * len(task._path) - 1)
                    ]
                candidates = [np.r_[point, 0.0, 0.0] for point in path]
                if is_plug and task.coordinated_grasp:
                    candidates = [task.planned_path_reference(t)
                                  for t in np.linspace(0., task._duration, 2 * len(path) - 1)]
                if not is_plug and stage == "approach":
                    candidates = [
                        task.approach_reference(u)
                        for u in np.linspace(0, 1, 2 * len(task._path) - 1)
                    ]
                for candidate in candidates:
                    hit = collision(candidate)
                    if hit:
                        return dict(
                            accepted=False,
                            reason="approach_collision",
                            stage=stage,
                            contact=hit,
                            checked_poses=checked,
                        )
                task.reference[:6] = task._path[-1][:6]
                if is_plug and task.coordinated_grasp:
                    task.reference[:] = candidates[-1]
                if not is_plug and stage == "approach":
                    task.reference[:] = task.approach_reference(1.0)
            # Descend/lift/return IK must exist too. Contact intentionally starts here.
            for stage in (
                (() if task.coordinated_grasp else ("descend",)) if is_plug else ("lift", "return", "retract")
            ):
                task._enter(stage, sample)
                task.reference[:6] = task._path[-1][:6]
        except ValueError as exc:
            return dict(
                accepted=False,
                reason="planning_failed",
                detail=str(exc),
                checked_poses=checked,
            )
        return dict(
            accepted=True,
            checked_poses=checked,
            motion_plans=getattr(task,"motion_plans",[]),
            collision_scope="initial and discrete approach path; original native geometry; 0.5 mm penetration threshold; internal gripper contacts excluded",
        )

    def close(self):
        if self.handle:
            self.lib.lms_destroy(self.handle)
            self.handle = None
