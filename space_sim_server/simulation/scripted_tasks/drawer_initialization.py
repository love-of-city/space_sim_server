"""Reproducible reset configuration and read-only native collision preflight."""

from copy import deepcopy
import ctypes
import math
import numpy as np
from .drawer_cycle import DrawerCycleTask
from .drawer_lift import DrawerSample

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
    scene = deepcopy(source)
    baseline = np.asarray(
        source["randomization"]["arm_joint_position_rad"][:6], dtype=float
    )
    if baseline.shape != (6,) or not np.isfinite(baseline).all():
        raise ValueError("source scene must supply six finite baseline joint angles")
    rng = np.random.Generator(np.random.PCG64(seed))
    offset = rng.uniform(-half_range_deg, half_range_deg, 6)
    scene["seed"] = int(seed)
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
        task_version="drawer-cycle/1"
        if task_name == "drawer_cycle"
        else "plug-insert/2",
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
        disturbances=NOISE_DISABLED.copy(),
        lighting_randomized=False,
        dynamics_randomized=False,
    )
    return scene


class NativePoseProbe:
    """Separate model/data with the *same* Basilisk runtime and original geometry.

    Only this private preflight state is changed; no production simulation state
    is touched, and the Python mujoco package is never imported.
    """

    def __init__(self, model_path):
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
        handle = self.lib.lms_create(
            str(_basilisk_mujoco_runtime()),
            str(model_path.resolve()).encode(),
            error,
            len(error),
        )
        if not handle:
            raise RuntimeError(error.value.decode(errors="replace"))
        self.handle = ctypes.c_void_p(handle)
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
        self.names = {}
        for body in range(int(dims[3])):
            buf = ctypes.create_string_buffer(256)
            self.lib.lms_id2name(self.handle, 1, body, buf, len(buf))
            self.names[body] = buf.value.decode()

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
        self.check(
            self.lib.lms_set_state(self.handle, self.ptr(self.q), self.ptr(self.v))
        )
        count = self.lib.lms_contacts(self.handle, None, 0)
        self.check(count)
        rows = np.empty((count, 5))
        self.check(self.lib.lms_contacts(self.handle, self.ptr(rows), count))
        return rows

    def validate(self, model, scene, task_name="drawer_cycle"):
        self.q[:] = self.home
        target = scene.get("randomization", {})
        if "target_position_m" in target:
            address = self.address("capture_target_free")
            self.q[address : address + 3] = target["target_position_m"]
            self.q[address + 3 : address + 7] = target["target_orientation_wxyz"]
        q = np.array(scene["randomization"]["arm_joint_position_rad"])
        from .plug_profiles import is_plug_task, plug_profile

        is_plug = is_plug_task(task_name)
        if is_plug:
            from .plug_insert import PlugInsertTask, PlugInsertConfig, PlugSample
            from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
            import xml.etree.ElementTree as ET

            profile = plug_profile(task_name)
            task = PlugInsertTask(model, PlugInsertConfig(plug_body=profile.body))
        else:
            task = DrawerCycleTask(model)
        if np.any(q[:6] < task.lower) or np.any(q[:6] > task.upper):
            return dict(accepted=False, reason="joint_limit")
        checked = 0

        def collision(candidate):
            nonlocal checked
            checked += 1
            for _, _, first, second, distance in self.contacts(candidate):
                a, b = self.names[int(first)], self.names[int(second)]
                is_arm = lambda name: (
                    name.startswith("ee") or name in {f"link{i}" for i in range(1, 6)}
                )
                # Internal closed four-bar contacts belong to the gripper itself.
                if a.startswith("ee") and b.startswith("ee"):
                    continue
                if (is_arm(a) or is_arm(b)) and distance < -0.0005:
                    return dict(body1=a, body2=b, penetration_m=float(-distance))
            return None

        hit = collision(q)
        if hit:
            return dict(
                accepted=False,
                reason="initial_collision",
                contact=hit,
                checked_poses=checked,
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
                ("raise", "above_pickup", "align")
                if is_plug
                else ("raise", "above_drawer", "align")
            ):
                task._enter(stage, sample)
                for point in task._path:
                    candidate = np.r_[point, 0.0, 0.0]
                    hit = collision(candidate)
                    if hit:
                        return dict(
                            accepted=False,
                            reason="approach_collision",
                            stage=stage,
                            contact=hit,
                            checked_poses=checked,
                        )
                task.reference[:6] = task._path[-1]
            # Descend/lift/return IK must exist too. Contact intentionally starts here.
            for stage in (
                ("descend",) if is_plug else ("descend", "lift", "return", "retract")
            ):
                task._enter(stage, sample)
                task.reference[:6] = task._path[-1]
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
            collision_scope="initial and discrete approach path; original native geometry; 0.5 mm penetration threshold; internal gripper contacts excluded",
        )

    def close(self):
        if self.handle:
            self.lib.lms_destroy(self.handle)
            self.handle = None
