"""Freeze reproducible episode initialization and current task configuration."""

from copy import deepcopy
import math
import numpy as np
from .drawer_tasks import DRAWER_TASKS, drawer_task_class
from .plug_insert import PlugInsertTask
from .plug_profiles import is_plug_task
from .threaded_tasks import THREADED_TASKS, threaded_task_class

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


def randomized_scene(source, seed, half_range_deg=30.0, task_name="drawer_cycle", *, color_assignment=None):
    if task_name == 'capture_target_grasp':
        from .capture_grasp import VERSION
        scene=deepcopy(source)
        scene.update(seed=int(seed),scripted_task_name=task_name,arm_preparation_required=False)
        scene['collection_config']=dict(task=task_name,task_version=VERSION,seed=int(seed),
            initialization=deepcopy(scene['capture_initial_state']))
        from .randomization import prepare_episode
        return prepare_episode(scene)
    if not math.isfinite(half_range_deg) or not 0 <= half_range_deg <= 50:
        raise ValueError("initialization range must be finite and in [0,50] degrees")
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
        else drawer_task_class(task_name).version
        if task_name in DRAWER_TASKS
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
    if color_assignment is not None:
        scene["collection_config"]["episode_color_assignment"] = deepcopy(color_assignment)
    if is_plug_task(task_name):
        from .plug_motion_config import plug_motion_config, config_snapshot
        for field in ("plug_motion_parameters", "plug_config_snapshot"):
            if field in source.get("collection_config", {}):
                scene["collection_config"][field] = deepcopy(source["collection_config"][field])
        config = plug_motion_config(task_name, scene)
        scene["collection_config"]["plug_config_snapshot"] = config_snapshot(config, task_name)
    from .randomization import prepare_episode
    return prepare_episode(scene)


