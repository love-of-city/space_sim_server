"""Shared free-space planning adapters; constrained mechanism motion stays physical."""

from types import SimpleNamespace
import numpy as np
from .collision_motion import plan_free_motion


def enable(task):
    task.environment_planning = True
    if not hasattr(task, "motion_plans"):
        task.motion_plans = []
    task.chain.enable_native_geometry()


def reference(task, progress):
    q = task._timing.path.at(progress)
    return np.r_[q[:6], q[6], -q[6]] if len(q) == 7 else None


def plan(
    task,
    sample,
    position,
    rotation,
    axis,
    *,
    held=False,
    error=None,
    opening=None,
    stage=None,
    clearance=None,
    path_reference=None,
    fallback_positions=None,
):
    """Normalize task APIs without replacing their contact controllers."""
    from .plug_profiles import is_plug_task

    name = getattr(task, "name", "drawer_cycle")
    plug = is_plug_task(name)
    drawer = name.startswith("drawer")
    cfg = task.config
    config = SimpleNamespace(
        pregrasp_distance_m=clearance or getattr(cfg, "pregrasp_distance_m", 0.045),
        approach_open_rad=opening
        if opening is not None
        else getattr(cfg, "approach_open_rad", 1.0),
        arm_speed_rad_s=cfg.arm_speed_rad_s,
        linear_speed_m_s=(
            cfg.carry_speed_m_s if plug and held else cfg.linear_speed_m_s
        ),
        angular_speed_rad_s=getattr(cfg, "angular_speed_rad_s", 0.2),
        gripper_speed_rad_s=cfg.gripper_speed_rad_s,
        grasp_approach_speed_m_s=getattr(
            cfg, "grasp_approach_speed_m_s", getattr(cfg, "approach_speed_m_s", 0.025)
        ),
    )
    proxy = SimpleNamespace(**vars(task))
    proxy.config = config
    proxy.path_reference = path_reference or task.path_reference
    if drawer:
        proxy.planning_part_body = "drawer_base"
        proxy.planning_part_joint = None
        proxy.planning_scalars = {
            "drawer_base_slide": sample.drawer_m,
            "drawer_buckle_joint": sample.buckle_rad,
        }
        part_p = task.drawer_origin + task.axis * sample.drawer_m
        part_r = task.drawer_rotation
    elif plug:
        proxy.planning_part_body = cfg.plug_body
        proxy.planning_part_joint = cfg.plug_body + "_freejoint"
        # Delivered models use *_free, resolve the actual freejoint name.
        import xml.etree.ElementTree as ET

        body = (
            ET.parse(task.model_path)
            .getroot()
            .find(f'.//body[@name="{cfg.plug_body}"]')
        )
        joint = body.find("freejoint")
        if joint is None:
            joint = body.find('joint[@type="free"]')
        proxy.planning_part_joint = joint.get("name")
        part_p, part_r = sample.plug_position_body_m, sample.plug_rotation_body
    else:
        proxy.planning_part_body = cfg.part_body
        proxy.planning_part_joint = None
        proxy.planning_scalars = {
            k: sample.scalar_joints[k] for k in task._required_scalars
        }
        part_p, part_r = sample.part_position_body_m, sample.part_rotation_body
    observed = SimpleNamespace(
        native_qpos=getattr(sample, "native_qpos", None),
        joints=sample.joints,
        part_position_body_m=part_p,
        part_rotation_body=part_r,
    )
    report = plan_free_motion(
        proxy,
        observed,
        position,
        rotation,
        axis,
        held=held,
        template_error=error,
        opening=config.approach_open_rad,
        stage=stage or task.stage,
        fallback_positions=fallback_positions,
    )
    for key in ("_planning_native_state", "_planning_bus_pose", "_timeout"):
        if hasattr(proxy, key):
            setattr(task, key, getattr(proxy, key))
    if report["route"] == "searched":
        task._path = proxy._path[:, :6].copy()
        task._timing = proxy._timing
        task._duration = proxy._duration
        task._motion_s = 0.0
        task._path_opening = getattr(proxy, "_path_opening", None)
        task._goal,task._rotation = proxy._goal.copy(),proxy._rotation.copy()
    return report


def threaded_plan(task, sample, p, r, *, approach=False, clearance=None):
    """Plan an axial grasp or free withdrawal with the same opening as execution."""
    error = None
    opening = (
        task.config.approach_open_rad if approach else task.config.release_open_rad
    )
    try:
        task._plan_pose(
            p, r, opening=opening, approach=approach, approach_clearance=clearance
        )
    except ValueError as exc:
        if not task.environment_planning:
            raise
        error = str(exc)
    if task.environment_planning:
        plan(
            task,
            sample,
            p,
            r,
            -r[:, 2] if approach else r[:, 2],
            error=error,
            opening=opening,
            clearance=clearance,
            path_reference=task.path_reference,
        )
