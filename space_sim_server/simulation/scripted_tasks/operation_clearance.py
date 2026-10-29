"""Check constrained task strokes without asking a mechanism to detour off-axis."""

import time
import xml.etree.ElementTree as ET
import numpy as np
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
from .drawer_initialization import NativePoseProbe
from .motion import solve_pose
from .obstacle_planner import PlanningFailure


def check_operation(task, sample):
    """Read-only nominal sweep at stage entry, based on the measured scene.

    Feedback still handles contact and tracking during execution. An obstructed
    rail/bore is rejected: free-space RRT cannot move a fixed hole or guide rail.
    """
    started = time.monotonic()
    plug = hasattr(sample, "plug_position_body_m")
    probe = NativePoseProbe(task.model_path)
    report = dict(
        stage=task.stage,
        route="constrained_sweep",
        checked_poses=0,
        accepted=False,
        nominal_only=True,
    )
    try:
        if sample.native_qpos is not None:
            probe.use_measured_qpos(sample.native_qpos, sample.joints)
        probe.contacts(sample.joints)
        bus_p, bus_r = probe.body_pose("cubesat_bus")
        baseline = probe.home.copy()
        q = sample.joints[:6].copy()
        if plug:
            body = (
                ET.parse(task.model_path)
                .getroot()
                .find(f'.//body[@name="{task.config.plug_body}"]')
            )
            joint = body.find("freejoint")
            if joint is None:
                joint = body.find('joint[@type="free"]')
            part_start = sample.plug_position_body_m
            part_end = part_start.copy()
            pickup = task.stage == "extract"
            part_end[:2] = (task.pickup if pickup else task.hole)[:2]
            part_end[2] = task._extract_z if pickup else task._seat_z
            p, r = task.chain.forward(sample.joints[:6])
            relp = r.T @ (part_start - p)
            relr = r.T @ sample.plug_rotation_body
            rotation = task._extract_rotation if pickup else task.plug_rotation
            tool_r = rotation @ relr.T
        else:
            start_p, _ = task.chain.forward(task.reference[:6])
        for u in np.linspace(0.0, 1.0, 161):
            if plug:
                part_p = part_start + u * (part_end - part_start)
                q = solve_pose(
                    task.chain,
                    part_p - tool_r @ relp,
                    tool_r,
                    q,
                    task.lower,
                    task.upper,
                )
                overrides = {
                    joint.get("name"): np.r_[
                        bus_p + bus_r @ part_p,
                        matrix_to_quaternion_wxyz(bus_r @ rotation),
                    ]
                }
                allowed = (task.config.plug_body,)
                socket = (task.profile.prefix, pickup)
            else:
                q = task._timing.path.at(u)[:6]
                p, _ = task.chain.forward(q)
                slide = sample.drawer_m + float((p - start_p) @ task.axis)
                overrides = {
                    "drawer_base_slide": slide,
                    "drawer_buckle_joint": sample.buckle_rad,
                }
                allowed, socket = ("drawer_base", "drawer_buckle"), None
            opening = float(max(sample.joints[6], -sample.joints[7]))
            hit = probe.pose_collision(
                np.r_[q, opening, -opening],
                task.name,
                allowed,
                overrides,
                baseline,
                axial_socket=socket,
            )
            report["checked_poses"] += 1
            if hit:
                report["first_collision"] = hit
                raise PlanningFailure("constrained_operation_corridor_blocked", report)
        report["accepted"] = True
    finally:
        probe.close()
        report["wall_seconds"] = time.monotonic() - started
        task.motion_plans.append(report)
    return report
