"""Episode-only initial drawer pose; no live state or physical constraints change."""

import math
import xml.etree.ElementTree as ET
import numpy as np

from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix

MARKER = "scripted_drawer_initial_state"
DEFAULT_OPEN_HEIGHT_M = 0.08


def prepare_drawer_initial_state(root, task_name, *, open_height_m=DEFAULT_OPEN_HEIGHT_M):
    """Match slide ref and rest position so qpos0 is open and q=0 stays closed.

    Called on a private MJCF tree before creating the scene or preflight model.
    Reapplying or switching between split tasks preserves the original zero pose.
    """
    if task_name not in ("drawer_cycle", "drawer_open", "drawer_close"):
        raise ValueError(f"Unknown drawer task: {task_name}")
    if not math.isfinite(open_height_m) or not 0 < open_height_m <= 0.12:
        raise ValueError("Drawer opening must be finite and in (0, 0.12] m")
    body = root.find('.//body[@name="drawer_base"]')
    slide = body.find('joint[@name="drawer_base_slide"]') if body is not None else None
    buckle = root.find('.//joint[@name="drawer_buckle_joint"]')
    if slide is None or slide.get("type") != "slide" or buckle is None:
        raise ValueError("Drawer task requires the original drawer slide and buckle")
    desired = open_height_m if task_name == "drawer_close" else 0.0
    limits = np.fromstring(slide.get("range", ""), sep=" ")
    if limits.shape != (2,) or not limits[0] <= desired <= limits[1]:
        raise ValueError("Drawer initial opening exceeds its physical slide range")
    axis = np.fromstring(slide.get("axis", "0 0 1"), sep=" ")
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
        raise ValueError("Drawer slide has an invalid axis")
    axis = quaternion_wxyz_to_matrix(
        np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
    ) @ (axis / np.linalg.norm(axis))
    position = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
    previous = float(slide.get("ref", "0"))
    marker = root.find(f'./custom/numeric[@name="{MARKER}"]')
    if marker is None:
        if abs(previous) > 1e-12:
            raise ValueError("Unrecognized preexisting drawer joint reference")
        closed_position = position.copy()
    else:
        saved = np.fromstring(marker.get("data", ""), sep=" ")
        if (saved.shape != (4,) or not np.isfinite(saved).all()
                or not math.isclose(saved[0], previous, abs_tol=1e-12)
                or not np.allclose(position, saved[1:] + axis * previous, rtol=0, atol=1e-12)):
            raise ValueError("Drawer initial-state marker disagrees with its rest transform")
        closed_position = saved[1:]
    if abs(float(buckle.get("ref", "0"))) > 1e-12:
        raise ValueError("Drawer initial state requires the original relaxed buckle")
    body.set("pos", " ".join(format(float(v), ".17g") for v in closed_position + axis * desired))
    slide.set("ref", format(desired, ".17g"))
    if marker is None:
        custom = root.find("custom")
        if custom is None:
            custom = ET.SubElement(root, "custom")
        marker = ET.SubElement(custom, "numeric", name=MARKER)
    marker.set("data", " ".join(format(float(v), ".17g") for v in (desired, *closed_position)))
    return dict(
        task=task_name,
        drawer_slide_m=desired,
        drawer_buckle_rad=0.0,
        source="MJCF_joint_ref_and_matching_rest_transform",
        runtime_state_writes=False,
    )
