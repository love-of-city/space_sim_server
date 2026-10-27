"""Private native geometry queries for cylindrical connector grasps and retreats.

Only the separate probe changes coordinates. Production motion still consists
of the controller's timed actuator references, with all contacts enabled.
"""

import math

import numpy as np


def cylindrical_hold_opening(task, sample):
    from .drawer_initialization import NativePoseProbe

    probe = NativePoseProbe(task.model_path)
    try:
        probe.use_measured_qpos(sample.native_qpos, sample.joints)
        contact = probe.grasp_opening(task.reference.copy(), task.config.plug_body)
        if not .1 < contact < .8:
            raise ValueError("Cylindrical head has no usable parallel-jaw grasp")
        # Keep a real servo preload after reaching the original head tabs.
        # Driving all the way to zero compressed the blocked gripper by ~.26 rad
        # and tilted the long shaft against its pickup socket.
        return max(.01, contact - .06)
    finally:
        probe.close()


def plan_cylindrical_retract(task, sample):
    """Select a swept, rising retreat using the current randomized scene."""
    from .drawer_initialization import NativePoseProbe

    position, rotation = task.chain.forward(task.reference[:6])
    tangent = rotation[:, 1].copy()
    tangent[2] = 0.
    tangent /= np.linalg.norm(tangent)
    if tangent[0] < 0:
        tangent = -tangent
    probe = NativePoseProbe(task.model_path)
    attempts = []
    try:
        probe.use_measured_qpos(sample.native_qpos, sample.joints)
        # Prefer the short diagonal departure. Other headings/heights account
        # for neighboring plugs instead of always sweeping sideways at head level.
        for height in (.08, .12):
            for angle in (0., math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi):
                c, s = math.cos(angle), math.sin(angle)
                direction = np.array([c*tangent[0]-s*tangent[1],
                                      s*tangent[0]+c*tangent[1], 0.])
                goal = position + .12*direction + np.array([0., 0., height])
                try:
                    task._plan(goal, rotation)
                except ValueError as error:
                    attempts.append(dict(height_m=height, angle_rad=angle, reason=str(error)))
                    continue
                collision = None
                # The fingers keep opening during retreat. Check both ends of
                # that aperture range; neither permits touching the placed part.
                for opening in (max(sample.joints[6], -sample.joints[7]),
                                task.config.release_open_rad):
                    for u in np.linspace(0., 1., 161):
                        candidate = np.r_[task._timing.path.at(u), opening, -opening]
                        collision = probe.pose_collision(candidate, task.name,
                                                         arm_penetration_m=.0002)
                        if collision:
                            break
                    if collision:
                        break
                attempts.append(dict(height_m=height, angle_rad=angle, collision=collision))
                if collision is None:
                    task.retract_plan = dict(height_m=height, direction_body=direction.tolist(),
                                             swept_poses=322, attempts=attempts)
                    return
        raise ValueError("No collision-free cylindrical connector retreat")
    finally:
        probe.close()
