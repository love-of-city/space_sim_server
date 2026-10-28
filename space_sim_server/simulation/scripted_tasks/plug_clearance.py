"""Private native geometry queries for cylindrical connector grasps and retreats.

Only the separate probe changes coordinates. Production motion still consists
of the controller's timed actuator references, with all contacts enabled.
"""

import math

import numpy as np


def close_cylindrical_release_probe(task):
    probe = getattr(task, '_release_clearance_probe', None)
    if probe is not None:
        probe.close()
        task._release_clearance_probe = None


def cylindrical_release_clearance(task, sample):
    """Continue opening until actual and commanded wrists can start withdrawal.

    The opening threshold comes from the measured scene, not an added timer or
    a fixed jaw angle. The same 2-mm full-arm check used by free-motion planning
    applies. A single private model is reused while the existing opening clock
    advances, then handed to the retreat template checker.
    """
    from .drawer_initialization import NativePoseProbe

    measured = np.asarray(sample.joints)
    reference = np.asarray(task.reference)
    opening = float(min(measured[6],-measured[7],reference[6],-reference[7]))
    threshold = getattr(task,'_release_clearance_opening',None)
    if threshold is not None and opening < threshold-1e-9:
        return False
    probe = getattr(task,'_release_clearance_probe',None)
    if probe is None:
        probe = NativePoseProbe(task.model_path,clearance_m=.002)
        task._release_clearance_probe = probe
    probe.use_measured_qpos(sample.native_qpos,sample.joints)
    checks = getattr(task,'release_clearance_checks',None)
    if checks is None:
        checks = task.release_clearance_checks = []

    def hit_at(aperture=None):
        for q in (measured,reference):
            candidate = q.copy()
            if aperture is not None:
                candidate[6:] = [aperture,-aperture]
            hit = probe.pose_collision(candidate,'free_motion',arm_penetration_m=0.,clearance_m=.002)
            if hit is not None:
                return hit
        return None

    hit = hit_at()
    if hit is None:
        checks.append(dict(accepted=True,opening_rad=opening,time_s=sample.time_s))
        return True
    goal = float(task.config.release_open_rad)
    full_hit = hit_at(goal)
    if full_hit is not None:
        checks.append(dict(accepted=False,reason='release_clearance_blocked_at_full_open',contact=full_hit))
        close_cylindrical_release_probe(task)
        raise ValueError('Cylindrical release clearance blocked even with fully open fingers')
    if len(checks) >= 4:
        close_cylindrical_release_probe(task)
        raise ValueError('Cylindrical release clearance did not stabilize within geometry-check budget')
    # Search a small measured aperture interval. Stop at the first clear sample
    # and refine its boundary; actual updated geometry is rechecked on arrival.
    previous = opening
    for aperture in np.linspace(opening,goal,33)[1:]:
        if hit_at(float(aperture)) is None:
            low,high = previous,float(aperture)
            for _ in range(10):
                middle = (low+high)/2
                if hit_at(middle) is None:
                    high = middle
                else:
                    low = middle
            task._release_clearance_opening = high
            checks.append(dict(accepted=False,opening_rad=opening,required_opening_rad=high,
                               time_s=sample.time_s,contact=hit))
            return False
        previous = float(aperture)
    close_cylindrical_release_probe(task)
    raise ValueError('No cylindrical release aperture within the checked interval')


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


def cylindrical_retract_candidates(task):
    """The same short rising endpoints for direct checks and detour search."""
    position, rotation = task.chain.forward(task.reference[:6])
    tangent = rotation[:, 1].copy()
    tangent[2] = 0.
    tangent /= np.linalg.norm(tangent)
    if tangent[0] < 0:
        tangent = -tangent
    candidates = []
    for height in (.08, .12):
        for angle in (0., math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, math.pi):
            c, s = math.cos(angle), math.sin(angle)
            direction = np.array([c*tangent[0]-s*tangent[1],
                                  s*tangent[0]+c*tangent[1], 0.])
            candidates.append(dict(position=position+.12*direction+[0.,0.,height],
                                   rotation=rotation,height_m=height,angle_rad=angle,
                                   direction_body=direction.tolist()))
    return candidates


def plan_cylindrical_retract(task, sample, candidates=None):
    """Select a swept, rising retreat using the current randomized scene."""
    from .drawer_initialization import NativePoseProbe

    candidates = cylindrical_retract_candidates(task) if candidates is None else candidates
    probe = getattr(task,'_release_clearance_probe',None)
    if probe is None:
        probe = NativePoseProbe(task.model_path)
    else:
        task._release_clearance_probe = None
    attempts = []
    try:
        probe.use_measured_qpos(sample.native_qpos, sample.joints)
        # Prefer the short diagonal departure. Other headings/heights account
        # for neighboring plugs instead of always sweeping sideways at head level.
        for target in candidates:
            height,angle = target['height_m'],target['angle_rad']
            try:
                task._plan(target['position'], target['rotation'])
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
                task.retract_plan = dict(height_m=height, direction_body=target['direction_body'],
                                         swept_poses=322, attempts=attempts)
                return
        task.retract_plan = dict(accepted=False, attempts=attempts)
        raise ValueError("No collision-free cylindrical connector retreat")
    finally:
        probe.close()
