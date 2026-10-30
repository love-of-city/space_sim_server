"""Reproducible release of existing model parts, before dynamics starts."""
from dataclasses import dataclass
import hashlib
import math
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics, axis_angle_to_matrix, matrix_to_quaternion_wxyz,
    quaternion_wxyz_to_matrix,
)
from .floating_capture import FLOATING_TASKS, create_floating_task
from .motion import solve_pose


@dataclass(frozen=True)
class FloatingInitialConfig:
    min_distance_m: float = .12
    max_distance_m: float = .75
    max_speed_m_s: float = .001
    max_spin_rad_s: float = .003
    max_tilt_deg: float = 8.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError("Floating initial conditions must be finite")
        if not .12 <= self.min_distance_m <= self.max_distance_m <= .75:
            raise ValueError("Camera-to-grasp distance must be within [0.12, 0.75] m")
        if not (0 <= self.max_speed_m_s <= .01 and 0 <= self.max_spin_rad_s <= .05
                and 0 <= self.max_tilt_deg <= 20):
            raise ValueError("Floating velocity, spin or tilt exceeds task envelope")


def _fixed_pose(root, element, ancestor):
    parents = {child: parent for parent in root.iter() for child in parent}
    p, r = np.zeros(3), np.eye(3)
    while element is not ancestor:
        if element is None or (element.tag == "body" and
                (element.find("joint") is not None or element.find("freejoint") is not None)):
            raise ValueError("Camera and tool must have a fixed installation transform")
        rotation = quaternion_wxyz_to_matrix(np.fromstring(element.get("quat", "1 0 0 0"), sep=" "))
        p = np.fromstring(element.get("pos", "0 0 0"), sep=" ") + rotation@p
        r = rotation@r
        element = parents.get(element)
    return p, r


def camera_in_tool(root):
    wrist = root.find('.//body[@name="ee_base"]')
    camera = root.find('.//camera[@name="sarm_wrist_cam"]')
    tool = root.find('.//site[@name="sarm_ee"]')
    if wrist is None or camera is None or tool is None:
        raise ValueError("Floating capture requires the original wrist camera and tool")
    cp, cr = _fixed_pose(root, camera, wrist)
    tp, tr = _fixed_pose(root, tool, wrist)
    return tr.T@(cp-tp), tr.T@cr


def apply_floating_model(root, source, scene):
    """Detach only the selected part; retain geometry, inertial and contact data."""
    name = scene.get("scripted_task_name") or scene["collection_config"]["task"]
    asset = FLOATING_TASKS[name]
    cfg = scene["collection_config"]
    options = FloatingInitialConfig(**scene.get("floating_capture", {}))
    body = root.find(f'.//body[@name="{asset.body}"]')
    if body is None:
        raise ValueError(f"Team asset {asset.body} is missing from the selected model")
    chain = SerialChainKinematics.from_mjcf(source, base_body="cubesat_bus",
        joint_names=tuple(f"joint{i}" for i in range(1, 7)), tool_site="sarm_ee")
    initial = np.asarray(scene["randomization"]["arm_joint_position_rad"][:6])
    p, r = chain.forward(initial)
    cp, cr = camera_in_tool(root)
    camera_position, camera_rotation = p+r@cp, r@cr
    lower, upper = np.array([np.fromstring(root.find(f'.//joint[@name="joint{i}"]').get("range"), sep=" ")
                             for i in range(1, 7)]).T
    basis = np.column_stack((asset.closing_axis, np.cross(asset.approach_axis, asset.closing_axis), asset.approach_axis))
    saved = cfg.get("floating_initial_state")
    if saved is None:
        from .randomization import generator
        rng = generator(scene.get("seed", 0), "floating-release-v1")
        for attempt in range(128):
            distance = float(rng.uniform(options.min_distance_m, options.max_distance_m))
            ray = camera_rotation @ np.array([rng.uniform(-.08, .08), rng.uniform(-.08, .08), -1.])
            ray /= np.linalg.norm(ray)
            point = camera_position + distance*ray
            axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
            desired = axis_angle_to_matrix(axis, math.radians(rng.uniform(-options.max_tilt_deg, options.max_tilt_deg)))@r
            rotation = desired@basis.T
            try:
                goal = solve_pose(chain, point, desired, initial, lower, upper)
            except ValueError:
                continue
            if np.min(np.minimum(goal-lower, upper-goal)) < .15:
                continue
            # Magnitude bounded sampling, not three independently maximal axes.
            velocity = rng.normal(size=3); velocity *= rng.uniform(.25, 1.)*options.max_speed_m_s/np.linalg.norm(velocity)
            spin = rng.normal(size=3); spin *= rng.uniform(.25, 1.)*options.max_spin_rad_s/np.linalg.norm(spin)
            saved = dict(schema="floating-release/1", task=name, target_body=asset.body,
                seed=int(scene.get("seed", 0)), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                options=vars(options), position_body_m=(point-rotation@asset.grip_point).tolist(),
                orientation_wxyz=matrix_to_quaternion_wxyz(rotation).tolist(),
                velocity_body_m_s=velocity.tolist(), angular_velocity_body_rad_s=spin.tolist(),
                camera_position_body_m=camera_position.tolist(), camera_to_grasp_distance_m=distance,
                grasp_position_body_m=point.tolist(), ik_joint_position_rad=goal.tolist(),
                sampling="uniform distance and small view-cone offsets, conditioned on pose IK; native collision preflight follows",
                rejected_ik_candidates=attempt, camera_intrinsics="unchanged source MJCF")
            break
        else:
            raise ValueError("No reachable floating release in the requested distance range")
        cfg["floating_initial_state"] = saved
    if (saved.get("schema") != "floating-release/1" or saved.get("task") != name
            or saved.get("target_body") != asset.body
            or saved.get("seed") != int(scene.get("seed", 0))
            or saved.get("source_sha256") != hashlib.sha256(source.read_bytes()).hexdigest()
            or saved.get("options") != vars(options)):
        raise ValueError("Saved floating release does not match task, source or configuration")
    vectors = {}
    for key, size in (("position_body_m", 3), ("orientation_wxyz", 4),
                      ("velocity_body_m_s", 3), ("angular_velocity_body_rad_s", 3),
                      ("camera_position_body_m", 3), ("grasp_position_body_m", 3),
                      ("ik_joint_position_rad", 6)):
        value = np.asarray(saved.get(key), dtype=float)
        if value.shape != (size,) or not np.isfinite(value).all():
            raise ValueError(f"Invalid saved floating release: {key}")
        vectors[key] = value
    quat = vectors["orientation_wxyz"]
    if abs(np.linalg.norm(quat)-1.) > 1e-9:
        raise ValueError("Saved floating release contains a non-unit quaternion")
    point = vectors["position_body_m"]+quaternion_wxyz_to_matrix(quat)@asset.grip_point
    delta = point-camera_position
    distance = float(saved.get("camera_to_grasp_distance_m", float("nan")))
    goal = vectors["ik_joint_position_rad"]
    gp, gr = chain.forward(goal)
    if (not math.isfinite(distance) or not options.min_distance_m-1e-10 <= distance <= options.max_distance_m+1e-10
            or np.linalg.norm(vectors["velocity_body_m_s"]) > options.max_speed_m_s+1e-12
            or np.linalg.norm(vectors["angular_velocity_body_rad_s"]) > options.max_spin_rad_s+1e-12
            or np.any(goal < lower) or np.any(goal > upper)
            or not np.allclose(camera_position, vectors["camera_position_body_m"], atol=1e-10, rtol=0)
            or not np.allclose(point, vectors["grasp_position_body_m"], atol=1e-10, rtol=0)
            or abs(np.linalg.norm(delta)-distance) > 1e-10
            or np.dot(delta/distance, -camera_rotation[:, 2]) < .99
            or np.linalg.norm(gp-point) > 3e-6
            or np.linalg.norm(gr-quaternion_wxyz_to_matrix(quat)@basis) > 2e-5):
        raise ValueError("Saved floating release violates its physical configuration")
    parents = {child: parent for parent in root.iter() for child in parent}
    if parents[body] is not root.find("worldbody"):
        parents[body].remove(body)
        root.find("worldbody").append(body)
        for joint in list(body.findall("joint")):
            body.remove(joint)
        ET.SubElement(body, "freejoint", name="floating_panel_free")
        # Disassembly removes the original assembly exclusion, not collisions.
        for exclude in list(root.findall("./contact/exclude")):
            if asset.body in (exclude.get("body1"), exclude.get("body2")):
                root.find("contact").remove(exclude)
    for key, values in (("pos", saved["position_body_m"]), ("quat", saved["orientation_wxyz"])):
        body.set(key, " ".join(format(float(v), ".17g") for v in values))
    cfg["task_version"] = "floating-capture/1"
    # One body-fixed overview pose for all seeds; preserve the delivered lens
    # and mounting location. The wrist installation is left entirely intact.
    overview = root.find('.//camera[@name="spacecraft_overview"]')
    if overview is None:
        raise ValueError("Floating capture requires the team overview camera")
    center = np.fromstring(overview.get("pos"), sep=" ")
    z = center-np.array([.3, .65, .36]); z /= np.linalg.norm(z)
    x = np.cross([0., 0., 1.], z); x /= np.linalg.norm(x)
    orientation = matrix_to_quaternion_wxyz(np.column_stack((x, np.cross(z, x), z)))
    overview.set("quat", " ".join(format(float(v), ".17g") for v in orientation))
    cfg["floating_fixed_overview"] = dict(position_body_m=center.tolist(),
        orientation_mjcf_wxyz=orientation.tolist(), fovy_deg=float(overview.get("fovy")),
        resolution=overview.get("resolution"))
    return saved


def initialize_floating_body(scene, orbit_state, native, config):
    """Called only at scene initialization, never during interception or contact."""
    state = config["floating_initial_state"]
    body = scene.getBody(state["target_body"])
    body.setPosition(np.asarray(orbit_state["initial_position_m"])+state["position_body_m"])
    body.setVelocity(np.asarray(orbit_state["initial_velocity_m_s"])+state["velocity_body_m_s"])
    body.setAttitude(native._quaternion_to_mrp(np.asarray(state["orientation_wxyz"])))
    rotation = quaternion_wxyz_to_matrix(np.asarray(state["orientation_wxyz"]))
    body.setAttitudeRate(rotation.T@state["angular_velocity_body_rad_s"])


def validate_floating_preflight(probe, model, scene, name):
    task = create_floating_task(name, model)
    initial = np.asarray(scene["randomization"]["arm_joint_position_rad"])
    state = scene["collection_config"]["floating_initial_state"]
    goal = np.asarray(state["ik_joint_position_rad"])
    if np.any(initial[:6] < task.lower) or np.any(initial[:6] > task.upper):
        return dict(accepted=False, reason="joint_limit")
    # Full real geometry at initial pose and an open-finger approach. This is a
    # finite static path check; runtime still stops on unexpected contacts.
    target = scene["randomization"]
    overrides = {"capture_target_free": [*target["target_position_m"], *target["target_orientation_wxyz"]]}
    candidates = [np.r_[initial[:6], opening, -opening] for opening in np.linspace(0., 1., 21)]
    candidates += [np.r_[(1-fraction)*initial[:6]+fraction*goal, 1., -1.] for fraction in np.linspace(0., 1., 81)]
    count = 0
    for q in candidates:
        allowed = () if count < 21 else (task.asset.body,)
        hit = probe.pose_collision(q, name, allowed_contact_bodies=allowed, joint_values=overrides)
        count += 1
        if hit:
            return dict(accepted=False, reason="approach_collision", contact=hit, checked_poses=count)
    return dict(accepted=True, checked_poses=count, camera_to_grasp_distance_m=state["camera_to_grasp_distance_m"],
                collision_scope="native geometry at 21 opening and 81 approach poses; online moving-target collision monitoring remains enabled")
