"""Object-local grasp candidates, selected only after the scene is frozen."""
from dataclasses import dataclass, asdict
import hashlib
import json
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict

from .floating_assets import FLOATING_TASKS, FloatingAsset
from .motion import solve_pose
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz, rotation_matrix_to_vector, axis_angle_to_matrix


@dataclass(frozen=True)
class GraspCandidate:
    id: str
    grip_point: tuple
    closing_axis: tuple
    approach_axis: tuple
    pregrasp_m: float = .025

    def __post_init__(self):
        FloatingAsset("", "", "", self.grip_point, self.closing_axis, self.approach_axis)
        if not self.id or not np.isfinite(self.pregrasp_m) or not 0 < self.pregrasp_m <= .1:
            raise ValueError("Invalid grasp candidate ID or pregrasp distance")

    def snapshot(self):
        return json.loads(json.dumps(asdict(self)))

    def pose(self, position, rotation):
        x, z = rotation @ self.closing_axis, rotation @ self.approach_axis
        return position + rotation @ self.grip_point, np.column_stack((x, np.cross(z, x), z))


class FloatingGraspConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    selection: Literal["ranked", "random"] = "ranked"
    candidate_ids: tuple[str, ...] = ()


def grasp_candidates(name):
    """Small, explicit local neighborhoods; native geometry decides feasibility."""
    asset = FLOATING_TASKS[name]
    p, x, z = map(np.asarray, (asset.grip_point, asset.closing_axis, asset.approach_axis))
    candidates = [GraspCandidate("baseline", tuple(p), tuple(x), tuple(z))]
    # A half turn exchanges the two jaws without changing the contact surfaces.
    candidates.append(GraspCandidate("jaw-swap", tuple(p), tuple(-x), tuple(z)))
    # A depth neighborhood tests whether the original TCP lies on a fingertip taper.
    for tag, offset in (("shallow", -.005), ("deep", .005)):
        candidates.append(GraspCandidate(tag, tuple(p+offset*z), tuple(x), tuple(z)))
    axial = ("plug", "aviation_plug", "guide_plug", "bolt", "fastener", "locking_rod",
             "sun_sensor", "gnss_antenna", "data_antenna")
    if name in {f"floating_{kind}_capture" for kind in axial}:
        candidates.append(GraspCandidate("quarter-turn", tuple(p), tuple(np.cross(z, x)), tuple(z)))
    if name == "floating_data_antenna_capture":
        # Plate faces z=0.002114..0.006130; test a jaw pair across thickness.
        for edge in (-1, 1):
            for depth in (.027, .017):
                candidates.append(GraspCandidate(f"thin-edge-{edge:+d}-{depth:.3f}",
                    (edge*depth, .011, .00412167221513), (0., 0., 1.), (-float(edge), 0., 0.)))
    if name in {"floating_panel_capture", "floating_inner_panel_capture", "floating_board1_capture",
                "floating_board2_capture", "floating_board3_capture"}:
        # Slide along the known thin edge, maintaining the physical closing axis.
        tangent = np.cross(z, x)
        for offset in (-.04, .04):
            candidates.append(GraspCandidate(f"edge-slide-{offset:+.2f}", tuple(p+offset*tangent), tuple(x), tuple(z)))
    if name == "floating_drawer_capture":
        # STL has an open central handle: its origin has no opposed surfaces.
        # The two front shoulders at x=+/-40 mm provide real 60 mm jaw spans.
        for x in (-.04, .04):
            candidates.append(GraspCandidate(f"front-{x:+.3f}", (x, 0., .025), (0., 1., 0.), (0., 0., 1.)))
        # Side candidates remain orientation-dependent alternatives.
        for height in (.025, .06, .095):
            candidates.append(GraspCandidate(f"side-{height:.3f}", (.04, 0., height), (0., 1., 0.), (-1., 0., 0.)))
    return tuple(candidates)


def requested_candidates(name, scene):
    options = FloatingGraspConfig.model_validate(scene.get("floating_grasp", {}))
    candidates = grasp_candidates(name)
    ids = [c.id for c in candidates]
    if (len(set(options.candidate_ids)) != len(options.candidate_ids)
            or any(i not in ids for i in options.candidate_ids)):
        raise ValueError(f"Invalid candidate_ids for {name}; available: {ids}")
    return options, tuple(c for c in candidates if not options.candidate_ids or c.id in options.candidate_ids)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _context(name, model, scene):
    options, candidates = requested_candidates(name, scene)
    cfg = scene["collection_config"]
    return dict(task=name, seed=scene["seed"], model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                release=cfg["floating_initial_state"], objects=cfg["floating_objects"],
                randomization=scene["randomization"],
                controller=cfg["floating_controller"], options=options.model_dump(mode="json"),
                candidates=[c.snapshot() for c in candidates])


def selected_grasp(name, scene):
    record = scene.get("collection_config", {}).get("floating_grasp")
    if record is None:
        raise ValueError("Floating grasp must be selected by native preflight before execution")
    options, candidates = requested_candidates(name, scene)
    for candidate in candidates:
        if candidate.snapshot() == record.get("candidate"):
            return candidate
    raise ValueError("Saved floating grasp does not match current candidate definition")


def validate_selection(name, model, scene):
    record = scene["collection_config"]["floating_grasp"]
    selected_grasp(name, scene)
    content = {k: v for k, v in record.items() if k != "sha256"}
    if (record.get("schema") != "floating-grasp/1" or record.get("context_sha256") != _digest(_context(name, model, scene))
            or record.get("sha256") != _digest(content)):
        raise ValueError("Saved floating grasp does not match scene, model or selection record")
    return record


def evaluate_grasps(probe, task, initial, position, rotation, instances, overrides, candidates):
    """Shared geometric selection for one arm/target in a complete native scene."""
    name = task.name
    moving = tuple(i["instance_id"] for i in instances)
    target_joint = next(i["free_joint"] for i in instances if i["instance_id"] == task.asset.body)
    results, feasible = [], []
    if np.any(initial[:6] < task.lower) or np.any(initial[:6] > task.upper):
        return [], dict(accepted=False, reason="joint_limit")
    def collision(q, *, carry=None, allow=False):
        joints = dict(overrides)
        if carry is not None:
            joints[target_joint] = carry
        return probe.pose_collision(q, name, allowed_contact_bodies=(task.asset.body,) if allow else (),
            joint_values=joints, mechanism_clearance_m=0., moving_bodies=moving)
    for opening in np.linspace(0., 1., 21):
        hit = collision(np.r_[initial[:6], opening, -opening])
        if hit:
            return [], dict(accepted=False, reason="initial_collision", contact=hit)
    for candidate in candidates:
        row = dict(id=candidate.id, accepted=False)
        results.append(row)
        point, desired = candidate.pose(position, rotation)
        prepoint = point - candidate.pregrasp_m*desired[:, 2]
        try:
            pre = solve_pose(task.chain, prepoint, desired, initial[:6], task.lower, task.upper)
            goal = solve_pose(task.chain, point, desired, pre, task.lower, task.upper)
        except ValueError:
            row["reason"] = "grasp_ik"; continue
        margin = float(np.min(np.minimum(np.minimum(pre-task.lower, task.upper-pre),
                                          np.minimum(goal-task.lower, task.upper-goal))))
        if margin < .02:
            row["reason"] = "joint_margin"; continue
        hit = None
        path_q = initial[:6].copy()
        start_point, start_rotation = task.chain.forward(path_q)
        try:
            for end_point, end_rotation, allow in ((prepoint, desired, False), (point, desired, False)):
                rotation_vector = rotation_matrix_to_vector(end_rotation@start_rotation.T)
                angle = np.linalg.norm(rotation_vector)
                for t in np.linspace(0., 1., 41):
                    path_rotation = (axis_angle_to_matrix(rotation_vector/angle, t*angle)@start_rotation
                                     if angle > 1e-10 else start_rotation)
                    previous_q = path_q.copy()
                    path_q = solve_pose(task.chain, (1-t)*start_point+t*end_point, path_rotation,
                                        path_q, task.lower, task.upper)
                    margin = min(margin, float(np.min(np.minimum(path_q-task.lower, task.upper-path_q))))
                    if margin < .02 or np.max(np.abs(path_q-previous_q)) > .3:
                        raise ValueError("Discontinuous or limit-adjacent approach")
                    hit = collision(np.r_[path_q, 1., -1.], allow=allow)
                    if hit: break
                if hit: break
                if not np.array_equal(end_point, point):
                    pre = path_q.copy()
                start_point, start_rotation = end_point, end_rotation
        except ValueError:
            row["reason"] = "approach_ik"; continue
        goal = path_q
        if hit:
            row.update(reason="approach_collision", contact=hit); continue
        collision(np.r_[goal, 1., -1.], allow=True)  # restore all free bodies before jaw sweep
        try:
            opening = probe.grasp_opening(np.r_[goal, 1., -1.], task.asset.body)
        except ValueError as exc:
            row.update(reason="no_bilateral_contact", detail=str(exc)); continue
        first_contacts, max_penetration, hit = {}, 0., None
        for closing in np.linspace(1., opening, 101):
            hit = collision(np.r_[goal, closing, -closing], allow=True)
            if hit: break
            for _, _, first, second, distance in probe.contacts(np.r_[goal, closing, -closing]):
                names = {probe.names[int(first)], probe.names[int(second)]}
                if task.asset.body in names and distance <= 0:
                    for finger in names.intersection(task.arm.finger_bodies):
                        first_contacts.setdefault(finger, float(closing))
                        max_penetration = max(max_penetration, -float(distance))
        row.update(first_contact_openings=first_contacts, max_finger_penetration_m=max_penetration)
        if hit:
            row.update(reason="closing_collision", contact=hit); continue
        # A frozen target cannot yield or recenter under one-sided contact.
        # Penalize asymmetry rather than rejecting known dynamically valid grasps.
        row["contact_asymmetry_warning"] = max_penetration > .0005
        # The private probe transports the target with a fixed tool-relative pose.
        # This predicts clearance only, not frictional retention.
        bp, br = probe.body_pose("cubesat_bus")
        target_p, target_r = probe.body_pose(task.asset.body)
        if not (np.allclose(target_p, bp+br@position, atol=1e-5, rtol=0)
                and np.allclose(target_r, br@rotation, atol=1e-5, rtol=0)):
            raise ValueError("Private carry probe disagrees with the frozen target pose")
        local_position, local_rotation = desired.T@(position-point), desired.T@rotation
        carry_q = goal.copy()
        carry_margin = margin
        try:
            for t in np.linspace(0., 1., 31):
                tcp = (1-t)*point+t*np.asarray(task.config.retract_position_body_m)
                previous_q = carry_q.copy()
                carry_q = solve_pose(task.chain, tcp, desired, carry_q, task.lower, task.upper)
                if np.max(np.abs(carry_q-previous_q)) > .3:
                    raise ValueError("Discontinuous carry")
                carry_margin = min(carry_margin, float(np.min(np.minimum(carry_q-task.lower, task.upper-carry_q))))
                pos = bp+br@(tcp+desired@local_position)
                quat = matrix_to_quaternion_wxyz(br@desired@local_rotation)
                hit = collision(np.r_[carry_q, opening, -opening], carry=[*pos, *quat], allow=True)
                if hit: break
        except ValueError:
            row["reason"] = "retract_ik"; continue
        if hit:
            row.update(reason="retract_collision", contact=hit, path_fraction=float(t),
                       tcp_position_body_m=tcp.tolist(), target_position_world_m=pos.tolist()); continue
        if carry_margin < .02:
            row["reason"] = "retract_joint_margin"; continue
        score = carry_margin - .05*float(np.linalg.norm(pre-initial[:6])) - 200.*max_penetration
        row.update(accepted=True, score=score, joint_margin_rad=carry_margin, contact_opening_rad=opening)
        feasible.append((candidate, row, goal, pre, carry_q))
    if not feasible:
        return [], dict(accepted=False, reason="no_feasible_grasp", candidates=results)
    return feasible, dict(accepted=True, candidates=results)


def select_grasp(probe, model, scene, name):
    """Private native geometry only. No state is imposed on the actual simulation."""
    from .floating_capture import FloatingCaptureTask, floating_controller_config
    from .randomization import generator
    cfg = scene["collection_config"]
    if "floating_grasp" in cfg:
        record = validate_selection(name, model, scene)
        return dict(accepted=True, selected=record["candidate"]["id"], replay=True)
    options, candidates = requested_candidates(name, scene)
    task = FloatingCaptureTask(model, floating_controller_config(name, scene))
    state = cfg["floating_initial_state"]
    initial = np.asarray(scene["randomization"]["arm_joint_position_rad"])
    position = np.asarray(state["position_body_m"])
    rotation = quaternion_wxyz_to_matrix(np.asarray(state["orientation_wxyz"]))
    instances = cfg["floating_objects"]["instances"]
    target = scene["randomization"]
    overrides = {"capture_target_free": [*target["target_position_m"], *target["target_orientation_wxyz"]]}
    feasible, report = evaluate_grasps(probe, task, initial, position, rotation, instances, overrides, candidates)
    if not feasible:
        return report
    results = report["candidates"]
    if options.selection == "random":
        chosen = feasible[int(generator(scene["seed"], "floating-grasp-v1").integers(len(feasible)))]
    else:
        chosen = max(feasible, key=lambda item: (not item[1]["contact_asymmetry_warning"],
                                                item[0].id == "baseline", item[1]["score"]))
    candidate, row, goal, pre, carry_q = chosen
    record = dict(schema="floating-grasp/1", context_sha256=_digest(_context(name, model, scene)),
                  options=options.model_dump(mode="json"), candidate=candidate.snapshot(),
                  grasp_joint_position_rad=goal.tolist(), pregrasp_joint_position_rad=pre.tolist(),
                  retract_joint_position_rad=carry_q.tolist(),
                  candidates=results, collision_scope="finite static approach and fixed-relative-pose carry; dynamic retention checked at runtime")
    record["sha256"] = _digest(record)
    cfg["floating_grasp"] = record
    return dict(accepted=True, selected=candidate.id, candidates=results)


def prepare_floating_grasp(model, scene, name):
    """Shared direct-record/direct-run preparation, before metadata or task creation."""
    if "floating_grasp" in scene["collection_config"]:
        return validate_selection(name, model, scene)
    from .drawer_initialization import NativePoseProbe
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    import sys
    from pathlib import Path
    if "bsk_native_cubesat_so101_grasp" not in sys.modules:
        load_native_grasp_module(Path(__file__).resolve().parents[2]/"model/SARM/platform")
    probe = NativePoseProbe(model)
    try:
        report = select_grasp(probe, model, scene, name)
    finally:
        probe.close()
    if not report["accepted"]:
        raise ValueError("initialization_rejected: " + json.dumps(report))
    return scene["collection_config"]["floating_grasp"]
