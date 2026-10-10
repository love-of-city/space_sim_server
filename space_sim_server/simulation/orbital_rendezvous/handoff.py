"""Validate and restore a successful flight into the original articulated scene."""

import hashlib
import json
from pathlib import Path

import numpy as np


def load_handoff(path: Path, source_model: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    return validate_handoff(data, source_model)


def validate_handoff(data, source_model: Path):
    if (
        data.get("schema") != "spacesim-near-field-handoff/1"
        or data.get("accepted") is not True
    ):
        raise ValueError(
            "Only an accepted team flight can initialize a near-field scene"
        )
    if hashlib.sha256(source_model.read_bytes()).hexdigest() != data.get(
        "source_sha256"
    ):
        raise ValueError("Handoff source model differs from the near-field model")
    bodies = data["state"]["bodies"]
    if len(bodies) != 2 or {b["body_id"] for b in bodies} != {
        "rendezvous/chaser",
        "rendezvous/target",
    }:
        raise ValueError("Handoff requires exactly one chaser and one target")
    for body in bodies:
        for key, count in (
            ("position_m", 3),
            ("velocity_m_s", 3),
            ("attitude_wxyz", 4),
            ("angular_velocity_rad_s", 3),
        ):
            value = np.asarray(body[key])
            if value.shape != (count,) or not np.isfinite(value).all():
                raise ValueError(f"Invalid handoff {key}")
        if not np.isclose(np.linalg.norm(body["attitude_wxyz"]), 1.0, atol=1e-6):
            raise ValueError("Handoff quaternion must be normalized")
    return data


def apply_handoff(scene, data):
    """Call once after InitializeSimulation, before the first near-field step.

    The receiving scenario must preserve the handoff epoch and navigation
    covariance; this function only restores physical coordinates and wheel states.
    No continuing mission loop may invoke it.
    """
    from Basilisk.utilities import RigidBodyKinematics as rbk

    for item in data["stowed_joints"]:
        joint = scene.getBody(item["body"]).getScalarJoint(item["joint"]["name"])
        joint.setPosition(float(item["position"]))
        joint.setVelocity(0.0)
    for wheel in data["reaction_wheels"]:
        joint = scene.getBody(wheel["body"]).getScalarJoint(wheel["joint"])
        joint.setPosition(float(wheel["position_rad"]))
        joint.setVelocity(float(wheel["speed_rad_s"]))
    for body in data["state"]["bodies"]:
        name = (
            "cubesat_bus"
            if body["body_id"] == "rendezvous/chaser"
            else "capture_target"
        )
        target = scene.getBody(name)
        target.setPosition(body["position_m"])
        target.setVelocity(body["velocity_m_s"])
        # EP2MRP flips a negative scalar quaternion with unary minus. JSON
        # supplies a list, so normalize the representation at this boundary.
        target.setAttitude(rbk.EP2MRP(np.asarray(body["attitude_wxyz"], dtype=float)).tolist())
        target.setAttitudeRate(body["angular_velocity_rad_s"])
    return data["state"]["sim_time_s"]
