"""Connector engagement and guidance, with finite-range axial dry friction.

No axial displacement spring: the native dynamics core solves bounded static
friction and constant kinetic friction. This module owns socket engagement,
lateral/angular guidance and the seated backstop, without rewriting geometry.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix,
)


@dataclass(frozen=True)
class RetentionParameters:
    static_friction_n: float = 20.0
    sliding_friction_n: float = 20.0
    friction_travel_m: float = 0.008
    lateral_stiffness_n_m: float = 1000.0
    lateral_damping_ns_m: float = 4.0
    lateral_force_limit_n: float = 10.0
    rotational_stiffness_nm_rad: float = 0.2
    rotational_damping_nms_rad: float = 0.002
    torque_limit_nm: float = 0.1
    backstop_stiffness_n_m: float = 2000.0
    backstop_force_limit_n: float = 15.0
    capture_radius_m: float = 0.001
    seated_tolerance_m: float = 0.001
    capture_angle_deg: float = 5.0
    lateral_release_m: float = 0.02
    angular_release_deg: float = 45.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be finite and positive")
        if self.capture_radius_m >= self.lateral_release_m:
            raise ValueError("capture_radius_m must be smaller than lateral_release_m")
        if self.sliding_friction_n > self.static_friction_n:
            raise ValueError("sliding_friction_n must not exceed static_friction_n")
        if self.seated_tolerance_m >= self.friction_travel_m / 2:
            raise ValueError("seated_tolerance_m must be inside the friction travel")
        if not 0 < self.capture_angle_deg < self.angular_release_deg < 180:
            raise ValueError("require 0 < capture_angle_deg < angular_release_deg < 180")


@dataclass(frozen=True)
class RetentionSocket:
    name: str
    position_body_m: np.ndarray
    rotation_body: np.ndarray
    plug_rotation_body: np.ndarray
    initial: bool = False
    keyed: bool = True


def load_retention_parameters(path: str | Path | None = None) -> RetentionParameters | None:
    source = Path(path or os.environ.get("SPACE_SIM_PLUG_RETENTION_CONFIG") or
                  Path(__file__).resolve().parents[1] / "config/plug_retention.json")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or set(payload) - {"enabled", "parameters"}:
        raise ValueError("plug retention config accepts only enabled and parameters")
    if not isinstance(payload.get("enabled"), bool):
        raise ValueError("plug retention enabled must be a boolean")
    parameters = RetentionParameters(**payload.get("parameters", {}))
    return parameters if payload["enabled"] else None


def model_retention_sockets(model_path: Path) -> dict[str, tuple[RetentionSocket, ...]]:
    """Read seat frames from existing model poses/sites; never rewrite the MJCF."""
    from simulation.scripted_tasks.plug_insert import fixed_site_pose
    from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES

    root = ET.parse(model_path).getroot()
    result = {}
    for profile in PLUG_PROFILES.values():
        body = root.find(f'./worldbody/body[@name="{profile.body}"]')
        if body is None or (body.find("freejoint") is None and body.find('joint[@type="free"]') is None):
            continue
        position = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
        rotation = quaternion_wxyz_to_matrix(np.fromstring(body.get("quat", "1 0 0 0"), sep=" "))
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError(f"Invalid initial position for {profile.body}")
        pickup_site = f"{profile.prefix}_pickup"
        seat_rotation = (fixed_site_pose(root, pickup_site)[1]
                         if root.find(f'.//site[@name="{pickup_site}"]') is not None else np.eye(3))
        sockets = [RetentionSocket(f"{profile.prefix}_initial", position, seat_rotation, rotation,
                                   initial=True, keyed=profile.keyed)]
        for suffix in ("_insert_40p1", "_insert_40p5"):
            name = profile.prefix + suffix
            if root.find(f'.//site[@name="{name}"]') is None:
                continue
            hole, orientation = fixed_site_pose(root, name)
            # Existing insertion tasks seat the flange at the hole surface;
            # plug +Z points into the hole, socket +Z points out of it.
            seat = hole + orientation[:, 2] * profile.flange_m
            sockets.append(RetentionSocket(name, seat, orientation,
                                            orientation @ np.diag([1., -1., -1.]), keyed=profile.keyed))
        result[profile.body] = tuple(sockets)
    return result


def axial_backstop_force(displacement_m: float, parameters: RetentionParameters) -> float:
    """Seated-end penetration resistance only; no spring along the sliding zone."""
    z, p = float(displacement_m), parameters
    if not math.isfinite(z):
        raise ValueError("axial displacement must be finite")
    if z < 0:
        return min(-p.backstop_stiffness_n_m * z, p.backstop_force_limit_n)
    return 0.0


def _limited(vector, maximum):
    magnitude = float(np.linalg.norm(vector))
    return vector * min(1., maximum / magnitude) if magnitude else vector


def _cross3(a, b):
    """Cross product of two 3-vectors without NumPy's broadcasting setup."""
    return np.array((a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]))


def _axis_error(desired, actual, fallback_axis):
    """Shortest tilt rotation, leaving a round plug's axial roll unconstrained."""
    cross = _cross3(desired, actual)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(desired, actual), -1., 1.))
    if sine < 1e-10:
        return np.zeros(3) if cosine >= 0 else math.pi * fallback_axis
    return cross * (math.atan2(sine, cosine) / sine)


def _rotation_error(rotation):
    # A quaternion log also handles exact half-turns, where the usual
    # skew(R)/sin(angle) expression is singular and can report zero error.
    q = matrix_to_quaternion_wxyz(rotation)
    sine = float(np.linalg.norm(q[1:]))
    return 2 * q[1:] if sine < 1e-10 else q[1:] * (2 * math.atan2(sine, q[0]) / sine)


class PlugFriction:
    """One plug, several compatible seats, one force-producing owner at a time."""

    def __init__(self, sockets: tuple[RetentionSocket, ...], parameters: RetentionParameters):
        if not sockets or len({s.name for s in sockets}) != len(sockets):
            raise ValueError("retention requires distinct compatible sockets")
        self.sockets, self.parameters = sockets, parameters
        self._capture_angle_rad = math.radians(parameters.capture_angle_deg)
        self._angular_release_rad = math.radians(parameters.angular_release_deg)
        self.reset()

    def reset(self):
        self.active: RetentionSocket | None = None
        self.first_sample = True
        self.status = "free"
        self.displacement_m = None

    def wrench(self, relative_position, relative_rotation, relative_velocity, relative_omega):
        p = self.parameters
        values = [np.asarray(v, dtype=float) for v in
                  (relative_position, relative_rotation, relative_velocity, relative_omega)]
        if [v.shape for v in values] != [(3,), (3, 3), (3,), (3,)] or not all(np.isfinite(v).all() for v in values):
            raise ValueError("retention requires finite relative pose and velocity")
        position, rotation, velocity, omega = values

        def coordinates(socket):
            delta = socket.rotation_body.T @ (position - socket.position_body_m)
            speed = socket.rotation_body.T @ velocity
            error = (_rotation_error(rotation @ socket.plug_rotation_body.T) if socket.keyed else
                     _axis_error(socket.plug_rotation_body[:, 2], rotation[:, 2], socket.plug_rotation_body[:, 0]))
            return delta, speed, error

        if self.active is not None:
            delta, speed, error = coordinates(self.active)
            if (delta[2] >= p.friction_travel_m or np.linalg.norm(delta[:2]) > p.lateral_release_m
                    or np.linalg.norm(error) > self._angular_release_rad):
                self.active = None
        if self.active is None:
            # No attraction at a distance or across a wrong key orientation.
            # An initialized plug may already be seated or partially inserted.
            for socket in self.sockets:
                delta, speed, error = coordinates(socket)
                initial_capture = self.first_sample and (socket.initial or abs(speed[2]) <= 1e-6)
                if (np.linalg.norm(delta[:2]) <= p.capture_radius_m
                        and np.linalg.norm(error) <= self._capture_angle_rad
                        and -.001 <= delta[2] < p.friction_travel_m
                        and (speed[2] < -1e-6 or initial_capture)):
                    self.active = socket
                    break
        self.first_sample = False
        if self.active is None:
            self.status, self.displacement_m = "free", None
            return np.zeros(3), np.zeros(3)
        # The active/capture checks already computed this socket's coordinates
        # from the same immutable input sample; reuse them for the wrench.
        force = np.zeros(3)
        force[:2] = _limited(-p.lateral_stiffness_n_m * delta[:2] - p.lateral_damping_ns_m * speed[:2],
                              p.lateral_force_limit_n)
        # Axial dry friction is resolved by the physics port, with the current
        # substep's external loads. A sign(velocity) held force would chatter.
        force[2] = axial_backstop_force(delta[2], p)
        damping_omega = omega
        if not self.active.keyed:
            axis = rotation[:, 2]
            damping_omega = omega - np.dot(omega, axis) * axis
        torque = _limited(-p.rotational_stiffness_nm_rad * error - p.rotational_damping_nms_rad * damping_omega,
                          p.torque_limit_nm)
        self.displacement_m = float(delta[2])
        near_seat = (abs(delta[2]) < p.seated_tolerance_m
                     and np.linalg.norm(delta[:2]) <= p.capture_radius_m
                     and np.linalg.norm(error) <= self._capture_angle_rad)
        self.status = ("retained" if near_seat else "held" if abs(speed[2]) <= 1e-6
                       else "extracting" if speed[2] > 0 else "inserting")
        return self.active.rotation_body @ force, torque


def paired_com_wrenches(force_n, torque_n, point_n, plug_com_n, base_com_n,
                        plug_rotation_n, base_rotation_n):
    """Equal/opposite pair at one interface, shifted to each body's COM.

    Returns inertial forces and body-axis torques for the existing physics port.
    The offset term is essential for conservation of angular momentum.
    """
    plug_torque = torque_n + _cross3(point_n - plug_com_n, force_n)
    base_torque = -torque_n + _cross3(point_n - base_com_n, -force_n)
    return (force_n, plug_rotation_n.T @ plug_torque,
            -force_n, base_rotation_n.T @ base_torque)
