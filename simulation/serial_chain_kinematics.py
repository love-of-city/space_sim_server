"""Renderer-independent serial-chain kinematics parsed from MJCF.

Only fixed transforms, hinge joints and one tool site are needed here.  The
authoritative dynamics and contact response still come from MJScene; this
module converts a Cartesian teleoperation command into PID joint references.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class Segment:
    position: np.ndarray
    rotation: np.ndarray
    joint_index: int | None = None
    joint_position: np.ndarray | None = None
    joint_axis: np.ndarray | None = None


@dataclass(frozen=True)
class IkResult:
    joint_velocity_rad_s: np.ndarray
    achieved_twist: np.ndarray
    residual_twist: np.ndarray
    jacobian_rank: int


class SerialChainKinematics:
    """Forward kinematics and damped least-squares velocity IK for MJCF hinges."""

    def __init__(self, segments: list[Segment], joint_names: tuple[str, ...]) -> None:
        self.segments = segments
        self.joint_names = joint_names

    @classmethod
    def from_mjcf(
        cls,
        path: str | Path,
        *,
        base_body: str,
        joint_names: tuple[str, ...],
        tool_site: str,
    ) -> "SerialChainKinematics":
        root = ET.parse(path).getroot()
        base = next((element for element in root.iter("body") if element.get("name") == base_body), None)
        if base is None:
            raise ValueError(f"MJCF base body not found: {base_body}")
        element_path = _find_descendant_path(base, tool_site)
        if element_path is None:
            raise ValueError(f"MJCF tool site {tool_site!r} is not below {base_body!r}")
        joint_lookup = {name: index for index, name in enumerate(joint_names)}
        segments: list[Segment] = []
        encountered: list[str] = []
        for element in element_path:
            position = _vector(element.get("pos"), 3, (0.0, 0.0, 0.0))
            rotation = quaternion_wxyz_to_matrix(_vector(element.get("quat"), 4, (1.0, 0.0, 0.0, 0.0)))
            joint_index: int | None = None
            joint_position: np.ndarray | None = None
            joint_axis: np.ndarray | None = None
            if element.tag == "body":
                matching = [joint for joint in element.findall("joint") if joint.get("name") in joint_lookup]
                if len(matching) > 1:
                    raise ValueError(f"body {element.get('name')} contains multiple selected joints")
                if matching:
                    joint = matching[0]
                    name = str(joint.get("name"))
                    if joint.get("type", "hinge") != "hinge":
                        raise ValueError(f"selected joint is not a hinge: {name}")
                    joint_index = joint_lookup[name]
                    joint_position = _vector(joint.get("pos"), 3, (0.0, 0.0, 0.0))
                    joint_axis = _vector(joint.get("axis"), 3, (0.0, 0.0, 1.0))
                    norm = float(np.linalg.norm(joint_axis))
                    if norm <= 1e-12:
                        raise ValueError(f"selected joint has a zero axis: {name}")
                    joint_axis = joint_axis / norm
                    encountered.append(name)
            segments.append(Segment(position, rotation, joint_index, joint_position, joint_axis))
        if encountered != list(joint_names):
            raise ValueError(f"MJCF joint path mismatch: expected {joint_names}, got {encountered}")
        return cls(segments, joint_names)

    def forward(self, joint_position_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(joint_position_rad, dtype=float)
        if q.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} joint positions, got {q.shape}")
        transform = np.eye(4)
        for segment in self.segments:
            transform = transform @ _transform(segment.position, segment.rotation)
            if segment.joint_index is not None:
                assert segment.joint_position is not None and segment.joint_axis is not None
                transform = (
                    transform
                    @ _translation(segment.joint_position)
                    @ _transform(np.zeros(3), axis_angle_to_matrix(segment.joint_axis, q[segment.joint_index]))
                    @ _translation(-segment.joint_position)
                )
        return transform[:3, 3].copy(), transform[:3, :3].copy()

    def jacobian(self, joint_position_rad: np.ndarray) -> np.ndarray:
        q = np.asarray(joint_position_rad, dtype=float)
        if q.shape != (len(self.joint_names),):
            raise ValueError(f"expected {len(self.joint_names)} joint positions, got {q.shape}")
        transform = np.eye(4)
        axes = np.zeros((len(self.joint_names), 3))
        origins = np.zeros((len(self.joint_names), 3))
        for segment in self.segments:
            transform = transform @ _transform(segment.position, segment.rotation)
            if segment.joint_index is not None:
                assert segment.joint_position is not None and segment.joint_axis is not None
                origins[segment.joint_index] = (
                    transform @ np.array([*segment.joint_position, 1.0])
                )[:3]
                axes[segment.joint_index] = transform[:3, :3] @ segment.joint_axis
                transform = (
                    transform
                    @ _translation(segment.joint_position)
                    @ _transform(np.zeros(3), axis_angle_to_matrix(segment.joint_axis, q[segment.joint_index]))
                    @ _translation(-segment.joint_position)
                )
        tool_position = transform[:3, 3]
        jacobian = np.zeros((6, len(self.joint_names)))
        for index in range(len(self.joint_names)):
            jacobian[:3, index] = np.cross(axes[index], tool_position - origins[index])
            jacobian[3:, index] = axes[index]
        return jacobian

    def inverse_velocity(
        self,
        joint_position_rad: np.ndarray,
        desired_twist: np.ndarray,
        *,
        damping: float = 0.06,
        angular_weight: float = 0.30,
        joint_velocity_limits: np.ndarray | None = None,
    ) -> IkResult:
        twist = np.asarray(desired_twist, dtype=float)
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            raise ValueError("desired twist must contain six finite values")
        jacobian = self.jacobian(np.asarray(joint_position_rad, dtype=float))
        weights = np.diag([1.0, 1.0, 1.0, angular_weight, angular_weight, angular_weight])
        weighted_jacobian = weights @ jacobian
        weighted_twist = weights @ twist
        lhs = weighted_jacobian.T @ weighted_jacobian + float(damping) ** 2 * np.eye(jacobian.shape[1])
        velocity = np.linalg.solve(lhs, weighted_jacobian.T @ weighted_twist)
        if joint_velocity_limits is not None:
            limits = np.asarray(joint_velocity_limits, dtype=float)
            if limits.shape != velocity.shape:
                raise ValueError("joint velocity limit shape does not match the chain")
            velocity = np.clip(velocity, -limits, limits)
        achieved = jacobian @ velocity
        return IkResult(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=twist - achieved,
            jacobian_rank=int(np.linalg.matrix_rank(jacobian, tol=1e-5)),
        )


def quaternion_wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale, (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale])
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([(matrix[0, 2] - matrix[2, 0]) / scale, (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale])
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([(matrix[1, 0] - matrix[0, 1]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale])
    quaternion /= np.linalg.norm(quaternion)
    return -quaternion if quaternion[0] < 0.0 else quaternion


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    unit = np.asarray(axis, dtype=float)
    unit = unit / np.linalg.norm(unit)
    x, y, z = unit
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def rotation_matrix_to_vector(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    cosine = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) * 0.5))
    angle = math.acos(cosine)
    skew_vector = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if angle < 1e-8:
        return 0.5 * skew_vector
    return angle * skew_vector / (2.0 * math.sin(angle))


def _find_descendant_path(base: ET.Element, tool_site: str) -> list[ET.Element] | None:
    for child in base:
        if child.tag == "site" and child.get("name") == tool_site:
            return [child]
        if child.tag not in {"body", "frame"}:
            continue
        nested = _find_descendant_path(child, tool_site)
        if nested is not None:
            return [child, *nested]
    return None


def _vector(text: str | None, length: int, default: tuple[float, ...]) -> np.ndarray:
    values = np.array(default if text is None else [float(value) for value in text.split()], dtype=float)
    if values.shape != (length,):
        raise ValueError(f"expected {length} values, got {values.shape}")
    return values


def _translation(position: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, 3] = position
    return result


def _transform(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    result = _translation(position)
    result[:3, :3] = rotation
    return result
