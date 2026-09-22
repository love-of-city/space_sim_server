"""Renderer-independent serial-chain kinematics parsed from MJCF.

Only fixed transforms, hinge joints and one tool site are needed here.  The
authoritative dynamics and contact response still come from MJScene; this
module converts a Cartesian teleoperation command into PID joint references.
"""

from __future__ import annotations

from collections import OrderedDict
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
    velocity_scale: float = 1.0
    minimum_singular_value: float = 0.0
    condition_number: float = math.inf
    damping: float = 0.0
    nullspace_correction_norm: float = 0.0
    # Read-only evidence from the existing limiter; never changes the IK step.
    limit_reasons: tuple[dict, ...] = ()


@dataclass(frozen=True)
class _StepContext:
    """Validated geometry, joint bounds and spectrum of one differential-IK step."""

    joint_position: np.ndarray
    twist: np.ndarray
    jacobian: np.ndarray
    singular_values: np.ndarray
    rank: int
    lower: np.ndarray
    upper: np.ndarray
    right_vectors: np.ndarray | None = None

    @property
    def minimum_singular_value(self) -> float:
        return float(self.singular_values[-1]) if self.singular_values.size else 0.0

    @property
    def condition_number(self) -> float:
        if not self.singular_values.size or self.singular_values[-1] <= 0.0:
            return math.inf
        return float(self.singular_values[0] / self.singular_values[-1])

    def nullspace_projector(self, *, rcond: float) -> np.ndarray:
        """Return ``I - pinv(J) @ J`` reusing the singular value decomposition."""

        size = len(self.joint_position)
        if self.right_vectors is None:
            raise ValueError("right singular vectors are required for nullspace control")
        if not math.isfinite(rcond) or rcond <= 0.0:
            raise ValueError("rcond must be positive and finite")
        if self.singular_values.size == 0:
            return np.eye(size)
        task_directions = self.singular_values > rcond * float(self.singular_values[0])
        if not np.any(task_directions):
            return np.eye(size)
        task_basis = self.right_vectors[task_directions]
        return np.eye(size) - task_basis.T @ task_basis


class SerialChainKinematics:
    """Forward kinematics plus three differential IK solvers for MJCF hinges.

    ``inverse_velocity`` is a legacy weighted reference, ``inverse_velocity_bounded``
    is the strict pose-preserving fallback, and ``inverse_velocity_ik_pose`` is the
    robosuite ``IK_POSE``-style default used by live teleoperation.
    """

    def __init__(self, segments: list[Segment], joint_names: tuple[str, ...]) -> None:
        self.segments = segments
        self.joint_names = joint_names
        # MJCF geometry is constant for the lifetime of a chain. Precompute
        # only its fixed transforms; poses remain keyed by joint VALUES, never
        # the identity of the mutable controller arrays.
        self._fixed_transforms = tuple(_transform(s.position, s.rotation) for s in segments)
        self._joint_offsets = tuple(
            (_translation(s.joint_position), _translation(-s.joint_position))
            if s.joint_index is not None else None for s in segments
        )
        self._geometry_cache: OrderedDict[tuple[float, ...], tuple[np.ndarray, ...]] = OrderedDict()

    def _geometry(self, joint_position_rad: np.ndarray) -> tuple[np.ndarray, ...]:
        """One exact chain traversal shared by FK, Jacobian and posture queries.

        A small bounded cache covers measured/reference/line-search poses. It
        stores geometry, NOT native physics state or IK results; changing any
        joint invalidates the key. Public methods always return independent
        arrays so callers cannot corrupt a later query.
        """
        q = np.asarray(joint_position_rad, dtype=float)
        if q.shape != (len(self.joint_names),) or not np.all(np.isfinite(q)):
            raise ValueError("expected finite joint positions matching the chain")
        key = tuple(q)
        cached = self._geometry_cache.get(key)
        if cached is not None:
            self._geometry_cache.move_to_end(key)
            return cached
        transform = np.eye(4)
        origins = np.zeros((len(self.joint_names), 3))
        axes = np.zeros_like(origins)
        for segment, fixed, offsets in zip(self.segments, self._fixed_transforms, self._joint_offsets):
            transform = transform @ fixed
            if segment.joint_index is not None:
                index = segment.joint_index
                origins[index] = (transform @ np.array([*segment.joint_position, 1.0]))[:3]
                axes[index] = transform[:3, :3] @ segment.joint_axis
                transform = (transform @ offsets[0]
                    @ _transform(np.zeros(3), axis_angle_to_matrix(segment.joint_axis, q[index]))
                    @ offsets[1])
        jacobian = np.empty((6, len(self.joint_names)))
        jacobian[:3] = np.cross(axes, transform[:3, 3] - origins).T
        jacobian[3:] = axes.T
        result = (transform, origins, axes, jacobian)
        self._geometry_cache[key] = result
        if len(self._geometry_cache) > 8:
            self._geometry_cache.popitem(last=False)
        return result

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
        transform = self._geometry(joint_position_rad)[0]
        return transform[:3, 3].copy(), transform[:3, :3].copy()

    def joint_origins(self, joint_position_rad: np.ndarray) -> np.ndarray:
        """Joint centres in the same base-body frame as :meth:`forward`."""
        return self._geometry(joint_position_rad)[1].copy()

    def joint_geometry(self, joint_position_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Joint centres and unit rotation axes, ordered along the serial chain."""
        _, origins, axes, _ = self._geometry(joint_position_rad)
        return origins.copy(), axes.copy()

    def relative_joint_height(
        self, joint_position_rad: np.ndarray, *, shoulder_joint: str = "joint2",
        elbow_joint: str = "joint3", up_axis=(0.0, 0.0, 1.0),
    ) -> tuple[float, np.ndarray]:
        """Geometric elbow height AND its analytic joint-space gradient.

        Up is defined in the base-body frame, independent of world attitude.
        Uses joint centres/axes, not a robot-specific joint-angle sign rule.
        """
        axis = np.asarray(up_axis, dtype=float)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-12:
            raise ValueError("up_axis must be a finite nonzero 3-vector")
        axis = axis / np.linalg.norm(axis)
        shoulder = self.joint_names.index(shoulder_joint)
        elbow = self.joint_names.index(elbow_joint)
        origins, axes = self.joint_geometry(joint_position_rad)
        gradient = np.zeros(len(self.joint_names))
        for index in range(len(self.joint_names)):
            # Each joint origin is affected only by its ancestors, not itself.
            if index < elbow:
                gradient[index] += axis @ np.cross(axes[index], origins[elbow] - origins[index])
            if index < shoulder:
                gradient[index] -= axis @ np.cross(axes[index], origins[shoulder] - origins[index])
        return float(axis @ (origins[elbow] - origins[shoulder])), gradient

    def jacobian(self, joint_position_rad: np.ndarray) -> np.ndarray:
        return self._geometry(joint_position_rad)[3].copy()

    def inverse_velocity(
        self,
        joint_position_rad: np.ndarray,
        desired_twist: np.ndarray,
        *,
        damping: float = 0.06,
        angular_weight: float = 0.30,
        joint_velocity_limits: np.ndarray | None = None,
    ) -> IkResult:
        """Legacy weighted damped least-squares step without joint bounds.

        Kept as a numerical reference for benchmarks and equivalence tests.
        Live teleoperation uses :meth:`inverse_velocity_ik_pose` by default and
        :meth:`inverse_velocity_bounded` as the strict fallback.
        """

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

    def inverse_velocity_bounded(
        self,
        joint_position_rad: np.ndarray,
        desired_twist: np.ndarray,
        *,
        joint_velocity_limits: np.ndarray,
        joint_position_min: np.ndarray | None = None,
        joint_position_max: np.ndarray | None = None,
        dt: float | None = None,
        rcond: float = 1.0e-8,
        residual_absolute_tolerance: float = 1.0e-8,
        residual_relative_tolerance: float = 1.0e-5,
    ) -> IkResult:
        """Solve a direction-preserving, speed-bounded differential IK step.

        The six-dimensional task is solved before limits are applied.  A single
        scale factor is then applied to every joint velocity, so joint limits
        slow the complete Cartesian command instead of changing its direction
        or introducing tool rotation.  If the requested twist is outside the
        Jacobian column space, the strict pose task is held rather than silently
        replacing it with a different Cartesian motion.
        """

        context = self._step_context(
            joint_position_rad,
            desired_twist,
            joint_velocity_limits,
            joint_position_min,
            joint_position_max,
            dt,
        )
        size = len(self.joint_names)
        if float(np.linalg.norm(context.twist)) <= 1.0e-14:
            return IkResult(
                joint_velocity_rad_s=np.zeros(size),
                achieved_twist=np.zeros(6),
                residual_twist=np.zeros(6),
                jacobian_rank=context.rank,
                velocity_scale=1.0,
                minimum_singular_value=context.minimum_singular_value,
                condition_number=context.condition_number,
            )

        unscaled, *_ = np.linalg.lstsq(context.jacobian, context.twist, rcond=float(rcond))
        projected = context.jacobian @ unscaled
        projection_error = float(np.linalg.norm(context.twist - projected))
        allowed_error = float(residual_absolute_tolerance) + float(residual_relative_tolerance) * float(
            np.linalg.norm(context.twist)
        )
        if projection_error > allowed_error:
            return IkResult(
                joint_velocity_rad_s=np.zeros(size),
                achieved_twist=np.zeros(6),
                residual_twist=context.twist.copy(),
                jacobian_rank=context.rank,
                limit_reasons=({"code": "task_constraint", "joints": []},),
                velocity_scale=0.0,
                minimum_singular_value=context.minimum_singular_value,
                condition_number=context.condition_number,
            )

        scale = _uniform_limit_scale(unscaled, context.lower, context.upper)
        velocity = unscaled * scale
        achieved = context.jacobian @ velocity
        return IkResult(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=context.twist - achieved,
            jacobian_rank=context.rank,
            velocity_scale=scale,
            minimum_singular_value=context.minimum_singular_value,
            condition_number=context.condition_number,
            limit_reasons=_uniform_limit_reasons(unscaled, context.lower, context.upper,
                                                np.asarray(joint_velocity_limits), scale),
        )

    def inverse_velocity_ik_pose(
        self,
        joint_position_rad: np.ndarray,
        desired_twist: np.ndarray,
        *,
        joint_velocity_limits: np.ndarray,
        joint_position_min: np.ndarray | None = None,
        joint_position_max: np.ndarray | None = None,
        dt: float | None = None,
        base_damping: float = 1.0e-3,
        maximum_damping: float = 5.0e-2,
        singular_value_threshold: float = 2.0e-2,
        nullspace_reference: np.ndarray | None = None,
        nullspace_gains: np.ndarray | None = None,
        nullspace_rcond: float = 1.0e-6,
    ) -> IkResult:
        """robosuite ``IK_POSE``-style damped differential IK with posture control.

        This mirrors ``robosuite/controllers/parts/arm/ik.py`` and
        ``robosuite/utils/ik_utils.py``::

            dq = J.T @ solve(J @ J.T + damping**2 * I, twist)
            dq += (I - pinv(J) @ J) @ (gains * (posture - q))
            dq = scale_to_joint_speed_and_position_limits(dq)

        Three properties are deliberately kept from the current platform:

        * the damping ramps from ``base_damping`` to ``maximum_damping`` as the
          smallest singular value falls below ``singular_value_threshold``, so
          a well-conditioned pose keeps near-exact task tracking while a
          singular direction stays bounded instead of freezing;
        * joint speed and position limits rescale the whole step by one factor,
          which preserves the Cartesian direction of the command;
        * ``nullspace_reference`` is optional and used only by explicit
          numerical experiments. Live elbow preference is applied separately,
          without a home reference; idle/gripper-only commands skip IK entirely.
        """

        base = float(base_damping)
        maximum = float(maximum_damping)
        threshold = float(singular_value_threshold)
        if not (math.isfinite(base) and math.isfinite(maximum)) or base < 0.0 or maximum < base:
            raise ValueError("damping must satisfy 0 <= base_damping <= maximum_damping")
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ValueError("singular value threshold must be positive and finite")

        reference = self._nullspace_reference_array(nullspace_reference)
        gains = self._nullspace_gain_array(nullspace_gains) if reference is not None else None
        context = self._step_context(
            joint_position_rad,
            desired_twist,
            joint_velocity_limits,
            joint_position_min,
            joint_position_max,
            dt,
            full_svd=reference is not None,
        )
        damping = _adaptive_damping(
            context.singular_values, base, maximum, threshold
        )
        step = context.jacobian.T @ np.linalg.solve(
            context.jacobian @ context.jacobian.T + damping**2 * np.eye(6),
            context.twist,
        )
        correction_norm = 0.0
        if reference is not None:
            assert gains is not None
            projector = context.nullspace_projector(rcond=float(nullspace_rcond))
            correction = projector @ (gains * (reference - context.joint_position))
            step = step + correction
            correction_norm = float(np.linalg.norm(correction))

        scale = _uniform_limit_scale(step, context.lower, context.upper)
        velocity = step * scale
        achieved = context.jacobian @ velocity
        return IkResult(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=context.twist - achieved,
            jacobian_rank=context.rank,
            velocity_scale=scale,
            minimum_singular_value=context.minimum_singular_value,
            condition_number=context.condition_number,
            damping=damping,
            nullspace_correction_norm=correction_norm * scale,
            limit_reasons=_uniform_limit_reasons(step, context.lower, context.upper,
                                                np.asarray(joint_velocity_limits), scale),
        )

    def _step_context(
        self,
        joint_position_rad: np.ndarray,
        desired_twist: np.ndarray,
        joint_velocity_limits: np.ndarray,
        joint_position_min: np.ndarray | None,
        joint_position_max: np.ndarray | None,
        dt: float | None,
        *,
        full_svd: bool = False,
    ) -> "_StepContext":
        """Validate one differential-IK step and return its geometry and bounds."""

        q = np.asarray(joint_position_rad, dtype=float)
        twist = np.asarray(desired_twist, dtype=float)
        limits = np.asarray(joint_velocity_limits, dtype=float)
        size = len(self.joint_names)
        if q.shape != (size,) or not np.all(np.isfinite(q)):
            raise ValueError(f"expected {size} finite joint positions, got {q.shape}")
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            raise ValueError("desired twist must contain six finite values")
        if limits.shape != (size,) or not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
            raise ValueError("joint velocity limits must be positive and match the chain")
        if dt is not None and (not math.isfinite(float(dt)) or float(dt) <= 0.0):
            raise ValueError("dt must be positive and finite")

        jacobian = self.jacobian(q)
        right_vectors: np.ndarray | None = None
        if full_svd:
            _, singular_values, right_vectors = np.linalg.svd(jacobian, full_matrices=False)
        else:
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
        rank = int(np.count_nonzero(singular_values > 1.0e-5))

        lower = -limits.copy()
        upper = limits.copy()
        if any(value is not None for value in (joint_position_min, joint_position_max, dt)):
            if joint_position_min is None or joint_position_max is None or dt is None:
                raise ValueError("joint position bounds and dt must be supplied together")
            minimum = np.asarray(joint_position_min, dtype=float)
            maximum = np.asarray(joint_position_max, dtype=float)
            if minimum.shape != (size,) or maximum.shape != (size,):
                raise ValueError("joint position bounds must match the chain")
            if np.any(np.isnan(minimum)) or np.any(np.isnan(maximum)) or np.any(minimum > maximum):
                raise ValueError("joint position bounds must be ordered and not NaN")
            lower = np.maximum(lower, (minimum - q) / float(dt))
            upper = np.minimum(upper, (maximum - q) / float(dt))

        return _StepContext(
            joint_position=q,
            twist=twist,
            jacobian=jacobian,
            singular_values=singular_values,
            rank=rank,
            lower=lower,
            upper=upper,
            right_vectors=right_vectors,
        )

    def _nullspace_reference_array(self, reference: np.ndarray | None) -> np.ndarray | None:
        if reference is None:
            return None
        posture = np.asarray(reference, dtype=float)
        size = len(self.joint_names)
        if posture.shape != (size,) or not np.all(np.isfinite(posture)):
            raise ValueError(f"nullspace reference must contain {size} finite joint positions")
        return posture

    def _nullspace_gain_array(self, gains: np.ndarray | None) -> np.ndarray:
        size = len(self.joint_names)
        if gains is None:
            return np.ones(size)
        posture_gains = np.asarray(gains, dtype=float)
        if posture_gains.shape != (size,) or not np.all(np.isfinite(posture_gains)):
            raise ValueError(f"nullspace gains must contain {size} finite values")
        if np.any(posture_gains < 0.0):
            raise ValueError("nullspace gains must not be negative")
        return posture_gains


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


def _uniform_limit_scale(
    joint_velocity: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> float:
    """Return one scale factor that fits every joint step inside its own bounds."""

    scale = 1.0
    for value, low, high in zip(joint_velocity, lower, upper, strict=True):
        if value > 1.0e-14:
            scale = min(scale, max(0.0, float(high / value)))
        elif value < -1.0e-14:
            scale = min(scale, max(0.0, float(low / value)))
    return max(0.0, min(1.0, scale))


def _uniform_limit_reasons(
    step: np.ndarray, lower: np.ndarray, upper: np.ndarray,
    velocity_limits: np.ndarray, scale: float,
) -> tuple[dict, ...]:
    """Explain the already-computed scalar limit, including a zero-scale stop.

    Observational only: no alternative solve, look-ahead, feedback or gating.
    Inspect the unscaled step so that a stationary joint at its boundary is not
    falsely named as the cause of another joint's stop.
    """
    if scale >= 1.0 - 1e-8:
        return ()
    positions, speeds = [], []
    for i, value in enumerate(step):
        if abs(value) <= 1e-14:
            continue
        bound = upper[i] if value > 0 else lower[i]
        allowed = max(0.0, float(bound / value))
        if not math.isclose(allowed, scale, rel_tol=1e-6, abs_tol=1e-8):
            continue
        if abs(bound) < velocity_limits[i] - 1e-10:
            positions.append(i + 1)
        else:
            speeds.append(i + 1)
    return tuple({"code": code, "joints": joints} for code, joints in
                 (("joint_position_limit", positions), ("joint_speed_limit", speeds)) if joints)


def _adaptive_damping(
    singular_values: np.ndarray,
    base_damping: float,
    maximum_damping: float,
    singular_value_threshold: float,
) -> float:
    """Ramp damping up as the smallest singular value approaches zero."""

    if not singular_values.size:
        return maximum_damping
    smallest = float(singular_values[-1])
    if not math.isfinite(smallest):
        return maximum_damping
    proximity = 1.0 - max(0.0, min(1.0, smallest / singular_value_threshold))
    return max(base_damping, maximum_damping * proximity)
