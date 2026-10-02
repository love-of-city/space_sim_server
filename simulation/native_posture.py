"""Six-axis C++ posture correction, with the Python policy retained as reference.

Only kinematics/reference arithmetic is native. This module neither imports
MuJoCo nor accesses Basilisk, physical state, actuator messages, or threads.
"""
from __future__ import annotations

import ctypes
from dataclasses import replace
from functools import lru_cache
import math
import weakref

import numpy as np

from simulation.native_acceleration import load_native_library

PARAMETERS = (
    "preferred_height_m", "height_gain_s", "maximum_height_speed_m_s", "posture_weight",
    "preferred_wrist_drop_m", "wrist_height_gain_s", "maximum_wrist_drop_speed_m_s", "wrist_weight",
    "joint3_negative_margin_rad", "joint3_angle_scale_rad", "joint3_length_scale_m", "joint3_gain_s",
    "maximum_joint3_preference_speed_rad_s", "joint3_weight", "regularization",
    "maximum_correction_rad_s", "maximum_linear_disturbance_m_s", "maximum_angular_disturbance_rad_s",
    "relative_disturbance", "characteristic_length_m", "nominal_command_speed_m_s",
    "maximum_position_offset_m", "maximum_orientation_offset_rad",
)
DIAGNOSTICS = (
    "height_m", "predicted_height_m", "wrist_drop_m", "predicted_wrist_drop_m",
    "joint3_rad", "predicted_joint3_rad", "elbow_objective_active", "wrist_objective_active",
    "joint3_objective_active", "base_posture_cost", "predicted_posture_cost", "correction_norm_rad_s",
    "linear_disturbance_m_s", "angular_disturbance_rad_s", "linear_budget_m_s",
    "angular_budget_rad_s", "scale", "position_offset_m", "orientation_offset_rad",
)
STATUSES = (
    "idle", "shape_satisfied", "height_satisfied", "task_already_improves_shape",
    "task_already_raises_elbow", "no_correction", "limited", "active",
)
_CHAINS = weakref.WeakKeyDictionary()


@lru_cache(maxsize=1)
def _function():
    dll = load_native_library("posture")
    dll.posture_abi.restype = ctypes.c_int
    if dll.posture_abi() != 1:
        raise RuntimeError("Native posture ABI mismatch")
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    integers = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
    fn = dll.posture_step
    fn.argtypes = [ctypes.c_int, array, array, integers, array, ctypes.c_double,
                   ctypes.c_int, array, array, array]
    fn.restype = ctypes.c_int
    return fn


def available() -> bool:
    """True when a current, ABI-compatible native helper is installed.

    A stale or mismatched build is "not available" rather than an error: callers
    only ask so they can skip optional real-helper tests, and the load path still
    raises loudly for real callers.
    """
    try:
        _function()
    except (FileNotFoundError, RuntimeError):
        return False
    return True


@lru_cache(maxsize=1)
def _geometry_function():
    _function()  # Same library/ABI validation as the correction kernel.
    fn = load_native_library("posture").posture_geometry
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    fn.argtypes = [ctypes.c_int, array, array, array]
    fn.restype = ctypes.c_int
    return fn


def native_geometry(chain, q):
    """Same FK/origins/axes/Jacobian tuple consumed by the existing exact cache."""
    if len(chain.joint_names) != 6:
        raise ValueError("Native geometry supports six-axis chains only.")
    packed = _packed_chain(chain)
    q = np.ascontiguousarray(q, dtype=np.float64)
    if q.shape != (6,):
        raise ValueError("expected six joint positions")
    output = np.empty(88)
    if _geometry_function()(len(packed), packed, q, output):
        raise ValueError("native kinematic geometry failed")
    return (output[:16].reshape(4,4), output[16:34].reshape(6,3),
            output[34:52].reshape(6,3), output[52:].reshape(6,6))


def _packed_chain(chain):
    # Match the immutable-for-chain-lifetime geometry contract used by the
    # existing _fixed_transforms / _geometry_cache. Do not cache joint states.
    cached = _CHAINS.get(chain)
    if cached is None:
        cached = np.zeros((len(chain.segments), 23), dtype=np.float64)
        for row, segment, fixed in zip(cached, chain.segments, chain._fixed_transforms, strict=True):
            row[:16] = fixed.ravel()
            row[16] = -1 if segment.joint_index is None else segment.joint_index
            if segment.joint_index is not None:
                row[17:20] = segment.joint_position
                row[20:23] = segment.joint_axis
        _CHAINS[chain] = cached
    return cached


@lru_cache(maxsize=64)
def _configuration(preference, joint_names):
    p = preference
    indices = [joint_names.index(p.shoulder_joint), joint_names.index(p.elbow_joint)]
    indices += [joint_names.index(p.wrist_upper_joint), joint_names.index(p.wrist_lower_joint)] if p.wrist_enabled else [-1, -1]
    indices += [joint_names.index("joint3") if p.joint3_enabled else -1]
    params = np.array([*(getattr(p, name) for name in PARAMETERS), *p.up_axis], dtype=np.float64)
    return params, np.array(indices, dtype=np.int32)


def apply_native(chain, q, desired_twist, base, *, joint_velocity_limits,
                 joint_position_min, joint_position_max, dt, preference, deviation_state=None):
    from simulation.online_elbow_ik import ElbowStepDiagnostics
    if not preference.enabled:
        return base, ElbowStepDiagnostics()
    if len(chain.joint_names) != 6:
        raise ValueError("The native posture backend supports six-axis chains only; select python for other chains.")
    arrays = [np.asarray(x, dtype=float) for x in
              (q, desired_twist, base.joint_velocity_rad_s, joint_velocity_limits,
               joint_position_min, joint_position_max)]
    q, twist, velocity, speed, low, high = arrays
    if (any(a.shape != (6,) for a in (q, twist, speed, low, high)) or
            not math.isfinite(dt) or dt <= 0):
        raise ValueError("invalid online elbow step inputs")
    # Python reference ignores the base for a zero command, but still validates q/limits.
    idle = not np.any(twist != 0)
    if not idle and velocity.shape != (6,):
        raise ValueError("base velocity must be finite and match the chain")
    if idle:
        arrays[2] = np.zeros(6)
    data = np.ascontiguousarray(np.concatenate(arrays), dtype=np.float64)
    packed = _packed_chain(chain)
    # The public Python dataclass also accepts list/array up axes. Canonicalize
    # only those uncommon forms; never key the cache by mutable array identity.
    config = preference
    if not isinstance(preference.up_axis, tuple):
        config = replace(preference, up_axis=tuple(float(x) for x in preference.up_axis))
    params, indices = _configuration(config, tuple(chain.joint_names))
    state = np.zeros(12, dtype=np.float64)
    initialized = deviation_state is not None and deviation_state.position is not None
    if initialized:
        pos = np.asarray(deviation_state.position, dtype=float)
        rot = np.asarray(deviation_state.rotation, dtype=float)
        if pos.shape != (3,) or rot.shape != (3, 3) or not np.all(np.isfinite(pos)) or not np.all(np.isfinite(rot)):
            raise ValueError("invalid posture deviation state")
        state[:3], state[3:] = pos, rot.ravel()
    output, diagnostic = np.empty(18), np.empty(len(DIAGNOSTICS))
    status = _function()(len(packed), packed, params, indices, data, dt,
                         int(initialized), state, output, diagnostic)
    if status < 0:
        raise ValueError({-1: "invalid online elbow step inputs",
                          -2: "base IK step must be finite and already satisfy joint bounds",
                          -3: "native posture arithmetic failed"}.get(status, "native posture failed"))
    if status >= len(STATUSES):
        raise RuntimeError("invalid native posture status")
    if not idle and deviation_state is not None:
        deviation_state.position, deviation_state.rotation = state[:3].copy(), state[3:].reshape(3, 3).copy()
    values = dict(zip(DIAGNOSTICS, diagnostic.tolist(), strict=True))
    for name in ("elbow_objective_active", "wrist_objective_active", "joint3_objective_active"):
        values[name] = bool(values[name])
    diag = ElbowStepDiagnostics(enabled=True, status=STATUSES[status],
                               wrist_enabled=preference.wrist_enabled,
                               joint3_enabled=preference.joint3_enabled, **values)
    if status != 7:
        return base, diag
    return replace(base, joint_velocity_rad_s=output[:6].copy(),
                   achieved_twist=output[6:12].copy(), residual_twist=output[12:].copy()), diag
