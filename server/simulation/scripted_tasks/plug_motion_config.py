"""Current plug configuration, motion limits and reproducible parameter snapshots.

Geometry and per-plug differences live in plug_profiles. This module has no
controller or drawer dependency; preflight, runtime and optimization use the
same resolver. Historical recording fields belong to collection contracts.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
import numpy as np

from .plug_profiles import PLUG_PROFILES, plug_profile

CONFIG_SCHEMA = "plug-motion-config/2"
CURRENT_TASK_VERSION = "plug-insert/19-axis-contact-continuity"


@dataclass(frozen=True)
class PlugTaskRules:
    """Current phase gates; these are versioned rules, not tuning switches."""
    initial_settle_s: float = 1.0
    grasp_confirm_s: float = 0.15
    seated_confirm_s: float = 0.15
    release_confirm_s: float = 0.1
    overload_confirm_s: float = 0.15
    final_alignment_tolerance_rad: float = 0.01


TASK_RULES = PlugTaskRules()


@dataclass(frozen=True)
class PlugMotionLimits:
    """Validated trajectory limits, independent of other task defaults."""
    max_arm_speed_rad_s: float = 0.35
    linear_speed_m_s: float = 0.1
    angular_speed_rad_s: float = 0.2
    linear_acceleration_m_s2: float = 0.2
    angular_acceleration_rad_s2: float = 0.4
    joint_acceleration_rad_s2: float = 0.7
    linear_jerk_m_s3: float = 0.8
    angular_jerk_rad_s3: float = 1.6
    joint_jerk_rad_s3: float = 8.0
    gripper_acceleration_rad_s2: float = 0.4
    gripper_jerk_rad_s3: float = 1.6
    slow_distance_m: float = 0.02


MOTION_LIMITS = PlugMotionLimits()


@dataclass(frozen=True)
class PlugInsertConfig:
    target_site: str | None = None
    plug_body: str = "cylindrical_plug"
    grasp_offset_m: tuple[float, float, float] | None = None
    transport_height_m: float = 0.245
    arm_speed_rad_s: float | None = None
    linear_speed_m_s: float | None = None
    retract_speed_m_s: float | None = None
    insert_speed_m_s: float | None = None
    gripper_speed_rad_s: float = 0.2
    release_speed_rad_s: float | None = None
    release_open_rad: float = 0.65
    force_limit_n: float = 40.0
    torque_limit_nm: float = 4.0
    overload_stop_enabled: bool = False
    shaft_tip_local_m: float | None = None
    flange_local_m: float = 0.025
    radial_tolerance_m: float = 0.0004
    seat_tolerance_m: float = 0.001
    completion_confirm_s: float = 0.2
    insert_slowdown_ratio: float = 0.75
    insert_resume_ratio: float = 0.90
    insert_orientation_tolerance_rad: float | None = None
    # A small, load-limited axial creep keeps mild contact misalignment from
    # producing long zero-velocity windows in the demonstration.  Large
    # errors and elevated contact loads still hold the insertion target.
    insert_creep_ratio: float = 1.25
    insert_creep_speed_scale: float = 0.18
    insert_creep_delay_s: float = 0.25
    insert_creep_force_fraction: float = 0.5
    insert_creep_torque_fraction: float = 0.5
    grasp_relative_speed_m_s: float = 0.015
    tracking_speed_rad_s: float | None = None
    pregrasp_distance_m: float = 0.045
    grasp_approach_speed_m_s: float | None = None
    extraction_speed_m_s: float = 0.025
    extraction_lateral_gain_s_inv: float | None = None
    extraction_velocity_gain: float | None = None
    carry_speed_m_s: float = 0.05
    carry_acceleration_m_s2: float = 0.15
    carry_jerk_m_s3: float = 0.8

    def __post_init__(self):
        profile = next(
            (p for p in PLUG_PROFILES.values() if p.body == self.plug_body), None
        )
        if profile is None:
            raise ValueError("Unknown plug body")
        if self.grasp_offset_m is None:
            object.__setattr__(self, "grasp_offset_m", profile.grasp_offset_m)
        if self.insert_orientation_tolerance_rad is None:
            object.__setattr__(self, "insert_orientation_tolerance_rad",
                               profile.insert_orientation_tolerance_rad)
        if self.extraction_lateral_gain_s_inv is None:
            object.__setattr__(self, "extraction_lateral_gain_s_inv",
                               profile.extraction_lateral_gain_s_inv)
        if self.extraction_velocity_gain is None:
            object.__setattr__(self, "extraction_velocity_gain",
                               profile.extraction_velocity_gain)
        for name, default in (
            ("insert_speed_m_s", 0.025),
            ("release_speed_rad_s", 0.2),
            ("arm_speed_rad_s", 0.35),
            ("linear_speed_m_s", 0.1),
            ("retract_speed_m_s", 0.1),
            ("tracking_speed_rad_s", 0.35),
            ("grasp_approach_speed_m_s", 0.025),
        ):
            if getattr(self, name) is None:
                object.__setattr__(self, name, default)
        if self.target_site is None:
            object.__setattr__(self, "target_site", profile.target_site)
        if self.shaft_tip_local_m is None:
            object.__setattr__(self, "shaft_tip_local_m", profile.tip_m)
        if self.target_site not in (
            profile.prefix + "_insert_40p1",
            profile.target_site,
        ):
            raise ValueError(
                "Target must be this plug's forward assembly hole, not the pickup location"
            )
        if not isinstance(self.overload_stop_enabled, bool):
            raise ValueError("overload_stop_enabled must be boolean")
        if not 0 < self.insert_slowdown_ratio < self.insert_resume_ratio < 1:
            raise ValueError("Require 0 < insert_slowdown_ratio < insert_resume_ratio < 1")
        if not 0 < self.insert_orientation_tolerance_rad <= .008:
            raise ValueError("insert_orientation_tolerance_rad must be in (0, 0.008]")
        if not 0 < self.extraction_lateral_gain_s_inv <= 6.:
            raise ValueError("extraction_lateral_gain_s_inv must be in (0, 6]")
        if not 0 < self.extraction_velocity_gain <= 8.:
            raise ValueError("extraction_velocity_gain must be in (0, 8]")
        if self.insert_creep_ratio <= 1.0:
            raise ValueError("insert_creep_ratio must be greater than 1")
        if not 0 < self.insert_creep_speed_scale <= 1:
            raise ValueError("insert_creep_speed_scale must be in (0, 1]")
        if not 0 < self.insert_creep_force_fraction <= 1:
            raise ValueError("insert_creep_force_fraction must be in (0, 1]")
        if not 0 < self.insert_creep_torque_fraction <= 1:
            raise ValueError("insert_creep_torque_fraction must be in (0, 1]")
        for name, value in vars(self).items():
            if name == "overload_stop_enabled":
                continue
            if isinstance(value, (float, int)) and (
                isinstance(value, bool) or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (self.arm_speed_rad_s > MOTION_LIMITS.max_arm_speed_rad_s
                or self.linear_speed_m_s > MOTION_LIMITS.linear_speed_m_s
                or self.insert_speed_m_s > 0.025):
            raise ValueError("Configured motion exceeds the validated planning limits")
        if (
            np.asarray(self.grasp_offset_m).shape != (3,)
            or not np.isfinite(self.grasp_offset_m).all()
        ):
            raise ValueError("Invalid grasp offset")
        object.__setattr__(self, "grasp_offset_m", tuple(self.grasp_offset_m))


MOTION_PARAMETERS = frozenset({
    "insert_speed_m_s", "extraction_speed_m_s",
    "extraction_lateral_gain_s_inv",
    "extraction_velocity_gain",
    "insert_orientation_tolerance_rad", "insert_slowdown_ratio", "insert_resume_ratio",
    "insert_creep_ratio", "insert_creep_speed_scale", "insert_creep_delay_s",
    "insert_creep_force_fraction", "insert_creep_torque_fraction",
})

# Bounds constrain automated experiments to the current controller's motion
# envelope. Raising an upper bound requires separate physical validation;
# a saved scene cannot silently request an arbitrary speed or load fraction.
MOTION_PARAMETER_MAXIMUMS = {
    "insert_speed_m_s": .025,
    "extraction_speed_m_s": .025,
    "extraction_lateral_gain_s_inv": 6.,
    "extraction_velocity_gain": 8.,
    "insert_orientation_tolerance_rad": .008,
    "insert_slowdown_ratio": 1.,
    "insert_resume_ratio": 1.,
    "insert_creep_ratio": 1.75,
    "insert_creep_speed_scale": .5,
    "insert_creep_delay_s": 1.,
    "insert_creep_force_fraction": .5,
    "insert_creep_torque_fraction": .5,
}


def _snapshot_hash(document):
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"),
                         allow_nan=False).encode()
    return hashlib.sha256(payload).hexdigest()


def config_snapshot(config, task_name):
    """Freeze effective values and rules; source/model identities remain separate."""
    profile = plug_profile(task_name)
    if config.plug_body != profile.body:
        raise ValueError("Plug configuration does not match the task")
    document = dict(schema=CONFIG_SCHEMA, task=profile.task, task_version=CURRENT_TASK_VERSION,
                    values=asdict(config), rules=asdict(TASK_RULES),
                    limits=asdict(MOTION_LIMITS), profile=asdict(profile))
    # JSON normalization also freezes tuple-valued geometry as saved arrays.
    document = json.loads(json.dumps(document, allow_nan=False))
    return {**document, "sha256": _snapshot_hash(document)}


def _config_from_snapshot(snapshot, task_name):
    if not isinstance(snapshot, dict) or snapshot.get("schema") != CONFIG_SCHEMA:
        raise ValueError("Unsupported plug configuration snapshot schema")
    document = {key: value for key, value in snapshot.items() if key != "sha256"}
    if snapshot.get("sha256") != _snapshot_hash(document):
        raise ValueError("Plug configuration snapshot hash mismatch")
    values = snapshot.get("values")
    if not isinstance(values, dict) or set(values) != {field.name for field in fields(PlugInsertConfig)}:
        raise ValueError("Plug configuration snapshot must contain all current fields")
    config = PlugInsertConfig(**values)
    if config_snapshot(config, task_name) != snapshot:
        raise ValueError("Plug snapshot task, profile or motion limits do not match this runtime")
    return config


def plug_motion_config(task_name, scene=None, **explicit):
    """Resolve one current config, or read an exact supported frozen snapshot."""
    collection = (scene or {}).get("collection_config", {})
    overrides = collection.get("plug_motion_parameters", {})
    if not isinstance(overrides, dict):
        raise ValueError("plug_motion_parameters must be an object")
    unknown = set(overrides) - MOTION_PARAMETERS
    if unknown:
        raise ValueError(f"Unsupported plug motion parameters: {sorted(unknown)}")
    options = {**overrides, **{key: value for key, value in explicit.items() if value is not None}}
    for name, maximum in MOTION_PARAMETER_MAXIMUMS.items():
        if name not in options:
            continue
        value = options[name]
        if (isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value) or not 0 < value <= maximum):
            raise ValueError(f"{name} must be finite and in (0, {maximum}]")
    snapshot = collection.get("plug_config_snapshot")
    if snapshot is not None:
        config = _config_from_snapshot(snapshot, task_name)
        for name, value in options.items():
            if not hasattr(config, name) or getattr(config, name) != value:
                raise ValueError(f"Explicit override conflicts with frozen plug configuration: {name}")
        return config
    return PlugInsertConfig(plug_body=plug_profile(task_name).body, **options)
