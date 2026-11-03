"""Immutable stage and acceptance rules for already recorded task versions.

These are data contracts, not executable historical controllers. Never infer a
missing version from today's controller, nor add today's defaults to an old
record and claim to reproduce its original configuration.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
from collections.abc import Sequence

from space_arm_platform.task_catalog import normalize_task_name


@dataclass(frozen=True)
class RecordedTaskContract:
    stages: tuple[str, ...]
    repeated_rotation_stages: bool = False
    latch_grasp_required: bool = False
    released_velocity_required: bool = True
    geometric_completion: bool = False
    released_closed_confirmation: bool = False
    validation_supported: bool = True
    minimum_release_opening_rad: float = .40


# Keep historical snapshots here even when the current controller changes.
_PLUG_V2 = ("settle", "raise", "above_pickup", "align", "open", "descend", "grip",
            "lift", "transfer", "above_hole", "align_hole", "insert", "seat", "release", "retract", "verify")
_PLUG_V3 = ("settle", "raise", "above_pickup", "approach", "descend", "grip", "extract",
            "lift", "transfer", "above_hole", "align_hole", "insert", "seat", "release", "retract", "verify")
_PLUG_V4 = ("settle", "approach", "descend", "grip", "extract", "lift", "transfer",
            "above_hole", "align_hole", "insert", "seat", "release", "retract", "verify")
_PLUG_V5 = tuple(stage for stage in _PLUG_V4 if stage != "descend")
_PLUG_V8 = tuple(stage for stage in _PLUG_V5 if stage not in ("align_hole", "seat"))
_PLUG_VERSIONS = {
    None: RecordedTaskContract(_PLUG_V2),
    "plug-insert/2": RecordedTaskContract(_PLUG_V2),
    "plug-insert/3-latch-grasp": RecordedTaskContract(_PLUG_V3, latch_grasp_required=True),
    "plug-insert/4-coordinated-approach": RecordedTaskContract(_PLUG_V4, latch_grasp_required=True),
}
for _version in ("5-smooth-tracking", "6-gentle-carry", "7-continuous-carry"):
    _PLUG_VERSIONS["plug-insert/" + _version] = RecordedTaskContract(_PLUG_V5, latch_grasp_required=True)
for _version in (
    "8-continuous-contact", "9-overlapped-release", "10-shared-continuous-motion",
    "11-all-latch-aligned", "12-round-plug-release", "12-cylindrical-clearance",
    "13-cylindrical-clearance-gentle-release", "13-straight-approach",
    "14-straight-approach-gentle-release", "15-seated-completion",
    "16-restored-cylinder-release", "17-smooth-insertion-gate",
    "18-continuous-alignment-creep", "19-axis-contact-continuity",
):
    _PLUG_VERSIONS["plug-insert/" + _version] = RecordedTaskContract(
        _PLUG_V8,
        # The old validator did not require latch evidence for the two straight
        # approach labels; preserve that historical distinction.
        latch_grasp_required=_version not in ("13-straight-approach", "14-straight-approach-gentle-release"),
        released_velocity_required=_version not in (
            "15-seated-completion", "16-restored-cylinder-release", "17-smooth-insertion-gate",
            "18-continuous-alignment-creep", "19-axis-contact-continuity"),
    )

_DRAWER_LEGACY = ("settle", "raise", "above_drawer", "align", "open", "descend", "grip",
                  "lift", "hold", "return", "seat", "release", "retract", "verify")
_DRAWER_V4 = tuple(stage for stage in _DRAWER_LEGACY if stage != "align")
_DRAWER_V5 = ("settle", "approach", "grip", "lift", "return", "release", "retract", "verify")
_DRAWER_VERSIONS = {version: RecordedTaskContract(_DRAWER_LEGACY)
                    for version in (None, "drawer-cycle/1", "drawer-cycle/2", "drawer-cycle/3")}
_DRAWER_VERSIONS["drawer-cycle/4"] = RecordedTaskContract(_DRAWER_V4, released_closed_confirmation=True)
for _version in ("drawer-cycle/5", "drawer-cycle/6-straight-approach", "drawer-cycle/7-closed-completion"):
    _DRAWER_VERSIONS[_version] = RecordedTaskContract(
        _DRAWER_V5, released_closed_confirmation=_version == "drawer-cycle/7-closed-completion")

_BOLT_STAGES = ("settle", "approach", "grip", "unscrew", "regrasp_release", "regrasp_rewind",
                "regrasp_grip", "carry", "screw_in", "release", "retract", "verify")
_BOLT_VERSIONS = {
    version: RecordedTaskContract(_BOLT_STAGES, repeated_rotation_stages=True,
                                  validation_supported=version in (
                                      "aviation-bolt/4-straight-approach", "aviation-bolt/5-seated-completion"),
                                  geometric_completion=version == "aviation-bolt/5-seated-completion")
    for version in ("aviation-bolt/2-latch-smooth-regrasp", "aviation-bolt/3-environment-planning",
                    "aviation-bolt/4-straight-approach", "aviation-bolt/5-seated-completion")
}
_ROD_STAGES = ("settle", "approach", "grip", "loosen", "fold_release", "fold_clear",
               "fold_approach", "fold_grip", "lower", "release_down", "clear_down",
               "reapproach", "regrip", "raise", "turn_release", "turn_clear", "turn_approach",
               "turn_grip", "tighten", "release", "retract", "verify")
_ROD_VERSIONS = {
    version: RecordedTaskContract(_ROD_STAGES, validation_supported=version in (
        "locking-rod-cycle/3-straight-approach", "locking-rod-cycle/4-locked-completion"),
        geometric_completion=version == "locking-rod-cycle/4-locked-completion")
    for version in (None, "locking-rod-cycle/2-coaxial-latch", "locking-rod-cycle/3-straight-approach",
                    "locking-rod-cycle/4-locked-completion")
}
_CONTRACTS = {
    "drawer_cycle": _DRAWER_VERSIONS,
    "drawer_open": {"drawer-open/1": RecordedTaskContract(
        ("settle", "approach", "grip", "lift", "release", "retract", "verify"))},
    "drawer_close": {"drawer-close/1": RecordedTaskContract(
        ("settle", "approach", "grip", "return", "release", "retract", "verify"))},
    "aviation_bolt_insert": _BOLT_VERSIONS,
    "locking_rod_cycle": _ROD_VERSIONS,
}
for _name in ("aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"):
    _CONTRACTS[_name] = _PLUG_VERSIONS
for _name, _version, _stages in (
    ("locking_rod_fold", "locking-rod-fold/1-straight-approach", _ROD_STAGES[:11] + ("verify",)),
    ("locking_rod_restore", "locking-rod-restore/1-straight-approach", ("settle",) + _ROD_STAGES[11:]),
):
    _contract = RecordedTaskContract(_stages)
    _CONTRACTS[_name] = {_version: _contract, None: replace(_contract, validation_supported=False)}


def recorded_contract(task_name: str, version: str | None) -> RecordedTaskContract:
    """Select an explicitly supported saved contract; unknown versions fail."""
    task_name = normalize_task_name(task_name)
    try:
        return _CONTRACTS[task_name][version]
    except KeyError:
        raise ValueError(f"Unsupported recorded task/version: {task_name} {version!r}") from None


def recorded_stages(task_name: str, version: str | None) -> tuple[str, ...]:
    return recorded_contract(task_name, version).stages


def validate_recorded_stages(task_name: str, stages: Sequence[str], version: str | None) -> bool:
    """Check sampled stage order, including the bolt's repeated regrasp loop."""
    contract = recorded_contract(task_name, version)
    compact = tuple(name for i, name in enumerate(stages) if i == 0 or name != stages[i - 1])
    if not contract.repeated_rotation_stages:
        return compact == contract.stages
    edges = {
        "settle": {"approach"}, "approach": {"grip"}, "grip": {"unscrew"},
        "unscrew": {"regrasp_release", "carry"}, "regrasp_release": {"regrasp_rewind"},
        "regrasp_rewind": {"regrasp_grip"}, "regrasp_grip": {"unscrew", "screw_in"},
        "carry": {"screw_in"}, "screw_in": {"regrasp_release", "release"},
        "release": {"retract"}, "retract": {"verify"}, "verify": set(),
    }
    return bool(compact and compact[0] == "settle" and compact[-1] == "verify"
                and all(b in edges.get(a, set()) for a, b in zip(compact, compact[1:]))
                and all(stage in compact for stage in ("unscrew", "carry", "screw_in", "release")))


# Frozen schema fields, independent of the current executable config class.
_PLUG_CONFIG_FIELDS_V2 = frozenset({
    'target_site', 'plug_body', 'grasp_offset_m',
    'transport_height_m', 'arm_speed_rad_s', 'linear_speed_m_s',
    'retract_speed_m_s', 'insert_speed_m_s', 'gripper_speed_rad_s',
    'release_speed_rad_s', 'release_open_rad', 'force_limit_n',
    'torque_limit_nm', 'overload_stop_enabled', 'shaft_tip_local_m',
    'flange_local_m', 'radial_tolerance_m', 'seat_tolerance_m',
    'completion_confirm_s', 'insert_slowdown_ratio', 'insert_resume_ratio',
    'insert_orientation_tolerance_rad', 'insert_creep_ratio', 'insert_creep_speed_scale',
    'insert_creep_delay_s', 'insert_creep_force_fraction', 'insert_creep_torque_fraction',
    'grasp_relative_speed_m_s', 'tracking_speed_rad_s', 'pregrasp_distance_m',
    'grasp_approach_speed_m_s', 'extraction_speed_m_s', 'extraction_lateral_gain_s_inv',
    'extraction_velocity_gain', 'carry_speed_m_s', 'carry_acceleration_m_s2',
    'carry_jerk_m_s3',
})


def validate_recorded_configuration(scene, task_name, task_rows):
    """Bind declared new config schemas to saved values without inventing defaults.

    Older rows without a complete snapshot remain readable as their original
    flat parameters, including legacy ``hold_s``. They carry no claim of exact
    configuration reconstruction.
    """
    snapshot = (scene.get("collection_config") or {}).get("plug_config_snapshot")
    declared = any(row.get("task_parameters_schema") is not None for row in task_rows)
    if snapshot is None:
        if declared:
            raise ValueError("Recorded parameter schema requires its complete configuration snapshot")
        return
    if not isinstance(snapshot, dict) or snapshot.get("schema") != "plug-motion-config/2":
        raise ValueError("Unsupported recorded configuration snapshot schema")
    if set(snapshot) != {"schema", "task", "task_version", "values", "rules", "limits", "profile", "sha256"}:
        raise ValueError("Incomplete recorded configuration snapshot")
    if not isinstance(snapshot["values"], dict) or set(snapshot["values"]) != _PLUG_CONFIG_FIELDS_V2:
        raise ValueError("Recorded configuration snapshot must contain every schema field")
    document = {key: value for key, value in snapshot.items() if key != "sha256"}
    digest = hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"),
                                       allow_nan=False).encode()).hexdigest()
    if digest != snapshot["sha256"]:
        raise ValueError("Recorded configuration snapshot hash mismatch")
    if snapshot["task"] != normalize_task_name(task_name):
        raise ValueError("Recorded configuration belongs to a different task")
    recorded_contract(task_name, snapshot["task_version"])
    for row in task_rows:
        if (row.get("task_parameters_schema") != snapshot["schema"]
                or row.get("task_parameters_sha256") != digest
                or row.get("task_parameters") != snapshot["values"]
                or row.get("task_version") != snapshot["task_version"]):
            raise ValueError("Recorded task parameters differ from the frozen configuration")
