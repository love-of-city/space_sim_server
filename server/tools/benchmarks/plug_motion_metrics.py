"""Measure actual plug contact motion separately from seating and jaw release.

Consumes diagnostic trace.jsonl/summary.json without changing either input.
Axial speed is spacecraft z, matching the production contact controller.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _time(row):
    return float(row["sim_time_ns"]) / 1e9 if "sim_time_ns" in row else float(row["sim_time_s"])


def _wrench_magnitudes(wrench):
    """Read production sensor dictionaries or legacy six-value trace arrays."""
    if isinstance(wrench, dict):
        if wrench.get("valid") is not True:
            return None
        force, torque = wrench.get("force_n"), wrench.get("torque_nm")
        if not all(isinstance(v, list) and len(v) == 3 for v in (force, torque)):
            return None
        raw = force + torque
    elif isinstance(wrench, list) and len(wrench) == 6:
        raw = wrench
    else:
        return None
    values = [_number(value) for value in raw]
    if any(value is None for value in values):
        return None
    magnitudes = math.hypot(*values[:3]), math.hypot(*values[3:])
    return magnitudes if all(math.isfinite(value) for value in magnitudes) else None


def _stage_metrics():
    return dict(
        contact_seconds=0.0, stagnant_seconds=0.0, max_stagnant_seconds=0.0,
        zero_reference_samples=0, zero_reference_seconds=0.0,
        zero_target_samples=0, zero_target_seconds=0.0,
        stalled_with_reference_seconds=0.0, measured_motion_with_zero_reference_seconds=0.0,
        seated_confirmation_seconds=0.0, contact_samples=0, reference_samples=0,
        target_samples=0, wrench_samples=0, invalid_wrench_samples=0,
        max_wrist_force_n=0.0, max_wrist_torque_nm=0.0,
    )


def measure_trace(rows, summary, *, speed_threshold_m_s=0.001):
    """Integrate sample intervals; never use a zero command as proof of a stop.

    Older traces have no extraction remaining-distance field: conservatively
    count the entire extract stage. Their insert stage uses measured seat gap.
    A trace stage can differ from its metrics by one control sample because the
    runner samples metrics immediately before advancing the task state.
    """
    config = summary.get("config", {})
    seat_tolerance = float(config.get("seat_tolerance_m", 0.001))
    stages = {}
    elapsed = {}
    invalid_samples = 0
    samples = 0
    fallback_remaining_samples = 0
    peaks = [0.0, 0.0]
    previous = None
    consecutive = 0.0
    previous_stage = None
    for row in rows:
        samples += 1
        stamp = _time(row)
        if not math.isfinite(stamp) or (previous is not None and stamp <= _time(previous)):
            raise ValueError("Trace timestamps must be finite and strictly increasing")
        wrench = _wrench_magnitudes(row.get("wrist_wrench"))
        if wrench is not None:
            for i in range(2):
                peaks[i] = max(peaks[i], wrench[i])
        if row.get("stage") in ("extract", "insert"):
            # Include all samples of a contact stage, including its final
            # observation and seated confirmation, in the peak-load gate.
            stage_metrics = stages.setdefault(row["stage"], _stage_metrics())
            stage_metrics["wrench_samples"] += 1
            if wrench is None:
                stage_metrics["invalid_wrench_samples"] += 1
            else:
                for key, value in zip(("max_wrist_force_n", "max_wrist_torque_nm"), wrench):
                    stage_metrics[key] = max(stage_metrics[key], value)
        if previous is not None:
            dt = stamp - _time(previous)
            stage = previous.get("stage", "unknown")
            elapsed[stage] = elapsed.get(stage, 0.0) + dt
            if stage not in ("extract", "insert"):
                consecutive = 0.0
                previous_stage = stage
                previous = row
                continue
            values = stages[stage]
            remaining = _number(previous.get("contact_remaining_m"))
            if remaining is None:
                fallback_remaining_samples += 1
                gap = _number(previous.get("seat_gap_m"))
                remaining = max(0.0, gap-seat_tolerance) if stage == "insert" and gap is not None else 1.0
            if remaining <= 0.0:
                values["seated_confirmation_seconds"] += dt
                consecutive = 0.0
            else:
                values["contact_samples"] += 1
                values["contact_seconds"] += dt
                velocity = previous.get("plug_velocity_body_m_s", [])
                axial = _number(velocity[2]) if isinstance(velocity, list) and len(velocity) >= 3 else None
                if axial is None:
                    invalid_samples += 1
                    consecutive = 0.0
                else:
                    stagnant = abs(axial) < speed_threshold_m_s
                    consecutive = (consecutive if previous_stage == stage else 0.0) + dt if stagnant else 0.0
                    values["stagnant_seconds"] += dt if stagnant else 0.0
                    values["max_stagnant_seconds"] = max(values["max_stagnant_seconds"], consecutive)
                    reference = _number(previous.get("axial_reference_speed_m_s"))
                    if reference is not None:
                        values["reference_samples"] += 1
                        zero = abs(reference) < 1e-8
                        values["zero_reference_samples"] += int(zero)
                        values["zero_reference_seconds"] += dt if zero else 0.0
                        values["stalled_with_reference_seconds"] += dt if stagnant and not zero else 0.0
                        values["measured_motion_with_zero_reference_seconds"] += dt if not stagnant and zero else 0.0
                    target = _number(previous.get("axial_target_speed_m_s"))
                    if target is None and stage == "insert":
                        target = _number(previous.get("insertion_speed_scale"))
                    if target is not None:
                        values["target_samples"] += 1
                        zero = abs(target) < 1e-8
                        values["zero_target_samples"] += int(zero)
                        values["zero_target_seconds"] += dt if zero else 0.0
            previous_stage = stage
        previous = row
    for index, name in enumerate(("max_wrist_force_n", "max_wrist_torque_nm")):
        value = _number(summary.get(name))
        if value is not None:
            peaks[index] = max(peaks[index], value)
    loads_complete = all(stages.get(stage, {}).get("wrench_samples", 0) > 0
                         and stages[stage]["invalid_wrench_samples"] == 0
                         for stage in ("extract", "insert"))
    return dict(
        schema="plug-motion-metrics/2", result=summary.get("result"), reason=summary.get("reason"),
        trace_samples=samples, invalid_contact_samples=invalid_samples,
        fallback_remaining_samples=fallback_remaining_samples,
        speed_threshold_m_s=speed_threshold_m_s, stages=stages, stage_seconds=elapsed,
        max_stagnant_seconds=max((v["max_stagnant_seconds"] for v in stages.values()), default=None),
        max_wrist_force_n=peaks[0], max_wrist_torque_nm=peaks[1],
        load_measurement_complete=loads_complete,
        measurement_complete=(samples > 1 and invalid_samples == 0 and loads_complete and all(
            stages.get(stage, {}).get("contact_samples", 0) > 0 for stage in ("extract", "insert"))),
        release_seconds=elapsed.get("release", 0.0), verify_seconds=elapsed.get("verify", 0.0),
        task_seconds=summary.get("task_seconds"),
        note="Physics trace only; no camera recording validation. Stage boundaries have one control-sample uncertainty.",
    )


def analyse_run(directory: Path, *, speed_threshold_m_s=0.001):
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    summary = summary.get("task", summary)
    with (directory / "trace.jsonl").open(encoding="utf-8") as stream:
        return measure_trace((json.loads(line) for line in stream if line.strip()), summary,
                             speed_threshold_m_s=speed_threshold_m_s)


def rejection_reasons(metrics, baseline=None, *, max_stagnant_seconds=0.5, load_multiplier=1.1):
    """Require measured completion before making any tuning recommendation."""
    reasons = []
    if metrics.get("result") != "success":
        reasons.append("task_failed")
    if not metrics.get("measurement_complete"):
        reasons.append("incomplete_contact_measurement")
    if not metrics.get("load_measurement_complete"):
        reasons.append("incomplete_contact_load_measurement")
    stagnant = metrics.get("max_stagnant_seconds")
    if stagnant is None or stagnant > max_stagnant_seconds + 1e-9:
        reasons.append("stagnation_budget_exceeded")
    if baseline is not None:
        if baseline.get("result") != "success":
            reasons.append("baseline_failed")
        if not baseline.get("measurement_complete"):
            reasons.append("baseline_measurement_incomplete")
        for key in ("max_wrist_force_n", "max_wrist_torque_nm"):
            limit = _number(baseline.get(key))
            if limit is None or limit < 0:
                reasons.append("baseline_" + key + "_unavailable")
            elif metrics.get(key, math.inf) > limit * load_multiplier + 1e-9:
                reasons.append(key + "_regression")
        for stage in ("extract", "insert"):
            previous = baseline.get("stages", {}).get(stage, {})
            current = metrics.get("stages", {}).get(stage, {})
            for label, measurements in (("baseline", previous), ("candidate", current)):
                if measurements.get("wrench_samples", 0) <= 0 or measurements.get("invalid_wrench_samples", 1) != 0:
                    reasons.append(f"{label}_{stage}_load_unavailable")
            for key in ("max_wrist_force_n", "max_wrist_torque_nm"):
                limit = _number(previous.get(key))
                value = _number(current.get(key))
                if limit is None or value is None:
                    reasons.append(f"{stage}_{key}_unavailable")
                elif value > limit * load_multiplier + 1e-9:
                    reasons.append(f"{stage}_{key}_regression")
    return reasons
