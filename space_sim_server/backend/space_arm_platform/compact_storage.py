"""Versioned, bounded columnar sidecars for compact LeRobot recordings.

Main training features remain in the official LeRobot table. Sidecars retain
camera geometry, task/physics context and the independent sensor clock.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

COMPACT_SCHEMA = "space-arm-compact/1"
CONTEXT_FIELDS = (
    "render_session_id", "reset_generation", "capture_episode_id",
    "gripper_drive_position_rad", "target_gripper_drive_position_rad",
    "gripper_position_m", "target_gripper_position_m",
    "scripted_task", "orbital_dynamics", "attitude_control", "reaction_wheels",
    "dynamics_rate_hz", "scripted_task_rate_hz", "wrist_wrench_rate_hz",
)


def arrow_value(value):
    # Parquet cannot encode empty structs. No numeric precision is discarded.
    if hasattr(value, "model_dump"):
        value = value.model_dump(mode="json")
    if isinstance(value, dict):
        return {k: arrow_value(v) for k, v in value.items()} if value else None
    if isinstance(value, list):
        return [arrow_value(v) for v in value]
    return value


class ParquetChunks:
    """Bounded buffering, with independently typed chunks for optional fields."""
    def __init__(self, root, relative, batch_size=1024):
        self.path = Path(root) / relative
        self.batch_size = batch_size
        self.rows = []
        self.chunks = 0
        self.count = 0

    def append(self, row):
        self.rows.append(arrow_value(row))
        self.count += 1
        if len(self.rows) >= self.batch_size:
            self.flush()

    def flush(self):
        if not self.rows:
            return
        import pyarrow as pa
        import pyarrow.parquet as pq
        self.path.mkdir(parents=True, exist_ok=True)
        destination = self.path / f"part-{self.chunks:06d}.parquet"
        temporary = destination.with_suffix(".tmp")
        pq.write_table(pa.Table.from_pylist(self.rows), temporary, compression="zstd")
        temporary.replace(destination)
        self.rows.clear()
        self.chunks += 1


def read_chunks(path):
    import pyarrow.parquet as pq
    path = Path(path)
    files = [path] if path.is_file() else sorted(path.glob("*.parquet"))
    for file in files:
        yield from pq.read_table(file).to_pylist()


def numeric_digest(frame):
    """Digest of original float32/int64 training values, before dataset append."""
    values = {}
    for key, value in frame.items():
        if key != "action" and not key.startswith("observation."):
            continue
        if key.startswith(("observation.images.", "observation.camera_metadata.")) or key == "observation.platform_json":
            continue
        if hasattr(value, "tolist"):
            value = value.tolist()
        if isinstance(value, list) and len(value) == 1:
            value = value[0]
        values[key] = value
    encoded = json.dumps(values, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def wrench_bytes(row):
    return (json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


class CompactSidecars:
    def __init__(self, root):
        self.context = ParquetChunks(root, "auxiliary/context")
        self.cameras = ParquetChunks(root, "auxiliary/cameras")

    def append(self, observation, captures, frame):
        extra = observation.model_extra or {}
        context = {key: getattr(observation, key, extra.get(key)) for key in CONTEXT_FIELDS}
        context.update(source_frame_id=int(observation.render_frame_id),
                       sim_time_ns=int(observation.sim_time_ns),
                       source_values_sha256=numeric_digest(frame))
        self.context.append(context)
        for metadata, _ in captures.values():
            self.cameras.append(metadata)

    def finish(self):
        self.context.flush()
        self.cameras.flush()


def is_compact(episode):
    return (Path(episode) / "lerobot/auxiliary/context").is_dir()


def read_observations(episode):
    """Read either historical audit JSONL or compact training+context tables."""
    episode = Path(episode)
    if not is_compact(episode):
        with (episode / "steps.jsonl").open(encoding="utf-8-sig") as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)["observation"]
        return
    import pyarrow.parquet as pq
    contexts = iter(read_chunks(episode / "lerobot/auxiliary/context"))
    for file in sorted((episode / "lerobot/data").rglob("*.parquet")):
        for row in pq.read_table(file).to_pylist():
            context = next(contexts, None)
            if context is None or context["source_frame_id"] != row["observation.source_frame_id"]:
                raise ValueError("Compact context/frame mismatch")
            if context["sim_time_ns"] != row["observation.sim_time_ns"] or context["source_values_sha256"] != numeric_digest(row):
                raise ValueError("Compact source values checksum/time mismatch")
            pose = row["observation.end_effector_pose_body"]
            obs = {**context,
                "render_frame_id": str(context["source_frame_id"]),
                "sim_time_ns": str(context["sim_time_ns"]),
                "joint_position_rad": row.get("observation.raw_joint_position", row["observation.state"]),
                "target_joint_position_rad": row.get("observation.raw_target_joint_position", row["action"]),
                "joint_velocity_rad_s": row["observation.joint_velocity"],
                "end_effector_position_body_m": pose[:3],
                "end_effector_orientation_body_wxyz": pose[3:],
                "end_effector_twist_body": row["observation.end_effector_twist_body"],
            }
            if len(row["observation.state"]) == 7:
                obs["gripper_opening_normalized"] = row["observation.state"][-1]
                obs["target_gripper_opening_normalized"] = row["action"][-1]
            if "observation.wrist_wrench" in row:
                wrench = row["observation.wrist_wrench"]
                obs["wrist_wrench"] = dict(force_n=wrench[:3], torque_nm=wrench[3:],
                    valid=bool(row["observation.wrist_wrench_valid"]),
                    sample_time_ns=row["observation.wrist_wrench_sample_time_ns"],
                    interval_end_time_ns=row["observation.wrist_wrench_interval_end_time_ns"])
            yield obs
    if next(contexts, None) is not None:
        raise ValueError("Extra compact context frames")


def read_wrench(episode):
    root = Path(episode) / "lerobot/high_frequency"
    if (root / "wrist_wrench").is_dir():
        yield from read_chunks(root / "wrist_wrench")
    elif (root / "wrist_wrench.jsonl").is_file():
        with (root / "wrist_wrench.jsonl").open(encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)
