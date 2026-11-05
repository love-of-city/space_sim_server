"""Restore a recorded floating scene for a policy, without its expert controller."""
from copy import deepcopy
import json
import hashlib
from pathlib import Path
import re
import shutil
from uuid import uuid4


def _contained(root: Path, path: Path) -> Path:
    resolved = path.resolve()
    if not resolved.is_relative_to(root.resolve()) or not resolved.is_file():
        raise ValueError("Recorded scene artifact is outside its collection or missing")
    return resolved


def recorded_model(episodes_root: Path, episode_id: str, scene: dict) -> tuple[Path, str]:
    collections = episodes_root.resolve().parent / "collections"
    batch = scene["collection_batch"]
    summary_path = _contained(collections, collections / batch["batch_id"] / "attempts" / batch["attempt_id"] / "summary.json")
    summary = json.loads(summary_path.read_text())
    matches = [row for row in summary["attempts"] if row.get("episode_id") == episode_id and row.get("status") == "accepted"]
    if len(matches) != 1:
        raise ValueError("Source episode must match exactly one accepted collection attempt")
    directory = Path(matches[0]["directory"]) / "recording"
    recording = json.loads(_contained(collections, directory / "summary.json").read_text())
    if recording.get("episode_id") != episode_id:
        raise ValueError("Recording episode identity does not match")
    dynamics = json.loads(_contained(collections, directory / "dynamics/summary.json").read_text())
    model = _contained(collections, directory / "dynamics/model.xml")
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    if digest != dynamics["model_sha256"] or digest != dynamics["render_model_sha256"]:
        raise ValueError("Recorded physics/render model digest mismatch")
    return model, digest


def restore_scene(episodes_root: Path, episode_id: str, created_by: dict | None) -> dict:
    if re.fullmatch(r"episode-[A-Za-z0-9_-]+", episode_id) is None:
        raise ValueError("Invalid source episode ID")
    root = episodes_root.resolve()
    path = (root / episode_id / "metadata.json").resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("Source episode metadata is unavailable")
    metadata = json.loads(path.read_text())
    scene = deepcopy(metadata.get("scene_instance"))
    if not isinstance(scene, dict) or scene.get("template_id") != "sarm-task-box-20261012":
        raise ValueError("Source episode must contain a recorded SARM scene")
    cfg = scene.get("collection_config", {})
    if any(cfg.get("disturbances", {}).values()):
        raise ValueError("Policy scene replay currently requires source disturbances disabled")
    if not cfg.get("floating_objects", {}).get("instances"):
        raise ValueError("Source episode must contain saved floating object initial states")
    if metadata.get("dataset_status") != "complete":
        raise ValueError("Source recording is incomplete")
    model, digest = recorded_model(root, episode_id, scene)
    scene["source_instance_id"] = scene["instance_id"]
    scene["instance_id"] = "policy-" + uuid4().hex[:16]
    scene["created_by"] = dict(created_by or {})
    scene["policy_replay"] = {"source_episode_id": episode_id, "instruction": metadata["instruction"],
                              "model_path": str(model), "model_sha256": digest}
    scene["collection_batch"] = {"batch_id": "policy-rollouts", "group_id": cfg["task"],
                                 "cell_id": episode_id, "attempt_id": scene["instance_id"]}
    for key in ("scripted_task_name", "scripted_task_id", "scripted_task_target_site", "scripted_task_instruction"):
        scene.pop(key, None)
    return scene


def copy_recorded_model(scene: dict, output: Path) -> Path:
    record = scene["policy_replay"]
    source = Path(record["model_path"])
    if hashlib.sha256(source.read_bytes()).hexdigest() != record["model_sha256"]:
        raise ValueError("Recorded model changed after scene restoration")
    destination = output / "policy-model.xml"
    shutil.copy2(source, destination)
    attitude = _contained(source.parent, source.with_name("attitude_control.json"))
    shutil.copy2(attitude, output / "attitude_control.json")
    return destination
