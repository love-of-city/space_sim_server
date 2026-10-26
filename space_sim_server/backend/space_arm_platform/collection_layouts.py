"""Immutable, validated layout pools for API and scripted collectors.

Prepare meshes and UE assets before workers start. Workers only read the pool;
they never import assets into a project while another worker is rendering.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

STREAM = "task-box-layout-selection-v1"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class LayoutEntry(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str = Field(min_length=1, max_length=128)
    model: str
    catalog: str
    native_report: str
    model_sha256: str
    catalog_sha256: str
    manifest_sha256: str
    native_report_sha256: str


class PoolDocument(BaseModel):
    model_config = ConfigDict(extra="forbid")
    schema_version: Literal["collection-layout-pool/1"] = Field(alias="schema")
    layouts: list[LayoutEntry] = Field(min_length=2, max_length=1000)


def validate_entry(entry):
    from simulation.task_box_layout import validated_capture_layout

    model, catalog, report = (Path(entry[k]).resolve() for k in ("model", "catalog", "native_report"))
    manifest = model.with_suffix(".layout.json")
    for path, key in ((model, "model"), (catalog, "catalog"),
                      (report, "native_report"), (manifest, "manifest")):
        if digest(path) != entry[key + "_sha256"]:
            raise ValueError(f"Layout {entry['id']} has a changed {key}")
    record = validated_capture_layout(model, catalog)
    if record["schema"] not in ("task-box-layout/2", "task-box-layout/3"):
        raise ValueError("Collection requires independent installation-unit layouts")
    native = json.loads(report.read_text())
    if (native.get("passed") is not True or native.get("initial_states_preserved") is not True
            or native.get("model_sha256") != entry["model_sha256"]
            or native.get("source_model_sha256") != record["source_sha256"]):
        raise ValueError(f"Layout {entry['id']} lacks matching native geometry acceptance")
    return record


def pool_entry(identifier, model, catalog, native_report):
    """Register an existing accepted layout, with content hashes pinned."""
    paths = dict(model=Path(model).resolve(), catalog=Path(catalog).resolve(),
                 native_report=Path(native_report).resolve())
    entry = dict(id=identifier, **{key: str(path) for key, path in paths.items()},
                 **{key + "_sha256": digest(path) for key, path in paths.items()},
                 manifest_sha256=digest(paths["model"].with_suffix(".layout.json")))
    validate_entry(entry)
    return entry


class LayoutPool:
    def __init__(self, path):
        self.path = Path(path).resolve()
        raw = self.path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.document = PoolDocument.model_validate_json(raw).model_dump(by_alias=True)
        entries = self.document["layouts"]
        if len({e["id"] for e in entries}) != len(entries):
            raise ValueError("Layout ids must be unique")
        if len({e["model_sha256"] for e in entries}) != len(entries):
            raise ValueError("Pool must contain distinct layouts")
        for entry in entries:
            for key in ("model", "catalog", "native_report"):
                if not Path(entry[key]).is_absolute():
                    raise ValueError("Pool entries require absolute paths")
            validate_entry(entry)

    def select(self, seed):
        from simulation.scripted_tasks.randomization import generator

        entries = self.document["layouts"]
        index = int(generator(seed, STREAM).integers(len(entries)))
        entry = deepcopy(entries[index])
        record = validate_entry(entry)  # Fail closed if assets changed since startup.
        selection = dict(enabled=True, stream=STREAM, selection_seed=int(seed),
                         pool_sha256=self.sha256, pool_size=len(entries), pool_index=index,
                         layout_id=entry["id"], layout_seed=record["seed"],
                         model_sha256=entry["model_sha256"], catalog_sha256=entry["catalog_sha256"],
                         offsets_body_xy_m=record["offsets_body_xy_m"],
                         installation_yaw_rad=record.get("installation_yaw_rad", {}),
                         initial_states_randomized=False)
        return Path(entry["model"]), Path(entry["catalog"]), record, selection


def attach_layout(scene, pool, seed):
    """Attach after arm/noise/orbit randomization so no metadata is overwritten."""
    model, catalog, record, selection = pool.select(seed)
    scene["task_box_layout"] = deepcopy(record)
    scene["collection_config"]["layout_randomization"] = selection
    return model, catalog


def validate_episode_layout(episode, selection):
    meta = json.loads((Path(episode) / "metadata.json").read_text())
    scene = meta["scene_instance"]
    if scene["collection_config"].get("layout_randomization") != selection:
        raise ValueError("Recorded episode used a different layout selection")
    if scene["task_box_layout"]["model_sha256"] != selection["model_sha256"]:
        raise ValueError("Recorded layout model does not match selected layout")
    return selection
