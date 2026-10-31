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
OPERATION_CLEARANCE_SCHEMA = "task-box-operation-clearance/1"


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
    relocation: dict | None = None


class PoolDocument(BaseModel):
    model_config = ConfigDict(extra="forbid")
    schema_version: Literal["collection-layout-pool/1", "collection-layout-pool/2"] = Field(alias="schema")
    layouts: list[LayoutEntry] = Field(min_length=2, max_length=1000)
    root: Literal["."] | None = None
    files_sha256: dict[str, str] | None = None
    source_pool_sha256: str | None = None
    generator: str | None = None


def validate_entry(entry, *, root=None, verified_hashes=None):
    from simulation.task_box_layout import validated_capture_layout

    accepted_model_sha = entry["model_sha256"]
    if root is not None:
        from bsk_render_adapter.portable_assets import package_path
        from simulation.layout_package import validate_relocation
        proof = validate_relocation(root, entry, verified_hashes=verified_hashes)
        accepted_model_sha = proof["original_model_sha256"]
        model, catalog, report = (package_path(root, entry[k]) for k in ("model", "catalog", "native_report"))
    else:
        model, catalog, report = (Path(entry[k]).resolve() for k in ("model", "catalog", "native_report"))
    manifest = model.with_suffix(".layout.json")
    for path, key in ((model, "model"), (catalog, "catalog"),
                      (report, "native_report"), (manifest, "manifest")):
        if digest(path) != entry[key + "_sha256"]:
            raise ValueError(f"Layout {entry['id']} has a changed {key}")
    saved = json.loads(manifest.read_text())
    clearance = saved.get("operational_clearance")
    if (not isinstance(clearance, dict)
            or clearance.get("schema") != OPERATION_CLEARANCE_SCHEMA):
        raise ValueError(
            f"Layout {entry['id']} lacks current operational clearance; "
            "regenerate layouts and prepare a new pool"
        )
    record = validated_capture_layout(model, catalog)
    if record["schema"] not in ("task-box-layout/2", "task-box-layout/3"):
        raise ValueError("Collection requires independent installation-unit layouts")
    native = json.loads(report.read_text())
    if (native.get("passed") is not True or native.get("initial_states_preserved") is not True
            or native.get("model_sha256") != accepted_model_sha
            or native.get("source_model_sha256") != record["source_sha256"]
            or native.get("operation_clearance_schema") != clearance["schema"]):
        raise ValueError(
            f"Layout {entry['id']} lacks matching native geometry acceptance with current "
            "operational clearance; regenerate layouts and prepare a new pool"
        )
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
        reference = json.loads(raw)
        if reference.get("schema") == "collection-layout-pool-reference/1":
            self.path = Path(reference["pool_path"]).resolve()
            raw = self.path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != reference["pool_sha256"]:
                raise ValueError("Referenced frozen layout pool changed")
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.document = PoolDocument.model_validate_json(raw).model_dump(by_alias=True, exclude_none=True)
        self.package_root = self.path.parent if self.document['schema'] == 'collection-layout-pool/2' else None
        verified_hashes = None
        if self.package_root is not None:
            if self.document.get('root') != '.' or not self.document.get('files_sha256'):
                raise ValueError('Portable layout pools require an explicit root and file manifest')
            verified_hashes = self._validate_package_files()
        entries = self.document["layouts"]
        if len({e["id"] for e in entries}) != len(entries):
            raise ValueError("Layout ids must be unique")
        if len({e["model_sha256"] for e in entries}) != len(entries):
            raise ValueError("Pool must contain distinct layouts")
        for entry in entries:
            if self.package_root is None:
                if entry.get('relocation') is not None:
                    raise ValueError('Relocation proof requires a portable pool')
                for key in ("model", "catalog", "native_report"):
                    if not Path(entry[key]).is_absolute():
                        raise ValueError("Pool entries require absolute paths")
            elif entry.get('relocation') is None:
                raise ValueError('Portable pool lacks original native acceptance relocation proof')
            validate_entry(entry, root=self.package_root, verified_hashes=verified_hashes)

    def _validate_package_files(self):
        from bsk_render_adapter.portable_assets import package_path, package_root, ue_package_relative
        from simulation.layout_package import mjcf_files

        declared = self.document['files_sha256']
        verified_hashes = {}
        for relative, expected in declared.items():
            path = package_path(self.package_root, relative)
            actual = digest(path) if path.is_file() else None
            if actual != expected:
                raise ValueError(f'Portable layout package missing or changed file: {relative}')
            verified_hashes[path] = actual

        def require(relative):
            path = package_path(self.package_root, relative)
            if path.relative_to(self.package_root).as_posix() not in declared:
                raise ValueError(f'Portable layout dependency missing from file manifest: {relative}')
            return path

        require('layout-package.json')
        for entry in self.document['layouts']:
            model, catalog, _ = (require(entry[key]) for key in ('model', 'catalog', 'native_report'))
            manifest = require(model.with_suffix('.layout.json').relative_to(self.package_root).as_posix())
            for document in (manifest, catalog):
                payload = json.loads(document.read_text())
                if package_root(document, payload['package_root']) != self.package_root:
                    raise ValueError('Portable document uses a different package root')
            for _, _, asset in mjcf_files(model)[1]:
                if not asset.is_relative_to(self.package_root):
                    raise ValueError('Relocated MJCF asset escapes package root')
                require(asset.relative_to(self.package_root).as_posix())
            record = json.loads(manifest.read_text())
            require(record['model'])
            if record.get('source'):
                require(record['source'])
            require(record['adapted_base']['visual'])
            for visual in record.get('generated_visuals', []):
                require(visual['file'])
            for source in record.get('source_asset_sha256', {}):
                require(source)
            assets = json.loads(catalog.read_text())
            require(assets['source_mjcf'])
            for source, value in assets.get('assets', {}).items():
                require(source)
                if isinstance(value, dict) and value.get('import_source'):
                    require(value['import_source'])
                package = value if isinstance(value, str) else value.get('asset_path')
                if package:
                    require('ue-content/' + ue_package_relative(package))
            for source, package in assets.get('textures', {}).items():
                require(source)
                require('ue-content/' + ue_package_relative(package))
            proof = entry.get('relocation')
            if proof is None:
                raise ValueError('Portable pool lacks original native acceptance relocation proof')
            require(proof['original_model'])
            require(proof['original_manifest'])
            for relative in proof['asset_mapping'].values():
                require(relative)
            companion = model.with_name('attitude_control.json')
            if companion.exists():
                require(companion.relative_to(self.package_root).as_posix())
        # Reuse reads only during this verification. Every new selection hashes
        # all declared files again; mutable stat/mtime caches cannot certify data.
        return verified_hashes

    def write_snapshot(self, path):
        """Freeze legacy bytes or a checked reference to the immutable portable package."""
        document = self.document if self.package_root is None else dict(
            schema='collection-layout-pool-reference/1', pool_path=str(self.path), pool_sha256=self.sha256)
        Path(path).write_text(json.dumps(document, indent=2) + '\n')

    def select(self, seed):
        from simulation.scripted_tasks.randomization import generator

        entries = self.document["layouts"]
        index = int(generator(seed, STREAM).integers(len(entries)))
        return self.select_index(index, seed)

    def select_index(self, index, seed, *, stream=STREAM):
        """Select an explicitly planned layout without replacement or resampling."""
        entries = self.document["layouts"]
        if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(entries):
            raise ValueError("Layout index is outside the pool")
        entry = deepcopy(entries[index])
        verified_hashes = None
        if self.package_root is not None:
            verified_hashes = self._validate_package_files()
        record = validate_entry(entry, root=self.package_root, verified_hashes=verified_hashes)
        selection = dict(enabled=True, stream=stream, selection_seed=int(seed),
                         pool_sha256=self.sha256, pool_size=len(entries), pool_index=index,
                         layout_id=entry["id"], layout_seed=record["seed"],
                         model_sha256=entry["model_sha256"], catalog_sha256=entry["catalog_sha256"],
                         operation_clearance_schema=record["operational_clearance"]["schema"],
                         native_report_sha256=entry["native_report_sha256"],
                         acceptance_scope="native_geometry",
                         offsets_body_xy_m=record["offsets_body_xy_m"],
                         installation_yaw_rad=record.get("installation_yaw_rad", {}),
                         initial_states_randomized=False)
        if self.package_root is not None:
            from bsk_render_adapter.portable_assets import package_path
            return (package_path(self.package_root, entry["model"]),
                    package_path(self.package_root, entry["catalog"]), record, selection)
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
