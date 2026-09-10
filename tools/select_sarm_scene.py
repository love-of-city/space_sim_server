"""Resolve and check the selected runtime XML for startup (standard library only).

No model regeneration, NumPy, MuJoCo or Basilisk imports. Coarse self-contact is
the platform default; preserved triangle artifacts load only for their opt-in template.
Missing/stale selected assets fail without changing templates. ModelRoot remains
SARM/platform for the native scenario loader.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, MESH_TARGET_TEMPLATE, capture_target


def fingerprint(path: Path) -> str:
    data = path.read_bytes()
    if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise ValueError(f"Mesh is an LFS pointer: {path}. Run git lfs pull.")
    if path.suffix in (".xml", ".json"):
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def check_fingerprints(root: Path, entries: dict[str, str]):
    for name, expected in entries.items():
        path = (root / name).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Missing selected scene dependency: {path}. Run git lfs pull / rebuild the model.")
        if fingerprint(path) != expected:
            raise ValueError(f"Stale/modified scene dependency: {path}. Rebuild and validate the model; no template fallback is used.")


def selected_scene(model_root: Path, template_id: str = DEFAULT_TEMPLATE, check: bool = False) -> Path:
    model_root = model_root.resolve()
    path = capture_target(template_id).resolve_model(model_root)
    if not path.is_file():
        raise FileNotFoundError(f"Selected runtime XML is missing: {path}")
    if check:
        repository = model_root.parents[2]
        if template_id == MESH_TARGET_TEMPLATE:
            manifest = json.loads(path.with_name("manifest.json").read_text(encoding="utf-8"))
            if manifest.get("schema") != "original-triangle-collision-trial/1":
                raise ValueError("Unsupported triangle collision manifest")
            check_fingerprints(repository, manifest["source_hashes"])
            check_fingerprints(path.parent, manifest["output_sha256"])
            check_fingerprints(repository, {s["source_obj"]: s["sha256"] for s in manifest["surfaces"]})
        elif path.with_suffix(".manifest.json").is_file():
            manifest = json.loads(path.with_suffix(".manifest.json").read_text(encoding="utf-8"))
            check_fingerprints(path.parent, {path.name: manifest["sha256"]})
            check_fingerprints(repository, manifest["sources"])
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-root", type=Path, default=ROOT / "model/SARM/platform")
    parser.add_argument("--template-id", default=DEFAULT_TEMPLATE)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        print(selected_scene(args.model_root, args.template_id, args.check))
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Scene selection failed: {error}\n")


if __name__ == "__main__":
    main()
