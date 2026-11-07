"""Freeze actual collection code, native/UE builds and inputs before execution.

Documentation-only changes do not invalidate a run. Runtime code (including
untracked files), native products and frozen inputs do. Legacy batches remain
readable but cannot silently gain a new implementation by being resumed.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "collection-provenance/1"
SOURCE_DIRECTORIES = ("backend", "collection", "simulation", "native", "config", "contracts",
                      "third_party/mujoco/include")
SOURCE_SUFFIXES = {".py", ".cpp", ".h", ".hpp", ".c", ".json", ".toml", ".yaml", ".yml"}
RUNTIME_ENVIRONMENT = (
    "SPACE_SIM_DYNAMICS_PYTHON",
    "SPACE_SIM_DYNAMICS_BACKEND", "SPACE_SIM_EXTRA_EOM_CALL", "SPACE_SIM_IK_MODE",
    "SPACE_SIM_LOCAL_PUBLISH_STRIDE", "SPACE_SIM_LOCAL_SUBSTEPS", "SPACE_SIM_ONLINE_ELBOW_MODE",
    "SPACE_SIM_ONLINE_JOINT3_MODE", "SPACE_SIM_ONLINE_WRIST_MODE", "SPACE_SIM_ORBITAL_FORCE_STRIDE",
    "SPACE_SIM_ORBITAL_MODE", "SPACE_SIM_PLUG_RETENTION_CONFIG", "SPACE_SIM_POSTURE_BACKEND",
    "SPACE_SIM_REUSE_GRAPH", "SPACE_SIM_RKF45_ABSOLUTE_TOLERANCE", "SPACE_SIM_RKF45_RELATIVE_TOLERANCE",
    "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
    "CYLINDER_FILTER_MODE", "MESH_FILTER_MODE", "THREAD_FILTER_MODE",
)


def load_build_identity(adapter: Path):
    """Avoid importing the adapter package and its optional simulation runtime."""
    path = Path(adapter) / "Adapters/bsk_render_adapter/build_identity.py"
    spec = importlib.util.spec_from_file_location("spacesim_build_identity", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode()).hexdigest()


def source_identity(server=ROOT, adapter=None):
    server = Path(server).resolve()
    adapter = Path(adapter or server.parent / "ue-adapter").resolve()
    entries = {}
    roots = [("server/" + name, server / name) for name in SOURCE_DIRECTORIES]
    roots.append(("ue-adapter/Adapters", adapter / "Adapters"))
    for prefix, directory in roots:
        for path in sorted(directory.rglob("*")):
            if path.is_file() and path.suffix in SOURCE_SUFFIXES and "__pycache__" not in path.parts:
                entries[prefix + "/" + path.relative_to(directory).as_posix()] = digest(path)
    # Scene entry points are executable runtime code despite living beside models
    # and examples; model/asset content itself is frozen by the layout manifests.
    for prefix, directory in (("server/model", server / "model"),
                              ("ue-adapter/Unreal/BskUnrealRenderer/examples",
                               adapter / "Unreal/BskUnrealRenderer/examples")):
        for path in sorted(directory.rglob("*.py")):
            if path.is_file():
                entries[prefix + "/" + path.relative_to(directory).as_posix()] = digest(path)
    for prefix, path in (("server/pyproject.toml", server / "pyproject.toml"),
                         ("server/tools/collect.py", server / "tools/collect.py"),
                         ("ue-adapter/pyproject.toml", adapter / "pyproject.toml")):
        if path.exists():
            entries[prefix] = digest(path)
    return {"sha256": fingerprint(entries), "files": entries}


def native_identity(server=ROOT):
    server = Path(server).resolve()
    result = {}
    directory = server / "run/native_acceleration"
    for path in sorted(directory.glob("*.json")):
        document = json.loads(path.read_text(encoding="utf-8"))
        name = path.stem
        library = (server / document["library"]).resolve()
        if not library.is_relative_to(directory.resolve()):
            raise ValueError(f"Native library escapes runtime directory: {name}")
        library_hash = digest(library)
        expected = document.get("library_sha256", document.get("plugin_sha256"))
        if expected != library_hash:
            raise RuntimeError(f"Native library fingerprint differs: {name}; rebuild")
        source = server / "native" / (name + ".cpp")
        if source.is_file() and name != "sarm_thread" and digest(source) != document.get("source_sha256"):
            raise RuntimeError(f"Native source differs: {name}; rebuild")
        if name == "sarm_thread":
            for key, relative in (("source_sha256", "native/sarm_thread_sdf.cpp"),
                                  ("delivered_source_sha256", "model/SARM_mjcf_20261012/thread_sdf.cpp"),
                                  ("checker_sha256", "native/sarm_thread_sdf_check.cpp")):
                if digest(server / relative) != document.get(key):
                    raise RuntimeError(f"SARM plugin source differs: {relative}; rebuild")
        result[name] = {"source_sha256": document.get("source_sha256"),
                        "library_sha256": library_hash, "library": document["library"],
                        "abi": document.get("abi", document.get("runtime_version")),
                        "compiler_flags": document.get("compiler_flags", [])}
        if name == "sarm_thread":
            result[name]["delivered_source_sha256"] = document["delivered_source_sha256"]
            result[name]["checker_sha256"] = document["checker_sha256"]
    if not result:
        raise RuntimeError("Native build manifests are missing; prepare the runtime before collection")
    return result


def environment_identity():
    # ``importlib.metadata`` walks every entry on ``sys.path``.  The
    # collector and its recording child intentionally prepend the same
    # source directories, so an editable distribution can otherwise appear
    # multiple times and make the runtime fingerprint differ between the two
    # processes.  Package identity is the normalized name/version pair; keep
    # one record per pair.
    distributions = sorted({(d.metadata.get("Name", "").lower(), d.version)
                            for d in importlib.metadata.distributions()})
    native_packages = {}
    for name in ("bsk", "mujoco", "numpy"):
        try:
            distribution = importlib.metadata.distribution(name)
        except importlib.metadata.PackageNotFoundError:
            continue
        files = {}
        for entry in distribution.files or []:
            relative = str(entry)
            if (relative.endswith((".py", ".so", ".pyd", ".dll", ".dylib", "/RECORD"))
                    or ".so." in relative):
                path = distribution.locate_file(entry)
                if path.is_file():
                    files[relative] = digest(path)
        native_packages[name] = {"version": distribution.version, "files_sha256": fingerprint(files)}
    override_files = {name: digest(Path(os.environ[name]).expanduser())
                      for name in RUNTIME_ENVIRONMENT if name.endswith("_CONFIG")
                      and os.environ.get(name) and Path(os.environ[name]).expanduser().is_file()}
    return {"python": platform.python_version(), "implementation": sys.implementation.name,
            "cache_tag": sys.implementation.cache_tag, "executable_sha256": digest(Path(sys.executable).resolve()),
            "platform": sys.platform, "architecture": platform.machine(), "packages": distributions,
            "native_packages": native_packages,
            "override_files_sha256": override_files,
            "overrides": {name: os.environ[name] for name in RUNTIME_ENVIRONMENT if name in os.environ}}


def loaded_native_provenance(server_root=ROOT):
    """Inspect already mapped helper libraries without causing any library load."""
    server = Path(server_root).resolve()
    expected = native_identity(server)
    expected_paths = {(server / item["library"]).resolve(): (name, item)
                      for name, item in expected.items()}
    if sys.platform.startswith("linux"):
        paths = []
        for line in Path("/proc/self/maps").read_text().splitlines():
            fields = line.split(maxsplit=5)
            if len(fields) == 6 and "native_acceleration/" in fields[5]:
                if fields[5].endswith(" (deleted)"):
                    raise RuntimeError("Loaded native helper was replaced while the worker was running")
                paths.append(Path(fields[5].replace("\\040", " ")).resolve())
    else:
        import psutil
        paths = [Path(entry.path).resolve() for entry in psutil.Process().memory_maps()
                 if "native_acceleration" in Path(entry.path).parts]
    loaded = []
    for path in sorted(set(paths)):
        if path not in expected_paths:
            raise RuntimeError(f"Worker loaded an uncertified native helper: {path}")
        name, item = expected_paths[path]
        actual = digest(path)
        if actual != item["library_sha256"]:
            raise RuntimeError(f"Worker native helper differs from its manifest: {name}")
        loaded.append({"name": name, "path": str(path), "sha256": actual, "manifest_verified": True})
    return loaded


def runtime_identity(*, server=ROOT, adapter=None, unreal=None):
    from space_arm_platform.unreal_runtime import default_unreal_root
    server = Path(server).resolve()
    adapter = Path(adapter or server.parent / "ue-adapter").resolve()
    unreal = Path(unreal or default_unreal_root()).resolve()
    build = load_build_identity(adapter).verify_build_manifest(adapter / "Unreal/BskUnrealRenderer", unreal)
    return {"source": source_identity(server, adapter), "native": native_identity(server),
            "environment": environment_identity(), "ue_build_id": build["build_id"]}


def git_identity(server=ROOT):
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=server, text=True).strip()
        status = subprocess.check_output(["git", "status", "--porcelain"], cwd=server, text=True)
        return {"commit": commit, "dirty": bool(status.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None, "source": "unversioned_distribution"}


def write_json(path, document):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def freeze_or_verify(output, config, *, server=ROOT, identity=None):
    """Create a new provenance record or refuse implementation changes on resume."""
    output = Path(output)
    path = output / "launch-provenance.json"
    inputs = {name: digest(output / name) for name in
              ("config.snapshot.json", "plan.json", "pool.snapshot.json", "source-scene.json", "inputs.json")}
    inputs["recording_config"] = digest(config.recording_config)
    value = identity if identity is not None else runtime_identity(
        server=server, adapter=config.adapter, unreal=config.unreal)
    current = {"runtime": value, "inputs": inputs, "seed": config.seed}
    runtime_hash = fingerprint(current)
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("schema") != SCHEMA:
            raise RuntimeError("Legacy batch has no certified runtime identity; preserve it and start a new batch")
        if previous.get("identity_sha256") != runtime_hash or fingerprint(previous.get("identity")) != runtime_hash:
            raise RuntimeError("Collection runtime or frozen inputs changed; use a new batch, not --resume")
        return previous
    progress = json.loads((output / "progress.json").read_text())
    if progress.get("jobs"):
        raise RuntimeError("Cannot certify an existing batch retroactively; use a new batch")
    document = {"schema": SCHEMA, "identity_sha256": runtime_hash, "identity": current,
                "created_at": datetime.now(timezone.utc).isoformat(), "git": git_identity(server),
                "locations": {"server": str(Path(server).resolve()), "python": sys.executable},
                "seed_source": "resolved_and_frozen_config",
                "runtime_reports": "Per-renderer runtime-identity.json and per-attempt recording summaries"}
    write_json(path, document)
    return document
