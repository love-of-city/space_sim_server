"""Generate the placement stage of an offline task-box layout prototype.

The delivered assembly has two separate boards. The pickup board owns the
three plugs' starting seats; the assembly board owns the receiving bores and
the bolt's storage seat. This stage translates them without adding a joint or
changing a mechanism's initial joint state. The base apertures still need to
be relocated by tools/assets/adapt_task_box_layout_base.py before runtime validation.

This prototype does not move the drawer/latch or split a shared board into
individual socket modules. Geometric placement limits are not a robot motion
planner; a generated layout still needs native and rendering acceptance.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET


SCHEMA = "task-box-layout/1"
MODULES = {
    "pickup_board": {
        "fixed": ("asm_board2",),
        "free": ("aviation_plug", "guide_plug", "cylindrical_plug"),
    },
    "assembly_board": {
        "fixed": ("asm_board1", "board1_plug_bore_colliders"),
        "free": ("bolt",),
    },
}


def validated_capture_layout(model: Path, catalog: Path) -> dict:
    """Bind a generated model and its adapted visual to one capture catalog."""
    model, catalog = Path(model).resolve(), Path(catalog).resolve()
    manifest = json.loads(model.with_suffix(".layout.json").read_text())
    independent = manifest.get("schema") in ("task-box-layout/2", "task-box-layout/3")
    if manifest.get("schema") not in (SCHEMA, "task-box-layout/2", "task-box-layout/3") or not manifest.get("adapted_base"):
        raise ValueError("Capture requires an adapted task-box layout manifest")
    if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["model_sha256"]:
        raise ValueError("Layout manifest does not match the selected model")
    assets = json.loads(catalog.read_text())
    if Path(assets["source_mjcf"]).resolve() != model:
        raise ValueError("Capture catalog belongs to a different model")
    visual = Path(manifest["adapted_base"]["visual"]).resolve()
    entry = assets["assets"].get(str(visual), {})
    if entry.get("source_sha256") != hashlib.sha256(visual.read_bytes()).hexdigest():
        raise ValueError("Capture catalog does not contain the adapted base visual")
    if independent:
        from .independent_task_box_layout import PICKUPS, TARGETS, validate_layout_record
        validate_layout_record(manifest)
        visuals = manifest.get("generated_visuals", [])
        if len(visuals) != 12 or {v["name"] for v in visuals} != {*PICKUPS, *TARGETS, "task_box_base"}:
            raise ValueError("Independent layout requires all 12 generated visual assets")
        for generated in visuals:
            file = Path(generated["file"]).resolve()
            sha = hashlib.sha256(file.read_bytes()).hexdigest()
            if sha != generated["sha256"] or assets["assets"].get(str(file), {}).get("source_sha256") != sha:
                raise ValueError(f"Capture catalog has a missing or stale independent visual: {file.name}")
    return manifest


def sampled_offsets(seed: int, half_range_m: float = .02) -> dict[str, list[float]]:
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    if not math.isfinite(half_range_m) or not 0 <= half_range_m <= .04:
        raise ValueError("prototype half-range must be between 0 and 0.04 m")
    rng = random.Random(f"{SCHEMA}:{seed}")
    return {name: [rng.uniform(-half_range_m, half_range_m) for _ in range(2)]
            for name in MODULES}


def _vector(value: str | None, length: int, default: str) -> list[float]:
    values = [float(x) for x in (value or default).split()]
    if len(values) != length or not all(math.isfinite(x) for x in values):
        raise ValueError("Invalid assembly transform")
    return values


def build_layout(source: Path, output: Path, offsets: dict, *, seed: int | None = None) -> dict:
    """Write a separate model, retaining node names, shape, orientation and DOFs.

    Offsets are [dx, dy] in the satellite body frame, in metres. Plug and bolt
    initial poses follow their own storage board. Every collider/site/SDF in
    a translated board inherits exactly its board's transform.
    """
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError("Output must be a separate model")
    if set(offsets) != set(MODULES):
        raise ValueError(f"Offsets must specify exactly {tuple(MODULES)}")
    checked = {}
    for name, values in offsets.items():
        if not isinstance(values, (list, tuple)) or len(values) != 2:
            raise ValueError(f"{name} requires [dx, dy]")
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) or abs(v) > .04 for v in values):
            raise ValueError(f"{name} offsets must be finite and within +/-0.04 m")
        checked[name] = [float(v) for v in values]
    before = source.read_bytes()
    root = ET.fromstring(before)
    if root.find("include") is not None or root.find("keyframe") is not None:
        raise ValueError("Prototype requires a standalone model without keyframes")
    bus = root.find('./worldbody/body[@name="cubesat_bus"]')
    base = root.find('.//body[@name="task_box_base"]')
    if bus is None or base is None or base not in list(bus):
        raise ValueError("Expected the SARM task-box assembly attached to cubesat_bus")
    # Adding a body-frame translation directly requires this fixed parent frame.
    if _vector(base.get("quat"), 4, "1 0 0 0") != [1., 0., 0., 0.]:
        raise ValueError("Expected identity task-box orientation")
    records = []
    for name, ownership in MODULES.items():
        dx, dy = checked[name]
        for kind, names in ownership.items():
            for body_name in names:
                parent = base if kind == "fixed" else root.find("worldbody")
                body = parent.find(f'body[@name="{body_name}"]')
                if body is None:
                    raise ValueError(f"Missing {kind} module body {body_name}")
                is_free = (body.find("freejoint") is not None
                           or body.find('joint[@type="free"]') is not None)
                if (kind == "free") != is_free:
                    raise ValueError(f"Unexpected joint ownership for {body_name}")
                old = _vector(body.get("pos"), 3, "0 0 0")
                new = [old[0] + dx, old[1] + dy, old[2]]
                body.set("pos", " ".join(format(v, ".17g") for v in new))
                records.append(dict(module=name, body=body_name, kind=kind,
                                    before_parent_position_m=old, after_parent_position_m=new,
                                    offset_body_m=[dx, dy, 0.]))
    # Resolve existing mesh/texture references without copying or changing assets.
    compiler = root.find("compiler")
    if compiler is None:
        raise ValueError("Missing model compiler configuration")
    for tag, directory in (("mesh", "meshdir"), ("texture", "texturedir"),
                           ("hfield", "assetdir")):
        folder = source.parent / compiler.get(directory, compiler.get("assetdir", "."))
        for asset in root.findall(f"./asset/{tag}[@file]"):
            asset.set("file", str((folder / asset.get("file")).resolve()))
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    compiler.attrib.pop("assetdir", None)
    root.set("model", "SARM_task_box_layout_prototype")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    settings = source.with_name("attitude_control.json")
    if settings.is_file() and settings.resolve() != output.with_name(settings.name).resolve():
        output.with_name(settings.name).write_bytes(settings.read_bytes())
    if hashlib.sha256(source.read_bytes()).digest() != hashlib.sha256(before).digest():
        raise RuntimeError("Source model changed during generation")
    manifest = dict(schema=SCHEMA, source=str(source), output=str(output), seed=seed,
                    source_sha256=hashlib.sha256(before).hexdigest(),
                    model_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                    offsets_body_xy_m=checked, translated_bodies=records,
                    state_randomization=False, orientation_randomization=False,
                    placement_scope="two independently translated boards; plugs remain a group",
                    unmodified_mechanisms=["drawer", "locking rod", "captive fastener"],
                    validation_scope="placement stage only; base aperture adaptation, native contact, dynamics, robot reachability and UE need separate validation")
    output.with_suffix(".layout.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest
