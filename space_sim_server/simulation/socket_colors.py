"""Episode-scoped visual identities for the task-box plugs and holes.

The identities in this module are deliberately independent from the physical
model. A colour assignment only changes the material on an existing, visible
(``group=2`` and non-contact) geom. Sites, body names, collision geoms,
meshes, joints and inertias remain untouched, so the scripted task still uses
the stable ``source_body -> target_site`` mapping.

``sample_color_assignment`` is used at an episode boundary. It gives each of
the seven target holes a different colour and gives the three plugs plus the
bolt four different colours. The two groups are sampled independently, so a
plug colour may also be used by a hole. ``apply_socket_colors`` accepts that
record and applies it to an MJCF tree. Calling it without an assignment keeps
the deterministic legacy mapping used while building a layout pool.
"""
from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET


SCHEMA = "socket-color-identity/3"
ASSIGNMENT_SCHEMA = "episode-color-assignment/1"

# The spelling of these keys is part of the collection metadata contract.
COLOR_KEYS = (
    "red",
    "orange",
    "yellow",
    "green",
    "black",
    "blue",
    "purple",
)
COLOUR_KEYS = COLOR_KEYS

# Display sRGB values. MJCF/UE PBR material inputs are linear and are derived
# by _linear_rgba below. Black is intentional: it replaced cyan because it
# is easier to distinguish in natural-language instructions.
COLOR_PALETTE = {
    "red": {"srgb_hex": "#D73027", "color_name": "red", "color_name_zh": "红色"},
    "orange": {"srgb_hex": "#E67E22", "color_name": "orange", "color_name_zh": "橙色"},
    "yellow": {"srgb_hex": "#F1C40F", "color_name": "yellow", "color_name_zh": "黄色"},
    "green": {"srgb_hex": "#2EAD60", "color_name": "green", "color_name_zh": "绿色"},
    "black": {"srgb_hex": "#202124", "color_name": "black", "color_name_zh": "黑色"},
    "blue": {"srgb_hex": "#2878B5", "color_name": "blue", "color_name_zh": "蓝色"},
    "purple": {"srgb_hex": "#7A3E9D", "color_name": "purple", "color_name_zh": "紫色"},
}
NEUTRAL_COLOR_KEY = "neutral"
NEUTRAL_COLOR = {"srgb_hex": "#A6A8AB", "color_name": "neutral", "color_name_zh": "中性灰"}
# British spelling and the old public name are kept for callers outside this
# package. SOCKET_SRGB_HEX now describes the seven colour palette, not a
# fixed socket-to-colour assignment.
COLOUR_PALETTE = COLOR_PALETTE
SOCKET_SRGB_HEX = {key: value["srgb_hex"] for key, value in COLOR_PALETTE.items()}

TARGET_SOCKET_IDS = (
    "aviation_insert_40p1",
    "aviation_insert_40p5",
    "guide_insert_40p1",
    "guide_insert_40p5",
    "cylindrical_insert_40p1",
    "cylindrical_insert_40p5",
    "bolt_assembly_hole",
)
HOLE_IDS = TARGET_SOCKET_IDS
SOURCE_IDS = ("aviation_plug", "guide_plug", "cylindrical_plug", "bolt")

# Storage/pickup openings are not instruction targets. Episode appearances
# keep them neutral so only the seven target holes carry instruction colours.
STORAGE_SOCKET_ID = "bolt_storage_hole"

_CANONICAL_HOLE_COLORS = {
    "aviation_insert_40p1": "yellow",
    "aviation_insert_40p5": "orange",
    "guide_insert_40p1": "green",
    "guide_insert_40p5": "blue",
    "cylindrical_insert_40p1": "black",
    "cylindrical_insert_40p5": "purple",
    "bolt_assembly_hole": "red",
}
_CANONICAL_SOURCE_COLORS = {
    "aviation_plug": "orange",
    "guide_plug": "blue",
    "cylindrical_plug": "purple",
    "bolt": "red",
}

# Compatibility constants used by older code which expected a socket-keyed
# name table. For an episode, the colour name is read from COLOR_PALETTE.
_COLOUR_NAMES = {
    socket: COLOR_PALETTE[color]["color_name"]
    for socket, color in _CANONICAL_HOLE_COLORS.items()
}


def _linear_rgba(hex_colour: str) -> tuple[float, float, float, float]:
    """Convert display sRGB hex to the linear RGBA expected by MJCF."""
    srgb = tuple(int(hex_colour[index:index + 2], 16) / 255 for index in (1, 3, 5))
    return (*(value / 12.92 if value <= .04045 else ((value + .055) / 1.055) ** 2.4
              for value in srgb), 1.)


def _color_metadata(color_key: str, *, material_name: str | None = None) -> dict:
    color = NEUTRAL_COLOR if color_key == NEUTRAL_COLOR_KEY else COLOR_PALETTE[color_key]
    rgba = _linear_rgba(color["srgb_hex"])
    return {
        "color_key": color_key,
        "colour_key": color_key,
        "color_name": color["color_name"],
        "colour_name": color["color_name"],
        "color_name_en": color["color_name"],
        "color_name_zh": color["color_name_zh"],
        "prompt_name": color["color_name_zh"],
        "prompt_color_name": color["color_name_zh"],
        "label": color["color_name_zh"],
        "srgb_hex": color["srgb_hex"],
        "linear_rgba": list(rgba),
        **({"material_name": material_name} if material_name is not None else {}),
    }


def _assignment_digest(holes: dict, sources: dict) -> str:
    payload = {"holes": holes, "sources": sources}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _palette_snapshot() -> dict:
    return {key: dict(value) for key, value in COLOR_PALETTE.items()}


def _permutation(rng, values):
    """Return a list from NumPy or Python-style RNGs without global state."""
    if hasattr(rng, "permutation"):
        return [values[int(index)] for index in rng.permutation(len(values))]
    if hasattr(rng, "sample"):
        return list(rng.sample(list(values), len(values)))
    raise TypeError("rng must provide permutation() or sample()")


def sample_color_assignment(seed=None, rng=None, *, hole_rng=None, source_rng=None) -> dict:
    """Sample a reproducible episode colour assignment.

    ``rng`` is retained for callers that use one stream. Collection uses the
    two named streams ``hole-colors-v1`` and ``source-colors-v1`` via
    ``hole_rng`` and ``source_rng`` so adding a source colour cannot perturb
    hole colours. ``seed`` is a convenience for standalone tools.
    """
    if seed is not None and any(value is not None for value in (rng, hole_rng, source_rng)):
        raise ValueError("pass seed or explicit rng streams, not both")
    if rng is not None and (hole_rng is not None or source_rng is not None):
        raise ValueError("pass rng or hole_rng/source_rng, not both")
    if hole_rng is None:
        if rng is not None:
            hole_rng = rng
            source_rng = rng
        else:
            try:
                import numpy as np
            except ImportError as error:  # pragma: no cover - project already depends on NumPy
                raise RuntimeError("colour sampling requires NumPy or an explicit rng") from error
            if seed is None:
                hole_rng = source_rng = np.random.default_rng()
            else:
                hole_rng = np.random.default_rng(int(seed))
                source_rng = np.random.default_rng(int(seed) ^ 0x9E3779B9)
    if source_rng is None:
        source_rng = hole_rng
    hole_order = _permutation(hole_rng, COLOR_KEYS)
    source_order = _permutation(source_rng, COLOR_KEYS)
    holes = dict(zip(TARGET_SOCKET_IDS, hole_order))
    sources = dict(zip(SOURCE_IDS, source_order[:len(SOURCE_IDS)]))
    return {
        "schema": ASSIGNMENT_SCHEMA,
        "seed": None if seed is None else int(seed),
        "randomized": True,
        "palette_schema": SCHEMA,
        "palette": _palette_snapshot(),
        "hole_colors": holes,
        "holes": dict(holes),
        "target_hole_colors": dict(holes),
        "source_colors": sources,
        "source_part_colors": dict(sources),
        "sources": dict(sources),
        "bolt_storage_color": NEUTRAL_COLOR_KEY,
        "assignment_sha256": _assignment_digest(holes, sources),
        "stream": "socket-colors-v1",
    }


def _canonical_assignment() -> dict:
    holes = dict(_CANONICAL_HOLE_COLORS)
    sources = dict(_CANONICAL_SOURCE_COLORS)
    return {
        "schema": ASSIGNMENT_SCHEMA,
        "seed": None,
        "randomized": False,
        "palette_schema": SCHEMA,
        "palette": _palette_snapshot(),
        "hole_colors": holes,
        "holes": dict(holes),
        "target_hole_colors": dict(holes),
        "source_colors": sources,
        "source_part_colors": dict(sources),
        "sources": dict(sources),
        "bolt_storage_color": sources["bolt"],
        "assignment_sha256": _assignment_digest(holes, sources),
        "stream": "canonical-layout-v1",
    }


def _normalise_color(value) -> str:
    if isinstance(value, dict):
        value = value.get("color_key", value.get("colour_key", value.get("color_name", value.get("colour_name", value.get("color_name_zh", value.get("colour_name_zh", value.get("srgb_hex")))))))
    if not isinstance(value, str):
        raise ValueError(f"unknown socket colour value: {value!r}")
    value = value.strip().lower()
    for key, data in COLOR_PALETTE.items():
        if value in {key, data["color_name"], data["color_name_zh"], data["srgb_hex"].lower()}:
            return key
    raise ValueError(f"unknown socket colour value: {value!r}")


def _normalise_assignment(assignment) -> dict:
    if assignment is None:
        return _canonical_assignment()
    if not isinstance(assignment, dict):
        raise TypeError("colour assignment must be a mapping")
    holes = assignment.get("hole_colors", assignment.get("holes", assignment.get("target_hole_colors")))
    sources = assignment.get("source_colors", assignment.get("sources", assignment.get("source_part_colors")))
    # A direct seven-key mapping is convenient for small tools and remains
    # unambiguous because all target IDs are namespaced by their role.
    if holes is None and set(assignment).issuperset(TARGET_SOCKET_IDS):
        holes = {key: assignment[key] for key in TARGET_SOCKET_IDS}
    if sources is None:
        sources = _CANONICAL_SOURCE_COLORS
    if not isinstance(holes, dict) or not isinstance(sources, dict):
        raise ValueError("colour assignment requires hole_colors and source_colors mappings")
    holes = {key: _normalise_color(holes.get(key)) for key in TARGET_SOCKET_IDS}
    source_values = {}
    for key in SOURCE_IDS:
        value = sources.get(key)
        # Compact manifests often use ``aviation`` instead of the stable body
        # ID ``aviation_plug``. Accept both while emitting the stable form.
        if value is None and key.endswith("_plug"):
            value = sources.get(key[:-5])
        if value is None and key == "bolt":
            value = sources.get("bolt_source")
        source_values[key] = _normalise_color(value)
    sources = source_values
    if len(set(holes.values())) != len(TARGET_SOCKET_IDS):
        raise ValueError("target hole colours must be a permutation of the seven colours")
    if len(set(sources.values())) != len(SOURCE_IDS):
        raise ValueError("source plug/bolt colours must be unique within the source group")
    digest = _assignment_digest(holes, sources)
    if (assignment.get("assignment_sha256") is not None
            and assignment["assignment_sha256"] != digest):
        raise ValueError("colour assignment SHA256 does not match its hole/source colours")
    # Old assignments followed the loose bolt colour at its storage seat.
    # Accept that historical metadata, but migrate the non-target seat to the
    # neutral appearance contract. It is outside the hole/source digest.
    storage_color = assignment.get("bolt_storage_color", NEUTRAL_COLOR_KEY)
    if storage_color != NEUTRAL_COLOR_KEY and _normalise_color(storage_color) != sources["bolt"]:
        raise ValueError("bolt storage colour must be neutral or match the legacy loose bolt colour")
    result = dict(assignment)
    result.update(
        schema=assignment.get("schema", ASSIGNMENT_SCHEMA),
        palette_schema=SCHEMA,
        palette=assignment.get("palette") or _palette_snapshot(),
        hole_colors=holes,
        holes=dict(holes),
        target_hole_colors=dict(holes),
        source_colors=sources,
        sources=dict(sources),
        source_part_colors=dict(sources),
        bolt_storage_color=NEUTRAL_COLOR_KEY,
        assignment_sha256=digest,
    )
    return result


def normalize_color_assignment(assignment) -> dict:
    """Validate and canonicalize a saved episode assignment.

    Collection metadata readers can use this without mutating an MJCF tree.
    The returned mappings contain only canonical colour keys, even when the
    input used a hex value or a Chinese prompt label. A saved digest must match
    those colours; changing a valid permutation must not silently reseal it.
    """
    return _normalise_assignment(assignment)


validate_color_assignment = normalize_color_assignment


def _geom(root: ET.Element, body_name: str, geom_name: str):
    bodies = root.findall(f'.//body[@name="{body_name}"]')
    geoms = bodies[0].findall(f'geom[@name="{geom_name}"]') if len(bodies) == 1 else []
    if len(geoms) != 1:
        raise ValueError(f"Socket colours require exactly one {body_name}/{geom_name}")
    geom = geoms[0]
    if (geom.get("group") != "2" or geom.get("contype") != "0"
            or geom.get("conaffinity") != "0"):
        raise ValueError(f"Socket colours may only change noncontact visible geom {geom_name}")
    return geom


def _known_material_attributes(element: ET.Element) -> bool:
    """Accept an existing material emitted by this module or its old palette."""
    try:
        rgba = tuple(float(part) for part in element.get("rgba", "").split())
    except ValueError:
        return False
    if len(rgba) != 4:
        return False
    known = [_linear_rgba(data["srgb_hex"]) for data in (*COLOR_PALETTE.values(), NEUTRAL_COLOR)]
    # Legacy six-colour values are included so an existing layout can migrate
    # to an episode assignment without touching its physical model.
    known.extend(_linear_rgba(value) for value in (
        "#F28E2B", "#2667E8", "#C238C3", "#E5CF23", "#2DB66D", "#1BCAD5"))
    return any(all(abs(a - b) < 2e-6 for a, b in zip(rgba, candidate)) for candidate in known)


def apply_socket_colors(root: ET.Element, assignment=None) -> dict:
    """Apply an episode assignment to visible installation geoms.

    Missing bolt bodies are tolerated only for legacy six-hole test fixtures;
    the delivered independent layout contains all seven targets and the bolt
    pair, which are then checked strictly. The function validates the whole
    selection before mutating any geometry or asset.
    """
    explicit_assignment = assignment is not None
    asset = root.find("asset")
    if asset is None:
        raise ValueError("Socket colours require an MJCF asset section")
    assignment = _normalise_assignment(assignment)

    bindings = []
    for kind in ("aviation", "guide", "cylindrical"):
        matched_socket = f"{kind}_insert_40p5"
        source_id = kind + "_plug"
        bindings.extend([
            (source_id, source_id + "_geom", matched_socket, "plug", assignment["source_colors"][source_id], None),
            ("layout_" + kind + "_pickup", "layout_" + kind + "_pickup_visual",
             matched_socket, "pickup", NEUTRAL_COLOR_KEY if explicit_assignment else assignment["source_colors"][source_id], None),
        ])
        for size, diameter in (("40p1", 40.1), ("40p5", 40.5)):
            socket = f"{kind}_insert_{size}"
            bindings.append(("layout_" + socket, "layout_" + socket + "_visual",
                             socket, "insertion_socket", assignment["hole_colors"][socket], diameter))

    # The bolt and storage opening are present in the real layout. Keep all
    # three together so a partially generated model cannot silently lose its
    # visual identity.
    bolt_names = (("bolt", "bolt_geom", "bolt_assembly_hole", "bolt", assignment["source_colors"]["bolt"], None),
                  ("layout_bolt_storage_hole", "layout_bolt_storage_hole_visual", STORAGE_SOCKET_ID,
                   "storage_socket", assignment["bolt_storage_color"], None),
                  ("layout_bolt_assembly_hole", "layout_bolt_assembly_hole_visual", "bolt_assembly_hole",
                   "insertion_socket", assignment["hole_colors"]["bolt_assembly_hole"], None))
    present = [root.find(f'.//body[@name="{body}"]') is not None for body, *_ in bolt_names]
    if any(present) and not all(present):
        raise ValueError("Socket colours require the complete bolt/storage/assembly binding")
    if explicit_assignment and not all(present):
        raise ValueError("An episode colour assignment requires the bolt/storage/assembly binding")
    if all(present):
        bindings.extend(bolt_names)

    selected = []
    for body_name, geom_name, socket, role, color_key, diameter in bindings:
        selected.append((_geom(root, body_name, geom_name), body_name, socket, role, color_key, diameter))

    # Material names are colour-keyed so source and target may deliberately
    # receive different colours in one episode. Validate legacy names before
    # adding anything, preserving the no-partial-mutation guarantee.
    material_by_name = {}
    for color_key in sorted({entry[4] for entry in selected}):
        name = "socket_identity_color_" + color_key
        existing = asset.findall(f'material[@name="{name}"]')
        if len(existing) > 1:
            raise ValueError(f"Conflicting socket identity material {name}")
        material_by_name[color_key] = (existing[0] if existing else None, name)
    for material in asset.findall("material"):
        name = material.get("name", "")
        if name.startswith("socket_identity_") and not name.startswith("socket_identity_color_"):
            if not _known_material_attributes(material):
                raise ValueError(f"Conflicting socket identity material {name}")

    attributes_by_color = {}
    for color_key, (existing, name) in material_by_name.items():
        color = NEUTRAL_COLOR if color_key == NEUTRAL_COLOR_KEY else COLOR_PALETTE[color_key]
        rgba = _linear_rgba(color["srgb_hex"])
        attributes_by_color[color_key] = dict(
            name=name, rgba=" ".join(f"{v:.12g}" for v in rgba),
            specular="0.25", shininess="0.08", reflectance="0", emission="0",
        )
        if existing is not None and existing.attrib != attributes_by_color[color_key]:
            if not _known_material_attributes(existing):
                raise ValueError(f"Conflicting socket identity material {name}")

    # All validation is complete. Only material and visible-geom appearance
    # fields are changed from here onward.
    for color_key, (existing, _) in material_by_name.items():
        attributes = attributes_by_color[color_key]
        if existing is None:
            ET.SubElement(asset, "material", **attributes)
        else:
            existing.attrib.clear()
            existing.attrib.update(attributes)

    identities = {}
    # ``palette`` preserves the historical socket-ID keyed view and contains
    # the seven target holes only. Source and storage appearances are exposed
    # separately so a randomized source can never overwrite a target entry.
    palette = {}
    source_palette = {}
    storage_palette = {}
    for geom, body_name, socket, role, color_key, diameter in selected:
        attributes = attributes_by_color[color_key]
        geom.set("material", attributes["name"])
        geom.set("rgba", attributes["rgba"])
        metadata = _color_metadata(color_key, material_name=attributes["name"])
        # Target palette entries are keyed by stable socket ID. A source may
        # share that ID (the legacy matching colour), but a randomized source
        # must not overwrite the target's metadata.
        if role == "insertion_socket":
            palette[socket] = metadata
        identities[body_name] = dict(
            geometry_name=geom.get("name"), role=role,
            # colour_identity remains the stable socket identity for existing
            # consumers; color_key is the episode-random appearance identity.
            colour_identity=socket, color_identity=socket,
            color_key=color_key, colour_key=color_key,
            colour_name=metadata["color_name"], color_name=metadata["color_name"],
            color_name_zh=metadata["color_name_zh"],
            diameter_mm=diameter, matched_insertion_site=socket,
            material_name=attributes["name"], linear_rgba=metadata["linear_rgba"],
        )
        if role in {"plug", "bolt"}:
            source_palette[body_name] = metadata
        elif role == "storage_socket":
            storage_palette[socket] = metadata

    colors = {key: _color_metadata(key) for key in COLOR_KEYS}
    return dict(
        schema=SCHEMA, assignment_schema=assignment["schema"],
        seed=assignment.get("seed"), randomized=assignment.get("randomized", False),
        stream=assignment.get("stream"), assignment_sha256=assignment["assignment_sha256"],
        material_colour_space="linear_rgb", display_colour_space="srgb",
        scripted_target_diameter_mm=40.5, palette=palette, colors=colors,
        source_palette=source_palette, storage_palette=storage_palette,
        color_palette=colors, hole_colors=dict(assignment["hole_colors"]),
        neutral_appearance=_color_metadata(NEUTRAL_COLOR_KEY),
        target_hole_colors=dict(assignment["hole_colors"]),
        source_colors=dict(assignment["source_colors"]),
        bolt_storage_color=assignment["bolt_storage_color"],
        identities=identities,
        target_socket_ids=list(TARGET_SOCKET_IDS), source_ids=list(SOURCE_IDS),
        scope="Existing visible plug, bolt, storage and mounting surfaces only; physical mating geometry unchanged",
    )


__all__ = [
    "ASSIGNMENT_SCHEMA", "COLOR_KEYS", "COLOR_PALETTE", "COLOUR_PALETTE",
    "HOLE_IDS", "SOURCE_IDS", "SOCKET_SRGB_HEX", "TARGET_SOCKET_IDS",
    "apply_socket_colors", "normalize_color_assignment", "sample_color_assignment",
    "validate_color_assignment",
]
