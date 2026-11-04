"""Episode-scoped physical and visual changes to the released target only."""
import xml.etree.ElementTree as ET

from pydantic import BaseModel, ConfigDict, model_validator
from simulation.socket_colors import COLOR_PALETTE, _linear_rgba


class FloatingTargetConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    mass_scale_range: tuple[float, float] = (1., 1.)
    friction_scale_range: tuple[float, float] = (1., 1.)
    color_choices: tuple[str, ...] = ()

    @model_validator(mode="after")
    def bounds(self):
        for interval in (self.mass_scale_range, self.friction_scale_range):
            if not .5 <= interval[0] <= interval[1] <= 1.5:
                raise ValueError("Target scales must be ordered within [0.5, 1.5]")
        if len(set(self.color_choices)) != len(self.color_choices) or any(c not in COLOR_PALETTE for c in self.color_choices):
            raise ValueError("Target colors must be unique keys from COLOR_PALETTE")
        return self


def _defaults(root, tag):
    classes = {"": {}}
    def visit(node, inherited):
        values = dict(inherited)
        geom = node.find(tag)
        if geom is not None:
            values.update(geom.attrib)
        classes[node.get("class", "")] = values
        for child in node.findall("default"):
            visit(child, values)
    for node in root.findall("default"):
        visit(node, {})
    return classes


def apply_target_variations(root, scene, body_name, *, options_override=None, instance_key=None):
    """Always receives a fresh source tree after target release/copy preparation."""
    from .randomization import generator, options_for_scene
    options = FloatingTargetConfig.model_validate(scene.get("floating_target", {}) if options_override is None else options_override)
    if options_for_scene(scene).dynamics.enabled:
        raise ValueError("Floating target variants cannot be combined with global dynamics scaling")
    cfg = scene["collection_config"]
    seed = int(scene.get("seed", 0))
    mass = float(generator(seed, "floating-target-mass-v1").uniform(*options.mass_scale_range))
    friction = float(generator(seed, "floating-target-friction-v1").uniform(*options.friction_scale_range))
    color = str(generator(seed, "floating-target-color-v1").choice(options.color_choices)) if options.color_choices else None
    body = root.find(f'.//body[@name="{body_name}"]')
    if body is None or body.find("body") is not None:
        raise ValueError("Target variations require a root-only released body")
    defaults = _defaults(root, "geom")
    parents = {child: parent for parent in root.iter() for child in parent}
    childclass = ""
    cursor = body
    while cursor is not None:
        if "childclass" in cursor.attrib:
            childclass = cursor.get("childclass")
            break
        cursor = parents.get(cursor)
    inertial = body.find("inertial")
    compiler = root.find("compiler")
    from_geom = compiler.get("inertiafromgeom", "auto") if compiler is not None else "auto"
    use_inertial = inertial is not None and from_geom != "true"
    def scaled(values, factor):
        return " ".join(format(float(v)*factor, ".17g") for v in values.split())
    if mass != 1 and use_inertial:
        for key in ("mass", "diaginertia", "fullinertia"):
            if key in inertial.attrib:
                inertial.set(key, scaled(inertial.get(key), mass))
    names, visual, physical = set(), [], []
    prefix = "floating_target" if instance_key is None else instance_key
    material_name = f"{prefix}_color_{color}" if color else None
    rgba = list(_linear_rgba(COLOR_PALETTE[color]["srgb_hex"])) if color else None
    for geom in body.findall("geom"):
        cls = geom.get("class", childclass)
        if cls not in defaults:
            raise ValueError(f"Unknown target geom class {cls}")
        effective = {**defaults[cls], **geom.attrib}
        names.add(geom.get("name"))
        if mass != 1 and not use_inertial:
            key = "mass" if "mass" in effective else "density"
            geom.set(key, scaled(effective.get(key, "1000"), mass))
        if friction != 1:
            geom.set("friction", scaled(effective.get("friction", "1 0.005 0.0001"), friction))
        physical.append(dict(name=geom.get("name"), mass=geom.get("mass"), density=geom.get("density"),
                             friction=geom.get("friction", effective.get("friction", "1 0.005 0.0001")),
                             priority=effective.get("priority", "0")))
        if color and (effective.get("group", "0") == "2"
                      and effective.get("contype", "1") == "0"
                      and effective.get("conaffinity", "1") == "0"
                      and float(effective.get("rgba", "0.5 0.5 0.5 1").split()[-1]) > 0):
            geom.set("material", material_name)
            geom.set("rgba", " ".join(format(v, ".12g") for v in rgba))
            visual.append(geom.get("name"))
    pairs = []
    pair_defaults = _defaults(root, "pair")
    for pair in root.findall("./contact/pair"):
        if names.intersection((pair.get("geom1"), pair.get("geom2"))):
            if friction != 1:
                effective = {**pair_defaults[pair.get("class", "")], **pair.attrib}
                pair.set("friction", scaled(effective.get("friction", "1 1 0.005 0.0001 0.0001"), friction))
            pairs.append(dict(pair.attrib))
    if color:
        if not visual:
            raise ValueError("Target has no eligible visible geoms for coloring")
        if root.find(f'./asset/material[@name="{material_name}"]') is not None:
            raise ValueError("Target color material name already exists")
        ET.SubElement(root.find("asset"), "material", name=material_name,
                      rgba=" ".join(format(v, ".12g") for v in rgba),
                      specular="0.25", shininess="0.08", reflectance="0", emission="0")
    realized = dict(schema="floating-target-variation/1", seed=seed, target_body=body_name,
                    options=options.model_dump(mode="json"), mass_scale=mass, friction_scale=friction,
                    inertia_source="explicit_inertial" if use_inertial else "geom",
                    inertial=dict(inertial.attrib) if inertial is not None else None,
                    geoms=physical, contact_pairs=pairs, color=color, linear_rgba=rgba,
                    material_name=material_name, colored_geoms=visual,
                    friction_mixing="unchanged MuJoCo priority/max rules; explicit pairs override")
    saved = cfg.get("floating_target_variation")
    if saved is not None and saved != realized:
        raise ValueError("Saved floating target variation does not match source/configuration")
    cfg["floating_target_variation"] = realized
    return realized
