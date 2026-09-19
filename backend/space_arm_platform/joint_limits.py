"""Resolve scalar joint limits from MJCF without loading meshes or a second simulator.

Handles compiler units, inherited default classes, childclass and includes.
Unknown/malformed limits fail closed; continuous hinges have infinite bounds.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointLimits:
    names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    limited: tuple[bool, ...]


def load_joint_limits(path: str | Path, names: tuple[str, ...]) -> JointLimits:
    def load(file, stack=()):
        file = Path(file).resolve()
        if file in stack:
            raise ValueError("recursive MJCF include")
        root = ET.parse(file).getroot()
        def expand(parent):
            for child in list(parent):
                if child.tag == "include":
                    included = load(file.parent / child.attrib["file"], (*stack, file))
                    index = list(parent).index(child)
                    parent.remove(child)
                    for offset, node in enumerate(list(included)):
                        parent.insert(index + offset, node)
                else:
                    expand(child)
        expand(root)
        return root

    root = load(path)
    compiler = root.find("compiler")
    settings = compiler.attrib if compiler is not None else {}
    radians = settings.get("angle", "degree") == "radian"
    autolimits = settings.get("autolimits", "true") == "true"
    defaults = {"": {}}
    def read_default(node, inherited):
        values = dict(inherited)
        joint = node.find("joint")
        if joint is not None:
            values.update(joint.attrib)
        defaults[node.get("class", "")] = values
        for child in node.findall("default"):
            read_default(child, values)
    for node in root.findall("default"):
        read_default(node, defaults[""])
    found = {}
    def walk(node, inherited_class=""):
        selected = node.get("childclass", inherited_class)
        if selected not in defaults:
            raise ValueError(f"unknown MJCF default class: {selected}")
        for child in node:
            if child.tag == "joint" and child.get("name") in names:
                name = child.get("name")
                if name in found:
                    raise ValueError(f"duplicate joint: {name}")
                cls = child.get("class", selected)
                if cls not in defaults:
                    raise ValueError(f"unknown joint class: {cls}")
                attrs = {**defaults[cls], **child.attrib}
                kind = attrs.get("type", "hinge")
                if kind not in ("hinge", "slide"):
                    raise ValueError(f"not a scalar arm joint: {name}")
                flag = attrs.get("limited", "auto")
                if flag not in ("auto", "true", "false"):
                    raise ValueError(f"invalid limited flag: {name}")
                if flag == "auto" and "range" in attrs and not autolimits:
                    raise ValueError(f"explicit limited flag required: {name}")
                limited = flag == "true" or (flag == "auto" and autolimits and "range" in attrs)
                low, high = -math.inf, math.inf
                if limited:
                    values = [float(v) for v in attrs.get("range", "").split()]
                    if len(values) != 2 or not all(math.isfinite(v) for v in values) or values[0] >= values[1]:
                        raise ValueError(f"invalid joint range: {name}")
                    low, high = values
                    if kind == "hinge" and not radians:
                        low, high = math.radians(low), math.radians(high)
                found[name] = (low, high, limited)
            elif child.tag in ("body", "frame", "worldbody"):
                walk(child, selected)
    walk(root)
    missing = set(names) - found.keys()
    if missing:
        raise ValueError(f"missing joints: {sorted(missing)}")
    return JointLimits(names, tuple(found[n][0] for n in names),
                       tuple(found[n][1] for n in names), tuple(found[n][2] for n in names))
