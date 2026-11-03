"""Explicit uniform satellite scaling; installed task hardware stays unchanged."""
from copy import deepcopy
import math


def scale_satellite(root, body_name, factor, *, mass_rule="constant_density"):
    """Scale a satellite subtree before mounting task hardware.

    Returns a new XML tree and provenance. Shared meshes are cloned. Caller must
    revalidate contacts, mounting, control and dynamics; no acceptance is implied.
    Joint damping/friction are retained as engineering assumptions, not rescaled
    actuator specifications. No actuator or mechanism may already be mounted.
    """
    if isinstance(factor, bool) or not math.isfinite(factor) or not 1 <= factor <= 10:
        raise ValueError("Satellite scale must be finite and in [1,10]")
    if mass_rule not in ("constant_density", "fixed_mass"):
        raise ValueError("Choose constant_density or fixed_mass")
    result = deepcopy(root)
    body = result.find(f'.//body[@name="{body_name}"]')
    if body is None:
        raise ValueError(f"Missing satellite body {body_name}")
    bodies = {b.get("name") for b in body.iter("body")}
    if bodies.intersection({"task_box_base", "drawer_base", "link1"}):
        raise ValueError("Scale the bare satellite before mounting arm/task modules")
    joint_names = {j.get("name") for j in body.iter("joint")}
    actuator = result.find("actuator")
    if actuator is not None and any(n.get("joint") in joint_names for n in actuator):
        raise ValueError("Scaling an actuated satellite requires an actuator-specific rule")
    mass_factor = factor**3 if mass_rule == "constant_density" else 1.
    inertia_factor = mass_factor * factor**2

    def multiply(node, attribute, value):
        if attribute in node.attrib:
            node.set(attribute, " ".join(format(float(v)*value, ".17g") for v in node.get(attribute).split()))

    assets = result.find("asset")
    meshes = {} if assets is None else {n.get("name"): n for n in assets.findall("mesh")}
    clones = {}
    for node in body.iter():
        if node is not body:
            multiply(node, "pos", factor)
        if node.tag == "inertial":
            multiply(node, "mass", mass_factor)
            for key in ("diaginertia", "fullinertia"):
                multiply(node, key, inertia_factor)
        elif node.tag in ("geom", "site"):
            for key in ("size", "fromto", "margin", "gap"):
                multiply(node, key, factor)
            multiply(node, "mass", mass_factor)
            if node.tag == "geom" and mass_rule == "fixed_mass" and "mass" not in node.attrib:
                # Explicit inertias override geom density; otherwise make the assumption explicit.
                if "class" in node.attrib or "density" not in node.attrib:
                    parents = {c: p for p in body.iter() for c in p}
                    if parents[node].find("inertial") is None:
                        raise ValueError("Fixed-mass scaling needs explicit inertia or geom density")
                multiply(node, "density", factor**-3)
            name = node.get("mesh")
            if name:
                if name not in clones:
                    mesh = deepcopy(meshes[name])
                    if mesh.get("refpos") or mesh.get("refquat"):
                        raise ValueError("Mesh reference transforms require explicit conversion")
                    clone = f"{body_name}_scale_{name}"
                    if clone in meshes:
                        raise ValueError("Scaled mesh name collision")
                    mesh.set("name", clone)
                    mesh.set("scale", " ".join(format(float(v)*factor, ".17g") for v in mesh.get("scale", "1 1 1").split()))
                    assets.append(mesh)
                    clones[name] = clone
                node.set("mesh", clones[name])
        elif node.tag == "joint" and node.get("type") == "slide":
            for key in ("range", "ref", "springref"):
                multiply(node, key, factor)
    return result, dict(body=body_name, uniform_scale=factor, mass_rule=mass_rule,
                        mass_factor=mass_factor, inertia_factor=inertia_factor,
                        cloned_meshes=clones, task_hardware_scaled=False,
                        validation="requires_mount_collision_and_native_dynamics_validation")
