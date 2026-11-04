"""Reduce compilation work in the local backend's state-publication MJScene.

The separate local stepper loads the original MJCF and owns all physics. This
copy only supplies body/joint/site messages and visible geometry to the bridge.
Keep geom count and order: the bridge pairs them with source MJCF metadata by
index. Only explicitly invisible, massless collision meshes become tiny spheres.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_publication_scene(root: ET.Element) -> dict[str, int]:
    """Simplify an in-memory publication copy; never rewrite the source model.

    Be conservative about defaults, plugins, materials and explicit references.
    Included documents are left intact because their references and defaults
    are unavailable here. Mass-bearing visual meshes still determine inertia.
    This copy must never be used as the dynamics model.
    """
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    flag.set("contact", "disable")
    flag.set("constraint", "disable")
    stats = dict(collision_mesh_proxies=0, discarded_mesh_assets=0, rigid_flexcomps=0)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "flexcomp" and child.get("rigid", "false").lower() == "true":
                parent.remove(child)
                stats["rigid_flexcomps"] += 1

    if root.find(".//include") is not None:
        return stats

    defaults = root.findall(".//default/geom")
    default_fields = {key for geom in defaults for key in geom.attrib}
    if {"mesh", "material"} & default_fields or any(g.find("plugin") is not None for g in defaults):
        return stats
    # Preserve any named geometry/mesh referenced by contacts, sensors, plugins,
    # or another XML element. Looking at all attribute values is intentionally
    # broader than MuJoCo's known geom1/geom2/objname reference attributes.
    references = {value for element in root.iter() for key, value in element.attrib.items()
                  if key not in {"name", "mesh", "file"}}
    plugin_meshes = {mesh.get("name", Path(mesh.get("file", "")).stem)
                     for mesh in root.findall("./asset/mesh") if mesh.find("plugin") is not None}
    replaced_meshes = set()
    for geom in root.findall("./worldbody//geom"):
        mesh = geom.get("mesh")
        if (not mesh or mesh in plugin_meshes or geom.get("type") not in {None, "mesh"}
                or geom.get("group") != "3"
                or geom.get("name") in references or len(geom) or geom.get("material")
                or geom.get("fromto")):
            continue
        rgba = geom.get("rgba", "").split()
        if len(rgba) != 4 or float(rgba[3]) != 0.0:
            continue
        # Explicit mass overrides density, including an inherited density.
        # A default mass may override explicit density, so leave that case alone.
        if "mass" in geom.attrib:
            massless = float(geom.get("mass")) == 0.0
        else:
            massless = ("mass" not in default_fields and geom.get("density") is not None
                        and float(geom.get("density")) == 0.0)
        if not massless:
            continue
        replaced_meshes.add(mesh)
        del geom.attrib["mesh"]
        geom.set("type", "sphere")
        geom.set("size", "0.000001")
        geom.set("mass", "0")
        stats["collision_mesh_proxies"] += 1

    remaining_meshes = {element.get("mesh") for element in root.iter() if element.get("mesh")}
    for asset in root.findall("./asset"):
        for mesh in list(asset):
            if mesh.tag != "mesh":
                continue
            name = mesh.get("name", Path(mesh.get("file", "")).stem)
            if name in replaced_meshes and name not in remaining_meshes and name not in references:
                asset.remove(mesh)
                stats["discarded_mesh_assets"] += 1
    return stats
