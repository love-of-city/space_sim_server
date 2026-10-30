"""Reduce compilation work in the local backend's state-publication MJScene.

The separate local stepper loads the original MJCF and owns all physics. This
copy only supplies body/joint/site messages and visible geometry to the bridge.
Keep geom count and order: the bridge pairs them with source MJCF metadata by
index. Only explicitly invisible, massless collision meshes become tiny spheres.
"""

from pathlib import Path
import xml.etree.ElementTree as ET


def _proxy_thread_sdfs(root: ET.Element, default_fields: set[str]) -> tuple[set[str], int]:
    """Drop only invisible, massless thread collision SDFs from publication.

    These known SARM plugins supply collision distances only. Their generated
    meshes are expensive to compile even with contact disabled. Keep geom IDs
    and poses, and remove their now-inert explicit contact pairs. A sensor or
    any other consumer referencing the geom keeps the original geometry.
    """
    instances = {i.get("name") for i in root.findall(
        './extension/plugin[@plugin="sarm.sdf.two_start_thread"]/instance')}
    meshes = {m.get("name"): m for m in root.findall("./asset/mesh")}
    contact_elements = {id(e) for c in root.findall("./contact") for e in c.iter()}
    references = {value for e in root.iter() if id(e) not in contact_elements
                  for key, value in e.attrib.items() if key not in {"name", "mesh", "file"}}
    replaced_meshes, replaced_geoms = set(), set()
    for geom in root.findall("./worldbody//geom"):
        if (not geom.get("name") or geom.get("type") != "sdf" or geom.get("group") != "3"
                or geom.get("name") in references or geom.get("material") or geom.get("fromto")):
            continue
        rgba = geom.get("rgba", "").split()
        massless = (float(geom.get("mass")) == 0. if "mass" in geom.attrib else
                    "mass" not in default_fields and geom.get("density") is not None
                    and float(geom.get("density")) == 0.)
        if len(rgba) != 4 or float(rgba[3]) != 0. or not massless:
            continue
        plugin = geom.find("plugin")
        mesh = meshes.get(geom.get("mesh"))
        mesh_plugin = mesh.find("plugin") if mesh is not None else None
        if (len(geom) != 1 or plugin is None or plugin.get("instance") not in instances
                or mesh_plugin is None or mesh_plugin.get("instance") != plugin.get("instance")):
            continue
        replaced_meshes.add(geom.attrib.pop("mesh"))
        replaced_geoms.add(geom.get("name"))
        geom.remove(plugin)
        geom.set("type", "sphere")
        geom.set("size", "0.000001")
        geom.set("mass", "0")
    for contact in root.findall("./contact"):
        for pair in list(contact):
            if pair.tag == "pair" and {pair.get("geom1"), pair.get("geom2")} & replaced_geoms:
                contact.remove(pair)
    return replaced_meshes, len(replaced_geoms)


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
    stats = dict(collision_mesh_proxies=0, collision_sdf_proxies=0,
                 discarded_mesh_assets=0, rigid_flexcomps=0)
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
    replaced_meshes, stats["collision_sdf_proxies"] = _proxy_thread_sdfs(root, default_fields)
    # Preserve any named geometry/mesh referenced by contacts, sensors, plugins,
    # or another XML element. Looking at all attribute values is intentionally
    # broader than MuJoCo's known geom1/geom2/objname reference attributes.
    references = {value for element in root.iter() for key, value in element.attrib.items()
                  if key not in {"name", "mesh", "file"}}
    plugin_meshes = {mesh.get("name", Path(mesh.get("file", "")).stem)
                     for mesh in root.findall("./asset/mesh") if mesh.find("plugin") is not None}
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
    # Remove only unreferenced instances of this known, stateless SDF plugin.
    used_instances = {p.get("instance") for p in root.iter("plugin") if p.get("instance")}
    for extension in root.findall("./extension"):
        for plugin in list(extension):
            if plugin.get("plugin") != "sarm.sdf.two_start_thread":
                continue
            for instance in list(plugin):
                if instance.tag == "instance" and instance.get("name") not in used_instances:
                    plugin.remove(instance)
            if not len(plugin):
                extension.remove(plugin)
    return stats
