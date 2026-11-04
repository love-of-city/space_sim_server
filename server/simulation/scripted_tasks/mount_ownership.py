"""Conservative source/CAD ownership shared by planning and contact auditing."""
import hashlib
import struct
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from simulation.independent_task_box_layout import MODULES

def assign_delivered_shared_mounts(model, root, geom_modules):
    """Identify wholly owned original convex plate pieces conservatively.

    The delivered model predates separate installation bodies. A convex
    collider belongs to a site only when *all* its vertices lie strictly
    inside that site's XY Voronoi cell, as used by the layout partitioner.
    A piece spanning cells stays unknown. This never allows a shared board
    wholesale, and generated layouts use their explicit body ownership.
    """
    geom_modules = dict(geom_modules)
    ownership_assets = {}
    assets = {node.get("name"): node for node in root.findall("./asset/mesh")}
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", ".") if compiler is not None else "."
    for board, sites in ((namespace+board, tuple(namespace+site for site in sites))
                         for namespace in ('', 'service_')
                         for board, sites in (("asm_board2", MODULES[:3]), ("asm_board1", MODULES[3:11]))):
        body = root.find(f'.//body[@name="{board}"]')
        if body is None:
            continue
        points = []
        for name in sites:
            node = body.find(f'site[@name="{name}"]')
            if node is not None:
                points.append((name, tuple(map(float, node.get("pos", "0 0 0").split()))))
        if len(points) != len(sites):
            continue
        for geom in body.findall("geom"):
            if (geom.get("name") in geom_modules
                    or geom.get("pos", "0 0 0") != "0 0 0"
                    or geom.get("quat", "1 0 0 0") != "1 0 0 0"
                    or any(geom.get(tag) for tag in ("euler", "axisangle", "xyaxes", "zaxis"))):
                continue
            asset = assets.get(geom.get("mesh"))
            if asset is None or any(asset.get(tag) for tag in ("refpos", "refquat")):
                continue
            filename = asset.get("file", "")
            if not filename.lower().endswith(".stl"):
                continue
            path = (model.parent / meshdir / filename).resolve()
            data = path.read_bytes()
            if len(data) < 84:
                continue
            triangles = struct.unpack_from("<I", data, 80)[0]
            if len(data) != 84 + triangles * 50:
                continue
            ownership_assets[str(path)] = hashlib.sha256(data).hexdigest()
            scale = tuple(map(float, asset.get("scale", "1 1 1").split()))
            owner = None
            for i in range(triangles):
                for j in range(3):
                    vertex = struct.unpack_from("<fff", data, 84 + i * 50 + 12 + j * 12)
                    x, y = vertex[0] * scale[0], vertex[1] * scale[1]
                    ranked = sorted(((x - p[0]) ** 2 + (y - p[1]) ** 2, name)
                                    for name, p in points)
                    if (ranked[1][0] - ranked[0][0] <= 1e-12
                            or owner is not None and owner != ranked[0][1]):
                        owner = "ambiguous"
                        break
                    owner = ranked[0][1]
                if owner == "ambiguous":
                    break
            if owner not in (None, "ambiguous"):
                geom_modules[geom.get("name", "")] = owner
    return geom_modules, ownership_assets

def source_mount_ownership(model):
    model=Path(model)
    root=ET.parse(model).getroot()
    owners={}
    manifest=model.with_suffix('.layout.json')
    if manifest.is_file():owners.update(json.loads(manifest.read_text(encoding='utf-8')) .get('precision_bore_ownership',{}))
    for body in root.iter('body'):
        name=body.get('name','')
        if name.startswith('layout_'):
            module=name.removeprefix('layout_').removesuffix('_bores')
            if module.endswith(('_pickup','_40p1','_40p5')):
                for geom in body.iter('geom'):owners[geom.get('name','')]=module
    return assign_delivered_shared_mounts(model,root,owners)
