"""CAD-derived plug collision geometry shared by asset builds and diagnostics.

No physics runtime or command line side effects are imported here.
"""
from __future__ import annotations
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

SOURCE = Path(__file__).resolve().parents[2] / "model/SARM/platform/sarm_task_box_plugs.xml"

def fmt(values):
    return " ".join(f"{float(v):.12g}" for v in values)

def mesh_read(path):
    vertices, faces = [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append(list(map(float, fields[1:4])))
        elif fields and fields[0] == "f":
            face = [int(v.split("/")[0]) - 1 for v in fields[1:]]
            if len(face) != 3:
                raise ValueError("Expected triangulated CAD")
            faces.append(face)
    return np.array(vertices), np.array(faces, dtype=int)

def mesh_write(path, vertices, faces):
    used, indices = np.unique(faces.reshape(-1), return_inverse=True)
    text = "# Generated diagnostic mesh; production CAD remains unchanged.\n"
    text += "".join("v " + fmt(v) + "\n" for v in vertices[used])
    text += "".join("f " + " ".join(str(int(i) + 1) for i in f) + "\n"
                    for f in indices.reshape(-1, 3))
    Path(path).write_text(text, encoding="utf-8")

def proxy(body, name, kind, position, size, contact, angle=0):
    attrs = {k: v for k, v in contact.attrib.items()
             if k in ("contype", "conaffinity", "condim", "friction", "solref", "solimp", "margin", "gap")}
    attrs.update(name=name, type=kind, pos=fmt(position), size=fmt(size),
                 mass="0", group="3", rgba="0 .7 .3 0")
    if angle:
        attrs["quat"] = fmt([np.cos(angle/2), 0, 0, np.sin(angle/2)])
    return ET.SubElement(body, "geom", attrs)

def circle_groups(points):
    """Separate four disjoint pin perimeter rings, ignoring cap-center vertices."""
    left, groups = set(range(len(points))), []
    while left:
        group = {left.pop()}
        todo = list(group)
        while todo:
            i = todo.pop()
            near = {j for j in left if np.linalg.norm(points[i] - points[j]) < .001}
            left -= near
            group |= near
            todo.extend(near)
        if len(group) >= 6:
            groups.append(points[sorted(group)])
    if len(groups) != 4:
        raise ValueError("CAD pin geometry changed; expected four perimeter rings")
    circles = []
    for points in groups:
        a = np.column_stack([2 * points, np.ones(len(points))])
        sol = np.linalg.lstsq(a, np.sum(points**2, axis=1), rcond=None)[0]
        radius = np.sqrt(sol[2] + np.sum(sol[:2]**2))
        residual = np.max(abs(np.linalg.norm(points-sol[:2], axis=1) - radius))
        if residual > 1e-7 or abs(radius-.0025) > 1e-7:
            raise ValueError("CAD pin circle fit exceeds tolerance")
        circles.append((sol[:2], radius))
    return circles

def simplify_plugs(root, folder, metadata, preserve_pins=False, mesh_source=None):
    result = []
    for item in metadata:
        name = item["name"]
        body = root.find(f"./worldbody/body[@name='{name}']")
        surface = body.find("flexcomp")
        contact = copy.deepcopy(surface.find("contact"))
        vertices, faces = mesh_read(mesh_source(name) if mesh_source else SOURCE.parent / surface.get("file"))
        center = np.asarray(item["bus_center_m"])
        axis = np.array([.28, .275] if name == "guide_plug" else [.37, .22])
        local_xy = axis - center[:2]
        assembly_z = vertices[:, 2] + center[2]
        # Retain the exact CAD grip head (including its gripping recess), not a
        # bounding cylinder that would erase the gripper's contact features.
        head = faces[np.min(assembly_z[faces], axis=1) >= -1e-7]
        if name == "guide_plug":
            # Preserve the protruding CAD key's actual collision triangles.
            # The analytic box did not register against the original flex bore
            # in the rotated-key regression, despite detecting the bore boxes.
            radius = np.linalg.norm(vertices[:, :2] + center[:2] - axis, axis=1)
            key = faces[(np.min(assembly_z[faces], axis=1) < -1e-7)
                        & (np.max(radius[faces], axis=1) > .0203)]
            head = np.concatenate((head, key))
        elif preserve_pins:
            # Keep the complete CAD pin interface, including the underside
            # plate. This preserves faceted pin sides/tips instead of fitting
            # analytic circles at the precision mating interface.
            pins = faces[np.max(assembly_z[faces], axis=1) <= -.04+1e-7]
            head = np.concatenate((head, pins))
        head_path = folder / f"{name}_head.obj"
        mesh_write(head_path, vertices, head)
        surface.set("file", str(head_path.resolve()))
        surface.set("name", f"{name}_proxy_head")
        bottom = -.07 if name == "guide_plug" else -.04
        proxy(body, name+"_proxy_shaft", "cylinder",
              [*local_xy, bottom/2-center[2]], [.02, -bottom/2], contact)
        if name == "guide_plug" or preserve_pins:
            count = 1
        else:
            points = vertices[abs(assembly_z+.055) < 1e-7, :2]
            for i, (xy, radius) in enumerate(circle_groups(points)):
                proxy(body, name+f"_proxy_pin_{i}", "cylinder",
                      [*xy, -.0475-center[2]], [radius, .0075], contact)
            count = 5
        result.append(dict(name=name, primitive_count=count, retained_head_and_key_faces=len(head),
                           original_faces=len(faces), shaft_radius_m=.02,
                           original_pins_retained=bool(preserve_pins and name == "aviation_plug"),
                           note="Analytic cylinders replace faceted circles; not identical contact geometry."))
    return result
