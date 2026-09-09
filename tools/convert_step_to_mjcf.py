"""Local STEP/XCAF -> fixed visual MJCF + explicitly synthetic free-body preview.

No kinematic joints, material densities or engineering mass properties are
inferred from assembly nesting. The source STEP and existing platform models
are never modified. Requires tools/requirements-step-to-mjcf.txt in an isolated
Python environment. See --help. All meshes and XML paths are portable.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np


SCHEMA = "step-mjcf-preview/1"
DEFAULT_RGBA = (0.64, 0.68, 0.73, 1.0)


def numbers(values) -> str:
    return " ".join(f"{float(x):.12g}" for x in values)


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized MuJoCo wxyz quaternion."""
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        raise ValueError("Invalid rotation matrix")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1, atol=1e-7):
        raise ValueError("STEP occurrence has scaling/reflection; cannot express it as a rigid MJCF pose")
    # Symmetric eigensystem is stable near 180 degrees as well as at identity.
    k = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
        [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
        [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)],
    ]) / 3
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def box_inertia(mass: float, size) -> np.ndarray:
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("Preview mass must be finite and positive")
    size = np.asarray(size, dtype=float)
    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
        raise ValueError("Bounding box must have three positive dimensions")
    x, y, z = size
    return mass / 12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])


def mesh_normals(vertices, triangles):
    """Smooth only within a CAD face, never across distinct CAD faces."""
    normals = np.zeros_like(vertices)
    tri = vertices[triangles]
    cross = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
    for i in range(3):
        np.add.at(normals, triangles[:,i], cross)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-20
    normals[valid] /= lengths[valid, None]
    normals[~valid] = (0, 0, 1)
    return normals


def write_obj(path, face_chunks, center):
    """Write a color patch mesh with explicit face-local normals and no textures."""
    vertex_count = sum(len(v) for v, _ in face_chunks)
    triangle_count = sum(len(t) for _, t in face_chunks)
    if vertex_count == 3 and triangle_count == 1:
        # MuJoCo requires >=4 mesh vertices. Split one edge without changing the
        # triangle's surface, winding or bounds (no fake thickness/extra volume).
        vertices, triangles = face_chunks[0]
        a, b, c = triangles[0]
        midpoint = (vertices[b] + vertices[c]) / 2
        face_chunks = [(np.vstack([vertices, midpoint]), np.array([[a,b,3],[a,3,c]], dtype=np.int32))]
        vertex_count, triangle_count = 4, 2
    elif vertex_count < 4:
        raise ValueError(f"{path.name}: fewer than four vertices; unsupported MuJoCo mesh")
    with path.open("w", encoding="ascii", newline="\n") as out:
        out.write("# STEP tessellation, metres, part-centered, normals split at CAD face boundaries\n")
        for vertices, triangles in face_chunks:
            for v in vertices - center:
                out.write("v " + numbers(v) + "\n")
        for vertices, triangles in face_chunks:
            for n in mesh_normals(vertices, triangles):
                out.write("vn " + numbers(n) + "\n")
        offset = 1
        for vertices, triangles in face_chunks:
            for face in triangles + offset:
                out.write("f " + " ".join(f"{i}//{i}" for i in face) + "\n")
            offset += len(vertices)
    return vertex_count, triangle_count


def load_cad(source: Path, deflection_mm: float, angular_deflection_rad: float, *, allow_incomplete=False, diagnostics_dir=None):
    """Return reusable colored meshes and placed leaf occurrences from XCAF."""
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString, TCollection_AsciiString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_Tool
    try:
        from OCP.TDF import TDF_LabelSequence
    except ImportError:
        from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen
    from OCP.Quantity import Quantity_ColorRGBA
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IMeshTools import IMeshTools_Parameters, IMeshTools_MeshAlgoType_Delabella
    from OCP.BRep import BRep_Tool
    from OCP.BRepTools import BRepTools
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy
    from OCP.ShapeFix import ShapeFix_Face
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location

    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    reader.SetSHUOMode(True)
    # Explicit OCCT target unit. STEP's declared units are converted by its reader.
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    print(f"Reading {source.name} ({source.stat().st_size / 1048576:.1f} MiB)...", flush=True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError("Open CASCADE could not read the STEP file")
    document = TDocStd_Document(TCollection_ExtendedString("XCAF"))
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer to XCAF failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(document.Main())

    def label_id(label):
        result = TCollection_AsciiString()
        TDF_Tool.Entry_s(label, result)
        return result.ToCString()

    def label_name(label):
        attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
            return attr.Get().ToExtString()
        return label_id(label)

    def rgba_for(item, is_label=False):
        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            value = Quantity_ColorRGBA()
            found = color_tool.GetColor_s(item, kind, value) if is_label else color_tool.GetColor(item, kind, value)
            if found:
                rgb = value.GetRGB()
                return tuple(round(float(x), 8) for x in (rgb.Red(), rgb.Green(), rgb.Blue(), value.Alpha()))
        return None

    def matrix(location):
        tr = location.Transformation()
        m = np.eye(4)
        for i in range(3):
            for j in range(4):
                m[i, j] = tr.Value(i+1, j+1)
        m[:3, 3] *= 0.001
        return m

    parts = {}
    nodes = []
    occurrences = []
    warnings = []
    # BRep bounds use original assembly locations independently of our traversal.
    exact_bounds = Bnd_Box()
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    for i in range(1, roots.Length()+1):
        BRepBndLib.AddOptimal_s(shape_tool.GetShape_s(roots.Value(i)), exact_bounds, False, False)

    def tessellate(definition):
        key = label_id(definition)
        if key in parts:
            return parts[key]
        shape = shape_tool.GetShape_s(definition)
        if shape.IsNull():
            raise ValueError(f"Null STEP shape {label_name(definition)}")
        mesher = BRepMesh_IncrementalMesh(shape, deflection_mm, False, angular_deflection_rad, True)
        if not mesher.IsDone():
            raise ValueError(f"Tessellation failed for {label_name(definition)}")
        chunks = defaultdict(list)
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        face_count = 0
        degenerate_count = 0
        face_repairs = []
        zero_area_faces = []
        unmeshed_faces = []
        shape_color = rgba_for(shape) or rgba_for(definition, True)
        while explorer.More():
            face = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(explorer.Current())
            face_count += 1
            location = TopLoc_Location()
            original_face = face
            triangulation = BRep_Tool.Triangulation_s(face, location)
            if triangulation is None or triangulation.NbTriangles() == 0:
                # A whole-shape mesher can reject a locally invalid trimming wire.
                # Work on a deep copy so healing does not move adjacent CAD faces.
                stats = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, stats)
                area_mm2 = abs(float(stats.Mass()))
                print(f"    Retry face {face_count} of {label_name(definition)}; area={area_mm2:.12g} mm^2", flush=True)
                if area_mm2 <= 1e-8:
                    zero_area_faces.append(dict(face=face_count, area_mm2=area_mm2))
                    explorer.Next()
                    continue
                for heal, local_tolerance, local_angle in ((False, deflection_mm/2, angular_deflection_rad/2), (True, deflection_mm/2, angular_deflection_rad/2), (True, min(0.01, deflection_mm/10), 0.05)):
                    candidate = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(BRepBuilderAPI_Copy(original_face).Shape())
                    if heal:
                        fixer = ShapeFix_Face(candidate)
                        fixer.Perform()
                        candidate = fixer.Face()
                    BRepTools.Clean_s(candidate)
                    BRepMesh_IncrementalMesh(candidate, local_tolerance, False, local_angle, False)
                    location = TopLoc_Location()
                    retry = BRep_Tool.Triangulation_s(candidate, location)
                    if retry is not None and retry.NbTriangles() > 0:
                        face, triangulation = candidate, retry
                        face_repairs.append(dict(face=face_count, area_mm2=area_mm2,
                                                 method="ShapeFix_Face" if heal else "isolated_face_remesh"))
                        break
                if triangulation is None or triangulation.NbTriangles() == 0:
                    # The default Watson triangulator can fail on valid trimmed
                    # B-spline faces; Delabella can mesh the original, unhealed face.
                    candidate = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(BRepBuilderAPI_Copy(original_face).Shape())
                    BRepTools.Clean_s(candidate)
                    parameters = IMeshTools_Parameters()
                    parameters.Deflection = min(deflection_mm, 0.05)
                    parameters.Angle = min(angular_deflection_rad, 0.1)
                    parameters.MeshAlgo = IMeshTools_MeshAlgoType_Delabella
                    parameters.InParallel = False
                    BRepMesh_IncrementalMesh(candidate, parameters)
                    location = TopLoc_Location()
                    retry = BRep_Tool.Triangulation_s(candidate, location)
                    if retry is not None and retry.NbTriangles() > 0:
                        face, triangulation = candidate, retry
                        face_repairs.append(dict(face=face_count, area_mm2=area_mm2, method="Delabella_original_face"))
                        print(f"    Recovered original face {face_count} with Delabella ({retry.NbTriangles()} triangles)", flush=True)
                if triangulation is None or triangulation.NbTriangles() == 0:
                    detail = dict(face=face_count, area_mm2=area_mm2, source_label=key)
                    if diagnostics_dir is not None:
                        diagnostics_dir.mkdir(exist_ok=True)
                        brep_name = f"part_{len(parts)+1:03d}_face_{face_count:05d}.brep"
                        BRepTools.Write_s(original_face, str(diagnostics_dir / brep_name))
                        detail['diagnostic_brep'] = brep_name
                    if not allow_incomplete:
                        raise ValueError(f"Unmeshed CAD face {face_count} in {label_name(definition)} (area={area_mm2} mm^2) even after isolated remesh/healing; use --allow-incomplete-preview only for a documented visual preview")
                    unmeshed_faces.append(detail)
                    print(f"    INCOMPLETE PREVIEW: omitted face {face_count}, {area_mm2:.9g} mm^2; BREP saved", flush=True)
                    explorer.Next()
                    continue
            vertices = np.array([triangulation.Node(i).Coord() for i in range(1, triangulation.NbNodes()+1)]) * 0.001
            transform = matrix(location)
            vertices = vertices @ transform[:3,:3].T + transform[:3,3]
            triangles = np.array([triangulation.Triangle(i).Get() for i in range(1, triangulation.NbTriangles()+1)], dtype=np.int32) - 1
            if face.Orientation() == TopAbs_REVERSED:
                triangles = triangles[:, [0,2,1]]
            points = vertices[triangles]
            area2 = np.linalg.norm(np.cross(points[:,1]-points[:,0], points[:,2]-points[:,0]), axis=1)
            valid = area2 > 1e-18
            degenerate_count += int((~valid).sum())
            triangles = triangles[valid]
            if len(triangles):
                used = np.unique(triangles)
                remap = np.full(len(vertices), -1, dtype=np.int32)
                remap[used] = np.arange(len(used))
                color = rgba_for(original_face) or shape_color
                chunks[color].append((vertices[used], remap[triangles]))
            explorer.Next()
        all_vertices = [v for group in chunks.values() for v, _ in group]
        if not all_vertices:
            warnings.append(f"No tessellatable faces in {label_name(definition)}; wire/datum geometry is not rendered")
            center = np.zeros(3)
            vertices = np.empty((0,3))
        else:
            vertices = np.concatenate(all_vertices)
            center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
        part = dict(id=f"part_{len(parts)+1:03d}", source_label=key,
                    name=label_name(definition), center=center, vertices=vertices,
                    chunks=dict(chunks), face_count=face_count,
                    dropped_zero_area_triangles=degenerate_count, face_repairs=face_repairs, zero_area_faces=zero_area_faces, unmeshed_faces=unmeshed_faces)
        parts[key] = part
        if unmeshed_faces:
            warnings.append(f"INCOMPLETE PREVIEW: {part['id']} has {len(unmeshed_faces)} omitted CAD faces, total {sum(f['area_mm2'] for f in unmeshed_faces):.9g} mm^2. See diagnostic BREP files.")
        if face_repairs or zero_area_faces:
            warnings.append(f"{part['id']}: {len(face_repairs)} locally remeshed/healed faces; {len(zero_area_faces)} zero-area faces omitted.")
        print(f"  {part['id']}: {face_count} faces, {sum(len(t) for g in chunks.values() for _,t in g):,} triangles, {len(chunks)} colors", flush=True)
        return part

    def visit(label, parent_matrix, parent_id=None, inherited_color=None, stack=()):
        definition = label
        is_reference = shape_tool.IsReference_s(label)
        if is_reference:
            definition = TDF_Label()
            if not shape_tool.GetReferredShape_s(label, definition):
                raise ValueError("Unresolved STEP instance reference")
        key = label_id(definition)
        if key in stack:
            raise ValueError(f"Cyclic STEP assembly at {label_name(definition)}")
        is_assembly = shape_tool.IsAssembly_s(definition)
        local_matrix = matrix(shape_tool.GetLocation_s(label)) if (is_reference or is_assembly) else np.eye(4)
        world_matrix = parent_matrix @ local_matrix
        quaternion(world_matrix[:3,:3])  # reject unsupported scaling/reflections early
        instance_color = rgba_for(label, True) if is_reference else None
        inherited = instance_color or inherited_color
        node_id = f"instance_{len(nodes)+1:04d}"
        node = dict(id=node_id, parent=parent_id, instance_label=label_id(label),
                    definition_label=key, instance_name=label_name(label),
                    definition_name=label_name(definition), assembly=bool(is_assembly),
                    transform_source_m=world_matrix.tolist())
        nodes.append(node)
        if is_assembly:
            children = TDF_LabelSequence()
            shape_tool.GetComponents_s(definition, children)
            for i in range(1, children.Length()+1):
                visit(children.Value(i), world_matrix, node_id, inherited, stack + (key,))
        else:
            part = tessellate(definition)
            if len(part['vertices']):
                occurrences.append(dict(node=node, part=part, transform=world_matrix, color=inherited))
            else:
                node['omitted_non_surface_geometry'] = True

    for i in range(1, roots.Length()+1):
        visit(roots.Value(i), np.eye(4))
    if not occurrences:
        raise ValueError("No renderable geometry in the STEP document")
    return list(parts.values()), nodes, occurrences, warnings, np.array([exact_bounds.CornerMin().Coord(), exact_bounds.CornerMax().Coord()]) * 0.001


def create_mjcf(meshes, geoms, extent, mass=None):
    free = mass is not None
    root = ET.Element("mujoco", model="ground_validation_satellite" + ("_SYNTHETIC_free_preview" if free else "_visual"))
    root.append(ET.Comment(" CAD geometry only. No inferred moving joints or real mass data. Source axes retained; origin shifted to bounding-box center. "))
    ET.SubElement(root, "compiler", angle="radian", meshdir="meshes", inertiafromgeom="false", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 0", integrator="implicitfast")
    ET.SubElement(root, "statistic", center="0 0 0", extent=f"{max(extent):.12g}")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1600", offheight="1200", azimuth="135", elevation="25")
    ET.SubElement(visual, "quality", offsamples="4")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.6 0.6 0.6", specular="0.2 0.2 0.2")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="preview_background", type="skybox", builtin="gradient", rgb1="0.13 0.17 0.23", rgb2="0.025 0.04 0.07", width="512", height="3072")
    for mesh in meshes:
        ET.SubElement(asset, "mesh", name=mesh['name'], file=mesh['file'], inertia="shell")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="key", directional="true", pos="1 -2 3", dir="-1 2 -3", diffuse="0.7 0.7 0.7", castshadow="false")
    ET.SubElement(world, "light", name="fill", directional="true", pos="-2 1 1", dir="2 -1 -1", diffuse="0.35 0.4 0.5", castshadow="false")
    body = ET.SubElement(world, "body", name="ground_validation_satellite")
    if free:
        body.append(ET.Comment(" SYNTHETIC preview only: total mass chosen by user/default, uniform bounding-box inertia, conservative box collision. NOT engineering data. "))
        ET.SubElement(body, "freejoint", name="satellite_free_preview")
        ET.SubElement(body, "inertial", pos="0 0 0", mass=f"{mass:.12g}", diaginertia=numbers(box_inertia(mass, extent)))
        ET.SubElement(body, "geom", name="preview_collision_bbox", type="box", size=numbers(np.asarray(extent)/2), group="3", mass="0", rgba="0.95 0.3 0.12 0.16", friction="0.8 0.005 0.0001")
    for g in geoms:
        ET.SubElement(body, "geom", name=g['name'], type="mesh", mesh=g['mesh'], pos=numbers(g['position_m']), quat=numbers(g['quaternion_wxyz']), rgba=numbers(g['rgba']), mass="0", contype="0", conaffinity="0", group="2")
    return root


def save_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def validate(output, geoms, meshes, manifest):
    import mujoco
    results = {}
    expected_bounds = np.asarray(manifest['bounds']['centered_m'])
    for filename in ('ground_validation_satellite.xml', 'satellite_free_preview.xml'):
        model = mujoco.MjModel.from_xml_path(str(output / filename))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        visual_ids = np.where(model.geom_group == 2)[0]
        if len(visual_ids) != len(geoms) or model.nmesh != len(meshes):
            raise AssertionError("Geometry lost during MuJoCo compilation")
        # Undo MuJoCo's automatic mesh centering/principal-axis rotation through
        # the compiled geom poses. This independently verifies OBJ + XML poses.
        compiled_low, compiled_high = np.full(3, np.inf), np.full(3, -np.inf)
        maximum_pose_error = 0.0
        for i in visual_ids:
            mid = model.geom_dataid[i]
            start = model.mesh_vertadr[mid]
            n = model.mesh_vertnum[mid]
            world_vertices = np.asarray(model.mesh_vert[start:start+n], dtype=float) @ data.geom_xmat[i].reshape(3,3).T + data.geom_xpos[i]
            low, high = world_vertices.min(axis=0), world_vertices.max(axis=0)
            compiled_low = np.minimum(compiled_low, low)
            compiled_high = np.maximum(compiled_high, high)
            gid = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(i))
            expected = next(g for g in geoms if g['name'] == gid)['bounds_centered_m']
            maximum_pose_error = max(maximum_pose_error, float(np.abs(np.array([low, high]) - expected).max()))
        if maximum_pose_error > 2e-5:
            raise AssertionError(f"Compiled component placement mismatch: {maximum_pose_error} m")
        if np.max(np.abs(np.array([compiled_low, compiled_high]) - expected_bounds)) > 2e-5:
            raise AssertionError("Compiled global bounds do not match tessellation")
        if filename.startswith('satellite_free'):
            if model.nq != 7 or model.nv != 6:
                raise AssertionError("Free preview must have exactly one free joint")
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ground_validation_satellite')
            if not np.isclose(model.body_mass[bid], manifest['physics']['synthetic_preview_mass_kg']):
                raise AssertionError("Unexpected preview mass")
            data.qvel[:] = [0.01, -0.02, 0.03, 0.015, 0.02, -0.01]
            for _ in range(500):
                mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                raise AssertionError("Nonfinite free-body state")
            if any(int(w.number) for w in data.warning):
                raise AssertionError("MuJoCo emitted a simulation warning")
        elif model.njnt != 0:
            raise AssertionError("Visual model should not infer any joints")
        results[filename] = dict(bodies=int(model.nbody), joints=int(model.njnt),
                                dofs=int(model.nv), meshes=int(model.nmesh), geoms=int(model.ngeom),
                                max_component_placement_error_m=maximum_pose_error,
                                compiled_bounds_centered_m=[compiled_low.tolist(), compiled_high.tolist()],
                                smoke_test_seconds=1.0 if model.njnt else 0.0,
                                physics_fidelity_verified=False)
    return results


def render_preview(output, extent):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    model = mujoco.MjModel.from_xml_path(str(output / 'ground_validation_satellite.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    option.geomgroup[4] = 0
    option.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    renderer = mujoco.Renderer(model, height=900, width=1200)
    images = []
    views = [('01_isometric', 135, -25), ('02_opposite', -45, -25), ('03_front', 90, 0), ('04_top', 90, 89)]
    try:
        for name, azimuth, elevation in views:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = 0
            camera.distance = float(np.linalg.norm(extent)) * 1.5
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera=camera, scene_option=option)
            image = Image.fromarray(renderer.render())
            image.save(output / 'preview' / (name + '.png'))
            images.append((name, image))
    finally:
        renderer.close()
    sheet = Image.new('RGB', (1600, 1320), (17, 25, 37))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 26)
        small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 20)
    except OSError:
        font, small = ImageFont.load_default(), ImageFont.load_default()
    draw.text((24, 14), 'STEP -> MJCF  |  Ground-validation satellite', font=font, fill=(225,235,248))
    draw.text((24, 49), 'Original CAD colors / assembly placements. Geometry preview; no inferred joints or true mass.', font=small, fill=(158,179,202))
    for i, (name, image) in enumerate(images):
        x, y = (i%2)*800, 88+(i//2)*600
        sheet.paste(image.resize((800,600), Image.Resampling.LANCZOS), (x,y))
        draw.text((x+18, y+12), name.replace('_',' '), font=small, fill=(224,232,243))
    path = output / 'preview' / 'overview.png'
    sheet.save(path)
    return str(path.name)


def convert(args):
    start = time.perf_counter()
    source, output = args.source.resolve(), args.output.resolve()
    if not source.is_file() or source.suffix.lower() not in ('.stp', '.step'):
        raise ValueError('Source must be an existing STEP/STP file')
    for v in (args.deflection_mm, args.angular_deflection_rad, args.preview_mass_kg):
        if not math.isfinite(v) or v <= 0:
            raise ValueError('Meshing tolerances and preview mass must be positive and finite')
    marker = output / '.step-mjcf-output.json'
    digest = sha256(source)
    if output.exists() and any(output.iterdir()):
        if not marker.is_file() or json.loads(marker.read_text('utf-8')).get('source_sha256') != digest:
            raise ValueError('Refusing to overwrite a nonempty folder not created for this STEP source')
    output.mkdir(parents=True, exist_ok=True)
    marker.write_text(json.dumps(dict(schema=SCHEMA, source_sha256=digest), indent=2), encoding='utf-8')
    (output / 'meshes').mkdir(exist_ok=True)
    (output / 'preview').mkdir(exist_ok=True)
    parts, nodes, occurrences, warnings, exact_bounds = load_cad(source, args.deflection_mm, args.angular_deflection_rad, allow_incomplete=args.allow_incomplete_preview, diagnostics_dir=output/'diagnostics')
    global_low, global_high = np.full(3, np.inf), np.full(3, -np.inf)
    for item in occurrences:
        transform = item['transform']
        world_vertices = item['part']['vertices'] @ transform[:3,:3].T + transform[:3,3]
        global_low = np.minimum(global_low, world_vertices.min(axis=0))
        global_high = np.maximum(global_high, world_vertices.max(axis=0))
    bounds = np.array([global_low, global_high])
    center, extent = bounds.mean(axis=0), global_high-global_low
    error = float(np.abs(bounds-exact_bounds).max())
    # Tight geometry-derived check catches common missing/double assembly transforms.
    if error > max(args.deflection_mm*0.001*4, 0.002):
        raise ValueError(f'Tessellated vs original STEP assembly bounds differ by {error:.6g} m; inspect transforms or omitted geometry')
    mesh_records, part_records, geoms = [], [], []
    for part in parts:
        patches = []
        for i, (color, chunks) in enumerate(part['chunks'].items()):
            mesh_name = f"{part['id']}_color_{i:02d}"
            filename = mesh_name + '.obj'
            v_count, t_count = write_obj(output/'meshes'/filename, chunks, part['center'])
            record = dict(name=mesh_name, file=filename, part_id=part['id'], vertices=v_count, triangles=t_count,
                          face_color_rgba=color, sha256=sha256(output/'meshes'/filename))
            mesh_records.append(record)
            patches.append((record, chunks))
        part['patches'] = patches
        part_records.append(dict(id=part['id'], source_name=part['name'], source_label=part['source_label'],
                                 cad_faces=part['face_count'], local_mesh_center_m=part['center'].tolist(),
                                 dropped_zero_area_triangles=part['dropped_zero_area_triangles'],
                                 face_repairs=part['face_repairs'], zero_area_faces=part['zero_area_faces'], unmeshed_faces=part['unmeshed_faces'],
                                 meshes=[r['name'] for r, _ in patches]))
    for item in occurrences:
        part, transform = item['part'], item['transform']
        position = transform[:3,:3] @ part['center'] + transform[:3,3] - center
        q = quaternion(transform[:3,:3])
        for patch, chunks in part['patches']:
            vertices = np.concatenate([v for v, _ in chunks])
            w = vertices @ transform[:3,:3].T + transform[:3,3] - center
            geoms.append(dict(name=f"{item['node']['id']}_{patch['name']}", mesh=patch['name'],
                              source_instance_id=item['node']['id'], position_m=position.tolist(),
                              quaternion_wxyz=q.tolist(), rgba=item['color'] or patch['face_color_rgba'] or DEFAULT_RGBA,
                              bounds_centered_m=[w.min(axis=0).tolist(), w.max(axis=0).tolist()]))
    if any(p['dropped_zero_area_triangles'] for p in parts):
        warnings.append('Numerically zero-area tessellation triangles were removed; counts are recorded per part.')
    manifest = dict(schema=SCHEMA, created_at_utc=datetime.now(timezone.utc).isoformat(),
                    source=dict(filename=source.name, sha256=digest, bytes=source.stat().st_size,
                                declared_length_unit='see STEP header; reader explicitly normalizes to mm'),
                    tools={n:importlib.metadata.version(n) for n in ('cadquery-ocp','mujoco','numpy','pillow')},
                    meshing=dict(occt_target_unit='mm', output_unit='m', linear_deflection_mm=args.deflection_mm,
                                 angular_deflection_rad=args.angular_deflection_rad, face_normals='split at CAD face boundaries'),
                    bounds=dict(step_brep_source_m=exact_bounds.tolist(), tessellated_source_m=bounds.tolist(),
                                origin_shift_source_m=center.tolist(), dimensions_m=extent.tolist(),
                                centered_m=(bounds-center).tolist(), step_vs_mesh_max_error_m=error),
                    physics=dict(visual_model_is_fixed=True, inferred_joints=False, actual_mass_known=False,
                                 synthetic_preview_mass_kg=args.preview_mass_kg,
                                 preview_inertia='uniform bounding box centered at geometric AABB center; not CAD mass properties',
                                 collision='free preview ONLY: one conservative AABB; not suitable for grasp holes/concavities'),
                    counts=dict(assembly_nodes=len(nodes), assembly_occurrences=sum(bool(n['assembly']) for n in nodes),
                                rendered_leaf_instances=len(occurrences), unique_parts=len(parts), meshes=len(mesh_records),
                                visual_geoms=len(geoms), omitted_cad_faces=sum(len(p['unmeshed_faces']) for p in parts),
                                omitted_unique_face_area_mm2=sum(f['area_mm2'] for p in parts for f in p['unmeshed_faces']),
                                unique_triangles=sum(m['triangles'] for m in mesh_records),
                                instanced_triangles=sum(next(m['triangles'] for m in mesh_records if m['name']==g['mesh']) for g in geoms)),
                    parts=part_records, assembly=nodes, meshes=mesh_records, geoms=geoms, warnings=warnings)
    for filename, mass in [('ground_validation_satellite.xml',None),('satellite_free_preview.xml',args.preview_mass_kg)]:
        root = create_mjcf(mesh_records, geoms, extent, mass)
        if manifest['counts']['omitted_cad_faces']:
            root.insert(0, ET.Comment(f" INCOMPLETE visual preview: {manifest['counts']['omitted_cad_faces']} CAD faces could not be tessellated; see conversion_manifest.json and diagnostics/. "))
        save_xml(root, output/filename)
    manifest['status'] = 'geometry_exported'
    manifest_path = output/'conversion_manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    manifest['validation'] = validate(output, geoms, mesh_records, manifest)
    manifest['status'] = 'validated'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    if not args.no_render:
        print('Rendering MuJoCo preview images...', flush=True)
        manifest['preview'] = render_preview(output, extent)
    manifest['status'] = 'complete'
    manifest['elapsed_seconds'] = time.perf_counter()-start
    (output/'conversion_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(output=str(output), dimensions_m=extent.tolist(), counts=manifest['counts'],
                          validation=manifest['validation'], elapsed_seconds=manifest['elapsed_seconds']), indent=2), flush=True)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--deflection-mm',type=float,default=0.25,help='Absolute OCCT tessellation tolerance in mm')
    parser.add_argument('--angular-deflection-rad',type=float,default=0.18)
    parser.add_argument('--preview-mass-kg',type=float,default=1.0,help='SYNTHETIC free-preview mass, NOT a STEP-derived measurement')
    parser.add_argument('--allow-incomplete-preview',action='store_true',help='Explicitly allow and document unrecoverable faces; save source BREP diagnostics')
    parser.add_argument('--no-render',action='store_true')
    convert(parser.parse_args())


if __name__ == '__main__':
    main()


