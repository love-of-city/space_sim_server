"""Build a 13-unit task-box layout with original mechanisms and mating surfaces.

Run in the separate geometry environment (numpy/trimesh/manifold3d/coacd).
Generated meshes, MJCF and manifests belong outside the source model folder.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import re
import shutil
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import coacd
import manifold3d as mf
import numpy as np
import trimesh
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.independent_task_box_layout import (
    FREE_BODY_MODULES, MODULES, PICKUPS, POSE_SCHEMA, SCHEMA, TARGETS,
    make_operational_clearance, validate_operational_placements,
    normalize_geometry_order, planar_transform, sample_placements, sample_pose_placements,
    validate_pose_placements,
)
from simulation.socket_colors import apply_socket_colors


def vector(value, default="0 0 0"):
    return np.fromstring(value or default, sep=" ")


def floats(values):
    return " ".join(format(float(x), ".17g") for x in values)


def solid(mesh):
    result = mf.Manifold(mf.Mesh64(vert_properties=np.array(mesh.vertices, dtype=np.float64, order="C", copy=True),
                                   tri_verts=np.array(mesh.faces, dtype=np.uint64, order="C", copy=True)))
    if result.status() != mf.Error.NoError:
        raise ValueError(f"Invalid closed source solid: {result.status()}")
    return result


def mesh_of(value):
    raw = value.to_mesh64()
    return trimesh.Trimesh(vertices=np.asarray(raw.vert_properties)[:, :3],
                           faces=np.asarray(raw.tri_verts), process=False)


def export_hard_edge_obj(mesh, path):
    """Separate normal indices keep the deck flat without duplicating positions."""
    with path.open("w") as stream:
        for point in mesh.vertices:
            stream.write("v "+floats(point)+"\n")
        for normal in mesh.face_normals:
            stream.write("vn "+floats(normal)+"\n")
        for index, face in enumerate(mesh.faces, 1):
            stream.write("f "+" ".join(f"{vertex+1}//{index}" for vertex in face)+"\n")


def cell(value, point, others):
    """Intersect with the point's planar Voronoi half-spaces, at every depth."""
    for other in others:
        difference = point-other
        length = np.linalg.norm(difference)
        if length < 1e-12:
            continue
        normal = np.r_[difference/length, 0.]
        distance = (np.dot(point, point)-np.dot(other, other))/(2*length)
        value = value.trim_by_plane(tuple(normal), float(distance))
    return value


def aperture(contour):
    area = np.sum(contour[:, 0]*np.roll(contour[:, 1], -1)
                  - contour[:, 1]*np.roll(contour[:, 0], -1))
    contour = contour if area > 0 else contour[::-1]
    return mf.CrossSection([contour]).extrude(.025).translate((0, 0, -.025))


def polygon_area(points):
    return abs(float(np.sum(points[:, 0]*np.roll(points[:, 1], -1)
                            - points[:, 1]*np.roll(points[:, 0], -1))))/2


def convex_apertures(cavity):
    """Exact convex partition of a simple deck aperture, merging triangles."""
    contours = cavity.slice(-.01).to_polygons()
    if len(contours) != 1:
        raise ValueError("Expected one simple aperture in each installation cell")
    # Rotating CAD sections can leave near-duplicate edges a few nanometres
    # apart. Successive half-space cuts along these edges are ill-conditioned
    # and can retain a large convex fragment inside an otherwise open hole.
    # Remove only sub-10-nm contour noise before computing clipping planes.
    simplified = mf.CrossSection(contours).simplify(1e-8).to_polygons()
    if len(simplified) != 1:
        raise ValueError("Aperture contour cleanup changed its topology")
    contour = np.asarray(simplified[0])
    if abs(ConvexHull(contour).volume-polygon_area(contour)) < 1e-10:
        signed = np.sum(contour[:, 0]*np.roll(contour[:, 1], -1)-contour[:, 1]*np.roll(contour[:, 0], -1))
        return [contour if signed > 0 else contour[::-1]]
    triangles = mf.triangulate([contour], epsilon=1e-12)
    polygons = [contour[np.asarray(triangle)] for triangle in triangles
                if polygon_area(contour[np.asarray(triangle)]) > 1e-14]
    while True:
        merged = False
        for i, first in enumerate(polygons):
            for j in range(i+1, len(polygons)):
                second = polygons[j]
                shared = sum(any(np.linalg.norm(p-q) < 1e-10 for q in second) for p in first)
                if shared < 2:
                    continue
                points = np.vstack((first, second))
                hull = ConvexHull(points)
                if abs(hull.volume-polygon_area(first)-polygon_area(second)) > 1e-12:
                    continue
                polygons[i] = points[hull.vertices]
                polygons.pop(j)
                merged = True
                break
            if merged:
                break
        if not merged:
            break
    if abs(sum(map(polygon_area, polygons))-polygon_area(contour)) > 1e-10:
        raise ValueError("Aperture partition lost area")
    return polygons


def carve_convex_hulls(hulls, polygons):
    """Subtract vertical aperture prisms with half-spaces, keeping convexity.

    CoACD may bridge an opening. Clipping its output against the exact aperture
    prevents this approximation from closing an operation's insertion path.
    """
    holes = []
    for polygon in polygons:
        prism = mf.CrossSection([polygon]).extrude(.2).translate((0, 0, -.1))
        holes.append((polygon, prism, np.array([polygon.min(0), polygon.max(0)])))
    output, discarded = [], []
    for vertices, faces in hulls:
        pieces = [solid(trimesh.Trimesh(vertices=vertices, faces=faces, process=False).convex_hull)]
        for polygon, prism, hole_bounds in holes:
            next_pieces = []
            for piece in pieces:
                box = np.asarray(piece.bounding_box()).reshape(2, 3)[:, :2]
                if (np.any(box[1] <= hole_bounds[0]) or np.any(hole_bounds[1] <= box[0])
                        or (piece ^ prism).volume() < 1e-14):
                    next_pieces.append(piece)
                    continue
                inside = piece
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    edge = b-a
                    inward = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
                    normal, distance = (*inward, 0.), float(np.dot(inward, a))
                    outside = inside.trim_by_plane(tuple(-np.asarray(normal)), -distance)
                    if outside.volume() > 1e-13:
                        next_pieces.append(outside)
                    inside = inside.trim_by_plane(normal, distance)
                    if inside.is_empty():
                        break
            pieces = next_pieces
        for piece in pieces:
            mesh = mesh_of(piece)
            axes = np.linalg.svd(mesh.vertices-mesh.vertices.mean(0), compute_uv=False)/np.sqrt(len(mesh.vertices))
            if piece.volume() <= 1e-13 or axes[-1] < 1e-6:
                discarded.append(abs(float(piece.volume())))
            else:
                output.append(mesh)
    return output, dict(discarded_collision_micro_slivers=len(discarded),
                        discarded_collision_volume_m3=sum(discarded), collision_minimum_principal_std_m=1e-6)


def inset_mount(piece, point):
    """Keep the original sleeve and a narrow rim, replacing its large plate.

    Both the sleeve and the complete top opening contribute to the envelope:
    the guide's keyway extends past its open-sided sleeve. Deriving a profile
    from the sleeve alone would cut away the closed rim around that keyway.
    """
    original = solid(piece)
    # Exclude the original plate's coplanar z=0 numerical fragments before
    # constructing the sleeve footprint. The full original sleeve is retained
    # by the later intersection, including the uppermost 0.1 micrometres.
    below = original.trim_by_plane((0, 0, -1), 1e-7)
    sleeves = [part for part in below.decompose() if part.volume() > 1e-12]
    if len(sleeves) != 1:
        raise ValueError("Expected one solid sleeve below each mounting hole")
    sleeve = sleeves[0]
    openings = []
    for raw in original.slice(.0025).to_polygons():
        contour = np.asarray(raw)
        signed_area = np.sum(contour[:, 0]*np.roll(contour[:, 1], -1)
                             - contour[:, 1]*np.roll(contour[:, 0], -1))/2
        if signed_area < -1e-7 and np.all(contour.min(0) < point) and np.all(point < contour.max(0)):
            openings.append(contour)
    if len(openings) != 1:
        raise ValueError("Cannot identify the original mounting hole's top opening")
    opening = openings[0]
    vertices = np.vstack((mesh_of(sleeve).vertices[:, :2], opening))
    hull = ConvexHull(vertices)
    profile = mf.CrossSection([vertices[hull.vertices]])
    # A regular outline following the sleeve; no Voronoi plate corners remain.
    rim = profile.offset(.001, circular_segments=64)
    cavity = profile.offset(.00025, circular_segments=64)
    contour = np.asarray(rim.to_polygons()[0])
    prism = rim.extrude(.1).translate((0, 0, -.075))
    kept = original ^ prism
    lost_sleeve = float((below-kept).volume())
    opening_section = mf.CrossSection([opening[::-1]])
    lost_opening_area = float((opening_section-rim).area())
    if lost_sleeve > 1e-13 or lost_opening_area > 1e-12:
        raise ValueError("Inset profile would change the original mating geometry")
    result = mesh_of(kept)
    if not result.is_volume or result.body_count != 1:
        raise ValueError("Inset mount must remain one closed solid")
    report = dict(original_volume_m3=float(piece.volume), retained_volume_m3=float(result.volume),
                  removed_plate_volume_m3=float(piece.volume-result.volume),
                  rim_extension_m=.001, deck_clearance_m=.00025,
                  rim_height_m=.005, preserved_sleeve_depth_m=float(-piece.bounds[0, 2]),
                  missing_original_sleeve_volume_m3=lost_sleeve,
                  missing_original_top_opening_area_m2=lost_opening_area,
                  rim_profile_body_xy_m=contour.tolist(),
                  deck_aperture_profile_body_xy_m=np.asarray(cavity.to_polygons()[0]).tolist())
    return result, contour, cavity.extrude(.025).translate((0, 0, -.025)), report


def clip_convex_profile(value, contour):
    """Crop a delivered convex collider to a convex mounting rim footprint."""
    for a, b in zip(contour, np.roll(contour, -1, axis=0)):
        edge = b-a
        inward = np.array([-edge[1], edge[0]])/np.linalg.norm(edge)
        value = value.trim_by_plane((*inward, 0.), float(np.dot(inward, a)))
        if value.is_empty():
            break
    return value


class Builder:
    def __init__(self, source, output, *, target_mounts="plates", pickup_mounts="plates"):
        if target_mounts not in {"plates", "inset"} or pickup_mounts not in {"plates", "inset"}:
            raise ValueError("Unknown mounting style")
        self.target_mounts = target_mounts
        self.pickup_mounts = pickup_mounts
        self.has_inset_mounts = "inset" in (target_mounts, pickup_mounts)
        self.inset_profiles, self.inset_cavities, self.inset_reports = {}, {}, {}
        self.source, self.output = source.resolve(), output.resolve()
        if self.output == self.source or self.output.exists():
            raise ValueError("Output must be a new, separate model")
        self.source_bytes = self.source.read_bytes()
        self.root = ET.fromstring(self.source_bytes)
        if self.root.find("include") is not None or self.root.find("keyframe") is not None:
            raise ValueError("Requires the standalone, delivered SARM assembly")
        self.base = self.body("task_box_base")
        self.bus = self.body("cubesat_bus")
        if self.base not in list(self.bus) or self.base.get("quat", "1 0 0 0") != "1 0 0 0":
            raise ValueError("Expected the delivered task-box parent frame")
        self.base_position = vector(self.base.get("pos"))
        self.assets = self.root.find("asset")
        self.source_assets = {a.get("name"): a for a in self.assets.findall("mesh")}
        self.mesh_cache, self.source_hashes = {}, {}
        self.visuals, self.collision_partitions, self.translated = [], [], []
        self.folder = self.output.parent / "geometry"
        self.folder.mkdir(parents=True, exist_ok=True)

    def body(self, name):
        node = self.root.find(f'.//body[@name="{name}"]')
        if node is None:
            raise ValueError(f"Missing body {name}")
        return node

    def asset_mesh(self, name):
        if name not in self.mesh_cache:
            asset = self.source_assets[name]
            file = (self.source.parent / asset.get("file")).resolve()
            self.source_hashes[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
            mesh = trimesh.load_mesh(file, process=True)
            mesh.apply_scale(vector(asset.get("scale"), "1 1 1"))
            self.mesh_cache[name] = mesh
        return self.mesh_cache[name]

    def transform(self, node):
        matrix = trimesh.transformations.quaternion_matrix(vector(node.get("quat"), "1 0 0 0"))
        matrix[:3, 3] = vector(node.get("pos"))
        return matrix

    def visible_vertices(self, body, parent=None, joints=None):
        parent = np.eye(4) if parent is None else parent
        joints = joints or {}
        matrix = parent @ self.transform(body)
        for joint in body.findall("joint"):
            q = joints.get(joint.get("name"), 0.)
            if not q:
                continue
            axis = vector(joint.get("axis"), "0 0 1")
            if joint.get("type", "hinge") == "hinge":
                matrix = matrix @ trimesh.transformations.rotation_matrix(q, axis, vector(joint.get("pos")))
            elif joint.get("type") == "slide":
                matrix = matrix @ trimesh.transformations.translation_matrix(q*axis)
        vertices = []
        for geom in body.findall('geom[@group="2"]'):
            if geom.get("mesh"):
                vertices.append(trimesh.transform_points(self.asset_mesh(geom.get("mesh")).vertices,
                                                         matrix @ self.transform(geom)))
        for child in body.findall("body"):
            vertices.extend(self.visible_vertices(child, matrix, joints))
        return vertices

    def export_visual(self, name, mesh, template, parent):
        if not mesh.is_volume:
            raise ValueError(f"{name} is not a closed solid")
        metadata = {}
        if name == "task_box_base" and self.has_inset_mounts:
            # Boolean seams can collapse in binary STL's float32 coordinates.
            # Use a closed double-precision OBJ for this newly rebuilt deck.
            mesh = mesh_of(solid(mesh).set_tolerance(1e-6))
            if not mesh.is_volume:
                raise ValueError("Inset deck export is not a closed solid")
            metadata = dict(surface_simplification_tolerance_m=1e-6, source_format="obj",
                            export_watertight=True)
        elif name in self.inset_profiles:
            # Clipping the original plate can produce sub-float32 seams too.
            # Remove seam slivers within 0.1 micrometres before the UE import,
            # retaining a closed solid and auditing the delivered surfaces.
            mesh = mesh_of(solid(mesh).set_tolerance(1e-7))
            if not mesh.is_volume:
                raise ValueError("Inset mount export is not a closed solid")
            metadata = dict(source_format="obj", export_watertight=True,
                            surface_simplification_tolerance_m=1e-7)
        elif name == "task_box_base":
            # Boolean seams contain triangles thinner than binary STL's
            # float32 coordinates. Simplify within 0.1 micrometres and remove
            # negligible seam faces after that conversion. Their removed
            # surface area is recorded rather than hidden by the UE importer.
            mesh = mesh_of(solid(mesh).simplify(1e-7))
            quantized = np.asarray(mesh.vertices, dtype=np.float32).astype(np.float64)
            triangles = quantized[mesh.faces]
            twice_area = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0],
                                                  triangles[:, 2]-triangles[:, 0]), axis=1)
            keep = twice_area > 1e-14
            metadata = dict(surface_simplification_tolerance_m=1e-7,
                            removed_negligible_export_faces=int(np.count_nonzero(~keep)),
                            removed_export_surface_area_m2=float(twice_area[~keep].sum()/2))
            mesh = trimesh.Trimesh(vertices=quantized, faces=mesh.faces[keep], process=False)
        suffix = ".obj" if metadata.get("source_format") == "obj" else ".stl"
        file = self.folder / (name+suffix)
        if suffix == ".obj":
            # Preserve hard CAD edges. Averaged vertex normals spanning the
            # deck and vertical aperture walls shade its flat top as wedges.
            export_hard_edge_obj(mesh, file)
            metadata["normal_mode"] = "per-face hard edges"
        else:
            mesh.export(file)
        asset_name = f"layout_{name}_mesh"
        ET.SubElement(self.assets, "mesh", name=asset_name, file=str(file))
        geom = copy.deepcopy(template)
        geom.set("name", f"layout_{name}_visual")
        geom.set("mesh", asset_name)
        parent.append(geom)
        self.visuals.append(dict(name=name, file=str(file), sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                                 triangle_count=len(mesh.faces), **metadata))

    def export_convex(self, name, mesh, template, parent):
        file = self.folder / f"{name}.obj"
        center = mesh.bounds.mean(0)
        mesh = mesh.copy()
        mesh.apply_translation(-center)
        mesh.export(file, digits=17)
        ET.SubElement(self.assets, "mesh", name=name+"_mesh", file=str(file))
        geom = copy.deepcopy(template)
        geom.set("name", name)
        geom.set("mesh", name+"_mesh")
        geom.set("pos", floats(center))
        geom.attrib.pop("quat", None)
        parent.append(geom)

    def split_board(self, name, identifiers):
        body = self.body(name)
        sites = {s.get("name"): s for s in body.findall("site")}
        if set(sites) != set(identifiers):
            raise ValueError(f"Unexpected sites in {name}")
        points = {key: vector(sites[key].get("pos"))[:2] for key in identifiers}
        for point in points.values():
            if min(np.linalg.norm(point-other)/2 for other in points.values()
                   if not np.array_equal(point, other)) < .0249:
                raise ValueError("Board cuts would approach the original mating region")
        whole_mesh = self.asset_mesh(name+"_mesh")
        whole = solid(whole_mesh)
        visual = body.find(f'geom[@name="{name}_geom"]')
        pieces, modules = {}, {}
        original_piece_volume = 0.
        for key in identifiers:
            piece = mesh_of(cell(whole, points[key], points.values()))
            original_piece_volume += float(piece.volume)
            mounting_style = self.target_mounts if name == "asm_board1" else self.pickup_mounts
            if mounting_style == "inset":
                piece, contour, cavity, report = inset_mount(piece, points[key])
                self.inset_profiles[key] = contour
                self.inset_cavities[key] = cavity
                self.inset_reports[key] = report
            pieces[key] = piece
            module = ET.SubElement(body, "body", name="layout_"+key)
            module.append(sites[key])
            body.remove(sites[key])
            self.export_visual(key, piece, visual, module)
            modules[key] = module
        body.remove(visual)
        if abs(original_piece_volume/whole_mesh.volume-1) > 1e-6:
            raise ValueError("Splitting lost board material")
        # Intersect delivered convex collision hulls with the same half-spaces.
        # A convex hull intersected with half-spaces remains convex. This keeps
        # original contact surfaces instead of approximating the narrow holes.
        originals = [g for g in body.findall("geom")
                     if g.get("name", "").startswith(name+"_geom_collision_")]
        for geom in originals:
            if not np.allclose(self.transform(geom), np.eye(4)):
                raise ValueError("Unsupported nonidentity board collider frame")
            hull = self.asset_mesh(geom.get("mesh")).convex_hull
            hull_solid = solid(hull)
            total, count, removed = 0., 0, 0.
            for key in identifiers:
                fragment = cell(hull_solid, points[key], points.values())
                if fragment.is_empty() or fragment.volume() < 1e-14:
                    continue
                before = float(fragment.volume())
                total += before
                if key in self.inset_profiles:
                    fragment = clip_convex_profile(fragment, self.inset_profiles[key])
                    removed += max(0., before-float(fragment.volume()))
                    if fragment.is_empty() or fragment.volume() < 1e-14:
                        continue
                part = mesh_of(fragment)
                self.export_convex("layout_"+geom.get("name")+"_"+key, part, geom, modules[key])
                count += 1
            if abs(total/float(hull.volume)-1) > 1e-5:
                raise ValueError(f"Collision partition lost material: {geom.get('name')}")
            self.collision_partitions.append(dict(source_geom=geom.get("name"), pieces=count,
                                                  relative_volume_error=total/float(hull.volume)-1,
                                                  intentionally_removed_mount_volume_m3=removed))
            body.remove(geom)
        return modules, points, pieces

    def translate(self, name, offset, module):
        body = self.body(name)
        before = vector(body.get("pos"))
        delta = np.r_[offset, 0.]
        body.set("pos", floats(before+delta))
        self.translated.append(dict(body=name, module=module, delta_body_m=delta.tolist(),
                                    original_parent_position_m=before.tolist()))

    def world_transform(self, node):
        parents = {child: parent for parent in self.root.iter() for child in parent}
        chain = []
        while node is not None:
            if node.tag == "body":
                chain.append(self.transform(node))
            node = parents.get(node)
        result = np.eye(4)
        for matrix in reversed(chain):
            result = result @ matrix
        return result

    def reposition(self, name, module, offset, yaw, matrix):
        if yaw == 0.:
            return self.translate(name, offset, module)
        body = self.body(name)
        before = self.transform(body)
        parents = {child: parent for parent in self.root.iter() for child in parent}
        parent_world = self.world_transform(parents[body])
        deck_world = self.world_transform(self.base)
        relative = np.linalg.inv(parent_world) @ deck_world
        after = relative @ matrix @ np.linalg.inv(relative) @ before
        body.set("pos", floats(after[:3, 3]))
        body.set("quat", floats(trimesh.transformations.quaternion_from_matrix(after)))
        self.translated.append(dict(body=name, module=module, delta_body_m=[*offset, 0.],
                                    installation_yaw_rad=yaw,
                                    original_parent_position_m=before[:3, 3].tolist()))

    def prepare(self):
        self.modules, self.points, self.pieces = {}, {}, {}
        for board, identifiers in (("asm_board2", PICKUPS), ("asm_board1", TARGETS)):
            modules, points, pieces = self.split_board(board, identifiers)
            self.modules.update(modules)
            self.points.update(points)
            self.pieces.update(pieces)
        # Keep precision bores on separate bodies: exclusions for the coarse
        # mounting-plate hulls must not suppress shaft/wall contact.
        bore_parent = self.body("board1_plug_bore_colliders")
        self.bores = {name: ET.SubElement(bore_parent, "body", name="layout_"+name+"_bores")
                      for name in TARGETS if "_insert_" in name}
        self.bore_ownership = {}
        for geom in list(bore_parent.findall("geom")):
            match = re.fullmatch(r"(aviation|guide|cylindrical)_(?:bore|pin_floor)_(40p1|40p5)_.+",
                                 geom.get("name", ""))
            if not match:
                raise ValueError(f"Unassigned precision bore: {geom.get('name')}")
            key = f"{match[1]}_insert_{match[2]}"
            self.bores[key].append(geom)
            bore_parent.remove(geom)
            self.bore_ownership[geom.get("name")] = key
        if len(self.bore_ownership) != 924:
            raise ValueError("Unexpected delivered precision bore count")
        from tools.bolt_storage_thread import attach_bolt_thread_mounts
        self.bolt_storage_thread = attach_bolt_thread_mounts(self.root, self.modules)

        self.base_mesh = self.asset_mesh("task_box_base_mesh")
        self.base_solid = solid(self.base_mesh)
        section = self.base_mesh.section(plane_origin=[0, 0, -.01], plane_normal=[0, 0, 1])
        contours = [section.vertices[e.points, :2] for e in section.entities]
        windows = {"pickup": ([.09, .10], [.29, .32]),
                   "target": ([.27, .09], [.45, .41]),
                   "drawer": ([.22, 0], [.44, .08])}
        self.old_apertures, self.cavities = {}, {}
        for name, (low, high) in windows.items():
            candidates = [c for c in contours if np.all(c.min(0) >= low)
                          and np.all(c.max(0) <= high)]
            if len(candidates) != 1:
                raise ValueError(f"Cannot uniquely identify {name} base aperture")
            whole = aperture(candidates[0])
            self.old_apertures[name] = whole
            identifiers = PICKUPS if name == "pickup" else TARGETS if name == "target" else ()
            if identifiers:
                for key in identifiers:
                    self.cavities[key] = (self.inset_cavities[key] if key in self.inset_cavities else
                        cell(whole, self.points[key], [self.points[k] for k in identifiers]))
            else:
                self.cavities["drawer"] = whole
        self.supports = self.base_solid.trim_by_plane((0, 0, 1), 0).trim_by_plane((1, 0, 0), .23)
        support_bounds = mesh_of(self.supports).bounds
        if not (np.all(support_bounds[0, :2] >= [.23, .46])
                and np.all(support_bounds[1, :2] <= [.43, .54])):
            raise ValueError("Unexpected locking support geometry")
        ridge = self.base_solid.trim_by_plane((0, 0, 1), 0) - self.supports
        self.obstacles = [mesh_of(ridge).bounds[:, :2].tolist()]
        vertices = {name: [piece.vertices] for name, piece in self.pieces.items()}
        for name, value in self.cavities.items():
            vertices.setdefault(name, []).append(mesh_of(value).vertices)
        minus_base = trimesh.transformations.translation_matrix(-self.base_position)
        for body, module in FREE_BODY_MODULES.items():
            vertices[module].extend(self.visible_vertices(self.body(body), minus_base))
        vertices["drawer"].extend(self.visible_vertices(self.body("asm_board3")))
        for slide in (-.005, 0., .15):
            for buckle in np.linspace(-.25, .1, 9):
                vertices["drawer"].extend(self.visible_vertices(self.body("drawer_base"), minus_base,
                    {"drawer_base_slide": slide, "drawer_buckle_joint": buckle}))
        vertices["locking_mechanism"] = [mesh_of(self.supports).vertices]
        for name in ("hook", "limit_clip"):
            vertices["locking_mechanism"].extend(self.visible_vertices(self.body(name)))
        for angle in np.linspace(-.75, .15, 37):
            for slide in (0., .054):
                for spin in np.linspace(0, 2*np.pi, 9):
                    vertices["locking_mechanism"].extend(self.visible_vertices(self.body("locking_rod"),
                        joints={"locking_rod_hinge": angle, "fastener_slide": slide, "fastener_spin": spin}))
        # A small extra envelope protects between the sampled joint angles.
        self.footprints = {}
        for name, sets in vertices.items():
            values = np.concatenate(sets)[:, :2]
            self.footprints[name] = (np.array([values.min(0), values.max(0)])
                                     + np.array([[-.001, -.001], [.001, .001]])).tolist()

    def rebuild_base(self, transforms):
        def moved_solid(value, name):
            return value.transform(transforms[name][:3, :])

        filled = self.base_solid
        for old in self.old_apertures.values():
            filled = filled + old
        slab_audit = {}
        if self.has_inset_mounts:
            # All three delivered deck openings have been filled. Verify that
            # its remaining slab is the 450 x 550 x 25 mm box, up to CAD noise,
            # before using that exact slab in the rebuilt mesh and collisions.
            slab = mf.Manifold.cube((.45, .55, .025)).translate((0, 0, -.025))
            original_slab = filled.trim_by_plane((0, 0, -1), 1e-7)
            comparison = slab.trim_by_plane((0, 0, -1), 1e-7)
            changed = float((original_slab-comparison).volume()+(comparison-original_slab).volume())
            if changed > 1e-9:
                raise ValueError("Delivered filled deck differs from the expected rectangular slab")
            above = filled.trim_by_plane((0, 0, 1), 0.).set_tolerance(1e-7)
            buried_roots = above.translate((0, 0, -.00001)).trim_by_plane((0, 0, -1), 0.)
            filled = slab + above + buried_roots
            slab_audit = dict(rectangular_slab_size_m=[.45, .55, .025],
                              filled_source_slab_difference_m3=changed,
                              filled_source_slab_difference_limit_m3=1e-9)
        result = filled - self.supports
        moved = {}
        for name, cavity in self.cavities.items():
            translated = moved_solid(cavity, name)
            moved[name] = mesh_of(translated).bounds.tolist()
            result = result - translated
        moved_supports = moved_solid(self.supports, "locking_mechanism")
        result = result + moved_supports
        # Delivered CAD features meet the deck within nanometres of z=0.
        # Extend only their buried roots by 10 micrometres to avoid numerical
        # disconnected pins after solid translation. Above-deck surfaces stay
        # identical, and the weld remains inside the reserved lock footprint.
        roots = moved_supports.translate((0., 0., -.00001)).trim_by_plane((0, 0, -1), 0.)
        result = result + roots
        components = [mesh_of(part) for part in result.decompose()]
        self.output.with_suffix(".base-components.json").write_text(json.dumps([
            dict(volume_m3=float(p.volume), bounds=p.bounds.tolist(), watertight=bool(p.is_watertight),
                 faces=len(p.faces)) for p in components], indent=2)+"\n")
        significant = [part for part in components if abs(part.volume) > 1e-12]
        if len(significant) != 1:
            raise ValueError("Relocated base has disconnected physical components")
        # Coplanar Boolean seams can leave zero-thickness components with
        # volumes around 1e-24 m^3; retain the single physical solid.
        numerical_fragments = [part for part in components if abs(part.volume) <= 1e-12]
        result = significant[0]
        if not result.is_volume or result.body_count != 1:
            raise ValueError("Relocated base must remain one closed solid")
        visual = self.base.find('geom[@name="task_box_base_geom"]')
        self.export_visual("task_box_base", result, visual, self.base)
        self.base.remove(visual)
        originals = [g for g in self.base.findall("geom")
                     if g.get("name", "").startswith("task_box_base_geom_collision_")]
        if not originals:
            raise ValueError("Delivered base collision decomposition missing")
        template = originals[0]
        for g in originals:
            self.base.remove(g)
        started = time.monotonic()
        if self.has_inset_mounts:
            # Avoid numerical CoACD failures on many narrow circular Boolean
            # holes. The slab is exact; reuse the delivered top-feature hulls
            # and carve every opening from this union of convex components.
            slab_mesh = mesh_of(slab)
            convex = [(slab_mesh.vertices, slab_mesh.faces)]
            for geom in originals:
                upper = solid(self.asset_mesh(geom.get("mesh")).convex_hull).trim_by_plane((0, 0, 1), 0.)
                for normal, offset, move in (((-1, 0, 0), -.23, False), ((1, 0, 0), .23, True)):
                    part = upper.trim_by_plane(normal, offset)
                    if part.is_empty() or part.volume() < 1e-12 or part.bounding_box()[5] < 1e-6:
                        continue
                    mesh = mesh_of(moved_solid(part, "locking_mechanism") if move else part)
                    convex.append((mesh.vertices, mesh.faces))
            collision_method = "Exact rectangular slab plus relocated original top-feature convex hulls; exact aperture subtraction"
        else:
            coacd.set_log_level("warn")
            convex = coacd.run_coacd(coacd.Mesh(np.asarray(result.vertices, dtype=np.float64),
                                              np.asarray(result.faces, dtype=np.int32)),
                                     threshold=.02, max_convex_hull=128, preprocess_mode="off",
                                     resolution=2000, mcts_iterations=100, mcts_nodes=20,
                                     mcts_max_depth=3, seed=0)
            collision_method = "CoACD followed by exact vertical aperture half-space subtraction; native shaft-clearance validation required"
        coacd_parts = len(convex)
        polygons = [polygon for name, cavity in self.cavities.items()
                    for polygon in convex_apertures(moved_solid(cavity, name))]
        convex, numerical_collision = carve_convex_hulls(convex, polygons)
        for index, part in enumerate(convex):
            self.export_convex(f"layout_base_collision_{index:03d}",
                               part, template, self.base)
        return dict(visual=self.visuals[-1]["file"], original_volume_m3=float(self.base_mesh.volume),
                    **numerical_collision, **slab_audit,
                    adapted_volume_m3=float(result.volume),
                    material_volume_change_fraction=float(result.volume/self.base_mesh.volume-1),
                    method="Fill old openings; cut independently moved installation apertures; relocate locking supports",
                    support_root_weld_depth_m=.00001,
                    discarded_numerical_components=len(numerical_fragments),
                    discarded_absolute_volume_m3=sum(abs(p.volume) for p in numerical_fragments),
                    collision_method=collision_method,
                    aperture_contour_cleanup_tolerance_m=1e-8,
                    collision_parts=len(convex), initial_collision_parts=coacd_parts,
                    coacd_initial_parts=coacd_parts if not self.has_inset_mounts else None,
                    collision_threshold=.02 if not self.has_inset_mounts else None,
                    collision_preprocess="off" if not self.has_inset_mounts else None,
                    collision_max_parts=128 if not self.has_inset_mounts else None,
                    decomposition_seconds=time.monotonic()-started, cavity_bounds=moved)

    def build(self, seed, *, plan_only=False, placements=None, yaw_range_deg=0., yaws=None):
        self.prepare()
        self.output.with_suffix(".footprints.json").write_text(json.dumps(
            dict(footprints=self.footprints, obstacles=self.obstacles), indent=2)+"\n")
        anchors = {name: [*point, .005] for name, point in self.points.items()}
        anchors["drawer"] = (vector(self.body("drawer_base").get("pos"))-self.base_position).tolist()
        anchors["locking_mechanism"] = vector(self.body("locking_rod").get("pos")).tolist()
        rotated = yaws is not None or yaw_range_deg != 0.
        if placements is None:
            if yaws is not None:
                raise ValueError("Explicit yaws require explicit XY placements")
            if rotated:
                placements, yaws, restarts = sample_pose_placements(
                    self.footprints, anchors, seed, self.obstacles, yaw_range_deg=yaw_range_deg,
                    operational_clearance=True)
            else:
                placements, restarts = sample_placements(self.footprints, seed, self.obstacles,
                                                        operation_anchors=anchors)
        else:
            if yaw_range_deg != 0.:
                raise ValueError("Use explicit --yaws with --placements, or sample both with --yaw-range-deg")
            restarts = 0
        yaws = dict.fromkeys(MODULES, 0.) if yaws is None else yaws
        rectangles = validate_pose_placements(self.footprints, placements, anchors, yaws, self.obstacles)
        operation_boxes = validate_operational_placements(self.footprints, placements, anchors, yaws, self.obstacles)
        transforms = {name: planar_transform(anchors[name], placements[name], yaws[name]) for name in MODULES}
        record = dict(schema=POSE_SCHEMA if rotated else SCHEMA, seed=seed, source=str(self.source),
                      source_sha256=hashlib.sha256(self.source_bytes).hexdigest(),
                      initial_states_randomized=False, orientations_randomized=rotated,
                      offsets_body_xy_m=placements, original_footprints_body_xy_m=self.footprints,
                      placed_footprints_body_xy_m=rectangles, fixed_obstacles_body_xy_m=self.obstacles,
                      sampling_restarts=restarts, minimum_module_gap_m=.008, deck_edge_margin_m=.005,
                      operational_clearance=make_operational_clearance(self.footprints, anchors, self.obstacles),
                      placed_operation_envelopes_body_xy_m=operation_boxes,
                      target_mounts=self.target_mounts, pickup_mounts=self.pickup_mounts,
                      inset_targets={key: value for key, value in self.inset_reports.items() if key in TARGETS},
                      inset_pickups={key: value for key, value in self.inset_reports.items() if key in PICKUPS},
                      scope="13 independent installation units; optional deck-normal yaw; original mechanism joint states",
                      limits="Tool-operation reservations do not establish full-arm reachability or dynamic task success; native trajectory acceptance remains required",
                      modules={name:dict(delta_body_m=[*placements[name], 0.],
                                        original_anchor_body_m=anchors[name],
                                        transform_body=transforms[name].tolist())
                               for name in MODULES})
        if rotated:
            record.update(installation_yaw_rad=yaws, rotation_axis="task_box_base/+Z",
                          yaw_sampling_half_range_deg=yaw_range_deg if restarts else None,
                          sampling_method="Continuous XY/yaw rejection with conservative rotated swept AABBs")
        self.output.with_suffix(".plan.json").write_text(json.dumps(record, indent=2)+"\n")
        if plan_only:
            print(json.dumps(record, indent=2), flush=True)
            return record
        print(json.dumps(dict(stage="placement", offsets=placements, restarts=restarts)), flush=True)
        for name, module in self.modules.items():
            self.reposition(module.get("name"), name, placements[name], yaws[name], transforms[name])
        for name, body in self.bores.items():
            self.reposition(body.get("name"), name, placements[name], yaws[name], transforms[name])
        for body, module in FREE_BODY_MODULES.items():
            self.reposition(body, module, placements[module], yaws[module], transforms[module])
        for body in ("asm_board3", "drawer_base"):
            self.reposition(body, "drawer", placements["drawer"], yaws["drawer"], transforms["drawer"])
        for body in ("locking_rod", "hook", "limit_clip"):
            self.reposition(body, "locking_mechanism", placements["locking_mechanism"],
                            yaws["locking_mechanism"], transforms["locking_mechanism"])
        record["adapted_base"] = self.rebuild_base(transforms)
        contact = self.root.find("contact")
        for plug in ("aviation_plug", "guide_plug", "cylindrical_plug"):
            for target in TARGETS:
                ET.SubElement(contact, "exclude", name=f"layout_{plug}_{target}_convex_exclude",
                              body1=plug, body2=self.modules[target].get("name"))
        compiler = self.root.find("compiler")
        # Resolve source paths before removing compiler directory prefixes.
        for element in self.assets:
            if element.get("file"):
                file = Path(element.get("file"))
                if not file.is_absolute():
                    directory = compiler.get("meshdir", ".") if element.tag == "mesh" else compiler.get("texturedir", ".")
                    element.set("file", str((self.source.parent/directory/file).resolve()))
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)
        # Avoid loading abandoned monolithic boards or base collider assets.
        used = {g.get("mesh") for g in self.root.findall(".//geom") if g.get("mesh")}
        for asset in list(self.assets.findall("mesh")):
            if asset.get("name") not in used:
                self.assets.remove(asset)
        self.root.set("model", self.root.get("model", "sarm")+"_independent_layout")
        record["bolt_storage_thread"] = self.bolt_storage_thread
        record["socket_color_identity"] = apply_socket_colors(self.root)
        normalize_geometry_order(self.root)
        ET.ElementTree(self.root).write(self.output, encoding="utf-8", xml_declaration=True)
        attitude = self.source.with_name("attitude_control.json")
        if attitude.is_file():
            shutil.copyfile(attitude, self.output.with_name("attitude_control.json"))
        if self.source.read_bytes() != self.source_bytes:
            raise ValueError("Source model changed during generation")
        for file, sha in self.source_hashes.items():
            if hashlib.sha256(Path(file).read_bytes()).hexdigest() != sha:
                raise ValueError(f"Source mesh changed during generation: {file}")
        record.update(model=str(self.output), model_sha256=hashlib.sha256(self.output.read_bytes()).hexdigest(),
                      translated_bodies=self.translated, generated_visuals=self.visuals,
                      source_asset_sha256=self.source_hashes, collision_partitions=self.collision_partitions,
                      render_geometry_order_normalized=True,
                      precision_bore_ownership=self.bore_ownership, site_modules={name:name for name in self.points})
        self.output.with_suffix(".layout.json").write_text(json.dumps(record, indent=2)+"\n")
        print(json.dumps(dict(stage="complete", model=str(self.output), modules=len(MODULES),
                              visual_count=len(self.visuals), adapted_base=record["adapted_base"])), flush=True)
        return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--placements", type=Path, help="JSON with all 13 XY deltas; checked for overlap/bounds")
    parser.add_argument("--yaw-range-deg", type=float, default=0.,
                        help="Sample independent yaw in +/- this angle, in [0,180]; 0 keeps legacy translation sampling")
    parser.add_argument("--yaws", type=Path, help="JSON with all 13 deck-normal yaws in radians; requires --placements")
    parser.add_argument("--target-mounts", choices=("plates", "inset"), default="plates",
                        help="Keep split target plates, or crop them to narrow rims over fitted deck openings")
    parser.add_argument("--pickup-mounts", choices=("plates", "inset"), default="plates",
                        help="Keep split starting-socket plates, or crop them to narrow rims without moving the seated plugs")
    args = parser.parse_args()
    Builder(args.source, args.output, target_mounts=args.target_mounts, pickup_mounts=args.pickup_mounts).build(args.seed, plan_only=args.plan_only,
        placements=json.loads(args.placements.read_text()) if args.placements else None,
        yaw_range_deg=args.yaw_range_deg, yaws=json.loads(args.yaws.read_text()) if args.yaws else None)
