"""Solid geometry operations for independent task-box asset preparation.

Use the separate geometry environment (numpy/trimesh/manifold3d/scipy).
Runtime collection imports capture_layout instead of loading these tools.
"""
from __future__ import annotations
import manifold3d as mf
import numpy as np
import trimesh
from scipy.spatial import ConvexHull

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
