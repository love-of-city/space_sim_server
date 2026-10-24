"""Deterministic placement of independently installed task-box mechanisms.

Coordinates are in the delivered task-box frame, in metres. Optional yaw is
about the deck normal at each installation anchor. Internal joint states stay
unchanged. The builder supplies conservative mechanism swept envelopes.
"""
from __future__ import annotations

import math
import random

import numpy as np

SCHEMA = "task-box-layout/2"
POSE_SCHEMA = "task-box-layout/3"
PICKUPS = ("aviation_pickup", "guide_pickup", "cylindrical_pickup")
TARGETS = tuple(f"{kind}_insert_{size}" for kind in ("aviation", "guide", "cylindrical")
                for size in ("40p1", "40p5")) + ("bolt_storage_hole", "bolt_assembly_hole")
MODULES = (*PICKUPS, *TARGETS, "drawer", "locking_mechanism")
FREE_BODY_MODULES = {f"{kind}_plug": f"{kind}_pickup"
                     for kind in ("aviation", "guide", "cylindrical")} | {"bolt": "bolt_storage_hole"}


def planar_transform(anchor, offset, yaw):
    """Rotate around the original anchor, then move it by offset, in deck axes."""
    anchor, offset = np.asarray(anchor, dtype=float), np.asarray(offset, dtype=float)
    if (anchor.shape != (3,) or offset.shape != (2,) or not np.isfinite(anchor).all()
            or not np.isfinite(offset).all() or isinstance(yaw, bool)
            or not math.isfinite(yaw) or not -math.pi <= yaw <= math.pi):
        raise ValueError("Expected a finite 3D anchor, XY offset and yaw in [-pi, pi]")
    c, s = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:2, :2] = [[c, -s], [s, c]]
    matrix[:3, 3] = anchor + np.r_[offset, 0.] - matrix[:3, :3] @ anchor
    return matrix


def rotated_footprints(footprints, anchors, yaws):
    """Conservative AABBs of the rotated swept envelopes, before translation."""
    if any(set(mapping) != set(MODULES) for mapping in (footprints, anchors, yaws)):
        raise ValueError("All 13 installation footprints, anchors and yaws are required")
    output = {}
    for name in MODULES:
        box = np.asarray(footprints[name], dtype=float)
        if box.shape != (2, 2) or not np.isfinite(box).all() or np.any(box[1] <= box[0]):
            raise ValueError(f"Invalid footprint for {name}")
        matrix = planar_transform(anchors[name], [0., 0.], yaws[name])
        corners = np.array([[x, y, 0.] for x in box[:, 0] for y in box[:, 1]])
        points = corners @ matrix[:3, :3].T + matrix[:3, 3]
        output[name] = [points[:, :2].min(0).tolist(), points[:, :2].max(0).tolist()]
    return output


def validate_pose_placements(footprints, placements, anchors, yaws, obstacles=(), *, margin=.005, gap=.008):
    return validate_placements(rotated_footprints(footprints, anchors, yaws), placements,
                               obstacles, margin=margin, gap=gap)


def validate_layout_record(record):
    """Validate old translation manifests and explicit planar-pose manifests."""
    schema = record.get("schema")
    if (schema not in {SCHEMA, POSE_SCHEMA} or record.get("initial_states_randomized") is not False
            or set(record.get("offsets_body_xy_m", {})) != set(MODULES)):
        raise ValueError("Invalid independent layout state/placement contract")
    footprints = record["original_footprints_body_xy_m"]
    if schema == POSE_SCHEMA:
        if (record.get("orientations_randomized") is not True
                or record.get("rotation_axis") != "task_box_base/+Z"
                or set(record.get("modules", {})) != set(MODULES)):
            raise ValueError("Invalid independent layout yaw contract")
        anchors = {k: v["original_anchor_body_m"] for k, v in record["modules"].items()}
        footprints = rotated_footprints(footprints, anchors, record["installation_yaw_rad"])
    elif record.get("orientations_randomized") is not False:
        raise ValueError("Invalid independent layout state/placement contract")
    return validate_placements(footprints, record["offsets_body_xy_m"],
                               record["fixed_obstacles_body_xy_m"],
                               margin=record["deck_edge_margin_m"], gap=record["minimum_module_gap_m"])


def sample_pose_placements(footprints, anchors, seed, obstacles=(), *, yaw_range_deg=180.,
                           margin=.005, gap=.008, max_restarts=100, max_attempts=400):
    """Bounded continuous XY/yaw rejection sampling; not a task-feasibility test.

    Large units go first. Each rotation uses an enclosing axis-aligned swept
    box, which is conservative and can reject otherwise feasible tight fits.
    The resulting full layouts are not uniformly distributed.
    """
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("Seed must be a nonnegative integer")
    if (isinstance(yaw_range_deg, bool) or not math.isfinite(yaw_range_deg)
            or not 0 <= yaw_range_deg <= 180):
        raise ValueError("Yaw half-range must be in [0, 180] degrees")
    if (not all(math.isfinite(x) and x >= 0 for x in (margin, gap))
            or any(isinstance(x, bool) or not isinstance(x, int) or x <= 0
                   for x in (max_restarts, max_attempts))):
        raise ValueError("Invalid clearance or retry budget")
    bounds = rotated_footprints(footprints, anchors, dict.fromkeys(MODULES, 0.))
    obstacles = [np.asarray(b, dtype=float) for b in obstacles]
    if any(b.shape != (2, 2) or not np.isfinite(b).all() or np.any(b[1] <= b[0]) for b in obstacles):
        raise ValueError("Invalid fixed deck obstacle")
    rng = random.Random(f"{POSE_SCHEMA}:{seed}")
    limit = math.radians(yaw_range_deg)
    for restart in range(max_restarts):
        order = sorted(MODULES, key=lambda n: -float(np.prod(np.diff(bounds[n], axis=0))) * rng.uniform(.9, 1.1))
        placements, yaws, placed = {}, {}, []
        for name in order:
            box = np.asarray(footprints[name])
            corners = np.array([[x, y, 0.] for x in box[:, 0] for y in box[:, 1]])
            for _ in range(max_attempts):
                yaw = rng.uniform(-limit, limit)
                matrix = planar_transform(anchors[name], [0., 0.], yaw)
                points = (corners @ matrix[:3, :3].T + matrix[:3, 3])[:, :2]
                rotated = np.array([points.min(0), points.max(0)])
                low = np.array([margin, margin]) - rotated[0]
                high = np.array([.45-margin, .55-margin]) - rotated[1]
                if np.any(high < low):
                    continue
                offset = np.array([rng.uniform(low[i], high[i]) for i in range(2)])
                rectangle = rotated + offset
                if all(separated(rectangle, other, gap) for other in (*obstacles, *placed)):
                    placements[name], yaws[name] = offset.tolist(), yaw
                    placed.append(rectangle)
                    break
            else:
                break
        if len(placements) == len(MODULES):
            validate_pose_placements(footprints, placements, anchors, yaws, obstacles, margin=margin, gap=gap)
            return ({n: placements[n] for n in MODULES}, {n: yaws[n] for n in MODULES}, restart+1)
    raise ValueError("No disjoint XY/yaw layout found within the sampling budget")


def normalize_geometry_order(root):
    """Keep own geoms before child bodies for index-based render metadata.

    MuJoCo compiles a body's geoms before any child's geoms, regardless of
    their XML order. The existing renderer metadata reader walks XML order.
    New geoms appended after child bodies would bind to the wrong mesh.
    Preserve joint order, body order, parentage and every node attribute.
    """
    for container in root.iter():
        if container.tag in {"body", "worldbody"}:
            children = list(container)
            container[:] = ([child for child in children if child.tag not in {"body", "frame"}]
                            + [child for child in children if child.tag in {"body", "frame"}])


def separated(a, b, gap):
    a, b = np.asarray(a), np.asarray(b)
    return bool(np.any(a[1] + gap <= b[0]+1e-10) or np.any(b[1] + gap <= a[0]+1e-10))


def validate_placements(footprints, placements, obstacles=(), *, margin=.005, gap=.008):
    if not all(math.isfinite(value) and value >= 0 for value in (margin, gap)):
        raise ValueError("Placement margin and gap must be finite and nonnegative")
    obstacles = [np.asarray(box, dtype=float) for box in obstacles]
    if any(box.shape != (2, 2) or not np.isfinite(box).all() or np.any(box[1] <= box[0])
           for box in obstacles):
        raise ValueError("Invalid fixed deck obstacle")
    if set(footprints) != set(MODULES) or set(placements) != set(MODULES):
        raise ValueError("All 13 independent installation units must be specified")
    placed = {}
    for name in MODULES:
        bounds = np.asarray(footprints[name], dtype=float)
        offset = np.asarray(placements[name], dtype=float)
        if (bounds.shape != (2, 2) or offset.shape != (2,) or not np.isfinite(bounds).all()
                or not np.isfinite(offset).all() or np.any(bounds[1] <= bounds[0])):
            raise ValueError(f"Invalid footprint or offset for {name}")
        rectangle = bounds + offset
        if np.any(rectangle[0] < margin-1e-10) or np.any(rectangle[1] > np.array([.45, .55])-margin+1e-10):
            raise ValueError(f"{name} leaves the usable deck")
        for other, previous in placed.items():
            if not separated(rectangle, previous, gap):
                raise ValueError(f"Overlapping installation envelopes: {name}, {other}")
        for obstacle in obstacles:
            if not separated(rectangle, obstacle, gap):
                raise ValueError(f"{name} overlaps a fixed deck feature")
        placed[name] = rectangle
    return {name: rectangle.tolist() for name, rectangle in placed.items()}


def sample_placements(footprints, seed, obstacles=(), *, margin=.005, gap=.008,
                      max_restarts=300):
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("Seed must be a nonnegative integer")
    if not all(math.isfinite(value) and value >= 0 for value in (margin, gap)):
        raise ValueError("Placement margin and gap must be finite and nonnegative")
    if set(footprints) != set(MODULES):
        raise ValueError("Expected the 13 independent installation units")
    bounds = {name: np.asarray(value, dtype=float) for name, value in footprints.items()}
    for name, box in bounds.items():
        if box.shape != (2, 2) or not np.isfinite(box).all() or np.any(box[1] <= box[0]):
            raise ValueError(f"Invalid footprint for {name}")
        if np.any(box[1]-box[0] > np.array([.45, .55])-2*margin):
            raise ValueError(f"{name} is larger than the usable deck")
    # Pack into maximal free rectangles. Uniform rejection sampling strands
    # empty slivers around the long drawer and cannot reliably fit all units.
    obstacles = [np.asarray(box, dtype=float) for box in obstacles]
    if any(box.shape != (2, 2) or not np.isfinite(box).all() or np.any(box[1] <= box[0])
           for box in obstacles):
        raise ValueError("Invalid fixed deck obstacle")
    rng = random.Random(f"{SCHEMA}:{seed}")
    ordered = sorted(MODULES, key=lambda name: -float(np.prod(bounds[name][1]-bounds[name][0])))

    def subtract(free, block):
        x0, y0, x1, y1 = block
        result = []
        for a, b, c, d in free:
            if x1 <= a or c <= x0 or y1 <= b or d <= y0:
                result.append((a, b, c, d))
                continue
            if a < x0:
                result.append((a, b, x0, d))
            if x1 < c:
                result.append((x1, b, c, d))
            if b < y0:
                result.append((a, b, c, y0))
            if y1 < d:
                result.append((a, y1, c, d))
        return [r for i, r in enumerate(result)
                if not any(j != i and s[0] <= r[0] and s[1] <= r[1]
                           and s[2] >= r[2] and s[3] >= r[3] and (s != r or j < i)
                           for j, s in enumerate(result))]

    for restart in range(max_restarts):
        order = sorted(ordered, key=lambda name: -float(
            np.prod(bounds[name][1]-bounds[name][0])) * rng.uniform(.65, 1.35))
        half = gap/2
        free = [(margin-half, margin-half, .45-margin+half, .55-margin+half)]
        for box in obstacles:
            free = subtract(free, (*list(box[0]-half), *list(box[1]+half)))
        output = {}
        for name in order:
            box = bounds[name]
            width, height = box[1]-box[0]
            w, h = width+gap, height+gap
            candidates = []
            for a, b, c, d in free:
                if c-a+1e-12 < w or d-b+1e-12 < h:
                    continue
                score = min(c-a-w, d-b-h) + rng.uniform(0, .02)
                for x, y in ((a, b), (c-w, b), (a, d-h), (c-w, d-h)):
                    candidates.append((score+rng.uniform(0, .005), x, y))
            if not candidates:
                break
            _, x, y = min(candidates)
            output[name] = [float(x+half-box[0, 0]), float(y+half-box[0, 1])]
            free = subtract(free, (x, y, x+w, y+h))
        if len(output) == len(MODULES):
            validate_placements(footprints, output, obstacles, margin=margin, gap=gap)
            return {name: output[name] for name in MODULES}, restart+1
    raise ValueError("No disjoint layout found within the sampling budget; adjust footprints or gap")
