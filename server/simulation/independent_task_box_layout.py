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
OPERATION_CLEARANCE_SCHEMA = "task-box-operation-clearance/1"
OPERATION_MODULES = (*PICKUPS, *(f"{kind}_insert_40p5" for kind in ("aviation", "guide", "cylindrical")))


def operation_footprints(anchors, yaws=None):
    """Reserve the tool, open fingers and 120 mm tangential withdrawal.

    These are conservative XY projections, including the measured 66 mm tool
    width and 170 mm fully open linkage span, with extra room for tracking.
    Operations are sequential: their reservations may overlap each other but
    must never contain another physical installation or fixed deck feature.
    The 40.1 mm alternatives remain obstacles, not scripted destinations.
    Full arm reachability and dynamic contacts require separate native checks.
    """
    if set(anchors) != set(MODULES):
        raise ValueError("All installation anchors are required for tool clearance")
    yaws = dict.fromkeys(MODULES, 0.) if yaws is None else yaws
    if set(yaws) != set(MODULES):
        raise ValueError("All installation yaws are required for tool clearance")
    result = {}
    for name in OPERATION_MODULES:
        anchor = np.asarray(anchors[name], dtype=float)
        matrix = planar_transform(anchor, [0., 0.], yaws[name])
        corners = np.array([[anchor[0]+x, anchor[1]+y, 0.]
                            for x in (-.04, .04) for y in (-.095, .095)])
        points = (corners @ matrix[:3, :3].T + matrix[:3, 3])[:, :2]
        if name not in PICKUPS:
            direction = matrix[:2, 0].copy()
            # Match PlugInsertTask's positive-deck-X tangential withdrawal.
            if direction[0] < 0:
                direction = -direction
            points = np.vstack((points, points + .12*direction))
        result[name] = np.array([points.min(0), points.max(0)])
    return result


def make_operational_clearance(footprints, anchors, obstacles=()):
    """Canonical, recomputable contract; old footprint-only layouts fail closed."""
    if set(footprints) != set(MODULES):
        raise ValueError("All installation footprints are required")
    operation_footprints(anchors)
    return dict(schema=OPERATION_CLEARANCE_SCHEMA, minimum_tool_gap_m=.005,
                tool_half_width_m=.04, open_linkage_half_span_m=.095,
                retract_distance_m=.12, scripted_target_diameter_mm=40.5,
                operational_modules=list(OPERATION_MODULES),
                scope="Sequential pickup/insertion/opening/withdrawal XY reservations versus all physical modules and fixed obstacles; full arm dynamics checked separately")


def validate_operational_placements(footprints, placements, anchors, yaws=None, obstacles=()):
    reservations = operation_footprints(anchors, yaws)
    yaws = dict.fromkeys(MODULES, 0.) if yaws is None else yaws
    physical = rotated_footprints(footprints, anchors, yaws)
    physical = {n: np.asarray(b)+placements[n] for n,b in physical.items()}
    for name, bounds in reservations.items():
        zone = bounds + placements[name]
        for other, box in physical.items():
            if other != name and not separated(zone, box, .005):
                raise ValueError(f"Tool operation clearance conflict: {name}, {other}")
        for box in obstacles:
            if not separated(zone, box, .005):
                raise ValueError(f"Tool operation clearance conflict: {name}, fixed deck feature")
    return {n: (b+placements[n]).tolist() for n,b in reservations.items()}


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
    """Validate translation/pose manifests against the current clearance policy."""
    schema = record.get("schema")
    if (schema not in {SCHEMA, POSE_SCHEMA} or record.get("initial_states_randomized") is not False
            or set(record.get("offsets_body_xy_m", {})) != set(MODULES)):
        raise ValueError("Invalid independent layout state/placement contract")
    if schema == POSE_SCHEMA:
        if (record.get("orientations_randomized") is not True
                or record.get("rotation_axis") != "task_box_base/+Z"
                or set(record.get("modules", {})) != set(MODULES)):
            raise ValueError("Invalid independent layout yaw contract")
    elif record.get("orientations_randomized") is not False:
        raise ValueError("Invalid independent layout state/placement contract")
    if (record["deck_edge_margin_m"] < .005 or record["minimum_module_gap_m"] < .008):
        raise ValueError("Layout requires at least 5 mm deck margin and 8 mm module clearance")
    footprints = record["original_footprints_body_xy_m"]
    anchors = {k: v["original_anchor_body_m"] for k, v in record.get("modules", {}).items()}
    expected = make_operational_clearance(footprints, anchors, record["fixed_obstacles_body_xy_m"])
    if record.get("operational_clearance") != expected:
        raise ValueError("Layout lacks current operational clearance; regenerate the layout and pool")
    validate_operational_placements(footprints, record["offsets_body_xy_m"], anchors,
                                   record.get("installation_yaw_rad"), record["fixed_obstacles_body_xy_m"])
    if schema == POSE_SCHEMA:
        footprints = rotated_footprints(footprints, anchors, record["installation_yaw_rad"])
    return validate_placements(footprints, record["offsets_body_xy_m"],
                               record["fixed_obstacles_body_xy_m"],
                               margin=record["deck_edge_margin_m"], gap=record["minimum_module_gap_m"])


def sample_pose_placements(footprints, anchors, seed, obstacles=(), *, yaw_range_deg=180.,
                           margin=.005, gap=.008, max_restarts=100, max_attempts=400,
                           operational_clearance=False):
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
    if operational_clearance:
        # Reuse vectorised XY rejection for a whole continuous orientation
        # proposal. Narrow long mechanisms near a deck axis keep useful space;
        # angles are still continuous and no fixed installation grid is used.
        for restart in range(max_restarts):
            yaws = {name: rng.uniform(-limit, limit) for name in MODULES}
            # Keep the low tool passages close to their two equivalent deck
            # headings. Uniformly rotating six broad reservations strands
            # unusable slivers on this small deck. Accepted angles are still
            # sampled continuously, including around the opposite heading.
            for name in OPERATION_MODULES:
                axis = rng.choice([v for v in (-math.pi, 0., math.pi) if -limit <= v <= limit])
                yaws[name] = rng.uniform(max(-limit, axis-.08), min(limit, axis+.08))
            for name in ("drawer", "locking_mechanism"):
                axis = rng.choice([v for v in (-math.pi, -math.pi/2, 0., math.pi/2, math.pi)
                                   if -limit <= v <= limit])
                yaws[name] = rng.uniform(max(-limit, axis-.08), min(limit, axis+.08))
            try:
                positions, _ = _sample_operational_placements(
                    footprints, anchors, rng.randrange(2**63), obstacles,
                    margin=margin, gap=gap, max_restarts=24, yaws=yaws)
            except ValueError:
                continue
            return positions, yaws, restart+1
        raise ValueError("No XY/yaw layout with tool operation clearance found within the sampling budget")
    for restart in range(max_restarts):
        order = sorted(MODULES, key=lambda n: -float(np.prod(np.diff(bounds[n], axis=0))) * rng.uniform(.9, 1.1))
        placements, yaws, placed = {}, {}, {}
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
                if all(separated(rectangle, other, gap) for other in (*obstacles, *placed.values())):
                    placements[name], yaws[name] = offset.tolist(), yaw
                    placed[name] = rectangle
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
                      max_restarts=300, operation_anchors=None):
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
    if operation_anchors is not None:
        return _sample_operational_placements(bounds, operation_anchors, seed, obstacles,
                                               margin=margin, gap=gap, max_restarts=max_restarts)
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


def _sample_operational_placements(bounds, anchors, seed, obstacles, *, margin, gap, max_restarts, yaws=None):
    """Vectorised bounded rejection, preserving continuous position diversity."""
    originals = bounds
    if yaws is not None:
        bounds = {n: np.asarray(b) for n,b in rotated_footprints(bounds, anchors, yaws).items()}
    operations = operation_footprints(anchors, yaws)
    rng = random.Random(f"{SCHEMA}:{OPERATION_CLEARANCE_SCHEMA}:{seed}")

    def free(boxes, other, clearance):
        return (np.any(boxes[:, 1]+clearance <= other[0]+1e-10, axis=1)
                | np.any(other[1]+clearance <= boxes[:, 0]+1e-10, axis=1))

    for restart in range(max_restarts):
        order = sorted(MODULES, key=lambda n: -float(np.prod(bounds[n][1]-bounds[n][0]))
                       * rng.uniform(.6, 1.4))
        placed, reserved, offsets = {}, {}, {}
        for name in order:
            box = bounds[name]
            low, high = margin-box[0], np.array([.45, .55])-margin-box[1]
            if np.any(high < low):
                break
            samples = np.array([[rng.uniform(low[i], high[i]) for i in range(2)] for _ in range(512)])
            # Edge positions leave room for long mechanisms; also sample along
            # edges continuously, rather than forcing a fixed installation grid.
            samples[:4] = [[x,y] for x in (low[0], high[0]) for y in (low[1], high[1])]
            samples[4:20:4, 0], samples[5:20:4, 0] = low[0], high[0]
            samples[6:20:4, 1], samples[7:20:4, 1] = low[1], high[1]
            rectangles = box[None, :, :] + samples[:, None, :]
            valid = np.ones(len(samples), dtype=bool)
            for other in (*obstacles, *placed.values()):
                valid &= free(rectangles, other, gap)
            for other in reserved.values():
                valid &= free(rectangles, other, .005)
            zones = None
            if name in operations:
                zones = operations[name][None, :, :]+samples[:, None, :]
                for other in (*obstacles, *placed.values()):
                    valid &= free(zones, other, .005)
            choices = np.flatnonzero(valid)
            if not len(choices):
                break
            if np.prod(box[1]-box[0]) > .01:
                edges = np.minimum(samples-low, high-samples).min(axis=1)
                choices = choices[edges[choices] <= edges[choices].min()+.002]
            chosen = rng.choice(choices.tolist())
            offsets[name] = samples[chosen].tolist()
            placed[name] = rectangles[chosen]
            if zones is not None:
                reserved[name] = zones[chosen]
        if len(offsets) == len(MODULES):
            validate_placements(bounds, offsets, obstacles, margin=margin, gap=gap)
            validate_operational_placements(originals, offsets, anchors, yaws, obstacles)
            return {name: offsets[name] for name in MODULES}, restart+1
    raise ValueError("No layout with tool operation clearance found within the sampling budget")
