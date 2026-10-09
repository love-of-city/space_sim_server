"""Offline, opt-in collision-proxy trials; never replaces a production model.

Build/validate with the separate Python MuJoCo environment. Run the generated
models through profile_simulation_runtime.py in the Basilisk environment.
No Python MuJoCo may be imported into that Basilisk process.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "model/SARM/platform/sarm_task_box_plugs.xml"
PLUGS = ("guide_plug", "aviation_plug")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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


def simplify_bores(root, folder):
    """Replace selected existing polygonal bore walls with backed thin boxes.

    Keep deck, hole mouths, bore floors, guide slots and all other triangles.
    Do not replace a hole by a filled cylinder or shrink its radius. Segment
    endpoints and axial extents are taken from CAD, rather than coarsening a
    2.5 mm pin-hole into an arbitrary low-sided polygon.
    """
    bus = root.find("./worldbody/body[@name='cubesat_bus']")
    surface = bus.find("flexcomp[@name='task_box_001_contact']")
    vertices, faces = mesh_read(SOURCE.parent / surface.get("file"))
    contact = surface.find("contact")
    bores = [(.28, .275, .0275), (.37, .22, .025),
             (.535, .275, .02025), (.535, .195, .02025)]
    bores += [(x, y, .0025) for x in (.5275, .5425) for y in (.1875, .2025)]
    removed = np.zeros(len(faces), dtype=bool)
    segments = {}
    for i, tri in enumerate(vertices[faces]):
        if np.ptp(tri[:, 2]) < 1e-6 or tri[:, 2].max() > 1e-7:
            continue
        xy = np.unique(tri[:, :2].round(7), axis=0)
        if len(xy) != 2:
            continue
        for j, (x, y, radius) in enumerate(bores):
            if np.max(abs(np.linalg.norm(tri[:, :2]-[x, y], axis=1)-radius)) > 1e-7:
                continue
            key = (j, *xy.reshape(-1))
            lo, hi = float(tri[:, 2].min()), float(tri[:, 2].max())
            if key in segments:
                old = segments[key]
                lo, hi = min(old[0], lo), max(old[1], hi)
            segments[key] = (lo, hi)
            removed[i] = True
            break
    if len(segments) < 60:
        raise ValueError("Unexpected bore CAD; not enough wall segments recognized")
    for i, (key, (lo, hi)) in enumerate(segments.items()):
        j, ax, ay, bx, by = key
        a, b = np.array([ax, ay]), np.array([bx, by])
        tangent = (b-a)/np.linalg.norm(b-a)
        normal = np.array([-tangent[1], tangent[0]])
        mid = (a+b)/2
        if np.dot(normal, mid-np.array(bores[j][:2])) < 0:
            tangent, normal = -tangent, -normal
        # The 10 um front extension matches the original flex radius. Backing
        # is into the surrounding solid, never across the bore opening.
        thickness = .001
        pos = mid+normal*thickness/2
        proxy(bus, f"trial_bore_{j}_segment_{i}", "box", [*pos, (lo+hi)/2],
              [np.linalg.norm(b-a)/2+1e-5, thickness/2+1e-5, (hi-lo)/2+1e-5],
              contact, np.arctan2(tangent[1], tangent[0]))
    path = folder / "task_box_remaining_surface.obj"
    mesh_write(path, vertices, faces[~removed])
    surface.set("file", str(path.resolve()))
    return dict(primitive_count=len(segments), replaced_triangles=int(removed.sum()),
                retained_triangles=int((~removed).sum()), bores=bores,
                note="Exact polygon wall segments; noncircular keyway faces remain CAD.")


def partition_surface(vertices, faces):
    """Assign whole triangles once; never cut, decimate, or fill a hole."""
    centers = np.array([[.28, .275], [.37, .22], [.535, .275], [.535, .195]])
    regions = np.full(len(faces), 4, dtype=int)
    for i, center in enumerate(centers):
        inside = np.max(np.linalg.norm(vertices[faces, :2]-center, axis=2), axis=1) <= .036
        regions[inside & (regions == 4)] = i
    return regions


def partition_bores(root, folder):
    """Keep exact CAD faces/radius/contact parameters, localize rigid flexes."""
    bus = root.find("./worldbody/body[@name='cubesat_bus']")
    surface = bus.find("flexcomp[@name='task_box_001_contact']")
    vertices, faces = mesh_read(SOURCE.parent / surface.get("file"))
    regions = partition_surface(vertices, faces)
    counts = []
    index = list(bus).index(surface)
    bus.remove(surface)
    for i in sorted(set(regions)):
        selected = faces[regions == i]
        path = folder/f"task_box_region_{i}.obj"
        mesh_write(path, vertices, selected)
        piece = copy.deepcopy(surface)
        piece.set("name", f"trial_task_box_region_{i}")
        piece.set("file", str(path.resolve()))
        bus.insert(index+len(counts), piece)
        counts.append(len(selected))
    if sum(counts) != len(faces):
        raise AssertionError("Partition lost or duplicated triangles")
    return dict(original_faces=len(faces), region_faces=counts,
                note="Every original triangle retained once; flex grouping may change contact reduction/order.")


def partition_plugs(root, folder, metadata):
    """Localize head, key/shaft and each pin without altering any triangle."""
    details = []
    for item in metadata:
        name = item["name"]
        body = root.find(f"./worldbody/body[@name='{name}']")
        surface = body.find("flexcomp")
        vertices, faces = mesh_read(SOURCE.parent / surface.get("file"))
        z = vertices[:, 2]+item["bus_center_m"][2]
        regions = np.full(len(faces), "shaft", dtype=object)
        regions[np.min(z[faces], axis=1) >= -1e-7] = "head"
        if name == "guide_plug":
            xy = vertices[:, :2]+np.asarray(item["bus_center_m"][:2])-[.28, .275]
            key = (regions != "head") & (np.max(np.linalg.norm(xy, axis=1)[faces], axis=1) > .0203)
            regions[key] = "key"
        else:
            circles = circle_groups(vertices[abs(z+.055) < 1e-7, :2])
            for i, (center, radius) in enumerate(circles):
                pin = (np.max(z[faces], axis=1) <= -.04+1e-7) & (
                    np.max(np.linalg.norm(vertices[faces, :2]-center, axis=2), axis=1) <= radius+1e-7)
                regions[pin] = f"pin{i}"
            if np.any((np.min(z[faces], axis=1) < -.04-1e-7) & (regions == "shaft")):
                raise ValueError("Unassigned pin triangles; CAD changed")
        index = list(body).index(surface)
        body.remove(surface)
        counts = {}
        for region in sorted(set(regions)):
            selected = faces[regions == region]
            path = folder/f"{name}_{region}_partition.obj"
            mesh_write(path, vertices, selected)
            piece = copy.deepcopy(surface)
            piece.set("name", f"trial_{name}_{region}_contact")
            piece.set("file", str(path.resolve()))
            body.insert(index+len(counts), piece)
            counts[region] = len(selected)
        if sum(counts.values()) != len(faces):
            raise AssertionError("Partition lost or duplicated triangles")
        details.append(dict(name=name, original_faces=len(faces), region_faces=counts,
                            note="Exact triangles; flex grouping/contact reduction may differ."))
    return details


def build(folder, suite="original"):
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    original = ET.parse(SOURCE).getroot()
    metadata = json.loads(SOURCE.with_suffix(".manifest.json").read_text())["plugs"]
    protected = [SOURCE, SOURCE.with_suffix(".manifest.json"),
                 ROOT/"model/task_box/free_plugs/configuration.json"]
    report = dict(schema="task-box-proxy-trial/1", production_sha256={str(p): digest(p) for p in protected},
                  models={"baseline": str(SOURCE)}, models_to_cleanup=[], geometry={})
    variants = {"original": ("plug_proxy", "plug_and_bore_proxy"),
                "precision": ("plug_proxy", "pins_retained", "exact_partition", "pins_retained_partition"),
                "local-pins": ("exact_plug_partition", "hybrid_pin_partition")}[suite]
    for variant in variants:
        root = copy.deepcopy(original)
        root.set("model", "SARM_diagnostic_"+variant)
        model = SOURCE.with_name("sarm_task_box_plugs_diag_"+folder.name+"_"+variant+".xml")
        if model.exists():
            raise FileExistsError(model)
        subdir = folder/variant
        subdir.mkdir()
        details = {}
        if variant not in ("exact_partition", "exact_plug_partition"):
            details["plugs"] = simplify_plugs(root, subdir, metadata,
                preserve_pins=variant.startswith("pins_retained") or variant == "hybrid_pin_partition")
        if variant == "plug_and_bore_proxy":
            details["bores"] = simplify_bores(root, subdir)
        if variant in ("exact_partition", "pins_retained_partition"):
            details["partition"] = partition_bores(root, subdir)
        if variant in ("exact_plug_partition", "hybrid_pin_partition"):
            details["plug_partition"] = partition_plugs(root, subdir, metadata)
        # Preserving explicit inertials and zero-mass proxies keeps inferred
        # masses out of this experiment. Compiled values are checked separately.
        ET.ElementTree(root).write(model, encoding="utf-8", xml_declaration=True)
        report["models"][variant] = str(model)
        report["models_to_cleanup"].append(str(model))
        report["geometry"][variant] = details
    (folder/"manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


def validate(folder):
    import mujoco as mj
    import sys
    sys.path.insert(0, str(ROOT/"tools"))
    from validate_task_box_free_plugs import pose
    report = json.loads((folder/"manifest.json").read_text())
    metadata = json.loads(SOURCE.with_suffix(".manifest.json").read_text())["plugs"]
    baseline = mj.MjModel.from_xml_path(str(SOURCE))
    angle = np.arctan2(.5469441386039887, .8371691043312224)
    results = {}
    for variant, path in report["models"].items():
        m = mj.MjModel.from_xml_path(path)
        # These checks include *all* bodies, not only the newly created proxies.
        preserved = {}
        for field in ("body_mass", "body_inertia", "body_ipos", "body_iquat", "dof_damping",
                      "dof_frictionloss", "dof_armature", "jnt_range", "actuator_ctrlrange",
                      "actuator_forcerange", "actuator_gear", "qpos0"):
            preserved[field] = bool(np.array_equal(getattr(m, field), getattr(baseline, field)))
        preserved["nq_nv_nu"] = (m.nq, m.nv, m.nu) == (baseline.nq, baseline.nv, baseline.nu)
        if not all(preserved.values()):
            raise AssertionError((variant, preserved))
        data = mj.MjData(m)
        mj.mj_forward(m, data)
        row = dict(preserved=preserved, initial_contacts=int(data.ncon),
                   min_initial_distance_m=min((float(c.dist) for c in data.contact), default=None),
                   nflex=m.nflex, ngeom=m.ngeom, cases={})
        for item in metadata:
            name = item["name"]
            bid = m.body(name).id
            geomids = set(np.flatnonzero(m.geom_bodyid == bid))
            flexids = {i for i in range(m.nflex) if np.all(
                m.flex_vertbodyid[m.flex_vertadr[i]:m.flex_vertadr[i]+m.flex_vertnum[i]] == bid)}
            target = [.535, .275 if name == "guide_plug" else .195, 0]
            for label, rotation, lift, offset in (
                ("aligned_above", angle, .081, 0), ("aligned_partial", angle, .02, 0),
                ("aligned_seated", angle, .0001, 0), ("wrong_angle", angle+.4, .0001, 0),
                ("wrong_position", angle, .0001, .004),
            ):
                mj.mj_resetData(m, data)
                pose(m, data, item, np.array(target)+[offset, 0, 0], rotation, lift)
                mj.mj_forward(m, data)
                distances, forces, pairs = [], [], []
                for i, c in enumerate(data.contact):
                    if not (set(c.geom)&geomids or set(c.flex)&flexids):
                        continue
                    distances.append(float(c.dist))
                    force = np.zeros(6)
                    mj.mj_contactForce(m, data, i, force)
                    forces.append(float(force[0]))
                    pairs.append([mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, int(g)) if g >= 0
                                  else mj.mj_id2name(m, mj.mjtObj.mjOBJ_FLEX, int(f))
                                  for g, f in zip(c.geom, c.flex)])
                row["cases"][name+"/"+label] = dict(contacts=len(distances),
                    min_distance_m=min(distances, default=None), max_normal_force_N=max(forces, default=0),
                    sample_pairs=pairs[:3], finite=bool(np.isfinite(data.qacc).all()),
                    warnings=data.warning.number.tolist())
        results[variant] = row
        print(variant, "initial contacts:", row["initial_contacts"], flush=True)
        print({k: (v["contacts"], v["min_distance_m"]) for k,v in row["cases"].items()}, flush=True)
    output = dict(mujoco_version=mj.__version__, models=results,
                  scope="Static geometric regression only; not a gripper-driven insertion acceptance.")
    (folder/"geometry-validation.json").write_text(json.dumps(output, indent=2), encoding="utf-8")


def cleanup(folder):
    report = json.loads((folder/"manifest.json").read_text())
    for path, expected in report["production_sha256"].items():
        if digest(path) != expected:
            raise RuntimeError(f"Production file changed during trial: {path}")
    for name in report["models_to_cleanup"]:
        path = Path(name).resolve()
        if path.parent != SOURCE.parent or not path.name.startswith("sarm_task_box_plugs_diag_"):
            raise ValueError("Refusing to clean a non-diagnostic model")
        if path.exists():
            path.unlink()
    print("Diagnostic XMLs removed; production hashes unchanged. Reports and generated mesh data retained.")


def contact_motion(folder, selected=None):
    """Four-second free approaches in separate MuJoCo, NOT native throughput."""
    import mujoco as mj
    import sys
    sys.path.insert(0, str(ROOT/"tools"))
    from validate_task_box_free_plugs import pose
    report = json.loads((folder/"manifest.json").read_text())
    variants = selected.split(",") if selected else list(report["models"])
    if len(set(variants)) != len(variants) or any(v not in report["models"] for v in variants):
        raise ValueError("Select distinct known models")
    metadata = json.loads(SOURCE.with_suffix(".manifest.json").read_text())["plugs"]
    angle = np.arctan2(.5469441386039887, .8371691043312224)
    results = {}
    output = folder/"contact-motion.json"
    if output.exists():
        raise FileExistsError(output)
    for variant in variants:
        path = report["models"][variant]
        model = mj.MjModel.from_xml_path(path)
        data = mj.MjData(model)
        cases = {}
        for item in metadata:
            name = item["name"]
            body = model.body(name).id
            geomids = set(np.flatnonzero(model.geom_bodyid == body))
            flexids = {i for i in range(model.nflex) if np.all(
                model.flex_vertbodyid[model.flex_vertadr[i]:model.flex_vertadr[i]+model.flex_vertnum[i]] == body)}
            joint = model.joint(name+"_free").id
            q, v = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
            target = [.535, .275 if name == "guide_plug" else .195, 0]
            for label, rotation in (("aligned", angle), ("misaligned", angle+.4)):
                mj.mj_resetData(model, data)
                pose(model, data, item, target, rotation, .081)
                data.qvel[v+2] = -.03
                first, peak, max_contacts, finite = None, 0., 0, True
                for _ in range(round(4/model.opt.timestep)):
                    mj.mj_step(model, data)
                    finite = finite and bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
                    count = 0
                    for i, c in enumerate(data.contact):
                        if not (set(c.geom)&geomids or set(c.flex)&flexids):
                            continue
                        count += 1
                        force = np.zeros(6)
                        mj.mj_contactForce(model, data, i, force)
                        peak = max(peak, float(force[0]))
                        if force[0] > 1e-6 and first is None:
                            first = dict(time_s=float(data.time), center_z_m=float(data.qpos[q+2]))
                    max_contacts = max(max_contacts, count)
                row = dict(simulated_s=float(data.time), first_contact=first,
                           peak_normal_force_N=peak, max_contacts=max_contacts,
                           final_center_m=data.qpos[q:q+3].tolist(), finite=finite,
                           warnings=data.warning.number.tolist(),
                           depth_only_reached=bool(abs(data.qpos[q+2]-item["bus_center_m"][2]) < .0005))
                cases[name+"/"+label] = row
                print(variant, name, label, row, flush=True)
        results[variant] = cases
        output.write_text(json.dumps(dict(mujoco_version=mj.__version__, models=results,
            scope="4 s free approach at initial 0.03 m/s, no pose overwrite/IK/grasp. XML implicitfast, not native RKF45. Depth alone is not insertion acceptance."),
            indent=2), encoding="utf-8")


def benchmark(folder, python, adapter, scene, repeats, long_duration, selected=None, motion="hold"):
    """Serial native-only trials, rotating order, with no native timer callbacks."""
    folder = folder.resolve()
    report = json.loads((folder/"manifest.json").read_text())
    python, adapter, scene = python.resolve(), adapter.resolve(), scene.resolve()
    for path, expected in report["production_sha256"].items():
        if digest(path) != expected:
            raise RuntimeError(f"Production file changed during trial: {path}")
    env = os.environ.copy()
    base = python.parent
    env["PATH"] = os.pathsep.join(str(p) for p in (
        base, base/"Library/mingw-w64/bin", base/"Library/usr/bin",
        base/"Library/bin", base/"Scripts")) + os.pathsep + env.get("PATH", "")
    variants = selected.split(",") if selected else list(report["models"])
    if "baseline" not in variants or len(set(variants)) != len(variants) or any(
            v not in report["models"] for v in variants):
        raise ValueError("Select distinct known models, including baseline")
    summary_path = folder/("runtime-summary.json" if motion == "hold" else f"runtime-summary-{motion}.json")
    if summary_path.exists():
        raise FileExistsError(summary_path)
    trials = [(r+1, v, 2.0) for r in range(repeats)
              for v in variants[r % len(variants):]+variants[:r % len(variants)]]
    if long_duration:
        trials += [(0, v, long_duration) for v in variants]
    rows = []
    for repeat, variant, duration in trials:
        state = ROOT/"run/scene_runtime.json"
        if state.exists():
            phase = json.loads(state.read_text(encoding="utf-8-sig")).get("phase")
            if phase in {"launching", "starting_renderer", "starting_simulation", "running"}:
                raise RuntimeError("Stop the live scene before running CPU benchmarks.")
        stem = f"runtime-{variant}-repeat{repeat}-{duration:g}s"
        if motion != "hold":
            stem += "-"+motion
        output = folder/(stem+".json")
        log = folder/(stem+".log")
        if output.exists() or log.exists():
            raise FileExistsError(f"Use a fresh folder; refusing to overwrite {stem}")
        cmd = [str(python), str(ROOT/"tools/profile_simulation_runtime.py"),
               "--adapter-root", str(adapter), "--scene-instance", str(scene),
               "--model-path", report["models"][variant], "--output", str(output),
               "--duration", str(duration), "--mj-threads", "0", "--motion", motion]
        print("Starting", stem, flush=True)
        with log.open("x", encoding="utf-8") as stream:
            subprocess.run(cmd, cwd=ROOT, env=env, stdout=stream,
                           stderr=subprocess.STDOUT, check=True, timeout=300)
        result = json.loads(output.read_text())
        rows.append(dict(variant=variant, repeat=repeat, duration_sim_s=duration,
                         execute_wall_s=result["execute_wall_s"],
                         rtf=result["compute_real_time_factor"],
                         probe=result["mjscene_probe"], output=str(output)))
        print(rows[-1], flush=True)
    medians = {v: statistics.median(row["execute_wall_s"] for row in rows
                                    if row["variant"] == v and row["repeat"] > 0)
               for v in variants}
    summary = dict(rows=rows, median_wall_s=medians,
                   speedup={v: medians["baseline"]/value for v, value in medians.items()},
                   median_derived_rtf={v: 2/value for v, value in medians.items()},
                   scope="Offline compute; excludes UE/network/recording/startup; not insertion acceptance.")
    summary["motion"] = motion
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("build", "validate", "contact-motion", "benchmark", "cleanup"))
    parser.add_argument("--folder", type=Path, required=True)
    parser.add_argument("--suite", choices=("original", "precision", "local-pins"), default="original")
    parser.add_argument("--variants", help="Comma-separated model selection for benchmark, including baseline")
    parser.add_argument("--motion", choices=("hold", "linear_x", "linear_y", "linear_z", "roll", "pitch", "yaw"), default="hold")
    parser.add_argument("--native-python", type=Path)
    parser.add_argument("--adapter-root", type=Path)
    parser.add_argument("--scene-instance", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--long-duration", type=float, default=13)
    args = parser.parse_args()
    if args.command == "benchmark":
        if not all((args.native_python, args.adapter_root, args.scene_instance)):
            parser.error("benchmark requires --native-python, --adapter-root and --scene-instance")
        if args.repeats < 1 or not np.isfinite(args.long_duration) or args.long_duration < 0:
            parser.error("repeats must be positive and long-duration must be finite and nonnegative")
        benchmark(args.folder, args.native_python, args.adapter_root, args.scene_instance,
                  args.repeats, args.long_duration, args.variants, args.motion)
    elif args.command == "build":
        build(args.folder, args.suite)
    elif args.command == "contact-motion":
        contact_motion(args.folder, args.variants)
    else:
        {"validate": validate, "cleanup": cleanup}[args.command](args.folder)
