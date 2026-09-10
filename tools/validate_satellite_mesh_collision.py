"""Validate the isolated original-triangle candidate, without starting UE/platform.

Run this file with MuJoCo 3.7 in its dedicated venv. --native-python runs a
SEPARATE Basilisk process, never mixing its DLL with Python MuJoCo.
"""
from __future__ import annotations
import argparse
from collections import Counter
import ctypes
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from types import ModuleType
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FOLDER = ROOT / "model/ground_validation_satellite/mesh_collision_trial"


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")


def native_probe(folder, output):
    # Import only Basilisk's MuJoCo in this interpreter.
    import numpy as np
    import Basilisk
    from Basilisk.simulation import mujoco
    lib = ctypes.CDLL(str(Path(Basilisk.__path__[0]) / "mujoco.dll"))
    lib.mj_versionString.restype = ctypes.c_char_p
    version = lib.mj_versionString().decode()
    if version != "3.7.0":
        raise RuntimeError(f"Expected runtime-matched MuJoCo 3.7.0; got {version}")
    model_path = folder / "sarm_mesh_collision.xml"
    xml = ET.parse(model_path).getroot()
    files = sorted({str((folder / e.get("file")).resolve()) for selector in
                    ("./asset/mesh[@file]", ".//flexcomp[@file]") for e in xml.findall(selector)})
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "backend"))
    # Existing native SARM scenario imports Python 'mujoco' for offline analysis
    # only. Prevent a second, incompatible DLL from loading in this process.
    sys.modules["mujoco"] = ModuleType("mujoco")
    source = ROOT / "model/SARM/platform/scenarios/scenario_sarm_grasp.py"
    spec = importlib.util.spec_from_file_location("native_triangle_trial", source)
    native = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = native
    spec.loader.exec_module(native)
    from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, capture_target
    target = capture_target(DEFAULT_TEMPLATE)
    native.MODEL_PATH = model_path
    native.TARGET_POS = np.asarray(target.position_m)
    native.TARGET_QUAT = np.asarray(target.orientation_wxyz)
    native._load_scene = lambda: mujoco.MJScene.fromFile(str(model_path), files=files)
    native.JointTrajectoryPublisher.reference = classmethod(lambda cls, t: (native.PREGRASP.copy(), np.zeros(8)))
    start = time.perf_counter()
    sim, scene, models, recorders = native._build_simulation()
    compile_seconds = time.perf_counter() - start
    native._initialize_state(sim, scene)
    hinge = scene.getBody("satellite_outer_panel").getScalarJoint("outer_panel_hinge")
    hinge.setVelocity(.1)
    sim.ConfigureStopTime(native.macros.sec2nano(.01))
    start = time.perf_counter()
    sim.ExecuteSimulation()
    wall = time.perf_counter() - start
    qpos = np.asarray(scene.stateOutMsg.read().qpos).reshape(-1)
    report = {"mujoco_version": version, "model": model_path.name,
              "compile_and_build_wall_s": compile_seconds, "simulation_duration_s": .01,
              "step_wall_s": wall, "measured_real_time_factor": .01 / wall,
              "bodies": list(scene.getBodyNames()), "body_count": len(list(scene.getBodyNames())),
              "qpos_size": len(qpos), "qpos_finite": bool(np.isfinite(qpos).all()),
              "arm_control_channels": len(native.ACTUATORS), "attitude_control_enabled": sim.attitude_control.enabled,
              "hinge_initial_rate_rad_s": .1, "hinge_final_rate_rad_s": float(hinge.stateDotOutMsg.read().state),
              "hinge_final_angle_rad": float(hinge.stateOutMsg.read().state),
              "scope": "Native SARM+target MJScene + existing arm/attitude controller smoke, NOT orbital/UE/browser acceptance"}
    assert report["body_count"] == 15 and report["qpos_size"] == 26 and report["qpos_finite"]
    assert report["arm_control_channels"] == 8 and report["attitude_control_enabled"]
    write_json(output, report)
    print(json.dumps(report), flush=True)


def geometry_report(model, folder, manifest):
    import mujoco as m
    import numpy as np
    from build_satellite_mesh_collision import load_obj
    records = {r["flex"]: r for r in manifest["surfaces"]}
    cache = {}
    result = []
    for fid in range(model.nflex):
        name = m.mj_id2name(model, m.mjtObj.mjOBJ_FLEX, fid)
        record = records[name]
        file = ROOT / record["source_obj"]
        if file not in cache:
            cache[file] = load_obj(file)
        v, f = cache[file]
        keep = np.ones(len(f), dtype=bool)
        keep[record["removed_float32_zero_area_face_indices"]] = False
        f = f[keep]
        scale = np.asarray([float(x) for x in record["scale"].split()])
        quat = np.asarray([float(x) for x in record["quat"].split()]); quat /= np.linalg.norm(quat)
        rotation = np.zeros(9); m.mju_quat2Mat(rotation, quat)
        expected = (v * scale) @ rotation.reshape(3,3).T + np.asarray([float(x) for x in record["pos"].split()])
        va = int(model.flex_vertadr[fid]); vn = int(model.flex_vertnum[fid])
        ea = int(model.flex_elemdataadr[fid]); en = int(model.flex_elemnum[fid])
        compiled = model.flex_vert[va:va+vn]
        elements = model.flex_elem[ea:ea+3*en].reshape(-1,3)
        assert len(f) == en == record["retained_triangles"], name
        error = float(np.max(np.linalg.norm(expected[f] - compiled[elements], axis=2)))
        assert error < 1e-7, (name, error)
        assert bool(model.flex_rigid[fid])
        body_ids = set(model.flex_vertbodyid[va:va+vn].tolist())
        expected_body = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, record["body"])
        assert body_ids == {expected_body}
        result.append({"flex": name, "body": record["body"], "triangles": en, "max_triangle_vertex_error_m": error})
    return {"all_surfaces_compared": len(result), "all_retained_triangles_compared": sum(r["triangles"] for r in result),
            "max_triangle_vertex_error_m": max(r["max_triangle_vertex_error_m"] for r in result),
            "comparison": "Every retained source triangle vertex vs compiled collision triangle, in body coordinates; not sparse sampling",
            "source_surfaces": result}


def contacts_report(model, data):
    import mujoco as m
    import numpy as np
    pairs = Counter(); root_outer = 0; root_outer_maximum_force = 0.; maximum_force = 0.; minimum_distance = 0.
    for i, c in enumerate(data.contact):
        names, bodies = [], []
        for side in (0,1):
            fid, gid = int(c.flex[side]), int(c.geom[side])
            if fid >= 0:
                names.append(m.mj_id2name(model, m.mjtObj.mjOBJ_FLEX, fid))
                body = int(model.flex_vertbodyid[model.flex_vertadr[fid]])
            else:
                names.append(m.mj_id2name(model, m.mjtObj.mjOBJ_GEOM, gid))
                body = int(model.geom_bodyid[gid])
            bodies.append(m.mj_id2name(model, m.mjtObj.mjOBJ_BODY, body))
        pairs[tuple(names)] += 1
        if set(bodies) == {"capture_target", "satellite_outer_panel"}:
            root_outer += 1
        minimum_distance = min(minimum_distance, float(c.dist))
        force = np.zeros(6); m.mj_contactForce(model, data, i, force)
        assert np.isfinite(force).all()
        maximum_force = max(maximum_force, float(abs(force[0])))
        if set(bodies) == {"capture_target", "satellite_outer_panel"}:
            root_outer_maximum_force = max(root_outer_maximum_force, float(abs(force[0])))
    return {"contacts": int(data.ncon), "root_outer_contacts": root_outer,
            "root_outer_maximum_normal_force_N": root_outer_maximum_force,
            "minimum_contact_distance_m": minimum_distance, "maximum_normal_force_N": maximum_force,
            "pairs": [{"surfaces": list(p), "count": n} for p,n in pairs.most_common()]}


def validate(folder, output, native_python=None):
    import mujoco as m
    import numpy as np
    from build_satellite_mesh_collision import digest
    if m.__version__ != "3.7.0":
        raise RuntimeError("Use the isolated MuJoCo 3.7.0 environment matching Basilisk, not the CAD venv's 3.12")
    manifest = json.loads((folder / "manifest.json").read_text())
    for path, value in manifest["source_hashes"].items():
        assert digest(ROOT / path) == value, f"Source changed: {path}"
    for path, value in manifest["output_sha256"].items():
        assert digest(folder / path) == value, f"Candidate changed since generation: {path}"
    start=time.perf_counter();model=m.MjModel.from_xml_path(str(folder / "satellite_mesh_collision.xml"))
    compile_s=time.perf_counter()-start
    assert model.nq == 8 and model.nv == 7 and model.nu == 0 and model.nbody == 4
    assert model.nflex == 208 and model.nflexelem == manifest["retained_triangle_instances"]
    geometry=geometry_report(model,folder,manifest)
    print('All source triangles compared; maximum vertex error (m):',geometry['max_triangle_vertex_error_m'],flush=True)
    data=m.MjData(model);jid=m.mj_name2id(model,m.mjtObj.mjOBJ_JOINT,"outer_panel_hinge")
    sweeps=[]
    for angle in (0,30,90,135,180,190,200,225,270,360):
        m.mj_resetData(model,data);data.qpos[model.jnt_qposadr[jid]]=math.radians(angle)
        start=time.perf_counter();m.mj_forward(model,data);wall=time.perf_counter()-start
        sample={"angle_deg":angle,"forward_wall_s":wall,**contacts_report(model,data)}
        sweeps.append(sample)
        print('Sweep',angle,'contacts',sample['contacts'],'body/outer',sample['root_outer_contacts'],'wall',wall,flush=True)
    assert any(s["root_outer_contacts"] > 0 and s["root_outer_maximum_normal_force_N"] > 0 for s in sweeps)
    # Record, do NOT conceal, the fact that literal preview hinge surfaces touch
    # at zero pose and resist otherwise free hinge motion.
    neutral=sweeps[0]
    original_radii=model.flex_radius.copy()
    model.flex_radius[:]=1e-7
    m.mj_resetData(model,data);m.mj_forward(model,data)
    smaller_skin={"radius_per_surface_m":1e-7,**contacts_report(model,data)}
    model.flex_radius[:]=original_radii
    m.mj_resetData(model,data);data.qvel[model.jnt_dofadr[jid]]=.1
    start=time.perf_counter()
    steps=10
    for _ in range(steps):
        m.mj_step(model,data)
    step_wall=time.perf_counter()-start
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    warnings={str(m.mjtWarning(i)).split('.')[-1]:int(w.number) for i,w in enumerate(data.warning) if w.number}
    report={"schema":"triangle-collision-validation/1","native_matched_python_mujoco":m.__version__,
            "candidate_sha256":manifest["output_sha256"],"compile_wall_s":compile_s,
            "model":{"nbody":model.nbody,"nq":model.nq,"nv":model.nv,"nu":model.nu,"nflex":model.nflex,
                     "collision_triangles":model.nflexelem,"collision_vertices":model.nflexvert},
            "geometry":geometry,"contact_radius_per_surface_m":manifest["contact_radius_per_surface_m"],
            "source_triangle_instances":manifest["triangle_instances"],
            "removed_zero_area_triangle_instances":manifest["triangle_instances"]-manifest["retained_triangle_instances"],
            "removed_original_area_m2":manifest["total_removed_original_area_m2"],
            "neutral_pose_contacts":neutral,"smaller_skin_neutral_probe":smaller_skin,"angle_sweep":sweeps,
            "sweep_is_static_overlap_detection_not_dynamic_nonpenetration_proof":True,
            "passive_smoke":{"simulation_duration_s":steps*model.opt.timestep,"wall_s":step_wall,
                             "measured_real_time_factor":steps*model.opt.timestep/step_wall,
                             "hinge_initial_rate_rad_s":.1,"hinge_final_rate_rad_s":float(data.qvel[model.jnt_dofadr[jid]]),
                             "hinge_final_angle_rad":float(data.qpos[model.jnt_qposadr[jid]]),"finite":True,"warnings":warnings},
            "conversion_verified":True,"ready_for_production":False,"default_scene_changed":False,
            "blocking_findings":["Preview-only split hinge surfaces/nearby moving hardware generate contact at neutral pose; engineering articulation/contact geometry must be resolved, not silently excluded",
                                 "Full-resolution triangle contact is substantially slower than the 500 Hz wall-time budget on this host",
                                 "Surface contact has an explicit skin, not CAD-exact volume contact; no high-speed tunnelling guarantee",
                                 "Mass/inertia and true hinge stops remain uncalibrated"],
            "ue_browser_and_long_duration_tested":False}
    write_json(output,report)  # Preserve geometric findings even if native integration fails.
    if native_python:
        native_out=output.with_name("native_validation.json")
        command=[str(native_python),str(Path(__file__).resolve()),"--folder",str(folder),"--native-only","--output",str(native_out)]
        flags=subprocess.CREATE_NO_WINDOW if sys.platform=="win32" else 0
        result=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=240,creationflags=flags)
        output.with_name("native_validation.log").write_text(result.stdout+result.stderr,encoding="utf-8")
        if result.returncode:
            report["native_validation_failed"] = {"returncode": result.returncode, "log": "native_validation.log"}
            write_json(output,report)
            raise RuntimeError(f"Native validation failed ({result.returncode}): {result.stderr[-2500:]}")
        report["native_basilisk"]=json.loads(native_out.read_text())
    write_json(output,report)
    print('VALIDATED candidate, NOT production ready:',output,flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--folder",type=Path,default=DEFAULT_FOLDER)
    parser.add_argument("--output",type=Path,default=ROOT/"run/mesh-collision-20260909/validation_report.json")
    parser.add_argument("--native-python",type=Path)
    parser.add_argument("--native-only",action="store_true")
    args=parser.parse_args()
    if args.native_only:
        native_probe(args.folder.resolve(),args.output.resolve())
    else:
        validate(args.folder.resolve(),args.output.resolve(),args.native_python)


if __name__=="__main__":
    main()
