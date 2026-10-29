"""Independent MuJoCo geometry preflight; never imports Basilisk or runs control.

Checks compiled transforms and real base clearance at every receiving hole.
An offline geometry pass is not a robot task success or UE rendering result.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.task_box_layout import MODULES


def name(model, kind, index):
    return mujoco.mj_id2name(model, kind, int(index))


def check(source, layout, report_path):
    placement = json.loads(layout.with_suffix(".layout.json").read_text())
    for path, field in ((source, "source_sha256"), (layout, "model_sha256")):
        if hashlib.sha256(path.read_bytes()).hexdigest() != placement[field]:
            raise ValueError(f"Layout manifest does not match {path}")
    if not placement.get("adapted_base"):
        raise ValueError("Relocate the base apertures before validating this layout")
    # The standalone Python binding needs the native symbols exported before
    # registering the same SDF plugin. Do not do this in a Basilisk process.
    libraries = list(Path(mujoco.__file__).parent.glob("libmujoco.so*"))
    keep = ctypes.CDLL(str(libraries[0]), mode=ctypes.RTLD_GLOBAL) if libraries else None
    record = json.loads((ROOT / "run/native_acceleration/sarm_thread.json").read_text())
    if mujoco.mj_version() != record["runtime_version"]:
        raise ValueError("Geometry probe and production MuJoCo versions differ")
    mujoco.mj_loadPluginLibrary(str(ROOT / record["library"]))
    a, b = (mujoco.MjModel.from_xml_path(str(path)) for path in (source, layout))
    da, db = mujoco.MjData(a), mujoco.MjData(b)
    mujoco.mj_forward(a, da)
    mujoco.mj_forward(b, db)
    assert (a.nq, a.nv, a.nu, a.njnt) == (b.nq, b.nv, b.nu, b.njnt)
    q_expected = a.qpos0.copy()
    transformed = []
    for module, ownership in MODULES.items():
        delta = np.r_[placement["offsets_body_xy_m"][module], 0.]
        for body_name in (*ownership["fixed"], *ownership["free"]):
            ia, ib = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name) for m in (a, b))
            np.testing.assert_allclose(db.xpos[ib] - da.xpos[ia], delta, atol=1e-10)
            np.testing.assert_allclose(db.xquat[ib], da.xquat[ia], atol=1e-10)
            if body_name in ownership["free"]:
                address = a.jnt_qposadr[a.body_jntadr[ia]]
                q_expected[address:address+3] += delta
            # Each compiled collision/visual geom must follow the same delta.
            for g in range(a.ngeom):
                if a.geom_bodyid[g] != ia:
                    continue
                geom_name = name(a, mujoco.mjtObj.mjOBJ_GEOM, g)
                other = mujoco.mj_name2id(b, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
                assert other >= 0
                np.testing.assert_allclose(db.geom_xpos[other]-da.geom_xpos[g], delta, atol=2e-8)
                np.testing.assert_allclose(db.geom_xmat[other], da.geom_xmat[g], atol=2e-8)
            transformed.append(body_name)
    np.testing.assert_allclose(b.qpos0, q_expected, atol=1e-14)
    # Sample actual shafts/pins at the selected model's sites, including a
    # fully seated pose. This specifically catches a board moved over solid
    # material in the old task-box base, even when both boards are welded.
    clearance = []
    for prefix in ("aviation", "guide", "cylindrical"):
        joint = mujoco.mj_name2id(b, mujoco.mjtObj.mjOBJ_JOINT, prefix+"_plug_freejoint")
        address = b.jnt_qposadr[joint]
        for suffix in ("40p1", "40p5"):
            site_name = prefix+"_insert_"+suffix
            site = mujoco.mj_name2id(b, mujoco.mjtObj.mjOBJ_SITE, site_name)
            for offset in (0., .004, .0079, .02):
                db.qpos[:] = b.qpos0
                mujoco.mj_forward(b, db)
                db.qpos[address:address+3] = db.site_xpos[site] + [0., 0., .025+offset]
                db.qpos[address+3:address+7] = [0., 1., 0., 0.]
                mujoco.mj_forward(b, db)
                hits = []
                for contact in db.contact:
                    geoms = [int(contact.geom1), int(contact.geom2)]
                    bodies = [name(b, mujoco.mjtObj.mjOBJ_BODY, b.geom_bodyid[g]) for g in geoms]
                    if prefix+"_plug" in bodies and "task_box_base" in bodies:
                        hits.append(dict(geoms=[name(b, mujoco.mjtObj.mjOBJ_GEOM, g) for g in geoms],
                                         penetration_m=max(0., float(-contact.dist))))
                maximum = max((hit["penetration_m"] for hit in hits), default=0.)
                clearance.append(dict(site=site_name, axial_offset_m=offset,
                                      max_base_penetration_m=maximum, contacts=hits,
                                      passed=maximum <= .0002))
    report = dict(passed=all(row["passed"] for row in clearance),
                  source_model_sha256=placement["source_sha256"],
                  model_sha256=placement["model_sha256"], layout_seed=placement["seed"],
                  runtime_version=mujoco.mj_version(),
                  scope="compiled translation/state invariants and shaft/pin base clearance; no robot task or UE claim",
                  initial_states_preserved=True, transformed_bodies=transformed,
                  nq=int(b.nq), nv=int(b.nv), receiving_hole_checks=clearance,
                  max_base_penetration_m=max(row["max_base_penetration_m"] for row in clearance))
    report_path.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k != "receiving_hole_checks"}, indent=2))
    return report["passed"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if check(args.source.resolve(), args.layout.resolve(), args.report.resolve()) else 1)
