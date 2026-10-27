"""Compile an independent layout and check mechanisms, mating frames and holes.

Uses standalone MuJoCo in its own process, never in a Basilisk process.
This is geometry/state acceptance, not a robot task success assertion.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.independent_task_box_layout import (
    FREE_BODY_MODULES, MODULES, PICKUPS, POSE_SCHEMA, SCHEMA, OPERATION_CLEARANCE_SCHEMA,
    planar_transform, validate_layout_record,
)
from simulation.plug_retention import model_retention_sockets
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix


def check(source, layout, report_path):
    record = json.loads(layout.with_suffix(".layout.json").read_text())
    if record["schema"] not in (SCHEMA, POSE_SCHEMA):
        raise ValueError("Expected the independent 13-unit layout")
    for file, field in ((source, "source_sha256"), (layout, "model_sha256")):
        if hashlib.sha256(file.read_bytes()).hexdigest() != record[field]:
            raise ValueError(f"Stale model/manifest: {file}")
    validate_layout_record(record)
    library = next(Path(mujoco.__file__).parent.glob("libmujoco.so*"))
    keep = ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    plugin = json.loads((ROOT/"run/native_acceleration/sarm_thread.json").read_text())
    if mujoco.mj_version() != plugin["runtime_version"]:
        raise ValueError("Geometry probe and production MuJoCo versions differ")
    mujoco.mj_loadPluginLibrary(str(ROOT/plugin["library"]))
    a, b = (mujoco.MjModel.from_xml_path(str(path)) for path in (source, layout))
    from bsk_render_adapter.mjcf_assets import parse_mjcf_geometry_metadata
    metadata = parse_mjcf_geometry_metadata(layout)
    assert len(metadata) == b.ngeom, "Render metadata geom count differs from compiled physics"
    for index, entry in enumerate(metadata):
        body = mujoco.mj_id2name(b, mujoco.mjtObj.mjOBJ_BODY, int(b.geom_bodyid[index]))
        assert entry.body_name == body, f"Render metadata body mismatch at geom {index}: {entry.body_name}, {body}"
        if entry.mesh_name:
            mesh = mujoco.mj_id2name(b, mujoco.mjtObj.mjOBJ_MESH, int(b.geom_dataid[index]))
            assert entry.mesh_name == mesh, f"Render metadata mesh mismatch at geom {index}: {entry.mesh_name}, {mesh}"
    da, db = mujoco.MjData(a), mujoco.MjData(b)
    mujoco.mj_forward(a, da)
    mujoco.mj_forward(b, db)

    def ident(model, kind, name):
        index = mujoco.mj_name2id(model, kind, name)
        if index < 0:
            raise ValueError(f"Missing compiled {kind}: {name}")
        return index

    def name(model, kind, index):
        return mujoco.mj_id2name(model, kind, int(index))

    base_id = ident(a, mujoco.mjtObj.mjOBJ_BODY, "task_box_base")
    deck = np.eye(4)
    deck[:3, :3], deck[:3, 3] = da.xmat[base_id].reshape(3, 3), da.xpos[base_id]

    def transform(module):
        matrix = planar_transform(record["modules"][module]["original_anchor_body_m"],
                                  record["offsets_body_xy_m"][module],
                                  record.get("installation_yaw_rad", {}).get(module, 0.))
        return deck @ matrix @ np.linalg.inv(deck)

    def transformed_position(position, module):
        matrix = transform(module)
        return matrix[:3, :3] @ position + matrix[:3, 3]

    for attribute in ("nq", "nv", "nu", "njnt", "neq"):
        assert getattr(a, attribute) == getattr(b, attribute), attribute
    for kind, count in ((mujoco.mjtObj.mjOBJ_JOINT, a.njnt), (mujoco.mjtObj.mjOBJ_ACTUATOR, a.nu),
                        (mujoco.mjtObj.mjOBJ_EQUALITY, a.neq)):
        assert [name(a, kind, i) for i in range(count)] == [name(b, kind, i) for i in range(count)]
    for attribute in ("jnt_type", "jnt_axis", "jnt_pos", "jnt_range", "jnt_limited", "dof_damping",
                      "dof_frictionloss", "dof_armature", "eq_data", "actuator_gainprm", "actuator_biasprm"):
        np.testing.assert_array_equal(getattr(a, attribute), getattr(b, attribute), err_msg=attribute)
    expected = a.qpos0.copy()
    for body, module in FREE_BODY_MODULES.items():
        i = ident(a, mujoco.mjtObj.mjOBJ_BODY, body)
        address = a.jnt_qposadr[a.body_jntadr[i]]
        expected[address:address+3] = transformed_position(expected[address:address+3], module)
        rotation = transform(module)[:3, :3] @ quaternion_wxyz_to_matrix(expected[address+3:address+7])
        quat = matrix_to_quaternion_wxyz(rotation)
        if np.dot(quat, b.qpos0[address+3:address+7]) < 0:
            quat = -quat
        expected[address+3:address+7] = quat
    np.testing.assert_allclose(b.qpos0, expected, atol=1e-14)
    for tag in ("equality", "actuator", "sensor", "extension"):
        old, new = (ET.parse(p).getroot().find(tag) for p in (source, layout))
        assert (ET.tostring(old) if old is not None else None) == (ET.tostring(new) if new is not None else None), tag
    for row in record["translated_bodies"]:
        body, module = row["body"], row["module"]
        ia = mujoco.mj_name2id(a, mujoco.mjtObj.mjOBJ_BODY, body)
        if ia < 0:  # New fixed installation containers have no original body.
            continue
        ib = ident(b, mujoco.mjtObj.mjOBJ_BODY, body)
        np.testing.assert_allclose(db.xpos[ib], transformed_position(da.xpos[ia], module), atol=2e-10)
        np.testing.assert_allclose(db.xmat[ib].reshape(3, 3),
                                   transform(module)[:3, :3] @ da.xmat[ia].reshape(3, 3), atol=2e-10)
    for site, module in record["site_modules"].items():
        ia, ib = (ident(m, mujoco.mjtObj.mjOBJ_SITE, site) for m in (a, b))
        np.testing.assert_allclose(db.site_xpos[ib], transformed_position(da.site_xpos[ia], module), atol=1e-12)
        np.testing.assert_allclose(db.site_xmat[ib].reshape(3, 3),
                                   transform(module)[:3, :3] @ da.site_xmat[ia].reshape(3, 3), atol=1e-12)
    moved_geoms = dict(record["precision_bore_ownership"], board_thread_sdf="bolt_assembly_hole")
    # Preserve the entire drawer/latch and free objects' collision geometry.
    body_modules = {**FREE_BODY_MODULES, "drawer_base":"drawer", "drawer_buckle":"drawer",
                    "asm_board3":"drawer", "locking_rod":"locking_mechanism",
                    "fastener":"locking_mechanism", "hook":"locking_mechanism", "limit_clip":"locking_mechanism"}
    for g in range(a.ngeom):
        owner = name(a, mujoco.mjtObj.mjOBJ_BODY, a.geom_bodyid[g])
        if owner in body_modules:
            moved_geoms[name(a, mujoco.mjtObj.mjOBJ_GEOM, g)] = body_modules[owner]
    for geom, module in moved_geoms.items():
        ia, ib = (ident(m, mujoco.mjtObj.mjOBJ_GEOM, geom) for m in (a, b))
        np.testing.assert_allclose(db.geom_xpos[ib], transformed_position(da.geom_xpos[ia], module), atol=3e-8)
        np.testing.assert_allclose(db.geom_xmat[ib].reshape(3, 3),
                                   transform(module)[:3, :3] @ da.geom_xmat[ia].reshape(3, 3), atol=3e-8)
        for attribute in ("geom_friction", "geom_contype", "geom_conaffinity", "geom_solref", "geom_solimp"):
            np.testing.assert_array_equal(getattr(a, attribute)[ia], getattr(b, attribute)[ib], err_msg=geom+":"+attribute)
    sockets_a, sockets_b = model_retention_sockets(source), model_retention_sockets(layout)
    for plug, old in sockets_a.items():
        for sa, sb in zip(old, sockets_b[plug], strict=True):
            module = FREE_BODY_MODULES[plug] if sa.initial else sa.name
            np.testing.assert_allclose(sb.position_body_m, transformed_position(sa.position_body_m, module), atol=1e-12)
            np.testing.assert_allclose(sb.rotation_body, transform(module)[:3, :3] @ sa.rotation_body, atol=1e-12)
            np.testing.assert_allclose(sb.plug_rotation_body, transform(module)[:3, :3] @ sa.plug_rotation_body, atol=1e-12)

    def base_contacts(prefix):
        hits = []
        for contact in db.contact:
            geoms = [int(contact.geom1), int(contact.geom2)]
            bodies = [name(b, mujoco.mjtObj.mjOBJ_BODY, b.geom_bodyid[g]) for g in geoms]
            if prefix+"_plug" in bodies and "task_box_base" in bodies:
                hits.append(dict(geoms=[name(b, mujoco.mjtObj.mjOBJ_GEOM, g) for g in geoms],
                                 penetration_m=max(0., float(-contact.dist))))
        return hits

    clearance = []
    for prefix in ("aviation", "guide", "cylindrical"):
        joint = ident(b, mujoco.mjtObj.mjOBJ_JOINT, prefix+"_plug_freejoint")
        address = b.jnt_qposadr[joint]
        for suffix in ("40p1", "40p5"):
            site = prefix+"_insert_"+suffix
            seat = next(socket for socket in sockets_b[prefix+"_plug"] if socket.name == site)
            for offset in (0., .004, .0079, .02):
                db.qpos[:] = b.qpos0
                mujoco.mj_forward(b, db)
                db.qpos[address:address+3] = seat.position_body_m + seat.rotation_body[:, 2]*offset
                db.qpos[address+3:address+7] = matrix_to_quaternion_wxyz(seat.plug_rotation_body)
                mujoco.mj_forward(b, db)
                hits = base_contacts(prefix)
                maximum = max((h["penetration_m"] for h in hits), default=0.)
                clearance.append(dict(site=site, axial_offset_m=offset, max_base_penetration_m=maximum,
                                      contacts=hits, passed=maximum <= .0002))
    pickup_clearance = []
    for site in PICKUPS:
        prefix = site.removesuffix("_pickup")
        joint = ident(b, mujoco.mjtObj.mjOBJ_JOINT, prefix+"_plug_freejoint")
        address = b.jnt_qposadr[joint]
        s = ident(b, mujoco.mjtObj.mjOBJ_SITE, site)
        for offset in (0., .004, .01, .03):
            db.qpos[:] = b.qpos0
            mujoco.mj_forward(b, db)
            direction = db.site_xmat[s].reshape(3, 3)[:, 2].copy()
            db.qpos[address:address+3] += offset*direction
            mujoco.mj_forward(b, db)
            hits = base_contacts(prefix)
            maximum = max((h["penetration_m"] for h in hits), default=0.)
            pickup_clearance.append(dict(site=site, axial_offset_m=offset, max_base_penetration_m=maximum,
                                         contacts=hits, passed=maximum <= .0002))
    all_clearance = clearance+pickup_clearance
    report = dict(passed=all(c["passed"] for c in all_clearance), layout_seed=record["seed"],
                  operation_clearance_schema=OPERATION_CLEARANCE_SCHEMA,
                  source_model_sha256=record["source_sha256"], model_sha256=record["model_sha256"],
                  scope="13-unit footprint, compiled joints/states, 11 mating frames, precision contact parameters and 36 shaft/base clearance samples",
                  initial_states_preserved=True, orientations_preserved=record["schema"] == SCHEMA,
                  installation_yaw_verified=record.get("installation_yaw_rad", {}),
                  internal_assembly_poses_preserved=True, module_count=len(MODULES),
                  render_metadata_geometry_count=len(metadata), render_metadata_matches_compiled_model=True,
                  nq=int(b.nq), nv=int(b.nv), validated_original_geoms=len(moved_geoms),
                  max_base_penetration_m=max(c["max_base_penetration_m"] for c in all_clearance),
                  receiving_hole_checks=clearance, pickup_hole_checks=pickup_clearance)
    report_path.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k not in {"receiving_hole_checks", "pickup_hole_checks"}}, indent=2))
    return report["passed"]


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--layout", type=Path, required=True)
    p.add_argument("--report", type=Path, required=True)
    args = p.parse_args()
    raise SystemExit(0 if check(args.source.resolve(), args.layout.resolve(), args.report.resolve()) else 1)
