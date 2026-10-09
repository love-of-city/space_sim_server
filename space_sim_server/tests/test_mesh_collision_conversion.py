"""Source-triangle export and runtime-matched rigid-flex contact tests.

Run separately from Basilisk, using tools/requirements-mesh-collision.txt.
No production scene is edited, even by the integration tests.
"""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("triangle_collision_builder", ROOT / "tools/build_satellite_mesh_collision.py")
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


@pytest.fixture
def small_scene(tmp_path, monkeypatch):
    mesh = tmp_path / "triangle.obj"
    mesh.write_text("v 0 0 0\nv .01 0 0\nv 0 .01 0\nf 1 2 3\n")
    source = tmp_path / "original.xml"
    source.write_text("<mujoco/>")
    base = tmp_path / "base.xml"
    base.write_text(f"""<mujoco><compiler angle="radian" meshdir="."/><option gravity="0 0 0"/>
    <asset><mesh name="target_part" file="{mesh.as_posix()}"/></asset>
    <worldbody><body name="robot"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    <body name="capture_target"><freejoint name="target_free"/><inertial pos="0 0 0" mass="8" diaginertia="1 1 1"/>
    <geom name="bus_visual" type="mesh" mesh="target_part" group="2" contype="0" conaffinity="0"/>
    <geom name="capture_target_collision" type="box" size=".1 .1 .1"/>
    <body name="satellite_inner_panel"><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom name="inner_visual" type="mesh" mesh="target_part" group="2" contype="0" conaffinity="0"/>
    <geom name="satellite_inner_panel_collision" type="box" size=".1 .1 .1"/>
    <body name="satellite_outer_panel"><joint name="outer_panel_hinge" type="hinge" limited="false"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom name="outer_visual" type="mesh" mesh="target_part" group="2" contype="0" conaffinity="0"/>
    <geom name="satellite_outer_panel_collision" type="box" size=".1 .1 .1"/>
    </body></body></body></worldbody><actuator/><keyframe/><custom/>
    <contact><exclude body1="capture_target" body2="satellite_outer_panel"/></contact></mujoco>""")
    monkeypatch.setattr(builder, "BASE", base)
    monkeypatch.setattr(builder, "SOURCE", source)
    return source, base, mesh


def test_export_keeps_original_sources_and_surfaces_and_removes_only_boxes(small_scene, tmp_path):
    hashes = [builder.digest(p) for p in small_scene]
    outputs, manifest = builder.build_candidates(tmp_path / "trial")
    combined = ET.fromstring(outputs["sarm_mesh_collision.xml"])
    target = combined.find('.//body[@name="capture_target"]')
    assert len(target.findall(".//geom")) == 3
    assert all(g.get("contype") == g.get("conaffinity") == "0" for g in target.findall(".//geom"))
    assert len(target.findall(".//flexcomp")) == 3
    assert not combined.findall("./contact/exclude")
    assert manifest["triangle_instances"] == manifest["retained_triangle_instances"] == 3
    assert manifest["default_scene_changed"] is False
    assert [builder.digest(p) for p in small_scene] == hashes
    for flex in target.findall(".//flexcomp"):
        assert flex.get("rigid") == "true" and flex.get("type") == "mesh" and flex.get("dim") == "2"
        assert float(flex.get("radius")) == 1e-5
    fixed, moving = [target.find(f'.//flexcomp[@name="contact_{name}"]/contact') for name in ("bus_visual", "outer_visual")]
    assert int(fixed.get("contype")) & int(moving.get("conaffinity"))
    assert int(moving.get("contype")) & int(fixed.get("conaffinity"))
    assert not int(fixed.get("contype")) & int(fixed.get("conaffinity"))
    standalone = ET.fromstring(outputs["satellite_mesh_collision.xml"])
    assert standalone.find("keyframe") is None and standalone.find("actuator") is None
    assert len(standalone.findall("./worldbody/body")) == 1


def test_degeneracy_cleanup_is_audited_and_does_not_move_source_vertices(tmp_path):
    p = tmp_path / "degenerate.obj"
    p.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 0\nf 1 2 3\nf 1 2 4\n")
    info, data = builder.audit_obj(p)
    assert info["removed_float32_zero_area_face_indices"] == [1]
    assert info["retained_triangles"] == 1 and info["removed_original_area_m2"] == 0
    cleaned = tmp_path / "cleaned.obj"
    cleaned.write_bytes(data)
    v0, f0 = builder.load_obj(p)
    v1, f1 = builder.load_obj(cleaned)
    np.testing.assert_array_equal(v0, v1)
    np.testing.assert_array_equal(f0[:1], f1)


def test_non_triangle_and_coarse_precision_loss_are_not_silently_accepted(tmp_path):
    p = tmp_path / "bad.obj"
    p.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n")
    with pytest.raises(ValueError, match="Non-triangle"):
        builder.audit_obj(p)
    p.write_text("v 10000.00001 0 0\nv 10001 0 0\nv 10000 1 0\nf 1 2 3\n")
    with pytest.raises(ValueError, match="precision loss"):
        builder.audit_obj(p)


@pytest.mark.parametrize("radius", [0, -1e-5, 1, float("nan"), float("inf")])
def test_rejects_invalid_or_concealed_large_contact_skin(tmp_path, radius):
    with pytest.raises(ValueError, match="radius_m"):
        builder.build_candidates(tmp_path / "trial", radius)


def test_managed_output_and_reproducibility_checks(small_scene, tmp_path):
    output = tmp_path / "trial"
    builder.write_candidates(output)
    builder.write_candidates(output, check=True)
    xml = output / "satellite_mesh_collision.xml"
    xml.write_bytes(xml.read_bytes() + b" ")
    with pytest.raises(ValueError, match="Stale"):
        builder.write_candidates(output, check=True)
    user = tmp_path / "user_files"
    user.mkdir()
    file = user / "keep.txt"
    file.write_text("do not overwrite")
    with pytest.raises(ValueError, match="Refusing"):
        builder.write_candidates(user)
    assert file.read_text() == "do not overwrite"


def test_rigid_triangle_contact_preserves_a_hole_instead_of_its_convex_hull():
    m = pytest.importorskip("mujoco")
    # Square frame with a genuine central hole. No contact in the opening;
    # finite contact on its material. A convex hull would fill the hole.
    v = [(-.02,-.02,0),(.02,-.02,0),(.02,.02,0),(-.02,.02,0),
         (-.01,-.01,0),(.01,-.01,0),(.01,.01,0),(-.01,.01,0)]
    f=[]
    for i in range(4):
        j=(i+1)%4
        f.extend([(i,j,j+4),(i,j+4,i+4)])
    points = " ".join(str(c) for row in v for c in row)
    elements = " ".join(str(c) for row in f for c in row)
    xml=f"""<mujoco><option gravity="0 0 0"/><worldbody>
    <body name="frame"><flexcomp name="ring" type="direct" rigid="true" dim="2" radius=".00001"
    point="{points}" element="{elements}"><contact contype="2" conaffinity="1" selfcollide="none"/></flexcomp></body>
    <body name="probe"><freejoint/><geom type="sphere" size=".003" mass="1" contype="1" conaffinity="2"/></body>
    </worldbody></mujoco>"""
    model=m.MjModel.from_xml_string(xml); data=m.MjData(model)
    assert model.nq == 7 and model.nv == 6 and model.nbody == 3
    assert bool(model.flex_rigid[0])
    m.mj_forward(model,data)
    assert data.ncon == 0
    data.qpos[0]=.015
    m.mj_forward(model,data)
    assert data.ncon > 0
    force=np.zeros(6);m.mj_contactForce(model,data,0,force)
    assert np.isfinite(force).all() and force[0] > 0


def test_native_attitude_settings_are_copied_without_retuning(small_scene, tmp_path):
    _, base, _ = small_scene
    settings = base.with_name("attitude_control.json")
    settings.write_text('{"enabled": true, "kp": 7.5}\n')
    outputs, manifest = builder.build_candidates(tmp_path / "trial")
    assert outputs["attitude_control.json"] == settings.read_bytes().replace(b"\r\n", b"\n")
    assert builder.relative(settings, builder.ROOT) in manifest["source_hashes"]


def test_stale_base_source_relationship_is_rejected(small_scene, tmp_path):
    source, base, _ = small_scene
    base.with_suffix(".manifest.json").write_text(json.dumps({
        "sha256": builder.digest(base), "sources": {builder.relative(source, builder.ROOT): "outdated"}
    }))
    with pytest.raises(ValueError, match="Base scene is stale"):
        builder.build_candidates(tmp_path / "trial")
