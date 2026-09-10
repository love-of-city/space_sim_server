"""Coarse self-contact default; preserve both historical coarse and mesh identities."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.scene_targets import (
    DEFAULT_TEMPLATE, GROUND_TARGET_TEMPLATE, LEGACY_TEMPLATE,
    MESH_TARGET_TEMPLATE, SELF_COLLISION_TEMPLATE, capture_target,
)

ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = ROOT / "model/SARM/platform"
spec = importlib.util.spec_from_file_location("select_sarm_scene", ROOT / "tools/select_sarm_scene.py")
selector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selector)


@pytest.mark.parametrize("template,relative,collision", [
    (SELF_COLLISION_TEMPLATE, "model/SARM/platform/sarm_ground_target_self_collision.xml", "coarse_boxes_with_target_self_collision"),
    (MESH_TARGET_TEMPLATE, "model/ground_validation_satellite/mesh_collision_trial/sarm_mesh_collision.xml",
     "original_triangle_rigid_flex"),
    (GROUND_TARGET_TEMPLATE, "model/SARM/platform/sarm_ground_target.xml", "coarse_boxes_no_target_self_collision"),
    (LEGACY_TEMPLATE, "model/SARM/platform/sarm_platform.xml", "legacy"),
])
def test_template_resolves_exact_existing_model_and_persists_collision_identity(tmp_path, template, relative, collision):
    target = capture_target(template)
    assert target.runtime_model == relative
    assert target.resolve_model(MODEL_ROOT) == (ROOT / relative).resolve()
    assert selector.selected_scene(MODEL_ROOT, template, check=True) == (ROOT / relative).resolve()
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(template_id=template, seed=9))
    metadata = instance["capture_target"]
    assert metadata["runtime_model"] == relative
    assert metadata["collision_model"] == collision
    assert bool(metadata["runtime_warning"]) == (template in (MESH_TARGET_TEMPLATE, SELF_COLLISION_TEMPLATE))
    saved = json.loads(Path(instance["config_path"]).read_text(encoding="utf-8"))
    assert saved["capture_target"] == metadata
    assert saved["template_id"] == template


def test_default_changes_collision_only_not_seeded_layout_or_saved_compatibility(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    catalog = manager.catalog()
    assert catalog["defaults"]["template_id"] == DEFAULT_TEMPLATE == SELF_COLLISION_TEMPLATE
    assert catalog["templates"][0]["id"] == DEFAULT_TEMPLATE
    assert {t["id"] for t in catalog["templates"]} == {SELF_COLLISION_TEMPLATE, MESH_TARGET_TEMPLATE, GROUND_TARGET_TEMPLATE, LEGACY_TEMPLATE}
    assert "内部碰撞" in catalog["templates"][0]["label"]
    for randomization_profile in ("none", "training-v1"):
        args = dict(seed=42, randomize_orbit_phase=True, randomization_profile=randomization_profile)
        coarse = manager.create_instance(SceneInstanceCreate(template_id=GROUND_TARGET_TEMPLATE, **args))
        active = manager.create_instance(SceneInstanceCreate(**args))
        assert active["template_id"] == SELF_COLLISION_TEMPLATE
        assert active["randomization"] == coarse["randomization"]
        assert active["environment"] == coarse["environment"]
        assert active["runtime"] == coarse["runtime"]
        mesh = manager.create_instance(SceneInstanceCreate(template_id=MESH_TARGET_TEMPLATE, **args))
        assert coarse["template_id"] == GROUND_TARGET_TEMPLATE
        assert mesh["template_id"] == MESH_TARGET_TEMPLATE
        assert mesh["randomization"] == coarse["randomization"]
        assert mesh["environment"] == coarse["environment"]
        assert mesh["runtime"] == coarse["runtime"]
        assert json.loads(Path(coarse["config_path"]).read_text(encoding="utf-8"))["template_id"] == GROUND_TARGET_TEMPLATE
        # The rollback must not rewrite a saved high-precision instance as boxes.
        assert json.loads(Path(mesh["config_path"]).read_text(encoding="utf-8"))["template_id"] == MESH_TARGET_TEMPLATE


@pytest.fixture
def scene_tree(tmp_path):
    model_root = tmp_path / "checkout with spaces/model/SARM/platform"
    model_root.mkdir(parents=True)
    # A valid coarse fallback exists, but must never be selected on failure.
    (model_root / "sarm_ground_target.xml").write_text("<mujoco/>")
    (model_root / "sarm_ground_target_self_collision.xml").write_text("<mujoco/>")
    selected = capture_target(MESH_TARGET_TEMPLATE).resolve_model(model_root)
    selected.parent.mkdir(parents=True)
    selected.write_text("<mujoco/>\n")
    collision = selected.parent / "cleaned_meshes/collision.obj"
    collision.parent.mkdir()
    collision.write_bytes(b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    manifest = {
        "schema": "original-triangle-collision-trial/1", "source_hashes": {}, "surfaces": [],
        "output_sha256": {selected.name: selector.fingerprint(selected),
                          "cleaned_meshes/collision.obj": selector.fingerprint(collision)},
    }
    selected.with_name("manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    return model_root, selected, collision


def test_selection_handles_spaces_and_accepts_xml_line_endings(scene_tree):
    model_root, selected, _ = scene_tree
    selected.write_bytes(b"<mujoco/>\r\n")
    assert selector.selected_scene(model_root, MESH_TARGET_TEMPLATE, check=True) == selected


@pytest.mark.parametrize("failure", ["missing", "modified", "lfs"])
def test_collision_only_asset_failure_has_no_coarse_fallback(scene_tree, failure):
    model_root, _, collision = scene_tree
    if failure == "missing":
        collision.unlink()
        error, message = FileNotFoundError, "Missing selected scene dependency"
    elif failure == "modified":
        collision.write_bytes(b"modified collision")
        error, message = ValueError, "Stale/modified scene dependency"
    else:
        collision.write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
        error, message = ValueError, "LFS pointer"
    with pytest.raises(error, match=message):
        selector.selected_scene(model_root, MESH_TARGET_TEMPLATE, check=True)


def test_missing_selected_xml_does_not_fall_back(scene_tree):
    model_root, selected, _ = scene_tree
    selected.unlink()
    with pytest.raises(FileNotFoundError, match="Selected runtime XML is missing"):
        selector.selected_scene(model_root, MESH_TARGET_TEMPLATE, check=True)


def test_rollback_default_does_not_depend_on_triangle_collision_assets(scene_tree):
    model_root, mesh, collision = scene_tree
    collision.write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
    mesh.with_name("manifest.json").write_text("invalid experimental manifest")
    assert selector.selected_scene(model_root, check=True) == model_root / "sarm_ground_target_self_collision.xml"
    with pytest.raises(ValueError):
        selector.selected_scene(model_root, MESH_TARGET_TEMPLATE, check=True)


def test_default_xml_keeps_satellite_visuals_and_original_coarse_collision():
    path = selector.selected_scene(MODEL_ROOT, check=True)
    assert path.name == "sarm_ground_target_self_collision.xml"
    root = ET.parse(path).getroot()
    target = root.find('.//body[@name="capture_target"]')
    assert len(target.findall('.//geom[@group="2"]')) == 208
    assert len(target.findall('.//geom[@group="3"]')) == 5
    assert len(root.findall("./contact/pair")) == 2
    assert not root.findall("./contact/exclude")
    assert root.findall('.//flexcomp') == []
    assert target.find('.//joint[@name="outer_panel_hinge"]') is not None
