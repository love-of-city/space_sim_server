"""The checked-in models must work without the original external workspace."""
from pathlib import Path
import ast
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

MODEL_ROOT = Path(__file__).resolve().parents[1] / "model" / "SARM"
SCENARIO = MODEL_ROOT / "platform/scenarios/scenario_sarm_grasp.py"


def assert_asset_file(path: Path):
    assert path.resolve().is_relative_to(MODEL_ROOT.resolve()), path
    assert path.is_file(), f"Missing model asset: {path}"
    with path.open("rb") as stream:
        assert not stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"), (
            f"Model asset is still an LFS pointer; run git lfs pull: {path}"
        )


@pytest.mark.parametrize("relative", ["platform/sarm_platform.xml", "mjcf/SARM.xml", "mjcf/SARM_scene.xml"])
def test_runtime_models_have_all_local_assets(relative):
    def check_document(path):
        assert_asset_file(path)
        root = ET.parse(path).getroot()
        compiler = root.find("compiler")
        for tag, directory_attribute in (("mesh", "meshdir"), ("texture", "texturedir")):
            directory = compiler.get(directory_attribute, "") if compiler is not None else ""
            for asset in root.findall(f"./asset/{tag}[@file]"):
                assert_asset_file(path.parent / directory / asset.get("file"))
        for include in root.findall(".//include[@file]"):
            check_document(path.parent / include.get("file"))

    check_document(MODEL_ROOT / relative)


def test_native_model_paths_follow_the_checkout(tmp_path):
    # Evaluate just path declarations, not Python MuJoCo/Basilisk imports (DLL conflict on Windows).
    tree = ast.parse(SCENARIO.read_text(encoding="utf-8"))
    declarations = [
        node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id in {"ROOT_DIR", "MODEL_PATH", "MESH_DIR"}
                for target in node.targets)
    ]
    assert len(declarations) == 3
    relocated = tmp_path / "new checkout/model/SARM/platform/scenarios/scenario_sarm_grasp.py"
    namespace = {"Path": Path, "__file__": str(relocated)}
    exec(compile(ast.Module(body=declarations, type_ignores=[]), str(SCENARIO), "exec"), namespace)
    expected_root = relocated.resolve().parents[2]
    assert namespace["ROOT_DIR"] == expected_root
    assert namespace["MODEL_PATH"] == expected_root / "platform/sarm_platform.xml"
    assert namespace["MESH_DIR"] == expected_root / "meshes"


def test_native_mesh_loader_includes_uppercase_exports(tmp_path):
    tree = ast.parse(SCENARIO.read_text(encoding="utf-8"))
    loader = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_load_scene")
    for name in ("link.STL", "lower.stl", "base.OBJ", "lower.obj", "export.log"):
        (tmp_path / name).write_bytes(b"fixture")
    (tmp_path / "not_a_file.STL").mkdir()
    scene_path = tmp_path / "scene.xml"
    scene_path.write_text('<mujoco><asset>' + ''.join(
        f'<mesh name="m{i}" file="{name}"/>' for i, name in enumerate(("link.STL", "lower.stl", "base.OBJ", "lower.obj"))
    ) + '</asset></mujoco>', encoding="utf-8")
    calls = []
    sentinel = object()

    def from_file(path, *, files):
        calls.append((path, files))
        return sentinel

    namespace = {
        "mujoco": SimpleNamespace(MJScene=SimpleNamespace(fromFile=from_file)),
        "MESH_DIR": tmp_path, "MODEL_PATH": tmp_path / "scene.xml", "ET": ET, "Path": Path,
    }
    exec(compile(ast.Module(body=[loader], type_ignores=[]), str(SCENARIO), "exec"), namespace)
    assert namespace["_load_scene"]() is sentinel
    assert calls[0][0] == str(tmp_path / "scene.xml")
    assert {Path(path).name for path in calls[0][1]} == {"link.STL", "lower.stl", "base.OBJ", "lower.obj"}


def test_native_vfs_includes_rigid_flex_collision_files_not_just_visuals(tmp_path):
    tree = ast.parse(SCENARIO.read_text(encoding="utf-8"))
    loader = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_load_scene")
    meshdir = tmp_path / "meshes"
    meshdir.mkdir()
    (meshdir / "visual.OBJ").write_bytes(b"visual fixture")
    (meshdir / "collision_only.obj").write_bytes(b"collision fixture")
    xml = tmp_path / "scene.xml"
    xml.write_text('<mujoco><compiler meshdir="meshes"/><asset><mesh file="visual.OBJ"/></asset>'
                   '<worldbody><body><flexcomp type="mesh" rigid="true" file="collision_only.obj"/>'
                   '</body></worldbody></mujoco>')
    calls = []
    namespace = {"mujoco": SimpleNamespace(MJScene=SimpleNamespace(fromFile=lambda *a, **k: calls.append((a,k)))),
                 "MODEL_PATH": xml, "ET": ET, "Path": Path}
    exec(compile(ast.Module(body=[loader], type_ignores=[]), str(SCENARIO), "exec"), namespace)
    namespace["_load_scene"]()
    assert set(calls[0][1]["files"]) == {str((meshdir / name).resolve()) for name in ("visual.OBJ", "collision_only.obj")}
    (meshdir / "collision_only.obj").write_bytes(b"version https://git-lfs.github.com/spec/v1\n")
    with pytest.raises(ValueError, match="LFS pointer"):
        namespace["_load_scene"]()
