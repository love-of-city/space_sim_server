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
    calls = []
    sentinel = object()

    def from_file(path, *, files):
        calls.append((path, files))
        return sentinel

    namespace = {
        "mujoco": SimpleNamespace(MJScene=SimpleNamespace(fromFile=from_file)),
        "MESH_DIR": tmp_path, "MODEL_PATH": tmp_path / "scene.xml",
    }
    exec(compile(ast.Module(body=[loader], type_ignores=[]), str(SCENARIO), "exec"), namespace)
    assert namespace["_load_scene"]() is sentinel
    assert calls[0][0] == str(tmp_path / "scene.xml")
    assert {Path(path).name for path in calls[0][1]} == {"link.STL", "lower.stl", "base.OBJ", "lower.obj"}
