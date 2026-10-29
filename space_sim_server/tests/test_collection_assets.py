"""Common runtime assets must survive checkout relocation and reject missing LFS."""
import hashlib
import json
from pathlib import Path
import shutil

import pytest

from simulation.collection_assets import prepare_base_catalog

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def assets(tmp_path):
    server = tmp_path / "checkout with spaces" / "space_sim_server"
    adapter = server.parent / "space_sim_UE_Adapter"
    mesh = server / "model/plug.obj"
    package = adapter / "Unreal/BskUnrealRenderer/Content/BSK/Common/plug.uasset"
    for path, data in ((mesh, b"v 0 0 0\n"), (package, b"test UE package"),
                       (server / "model/source.xml", b"<mujoco/>")):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    config = server / "config/collection-base-assets.json"
    config.parent.mkdir()
    config.write_text(json.dumps(dict(schema="collection-base-assets/1", source_mjcf="model/source.xml",
        assets={"model/plug.obj": dict(asset_path="/Game/BSK/Common/plug.plug", source_sha256=digest(mesh))},
        ue_assets={"BSK/Common/plug.uasset": digest(package)})))
    return server, adapter, mesh, package


def test_catalog_follows_relocated_checkout_without_saved_import_cache(assets, tmp_path):
    server, adapter, _, _ = assets
    relocated = tmp_path / "other machine & drive"
    shutil.copytree(server.parent, relocated)
    shutil.rmtree(server.parent)
    server = relocated / server.name
    adapter = relocated / adapter.name
    output = prepare_base_catalog(tmp_path / "new pool/base.json", server_root=server, adapter=adapter)
    catalog = json.loads(output.read_text())
    assert catalog["source_mjcf"] == str(server / "model/source.xml")
    assert set(catalog["assets"]) == {str(server / "model/plug.obj")}
    assert not (adapter / "Unreal/BskUnrealRenderer/Saved").exists()


@pytest.mark.parametrize("kind", ["missing", "pointer", "changed"])
@pytest.mark.parametrize("target", [2, 3])
def test_bad_mesh_or_ue_asset_fails_before_catalog_publication(assets, tmp_path, target, kind):
    server, adapter, _, _ = assets
    path = assets[target]
    if kind == "missing":
        path.unlink()
    else:
        path.write_bytes(b"version https://git-lfs.github.com/spec/v1\n" if kind == "pointer" else b"stale asset")
    output = tmp_path / "base.json"
    with pytest.raises(ValueError, match="Missing|LFS pointer|differs"):
        prepare_base_catalog(output, server_root=server, adapter=adapter)
    assert not output.exists()


def test_shipped_catalog_contains_only_reusable_assets_and_resolves(tmp_path):
    catalog = json.loads(prepare_base_catalog(tmp_path / "base.json").read_text())
    assert len(catalog["assets"]) == 131
    assert all("/Sarm20261012/" in v["asset_path"] for v in catalog["assets"].values())
    raw = (ROOT / "config/collection-base-assets.json").read_text()
    assert "/home/" not in raw and "ClearanceColor_" not in raw and "layout-000" not in raw
