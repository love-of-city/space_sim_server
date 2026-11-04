"""A checkout move must preserve asset identity and reject stale imports."""

import json

import pytest

from simulation.orbital_rendezvous.assets import bind_team_catalog


def fixture_catalog(tmp_path, entries):
    catalog = tmp_path / "catalog.json"
    catalog.write_text(json.dumps({"assets": entries}), encoding="utf-8")
    manifest = {"source_model": "model/team.xml", "meshes": {"model/SARM/meshes/link.obj": "sha-link"}}
    return catalog, manifest


def test_other_checkout_catalog_rebinds_without_changing_asset_or_original(tmp_path):
    entry = {"source_sha256": "sha-link", "asset_path": "/Game/Imported/link.link", "component_scale": [1, 2, 3]}
    catalog, manifest = fixture_catalog(tmp_path, {r"X:\old\model\SARM\meshes\link.obj": entry})
    original = catalog.read_bytes()
    result = bind_team_catalog(catalog, tmp_path / "new", manifest, tmp_path)
    assets = json.loads(result.read_text())["assets"]
    assert assets == {str((tmp_path / "new/model/SARM/meshes/link.obj").resolve()): entry}
    assert catalog.read_bytes() == original


@pytest.mark.parametrize("path,sha", [("/old/model/SARM/meshes/link.obj", "outdated"),
                                      ("/old/unrelated/link.obj", "sha-link")])
def test_stale_hash_or_same_basename_in_different_model_is_rejected(tmp_path, path, sha):
    catalog, manifest = fixture_catalog(tmp_path, {path: {"source_sha256": sha, "asset_path": "/Game/Wrong"}})
    with pytest.raises(ValueError, match="Missing, stale or ambiguous"):
        bind_team_catalog(catalog, tmp_path, manifest, tmp_path)


def test_conflicting_imports_are_rejected(tmp_path):
    catalog, manifest = fixture_catalog(tmp_path, {
        "/a/model/SARM/meshes/link.obj": {"source_sha256": "sha-link", "asset_path": "/Game/A"},
        "/b/model/SARM/meshes/link.obj": {"source_sha256": "sha-link", "asset_path": "/Game/B"}})
    with pytest.raises(ValueError, match="ambiguous"):
        bind_team_catalog(catalog, tmp_path, manifest, tmp_path)
