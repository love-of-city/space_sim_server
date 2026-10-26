"""Check the real assembly, not a synthetic substitute for its ownership graph."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.task_box_layout import MODULES, build_layout, sampled_offsets, validated_capture_layout
from simulation.plug_retention import model_retention_sockets

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def test_seed_reproduces_layout_without_changing_global_rng():
    import random
    before = random.getstate()
    first = sampled_offsets(20261021)
    assert first == sampled_offsets(20261021)
    assert first != sampled_offsets(20261022)
    assert first["pickup_board"] != first["assembly_board"]
    assert random.getstate() == before


def test_only_allowed_body_translations_and_asset_paths_change(tmp_path):
    original_bytes = MODEL.read_bytes()
    offsets = {"pickup_board": [-.025, .015], "assembly_board": [-.008, .025]}
    manifest = build_layout(MODEL, tmp_path / "layout.xml", offsets)
    old, new = ET.fromstring(original_bytes), ET.parse(tmp_path / "layout.xml").getroot()
    reconstructed = copy.deepcopy(new)
    for row in manifest["translated_bodies"]:
        before = old.find(f'.//body[@name="{row["body"]}"]')
        after = new.find(f'.//body[@name="{row["body"]}"]')
        delta = np.fromstring(after.get("pos"), sep=" ") - np.fromstring(before.get("pos", "0 0 0"), sep=" ")
        np.testing.assert_allclose(delta, [*offsets[row["module"]], 0], atol=1e-15)
        restored = reconstructed.find(f'.//body[@name="{row["body"]}"]')
        if before.get("pos") is None:
            restored.attrib.pop("pos")
        else:
            restored.set("pos", before.get("pos"))
    reconstructed.set("model", old.get("model"))
    reconstructed.find("compiler").attrib.clear()
    reconstructed.find("compiler").attrib.update(old.find("compiler").attrib)
    for a, b in zip(old.findall("./asset/*"), reconstructed.findall("./asset/*"), strict=True):
        if a.get("file") is not None:
            b.set("file", a.get("file"))
    assert ET.tostring(old) == ET.tostring(reconstructed)
    assert MODEL.read_bytes() == original_bytes
    assert (tmp_path / "attitude_control.json").read_bytes() == MODEL.with_name("attitude_control.json").read_bytes()
    for mesh in new.findall("./asset/mesh[@file]"):
        assert Path(mesh.get("file")).is_file()


def test_starting_seats_and_receiving_friction_frames_follow_their_own_board(tmp_path):
    offsets = {"pickup_board": [-.025, .015], "assembly_board": [-.008, .025]}
    output = tmp_path / "layout.xml"
    build_layout(MODEL, output, offsets)
    old, new = model_retention_sockets(MODEL), model_retention_sockets(output)
    assert set(new) == set(old)
    for name in old:
        for before, after in zip(old[name], new[name], strict=True):
            module = "pickup_board" if before.initial else "assembly_board"
            np.testing.assert_allclose(after.position_body_m - before.position_body_m,
                                       [*offsets[module], 0], atol=1e-15)
            np.testing.assert_array_equal(before.rotation_body, after.rotation_body)
            np.testing.assert_array_equal(before.plug_rotation_body, after.plug_rotation_body)
            assert (before.name, before.initial, before.keyed) == (after.name, after.initial, after.keyed)


@pytest.mark.parametrize("offsets", [
    {}, {"pickup_board": [0, 0]},
    {"pickup_board": [0, 0], "assembly_board": [0, 0], "drawer": [0, 0]},
    {"pickup_board": [.041, 0], "assembly_board": [0, 0]},
    {"pickup_board": [0, float("nan")], "assembly_board": [0, 0]},
    {"pickup_board": [0, 0, .01], "assembly_board": [0, 0]},
])
def test_invalid_offsets_fail_before_writing(tmp_path, offsets):
    output = tmp_path / "layout.xml"
    with pytest.raises(ValueError):
        build_layout(MODEL, output, offsets)
    assert not output.exists()


def test_refuses_to_overwrite_source():
    with pytest.raises(ValueError, match="separate"):
        build_layout(MODEL, MODEL, {key: [0, 0] for key in MODULES})


def test_capture_binding_rejects_stale_model_catalog_or_visual(tmp_path):
    """Metadata binding is independent of the native geometry acceptance."""
    model, visual, catalog = (tmp_path / x for x in ("layout.xml", "visual.stl", "catalog.json"))
    manifest = build_layout(MODEL, model, {key: [0, 0] for key in MODULES})
    catalog.write_text(json.dumps({"source_mjcf": str(model), "assets": {}}))
    with pytest.raises(ValueError, match="adapted"):
        validated_capture_layout(model, catalog)
    visual.write_bytes(b"asset-content-for-metadata-binding")
    manifest["adapted_base"] = {"visual": str(visual)}
    model.with_suffix(".layout.json").write_text(json.dumps(manifest))
    entry = {"source_sha256": hashlib.sha256(visual.read_bytes()).hexdigest()}
    catalog_data = {"source_mjcf": str(model), "assets": {str(visual): entry}}
    catalog.write_text(json.dumps(catalog_data))
    assert validated_capture_layout(model, catalog) == manifest
    catalog.write_text(json.dumps({**catalog_data, "source_mjcf": str(MODEL)}))
    with pytest.raises(ValueError, match="different model"):
        validated_capture_layout(model, catalog)
    catalog.write_text(json.dumps(catalog_data))
    visual.write_bytes(b"changed-asset-content")
    with pytest.raises(ValueError, match="adapted base visual"):
        validated_capture_layout(model, catalog)
    model.write_bytes(model.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="selected model"):
        validated_capture_layout(model, catalog)
