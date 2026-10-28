"""Colour identities survive permutation and preserve the physical model."""
import copy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.socket_colors import (
    COLOR_KEYS, SOURCE_IDS, TARGET_SOCKET_IDS,
    apply_socket_colors, normalize_color_assignment, sample_color_assignment,
)


def independent_model():
    root = ET.Element("mujoco")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "material", name="original", rgba=".5 .5 .5 1")
    world = ET.SubElement(root, "worldbody")
    for index, kind in enumerate(("aviation", "guide", "cylindrical")):
        for name, geom_name in [(kind + "_plug", kind + "_plug_geom"), *(
                ("layout_" + kind + suffix, "layout_" + kind + suffix + "_visual")
                for suffix in ("_pickup", "_insert_40p1", "_insert_40p5"))]:
            body = ET.SubElement(world, "body", name=name, pos=f"{index * .2} 0 0")
            ET.SubElement(body, "geom", name=geom_name, type="box", size=".01 .01 .01",
                          material="original", rgba=".6 .6 .6 1", group="2", contype="0",
                          conaffinity="0", mass="0")
            ET.SubElement(body, "geom", name=name + "_collision", type="box", size=".008 .008 .008",
                          group="3", rgba="0 0 0 0", friction=".4 .001 .001")
            ET.SubElement(body, "site", name=name + "_site", pos="0 0 .01")
    return root


def seven_hole_model():
    root = independent_model()
    world = root.find("worldbody")
    for index, name in enumerate(("bolt", "layout_bolt_storage_hole", "layout_bolt_assembly_hole")):
        geom_name = "bolt_geom" if name == "bolt" else name + "_visual"
        body = ET.SubElement(world, "body", name=name, pos=f"0 {index * .2} 0")
        ET.SubElement(body, "geom", name=geom_name, type="box", size=".01 .01 .01",
                      group="2", contype="0", conaffinity="0", mass="0")
        ET.SubElement(body, "geom", name=name + "_collision", type="box", size=".008 .008 .008",
                      group="3", rgba="0 0 0 0", friction=".4 .001 .001")
        ET.SubElement(body, "site", name=name + "_site", pos="0 0 .01")
    return root


def physical_xml(root):
    root = copy.deepcopy(root)
    for material in list(root.find("asset")):
        root.find("asset").remove(material)
    for geom in root.findall(".//geom"):
        geom.attrib.pop("material", None)
        geom.attrib.pop("rgba", None)
    return ET.tostring(root)


def test_socket_identity_is_unique_per_diameter_and_matches_plug_to_scripted_target(tmp_path):
    # Reverse body and geom order: colour must depend on stable identity.
    root = independent_model()
    root.find("worldbody")[:] = reversed(list(root.find("worldbody")))
    for body in root.findall(".//body"):
        body[:] = reversed(list(body))
    physical_before = physical_xml(root)
    record = apply_socket_colors(root)
    assert physical_xml(root) == physical_before
    assert len(record["identities"]) == 12
    assert len({p["srgb_hex"] for p in record["palette"].values()}) == 6
    for kind in ("aviation", "guide", "cylindrical"):
        target = f"{kind}_insert_40p5"
        for body in (kind + "_plug", "layout_" + kind + "_pickup", "layout_" + target):
            assert record["identities"][body]["colour_identity"] == target
        assert record["identities"]["layout_" + target]["diameter_mm"] == 40.5
        alternate = record["identities"][f"layout_{kind}_insert_40p1"]
        assert alternate["colour_identity"] != target and alternate["diameter_mm"] == 40.1
    # Compile both MJCFs to independently check contact/inertial values.
    mujoco = pytest.importorskip("mujoco")
    before = independent_model()
    first = mujoco.MjModel.from_xml_string(ET.tostring(before, encoding="unicode"))
    apply_socket_colors(before)
    second = mujoco.MjModel.from_xml_string(ET.tostring(before, encoding="unicode"))
    for field in ("body_mass", "body_inertia", "body_pos", "geom_pos", "geom_quat", "geom_size",
                  "geom_type", "geom_contype", "geom_conaffinity", "geom_friction", "geom_solref",
                  "geom_solimp", "geom_margin", "geom_gap", "geom_bodyid"):
        np.testing.assert_array_equal(getattr(first, field), getattr(second, field), err_msg=field)
    assert (first.nbody, first.ngeom) == (second.nbody, second.ngeom)


def test_adapter_reads_explicit_linear_material_on_every_coloured_mesh(tmp_path, monkeypatch):
    from bsk_render_adapter import mjcf_assets
    from bsk_render_adapter.bridge import _mj_mesh_geometry
    from types import SimpleNamespace

    root = independent_model()
    # A source-only mesh metadata fixture exercises the production MJCF ->
    # GeometryVisual path without importing generated UE assets.
    ET.SubElement(root.find("asset"), "mesh", name="test_mesh", file="test.stl")
    for geom in root.findall('.//geom[@group="2"]'):
        geom.set("mesh", "test_mesh")
        geom.attrib.pop("type")
        geom.attrib.pop("size")
    record = apply_socket_colors(root)
    path = tmp_path / "layout.xml"
    ET.ElementTree(root).write(path)
    monkeypatch.setattr(mjcf_assets, "_compiled_source_mesh_poses", lambda _: [])
    items = mjcf_assets.parse_mjcf_geometry_metadata(path)
    count = 0
    for item in items:
        if not item.mesh_name:
            continue
        expected = record["palette"][record["identities"][item.body_name]["colour_identity"]]
        geometry = _mj_mesh_geometry(SimpleNamespace(rgba=[.5, .5, .5, 1]), "test/geom", item, {}, {})
        np.testing.assert_allclose(geometry.color_rgba, expected["linear_rgba"], rtol=1e-10)
        assert not geometry.use_asset_materials
        assert geometry.material_name == expected["material_name"]
        assert geometry.material_emission == 0
        count += 1
    assert count == 12


@pytest.mark.parametrize("failure", ["missing", "duplicate", "contact", "group", "material_conflict"])
def test_invalid_colour_binding_fails_before_mutating_model(failure):
    root = independent_model()
    body = root.find('.//body[@name="layout_guide_insert_40p5"]')
    geom = body.find('geom[@group="2"]')
    if failure == "missing":
        body.remove(geom)
    elif failure == "duplicate":
        body.append(copy.deepcopy(geom))
    elif failure == "contact":
        geom.set("contype", "1")
    elif failure == "group":
        geom.set("group", "3")
    else:
        ET.SubElement(root.find("asset"), "material", name="socket_identity_guide_insert_40p5", rgba="1 1 1 1")
    before = ET.tostring(root)
    with pytest.raises(ValueError, match="Socket colours|Conflicting socket"):
        apply_socket_colors(root)
    assert ET.tostring(root) == before


def test_reapplying_colour_does_not_duplicate_assets_or_change_mapping():
    root = independent_model()
    first = apply_socket_colors(root)
    xml = ET.tostring(root)
    assert apply_socket_colors(root) == first
    assert ET.tostring(root) == xml


def test_colour_matches_the_actual_scripted_profile_target():
    # The task controller owns the pairing. A future change to its body or
    # target must fail here if the visual identity still uses the old pair.
    from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES

    root = independent_model()
    record = apply_socket_colors(root)
    for profile in PLUG_PROFILES.values():
        plug = root.find(f'.//body[@name="{profile.body}"]/geom[@name="{profile.body}_geom"]')
        target_body = "layout_" + profile.target_site
        target = root.find(f'.//body[@name="{target_body}"]/geom[@name="{target_body}_visual"]')
        assert plug is not None and target is not None
        assert plug.get("material") == target.get("material"), profile.task
        assert plug.get("rgba") == target.get("rgba"), profile.task
        assert record["identities"][profile.body]["matched_insertion_site"] == profile.target_site
        assert (record["identities"][profile.body]["colour_identity"]
                == record["identities"][target_body]["colour_identity"])


def test_episode_colours_are_unique_reproducible_and_do_not_use_global_rng():
    global_before = np.random.get_state()
    observed = {hole: set() for hole in TARGET_SOCKET_IDS}
    for seed in range(1000):
        assignment = sample_color_assignment(seed)
        assert assignment == sample_color_assignment(seed)
        assert set(assignment["hole_colors"]) == set(TARGET_SOCKET_IDS)
        assert set(assignment["hole_colors"].values()) == set(COLOR_KEYS)
        assert set(assignment["source_colors"]) == set(SOURCE_IDS)
        assert len(set(assignment["source_colors"].values())) == len(SOURCE_IDS)
        assert assignment["bolt_storage_color"] == "neutral"
        for hole, colour in assignment["hole_colors"].items():
            observed[hole].add(colour)
    global_after = np.random.get_state()
    assert global_before[0] == global_after[0]
    np.testing.assert_array_equal(global_before[1], global_after[1])
    assert global_before[2:] == global_after[2:]
    assert all(values == set(COLOR_KEYS) for values in observed.values())


def test_episode_colour_applies_to_all_seven_holes_without_changing_physics():
    from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES

    root = seven_hole_model()
    before = copy.deepcopy(root)
    assignment = sample_color_assignment(4)
    record = apply_socket_colors(root, assignment)
    assert physical_xml(root) == physical_xml(before)
    assert len(record["identities"]) == 15
    assert set(record["palette"]) == set(TARGET_SOCKET_IDS)
    for hole, key in assignment["hole_colors"].items():
        assert record["palette"][hole]["color_key"] == key
        assert record["identities"]["layout_" + hole]["color_key"] == key
    for source, key in assignment["source_colors"].items():
        assert record["source_palette"][source]["color_key"] == key
    for profile in PLUG_PROFILES.values():
        assert record["identities"][profile.body]["matched_insertion_site"] == profile.target_site
    assert record["storage_palette"]["bolt_storage_hole"]["color_key"] == "neutral"
    for kind in ("aviation", "guide", "cylindrical"):
        assert record["identities"][f"layout_{kind}_pickup"]["color_key"] == "neutral"
    # Exactly seven coloured hole surfaces identify the seven target sites.
    coloured_holes = {name for name, item in record["identities"].items()
                      if item["role"] in {"insertion_socket", "storage_socket", "pickup"}
                      and item["color_key"] != "neutral"}
    assert coloured_holes == {"layout_" + name for name in TARGET_SOCKET_IDS}
    # Check the compiled physics as well as the source XML contract.
    mujoco = pytest.importorskip("mujoco")
    first = mujoco.MjModel.from_xml_string(ET.tostring(before, encoding="unicode"))
    second = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    for field in ("body_mass", "body_inertia", "body_pos", "geom_pos", "geom_quat", "geom_size",
                  "geom_type", "geom_contype", "geom_conaffinity", "geom_friction", "geom_solref",
                  "geom_solimp", "geom_margin", "geom_gap", "geom_bodyid"):
        np.testing.assert_array_equal(getattr(first, field), getattr(second, field), err_msg=field)


@pytest.mark.parametrize("group", ["hole_colors", "source_colors"])
def test_duplicate_episode_colours_are_rejected_before_any_model_changes(group):
    root = seven_hole_model()
    assignment = sample_color_assignment(4)
    first, second, *_ = assignment[group]
    assignment[group][second] = assignment[group][first]
    before = ET.tostring(root)
    with pytest.raises(ValueError, match="permutation|unique"):
        apply_socket_colors(root, assignment)
    assert ET.tostring(root) == before


def test_episode_assignment_requires_complete_bolt_binding():
    root = independent_model()
    before = ET.tostring(root)
    with pytest.raises(ValueError, match="requires the bolt/storage/assembly binding"):
        apply_socket_colors(root, sample_color_assignment(4))
    assert ET.tostring(root) == before


def test_normalized_assignment_preserves_independent_source_and_target_colours():
    assignment = sample_color_assignment(4)
    normalized = normalize_color_assignment(assignment)
    assert normalized == assignment
    assert any(normalized["source_colors"][source] != normalized["hole_colors"][source[:-5] + "_insert_40p5"]
               for source in SOURCE_IDS if source.endswith("_plug"))


def test_changed_valid_colour_permutation_cannot_keep_an_old_digest():
    root = seven_hole_model()
    assignment = sample_color_assignment(4)
    first, second, *_ = assignment["hole_colors"]
    holes = assignment["hole_colors"]
    holes[first], holes[second] = holes[second], holes[first]
    before = ET.tostring(root)
    with pytest.raises(ValueError, match="SHA256 does not match"):
        apply_socket_colors(root, assignment)
    assert ET.tostring(root) == before


def test_canonical_colour_aliases_preserve_a_saved_digest():
    from simulation.socket_colors import COLOR_PALETTE

    assignment = sample_color_assignment(4)
    canonical = copy.deepcopy(assignment)
    assignment["hole_colors"] = {hole: COLOR_PALETTE[key]["srgb_hex"]
                                 for hole, key in assignment["hole_colors"].items()}
    assignment["source_colors"] = {source: COLOR_PALETTE[key]["color_name_zh"]
                                   for source, key in assignment["source_colors"].items()}
    assert normalize_color_assignment(assignment) == canonical


def test_hole_and_source_random_streams_are_independent():
    def sample(holes, sources):
        return sample_color_assignment(hole_rng=np.random.default_rng(holes),
                                       source_rng=np.random.default_rng(sources))
    first = sample(42, 100)
    different_sources = sample(42, 101)
    different_holes = sample(43, 100)
    assert first["hole_colors"] == different_sources["hole_colors"]
    assert first["source_colors"] != different_sources["source_colors"]
    assert first["hole_colors"] != different_holes["hole_colors"]
    assert first["source_colors"] == different_holes["source_colors"]


def test_replayed_legacy_storage_colour_migrates_only_the_non_target_seat():
    root = seven_hole_model()
    assignment = sample_color_assignment(4)
    assignment["palette_schema"] = "socket-color-identity/2"
    assignment["bolt_storage_color"] = assignment["source_colors"]["bolt"]
    record = apply_socket_colors(root, assignment)
    assert record["bolt_storage_color"] == "neutral"
    assert record["hole_colors"] == assignment["hole_colors"]
    assert record["source_colors"] == assignment["source_colors"]
    assert record["assignment_sha256"] == assignment["assignment_sha256"]
