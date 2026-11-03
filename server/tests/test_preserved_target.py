from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scene_library.preserved_target import (
    DEFAULT_PIVOT, INTERFACE_GEOMS, PROXY_GEOMS, resize_housing,
)


SOURCE = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_dual_capture26.xml"


@pytest.mark.parametrize('back_center', [None, (.03842, -.10264629987)])
def test_rear_mount_clearance_preserves_front_and_outer_panel(back_center):
    source = ET.parse(SOURCE).getroot()
    regular, _ = resize_housing(source, 2.1)
    candidate, report = resize_housing(source, 2.1, clear_back_mount=True, back_center_xz=back_center)
    if back_center is not None:
        assert np.allclose(report['back_mount_center_xz_m'], back_center)
    for name in report['protected_geometry']:
        assert ET.tostring(candidate.find(f'.//geom[@name="{name}"]')) == ET.tostring(source.find(f'.//geom[@name="{name}"]'))
    outer = './/body[@name="satellite_outer_panel"]'
    assert ET.tostring(candidate.find(outer)) == ET.tostring(regular.find(outer))
    assert candidate.find('.//geom[@name="satellite_inner_panel_collision"]') is None
    assert candidate.find('.//geom[@name="target_instance_0132_part_055_color_00"]') is None
    assert candidate.find('.//geom[@name="target_instance_0018_part_009_color_00"]') is not None
    edge_part = candidate.find('.//geom[@name="target_instance_0077_part_034_color_00"]')
    assert (edge_part is None) == (back_center is not None)


def test_real_xml_preserves_port_front_parts_arms_and_camera():
    source = ET.parse(SOURCE).getroot()
    before = ET.tostring(source)
    candidate, report = resize_housing(source)
    assert ET.tostring(source) == before
    assert len(INTERFACE_GEOMS) == 899
    assert len(report["protected_geometry"]) == 909
    for name in report["protected_geometry"]:
        old = source.find(f'.//geom[@name="{name}"]')
        new = candidate.find(f'.//geom[@name="{name}"]')
        assert ET.tostring(old) == ET.tostring(new)
        if old.get("mesh"):
            mesh = old.get("mesh")
            assert ET.tostring(source.find(f'./asset/mesh[@name="{mesh}"]')) == ET.tostring(candidate.find(f'./asset/mesh[@name="{mesh}"]'))
    assert ET.tostring(source.find('.//body[@name="cubesat_bus"]')) == ET.tostring(candidate.find('.//body[@name="cubesat_bus"]'))
    assert report["source_declared_inertial_mass_kg"] == report["target_declared_inertial_mass_kg"]


def test_housing_pivot_proxy_aperture_and_front_extension_are_consistent():
    source = ET.parse(SOURCE).getroot()
    factor = 5.6
    candidate, report = resize_housing(source, factor)
    for name in ("target_instance_0024_part_011_color_00", "target_instance_0027_part_013_color_00"):
        old = source.find(f'.//geom[@name="{name}"]')
        new = candidate.find(f'.//geom[@name="{name}"]')
        expected = np.array(DEFAULT_PIVOT) + factor * (np.fromstring(old.get("pos"), sep=" ") - DEFAULT_PIVOT)
        assert np.allclose(np.fromstring(new.get("pos"), sep=" "), expected)
    assert len(report["removed_geometry"]) == 5
    for record in report["removed_geometry"]:
        assert candidate.find(f'.//geom[@name="{record["name"]}"]') is None
    for name in PROXY_GEOMS:
        geom = candidate.find(f'.//geom[@name="{name}"]')
        assert (geom.get("contype"), geom.get("conaffinity")) == ("2", "1")
        pos, size = [np.fromstring(geom.get(k), sep=" ") for k in ("pos", "size")]
        assert pos[0] + size[0] <= .28865 + 1e-12
        assert pos[1] + size[1] <= .05475 + 1e-12
    assert len(report["added_geometry"]) == 4
    for strip in report["added_geometry"]:
        low, high = np.array(strip["bounds_m"])
        assert high[1] <= .04975 + 1e-12
        assert np.all(high > low)
        for x, z in ((.1, .15), (.25, .15), (.1, -.13), (.25, -.13)):
            # Original tag face plane remains in front of every added strip.
            assert not (low[0] <= x <= high[0] and low[2] <= z <= high[2] and high[1] >= .0578)
    source_i = source.find('.//body[@name="capture_target"]/inertial')
    new_i = candidate.find('.//body[@name="capture_target"]/inertial')
    assert np.allclose(np.fromstring(new_i.get("diaginertia"), sep=" "), factor**2 * np.fromstring(source_i.get("diaginertia"), sep=" "))


def test_rejects_non_authored_pivot_or_missing_interface():
    source = ET.parse(SOURCE).getroot()
    with pytest.raises(ValueError, match="pivot"):
        resize_housing(source, pivot=(0, 0, 0))
    for center in ((0, 0), (.2, .4), (float('nan'), 0), (0,)):
        with pytest.raises(ValueError, match="Back"):
            resize_housing(source, 2.1, clear_back_mount=True, back_center_xz=center)
    missing = deepcopy(source)
    target = missing.find('.//body[@name="capture_target"]')
    target.remove(target.find('geom[@name="capture_x_piece_0"]'))
    with pytest.raises(ValueError, match="interface geometry"):
        resize_housing(missing)
