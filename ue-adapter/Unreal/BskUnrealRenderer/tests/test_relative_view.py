"""Relative-motion presentation must keep explicit SI reference points."""
import pytest

from bsk_render_adapter import RelativeView, SceneSettings


def test_relative_view_is_optional_and_serializes_physical_centers():
    assert "relative_view" not in SceneSettings().to_payload()
    settings = SceneSettings(relative_view=RelativeView(
        "team/chaser", "team/target", (0.2, -0.1, 0.3), (0.01, 0.02, 0.03)))
    view = settings.to_payload()["relative_view"]
    assert view["primary_center_body_m"] == [0.2, -0.1, 0.3]
    assert view["secondary_center_body_m"] == [0.01, 0.02, 0.03]
    assert settings.to_payload()["spacecraft_scale_far_view"] == 1.0


def test_scene_settings_json_round_trip_preserves_nested_relative_view():
    import json
    original = SceneSettings(relative_view=RelativeView("a", "b", (0.3, -0.1, 0.2)))
    payload = json.loads(json.dumps(original.to_payload()))
    assert SceneSettings(**payload).to_payload() == payload
    payload["relative_view"]["secondary_object_id"] = "a"
    with pytest.raises(ValueError, match="distinct object IDs"):
        SceneSettings(**payload).to_payload()


@pytest.mark.parametrize("primary,secondary", [("", "b"), ("a", "a"), ("a", ""), (None, "b")])
def test_invalid_reference_id_is_rejected(primary, secondary):
    with pytest.raises(ValueError, match="distinct object IDs"):
        SceneSettings(relative_view=RelativeView(primary, secondary)).to_payload()


@pytest.mark.parametrize("center", [(0, 0), (0, 0, float("inf")), (0, float("nan"), 0)])
def test_nonphysical_center_is_rejected(center):
    with pytest.raises(ValueError, match="finite 3-vectors"):
        SceneSettings(relative_view=RelativeView("a", "b", center)).to_payload()
