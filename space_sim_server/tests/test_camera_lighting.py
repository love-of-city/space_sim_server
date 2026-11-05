"""Camera-mounted light wire contract."""
from bsk_render_adapter import VisualElement
from simulation.camera_lighting import overview_camera_lamp
CAMERA_ID="teleop/camera/spacecraft_overview"

def test_overview_lamp_uses_the_existing_camera_mounted_light_protocol():
    lamp = overview_camera_lamp()
    assert isinstance(lamp, VisualElement)
    payload = lamp.to_payload()
    assert payload["visual_id"] == "teleop/light/overview_lamp"
    assert payload["kind"] == "light" and payload["parent_id"] == CAMERA_ID
    assert payload["position_body_m"] == [0., 0., 0.]
    assert payload["normal_body"] == [1., 0., 0.]
    assert payload["range_m"] == 2.
    assert payload["properties"] == {
        "light_type": "spot", "diffuse_rgb": (1., 1., 1.),
        "specular_rgb": (.15, .15, .15), "intensity": 8000.,
        "cutoff_deg": 89., "cast_shadows": False,
    }
