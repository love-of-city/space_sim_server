"""Explicit lamp selection in a render manifest; no implicit change to legacy cameras."""
import pytest

from bsk_render_adapter import CameraVisual


def test_work_light_is_opt_in_and_does_not_change_camera_optics():
    camera = CameraVisual('right', capture_products=('rgb',), capture_consumer='vision')
    original = camera.to_payload()
    assert 'work_light_profile' not in original
    camera.work_light_profile = 'inspection-1500cd-v1'
    assert camera.to_payload() == original | {'work_light_profile': 'inspection-1500cd-v1'}


@pytest.mark.parametrize('profile', ['unknown', True, None, 1500, {}])
def test_invalid_profile_cannot_silently_use_other_illumination(profile):
    with pytest.raises(ValueError, match='work_light_profile'):
        CameraVisual('right', work_light_profile=profile).to_payload()
