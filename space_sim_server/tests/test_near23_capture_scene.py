"""Prevent silently ignored terminal-capture profiles on the flight runtime."""
from copy import deepcopy
from pathlib import Path

import pytest

from simulation.near23_capture import CONFIG, configure_scene

ROOT = Path(__file__).resolve().parents[1]


def test_standalone_capture_profile_preserves_environment_and_source(tmp_path):
    scene = dict(render_profile=dict(lighting={'native': True}),
                 randomization=dict(seed=7), runtime=dict(simulation_rate=1.))
    original = deepcopy(scene)
    enabled = configure_scene(scene, ROOT, tmp_path)
    assert scene == original
    assert enabled['render_profile']['lighting'] == original['render_profile']['lighting']
    assert enabled['runtime'] == original['runtime']
    assert enabled['randomization']['capture_window_trial'] is True
    visual = enabled['randomization']['capture_visual_test']
    assert (visual['input_source'], visual['mode'], visual['config_file']) == (
        'near23_stereo', 'predictive_grasp', CONFIG)


@pytest.mark.parametrize('handoff', [{}, {'phase': 'NearParked', 'accepted': True},
                                  {'phase': 'FarReady', 'accepted': True}])
def test_flight_handoff_cannot_silently_enable_unsupported_capture(handoff, tmp_path):
    scene = dict(rendezvous_handoff=handoff, render_profile={}, randomization={})
    original = deepcopy(scene)
    with pytest.raises(ValueError, match='not connected'):
        configure_scene(scene, ROOT, tmp_path)
    assert scene == original
    assert list(tmp_path.iterdir()) == []
