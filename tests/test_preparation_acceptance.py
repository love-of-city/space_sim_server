from types import SimpleNamespace

import pytest

from tools.verify_arm_preparation import Tester as AcceptanceRunner


def verifier(state):
    tester = AcceptanceRunner(SimpleNamespace(), None, None, {})
    tester.state = lambda: state
    calls = []
    tester.api = lambda *args: calls.append(args) or {}
    return tester, calls


@pytest.mark.parametrize('replace', [False, True])
def test_live_verifier_never_stops_an_existing_recording(replace):
    tester, calls = verifier({'active_episode': 'existing', 'scene_runtime': {
        'active': True, 'instance': {'instance_id': 'owned'},
    }})
    tester.scene = 'owned'
    with pytest.raises(RuntimeError, match='active recording'):
        tester.stop(replace=replace)
    assert calls == []


def test_live_verifier_requires_replacement_permission():
    tester, calls = verifier({'active_episode': None, 'scene_runtime': {
        'active': True, 'instance': {'instance_id': 'other'},
    }})
    with pytest.raises(RuntimeError, match='unowned scene'):
        tester.stop()
    assert calls == []


@pytest.mark.parametrize('pending_recording', [False, True])
def test_failed_recording_request_cannot_abort_an_unknown_episode(pending_recording):
    tester, calls = verifier({'active_episode': 'other-episode', 'scene_runtime': {
        'active': True, 'instance': {'instance_id': 'owned'},
    }})
    tester.scene = 'owned'
    tester.pending_recording = pending_recording
    with pytest.raises(RuntimeError, match='unrelated recording'):
        tester.cleanup()
    assert calls == []


def test_failure_cleanup_does_not_touch_replacement_scene():
    tester, calls = verifier({'active_episode': None, 'scene_runtime': {
        'active': True, 'instance': {'instance_id': 'replacement'},
    }})
    tester.scene = 'owned'
    with pytest.raises(RuntimeError, match='replacement scene'):
        tester.cleanup()
    assert calls == []
