"""A ready-file or requested lamp cannot stand in for actual sampled renderer evidence."""
from copy import deepcopy
import json

import pytest

from space_arm_platform.render_lighting_evidence import (
    append_lighting_receipt, score_work_lights, session_directory,
)
from space_arm_platform.vision_service import StereoVisionService


def fixture():
    lamp = dict(camera_id='R', profile='inspection-1500cd-v1', enabled=True,
        attached_to_camera=True, identity_camera_mount=True, intensity_units='cd',
        intensity_cd=1500., range_m=10., inner_cone_deg=35., outer_cone_deg=65., cast_shadows=False)
    config = dict(camera_ids=dict(left='L', right='R'), cameras=[dict(camera_id='L'),
        dict(camera_id='R', work_light_profile='inspection-1500cd-v1')])
    result = dict(mission_id='m', session_id='s', calibration_id='c', source_frame_id='10',
        sim_time_ns='100000000', status='ok')
    metadata = {eye: dict(session_id='s', camera_id=camera, source_frame_id='10',
        sim_time_ns='100000000', state_kind='authoritative', camera_work_lights=[deepcopy(lamp)])
        for eye, camera in config['camera_ids'].items()}
    handoff = dict(mission_id='m', calibration_id='c', navigation=dict(vision_identity=['m', 's', 'c']))
    return config, result, metadata, handoff


@pytest.mark.parametrize('damage', [None, 'off', 'wrong_unit', 'wrong_power', 'wrong_mount',
    'additional_lamp', 'missing_native_field', 'wrong_session', 'wrong_frame', 'changed_rgb',
    'missing_consumed_frame', 'duplicate_receipt', 'unbound_navigation'])
def test_lighting_audit_checks_actual_lamp_and_every_consumed_image(tmp_path, damage):
    config, result, metadata, handoff = fixture()
    camera = metadata['right']
    lamp = camera['camera_work_lights'][0]
    if damage == 'off':
        lamp['enabled'] = False
    elif damage == 'wrong_unit':
        lamp['intensity_units'] = 'unitless'
    elif damage == 'wrong_power':
        lamp['intensity_cd'] = 150000.
    elif damage == 'wrong_mount':
        lamp['identity_camera_mount'] = False
    elif damage == 'additional_lamp':
        camera['camera_work_lights'].append(deepcopy(lamp) | {'camera_id': 'L'})
    elif damage == 'missing_native_field':
        camera.pop('camera_work_lights')
    elif damage == 'wrong_session':
        camera['session_id'] = 'other'
    elif damage == 'wrong_frame':
        camera['source_frame_id'] = '9'
    elif damage == 'unbound_navigation':
        handoff['navigation']['vision_identity'] = ['m', 'other', 'c']
    service = StereoVisionService(tmp_path, config=config, estimator=lambda *_: None)
    service.evidence_dir = tmp_path/'vision-evidence'
    service._save_evidence(result, b'left', b'right', metadata)
    consumed = deepcopy(result)
    if damage == 'missing_consumed_frame':
        consumed['source_frame_id'] = '11'
    (tmp_path/'measurements.jsonl').write_text(json.dumps(dict(kind='visual', result=consumed))+'\n', 'utf-8')
    if damage == 'duplicate_receipt':
        append_lighting_receipt(service.evidence_dir, result, dict(left=b'left', right=b'right'), metadata)
    if damage == 'changed_rgb':
        (service.evidence_dir/'s/first_pair/right.png').write_bytes(b'substituted')
    audit = score_work_lights(tmp_path, handoff, config)
    assert audit['passed'] == (damage is None), audit
    if damage is None:
        assert audit['pairs'] == audit['consumed_frames'] == 1 and audit['saved_pairs'] == 3


def test_ready_flag_alone_never_proves_rendered_light(tmp_path):
    config, _, _, handoff = fixture()
    (tmp_path/'native_work_lights_ready.json').write_text('{"ready":true}', 'utf-8')
    assert not score_work_lights(tmp_path, handoff, config)['passed']


def test_task_evidence_is_bound_to_the_task_and_does_not_leak_to_solver_or_result(tmp_path):
    config, _, metadata, _ = fixture()
    config.update(calibration={}, layout={}, max_skew_ns=100000)
    calls, received = [], []
    service = StereoVisionService(tmp_path, config=config,
        estimator=lambda left, right: calls.append((left, right)) or dict(status='ok'))
    service.evidence_dir = tmp_path/'interactive'
    service.bind_task('m', 's', on_result=received.append, evidence_directory=tmp_path/'task')
    service.start()
    try:
        for eye in ('left', 'right'):
            service.on_capture(metadata[eye] | dict(stream_kind='vision', extrinsics={'secret': 'truth'}),
                {'rgb': eye.encode()})
        assert service.wait_idle()
        assert calls == [(b'left', b'right')] and len(received) == 1
        assert 'camera_work_lights' not in str(received) and 'secret' not in str(received)
        assert (tmp_path/'task/s/lighting.jsonl').is_file()
        assert not (tmp_path/'interactive').exists()
        service.unbind_task()
        assert service._task_evidence_dir is None and service.evidence_dir == tmp_path/'interactive'
    finally:
        service.close()


def test_session_cannot_redirect_evidence_outside_its_task(tmp_path):
    with pytest.raises(ValueError, match='inside its task'):
        session_directory(tmp_path/'task', '../other')
