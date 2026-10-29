import time

from space_arm_platform.vision_service import StereoVisionService


def make_service(tmp_path, solver):
    return StereoVisionService(tmp_path, config={
        'camera_ids': {'left': 'L', 'right': 'R'}, 'max_skew_ns': 100000,
        'calibration': {}, 'layout': {},
    }, estimator=solver)


def packet(camera, frame=1, timestamp=100000000, session='s', **extra):
    return dict(camera_id=camera, source_frame_id=str(frame), sim_time_ns=str(timestamp),
                session_id=session, stream_kind='vision', state_kind='authoritative', **extra)


def test_pairs_source_frames_and_never_passes_truth_to_solver(tmp_path):
    calls = []
    service = make_service(tmp_path, lambda left, right: calls.append((left, right)) or {'status': 'ok'})
    service.start()
    try:
        service.on_capture(packet('L', extrinsics={'secret': 'truth'},
                                  source_wall_time_ns='123', capture_pipeline_ms=12.), {'rgb': b'left'})
        service.on_capture(packet('R', frame=2, timestamp=200000000), {'rgb': b'other'})
        assert calls == []
        service.on_capture(packet('R', floating_origin={'truth': True}), {'rgb': b'right'})
        assert service.wait_idle()
        assert calls == [(b'left', b'right')]
        assert service.status()['last_result']['source_frame_id'] == '1'
        result = service.status()['last_result']
        assert result['capture_timing']['left'] == {'source_wall_time_ns': '123', 'capture_pipeline_ms': 12.}
        assert result['capture_timing']['right'] == {}
        assert int(result['solved_wall_time_ns']) > 0
        assert 'secret' not in str(result) and 'floating_origin' not in str(result)
        assert service.status()['matched_pairs'] == 1
    finally:
        service.close()


def test_rejects_wrong_session_timestamp_and_unknown_camera(tmp_path):
    calls = []
    service = make_service(tmp_path, lambda l, r: calls.append((l, r)))
    service.start()
    try:
        service.on_capture(packet('L', session='old'), {'rgb': b'l'})
        service.on_capture(packet('R', session='new'), {'rgb': b'r'})
        service.on_capture(packet('L', session='new', timestamp=100200000), {'rgb': b'l'})
        service.on_capture(packet('outside'), {'rgb': b'x'})
        assert service.wait_idle()
        assert calls == []
        assert service.status()['rejected_pairs'] == 1
        assert service.status()['session_id'] == 'new'
    finally:
        service.close()


def test_capture_error_cannot_reuse_previous_image(tmp_path):
    calls = []
    service = make_service(tmp_path, lambda l, r: calls.append((l, r)))
    service.start()
    try:
        service.on_capture(packet('L'), {'rgb': b'l'})
        service.on_capture(packet('R', type='capture_error', error='readback'), {})
        assert service.wait_idle()
        assert calls == []
        assert 'readback' in service.status()['last_error']
    finally:
        service.close()


def test_bound_task_requires_identical_exposure_epochs_and_keeps_all_queued_pairs(tmp_path):
    calls, invalid = [], []
    service = make_service(tmp_path, lambda l, r: calls.append((l, r)) or {'status': 'ok'})
    service.bind_task('mission', 's', on_result=invalid.append)
    service.on_capture(packet('L'), {'rgb': b'l'})
    service.on_capture(packet('R', timestamp=100000001), {'rgb': b'r'})
    assert calls == [] and len(invalid) == 1
    assert 'exposure binding' in invalid[0]['reason']
    for frame in range(2, 18):
        for eye in ('L', 'R'):
            service.on_capture(packet(eye, frame=frame, timestamp=frame*100000000), {'rgb': str(frame).encode()})
    assert service._queue.qsize() == 16
    for eye in ('L', 'R'):
        service.on_capture(packet(eye, frame=18, timestamp=1800000000), {'rgb': b'18'})
    assert service._queue.qsize() == 16
    assert invalid[-1]['source_frame_id'] == '18' and 'queue exhausted' in invalid[-1]['reason']
    service.start()
    try:
        assert service.wait_idle()
        assert calls == [(str(frame).encode(), str(frame).encode()) for frame in range(2, 18)]
    finally:
        service.close()


def test_new_session_flushes_pending_and_records_stale_result(tmp_path):
    service = make_service(tmp_path, lambda l, r: {'status': 'ok'})
    service.start()
    try:
        service.on_capture(packet('L'), {'rgb': b'l'})
        service.on_capture(packet('R'), {'rgb': b'r'})
        assert service.wait_idle()
        assert service.latest_image('left') == b'l'
        service.on_capture(packet('L', session='new'), {'rgb': b'new'})
        assert service.latest_image('right') is None
        assert service.status()['last_result'] is None
    finally:
        service.close()
