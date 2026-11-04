"""Vision backlogs and failures must never block or contaminate dataset packets."""
import threading

from space_arm_platform.capture_receiver import CaptureReceiver


def test_blocked_vision_drops_old_samples_and_preserves_authoritative_retry():
    entered, release = threading.Event(), threading.Event()
    seen, recorded = [], []

    def blocked(metadata, products):
        seen.append(metadata['capture_sequence'])
        entered.set()
        assert release.wait(5)

    receiver = CaptureReceiver('127.0.0.1', 0, lambda m, p: recorded.append(m), on_vision_capture=blocked)
    receiver.start()
    packet = dict(stream_kind='vision', state_kind='authoritative', ack_required=True,
                  session_id='s', camera_id='wrist', capture_sequence='0')
    try:
        receiver.route_capture(packet, {'rgb': b'image'})
        assert entered.wait(2)
        for sequence in range(1, 31):
            receiver.route_capture(dict(packet, capture_sequence=str(sequence)), {'rgb': b'image'})
        assert receiver.vision_dispatch.status()['dropped'] == 22
        assert receiver.vision_dispatch.status()['pending'] == 8
        # Identical transport keys in different consumers have independent ACK deduplication.
        authoritative = dict(packet, stream_kind='authoritative')
        receiver.route_capture(authoritative, {'rgb': b'dataset'})
        receiver.route_capture(authoritative, {'rgb': b'dataset'})
        receiver.route_capture(packet, {'rgb': b'image'})
        assert receiver.wait_for_authoritative_idle(2)
        assert recorded == [authoritative]
        assert seen == ['0'] and receiver.last_authoritative_error is None
    finally:
        release.set()
        receiver.close()
    assert receiver.vision_dispatch.queue.unfinished_tasks == 0


def test_estimator_failure_is_confined_to_vision():
    finished = threading.Event()
    def fail(metadata, products):
        finished.set()
        raise ValueError('bad calibration')
    receiver = CaptureReceiver('127.0.0.1', 0, lambda m, p: None, on_vision_capture=fail)
    receiver.start()
    try:
        receiver.route_capture(dict(stream_kind='vision', state_kind='authoritative'), {})
        assert finished.wait(2)
        receiver.vision_dispatch.queue.join()
        assert receiver.vision_dispatch.last_error == 'bad calibration'
        assert receiver.last_authoritative_error is None
    finally:
        receiver.close()
