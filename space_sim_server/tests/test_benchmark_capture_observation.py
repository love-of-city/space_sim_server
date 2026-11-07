"""Exposure-time encoders, delayed pixels and current-action causality."""
import base64
import hashlib
import struct
import zlib

import pytest

from space_arm_platform.benchmark_articulated_camera import MountedCameraImage, MountedPinholeCamera
from space_arm_platform.benchmark_capture_arm import CaptureArmCommand, CaptureEncoders
from space_arm_platform.benchmark_capture_observation import (
    DELIVERY_DELAY_NS, PERIOD_NS, CaptureArmAction, CaptureObservation, CaptureObservationBuffer,
)

EPOCH = 8_000_000_000_000


def png():
    def chunk(name, content):
        return struct.pack('!I', len(content))+name+content+struct.pack('!I', zlib.crc32(name+content)&0xffffffff)
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('!IIBBBBB', 4, 4, 8, 6, 0, 0, 0))
        +chunk(b'IDAT', zlib.compress((b'\0'+b'\x20\x40\x80\xff'*4)*4))+chunk(b'IEND', b''))


PNG = png()


def cameras():
    return tuple(MountedPinholeCamera(camera_id=name, parent_body_name='gripper', width=4, height=4,
        horizontal_fov_rad=1., position_body_m=(side*.06, 0., 0.),
        orientation_body_from_optical_wxyz=(1., 0., 0., 0.), sample_rate_hz=10)
        for name, side in (('left', -1), ('right', 1)))


def encoder(index, episode='capture-a'):
    return CaptureEncoders(episode_id=episode, sample_time_ns=EPOCH+index*PERIOD_NS,
        arm_position_rad=(index*.001,)*6, arm_velocity_rad_s=(.01,)*6,
        axial_position_m=.02, axial_velocity_m_s=0., cam_angle_rad=1., cam_velocity_rad_s=0.,
        jaw_position_m=(-.01,)*4, jaw_velocity_m_s=(0.,)*4)


def images(index, session='render-a', **changes):
    return tuple(MountedCameraImage(**dict(dict(camera_id=camera.camera_id,
        renderer_session_id=session, source_frame_id=17+index*40, sample_time_ns=EPOCH+index*PERIOD_NS,
        width=4, height=4, png_sha256=hashlib.sha256(PNG).hexdigest(),
        png_base64=base64.b64encode(PNG).decode('ascii')), **changes)) for camera in cameras())


def buffer(episode='capture-a'):
    return CaptureObservationBuffer(cameras(), episode_id=episode, epoch_ns=EPOCH)


def tick(pending, index, episode='capture-a'):
    measured = encoder(index, episode)
    pending.capture(images(index), measured)
    return pending.observe(measured.sample_time_ns, measured)


def test_original_delay_does_not_leak_future_pixels_or_pair_them_with_current_encoders():
    pending = buffer()
    for index in range(8):
        observed = tick(pending, index)
        assert observed.current_encoders == encoder(index)
        assert observed.decision_time_ns == EPOCH+index*PERIOD_NS and observed.sequence == index
        assert observed == CaptureObservation.model_validate_json(observed.model_dump_json())
        if index < 2:
            assert observed.vision_status == 'warming_up' and observed.vision is None
            assert 'png_base64' not in observed.model_dump_json()
        else:
            frame = observed.vision
            assert observed.vision_status == 'delivered'
            assert frame.capture_encoders == encoder(index-2) != observed.current_encoders
            assert frame.images == images(index-2)
            assert frame.sample_time_ns+DELIVERY_DELAY_NS == observed.decision_time_ns == frame.delivery_time_ns
        assert pending.pending_frames == min(index+1, 2)


@pytest.mark.parametrize('fault', ['missing_eye', 'repeated_eye', 'wrong_camera', 'swapped_cameras',
    'mixed_session', 'changed_session', 'stale_frame', 'future_image', 'wrong_encoder_time',
    'wrong_encoder_episode', 'private_image_pose', 'private_encoder_truth'])
def test_bad_exposure_latches_failure_before_advancing_or_emitting(fault):
    pending = buffer()
    tick(pending, 0)
    frames, measured = images(1), encoder(1)
    if fault == 'missing_eye': frames = frames[:1]
    elif fault == 'repeated_eye': frames = (frames[0], frames[0])
    elif fault == 'wrong_camera': frames = (frames[0].model_dump(), dict(frames[1].model_dump(), camera_id='unknown'))
    elif fault == 'swapped_cameras': frames = tuple(reversed(frames))
    elif fault == 'mixed_session': frames = (frames[0].model_dump(), dict(frames[1].model_dump(), renderer_session_id='other'))
    elif fault == 'changed_session': frames = images(1, session='another-renderer')
    elif fault == 'stale_frame': frames = images(1, source_frame_id=17)
    elif fault == 'future_image': frames = images(1, sample_time_ns=EPOCH+2*PERIOD_NS)
    elif fault == 'wrong_encoder_time': measured = encoder(2)
    elif fault == 'wrong_encoder_episode': measured = encoder(1, 'another-episode')
    elif fault == 'private_image_pose': frames = (dict(frames[0].model_dump(), camera_world_position=(1., 2., 3.)), frames[1])
    else: measured = dict(measured.model_dump(), target_truth=(1., 2., 3.))
    with pytest.raises(ValueError):
        pending.capture(frames, measured)
    assert pending.failed and pending.sequence == 1 and pending.pending_frames == 1
    with pytest.raises(ValueError, match='reset the episode'):
        pending.observe(EPOCH+PERIOD_NS, encoder(1))


def test_repeated_capture_before_a_decision_cannot_replace_or_accumulate_pending_pixels():
    pending = buffer()
    pending.capture(images(0), encoder(0))
    with pytest.raises(ValueError, match='duplicated'):
        pending.capture(images(0), encoder(0))
    assert pending.failed and pending.pending_frames == 1 and pending.sequence == 0


@pytest.mark.parametrize('fault', ['not_captured', 'different_current_encoder', 'past', 'future', 'fraction', 'bool', 'none'])
def test_decision_rejects_missing_capture_changed_encoders_and_noncontiguous_time(fault):
    pending = buffer()
    measured = encoder(0)
    if fault != 'not_captured':
        pending.capture(images(0), measured)
    stamp = EPOCH
    if fault == 'different_current_encoder': measured = dict(measured.model_dump(), arm_position_rad=(.1,)*6)
    elif fault == 'past': stamp -= PERIOD_NS
    elif fault == 'future': stamp += PERIOD_NS
    elif fault == 'fraction': stamp = float(stamp)
    elif fault == 'bool': stamp = True
    elif fault == 'none': stamp = None
    with pytest.raises(ValueError):
        pending.observe(stamp, measured)
    assert pending.failed and pending.sequence == 0


def test_duplicate_decision_cannot_emit_the_same_exposure_again():
    pending = buffer()
    observed = [tick(pending, index) for index in range(3)][-1]
    assert observed.vision.sample_time_ns == EPOCH
    with pytest.raises(ValueError):
        pending.observe(observed.decision_time_ns, encoder(2))
    assert pending.failed and pending.sequence == 3 and pending.pending_frames == 2


def test_observation_itself_rejects_early_images_or_stale_delivery():
    pending = buffer()
    observations = [tick(pending, index) for index in range(3)]
    with pytest.raises(ValueError, match='before their delivery'):
        CaptureObservation.model_validate(dict(observations[0].model_dump(),
            vision_status='delivered', vision=observations[2].vision))
    modified = observations[2].model_dump()
    modified['vision']['delivery_time_ns'] += PERIOD_NS
    with pytest.raises(ValueError, match='200 ms delivery'):
        CaptureObservation.model_validate(modified)
    with pytest.raises(ValueError):
        CaptureObservation.model_validate(dict(observations[2].model_dump(), target_state=(0.,)*6))


def test_action_identity_and_warmup_hold_are_checked_before_native_command_submission():
    pending = buffer()
    observations = [tick(pending, index) for index in range(3)]
    for observation in observations:
        action = CaptureArmAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            decision_time_ns=observation.decision_time_ns, command=CaptureArmCommand())
        action.validate_for(observation)
        motion = CaptureArmAction.model_validate(dict(action.model_dump(),
            command=CaptureArmCommand(enabled=True, joint6_velocity_rad_s=.1)))
        if observation.sequence < 2:
            with pytest.raises(ValueError, match='warmup'):
                motion.validate_for(observation)
        else:
            motion.validate_for(observation)
    for changes in (dict(episode_id='old-episode'), dict(observation_sequence=1), dict(decision_time_ns=EPOCH)):
        stale = CaptureArmAction.model_validate(dict(motion.model_dump(), **changes))
        with pytest.raises(ValueError, match='current decision'):
            stale.validate_for(observations[-1])


def test_new_buffer_reset_has_no_pending_frames_or_previous_renderer_identity():
    old = buffer()
    for index in range(4): tick(old, index)
    fresh = buffer('capture-b')
    assert fresh.pending_frames == 0 and fresh.sequence == 0
    fresh.capture(images(0, session='render-b'), encoder(0, 'capture-b'))
    first = fresh.observe(EPOCH, encoder(0, 'capture-b'))
    assert first.vision is None and first.vision_status == 'warming_up'
    assert first.current_encoders.episode_id == 'capture-b'
    assert old.sequence == 4 and old.pending_frames == 2


@pytest.mark.parametrize('fault', ['one_camera', 'same_camera', 'wrong_rate', 'bool_epoch', 'bad_episode'])
def test_buffer_requires_distinct_fixed_cadence_cameras_and_valid_epoch(fault):
    configuration, epoch, episode = cameras(), EPOCH, 'capture-a'
    if fault == 'one_camera': configuration = configuration[:1]
    elif fault == 'same_camera': configuration = (configuration[0], configuration[0])
    elif fault == 'wrong_rate': configuration = tuple(dict(camera.model_dump(), sample_rate_hz=5) for camera in configuration)
    elif fault == 'bool_epoch': epoch = True
    else: episode = ''
    with pytest.raises(ValueError):
        CaptureObservationBuffer(configuration, episode_id=episode, epoch_ns=epoch)
