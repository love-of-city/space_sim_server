"""Versioned, bounded real-process image transport; no rendered/physics claim."""
import base64
import hashlib
import json
from pathlib import Path
import struct
from types import SimpleNamespace
from typing import Literal
import zlib

import psutil
import pytest

from space_arm_platform.benchmark_articulated_camera import MAX_ARTICULATED_PNG_BYTES, MountedCameraImage
from space_arm_platform.benchmark_camera import CameraImage
from space_arm_platform.benchmark_contract import Contract, Count, SPEC_DIRECTORY, load_task
from space_arm_platform.benchmark_external import (
    IMAGE_PROTOCOL, MAX_IMAGE_REQUEST_BYTES, MAX_MESSAGE_BYTES, PROTOCOL,
    ExternalMethod, ExternalMethodDefinition, MethodRequest, PythonMethodSpec, decode_message,
    decode_request, request_limit,
)


class TransportContext(Contract):
    purpose: Literal['transport_fixture_only'] = 'transport_fixture_only'


class TransportObservation(Contract):
    episode_id: str
    sample_time_ns: Count
    delivery_time_ns: Count
    images: tuple[MountedCameraImage, MountedCameraImage]


WORKER = r'''
import hashlib
import sys
from typing import Literal
from space_arm_platform.benchmark_articulated_camera import MountedCameraImage
from space_arm_platform.benchmark_contract import Contract, Count
from space_arm_platform.benchmark_external import serve_method, PROTOCOL, IMAGE_PROTOCOL, MAX_MESSAGE_BYTES
class Context(Contract):
    purpose: Literal['transport_fixture_only'] = 'transport_fixture_only'
class Observation(Contract):
    episode_id: str
    sample_time_ns: Count
    delivery_time_ns: Count
    images: tuple[MountedCameraImage, MountedCameraImage]
class Method:
    def reset(self, context):
        assert context.purpose == 'transport_fixture_only'
    def act(self, observation):
        assert not any(name.startswith('simulation') or name == 'Basilisk' for name in sys.modules)
        if sys.argv[2] == 'oversized_reply':
            return dict(oversized='x'*(MAX_MESSAGE_BYTES+1))
        return dict(episode_id=observation.episode_id,sample_time_ns=observation.sample_time_ns,
            delivery_time_ns=observation.delivery_time_ns,
            images=[dict(camera_id=image.camera_id,source_frame_id=image.source_frame_id,
                width=image.width,height=image.height,png_sha256=hashlib.sha256(image.png_bytes()).hexdigest(),
                byte_count=len(image.png_bytes())) for image in observation.images])
    def close(self): pass
raise SystemExit(serve_method(Method(), Context, Observation,
    protocol=IMAGE_PROTOCOL if sys.argv[1]=='2' else PROTOCOL))
'''


def png(width, height, color):
    # A real, lossless, deliberately uncompressed RGBA PNG. These synthetic
    # pixels exercise the byte limit; they are not prerecorded vision evidence.
    def chunk(kind, body):
        return struct.pack('!I', len(body))+kind+body+struct.pack('!I', zlib.crc32(kind+body)&0xffffffff)
    raw = (b'\0'+bytes(color)*width)*height
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 8, 6, 0, 0, 0))
            +chunk(b'IDAT', zlib.compress(raw, level=0))+chunk(b'IEND', b''))


def images(size):
    result = []
    for index, color in enumerate(((40, 80, 160, 255), (160, 80, 40, 255))):
        data = png(size, size, color)
        result.append(MountedCameraImage(camera_id=f'camera-{index}', renderer_session_id='transport-fixture',
            source_frame_id=81, sample_time_ns=8_000_000_000_000, width=size, height=size,
            png_sha256=hashlib.sha256(data).hexdigest(), png_base64=base64.b64encode(data).decode('ascii')))
    return tuple(result)


def definition(folder, *, version='2', worker_version=None, mode='digest'):
    folder.mkdir()
    (folder/'worker.py').write_text(WORKER, encoding='utf-8')
    path = folder/'method.json'
    path.write_text(json.dumps(dict(schema_version='spacesim-autonomy-python-method/'+version,
        method_id='transport-fixture', task_schemas=['spacesim-autonomy-rendered-pose/1'],
        entrypoint='worker.py', files=['worker.py'], arguments=[worker_version or version, mode],
        initialization_timeout_s=30., response_timeout_s=30.)), encoding='utf-8')
    return ExternalMethodDefinition(path, SimpleNamespace(schema_version='spacesim-autonomy-rendered-pose/1'))


def assert_closed(identity):
    try:
        process = psutil.Process(identity['pid'])
        if process.create_time() == identity['created']:
            process.wait(timeout=5)
    except psutil.NoSuchProcess:
        pass


def test_two_original_size_rgba_images_cross_owned_process_unchanged_and_v1_still_rejects(tmp_path):
    frames = images(1920)
    observation = TransportObservation(episode_id='transport-only', sample_time_ns=8_000_000_000_000,
        delivery_time_ns=8_000_200_000_000, images=frames)
    raw = (MethodRequest(protocol=IMAGE_PROTOCOL, request_id=1, operation='act',
        payload=dict(observation=observation.model_dump(mode='json'))).model_dump_json()+'\n').encode()
    assert MAX_MESSAGE_BYTES < len(raw) < MAX_IMAGE_REQUEST_BYTES
    assert len(frames[0].png_bytes()) > 14_000_000
    for version in ('2', '1'):
        spec = definition(tmp_path/('v'+version), version=version)
        method = ExternalMethod(spec, spec.root)
        try:
            method.reset(TransportContext())
            identity = dict(pid=method.process.pid, created=method.owner.created)
            if version == '1':
                with pytest.raises(ValueError, match='message size'):
                    method.act(observation)
            else:
                reply = method.act(observation)
                assert reply['episode_id'] == observation.episode_id
                assert reply['sample_time_ns'] == observation.sample_time_ns
                assert reply['delivery_time_ns'] == observation.delivery_time_ns
                assert reply['images'] == [dict(camera_id=frame.camera_id, source_frame_id=81,
                    width=1920, height=1920, png_sha256=frame.png_sha256, byte_count=len(frame.png_bytes())) for frame in frames]
                assert method.telemetry()['maximum_act_request_bytes'] == MAX_IMAGE_REQUEST_BYTES
        finally:
            method.close()
        assert_closed(identity)
        assert method.telemetry()['maximum_response_bytes'] == MAX_MESSAGE_BYTES
        calls = [json.loads(row) for row in (spec.root/'method_calls.jsonl').read_text('utf-8').splitlines()]
        assert [row['operation'] for row in calls] == (['reset', 'act', 'close'] if version == '2' else ['reset', 'close'])
        assert all(row['status'] == 'ok' for row in calls)
    # The full declared dual-image base64 budget fits without resizing pixels.
    assert 2*4*((MAX_ARTICULATED_PNG_BYTES+2)//3)+1024*1024 < MAX_IMAGE_REQUEST_BYTES


@pytest.mark.parametrize('protocol', [PROTOCOL, IMAGE_PROTOCOL])
@pytest.mark.parametrize('operation', ['reset', 'act', 'close'])
def test_capacity_is_selected_by_explicit_protocol_and_operation(protocol, operation):
    expected = MAX_IMAGE_REQUEST_BYTES if protocol == IMAGE_PROTOCOL and operation == 'act' else MAX_MESSAGE_BYTES
    assert request_limit(protocol, operation) == expected


@pytest.mark.parametrize('operation', ['reset', 'close'])
def test_v2_control_messages_keep_the_original_bound(operation):
    raw = (MethodRequest(protocol=IMAGE_PROTOCOL, request_id=0, operation=operation,
        payload={'padding': 'x'*MAX_MESSAGE_BYTES}).model_dump_json()+'\n').encode()
    with pytest.raises(ValueError, match='operation size'):
        decode_request(raw, IMAGE_PROTOCOL)


def test_oversized_v2_act_and_default_reply_decoder_are_bounded():
    with pytest.raises(ValueError, match='size limit'):
        decode_request(b' '*MAX_IMAGE_REQUEST_BYTES+b'\n', IMAGE_PROTOCOL)
    with pytest.raises(ValueError, match='size limit'):
        decode_message(b' '*MAX_MESSAGE_BYTES+b'\n')


@pytest.mark.parametrize('raw', [b'{"value":NaN}\n', b'{"value":1,"value":2}\n', b'[]\n', b'{}'])
def test_v2_keeps_strict_json_and_newline_requirements(raw):
    with pytest.raises(ValueError):
        decode_request(raw, IMAGE_PROTOCOL)


@pytest.mark.parametrize('protocol', [PROTOCOL, IMAGE_PROTOCOL])
def test_protocol_versions_cannot_be_silently_mixed(protocol):
    other = IMAGE_PROTOCOL if protocol == PROTOCOL else PROTOCOL
    raw = (MethodRequest(protocol=other, request_id=0, operation='reset', payload={}).model_dump_json()+'\n').encode()
    with pytest.raises(ValueError, match='protocol differs'):
        decode_request(raw, protocol)


def test_v1_spec_serialization_has_no_new_field_and_v2_changes_frozen_identity():
    raw = dict(schema_version='spacesim-autonomy-python-method/1', method_id='wire-check',
        task_schemas=['spacesim-autonomy-rendered-pose/1'], entrypoint='worker.py', files=['worker.py'])
    first = PythonMethodSpec.model_validate(raw)
    expected = dict(raw, arguments=[], initialization_timeout_s=60., response_timeout_s=5.)
    assert first.model_dump(mode='json') == expected
    payload = json.dumps(expected, sort_keys=True, separators=(',', ':'), allow_nan=False)
    assert first.fingerprint() == hashlib.sha256(payload.encode()).hexdigest()
    second = PythonMethodSpec.model_validate(dict(raw, schema_version='spacesim-autonomy-python-method/2'))
    assert second.protocol == IMAGE_PROTOCOL and first.protocol == PROTOCOL
    assert second.fingerprint() != first.fingerprint()


def test_sdk_rejects_wrong_version_and_releases_owned_process(tmp_path):
    spec = definition(tmp_path/'wrong-sdk', worker_version='1')
    method = ExternalMethod(spec, spec.root)
    try:
        with pytest.raises(RuntimeError, match='closed its output'):
            method.reset(TransportContext())
        identity = dict(pid=method.process.pid, created=method.owner.created)
    finally:
        method.close()
    assert_closed(identity)


def test_host_rejects_response_with_another_protocol(tmp_path):
    spec = definition(tmp_path/'wrong-response')
    worker = spec.root/'worker.py'
    worker.write_text('import json,sys\nrequest=json.loads(sys.stdin.readline())\n'
        'print(json.dumps(dict(protocol="spacesim-autonomy-method-jsonl/1",request_id=0,status="ok",payload=None)),flush=True)\n',
        encoding='utf-8')
    spec = ExternalMethodDefinition(spec.path, SimpleNamespace(schema_version='spacesim-autonomy-rendered-pose/1'))
    method = ExternalMethod(spec, spec.root)
    try:
        with pytest.raises(RuntimeError, match='response protocol differs'):
            method.reset(TransportContext())
        identity = dict(pid=method.process.pid, created=method.owner.created)
    finally:
        method.close()
    assert_closed(identity)


def test_v2_sdk_bounds_oversized_action_reply_instead_of_sending_image_sized_response(tmp_path):
    spec = definition(tmp_path/'large-reply', mode='oversized_reply')
    method = ExternalMethod(spec, spec.root)
    observation = TransportObservation(episode_id='small-fixture', sample_time_ns=8_000_000_000_000,
        delivery_time_ns=8_000_200_000_000, images=images(2))
    try:
        method.reset(TransportContext())
        identity = dict(pid=method.process.pid, created=method.owner.created)
        with pytest.raises(RuntimeError, match='response exceeds 4 MiB'):
            method.act(observation)
    finally:
        method.close()
    assert_closed(identity)
    assert (spec.root/'method_calls.jsonl').stat().st_size < 1024


def test_transport_version_cannot_change_after_definition_is_frozen(tmp_path):
    spec = definition(tmp_path/'frozen')
    changed = json.loads(spec.path.read_text('utf-8'))
    changed['schema_version'] = 'spacesim-autonomy-python-method/1'
    spec.path.write_text(json.dumps(changed), encoding='utf-8')
    with pytest.raises(ValueError, match='changed'):
        spec.verify_unchanged()


def test_unknown_protocol_operation_or_unbounded_decoder_setting_is_rejected():
    for protocol, operation in (('unbounded', 'act'), (IMAGE_PROTOCOL, 'unknown')):
        with pytest.raises(ValueError, match='unknown'):
            request_limit(protocol, operation)
    with pytest.raises(ValueError, match='unknown'):
        decode_message(b'{}\n', maximum_bytes=2**40)


def test_delivered_v2_pose_example_matches_v1_on_declared_blank_stereo_observations(tmp_path):
    from space_arm_platform.benchmark_vision import (
        FiducialMarker, VisionPoseAction, VisionPoseContext, VisionPoseObservation,
    )
    task = load_task(SPEC_DIRECTORY/'d24-marked-stereo-pose-v1.json')
    context = VisionPoseContext(task=task, chaser_mass_kg=170., marker_dictionary='DICT_4X4_50',
        markers=(FiducialMarker(marker_id=0, corners_target_m=((0., 0., 0.), (.05, 0., 0.),
                                                              (.05, .05, 0.), (0., .05, 0.))),))
    data = png(640, 480, (0, 0, 0, 255))
    frames = tuple(CameraImage(camera_id=camera.camera_id, renderer_session_id='synthetic-no-markers',
        source_frame_id=0, sample_time_ns=0, width=640, height=480, png_sha256=hashlib.sha256(data).hexdigest(),
        png_base64=base64.b64encode(data).decode('ascii')) for camera in task.cameras)
    observation = VisionPoseObservation(episode_id='transport-parity-only', sequence=0,
        sample_time_ns=0, delivery_time_ns=0, images=frames)
    examples = Path(__file__).resolve().parents[1]/'examples/autonomy_methods'
    actions = []
    for filename in ('vision-pose.json', 'vision-pose-v2.json'):
        output = tmp_path/filename
        output.mkdir()
        method = ExternalMethod(ExternalMethodDefinition(examples/filename, task), output)
        try:
            method.reset(context)
            identity = dict(pid=method.process.pid, created=method.owner.created)
            actions.append(VisionPoseAction.model_validate(method.act(observation)))
        finally:
            method.close()
        assert_closed(identity)
    assert actions[0] == actions[1] and actions[0].pose is None
