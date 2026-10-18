"""Real cache contracts; these synthetic frames do not exercise UE or physics."""
import base64
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
import struct
import threading
import zlib

import pytest

from space_arm_platform.models import SimulationObservation
from space_arm_platform.policy_observation import PolicyObservationCache, observation_identity


CAMERAS = ("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam")
TICK_NS = 20_000_000
LEASE = "policy-episode-A"


def png_pixel(rgb):
    """A valid, distinguishable RGB image, without native imaging dependencies."""
    def chunk(kind, data):
        return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", zlib.crc32(kind + data))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack("!2I5B", 1, 1, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(b"\0" + bytes(rgb))) + chunk(b"IEND", b""))


RGB = (png_pixel((255, 0, 0)), png_pixel((0, 0, 255)))


def state(*, lease=LEASE, session="render-A", frame="9007199254740993", stamp="9007199260000000",
          opening=0.25, target_opening=0.75):
    joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    targets = [-0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    return SimulationObservation(
        protocol="space-arm-control/1", type="observation", simulation_id="sarm-test",
        render_session_id=session, render_frame_id=frame, sim_time_ns=stamp,
        step_id="1", wall_time_ns="123", applied_action_sequence="0", capture_episode_id=lease,
        joint_position_rad=joints + [opening, -opening], joint_velocity_rad_s=[0.0] * 8,
        target_joint_position_rad=targets + [target_opening, -target_opening],
        arm_joint_position_rad=joints, arm_joint_velocity_rad_s=[0.0] * 6,
        target_arm_joint_position_rad=targets, gripper_type="four_bar_tendon",
        gripper_drive_position_rad=opening, gripper_drive_velocity_rad_s=0.0,
        target_gripper_drive_position_rad=target_opening, model_sha256="a" * 64,
        policy_task_state={"task_name": "drawer", "success_available": True, "success": False},
    )


def identity(observation):
    return observation.render_session_id, observation.render_frame_id, observation.sim_time_ns


def camera_packet(observation, camera, **changes):
    return dict(type="capture_frame", capture_episode_id=observation.capture_episode_id,
                camera_id=camera, session_id=observation.render_session_id,
                source_frame_id=observation.render_frame_id, sim_time_ns=observation.sim_time_ns,
                stream_kind="authoritative", state_kind="authoritative", resolution=[1, 1],
                products=[{"name": "rgb", "file_name": "rgb.png"}], **changes)


def camera(cache, observation, index, changes=None, products=None):
    packet = camera_packet(observation, CAMERAS[index])
    packet.update(changes or {})
    cache.record_capture(packet, {"rgb": RGB[index]} if products is None else products)


def complete(cache, observation):
    cache.record_observation(observation)
    camera(cache, observation, 0)
    camera(cache, observation, 1)


@pytest.fixture
def cache():
    value = PolicyObservationCache(capacity=4, timeout=0.03)
    value.begin(LEASE)
    yield value
    value.end()


@pytest.mark.parametrize("order", [("state", 0, 1), (1, 0, "state"), (0, "state", 1)])
def test_pairing_accepts_either_arrival_order_with_exact_7d_state_and_camera_bytes(cache, order):
    observation = state()
    for item in order:
        if item == "state":
            cache.record_observation(observation)
        else:
            camera(cache, observation, item)
    result = cache.get(identity(observation))
    assert (result["render_session_id"], result["render_frame_id"], result["sim_time_ns"]) == identity(observation)
    assert result["render_frame_id"] == "9007199254740993"
    assert result["state"] == [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.25]
    assert result["applied_action"] == [-0.2, 0.3, -0.4, 0.5, -0.6, 0.7, 0.75]
    assert result["model_sha256"] == "a" * 64
    assert set(result["images"]) == {"base", "left_wrist"}
    for index, view in enumerate(("base", "left_wrist")):
        image = result["images"][view]
        assert image["camera_id"] == CAMERAS[index]
        assert (image["encoding"], image["width"], image["height"]) == ("png", 1, 1)
        assert base64.b64decode(image["data"], validate=True) == RGB[index]
        assert "bytes" not in image
    # The cache owns a snapshot, not the producer's mutable Pydantic object.
    observation.joint_position_rad[0] = 100
    result["state"][1] = 100
    result["images"]["base"]["data"] = ""
    again = cache.get(identity(observation))
    assert again["state"][:2] == [0.1, -0.2]
    assert base64.b64decode(again["images"]["base"]["data"]) == RGB[0]


@pytest.mark.parametrize("field,wrong", [
    ("session_id", "previous-render-session"), ("source_frame_id", "9007199254740994"),
    ("sim_time_ns", "9007199280000000"),
])
def test_every_camera_must_match_all_three_state_identity_fields(cache, field, wrong):
    observation = state()
    cache.record_observation(observation)
    camera(cache, observation, 0)
    camera(cache, observation, 1, {field: wrong})
    with pytest.raises(RuntimeError, match="timed out pairing"):
        cache.get(identity(observation))


@pytest.mark.parametrize("missing", ["state", "base", "left_wrist"])
def test_missing_member_never_falls_back_to_a_newer_complete_observation(cache, missing):
    wanted = state()
    if missing != "state":
        cache.record_observation(wanted)
    if missing != "base":
        camera(cache, wanted, 0)
    if missing != "left_wrist":
        camera(cache, wanted, 1)
    newer = state(frame="9007199254740994", stamp="9007199280000000")
    complete(cache, newer)
    with pytest.raises(RuntimeError, match="timed out pairing"):
        cache.get(identity(wanted))


def test_old_capture_lease_and_unrequested_camera_cannot_poison_current_pair(cache):
    wanted = state()
    old = state(lease="ended-episode")
    cache.record_observation(old)
    camera(cache, old, 0)
    cache.record_capture({"type": "capture_error", "capture_episode_id": "ended-episode", "error": "old"}, {})
    camera(cache, wanted, 0, {"camera_id": "preview/other", "stream_kind": "preview"})
    complete(cache, wanted)
    assert cache.get(identity(wanted))["state"][-1] == 0.25


@pytest.mark.parametrize("damage", ["duplicate", "preview", "interpolated", "error", "off-clock"])
def test_capture_errors_are_sticky_even_if_a_complete_pair_arrives_later(cache, damage):
    observation = state()
    if damage == "duplicate":
        camera(cache, observation, 0)
        camera(cache, observation, 0)
    elif damage == "preview":
        camera(cache, observation, 0, {"stream_kind": "preview"})
    elif damage == "interpolated":
        camera(cache, observation, 0, {"state_kind": "interpolated"})
    elif damage == "error":
        cache.record_capture({"type": "capture_error", "capture_episode_id": LEASE, "error": "GPU readback failed"}, {})
    else:
        cache.record_observation(state(stamp="9007199260000001"))
    with pytest.raises(RuntimeError) as first:
        cache.get(identity(observation))
    complete(cache, observation)
    with pytest.raises(RuntimeError) as second:
        cache.get(identity(observation))
    assert str(second.value) == str(first.value)
    cache.begin("recovered-episode")
    recovered = state(lease="recovered-episode", session="render-B", frame="0", stamp="0")
    complete(cache, recovered)
    assert cache.get(identity(recovered))["render_session_id"] == "render-B"


@pytest.mark.parametrize("changes,products", [
    ({}, {}), ({}, {"rgb": b""}), ({"resolution": [0, 1]}, None),
    ({"resolution": [1, 4097]}, None), ({"resolution": [True, 1]}, None),
    ({"resolution": [1]}, None), ({"products": [{"name": "rgb", "file_name": "frame.raw"}]}, None),
])
def test_invalid_image_packet_fails_the_lease(cache, changes, products):
    observation = state()
    camera(cache, observation, 0, changes, products)
    with pytest.raises(RuntimeError, match="encoding or resolution"):
        cache.get(identity(observation))


def test_timeout_cannot_be_repaired_by_late_frames_without_a_new_lease(cache):
    observation = state()
    cache.record_observation(observation)
    camera(cache, observation, 0)
    with pytest.raises(RuntimeError, match="timed out pairing"):
        cache.get(identity(observation))
    camera(cache, observation, 1)
    with pytest.raises(RuntimeError, match="timed out pairing"):
        cache.get(identity(observation))
    cache.end()
    cache.begin(LEASE)
    complete(cache, observation)
    assert cache.get(identity(observation))["state"][-1] == 0.25


def test_bounded_cache_evicts_old_identity_without_substituting_recent_frames():
    cache = PolicyObservationCache(capacity=2, timeout=0.01)
    cache.begin(LEASE)
    observations = [state(frame=str(i), stamp=str(i * TICK_NS)) for i in range(3)]
    for observation in observations:
        complete(cache, observation)
    assert cache.get(identity(observations[-1]))["render_frame_id"] == "2"
    with pytest.raises(RuntimeError, match="timed out pairing"):
        cache.get(identity(observations[0]))
    cache.end()


@pytest.mark.parametrize("end_lease", [False, True])
def test_waiter_is_woken_by_last_camera_or_lease_end(end_lease):
    cache = PolicyObservationCache(timeout=2)
    cache.begin(LEASE)
    observation = state()
    cache.record_observation(observation)
    camera(cache, observation, 0)
    entered = threading.Event()

    def receive():
        entered.set()
        return cache.get(identity(observation))

    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(receive)
        try:
            assert entered.wait(1)
            with pytest.raises(FutureTimeout):
                pending.result(timeout=0.03)
            if end_lease:
                cache.end()
                with pytest.raises(RuntimeError, match="lease ended"):
                    pending.result(timeout=1)
            else:
                camera(cache, observation, 1)
                assert pending.result(timeout=1)["render_frame_id"] == observation.render_frame_id
        finally:
            cache.end()


def test_waiter_cannot_cross_reset_that_reuses_the_same_capture_id(monkeypatch):
    cache = PolicyObservationCache(timeout=1)
    cache.begin(LEASE)
    observation = state()
    entered_wait = threading.Event()
    original_wait = cache._condition.wait

    def signal_wait(timeout=None):
        entered_wait.set()
        return original_wait(timeout)

    # A synchronization hook, not an assertion about the storage implementation:
    # force end/begin to finish before the old waiter can reacquire the condition.
    monkeypatch.setattr(cache._condition, "wait", signal_wait)
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(cache.get, identity(observation))
        try:
            assert entered_wait.wait(1)
            with cache._condition:
                cache.end()
                cache.begin(LEASE)
                complete(cache, observation)
            with pytest.raises(RuntimeError, match="lease|reset|ended"):
                pending.result(timeout=1)
            assert cache.get(identity(observation))["state"][-1] == 0.25
        finally:
            cache.end()


@pytest.mark.parametrize("field,value", [
    ("render_session_id", ""), ("render_session_id", 3), ("render_frame_id", "-1"),
    ("render_frame_id", "1.5"), ("sim_time_ns", "NaN"), ("sim_time_ns", None),
])
def test_identity_rejects_missing_or_nondigit_components(field, value):
    payload = dict(render_session_id="renderer", render_frame_id="1", sim_time_ns="20000000")
    payload[field] = value
    with pytest.raises(ValueError):
        observation_identity(payload)


@pytest.mark.parametrize("raw,normalized", [(-0.01, 0.0), (0.0, 0.0), (1.0, 1.0), (1.01, 1.0)])
def test_7d_gripper_state_uses_calibrated_clipped_opening_not_the_second_finger(cache, raw, normalized):
    observation = state(opening=raw, target_opening=raw)
    complete(cache, observation)
    result = cache.get(identity(observation))
    assert len(result["state"]) == len(result["applied_action"]) == 7
    assert result["state"][6] == result["applied_action"][6] == normalized


def test_uncalibrated_gripper_cannot_be_returned_as_valid_7d_state(cache):
    observation = SimulationObservation(
        protocol="space-arm-control/1", type="observation", simulation_id="legacy",
        render_session_id="legacy-render", render_frame_id="0", sim_time_ns="0", step_id="0",
        wall_time_ns="1", applied_action_sequence="0", capture_episode_id=LEASE,
        joint_position_rad=[0.0] * 6, joint_velocity_rad_s=[0.0] * 6,
        target_joint_position_rad=[0.0] * 6,
    )
    complete(cache, observation)
    with pytest.raises(RuntimeError, match="gripper|6\\+1"):
        cache.get(identity(observation))
