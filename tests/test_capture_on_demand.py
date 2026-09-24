"""Capture starts at a tagged frame, not at scene launch or HTTP arrival."""
import asyncio
import json
import time

import pytest

from space_arm_platform.capture_lifecycle import CaptureLifecycle
from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub
from simulation.control_client import SimulationControlClient


def observation(i, episode="", request=""):
    return SimulationObservation(protocol="space-arm-control/1", type="observation", simulation_id="test",
        render_session_id="session", step_id=str(i+1), render_frame_id=str(i),
        sim_time_ns=str((i * 1_000_000_000 + 15) // 30), wall_time_ns=str(time.time_ns()),
        capture_episode_id=episode, capture_request_id=request, applied_action_sequence="0",
        joint_position_rad=[0.] * 8, joint_velocity_rad_s=[0.] * 8, target_joint_position_rad=[0.] * 8)


def capture(i, episode):
    return dict(protocol="bsk-capture/1", session_id="session", camera_id="wrist", capture_sequence=str(i),
        source_frame_id=str(i), sim_time_ns=observation(i).sim_time_ns, capture_episode_id=episode,
        stream_kind="authoritative", state_kind="authoritative", products=[dict(name="rgb", file_name="rgb.jpg")])


class Writer:
    def __init__(self, root, request):
        self.fps = request.fps
        self.cameras = request.camera_ids
        self.frames = 0
    def append(self, obs, caps): self.frames += 1
    def finish(self, *args): pass
    def close_failed(self): pass


def test_preview_and_previous_episode_never_enter_recording(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=.1)
    info = r.start(EpisodeStart(camera_ids=["wrist"]), capture_on_demand=True)
    episode = info["episode_id"]
    try:
        r.record_observation(observation(10), None)
        r.record_observation(observation(11, "old-episode"), None)
        r.record_authoritative_capture(capture(11, "old-episode"), {"rgb": b"jpeg"})
        assert r.sync_status()["pending_dataset_samples"] == 0
        assert r.sync_status()["pending_capture_count"] == 0
        # A real strict image can overtake its observation at the start boundary.
        r.record_authoritative_capture(capture(12, episode), {"rgb": b"jpeg"})
        r.record_observation(observation(12, episode), None)
        result = r.stop(EpisodeStop())
        assert result["dataset_status"] == "complete", result
        assert result["dataset_frame_count"] == 1
        rows = (tmp_path/episode/"steps.jsonl").read_text(encoding="utf-8").splitlines()
        assert [json.loads(row)["render_frame_id"] for row in rows] == ["12"]
    finally: r.close()


def test_stop_freezes_states_but_drains_already_accepted_rgb(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=.1)
    episode = r.start(EpisodeStart(camera_ids=["wrist"]), capture_on_demand=True)["episode_id"]
    try:
        r.record_observation(observation(20, episode), None)
        r.freeze_observations()
        r.record_observation(observation(21, episode), None)
        r.record_authoritative_capture(capture(21, episode), {"rgb": b"after cutoff"})
        r.record_authoritative_capture(capture(20, episode), {"rgb": b"accepted before cutoff"})
        result = r.stop(EpisodeStop())
        assert result["dataset_status"] == "complete", result
        assert result["dataset_frame_count"] == result["step_count"] == result["capture_count"] == 1
        assert result["unmatched_capture_count"] == 0
    finally: r.close()


def test_second_episode_rejects_delayed_packets_from_first(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=.1)
    try:
        first = r.start(EpisodeStart(camera_ids=["wrist"]), capture_on_demand=True)["episode_id"]
        r.stop(EpisodeStop())
        second = r.start(EpisodeStart(camera_ids=["wrist"]), capture_on_demand=True)["episode_id"]
        r.record_observation(observation(100, first), None)
        r.record_authoritative_capture(capture(100, first), {"rgb": b"old"})
        r.record_observation(observation(101, second), None)
        r.record_authoritative_capture(capture(101, second), {"rgb": b"new"})
        result = r.stop(EpisodeStop())
        assert result["dataset_status"] == "complete", result
        assert result["dataset_frame_count"] == 1
    finally: r.close()


class Hub:
    connected = True
    resetting = False
    capture_on_demand_supported = True
    def __init__(self, recorder, fail_start=False):
        self.recorder, self.calls, self.fail_start = recorder, [], fail_start
    async def set_capture_episode(self, episode, **kwargs):
        self.calls.append(episode)
        if episode:
            assert self.recorder.episode_id == episode
            # In-flight previews must be excluded even while START awaits confirmation.
            self.recorder.record_observation(observation(0), None)
            if self.fail_start: raise RuntimeError("no acknowledgement")
        else:
            assert self.recorder.sync_status()["capture_boundary_frozen"]


def test_lifecycle_starts_only_after_recorder_ready_and_stops_before_finalize(tmp_path):
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer)
        hub = Hub(r)
        lifecycle = CaptureLifecycle(hub, r)
        assert not hub.calls
        info = await lifecycle.start(EpisodeStart(camera_ids=["wrist"]))
        assert hub.calls == [info["episode_id"]]
        assert r.sync_status()["pending_dataset_samples"] == 0
        await lifecycle.stop(EpisodeStop())
        assert hub.calls == [info["episode_id"], ""]
        assert r.episode_id is None
        r.close()
    asyncio.run(run())


def test_start_failure_sends_off_and_closes_failed_episode(tmp_path):
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer)
        hub = Hub(r, fail_start=True)
        with pytest.raises(RuntimeError, match="acknowledgement"):
            await CaptureLifecycle(hub, r).start(EpisodeStart(camera_ids=["wrist"]))
        assert len(hub.calls) == 2 and hub.calls[-1] == ""
        assert r.episode_id is None
        meta = json.loads(next(tmp_path.glob("*/metadata.json")).read_text(encoding="utf-8"))
        assert meta["status"] == "failed"
        r.close()
    asyncio.run(run())


def test_unsupported_peer_rejected_before_creating_episode(tmp_path):
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer)
        hub = Hub(r); hub.capture_on_demand_supported = False
        with pytest.raises(RuntimeError, match="重启场景"):
            await CaptureLifecycle(hub, r).start(EpisodeStart(camera_ids=["wrist"]))
        assert not list(tmp_path.iterdir()) and r.episode_id is None
        r.close()
    asyncio.run(run())


def test_capture_ack_is_an_actual_render_snapshot_not_socket_delivery():
    async def run():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        client = SimulationControlClient("127.0.0.1", hub.bound_port, "test")
        client.start()
        try:
            for _ in range(200):
                if hub.connected and client.capture_state()["capture_request_id"]: break
                await asyncio.sleep(.01)
            assert client.capture_state()["capture_episode_id"] == ""
            start = asyncio.create_task(hub.set_capture_episode("episode-A", timeout=2))
            for _ in range(200):
                if client.capture_state()["capture_episode_id"] == "episode-A": break
                await asyncio.sleep(.01)
            assert not start.done()  # Socket receipt is not the frame boundary.
            state = client.capture_state()
            client.send_observation(observation(10, "", "old").model_dump())
            await asyncio.sleep(.03)
            assert not start.done()
            client.send_observation(observation(11, state["capture_episode_id"], state["capture_request_id"]).model_dump())
            await start
            stop = asyncio.create_task(hub.set_capture_episode("", timeout=2))
            for _ in range(200):
                if client.capture_state()["capture_episode_id"] == "": break
                await asyncio.sleep(.01)
            state = client.capture_state()
            client.send_observation(observation(12, "", state["capture_request_id"]).model_dump())
            await stop
        finally:
            await asyncio.to_thread(client.close)
            await hub.close()
    asyncio.run(run())


def test_client_rejects_malformed_capture_commands():
    client = SimulationControlClient("127.0.0.1", 0, "test")
    for episode, request in [(True, "id"), ("ep", ""), ("ep", 1), ("x"*129, "id")]:
        client._accept_message(dict(protocol="space-arm-control/1", type="capture_control", episode_id=episode, request_id=request))
        assert client.capture_state()["capture_episode_id"] == ""


def test_cancelled_prepare_does_not_leave_an_orphan_episode(tmp_path):
    import threading
    entered, release = threading.Event(), threading.Event()
    def slow_writer(root, request):
        entered.set()
        assert release.wait(3)
        return Writer(root, request)
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=slow_writer)
        hub = Hub(r)
        lifecycle = CaptureLifecycle(hub, r)
        start = asyncio.create_task(lifecycle.start(EpisodeStart(camera_ids=["wrist"])))
        assert await asyncio.to_thread(entered.wait, 2)
        start.cancel()
        release.set()
        with pytest.raises(asyncio.CancelledError): await start
        assert r.episode_id is None and not hub.calls
        assert not lifecycle.busy
        r.close()
    asyncio.run(run())


def test_stop_confirmation_failure_cannot_report_a_complete_dataset(tmp_path):
    class FailedStopHub(Hub):
        async def set_capture_episode(self, episode, **kwargs):
            await super().set_capture_episode(episode, **kwargs)
            if not episode: raise RuntimeError("capture stop not acknowledged")
    async def run():
        r = EpisodeRecorder(tmp_path, writer_factory=Writer)
        hub = FailedStopHub(r)
        lifecycle = CaptureLifecycle(hub, r)
        info = await lifecycle.start(EpisodeStart(camera_ids=["wrist"]))
        r.record_observation(observation(1, info["episode_id"]), None)
        r.record_authoritative_capture(capture(1, info["episode_id"]), {"rgb": b"jpeg"})
        result = await lifecycle.stop(EpisodeStop())
        assert result["dataset_status"] == "failed"
        assert "stop not acknowledged" in result["dataset_error"]
        assert hub.calls[-1] == "" and r.episode_id is None
        r.close()
    asyncio.run(run())


def test_off_request_is_retained_across_disconnect_and_replayed():
    async def run():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        client = SimulationControlClient("127.0.0.1", hub.bound_port, "test")
        client.start()
        try:
            await asyncio.to_thread(client.wait_until_ready, 2.)
            start = asyncio.create_task(hub.set_capture_episode("episode-A", timeout=2))
            for _ in range(200):
                if client.capture_state()["capture_episode_id"] == "episode-A": break
                await asyncio.sleep(.01)
            state = client.capture_state()
            client.send_observation(observation(0, state["capture_episode_id"], state["capture_request_id"]).model_dump())
            await start
            # Hold reconnect by closing only the listener, not the peer state.
            hub._server.close()
            await hub._server.wait_closed()
            hub._writer.close()
            for _ in range(100):
                if not hub.connected: break
                await asyncio.sleep(.01)
            assert not hub.connected
            with pytest.raises(RuntimeError): await hub.set_capture_episode("", timeout=.02)
            request_id = hub._capture_request["request_id"]
            await hub.start("127.0.0.1", client.port)
            for _ in range(200):
                if client.capture_state()["capture_request_id"] == request_id: break
                await asyncio.sleep(.01)
            assert client.capture_state() == {"capture_episode_id": "", "capture_request_id": request_id}
            client.send_observation(observation(1, "", request_id).model_dump())
            for _ in range(100):
                if not hub.status()["capture_control_error"]: break
                await asyncio.sleep(.01)
            assert not hub.status()["capture_control_error"]
        finally:
            await asyncio.to_thread(client.close)
            await hub.close()
    asyncio.run(run())
