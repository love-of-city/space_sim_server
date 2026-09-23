"""Regression coverage for cold-start stalls, ACK loss, framing and bounded IO."""
import asyncio
import json
import socket
import threading
import time

import pytest

from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.protocol import FramedSocketReader, encode_packet
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub
from simulation.control_client import SimulationControlClient


def observation(i):
    return SimulationObservation(protocol="space-arm-control/1", type="observation", simulation_id="test",
        render_session_id="session", step_id=str(i + 1), render_frame_id=str(i),
        sim_time_ns=str((i * 1_000_000_000 + 15) // 30), wall_time_ns=str(time.time_ns()),
        applied_action_sequence="0", joint_position_rad=[0.] * 8, joint_velocity_rad_s=[0.] * 8,
        target_joint_position_rad=[0.] * 8)


class CountingWriter:
    def __init__(self, root, request):
        self.frames = 0
        self.fps = request.fps
        self.cameras = tuple(request.camera_ids)
        self.entered = threading.Event()
        self.release = threading.Event()
        self.release.set()
    def append(self, obs, caps):
        self.entered.set()
        assert self.release.wait(5)
        self.frames += 1
    def finish(self, *args): pass
    def close_failed(self): pass


def test_cold_initialization_does_not_hold_control_lock(tmp_path):
    def cold_writer(root, req):
        time.sleep(.3)
        return CountingWriter(root, req)
    recorder = EpisodeRecorder(tmp_path, writer_factory=cold_writer)
    async def run():
        task = asyncio.create_task(asyncio.to_thread(recorder.start, EpisodeStart(camera_ids=[])))
        gaps = []
        while not task.done():
            start = time.monotonic()
            assert recorder.episode_id is None or task.done() or not recorder.sync_status()["preparing"]
            recorder.sync_status()
            await asyncio.sleep(.01)
            gaps.append(time.monotonic() - start)
        await task
        assert max(gaps) < .15
    asyncio.run(run())


def test_slow_writer_and_full_queue_do_not_block_status_and_preserve_order(tmp_path):
    recorder = EpisodeRecorder(tmp_path, writer_factory=CountingWriter, drain_timeout_s=2)
    recorder.MAX_WRITE_JOBS = 4
    recorder.start(EpisodeStart(camera_ids=[]))
    recorder._dataset.release.clear()
    recorder.record_observation(observation(0), None)
    assert recorder._dataset.entered.wait(1)
    done = threading.Event()
    def produce():
        for i in range(1, 12):
            recorder.record_observation(observation(i), None)
        done.set()
    producer = threading.Thread(target=produce)
    producer.start()
    time.sleep(.1)
    start = time.monotonic()
    status = recorder.sync_status()
    assert recorder.episode_id
    assert time.monotonic() - start < .1
    assert status["pending_write_jobs"] <= 4
    assert not done.is_set()
    recorder._dataset.release.set()
    producer.join(5)
    assert done.is_set()
    result = recorder.stop(EpisodeStop())
    assert result["dataset_status"] == "complete", result
    assert result["dataset_frame_count"] == result["step_count"] == 12


def test_state_gap_fails_immediately_not_after_pairing_window_is_poisoned(tmp_path):
    r = EpisodeRecorder(tmp_path, writer_factory=CountingWriter)
    r.start(EpisodeStart())
    r.record_observation(observation(1299), None)
    r.record_observation(observation(1300), None)
    r.record_observation(observation(1636), None)
    error = r.sync_status()["dataset_error"]
    assert "expected_tick=1301" in error and "received_tick=1636" in error
    assert r.sync_status()["pending_dataset_samples"] == 2
    assert r.stop(EpisodeStop())["dataset_status"] == "failed"


def test_partial_packet_survives_timeout():
    left, right = socket.socketpair()
    try:
        right.settimeout(.02)
        packet = encode_packet({"hello": "world"})
        reader = FramedSocketReader()
        left.sendall(packet[:2])
        with pytest.raises(socket.timeout): reader.receive(right)
        left.sendall(packet[2:7])
        with pytest.raises(socket.timeout): reader.receive(right)
        left.sendall(packet[7:])
        assert reader.receive(right) == {"hello": "world"}
    finally:
        left.close(); right.close()


def test_ack_timeout_fails_closed_without_replacing_pending_state():
    c = SimulationControlClient("127.0.0.1", 0, "test", max_pending_observations=2, observation_timeout_s=.03)
    c.send_observation(observation(0).model_dump())
    c.send_observation(observation(1).model_dump())
    with pytest.raises(RuntimeError, match="refusing to drop"):
        c.send_observation(observation(2).model_dump())
    assert list(c._pending_observations) == [1, 2]
    c.close()


def test_lost_ack_reconnect_replays_without_duplicate_recording():
    async def run():
        hub = SimulationHub(); await hub.start("127.0.0.1", 0)
        seen = []
        async def record(obs): seen.append(int(obs.render_frame_id))
        hub.on_observation = record
        ack = hub._ack
        dropped = False
        async def lose_ack(writer, stream, sequence, kind="observation_ack"):
            nonlocal dropped
            if kind == "observation_ack" and sequence == 3 and not dropped:
                dropped = True
                writer.close()  # Simulate accepted state followed by loss of its ACK.
                return
            await ack(writer, stream, sequence, kind)
        hub._ack = lose_ack
        c = SimulationControlClient("127.0.0.1", hub.bound_port, "test")
        c.start()
        try:
            for i in range(12): c.send_observation(observation(i).model_dump())
            deadline = time.monotonic() + 8
            while (len(seen) < 12 or c.observation_transport_status()["observation_pending_ack"]) and time.monotonic() < deadline:
                await asyncio.sleep(.02)
            assert seen == list(range(12))
            assert not c.observation_transport_status()["observation_pending_ack"]
            assert c.observation_transport_status()["observation_reconnects"] >= 2
        finally:
            await asyncio.to_thread(c.close); await hub.close()
    asyncio.run(run())


def test_downstream_stall_backpressures_producer_but_not_event_loop():
    async def run():
        hub = SimulationHub(); await hub.start("127.0.0.1", 0)
        release = asyncio.Event(); seen=[]
        async def record(obs):
            await release.wait();seen.append(int(obs.render_frame_id))
        hub.on_observation=record
        c=SimulationControlClient("127.0.0.1",hub.bound_port,"test",max_pending_observations=4)
        c.start()
        def produce():
            for i in range(20):c.send_observation(observation(i).model_dump())
        task=asyncio.create_task(asyncio.to_thread(produce))
        try:
            await asyncio.sleep(.2)
            assert not task.done()
            assert c.observation_transport_status()["observation_pending_ack"] == 4
            release.set();await asyncio.wait_for(task,5)
            for _ in range(200):
                if len(seen)==20 and not c.observation_transport_status()["observation_pending_ack"]:break
                await asyncio.sleep(.01)
            assert seen==list(range(20))
        finally:
            release.set();await asyncio.to_thread(c.close);await hub.close()
    asyncio.run(run())


@pytest.mark.process_writer
def test_spawned_writer_cold_start_and_two_recordings(tmp_path):
    r = EpisodeRecorder(tmp_path)
    try:
        async def prepare():
            task = asyncio.create_task(asyncio.to_thread(r.prepare))
            gaps=[]
            while not task.done():
                t=time.monotonic();r.sync_status();await asyncio.sleep(.02);gaps.append(time.monotonic()-t)
            await task
            assert max(gaps, default=0) < .5
        asyncio.run(prepare())
        child_pid=r._service._process.pid
        for run in range(2):
            info=r.start(EpisodeStart(camera_ids=[]))
            for i in range(4):r.record_observation(observation(i),None)
            result=r.stop(EpisodeStop())
            assert result["dataset_status"]=="complete",result
            assert result["dataset_frame_count"]==4
            meta=json.loads((tmp_path/info["episode_id"]/'lerobot/meta/info.json').read_text())
            assert meta["total_frames"]==4 and meta["fps"]==30
            assert r._service._process.pid==child_pid
    finally:r.close()
