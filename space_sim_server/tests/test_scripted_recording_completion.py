"""Task completion must follow acceptance, both RGB images, and writer append."""
import asyncio
import json
import threading

import pytest

from space_arm_platform.capture_lifecycle import CaptureLifecycle
from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub
from tools.record_drawer_task import CAMERAS, ScriptedObservationRecorder, validate_episode


class Writer:
    def __init__(self, root, request):
        self.fps, self.cameras = request.fps, request.camera_ids
        self.frames = 0
        self.observations = []
        self.appending = threading.Event()
        self.allow_append = threading.Event()

    def append(self, observation, captures):
        assert set(captures) == set(CAMERAS)
        self.appending.set()
        assert self.allow_append.wait(5), "writer release timed out"
        self.observations.append(observation)
        self.frames += 1

    def finish(self, *args):
        pass

    def close_failed(self):
        pass


def terminal_observation(episode, result):
    return SimulationObservation(
        protocol="space-arm-control/1", type="observation", simulation_id="test",
        render_session_id="session", step_id="2", render_frame_id="1",
        sim_time_ns="20000000", wall_time_ns="1", applied_action_sequence="0",
        capture_episode_id=episode, joint_position_rad=[0.] * 8,
        joint_velocity_rad_s=[0.] * 8, target_joint_position_rad=[0.] * 8,
        scripted_task={"stage": "verify", "result": result, "reason": "finished"})


def image(camera, episode):
    return dict(protocol="bsk-capture/1", session_id="session", camera_id=camera,
                capture_sequence="1", source_frame_id="1", sim_time_ns="20000000",
                capture_episode_id=episode, stream_kind="authoritative",
                state_kind="authoritative", products=[dict(name="rgb", file_name="rgb.jpg")])


async def wait_event(event):
    assert await asyncio.wait_for(asyncio.to_thread(event.wait, 3), 4)


@pytest.mark.parametrize("result", ["success", "failed"])
def test_terminal_task_waits_for_observation_both_cameras_and_writer(tmp_path, monkeypatch, result):
    recorder = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=5)
    episode = recorder.start(EpisodeStart(camera_ids=CAMERAS), capture_on_demand=True)["episode_id"]
    writer = recorder._dataset
    record_entered, allow_record, pair_entered = (threading.Event() for _ in range(3))
    original_record = recorder.record_observation
    original_pair = recorder.wait_for_capture_pair

    def delayed_record(observation, action):
        record_entered.set()
        assert allow_record.wait(5), "observation release timed out"
        original_record(observation, action)

    def wait_for_pair(observation):
        pair_entered.set()
        original_pair(observation)

    monkeypatch.setattr(recorder, "record_observation", delayed_record)
    monkeypatch.setattr(recorder, "wait_for_capture_pair", wait_for_pair)

    async def scenario():
        hub = SimulationHub()
        hub._capabilities.add("capture_pair_ack_v1")
        observations = ScriptedObservationRecorder(hub, recorder)
        hub.on_observation = observations
        pending = asyncio.create_task(hub._accept_observation(terminal_observation(episode, result)))
        try:
            await wait_event(record_entered)
            # Reproduce the original ordering: the main loop sees the terminal
            # result while the recording callback has not acquired its lock yet.
            assert hub.latest_observation.model_extra["scripted_task"]["result"] == result
            assert recorder._step_index == 0
            assert observations.terminal_task is None

            allow_record.set()
            await wait_event(pair_entered)
            assert observations.terminal_task is None
            await asyncio.to_thread(recorder.record_authoritative_capture,
                                    image(CAMERAS[0], episode), {"rgb": b"first RGB"})
            assert observations.terminal_task is None
            await asyncio.to_thread(recorder.record_authoritative_capture,
                                    image(CAMERAS[1], episode), {"rgb": b"second RGB"})
            await wait_event(writer.appending)
            assert observations.terminal_task is None
            writer.allow_append.set()
            await asyncio.wait_for(pending, 3)
            assert observations.terminal_task["result"] == result
            assert recorder.sync_status()["capture_boundary_frozen"]
            # A later queued state must not extend the dataset beyond the
            # actual terminal pair while the main loop has not sent OFF yet.
            later = terminal_observation(episode, "failed").model_copy(
                update={"step_id": "3", "render_frame_id": "2", "sim_time_ns": "40000000"})
            await asyncio.wait_for(hub._accept_observation(later), 3)
            assert writer.frames == 1
            assert observations.terminal_task["result"] == result
            closed = await CaptureLifecycle(hub, recorder).stop(
                EpisodeStop(outcome="success" if result == "success" else "failure"))
            validate_episode(closed)
            assert closed["dataset_frame_count"] == closed["step_count"] == 1
            assert closed["capture_count"] == 2
            assert writer.observations[-1].model_extra["scripted_task"]["result"] == result
            saved = [json.loads(row) for row in (tmp_path / episode / "steps.jsonl").read_text().splitlines()]
            assert saved[-1]["observation"]["scripted_task"]["result"] == result
        finally:
            allow_record.set()
            writer.allow_append.set()
            if not pending.done():
                recorder.fail("test cleanup")
                await asyncio.wait_for(pending, 3)

    asyncio.run(scenario())


@pytest.mark.parametrize("release", ["failure", "freeze"])
def test_failed_or_frozen_pairing_never_confirms_terminal_task(tmp_path, monkeypatch, release):
    recorder = EpisodeRecorder(tmp_path, writer_factory=Writer, drain_timeout_s=5)
    episode = recorder.start(EpisodeStart(camera_ids=CAMERAS), capture_on_demand=True)["episode_id"]
    pair_entered = threading.Event()
    original_pair = recorder.wait_for_capture_pair

    def wait_for_pair(observation):
        pair_entered.set()
        original_pair(observation)

    monkeypatch.setattr(recorder, "wait_for_capture_pair", wait_for_pair)

    async def scenario():
        hub = SimulationHub()
        hub._capabilities.add("capture_pair_ack_v1")
        observations = ScriptedObservationRecorder(hub, recorder)
        hub.on_observation = observations
        pending = asyncio.create_task(hub._accept_observation(terminal_observation(episode, "success")))
        await wait_event(pair_entered)
        if release == "failure":
            recorder.fail("camera capture failed")
        else:
            recorder.freeze_observations()
        await asyncio.wait_for(pending, 3)
        assert observations.terminal_task is None

    asyncio.run(scenario())
