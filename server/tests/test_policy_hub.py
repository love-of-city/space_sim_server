"""A real loopback TCP peer checks transport only, not simulator execution."""
import asyncio
import contextlib

import pytest

from space_arm_platform.protocol import read_async, write_async
from space_arm_platform.simulation_hub import SimulationHub


PROTOCOL = "space-arm-control/1"


def test_policy_rpc_matches_tcp_receipts_and_invalidates_on_disconnect():
    async def scenario():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        writer = None
        pending = None
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", hub.bound_port)
            await write_async(writer, dict(protocol=PROTOCOL, type="sim_hello", simulation_id="fake-policy-sim",
                capabilities=["joint_policy_v1", "reliable_observations_v1"],
                observation_stream_id="policy-wire-test", observation_resume_after="0"))
            ready = await asyncio.wait_for(read_async(reader), 1)
            assert ready["type"] == "observation_ready"
            assert hub.connected and hub.joint_policy_supported
            actions = [[0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 1.0]]
            pending = asyncio.create_task(hub.policy_command("policy-A", "step", request_id="wire-step",
                timeout=10, actions=actions, period_ns=20_000_000))
            packet = await asyncio.wait_for(read_async(reader), 1)
            assert packet == dict(protocol=PROTOCOL, type="joint_policy_command", session_id="policy-A",
                operation="step", request_id="wire-step", reset_generation="", actions=actions, period_ns=20_000_000)
            with pytest.raises(RuntimeError, match="already pending"):
                await hub.policy_command("policy-A", "step", request_id="wire-step", actions=actions)
            receipt = dict(protocol=PROTOCOL, type="joint_policy_result", session_id="policy-A",
                request_id="wire-step", operation="step", ok=True, executed_steps=1,
                render_session_id="renderer-A", render_frame_id="9007199254740993",
                sim_time_ns="9007199260000000")
            await write_async(writer, {**receipt, "request_id": "unrelated-request"})
            # The ACK proves the prior unrelated result has been read. This avoids
            # sleeping and hoping the receive loop ran before checking the future.
            await write_async(writer, dict(protocol=PROTOCOL, type="observation", simulation_id="fake-policy-sim",
                observation_stream_id="policy-wire-test", observation_sequence="1", step_id="1",
                render_frame_id="9007199254740993", render_session_id="renderer-A",
                sim_time_ns="9007199260000000", wall_time_ns="1", applied_action_sequence="0",
                joint_position_rad=[0.0] * 6, joint_velocity_rad_s=[0.0] * 6, target_joint_position_rad=[0.0] * 6))
            assert (await asyncio.wait_for(read_async(reader), 1))["type"] == "observation_ack"
            assert not pending.done(), "An unrelated receipt must not complete the pending step"
            await write_async(writer, receipt)
            assert await asyncio.wait_for(pending, 1) == receipt
            assert hub.latest_observation.render_frame_id == "9007199254740993"

            pending = asyncio.create_task(hub.policy_command("policy-A", "acquire", request_id="wrong-session", timeout=10))
            assert (await asyncio.wait_for(read_async(reader), 1))["request_id"] == "wrong-session"
            await write_async(writer, {**receipt, "request_id": "wrong-session", "session_id": "other-policy"})
            with pytest.raises(RuntimeError, match="receipt"):
                await asyncio.wait_for(pending, 1)

            pending = asyncio.create_task(hub.policy_command("policy-A", "step", request_id="runtime-error", timeout=10))
            assert (await asyncio.wait_for(read_async(reader), 1))["request_id"] == "runtime-error"
            await write_async(writer, {**receipt, "request_id": "runtime-error", "ok": False, "error": "joint target violates limits"})
            with pytest.raises(RuntimeError, match="joint target violates limits"):
                await asyncio.wait_for(pending, 1)

            pending = asyncio.create_task(hub.policy_command("policy-A", "step", request_id="lost-step", timeout=10))
            assert (await asyncio.wait_for(read_async(reader), 1))["request_id"] == "lost-step"
            writer.close()
            await writer.wait_closed()
            # Must fail because the connection was lost, well before the RPC timeout.
            with pytest.raises(RuntimeError):
                await asyncio.wait_for(pending, 1)
            assert not hub.connected
        finally:
            if pending is not None:
                if not pending.done():
                    pending.cancel()
                await asyncio.gather(pending, return_exceptions=True)
            if writer is not None:
                writer.close()
                with contextlib.suppress(ConnectionError, OSError):
                    await writer.wait_closed()
            await hub.close()

    asyncio.run(scenario())
