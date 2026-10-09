"""An old backend must fail before physics, never after 128 unacknowledged frames."""
import asyncio
from pathlib import Path
import time

import pytest

from space_arm_platform.protocol import check_backend_capabilities, read_async
from space_arm_platform.simulation_hub import SimulationHub
from simulation.control_client import SimulationControlClient


def test_preflight_does_not_displace_existing_simulator():
    async def run():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        client = SimulationControlClient("127.0.0.1", hub.bound_port, "live")
        client.start()
        try:
            await asyncio.to_thread(client.wait_until_ready, 2.)
            writer = hub._writer
            stream = client._stream_id
            reply = await asyncio.to_thread(check_backend_capabilities, "127.0.0.1", hub.bound_port)
            assert "capture_ack_v1" in reply["capabilities"]
            assert hub._writer is writer and hub.connected
            assert hub.simulation_id == "live" and stream in hub._stream_acks
            assert not client.observation_transport_status()["observation_pending_ack"]
        finally:
            await asyncio.to_thread(client.close)
            await hub.close()
    asyncio.run(run())


def test_legacy_backend_rejected_without_simulator_registration():
    async def run():
        messages = []
        async def legacy(reader, writer):
            messages.append(await read_async(reader))
            writer.close()
            await writer.wait_closed()
        server = await asyncio.start_server(legacy, "127.0.0.1", 0)
        try:
            port = server.sockets[0].getsockname()[1]
            with pytest.raises(RuntimeError, match="Restart/update the whole platform"):
                await asyncio.to_thread(check_backend_capabilities, "127.0.0.1", port, .5)
            assert [m["type"] for m in messages] == ["capabilities_probe"]
        finally:
            server.close()
            await server.wait_closed()
    asyncio.run(run())


def test_no_ready_means_no_physics_or_queued_observations():
    client = SimulationControlClient("127.0.0.1", 0, "offline")
    with pytest.raises(RuntimeError, match="before physics startup"):
        client.wait_until_ready(.01)
    assert client.observation_transport_status()["observation_pending_ack"] == 0
    client.close()


def test_launcher_checks_backend_before_starting_renderer():
    source = (Path(__file__).resolve().parents[1] / "scripts/start_scene_instance.ps1").read_text(encoding="utf-8")
    assert source.index("tools/check_runtime_backend.py") < source.index("Write-RuntimeState 'starting_renderer'")


def test_physics_start_waits_for_negotiated_protocol():
    # Keep a regression guard on the production path, not only on the helper.
    source = (Path(__file__).resolve().parents[1] / "simulation/teleop_grasp_unreal.py").read_text(encoding="utf-8")
    assert source.index("client.wait_until_ready()") < source.index("reset_request = _run_session(")
