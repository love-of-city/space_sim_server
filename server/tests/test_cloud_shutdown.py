import asyncio

from simulation.control_client import SimulationControlClient
from space_arm_platform.simulation_hub import SimulationHub
from space_arm_platform.rendezvous_hub import RendezvousHub


def test_simulation_shutdown_closes_an_active_peer_before_waiting_for_server():
    async def run():
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        client = SimulationControlClient("127.0.0.1", hub.bound_port, "shutdown-test")
        client.start()
        try:
            await asyncio.to_thread(client.wait_until_ready, 2)
            await asyncio.wait_for(hub.close(), 2)
            assert not hub.connected and hub.bound_port is None
        finally:
            await asyncio.to_thread(client.close)
            await hub.close()
    asyncio.run(run())


def test_rendezvous_shutdown_closes_all_idle_connections():
    async def run():
        hub = RendezvousHub()
        await hub.start("127.0.0.1", 0)
        peers = [await asyncio.open_connection("127.0.0.1", hub.bound_port) for _ in range(2)]
        try:
            await asyncio.sleep(.05)
            await asyncio.wait_for(hub.close(), 2)
            for reader, _ in peers:
                assert await asyncio.wait_for(reader.read(), 1) == b""
            assert hub.bound_port is None and not hub.status()["connected"]
        finally:
            for _, writer in peers:
                writer.close()
                await writer.wait_closed()
            await hub.close()
    asyncio.run(run())
