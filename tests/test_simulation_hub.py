import asyncio

from space_arm_platform.models import AppliedAction
from space_arm_platform.protocol import read_async, write_async
from space_arm_platform.simulation_hub import SimulationHub


def test_simulation_hub_is_duplex_and_preserves_decimal_timestamps() -> None:
    async def scenario() -> None:
        hub = SimulationHub()
        await hub.start("127.0.0.1", 0)
        assert hub.bound_port is not None
        reader, writer = await asyncio.open_connection("127.0.0.1", hub.bound_port)
        await write_async(
            writer,
            {
                "protocol": "space-arm-control/1",
                "type": "sim_hello",
                "simulation_id": "test-sim",
                "capabilities": ["joint_velocity"],
            },
        )
        for _ in range(20):
            if hub.connected:
                break
            await asyncio.sleep(0.01)
        assert hub.connected
        action = AppliedAction(
            server_sequence="9007199254740993",
            server_time_ns="90071992547409930",
            client_sequence="1",
            client_time_ns="2",
            deadman=True,
            end_effector_linear_velocity_body_m_s=[0.01, 0.0, 0.0],
            end_effector_angular_velocity_body_rad_s=[0.0, 0.1, 0.0],
            gripper_velocity_rad_s=0.0,
            input_source="keyboard",
        )
        assert await hub.publish_action(action)
        received = await read_async(reader)
        assert received["server_sequence"] == "9007199254740993"
        await write_async(
            writer,
            {
                "protocol": "space-arm-control/1",
                "type": "observation",
                "simulation_id": "test-sim",
                "step_id": "10",
                "render_frame_id": "9",
                "sim_time_ns": "90071992547409931",
                "wall_time_ns": "4",
                "applied_action_sequence": "9007199254740993",
                "joint_position_rad": [0.0] * 6,
                "joint_velocity_rad_s": [0.0] * 6,
                "target_joint_position_rad": [0.0] * 6,
            },
        )
        _, observation = await asyncio.wait_for(hub.wait_for_observation(0), timeout=1.0)
        assert observation is not None
        assert observation.sim_time_ns == "90071992547409931"
        writer.close()
        await writer.wait_closed()
        await hub.close()

    asyncio.run(scenario())
