"""Send a short dead-man-protected Cartesian command for integration testing."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import time

import websockets


async def run(url: str, duration: float, axis: int, magnitude: float) -> None:
    latest_observation = None
    async with websockets.connect(url) as websocket:
        session = json.loads(await websocket.recv())
        if not session.get("control_granted"):
            raise RuntimeError("the test connection did not receive operator control")

        async def receive() -> None:
            nonlocal latest_observation
            async for raw in websocket:
                message = json.loads(raw)
                if message.get("type") == "observation":
                    latest_observation = message.get("payload")

        receiver = asyncio.create_task(receive())
        sequence = 0
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline:
                sequence += 1
                linear = [0.0, 0.0, 0.0]
                linear[axis] = magnitude
                await websocket.send(
                    json.dumps(
                        {
                            "type": "operator_action",
                            "client_sequence": sequence,
                            "client_time_ns": str(time.time_ns()),
                            "deadman": True,
                            "end_effector_linear_velocity": linear,
                            "end_effector_angular_velocity": [0.0, 0.0, 0.0],
                            "gripper_velocity": 0.0,
                            "input_source": "unknown",
                        }
                    )
                )
                await asyncio.sleep(1.0 / 30.0)
            sequence += 1
            await websocket.send(
                json.dumps(
                    {
                        "type": "operator_action",
                        "client_sequence": sequence,
                        "client_time_ns": str(time.time_ns()),
                        "deadman": False,
                        "end_effector_linear_velocity": [0.0, 0.0, 0.0],
                        "end_effector_angular_velocity": [0.0, 0.0, 0.0],
                        "gripper_velocity": 0.0,
                        "input_source": "unknown",
                    }
                )
            )
            await asyncio.sleep(0.2)
        finally:
            receiver.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await receiver
    print(json.dumps(latest_observation, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8000/ws/operator")
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--axis", type=int, choices=(0, 1, 2), default=0)
    parser.add_argument("--magnitude", type=float, default=0.25)
    args = parser.parse_args()
    asyncio.run(run(args.url, args.duration, args.axis, args.magnitude))


if __name__ == "__main__":
    main()
