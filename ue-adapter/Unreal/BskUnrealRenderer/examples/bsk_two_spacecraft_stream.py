"""Minimal real Basilisk simulation driving the Unreal runtime renderer."""

from __future__ import annotations

import argparse
import time

from bsk_unreal_adapter import BasiliskUnrealBridge
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    args = parser.parse_args()

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("bskProcess")
    dynamics_task = simulation.CreateNewTask("dynamicsTask", macros.sec2nano(0.01))
    render_task = simulation.CreateNewTask("renderTask", macros.sec2nano(1.0 / 30.0))
    process.addTask(dynamics_task, 10)
    process.addTask(render_task, 0)

    chaser = spacecraft.Spacecraft()
    chaser.ModelTag = "chaser"
    chaser.hub.r_CN_NInit = [7_000_000.0, 0.0, 0.0]
    chaser.hub.v_CN_NInit = [0.0, 0.0, 0.0]
    simulation.AddModelToTask("dynamicsTask", chaser)

    target = spacecraft.Spacecraft()
    target.ModelTag = "target"
    target.hub.r_CN_NInit = [7_000_000.0, 12.0, 0.5]
    target.hub.v_CN_NInit = [0.0, -0.12, 0.0]
    target.hub.sigma_BNInit = [0.0, 0.0, 0.15]
    target.hub.omega_BN_BInit = [0.0, 0.0, 0.05]
    simulation.AddModelToTask("dynamicsTask", target)

    bridge = BasiliskUnrealBridge(args.host, args.port, origin_object="chaser")
    bridge.add_spacecraft(chaser, name="chaser")
    bridge.add_spacecraft(target, name="target")
    simulation.AddModelToTask("renderTask", bridge)

    simulation.InitializeSimulation()
    # Advance in render-sized chunks and pace wall time. Basilisk remains the
    # sole dynamics authority; this only makes the live visualization human-
    # viewable instead of finishing an offline simulation as fast as possible.
    render_rate_hz = 30.0
    frame_count = max(1, int(args.duration * render_rate_hz))
    wall_start = time.monotonic()
    try:
        for frame_index in range(1, frame_count + 1):
            simulation.ConfigureStopTime(macros.sec2nano(frame_index / render_rate_hz))
            simulation.ExecuteSimulation()
            deadline = wall_start + frame_index / render_rate_hz
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
    print(
        f"BSK frames queued={bridge.publisher.stats.queued}, "
        f"sent={bridge.publisher.stats.sent}, dropped={bridge.publisher.stats.dropped}"
    )


if __name__ == "__main__":
    main()
