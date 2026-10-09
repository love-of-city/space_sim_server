"""Run the unmodified Demo 8 dynamics through the BSK render interface.

The scenario imports Basilisk's ``scenarioMJSceneVizard`` and temporarily
replaces only its visualization registration hook. Dynamics, controllers,
gravity, initial conditions, and MuJoCo XML remain owned by the upstream
scenario. It can either record every renderer state or stream states to UE
while the authoritative simulation is running.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
import time
from pathlib import Path

from bsk_render_adapter import (
    CameraVisual,
    RecordingOnlyPublisher,
    SceneSettings,
)
from bsk_render_adapter import ue_support as ueSupport


def install_wall_clock_pacing(simulation, simulation_rate: float) -> None:
    """Pace an existing simulation without inserting another BSK task model."""

    if simulation_rate <= 0.0:
        raise ValueError("simulation_rate must be greater than zero")
    original_execute = simulation.ExecuteSimulation

    def execute_paced() -> None:
        final_stop_time = int(simulation.StopTime)
        simulation_start_ns = int(simulation.TotalSim.NextTaskTime)
        wall_start = time.monotonic()
        show_progress = simulation.showProgressBar
        simulation.showProgressBar = False
        try:
            while int(simulation.TotalSim.NextTaskTime) <= final_stop_time:
                next_task_time = int(simulation.TotalSim.NextTaskTime)
                simulated_seconds = (next_task_time - simulation_start_ns) * 1.0e-9
                deadline = wall_start + simulated_seconds / simulation_rate
                time.sleep(max(0.0, deadline - time.monotonic()))
                simulation.ConfigureStopTime(next_task_time)
                original_execute()
        finally:
            simulation.ConfigureStopTime(final_stop_time)
            simulation.showProgressBar = show_progress
            # Remove the instance override and its bound-method reference cycle.
            del simulation.ExecuteSimulation

    simulation.ExecuteSimulation = execute_paced


def _load_upstream(basilisk_root: Path):
    scenario_path = basilisk_root / "examples" / "mujoco" / "scenarioMJSceneVizard.py"
    if not scenario_path.is_file():
        raise FileNotFoundError(
            f"Basilisk Demo 8 source was not found: {scenario_path}. "
            "Pass --basilisk-root or set BSK_SOURCE_ROOT when using the PowerShell scripts."
        )
    scenario_directory = str(scenario_path.parent)
    if scenario_directory not in sys.path:
        sys.path.insert(0, scenario_directory)
    specification = importlib.util.spec_from_file_location("bsk_upstream_demo8", scenario_path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {scenario_path}")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def run_demo(
    basilisk_root: Path,
    *,
    output: Path | None = None,
    live: bool = False,
    host: str = "127.0.0.1",
    port: int = 5558,
    simulation_rate: float = 120.0,
) -> None:
    if not live and output is None:
        raise ValueError("output is required in recording mode")
    if simulation_rate <= 0.0:
        raise ValueError("simulation_rate must be greater than zero")

    upstream = _load_upstream(basilisk_root)
    bridges = []
    pacing_installed = False

    def enable_unreal(simulation, task_name, simulation_objects, **kwargs):
        nonlocal pacing_installed
        transport_options = {
            "host": host,
            "port": port,
            "recordingPath": output,
        }
        if not live:
            transport_options["publisher"] = RecordingOnlyPublisher()
        bridge = ueSupport.enableUnrealVisualization(
            simulation,
            task_name,
            simulation_objects,
            originObject="primary/hub",
            namespaceList=["primary", "companion", "trailer"],
            **transport_options,
            **kwargs,
        )
        bridge.add_camera(
            CameraVisual(
                camera_id="primary/overview_camera",
                parent_id="primary/hub",
                position_body_m=[10.0, -16.0, 8.0],
                field_of_view_rad=1.0471975511965976,
            )
        )
        bridge.set_scene_settings(
            SceneSettings(
                origin_object_id="primary/hub",
                default_camera_target="primary/hub",
                orbit_lines=True,
                trajectory_history=True,
                interpolation_delay_ms=100.0,
                max_extrapolation_ms=100.0,
            )
        )
        bridges.append(bridge)
        if live and not pacing_installed:
            # The upstream task period is four simulated seconds. At the
            # default 120x rate this produces one state every 1/30 wall second.
            # Pacing wraps execution without changing the BSK task graph and
            # never waits for UE acknowledgements.
            install_wall_clock_pacing(simulation, simulation_rate)
            pacing_installed = True
        return bridge

    original_enable = upstream.vizSupport.enableUnityVisualization
    original_instrument = upstream.vizSupport.setInstrumentGuiSetting
    original_found = upstream.vizSupport.vizFound
    upstream.vizSupport.vizFound = True
    upstream.vizSupport.enableUnityVisualization = enable_unreal
    upstream.vizSupport.setInstrumentGuiSetting = ueSupport.setInstrumentGuiSetting
    try:
        upstream.run(liveStream=live)
    finally:
        for bridge in bridges:
            bridge.close()
            if live and hasattr(bridge.publisher, "stats"):
                stats = bridge.publisher.stats
                print(
                    "Demo 8 network frames "
                    f"queued={stats.queued}, sent={stats.sent}, dropped={stats.dropped}"
                )
        upstream.vizSupport.enableUnityVisualization = original_enable
        upstream.vizSupport.setInstrumentGuiSetting = original_instrument
        upstream.vizSupport.vizFound = original_found


def record_demo(basilisk_root: Path, output: Path) -> None:
    """Compatibility wrapper retained for recording-focused callers."""

    run_demo(basilisk_root, output=output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--basilisk-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--live", action="store_true", help="stream while BSK/MJScene is running")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument(
        "--simulation-rate",
        type=float,
        default=120.0,
        help="simulated seconds per wall-clock second in live mode (default: 120)",
    )
    args = parser.parse_args()
    if not args.live and args.output is None:
        parser.error("--output is required unless --live is selected")
    output = args.output.resolve() if args.output is not None else None
    run_demo(
        args.basilisk_root.resolve(),
        output=output,
        live=args.live,
        host=args.host,
        port=args.port,
        simulation_rate=args.simulation_rate,
    )
    if args.live:
        print(f"Demo 8 live stream finished ({args.simulation_rate:g}x simulation rate)")
    else:
        print(f"Demo 8 recording written to {output}")


if __name__ == "__main__":
    main()
