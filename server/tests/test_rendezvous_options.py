"""Bad mission input is rejected before touching the current run."""
import json
from pathlib import Path
import sys
import pytest
from space_arm_platform.rendezvous_config import FlightOptions
from space_arm_platform.rendezvous_runtime import RendezvousRuntime


@pytest.mark.parametrize("values", [{"seed": True}, {"seed": -1}, {"seed": 2**32},
    {"drag_enabled": "false"}, {"orbit_acceleration_sigma_m_s2": float("nan")},
    {"gnss_position_sigma_m": 0}, {"handoff_distance_m": 99}, {"arbitrary_script": "x"},
    {"differential_gnss": {"delay_s": .3}}, {"differential_gnss": {"delay_s": float('nan')}}])
def test_bad_options_do_not_start_or_interrupt_a_run(tmp_path, monkeypatch, values):
    runtime = RendezvousRuntime(None, Path(sys.executable), 123, tmp_path, tmp_path)
    def unexpected(*args, **kwargs):
        pytest.fail("invalid options changed process state")
    monkeypatch.setattr(runtime, "close", unexpected)
    monkeypatch.setattr(runtime, "stop", unexpected)
    monkeypatch.setattr("space_arm_platform.rendezvous_runtime.subprocess.Popen", unexpected)
    with pytest.raises(ValueError):
        runtime.start(scenario="team-in-orbit", options=values)


@pytest.mark.parametrize('delay', [0, .2])
def test_saved_options_are_exactly_what_the_flight_receives(tmp_path, monkeypatch, delay):
    entry = tmp_path / "simulation/orbital_rendezvous/run.py"
    entry.parent.mkdir(parents=True)
    entry.touch()
    launched = []
    class Process:
        pid = 42
        def __init__(self, command, **kwargs):
            launched.append(command)
        def poll(self):
            return 0
    monkeypatch.setattr("space_arm_platform.rendezvous_runtime.subprocess.Popen", Process)
    runtime = RendezvousRuntime(None, Path(sys.executable), 123, tmp_path / "logs", tmp_path)
    options = {"seed": 42, "drag_enabled": False,
               **({"differential_gnss": {"delay_s": delay}} if delay else {})}
    runtime.start(scenario="team-orbit-raise", options=options)
    path = Path(launched[0][launched[0].index("--config") + 1])
    assert json.loads(path.read_text()) == FlightOptions.from_dict(options).to_dict()
    runtime.close()


@pytest.mark.parametrize('scenario,mode', [('legacy', 'simulation'), ('legacy', 'demo'), ('team-in-orbit', 'demo')])
def test_delayed_options_cannot_silently_launch_an_unsupported_legacy_or_replay_path(tmp_path, monkeypatch, scenario, mode):
    runtime = RendezvousRuntime(None, Path(sys.executable), 123, tmp_path/'logs', tmp_path)
    def unexpected(*args, **kwargs):
        pytest.fail('Unsupported delayed navigation changed the existing run')
    monkeypatch.setattr(runtime, 'stop', unexpected)
    monkeypatch.setattr('space_arm_platform.rendezvous_runtime.subprocess.Popen', unexpected)
    with pytest.raises(ValueError, match='通信时延'):
        runtime.start(mode=mode, scenario=scenario, options={'differential_gnss': {'delay_s': .2}})


def test_restart_retains_scenario_and_parameters(tmp_path):
    folder = tmp_path / "run/team_rendezvous/completed"
    folder.mkdir(parents=True)
    (folder / "mission_report.json").write_text(json.dumps({"scenario": "orbit-raise", "mission_options": {"seed": 19}}))
    runtime = RendezvousRuntime(None, Path(sys.executable), 123, tmp_path / "logs", tmp_path)
    assert runtime.status()["scenario"] == "team-orbit-raise"
    assert runtime.status()["mission_options"]["seed"] == 19
