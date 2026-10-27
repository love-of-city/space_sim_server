import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.collect_parallel import CollectionConfig, worker_plan, validated_throughput
from space_arm_platform.collection_runtime import renderer_command


def test_workers_have_disjoint_ports_and_seed_budgets():
    config = CollectionConfig(workers=[{}, {}, {}], max_attempts_per_worker=7)
    plans = [worker_plan(config, i) for i in range(3)]
    ports = [port for plan in plans for port in plan["ports"].values()]
    seeds = [seed for plan in plans for seed in range(plan["seed"], plan["seed"] + 7)]
    assert len(set(ports)) == 12
    assert len(set(seeds)) == 21


@pytest.mark.parametrize("override", [
    {"episodes_per_worker": 5, "max_attempts_per_worker": 4},
    {"workers": []}, {"workers": [{"gpu_index": -1}]},
    {"workers": [{"cpu_affinity": [0, 0]}]}, {"reuse_warmup_seconds": -1.0},
    {"dynamics_max_episodes": 0}, {"dynamics_max_rss_mb": 0},
    {"simulation_startup_timeout_seconds": 0.0},
])
def test_invalid_resource_config_is_rejected(override):
    with pytest.raises(ValueError):
        CollectionConfig.model_validate({"workers": [{}], **override})


def test_throughput_counts_only_validated_episodes_in_whole_batch_time():
    assert validated_throughput([
        {"accepted_episodes": ["a", "b"], "attempts": [{}, {}, {}]},
        {"accepted_episodes": ["c"]},
    ], 180) == {"accepted_episodes": 3, "accepted_episodes_per_hour": 60}


def test_offline_keeps_authoritative_capture_and_binds_gpu(tmp_path, monkeypatch):
    monkeypatch.setattr("space_arm_platform.collection_runtime.unreal_editor", lambda root: root / "UnrealEditor")
    args = SimpleNamespace(adapter=tmp_path, unreal=tmp_path, offline=True, gpu_index=1,
                           control_port=21000, capture_port=21001, render_port=21002, signalling_port=21003)
    command = renderer_command(args, tmp_path)
    assert "-BskOfflineCapture" in command
    assert "-BskCaptureRate=50" in command
    assert "-BskCapturePort=21001" in command
    assert "-graphicsadapter=1" in command
    assert not any("PixelStreaming" in item for item in command)
    args.offline = False
    assert any("PixelStreamingConnectionURL=" in item for item in renderer_command(args, tmp_path))


def test_repository_config_loads_and_refers_to_recording_config():
    path = Path(__file__).resolve().parents[1] / "config/collection.json"
    config = CollectionConfig.model_validate_json(path.read_text(encoding="utf-8"))
    recording = json.loads((path.parent / config.recording_config).read_text(encoding="utf-8"))
    assert recording["capture_rate_hz"] == 50
    assert config.randomize_overview_camera is True
    assert config.reuse_dynamics is True
    assert len(config.workers) == 2


def test_legacy_configuration_does_not_silently_enable_dynamics_reuse():
    assert CollectionConfig(workers=[{}]).reuse_dynamics is False


def test_port_precheck_rejects_an_active_listener():
    import socket
    from space_arm_platform.collection_runtime import check_port_available
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        with pytest.raises(OSError):
            check_port_available(listener.getsockname()[1])


def test_explicit_nvenc_gpu_cannot_silently_use_default_nonstreaming_device():
    from space_arm_platform.recording_config import VideoEncodingConfig
    with pytest.raises(ValueError, match="GPU selection requires streaming"):
        VideoEncodingConfig(codec="h264_nvenc", gpu_index=1)
    assert VideoEncodingConfig(codec="h264_nvenc", gpu_index=1, streaming_encoding=True).gpu_index == 1


@pytest.mark.parametrize("fail", [False, True])
def test_coordinator_runs_concurrently_and_cleans_failed_workers(tmp_path, monkeypatch, fail):
    """Real child processes exercise orchestration without launching UE."""
    import socket
    import subprocess
    import sys
    import tools.collect_parallel as parallel
    monkeypatch.setattr(parallel.tempfile, "gettempdir", lambda: str(tmp_path))

    # Find a complete unused port block for both workers.
    for base in range(33000, 60000, 20):
        sockets = []
        try:
            for port in [base + i for i in (0, 1, 2, 3, 10, 11, 12, 13)]:
                sock = socket.socket()
                sockets.append(sock)
                sock.bind(("127.0.0.1", port))
            break
        except OSError:
            continue
        finally:
            for sock in sockets:
                sock.close()
    fake = tmp_path / "fake.py"
    fake.write_text('''import json,sys,time
from pathlib import Path
plan = json.loads(Path(sys.argv[1]).read_text())
command = plan['command']
output = Path(command[command.index('--output')+1])
cancel = Path(command[command.index('--cancel-file')+1])
output.mkdir()
started = time.time()
if sys.argv[2] == 'True' and output.parent.name == 'worker-00':
    time.sleep(.3)
    raise SystemExit(7)
if sys.argv[2] == 'True':
    deadline = time.monotonic()+10
    while not cancel.exists() and time.monotonic()<deadline: time.sleep(.05)
    (output/'cancelled').write_text(str(cancel.exists()))
    raise SystemExit(1)
time.sleep(.6)
(output/'summary.json').write_text(json.dumps(dict(status='complete',
    accepted_episodes=[str(output/'episode')], started=started, finished=time.time())))
''', encoding="utf-8")
    config = tmp_path / "collection.json"
    config.write_text(json.dumps(dict(workers=[{}, {}], port_base=base,
        episodes_per_worker=1, sample_seconds=1.0, reuse_dynamics=True)), encoding="utf-8")
    (tmp_path / "recording.json").write_text("{}", encoding="utf-8")
    real_popen = subprocess.Popen
    children = []

    def popen(command, **kwargs):
        assert "--worker-plan" in command
        plan = json.loads(Path(command[-1]).read_text())
        child_command = plan["command"]
        assert "--reuse-dynamics" in child_command
        assert child_command[child_command.index("--dynamics-max-episodes") + 1] == "4"
        assert child_command[child_command.index("--simulation-startup-timeout-seconds") + 1] == "600.0"
        child = real_popen([sys.executable, str(fake), command[-1], str(fail)], **kwargs)
        children.append(child)
        return child

    monkeypatch.setattr(parallel.subprocess, "Popen", popen)
    monkeypatch.setattr(parallel.ResourceSampler, "sample", lambda *_: {
        "worker_cpu_percent_one_core": 0, "worker_rss_bytes": 0, "host_cpu_percent": 0})
    args = SimpleNamespace(config=config, workers=None, episodes_per_worker=None,
        output=tmp_path / "result", episodes_root=None, scene=tmp_path / "scene.json",
        task="drawer_cycle", adapter=tmp_path, unreal=tmp_path)
    code = parallel.run(args)
    assert all(child.poll() is not None for child in children)
    report = json.loads((args.output / "summary.json").read_text(encoding="utf-8"))
    if fail:
        assert code == 1 and report["status"] == "failed"
        assert (args.output / "worker-01/batch/cancelled").read_text() == "True"
    else:
        assert code == 0 and report["accepted_episodes"] == 2
        batches = [worker["batch"] for worker in report["workers"]]
        assert max(batch["started"] for batch in batches) < min(batch["finished"] for batch in batches)
