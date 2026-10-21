"""Linux launch ownership, recovery and production scene-command contracts."""
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from unittest.mock import Mock

import pytest

from space_arm_platform import linux_deployment as deployment
from space_arm_platform import linux_scene
from space_arm_platform import linux_public
from space_arm_platform.linux_processes import Children, identity, owned_process, terminate
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.main import RedactWebSocketSecrets
from space_arm_platform.scene_runtime import SceneLaunchConfig, SceneRuntimeManager

pytestmark = pytest.mark.skipif(sys.platform != "linux", reason="Linux UID and process ownership")


def test_account_only_default_clears_inherited_access_key(tmp_path, monkeypatch):
    monkeypatch.setenv("SPACE_SIM_STREAM_ACCESS_KEY", "stale-shell-key")
    config = deployment.settings(tmp_path / "missing.json")
    assert config["require_access_key"] is False
    env = deployment.environment(config)
    assert "SPACE_SIM_STREAM_ACCESS_KEY" not in env


def test_python_venv_symlink_is_preserved(tmp_path):
    link = tmp_path / "venv/bin/python"
    link.parent.mkdir(parents=True)
    link.symlink_to(sys.executable)
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"python": str(link)}))
    assert deployment.settings(config)["python"] == str(link)


@pytest.mark.parametrize("updates", [{"api_port": 18080}, {"api_port": 0}, {"api_port": True}, {"unknown": 1}])
def test_bad_config_rejected(tmp_path, updates):
    config = tmp_path / "config.json"
    config.write_text(json.dumps(updates))
    with pytest.raises(ValueError):
        deployment.settings(config)


def test_stale_pid_and_other_uid_are_never_signalled():
    record = identity(os.getpid())
    assert owned_process(record) is not None
    assert owned_process({**record, "created": record["created"] - 1}) is None
    assert owned_process({**record, "uid": record["uid"] + 1}) is None
    assert owned_process({"pid": os.getpid()}) is None
    terminate({**record, "created": record["created"] - 1}, timeout=.01)


def test_children_are_reaped_and_unrelated_process_survives(tmp_path):
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    children = Children()
    try:
        child = children.start("test", [sys.executable, "-c", "import time; time.sleep(60)"],
                               cwd=tmp_path, log=tmp_path / "child.log")
        record = children.records["test"]
        assert owned_process(record) is not None
        children.close(["test"], timeout=2)
        assert child.poll() is not None
        assert owned_process(record) is None
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait()


def test_platform_start_failure_closes_started_children(tmp_path, monkeypatch):
    (tmp_path / "run").mkdir()
    (tmp_path / "logs").mkdir()
    config_file = tmp_path / "config.json"
    config_file.write_text(json.dumps({"mode": "local"}))
    monkeypatch.setattr(deployment, "ROOT", tmp_path)
    monkeypatch.setattr(deployment, "STATE", tmp_path / "run/linux_platform.json")
    monkeypatch.setattr(deployment, "credentials", lambda: {"admin_password": "test", "jwt_secret": "test-secret"})
    monkeypatch.setattr(deployment, "check_port_available", lambda port: None)
    closed = Mock()
    monkeypatch.setattr(deployment.Children, "start", Mock(side_effect=RuntimeError("test startup failure")))
    monkeypatch.setattr(deployment.Children, "close", closed)
    monkeypatch.setattr(linux_scene, "stop_scene", Mock())
    with pytest.raises(RuntimeError, match="test startup failure"):
        deployment.supervise(config_file)
    closed.assert_called_once()
    state = json.loads(deployment.STATE.read_text())
    assert state["phase"] == "failed"


def test_shutdown_cleans_descendant_in_a_separate_session(tmp_path):
    children = Children()
    pid_file = tmp_path / "grandchild.pid"
    command = [sys.executable, "-c",
               "import pathlib,subprocess,sys,time; "
               "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True); "
               "pathlib.Path(sys.argv[1]).write_text(str(p.pid)); time.sleep(60)", str(pid_file)]
    descendant = None
    try:
        children.start("parent", command, cwd=tmp_path, log=tmp_path / "parent.log")
        deadline = time.monotonic() + 5
        while not pid_file.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert pid_file.exists()
        descendant = identity(int(pid_file.read_text()))
        children.close(["parent"], timeout=2)
        assert owned_process(descendant) is None
    finally:
        children.close(["parent"], timeout=2)
        terminate(descendant, timeout=2)


def test_scene_stop_after_launcher_crash_recovers_owned_children(tmp_path):
    (tmp_path / "run").mkdir()
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        state = {"owner_os": "linux", "phase": "running", "children": {"simulation": identity(child.pid)},
                 "launcher_identity": {**identity(os.getpid()), "created": 0}}
        (tmp_path / "run/scene_runtime.json").write_text(json.dumps(state))
        linux_scene.stop_scene(tmp_path)
        child.wait(timeout=3)
        assert json.loads((tmp_path / "run/scene_runtime.json").read_text())["phase"] == "stopped"
    finally:
        if child.poll() is None:
            child.terminate()
        child.wait()


def test_stop_is_idempotent_without_deployment(tmp_path, monkeypatch):
    monkeypatch.setattr(deployment, "ROOT", tmp_path)
    monkeypatch.setattr(deployment, "STATE", tmp_path / "run/linux_platform.json")
    deployment.stop()
    deployment.stop()


def test_scene_manager_launches_native_python_and_stops_before_state_handoff(tmp_path, monkeypatch):
    launch = SceneLaunchConfig(
        project_root=tmp_path, adapter_root=tmp_path,
        model_root=Path(__file__).resolve().parents[1] / "model/SARM/platform",
        unreal_root=tmp_path, powershell_exe=None, python_exe=Path(sys.executable),
        control_port=1, capture_port=2, render_port=3, pixel_streamer_port=4,
        pixel_streaming_id="test", pixel_streaming_camera_ids=(),
        pixel_streaming_camera_width=64, pixel_streaming_camera_height=64,
        preview_rate=30, renderer_ready_timeout=30, ik_rate=100,
        simulation_rate=1, capture_rate=50, default_dataset_capture=False)
    manager = SceneRuntimeManager(launch)
    process = Mock(pid=123, returncode=None)
    process.poll.return_value = None
    popen = Mock(return_value=process)
    stop = Mock()
    monkeypatch.setattr("space_arm_platform.scene_runtime.subprocess.Popen", popen)
    monkeypatch.setattr("space_arm_platform.scene_runtime.subprocess.run", stop)
    manager.start(SceneInstanceCreate(template_id="spacecraft-arm-teleop", randomization_profile="none"))
    command = popen.call_args.args[0]
    assert command[:4] == [sys.executable, "-m", "space_arm_platform.linux_scene", "start"]
    assert popen.call_args.kwargs["start_new_session"] is True
    assert "powershell" not in " ".join(command)
    manager.stop()
    process.terminate.assert_called_once()
    assert stop.call_args.args[0][:4] == [sys.executable, "-m", "space_arm_platform.linux_scene", "stop"]


def test_ports_in_use_are_not_claimed():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        with pytest.raises(OSError):
            deployment.check_port_available(listener.getsockname()[1])


def test_public_auto_selects_tunnel_or_fixed_ip(tmp_path):
    path = tmp_path / "config.json"
    assert linux_public.mode(deployment.settings(path)) == "public"
    path.write_text(json.dumps({"public_ip": "117.50.205.199"}))
    config = deployment.settings(path)
    assert linux_public.mode(config) == "ip"
    assert config["public_url"] == "https://117.50.205.199"


@pytest.mark.parametrize("changes", [
    {"public_url": "http://example.com"}, {"public_url": "https://user:password@example.com"},
    {"public_url": "https://example.com/path"}, {"public_ip": "10.0.0.1"},
    {"mode": "fixed"}, {"mode": "ip"}, {"host": "0.0.0.0"},
    {"managed_turn": True}, {"turn_relay_min_port": 50000, "turn_relay_max_port": 40000},
    {"turn_urls": ["https://example.com"]}, {"require_access_key": "false"},
    {"gateway_port": 18000},
])
def test_invalid_public_config_rejected(tmp_path, changes):
    path = tmp_path / "config.json"
    path.write_text(json.dumps(changes))
    with pytest.raises(ValueError):
        deployment.settings(path)


def test_closed_gateway_and_fixed_ip_tls(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"public_ip": "117.50.205.199"}))
    config = deployment.settings(path)
    holding = linux_public.caddy_config(tmp_path, config, "nonce")
    assert "503" in holding and "reverse_proxy" not in holding
    active = linux_public.caddy_config(tmp_path, config, "nonce", config["public_url"])
    assert "profile shortlived" in active and "default_sni 117.50.205.199" in active
    assert "@signalling path /stream" in active
    assert "X-Space-Sim-Deployment nonce" in active
    assert "gamepad=(self)" in active


def test_turn_requires_secret_and_rejects_world_readable_file(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"turn_urls": ["turn:example.com:3478"]}))
    config = deployment.settings(path)
    with pytest.raises(ValueError, match="require"):
        linux_public.turn_settings(tmp_path, config, {})
    key = tmp_path / "turn.key"
    key.write_text("unit-test-secret")
    key.chmod(0o644)
    config["turn_secret_file"] = str(key)
    with pytest.raises(ValueError, match="600"):
        linux_public.turn_settings(tmp_path, config, {})
    key.chmod(0o600)
    assert linux_public.turn_settings(tmp_path, config, {}) == (config["turn_urls"], "unit-test-secret")


def test_managed_turn_secret_not_on_command_line(tmp_path, monkeypatch):
    (tmp_path / "run").mkdir()
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"public_ip": "117.50.205.199", "managed_turn": True}))
    config = deployment.settings(path)
    children = Mock()
    monkeypatch.setattr(linux_public.shutil, "which", lambda value: "/usr/bin/turnserver")
    secret = {"turn_secret": "private-unit-test-key"}
    linux_public.start_turn(tmp_path, config, secret, children)
    assert secret["turn_secret"] not in str(children.start.call_args)
    turn_file = tmp_path / "run/linux-turn.conf"
    assert turn_file.stat().st_mode & 0o777 == 0o600
    assert "static-auth-secret=" + secret["turn_secret"] in turn_file.read_text()
    assert linux_public.turn_settings(tmp_path, config, secret)[0] == [
        "turn:117.50.205.199:3478?transport=udp", "turn:117.50.205.199:3478?transport=tcp"]


def test_gateway_probe_does_not_accept_other_deployment(monkeypatch):
    response = Mock(status=200, headers={"X-Space-Sim-Deployment": "another-deployment"})
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(linux_public, "urlopen", Mock(return_value=response))
    monkeypatch.setattr(linux_public.time, "sleep", lambda _: None)
    clock = iter([0, 0, 5])
    monkeypatch.setattr(linux_public.time, "monotonic", lambda: next(clock))
    with pytest.raises(TimeoutError, match="HTTPS probe failed"):
        linux_public.verify_gateway("https://example.com", "our-deployment", Mock(), lambda: False, timeout=1)


def test_websocket_connection_log_redacts_query_secrets():
    record = logging.LogRecord("uvicorn.error", logging.INFO, __file__, 1,
                               'WebSocket %s [%s]',
                               ('/ws/operator?access_key=private-key&token=private-jwt&page=operate', 'accepted'), None)
    assert RedactWebSocketSecrets().filter(record)
    message = record.getMessage()
    assert 'private-key' not in message and 'private-jwt' not in message
    assert 'page=operate' in message and 'REDACTED' in message


def test_deployment_stop_recovers_public_children(tmp_path, monkeypatch):
    monkeypatch.setattr(deployment, "ROOT", tmp_path)
    records = {name: {"name": name} for name in ('gateway', 'tunnel', 'backend', 'signalling', 'turn')}
    monkeypatch.setattr(deployment, "load_state", lambda: {"children": records})
    terminated = Mock()
    monkeypatch.setattr(deployment, "terminate", terminated)
    monkeypatch.setattr(linux_scene, "stop_scene", Mock())
    deployment.stop()
    assert [call.args[0] for call in terminated.call_args_list] == [
        None, records['tunnel'], records['gateway'], records['backend'], records['signalling'], records['turn']]


def test_tunnel_children_do_not_inherit_application_secrets(monkeypatch):
    for name in ('SPACE_SIM_ADMIN_PASSWORD', 'PS_TURN_AUTH_SECRET', 'TUNNEL_TOKEN', 'CF_API_TOKEN'):
        monkeypatch.setenv(name, 'private-test-value')
    env = linux_public.tool_environment()
    assert 'private-test-value' not in env.values()
