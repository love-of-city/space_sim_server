"""One-command Linux deployment using the production backend and signalling service."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen
from urllib.parse import urlencode

from .collection_processes import ManagerLease
from .collection_runtime import check_port_available
from .linux_processes import Children, identity, owned_process, terminate, write_json
from . import linux_public as public
from .unreal_runtime import unreal_editor

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "deploy/linux.local.json"
STATE = ROOT / "run/linux_platform.json"


def settings(path: Path):
    local = json.loads(path.read_text()) if path.is_file() else {}
    config = {
        "python": sys.executable, "adapter": str(ROOT.parent / "space_sim_UE_Adapter"),
        "model_root": str(ROOT / "model/SARM/platform"),
        "unreal": os.environ.get("UE56_ROOT", str(Path.home() / "engines/UE_5.6")),
        "host": "127.0.0.1", "api_port": 18000, "player_port": 18080,
        "streamer_port": 18888, "control_port": 18766, "capture_port": 18767, "render_port": 18558,
        "preview_rate": 60, "encoder_min_quality": 60, "renderer_ready_timeout": 600,
        "node": "node", "npm": "npm", "admin_username": "admin", "allowed_origins": [],
        "ice_servers": [], "signalling_url": "", "data_root": str(ROOT / "data/episodes"),
        "cache_root": os.environ.get("XDG_CACHE_HOME", str(ROOT / "run/cache")),
        "mode": "auto", "public_url": "", "public_ip": "", "tls_mode": "acme",
        "certificate_file": "", "certificate_key_file": "",
        "gateway_port": 18443, "gateway_admin_port": 18444,
        "caddy_executable": "", "cloudflared_executable": "", "require_access_key": False,
        "turn_urls": [], "turn_secret_file": "", "managed_turn": False,
        "turnserver": "turnserver", "turn_port": 3478,
        "turn_relay_min_port": 49160, "turn_relay_max_port": 49259,
    }
    unknown = set(local) - set(config)
    if unknown:
        raise ValueError(f"Unknown deployment settings: {sorted(unknown)}")
    config.update(local)
    for key in ("python", "adapter", "model_root", "unreal", "data_root", "cache_root"):
        # Resolving a venv's python symlink would silently select the base interpreter.
        config[key] = str(Path(config[key]).expanduser().absolute())
    ports = [config[key] for key in ("api_port", "player_port", "streamer_port", "control_port", "capture_port", "render_port", "gateway_port", "gateway_admin_port", "turn_port")]
    if any(type(port) is not int or not 1 <= port <= 65535 for port in ports) or len(set(ports)) != len(ports):
        raise ValueError("Ports must be distinct integers in [1, 65535]")
    if not 1 <= config["preview_rate"] <= 120 or not 0 <= config["encoder_min_quality"] <= 100:
        raise ValueError("Invalid preview rate or encoder quality")
    for key in ("allowed_origins", "ice_servers"):
        if not isinstance(config[key], list):
            raise ValueError(f"{key} must be an array")
    public.validate(config)
    return config


def environment(config):
    env = os.environ.copy()
    # The deployment setting owns this optional gate; do not inherit a stale key
    # from a shell when account-only authentication is selected.
    env.pop("SPACE_SIM_STREAM_ACCESS_KEY", None)
    env["PATH"] = str(Path(config["node"]).parent) + os.pathsep + env.get("PATH", "")
    env["SPACE_SIM_PYTHON"] = config["python"]
    env["PYTHONPATH"] = str(ROOT / "backend") + os.pathsep + str(ROOT)
    env["PYTHONIOENCODING"] = "utf-8"
    env["XDG_CACHE_HOME"] = config["cache_root"]
    env["UE56_ROOT"] = config["unreal"]
    engine = Path(config["unreal"]) / "Engine"
    dotnet = engine / "Binaries/ThirdParty/DotNet/8.0.300/linux-x64"
    env["UE_DOTNET_DIR"] = env["DOTNET_ROOT"] = str(dotnet)
    env["DOTNET_CLI_HOME"] = str(ROOT / "run/dotnet")
    env["DOTNET_SYSTEM_GLOBALIZATION_APPLOCALICU"] = ":64.1"
    env["DOTNET_gcServer"] = "0"
    libraries = [engine / "Binaries/ThirdParty/ICU/icu4c-64_1/lib/Unix/x86_64-unknown-linux-gnu",
                 engine / "Binaries/ThirdParty/OpenSSL/Unix/lib/x86_64-unknown-linux-gnu"]
    env["LD_LIBRARY_PATH"] = os.pathsep.join([*(str(path) for path in libraries), env.get("LD_LIBRARY_PATH", "")])
    # Shared installations stay read-only; user caches belong to this deployment.
    env["XDG_CONFIG_HOME"] = str(ROOT / "run/linux-config")
    env["UE-LocalDataCachePath"] = str(Path(config["adapter"]) / "Unreal/BskUnrealRenderer/Saved/DerivedDataCache")
    return env


def preflight(config, prepare=False):
    if not sys.platform.startswith("linux"):
        raise RuntimeError("Use this launcher on Linux; Windows retains its existing deployment scripts")
    unreal_editor(Path(config["unreal"]))
    for key in ("python", "node", "npm"):
        executable = shutil.which(config[key])
        if executable is None:
            raise FileNotFoundError(f"{key} executable missing: {config[key]}")
        config[key] = executable
    project = Path(config["adapter"]) / "Unreal/BskUnrealRenderer"
    for path in (project / "BskUnrealRenderer.uproject", Path(config["model_root"]) / "sarm_20261012.xml"):
        if not path.is_file():
            raise FileNotFoundError(path)
    env = environment(config)
    subprocess.run([config["python"], str(ROOT / "scripts/check_basilisk.py")], env=env, cwd=ROOT, check=True)
    if public.mode(config) != "local" or config["host"] not in ("127.0.0.1", "localhost", "::1"):
        subprocess.run([config["python"], str(ROOT / "tools/check_deployment_auth.py"),
                        "--database", str(ROOT / "data/auth.sqlite3")], env=env, cwd=ROOT, check=True)
    if public.mode(config) != "local":
        caddy = public.resolve_tool(ROOT, "caddy", config["caddy_executable"], download=prepare)
        if public.mode(config) in ("ip", "fixed"):
            public.check_https_permission(caddy)
        if public.mode(config) == "public":
            public.resolve_tool(ROOT, "cloudflared", config["cloudflared_executable"], download=prepare)
        if config["managed_turn"] and not shutil.which(config["turnserver"]):
            raise FileNotFoundError("Managed TURN requires coturn; set turnserver to its executable")
        if config["turn_urls"]:
            public.turn_settings(ROOT, config, {})
    if prepare:
        for component in ("frontend", "signalling"):
            if not (ROOT / component / "node_modules").is_dir():
                subprocess.run([config["npm"], "ci", "--no-audit", "--no-fund"], cwd=ROOT / component, env=env, check=True)
        subprocess.run([config["npm"], "run", "build"], cwd=ROOT / "frontend", env=env, check=True)
        subprocess.run([config["npm"], "run", "check"], cwd=ROOT / "signalling", env=env, check=True)
        for tool, extra in (("build_native_acceleration.py", ["--component", "all"]), ("build_sarm_plugin.py", [])):
            subprocess.run([config["python"], str(ROOT / "tools" / tool), *extra], cwd=ROOT, env=env, check=True)
        binaries = [project / "Binaries/Linux/libUnrealEditor-BskUnrealRenderer.so",
                    project / "Plugins/BskUnrealRuntime/Binaries/Linux/libUnrealEditor-BskUnrealRuntime.so"]
        if not all(path.is_file() for path in binaries):
            subprocess.run([config["python"], str(ROOT / "tools/build_unreal_linux.py"),
                            "--adapter", config["adapter"], "--unreal", config["unreal"]], cwd=ROOT, env=env, check=True)
    return env


def credentials():
    path = ROOT / "deploy/linux-secrets.local.json"
    if not path.exists():
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            json.dump({"admin_password": secrets.token_urlsafe(20), "jwt_secret": secrets.token_urlsafe(48)}, stream)
    os.chmod(path, 0o600)
    secret = json.loads(path.read_text())
    changed = False
    for key in ("access_key", "turn_secret"):
        if not secret.get(key):
            secret[key] = secrets.token_urlsafe(48)
            changed = True
    if changed:
        with path.open("w") as stream:
            json.dump(secret, stream)
    return secret


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def url(config):
    host = "127.0.0.1" if config["host"] in ("0.0.0.0", "::") else config["host"]
    return f"http://{host}:{config['api_port']}"


def supervise(config_path, mode_override=None):
    config = settings(config_path)
    if mode_override:
        config["mode"] = mode_override
        public.validate(config)
    env = environment(config)
    lease = ManagerLease(ROOT / "run/linux-platform.lock")
    children = Children()
    stopping = False
    failed = None
    def request_stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    state = {"owner_os": "linux", "supervisor": identity(os.getpid()), "config": config,
             "url": url(config), "credentials_file": str(ROOT / "deploy/linux-secrets.local.json")}
    def update(phase, error=""):
        state["config"] = {**config, "ice_servers": [{"urls": item.get("urls", [])} for item in config["ice_servers"]]}
        state.update(phase=phase, children=children.records, error=error, updated_at_ns=str(time.time_ns()))
        write_json(STATE, state)
    try:
        update("starting")
        for key in ("api_port", "player_port", "streamer_port", "control_port", "capture_port", "render_port"):
            check_port_available(config[key])
        secret = credentials()
        choice = public.mode(config)
        origin = ""
        nonce = secrets.token_hex(16)
        if choice != "local":
            for key in ("gateway_port", "gateway_admin_port"):
                check_port_available(config[key])
            caddy = public.resolve_tool(ROOT, "caddy", config["caddy_executable"])
            gateway_path = ROOT / "run/linux-Caddyfile"
            gateway_path.write_text(public.caddy_config(ROOT, config, nonce))
            children.start("gateway", [caddy, "run", "--config", str(gateway_path), "--adapter", "caddyfile"],
                           cwd=ROOT, env=public.tool_environment(), log=ROOT / "logs/linux-gateway.log")
            update("starting")
            if choice == "public":
                cloudflared = public.resolve_tool(ROOT, "cloudflared", config["cloudflared_executable"])
                tunnel_config = ROOT / "run/linux-cloudflared.yml"
                tunnel_config.write_text("# Empty: do not inherit another Cloudflare tunnel\n")
                tunnel_log = ROOT / f"logs/linux-tunnel-{nonce}.log"
                children.start("tunnel", [cloudflared, "tunnel", "--config", str(tunnel_config),
                    "--no-autoupdate", "--protocol", "http2", "--url", f"http://127.0.0.1:{config['gateway_port']}",
                    "--metrics", "127.0.0.1:0"], cwd=ROOT, env=public.tool_environment(), log=tunnel_log)
                update("starting")
                origin = public.wait_tunnel(tunnel_log, children, lambda: stopping)
            else:
                origin = config["public_url"]
            config["allowed_origins"] = [origin]
            config["signalling_url"] = origin.replace("https:", "wss:") + "/stream"
            if not config["ice_servers"]:
                config["ice_servers"] = [{"urls": "stun:stun.cloudflare.com:3478"}]
            if config["managed_turn"]:
                check_port_available(config["turn_port"])
                public.start_turn(ROOT, config, secret, children)
                update("starting")
            env.update(SPACE_SIM_SECURE_COOKIES="1", SPACE_SIM_NO_ACCESS_LOG="1",
                       SPACE_SIM_FORWARDED_ALLOW_IPS="127.0.0.1,::1")
            if config["require_access_key"]:
                env["SPACE_SIM_STREAM_ACCESS_KEY"] = secret["access_key"]
            state["public_url"] = origin
            state["mode"] = choice
        turn_urls, turn_secret = public.turn_settings(ROOT, config, secret)
        env.update(SPACE_SIM_ADMIN_USERNAME=config["admin_username"],
                   SPACE_SIM_ADMIN_PASSWORD=secret["admin_password"],
                   SPACE_SIM_STREAM_JWT_SECRET=secret["jwt_secret"])
        signal_env = {**env, "PS_PLAYER_PORT": str(config["player_port"]), "PS_PLAYER_HOST": config["host"],
                      "PS_STREAMER_PORT": str(config["streamer_port"]), "PS_STREAMER_HOST": "127.0.0.1",
                      "PS_JWT_SECRET": secret["jwt_secret"], "PS_ALLOWED_ORIGINS": json.dumps(config["allowed_origins"]),
                      "PS_ICE_SERVERS_JSON": json.dumps(config["ice_servers"])}
        signal_env.update(PS_TURN_URLS_JSON=json.dumps(turn_urls), PS_TURN_AUTH_SECRET=turn_secret)
        children.start("signalling", [config["node"], "server.mjs"], cwd=ROOT / "signalling", env=signal_env,
                       log=ROOT / "logs/linux-signalling.log")
        cameras = [("teleop/camera/spacecraft_overview", "SpacecraftOverview"), ("teleop/camera/sarm_wrist_cam", "WristCamera")]
        command = [config["python"], "-m", "space_arm_platform.main", "--host", config["host"],
                   "--port", str(config["api_port"]), "--simulation-port", str(config["control_port"]),
                   "--capture-port", str(config["capture_port"]), "--pixel-streaming-player-port", str(config["player_port"]),
                   "--data-root", config["data_root"], "--runtime-adapter-root", config["adapter"],
                   "--runtime-model-root", config["model_root"], "--runtime-unreal-root", config["unreal"],
                   "--runtime-render-port", str(config["render_port"]),
                   "--runtime-pixel-streamer-port", str(config["streamer_port"]),
                   "--runtime-preview-rate", str(config["preview_rate"]),
                   "--runtime-encoder-min-quality", str(config["encoder_min_quality"]),
                   "--runtime-renderer-ready-timeout", str(config["renderer_ready_timeout"])]
        if config["signalling_url"]:
            command += ["--pixel-streaming-signalling-url", config["signalling_url"]]
        for camera, label in cameras:
            streamer = "BskRenderer__" + camera.replace("/", "_")
            command += ["--pixel-streaming-camera-streamer", streamer + "=" + label,
                        "--runtime-pixel-streaming-camera-id", camera]
        for origin in config["allowed_origins"]:
            command += ["--allowed-origin", origin]
        children.start("backend", command, cwd=ROOT, env=env, log=ROOT / "logs/linux-backend.log")
        update("starting")
        deadline = time.monotonic() + 120
        while True:
            children.check()
            if stopping:
                raise InterruptedError("startup stopped")
            try:
                with urlopen(url(config) + "/api/health", timeout=1) as response:
                    ready = response.status == 200
                with socket.create_connection(("127.0.0.1", config["player_port"]), timeout=1):
                    pass
                if ready:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("backend/signalling startup timed out; inspect logs/linux-*.log")
            time.sleep(.3)
        if origin:
            public.activate_gateway(ROOT, config, nonce, origin, caddy)
            public.verify_gateway(origin, nonce, children, lambda: stopping)
            state["url"] = origin
            state["public_verified"] = True
            state["turn_configured"] = bool(turn_urls or any(
                "turn" in json.dumps(server.get("urls", "")) for server in config["ice_servers"]))
            write_json(ROOT / "run/linux-public-access.json", {
                "public_url": origin, "public_verified": True, "mode": choice,
                "username": config["admin_username"], "require_access_key": config["require_access_key"],
                "turn_configured": state["turn_configured"], "credentials_file": state["credentials_file"]})
        update("running")
        while not stopping:
            children.check()
            time.sleep(.5)
    except InterruptedError:
        pass
    except BaseException as error:
        failed = error
    finally:
        update("stopping", str(failed) if failed else "")
        try:
            # Backend first: existing lifespan finalizes recording, then closes scenes.
            children.close(("tunnel", "gateway", "backend", "signalling", "turn"), timeout=720)
            from .linux_scene import stop_scene
            stop_scene(ROOT)
        except BaseException as error:
            failed = error
        update("failed" if failed else "stopped", str(failed) if failed else "")
        (ROOT / "run/linux-public-access.json").unlink(missing_ok=True)
        (ROOT / "run/linux-turn.conf").unlink(missing_ok=True)
        lease.close()
    if failed:
        raise RuntimeError(str(failed))


def start(config_path, check=False, mode_override=None, restart=False):
    if restart:
        stop()
    (ROOT / "run").mkdir(exist_ok=True)
    (ROOT / "logs").mkdir(exist_ok=True)
    lease = ManagerLease(ROOT / "run/linux-platform-command.lock")
    try:
        config = settings(config_path)
        if mode_override:
            config["mode"] = mode_override
            public.validate(config)
        previous = load_state()
        if not check and owned_process(previous.get("supervisor")):
            if previous.get("phase") != "running":
                raise RuntimeError(f"Platform is {previous.get('phase')}; inspect logs or stop it first")
            old_config = previous.get("config", {})
            old_mode = previous.get("mode", old_config.get("mode", "local"))
            if old_mode != public.mode(config) or (public.mode(config) in ("ip", "fixed") and config["public_url"] != previous.get("public_url")):
                raise RuntimeError("Deployment mode/URL changed; use --restart to apply it")
            print("Already running: " + previous["url"])
            return
        if not check and any(owned_process(record) for record in previous.get("children", {}).values()):
            raise RuntimeError("Recorded platform children are still running; run stop_deployment.sh first")
        env = preflight(config, prepare=not check)
        if check:
            print("Linux dependencies and configuration passed. No services started.")
            return
        credentials()
        with (ROOT / "logs/linux-platform.log").open("ab") as log:
            command = [config["python"], "-m", "space_arm_platform.linux_deployment",
                       "supervise", "--config", str(config_path)]
            if mode_override:
                command += ["--mode", mode_override]
            process = subprocess.Popen(command, cwd=ROOT, env=env,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 450
        try:
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("Platform startup failed; see logs/linux-platform.log")
                state = load_state()
                if state.get("supervisor", {}).get("pid") == process.pid and state.get("phase") == "running":
                    print("Platform ready: " + state["url"])
                    print("Login: " + config["admin_username"] + "; first-run password: deploy/linux-secrets.local.json (admin_password)")
                    print("An existing account database keeps its existing password.")
                    print("Stop: ./stop_deployment.sh")
                    if state.get("public_verified"):
                        print("HTTPS verified. Run ./show_deployment_access.sh for the access URL and login details.")
                        if not state.get("turn_configured"):
                            print("TURN is not configured; media connectivity still depends on client/server NAT.")
                    return
                time.sleep(.3)
            raise TimeoutError("Platform startup timed out")
        except BaseException:
            if process.poll() is None:
                terminate(identity(process.pid), timeout=780)
                process.wait(timeout=5)
            raise
    finally:
        lease.close()


def stop():
    (ROOT / "run").mkdir(exist_ok=True)
    lease = ManagerLease(ROOT / "run/linux-platform-command.lock")
    try:
        state = load_state()
        terminate(state.get("supervisor"), 780)
        # Also recover from a supervisor crash using exact UID/start-time records.
        for name in ("tunnel", "gateway", "backend", "signalling", "turn"):
            terminate(state.get("children", {}).get(name), 720)
        from .linux_scene import stop_scene
        stop_scene(ROOT)
        (ROOT / "run/linux-public-access.json").unlink(missing_ok=True)
        (ROOT / "run/linux-turn.conf").unlink(missing_ok=True)
        print("Linux platform and its scene have stopped.")
    finally:
        lease.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status", "access", "supervise"))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--mode", choices=("auto", "local", "public", "ip", "fixed"))
    parser.add_argument("--restart", action="store_true")
    args = parser.parse_args()
    if args.action == "start":
        if args.check and args.restart:
            parser.error("--check cannot be combined with --restart")
        start(args.config.resolve(), args.check, args.mode, args.restart)
    elif args.action == "stop":
        stop()
    elif args.action == "status":
        state = load_state()
        print(json.dumps({**state, "supervisor_alive": owned_process(state.get("supervisor")) is not None}, indent=2))
    elif args.action == "access":
        state = load_state()
        if not owned_process(state.get("supervisor")) or state.get("phase") != "running":
            parser.error("Platform is not running")
        secret = credentials()
        address = state["url"]
        if state.get("public_verified") and state["config"]["require_access_key"]:
            address += "?" + urlencode({"access_key": secret["access_key"]})
        print("Access URL: " + address)
        print("Username: " + state["config"]["admin_username"])
        print("Initial password: " + secret["admin_password"])
        print("An existing account keeps its current password. Keep this output private.")
    else:
        supervise(args.config.resolve(), args.mode)


if __name__ == "__main__":
    main()
