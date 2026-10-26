"""Linux HTTPS gateway, pinned deployment tools and TURN configuration."""
from __future__ import annotations

import hashlib
import ipaddress
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import tarfile
import tempfile
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


def public_origin(value):
    parsed = urlsplit(value)
    if (parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password
            or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
        raise ValueError("public_url must be an HTTPS origin without credentials, path or query")
    host = parsed.hostname
    if not re.fullmatch(r"[a-zA-Z0-9.-]+", host) or ".." in host:
        raise ValueError("Invalid public hostname")
    if parsed.port not in (None, 443):
        raise ValueError("Public HTTPS uses port 443")
    return f"https://{host}"


def mode(config):
    choice = config["mode"]
    if choice == "auto":
        if config["public_url"]:
            return "ip" if _is_ip(urlsplit(config["public_url"]).hostname) else "fixed"
        if config["public_ip"]:
            return "ip"
        return "public"
    return choice


def _is_ip(value):
    try:
        return ipaddress.ip_address(value).version == 4
    except ValueError:
        return False


def validate(config):
    if config["mode"] not in ("auto", "local", "public", "ip", "fixed"):
        raise ValueError("mode must be auto, local, public, ip or fixed")
    if config["public_ip"] and (not _is_ip(config["public_ip"])
                                or not ipaddress.ip_address(config["public_ip"]).is_global):
        raise ValueError("public_ip must be a public IPv4 address")
    if config["public_url"]:
        config["public_url"] = public_origin(config["public_url"])
    choice = mode(config)
    if choice == "ip":
        config["public_url"] = config["public_url"] or f"https://{config['public_ip']}"
        host = urlsplit(public_origin(config["public_url"])).hostname
        if not _is_ip(host) or not ipaddress.ip_address(host).is_global:
            raise ValueError("IP mode requires a public IPv4 HTTPS URL or public_ip")
    if choice == "fixed" and (not config["public_url"] or _is_ip(urlsplit(config["public_url"]).hostname)):
        raise ValueError("fixed mode requires a public DNS HTTPS URL")
    if choice != "local" and config["host"] != "127.0.0.1":
        raise ValueError("Public deployment keeps backend/signalling on 127.0.0.1 behind HTTPS")
    if config["tls_mode"] not in ("acme", "manual"):
        raise ValueError("tls_mode must be acme or manual")
    if config["tls_mode"] == "manual" and choice in ("ip", "fixed"):
        for key in ("certificate_file", "certificate_key_file"):
            if not Path(config[key]).is_file():
                raise ValueError(f"Missing {key}")
    if config["managed_turn"] and not config["public_ip"]:
        raise ValueError("managed_turn requires public_ip (the public NAT address)")
    if not isinstance(config["turn_urls"], list) or any(
            not isinstance(value, str) or not re.fullmatch(r"turns?:[^\s]+", value)
            for value in config["turn_urls"]):
        raise ValueError("turn_urls must be an array of TURN URLs")
    for key in ("managed_turn", "require_access_key"):
        if type(config[key]) is not bool:
            raise ValueError(f"{key} must be boolean")
    if not 1024 <= config["turn_relay_min_port"] <= config["turn_relay_max_port"] <= 65535:
        raise ValueError("Invalid TURN relay port range")


def resolve_tool(root, name, override="", download=True):
    override = override or os.environ.get(f"SPACE_SIM_{name.upper()}_EXE", "")
    if override:
        path = shutil.which(override)
        if path is None:
            raise FileNotFoundError(f"{name} missing: {override}")
        return path
    machine = platform.machine().lower()
    arch = {"x86_64": "amd64", "aarch64": "arm64"}.get(machine)
    if arch is None:
        raise ValueError(f"Set SPACE_SIM_{name.upper()}_EXE for {machine}")
    manifest = json.loads((root / f"deploy/{name}-release.json").read_text())
    version = manifest["version"]
    algorithm = "sha512" if name == "caddy" else "sha256"
    checksum = manifest[f"linux_{algorithm}"][arch]
    folder = root / f"run/deployment-tools/{name}-{version}-linux-{arch}"
    binary = folder / name
    archive = folder / ("release.tar.gz" if name == "caddy" else name)
    valid = False
    if archive.is_file():
        with archive.open("rb") as stream:
            valid = hashlib.file_digest(stream, algorithm).hexdigest() == checksum
    if binary.is_file() and valid:
        if name != "caddy" or (folder / "binary.sha256").is_file() and hashlib.sha256(binary.read_bytes()).hexdigest() == (folder / "binary.sha256").read_text():
            if not os.access(binary, os.X_OK):
                binary.chmod(0o755)
            return str(binary)
    if not download and not valid:
        raise FileNotFoundError(f"Run start once to download pinned {name}, or configure its executable")
    folder.mkdir(parents=True, exist_ok=True)
    asset = f"caddy_{version}_linux_{arch}.tar.gz" if name == "caddy" else f"cloudflared-linux-{arch}"
    repository = "caddyserver/caddy" if name == "caddy" else "cloudflare/cloudflared"
    tag = "v" + version if name == "caddy" else version
    address = f"https://github.com/{repository}/releases/download/{tag}/{asset}"
    if not valid:
        fd, temporary = tempfile.mkstemp(dir=folder)
        try:
            with os.fdopen(fd, "wb") as output, urlopen(address, timeout=60) as response:
                shutil.copyfileobj(response, output)
            with open(temporary, "rb") as stream:
                if hashlib.file_digest(stream, algorithm).hexdigest() != checksum:
                    raise ValueError(f"{name} checksum mismatch")
            Path(temporary).replace(archive)
        finally:
            Path(temporary).unlink(missing_ok=True)
    if name == "caddy":
        with tarfile.open(archive) as bundle:
            member = bundle.getmember("caddy")
            if not member.isfile():
                raise ValueError("Invalid Caddy archive")
            with bundle.extractfile(member) as source, binary.open("wb") as output:
                shutil.copyfileobj(source, output)
        (folder / "binary.sha256").write_text(hashlib.sha256(binary.read_bytes()).hexdigest())
    binary.chmod(0o755)
    return str(binary)


def check_https_permission(caddy):
    if os.geteuid() == 0:
        return
    threshold = Path("/proc/sys/net/ipv4/ip_unprivileged_port_start")
    if threshold.is_file() and int(threshold.read_text()) <= 80:
        return
    try:
        capabilities = os.getxattr(caddy, "security.capability")
        if int.from_bytes(capabilities[4:8], "little") & (1 << 10):
            return
    except OSError:
        pass
    raise PermissionError(f"HTTPS needs low-port permission. Run once: sudo setcap cap_net_bind_service=+ep {caddy}")


def tool_environment():
    return {key: value for key, value in os.environ.items()
            if not key.startswith(("SPACE_SIM_", "PS_", "TUNNEL_", "CLOUDFLARE_", "CF_"))}


def caddy_config(root, config, nonce, origin=""):
    admin = config["gateway_admin_port"]
    global_options = f"admin 127.0.0.1:{admin}\n    storage file_system {json.dumps(str(root / 'deploy/certs/linux'))}"
    if not origin or mode(config) == "public":
        global_options += "\n    auto_https off"
        site = f"http://:{config['gateway_port']}"
    else:
        site = public_origin(origin)
        if mode(config) == "ip":
            global_options += f"\n    default_sni {urlsplit(origin).hostname}"
    if not origin:
        routes = 'respond "Deployment is preparing" 503'
    else:
        host = urlsplit(public_origin(origin)).hostname
        headers = "\n".join([
            f"X-Space-Sim-Deployment {nonce}", "Cache-Control no-store", "X-Content-Type-Options nosniff",
            "Referrer-Policy no-referrer", "X-Frame-Options DENY", 'Permissions-Policy "gamepad=(self)"'])
        upstream = "\n".join([
            "header_up X-Forwarded-Proto https",
            "header_up X-Forwarded-For {http.request.remote.host}" if mode(config) != "public"
            else "header_up X-Forwarded-For {http.request.header.CF-Connecting-IP}"])
        routes = f"""@public host {host}
    handle @public {{
        header {{
            {headers}
        }}
        @signalling path /stream
        handle @signalling {{
            reverse_proxy 127.0.0.1:{config['player_port']} {{
                {upstream}
            }}
        }}
        handle {{
            reverse_proxy 127.0.0.1:{config['api_port']} {{
                {upstream}
            }}
        }}
    }}
    respond "Unknown site" 421"""
        if mode(config) in ("ip", "fixed"):
            if config["tls_mode"] == "manual":
                routes = f"tls {json.dumps(config['certificate_file'])} {json.dumps(config['certificate_key_file'])}\n" + routes
            elif mode(config) == "ip":
                routes = "tls {\n issuer acme {\n dir https://acme-v02.api.letsencrypt.org/directory\n profile shortlived\n }\n}\n" + routes
    if not origin or mode(config) == "public":
        routes = "bind 127.0.0.1\n" + routes
    return f"{{\n    {global_options}\n}}\n{site} {{\n    {routes}\n}}\n"


def wait_tunnel(log, children, stopping, timeout=90):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        children.check()
        if stopping():
            raise InterruptedError("Public startup stopped")
        content = log.read_text(errors="replace") if log.exists() else ""
        match = re.search(r"https://[a-z0-9]+(?:-[a-z0-9]+)*\.trycloudflare\.com(?=[\s|/]|$)", content)
        if match:
            return match[0]
        time.sleep(.3)
    raise TimeoutError("Cloudflare did not allocate a public URL; inspect tunnel log")


def verify_gateway(origin, nonce, children, stopping, timeout=150):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        children.check()
        if stopping():
            raise InterruptedError("Public startup stopped")
        try:
            request = Request(origin + "/api/health?deployment_probe=" + nonce,
                              headers={"Cache-Control": "no-cache"})
            with urlopen(request, timeout=5) as response:
                if (response.status == 200 and response.headers.get("X-Space-Sim-Deployment") == nonce
                        and json.load(response).get("ok") is True):
                    return
        except (OSError, ValueError):
            pass
        time.sleep(1)
    raise TimeoutError("Public HTTPS probe failed; check IP mapping, ports 80/443, ACME and gateway log")


def turn_settings(root, config, secret):
    urls = config["turn_urls"]
    turn_secret = os.environ.get("SPACE_SIM_TURN_AUTH_SECRET", "")
    if config["turn_secret_file"]:
        path = Path(config["turn_secret_file"])
        if path.stat().st_mode & 0o077:
            raise ValueError("TURN secret file must have mode 600")
        turn_secret = path.read_text().strip()
    if config["managed_turn"]:
        turn_secret = secret["turn_secret"]
        address, port = config["public_ip"], config["turn_port"]
        urls = [f"turn:{address}:{port}?transport=udp", f"turn:{address}:{port}?transport=tcp"]
    if urls and not turn_secret:
        raise ValueError("TURN URLs require turn_secret_file or SPACE_SIM_TURN_AUTH_SECRET")
    return urls, turn_secret


def start_turn(root, config, secret, children):
    executable = shutil.which(config["turnserver"])
    if not executable:
        raise FileNotFoundError("Managed TURN needs coturn: install it or set turnserver")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((config["public_ip"], 3478))
        local = sock.getsockname()[0]
    address = config["public_ip"]
    external = address if address == local else f"{address}/{local}"
    path = root / "run/linux-turn.conf"
    content = f"""listening-port={config['turn_port']}
listening-ip={local}
relay-ip={local}
external-ip={external}
min-port={config['turn_relay_min_port']}
max-port={config['turn_relay_max_port']}
realm=space-sim
userdb={root / 'run/linux-turn.sqlite3'}
fingerprint
use-auth-secret
static-auth-secret={secret['turn_secret']}
no-cli
no-tls
no-dtls
no-multicast-peers
no-software-attribute
relay-threads=2
total-quota=32
user-quota=8
pidfile={root / 'run/linux-turn.pid'}
log-file=stdout
simple-log
"""
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.chmod(path, 0o600)
    with os.fdopen(descriptor, "w") as stream:
        stream.write(content)
    children.start("turn", [executable, "-c", str(path)], cwd=root,
                   env=tool_environment(), log=root / "logs/linux-turn.log")


def activate_gateway(root, config, nonce, origin, caddy):
    path = root / "run/linux-Caddyfile"
    path.write_text(caddy_config(root, config, nonce, origin))
    subprocess.run([caddy, "reload", "--config", str(path), "--adapter", "caddyfile",
                    "--address", f"127.0.0.1:{config['gateway_admin_port']}"],
                   cwd=root, env=tool_environment(), check=True)
