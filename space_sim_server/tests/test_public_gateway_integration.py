"""Real Caddy on ephemeral LOOPBACK ports, with dummy HTTP/WebSocket upstreams.

No cloudflared process, public registration, real auth database, or robot endpoint.
"""
from __future__ import annotations

import base64
import hashlib
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or r"C:\Program Files\PowerShell\7\pwsh.exe"
CADDY = os.environ.get("SPACE_SIM_TEST_CADDY") or str(ROOT / "run/deployment-tools/caddy-2.11.4-amd64/caddy.exe")
pytestmark = pytest.mark.skipif(not Path(CADDY).is_file() or not Path(PWSH).is_file(), reason="Optional Caddy/PowerShell 7 binaries unavailable")
HOST = "isolated-proxy-test.trycloudflare.com"
ORIGIN = f"https://{HOST}"
NONCE = "ISOLATEDTESTNONCE"


def child_options():
    return dict(creationflags=subprocess.CREATE_NO_WINDOW) if os.name == "nt" else {}


def free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def upstream(route, requests):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_):
            pass

        def do_GET(self):
            record = {"route": route, "path": self.path, "origin": self.headers.get("Origin"),
                      "forwarded_for": self.headers.get("X-Forwarded-For"),
                      "forwarded_proto": self.headers.get("X-Forwarded-Proto")}
            requests.append(record)
            if self.headers.get("Upgrade", "").lower() == "websocket":
                accept = base64.b64encode(hashlib.sha1((self.headers["Sec-WebSocket-Key"] +
                    "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
                self.send_response(101)
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                payload = json.dumps({"route": route}).encode()
                assert len(payload) < 126
                self.wfile.write(bytes((0x81, len(payload))) + payload)
                self.wfile.flush()
                self.close_connection = True
                return
            body = json.dumps({"ok": True, **record}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def request(port, path="/api/health", *, host=HOST):
    connection = HTTPConnection("127.0.0.1", port, timeout=3)
    try:
        connection.request("GET", path, headers={"Host": host, "Origin": ORIGIN,
            "CF-Connecting-IP": "198.51.100.77", "X-Forwarded-For": "203.0.113.99",
            "X-Forwarded-Proto": "http"})
        response = connection.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally:
        connection.close()


def receive_exact(stream, count):
    data = b""
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            raise AssertionError("WebSocket closed before the dummy frame was received")
        data += chunk
    return data


def websocket_route(port, path):
    key = base64.b64encode(b"isolated-test-123").decode()
    with socket.create_connection(("127.0.0.1", port), timeout=3) as connection:
        connection.sendall((f"GET {path} HTTP/1.1\r\nHost: {HOST}\r\nOrigin: {ORIGIN}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
            f"Sec-WebSocket-Key: {key}\r\n\r\n").encode())
        with connection.makefile("rb") as stream:
            assert b" 101 " in stream.readline()
            while stream.readline() != b"\r\n":
                pass
            opcode, length = receive_exact(stream, 2)
            assert opcode == 0x81 and length < 126
            return json.loads(receive_exact(stream, length))["route"]


def test_real_caddy_closed_gate_reload_http_headers_and_both_websocket_routes(tmp_path):
    api_requests, player_requests = [], []
    api, api_thread = upstream("api", api_requests)
    player, player_thread = upstream("signalling", player_requests)
    proxy_port, admin_port = free_port(), free_port()
    while proxy_port == admin_port:
        admin_port = free_port()
    caddy_process = None
    stdout = (tmp_path / "proxy.out.log").open("wb")
    stderr = (tmp_path / "proxy.err.log").open("wb")
    env = {key: value for key, value in os.environ.items() if not key.startswith(("SPACE_SIM_", "TUNNEL_"))}
    try:
        helper = tmp_path / "render.ps1"
        helper.write_text("""
param($Helpers,$Directory,[int]$Api,[int]$Player,[int]$Proxy,[int]$Admin)
$ErrorActionPreference='Stop'
. $Helpers
$settings=@{PublicUrl='https://isolated-proxy-test.trycloudflare.com';Ports=@{api_port=$Api;player_port=$Player}}
Get-PublicProxyConfig $settings $Proxy $Admin 'ISOLATEDTESTNONCE' -Holding | Set-Content -LiteralPath (Join-Path $Directory 'holding.Caddyfile') -Encoding utf8NoBOM
Get-PublicProxyConfig $settings $Proxy $Admin 'ISOLATEDTESTNONCE' | Set-Content -LiteralPath (Join-Path $Directory 'active.Caddyfile') -Encoding utf8NoBOM
""", encoding="utf-8")
        render = subprocess.run([PWSH, "-NoProfile", "-File", str(helper), str(ROOT / "scripts/public_deployment_helpers.ps1"),
            str(tmp_path), str(api.server_port), str(player.server_port), str(proxy_port), str(admin_port)],
            capture_output=True, text=True, timeout=15, env=env, **child_options())
        assert render.returncode == 0, render.stdout + render.stderr
        caddy_process = subprocess.Popen([CADDY, "run", "--config", str(tmp_path / "holding.Caddyfile"), "--adapter", "caddyfile"],
            stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, cwd=tmp_path, env=env, **child_options())
        deadline = time.monotonic() + 10
        while True:
            assert caddy_process.poll() is None, (tmp_path / "proxy.err.log").read_text(errors="replace")
            try:
                status, headers, body = request(proxy_port)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.1)
        assert status == 503
        assert headers["X-Space-Sim-Preparing"] == NONCE
        assert not api_requests and not player_requests
        reload = subprocess.run([CADDY, "reload", "--config", str(tmp_path / "active.Caddyfile"), "--adapter", "caddyfile",
            "--address", f"127.0.0.1:{admin_port}"], capture_output=True, text=True, timeout=10, env=env, **child_options())
        assert reload.returncode == 0, reload.stdout + reload.stderr
        status, headers, body = request(proxy_port)
        assert status == 200 and headers["X-Space-Sim-Deployment"] == NONCE
        assert headers["Permissions-Policy"] == "gamepad=(self)"
        response = json.loads(body)
        assert response["origin"] == ORIGIN
        assert response["forwarded_proto"] == "https"
        assert response["forwarded_for"] == "198.51.100.77", response
        count = len(api_requests)
        assert request(proxy_port, host="wrong.example")[0] == 421
        assert len(api_requests) == count
        assert websocket_route(proxy_port, "/stream") == "signalling"
        assert websocket_route(proxy_port, "/ws/operator") == "api"
        assert player_requests[-1]["path"] == "/stream"
        assert api_requests[-1]["path"] == "/ws/operator"
    finally:
        if caddy_process is not None and caddy_process.poll() is None:
            caddy_process.terminate()
            try:
                caddy_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                caddy_process.kill()
                caddy_process.wait(timeout=5)
        stdout.close()
        stderr.close()
        for server, thread in ((api, api_thread), (player, player_thread)):
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)
