from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.auth import AuthStore, SESSION_COOKIE
from space_arm_platform.web_security import LoginRateLimiter, validate_origins

ORIGIN = "https://sim.test"
PASSWORD = "Deployment-Test-Password!"


def make_app(tmp_path, **changes):
    options = dict(
        project_root=Path(__file__).resolve().parents[1], data_root=tmp_path / "episodes",
        auth_database=tmp_path / "auth.sqlite3", simulation_port=0, capture_port=0,
        bootstrap_admin_password=PASSWORD, secure_cookies=True, allowed_origins=(ORIGIN,),
    )
    options.update(changes)
    return create_app(PlatformConfig(**options))


def login(client, **kwargs):
    return client.post("/api/auth/login", json={"username": "admin", "password": PASSWORD}, **kwargs)


def test_https_session_cookie_and_same_origin_control(tmp_path):
    with TestClient(make_app(tmp_path), base_url=ORIGIN, headers={"Origin": ORIGIN}) as client:
        response = login(client)
        assert response.status_code == 200
        cookie = response.headers["set-cookie"].lower()
        assert "; secure" in cookie and "; httponly" in cookie and "samesite=strict" in cookie
        assert response.headers["cache-control"] == "no-store"
        assert client.get("/api/auth/me").status_code == 200
        with client.websocket_connect("wss://sim.test/ws/operator") as socket:
            assert socket.receive_json()["type"] == "session"
            # Only protocol ping; never activate control or send motion.
            socket.send_json({"type": "ping"})
            assert socket.receive_json()["type"] == "pong"
        logout = client.post("/api/auth/logout")
        assert "; secure" in logout.headers["set-cookie"].lower()
        assert client.get("/api/auth/me").status_code == 401


@pytest.mark.parametrize("origin", [None, "null", "https://evil.test", "https://sim.test.evil.test"])
def test_http_writes_and_browser_ws_require_allowed_origin(tmp_path, origin):
    with TestClient(make_app(tmp_path), base_url=ORIGIN) as client:
        headers = {"Origin": origin} if origin is not None else {}
        assert login(client, headers=headers).status_code == 403
        assert login(client, headers={"Origin": ORIGIN}).status_code == 200
        with pytest.raises(WebSocketDisconnect) as rejected:
            with client.websocket_connect("wss://sim.test/ws/operator", headers=headers):
                pass
        assert rejected.value.code == 4403
        # Internal health probes must not need a browser origin or credentials.
        assert client.get("/api/health").status_code == 200


def test_local_http_compatibility_and_secure_cookie_not_sent_over_http(tmp_path):
    with TestClient(make_app(tmp_path, secure_cookies=False, allowed_origins=())) as client:
        assert login(client).status_code == 200
        assert client.get("/api/auth/me").status_code == 200
    with TestClient(make_app(tmp_path), base_url=ORIGIN, headers={"Origin": ORIGIN}) as client:
        assert login(client).status_code == 200
        assert client.get("http://sim.test/api/auth/me").status_code == 401


def test_login_rate_limit_does_not_block_health(tmp_path):
    with TestClient(make_app(tmp_path, login_attempts_per_minute=2), base_url=ORIGIN, headers={"Origin": ORIGIN}) as client:
        assert login(client).status_code == 200
        assert login(client).status_code == 200
        result = login(client)
        assert result.status_code == 429 and 1 <= int(result.headers["retry-after"]) <= 60
        assert client.get("/api/health").status_code == 200


def test_existing_default_admin_cannot_be_exposed_by_changing_bootstrap_flags(tmp_path):
    store = AuthStore(tmp_path / "auth.sqlite3", "admin", "ChangeMe123!")
    store.close()
    with pytest.raises(ValueError, match="existing default"):
        make_app(tmp_path)


def test_limiter_expiry_client_isolation_and_bounded_memory():
    limiter = LoginRateLimiter(limit=2, max_clients=2)
    assert limiter.retry_after("a", now=0) == 0
    assert limiter.retry_after("a", now=1) == 0
    assert limiter.retry_after("a", now=2) == 58
    assert limiter.retry_after("b", now=2) == 0
    assert limiter.retry_after("a", now=60) == 0
    assert limiter.retry_after("c", now=60) == 0
    assert len(limiter._clients) == 2


@pytest.mark.parametrize("origin", ["*", "null", "https://*.test", "https://sim.test/", "https://sim.test/path", "https://user@sim.test", "https://sim.test?x=1", "https://sim.test#x", "ws://sim.test", "https://sim.test:99999"])
def test_invalid_origin_configuration_fails_closed(origin):
    with pytest.raises(ValueError):
        validate_origins((origin,))


def test_valid_origins():
    assert validate_origins((ORIGIN, "http://localhost:8000")) == {ORIGIN, "http://localhost:8000"}


def test_cli_environment_wires_proxy_and_secure_settings_without_multiple_workers(monkeypatch):
    import sys
    from space_arm_platform import main
    captured = {}
    monkeypatch.setenv("SPACE_SIM_STREAM_JWT_SECRET", "test-jwt")
    monkeypatch.setenv("SPACE_SIM_STREAM_ACCESS_KEY", "test-key")
    monkeypatch.setenv("SPACE_SIM_SECURE_COOKIES", "1")
    monkeypatch.setenv("SPACE_SIM_ALLOWED_ORIGINS", '["https://sim.test"]')
    monkeypatch.setenv("SPACE_SIM_NO_ACCESS_LOG", "1")
    monkeypatch.setenv("SPACE_SIM_FORWARDED_ALLOW_IPS", "127.0.0.1,::1")
    monkeypatch.setattr(sys, "argv", ["backend"])
    def create(config):
        captured["config"] = config
        return "app-stub"
    monkeypatch.setattr(main, "create_app", create)
    monkeypatch.setattr(main.uvicorn, "run", lambda app, **kwargs: captured.update(kwargs))
    main.main()
    assert captured["config"].secure_cookies is True
    assert captured["config"].allowed_origins == (ORIGIN,)
    assert captured["config"].stream_access_jwt_secret == "test-jwt"
    assert captured["config"].stream_access_key == "test-key"
    assert captured["access_log"] is False
    assert captured["forwarded_allow_ips"] == "127.0.0.1,::1"
    assert captured.get("workers", 1) == 1


def test_default_admin_preflight_is_read_only(tmp_path):
    import runpy
    check_database = runpy.run_path(str(Path(__file__).resolve().parents[1] / "tools/check_deployment_auth.py"))["check_database"]
    missing = tmp_path / "does-not-exist.sqlite3"
    check_database(missing)
    assert not missing.exists()
    path = tmp_path / "auth.sqlite3"
    auth = AuthStore(path, "admin", "ChangeMe123!")
    auth.close()
    before = path.read_bytes()
    with pytest.raises(ValueError, match="existing default"):
        check_database(path)
    assert path.read_bytes() == before
    auth = AuthStore(path, "admin", "ChangeMe123!")
    admin = auth.authenticate("admin", "ChangeMe123!")
    auth.change_password(admin["user_id"], "ChangeMe123!", PASSWORD)
    auth.close()
    before = path.read_bytes()
    check_database(path)
    assert path.read_bytes() == before

@pytest.mark.parametrize("client_ip", [
    "192.0.2.17", "198.51.100.29", "203.0.113.41", "10.0.0.22", "2001:db8::17",
])
def test_public_origin_is_not_a_client_ip_allowlist(tmp_path, client_ip):
    """Source addresses may vary; origin, authentication and access-key checks stay on."""
    access_key = "test-public-access-" + "k" * 32
    with TestClient(
        make_app(tmp_path, stream_access_key=access_key), base_url=ORIGIN,
        headers={"Origin": ORIGIN}, client=(client_ip, 40000),
    ) as client:
        assert client.get("/api/auth/me").status_code == 401
        assert login(client).status_code == 200
        assert client.get("/api/auth/me").status_code == 200
        with client.websocket_connect(
            f"wss://sim.test/ws/operator?access_key={access_key}"
        ) as socket:
            assert socket.receive_json()["type"] == "session"
            # No control acquisition or motion; isolated protocol keepalive only.
            socket.send_json({"type": "ping"})
            assert socket.receive_json()["type"] == "pong"
        assert login(client, headers={"Origin": "https://other.test"}).status_code == 403
