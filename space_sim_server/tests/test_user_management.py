"""Account management and schema upgrades use disposable databases only."""
import sqlite3

import pytest
from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.auth import AuthError, AuthStore
from space_arm_platform.recorder import EpisodeRecorder


def test_legacy_database_upgrades_without_replacing_credentials(tmp_path):
    path = tmp_path / "legacy.sqlite3"
    digest, salt, iterations = AuthStore._password_record("Existing-Password!")
    with sqlite3.connect(path) as db:
        db.execute("""CREATE TABLE users (user_id TEXT PRIMARY KEY, username TEXT UNIQUE COLLATE NOCASE,
                   role TEXT, password_hash TEXT, password_salt TEXT, password_iterations INTEGER,
                   active INTEGER, created_at_ns TEXT)""")
        db.execute("INSERT INTO users VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                   ("original", "admin", "admin", digest, salt, iterations, 1, "123"))
    store = AuthStore(path, "admin", "Unused-Password!")
    user = store.authenticate("admin", "Existing-Password!")
    assert user["user_id"] == "original" and user["created_at_ns"] == "123"
    assert user["display_name"] == "admin" and user["last_login_at_ns"] is None
    token, _ = store.create_session(user["user_id"])
    assert int(store.session_user(token)["last_login_at_ns"]) > 0
    store.close()
    reopened = AuthStore(path, "admin", "Unused-Password!")
    assert reopened.authenticate("admin", "Existing-Password!")["user_id"] == "original"
    reopened.close()


def test_disable_and_role_change_revoke_sessions_and_protect_current_admin(tmp_path):
    store = AuthStore(tmp_path / "auth.sqlite3", "admin", "Admin-Password!")
    admin = store.list_users()[0]
    user = store.create_user("operator", "Operator-Password!", "采集成员")
    token, _ = store.create_session(user["user_id"])
    store.update_user(admin["user_id"], user["user_id"], "采集成员", "operator", False)
    assert store.session_user(token) is None
    assert store.authenticate("operator", "Operator-Password!") is None
    with pytest.raises(AuthError):
        store.create_session(user["user_id"])
    store.update_user(admin["user_id"], user["user_id"], "采集成员", "operator", True)
    token, _ = store.create_session(user["user_id"])
    store.update_user(admin["user_id"], user["user_id"], "采集成员", "admin", True)
    assert store.session_user(token) is None
    for role, active in (("operator", True), ("admin", False)):
        with pytest.raises(AuthError, match="自己的|当前"):
            store.update_user(admin["user_id"], admin["user_id"], "管理员", role, active)
    store.close()


def test_user_management_api_authorization_validation_and_password_reset(tmp_path, monkeypatch):
    project = tmp_path / "project"
    (project / "frontend").mkdir(parents=True)
    (project / "frontend/index.html").write_text("<main>test shell</main>", encoding="utf8")
    monkeypatch.setattr(EpisodeRecorder, "prepare", lambda self: None)
    app = create_app(PlatformConfig(project_root=project, data_root=tmp_path / "episodes", simulation_port=0,
                                    capture_port=0, auth_database=tmp_path / "auth.sqlite3"))
    with TestClient(app) as client:
        assert client.get("/api/users").status_code == 401
        client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).raise_for_status()
        response = client.post("/api/users", json={"username": "operator", "password": "Operator-Password!", "display_name": "采集成员"})
        assert response.status_code == 201
        user = response.json()
        assert user["role"] == "operator" and user["active"]
        assert not any("password" in key for key in user)
        assert client.post("/api/users", json={"username": "OPERATOR", "password": "Operator-Password!"}).status_code == 422
        assert client.patch(f"/api/users/{user['user_id']}", json={"display_name": " ", "role": "operator", "active": True}).status_code == 422
        client.post(f"/api/users/{user['user_id']}/reset-password", json={"password": "Replacement-Password!"}).raise_for_status()
        client.post("/api/auth/logout")
        assert client.post("/api/auth/login", json={"username": "operator", "password": "Operator-Password!"}).status_code == 401
        client.post("/api/auth/login", json={"username": "operator", "password": "Replacement-Password!"}).raise_for_status()
        assert client.get("/api/users").status_code == 403
        assert client.post("/api/users", json={"username": "new-admin", "password": "Another-Password!", "role": "admin"}).status_code == 403
        assert client.patch(f"/api/users/{user['user_id']}", json={"display_name": "提权", "role": "admin", "active": True}).status_code == 403
        assert client.post(f"/api/users/{user['user_id']}/reset-password", json={"password": "Another-Password!"}).status_code == 403
