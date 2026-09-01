from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.stream_access import verify_stream_access_token


def login_admin(client: TestClient) -> None:
    response = client.post(
        "/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}
    )
    assert response.status_code == 200
    assert response.json()["user"]["role"] == "admin"


def test_health_and_episode_lifecycle(tmp_path) -> None:
    project_root = __import__("pathlib").Path(__file__).resolve().parents[1]
    app = create_app(
        PlatformConfig(
            project_root=project_root,
            data_root=tmp_path,
            simulation_port=0,
            capture_port=0,
        )
    )
    with TestClient(app) as client:
        login_admin(client)
        assert client.get("/api/health").json()["ok"] is True
        started = client.post("/api/episodes/start", json={"instruction": "test"})
        assert started.status_code == 200
        assert client.get("/api/state").json()["active_episode"]
        stopped = client.post("/api/episodes/stop", json={"outcome": "success"})
        assert stopped.status_code == 200
        assert stopped.json()["outcome"] == "success"


def test_client_config_selects_pixel_streaming_without_changing_capture_channel(tmp_path) -> None:
    project_root = __import__("pathlib").Path(__file__).resolve().parents[1]
    app = create_app(
        PlatformConfig(
            project_root=project_root,
            data_root=tmp_path,
            simulation_port=0,
            capture_port=0,
            pixel_streaming_player_port=18080,
            pixel_streaming_streamer_id="test-streamer",
        )
    )
    with TestClient(app) as client:
        login_admin(client)
        assert client.get("/api/client-config").json() == {
            "preview_transport": "pixel_streaming_2",
            "pixel_streaming_player_port": 18080,
            "pixel_streaming_streamer_id": "test-streamer",
            "pixel_streaming_signalling_url": "",
            "pixel_streaming_streamers": [{"id": "test-streamer", "label": "主视口"}],
        }
        assert client.get("/api/state").json()["capture_channels"] == {
            "preview_cameras": [],
            "preview_count": 0,
            "authoritative_count": 0,
            "last_error": None,
            "last_authoritative_error": None,
        }
        index = client.get("/").text
        frontend = (project_root / "frontend" / "app.js").read_text(encoding="utf-8")
        assert 'id="pixelStream"' in index
        assert "/api/preview-stream" not in frontend
        assert "PixelStreaming" in frontend
        assert "operationActive" in frontend
        assert "enterOperationMode" in frontend
        assert "exitOperationMode" in frontend
        assert 'event.code === "Escape" && state.operationActive' in frontend
        assert "sendNeutralAction" in frontend
        assert 'id="operationHint"' in index


def test_remote_client_config_requires_access_key_and_scopes_streamers(tmp_path) -> None:
    project_root = __import__("pathlib").Path(__file__).resolve().parents[1]
    app = create_app(
        PlatformConfig(
            project_root=project_root,
            data_root=tmp_path / "episodes",
            simulation_port=0,
            capture_port=0,
            pixel_streaming_streamer_id="main",
            pixel_streaming_signalling_url="wss://example.test/stream",
            pixel_streaming_camera_streamers=(("wrist", "Wrist"),),
            stream_access_jwt_secret="jwt-secret",
            stream_access_key="page-key",
        )
    )
    with TestClient(app) as client:
        assert client.get("/api/client-config").status_code == 401
        login_admin(client)
        assert client.get("/api/client-config").status_code == 401
        response = client.get("/api/client-config", headers={"X-Space-Arm-Access-Key": "page-key"})
        assert response.status_code == 200
        payload = response.json()
        claims = verify_stream_access_token("jwt-secret", payload["pixel_streaming_access_token"])
        assert claims["streamer_ids"] == ["main", "wrist"]
        assert payload["pixel_streaming_signalling_url"] == "wss://example.test/stream"


def test_admin_manages_operator_accounts(tmp_path) -> None:
    project_root = __import__("pathlib").Path(__file__).resolve().parents[1]
    app = create_app(
        PlatformConfig(project_root=project_root, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0)
    )
    with TestClient(app) as client:
        login_admin(client)
        created = client.post(
            "/api/users/operators", json={"username": "operator1", "password": "Operator123!"}
        )
        assert created.status_code == 200
        operator = created.json()
        assert operator["role"] == "operator"
        assert any(user["username"] == "operator1" for user in client.get("/api/users").json()["users"])
        client.post("/api/auth/logout")
        assert client.post(
            "/api/auth/login", json={"username": "operator1", "password": "Operator123!"}
        ).status_code == 200
        assert client.get("/api/users").status_code == 403
        client.post("/api/auth/logout")
        login_admin(client)
        reset = client.post(
            f"/api/users/operators/{operator['user_id']}/reset-password",
            json={"password": "Operator456!"},
        )
        assert reset.status_code == 200
        deleted = client.delete(f"/api/users/operators/{operator['user_id']}")
        assert deleted.status_code == 200


def test_same_operator_new_page_automatically_takes_control(tmp_path) -> None:
    import json
    from pathlib import Path

    project_root = tmp_path / "project"
    frontend = project_root / "frontend"
    frontend.mkdir(parents=True)
    (frontend / "index.html").write_text("<!doctype html><title>test</title>", encoding="utf-8")
    app = create_app(
        PlatformConfig(project_root=project_root, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0)
    )
    with TestClient(app) as client:
        login_admin(client)
        operator = client.post(
            "/api/users/operators", json={"username": "pilot", "password": "PilotPass123!"}
        ).json()
        client.post("/api/auth/logout")
        assert client.post(
            "/api/auth/login", json={"username": "pilot", "password": "PilotPass123!"}
        ).status_code == 200
        run_root = project_root / "run"
        run_root.mkdir(parents=True, exist_ok=True)
        (run_root / "scene_runtime.json").write_text(
            json.dumps(
                {
                    "phase": "running",
                    "instance": {
                        "instance_id": "scene-test",
                        "created_by": {
                            "user_id": operator["user_id"],
                            "username": "pilot",
                            "role": "operator",
                        },
                    },
                }
            ),
            encoding="utf-8",
        )
        with client.websocket_connect("/ws/operator") as first:
            assert first.receive_json()["control_granted"] is True
            with client.websocket_connect("/ws/operator") as second:
                assert second.receive_json()["control_granted"] is False
                second.send_json({"type": "activate_control"})
                assert first.receive_json() == {
                    "type": "control_revoked",
                    "reason": "another_page_activated",
                }
                granted = second.receive_json()
                assert granted["type"] == "control_granted"
                assert granted["control_granted"] is True


def test_user_can_change_own_password(tmp_path) -> None:
    project_root = __import__("pathlib").Path(__file__).resolve().parents[1]
    app = create_app(
        PlatformConfig(project_root=project_root, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0)
    )
    with TestClient(app) as client:
        login_admin(client)
        changed = client.post(
            "/api/auth/change-password",
            json={"current_password": "ChangeMe123!", "new_password": "NewAdminPass123!"},
        )
        assert changed.status_code == 200
        assert client.get("/api/auth/me").status_code == 401
        assert client.post(
            "/api/auth/login", json={"username": "admin", "password": "NewAdminPass123!"}
        ).status_code == 200
