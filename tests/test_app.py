from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app


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
        assert client.get("/api/client-config").json() == {
            "preview_transport": "pixel_streaming_2",
            "pixel_streaming_player_port": 18080,
            "pixel_streaming_streamer_id": "test-streamer",
        }
        assert client.get("/api/health").json()["capture_channels"] == {
            "preview_cameras": [],
            "preview_count": 0,
            "authoritative_count": 0,
            "last_error": None,
            "last_authoritative_error": None,
        }
        index = client.get("/").text
        frontend = client.get("/static/app.js").text
        assert 'id="pixelStream"' in index
        assert "/api/preview-stream" not in frontend
        assert "AutoConnect" in frontend
