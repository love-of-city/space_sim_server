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
