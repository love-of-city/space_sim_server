"""Validate the public entry point with a temporary operator, leaving deployment running."""

import json
import os
from pathlib import Path
import secrets
import sqlite3
import subprocess
import sys
import time

import httpx

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.auth import AuthStore
from verify_local_platform import verify_control


def main():
    report = json.loads((ROOT / "run/public-access.json").read_text(encoding="utf-8-sig"))
    url = report["public_url"]
    require_access_key = report.get("require_access_key", True)
    access_key = os.environ["PIXEL_STREAMING_ACCESS_KEY"] if require_access_key else None
    database = ROOT / "data/auth.sqlite3"
    with sqlite3.connect(database.as_uri() + "?mode=ro", uri=True) as existing:
        if not existing.execute("SELECT 1 FROM users WHERE role='admin' AND active=1").fetchone():
            raise RuntimeError("An existing administrator is required; verifier will not bootstrap one")
    run = ROOT / "run" / f"remote-smoke-{time.time_ns()}"
    run.mkdir()
    username = "remote-smoke-" + secrets.token_hex(5)
    password = secrets.token_urlsafe(32)
    auth = AuthStore(database, "unused-bootstrap", secrets.token_urlsafe(32))
    operator = auth.create_operator(username, password)
    scene_id = None
    try:
        with httpx.Client(base_url=url, headers={"Origin": url}, trust_env=False, timeout=40) as client:
            client.get("/api/health").raise_for_status()
            assert client.get("/api/state").status_code == 401
            assert client.get("/api/client-config").status_code == 401
            client.post("/api/auth/login", json={"username": username, "password": password}).raise_for_status()
            if require_access_key:
                assert client.get("/api/client-config").status_code == 401
            configuration = client.get("/api/client-config", headers={"x-space-arm-access-key": access_key} if access_key else {})
            configuration.raise_for_status()
            configuration = configuration.json()
            assert configuration["pixel_streaming_signalling_url"] == url.replace("https:", "wss:") + "/stream"
            print(f"Public HTTPS login passed; access key required: {require_access_key}; WSS configured.", flush=True)
            state = client.get("/api/state").json()
            if state["scene_runtime"].get("active"):
                raise RuntimeError("A scene is already active; verifier will not replace it")
            try:
                started = client.post("/api/scenes/start", json={"seed": 42})
                started.raise_for_status()
                scene_id = started.json()["instance_id"]
                deadline = time.monotonic() + 300
                while time.monotonic() < deadline:
                    state = client.get("/api/state").json()
                    if state["scene_runtime"].get("phase") == "failed":
                        raise RuntimeError(state["scene_runtime"].get("error"))
                    observation = state["simulation"].get("latest_observation")
                    if state["simulation"]["connected"] and observation:
                        break
                    time.sleep(2)
                else:
                    raise TimeoutError("Public scene never produced observations")
                control = verify_control(client, observation, url=url, access_key=access_key)
                print("Public WSS control verified:", control, flush=True)
                environment = os.environ.copy()
                if not require_access_key:
                    environment.pop("PIXEL_STREAMING_ACCESS_KEY", None)
                environment.update(PIXEL_STREAMING_USERNAME=username, PIXEL_STREAMING_PASSWORD=password,
                                   PIXEL_STREAMING_WARMUP_MS="35000",
                                   CHROME_PATH=r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")
                with (run / "video.log").open("w", encoding="utf-8") as output:
                    video = subprocess.run(
                        [r"C:\Program Files\nodejs\node.exe", "tools/verify_pixel_streaming.mjs", url + "/"],
                        cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT,
                        timeout=180, creationflags=subprocess.CREATE_NO_WINDOW,
                    )
                print((run / "video.log").read_text(encoding="utf-8"), flush=True)
                assert video.returncode == 0, "Public-page video verification failed"
                if os.environ.get("PIXEL_STREAMING_FPS_TEST") == "1":
                    environment.update(BSK_STREAM_TEST_URL=url + "/", BSK_STREAM_TEST_USER=username,
                                       BSK_STREAM_TEST_PASSWORD=password, BSK_STREAM_TEST_SECONDS="20",
                                       BSK_STREAM_TEST_WARMUP="8", BSK_STREAM_TEST_FPS_TARGETS="90,60,30,90",
                                       BSK_STREAM_TEST_OUTPUT=str(run / "fps"))
                    with (run / "fps.log").open("w", encoding="utf-8") as output:
                        fps = subprocess.run(
                            [r"C:\Program Files\nodejs\node.exe", "scripts/verify_stream_fps.mjs"],
                            cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT,
                            timeout=300, creationflags=subprocess.CREATE_NO_WINDOW,
                        )
                    print((run / "fps.log").read_text(encoding="utf-8"), flush=True)
                    assert fps.returncode == 0, "Sustained video/FPS selector verification failed"
                result = {"public_url": url, "control": control, "video_passed": True,
                          "access_key_required": require_access_key,
                          "forced_turn_relay": os.environ.get("PIXEL_STREAMING_FORCE_RELAY") == "1",
                          "turn_transport": os.environ.get("PIXEL_STREAMING_TURN_TRANSPORT", ""),
                          "client_location": "server-side browser using public HTTPS/WSS; user's external network remains unverified"}
                (run / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            finally:
                if scene_id:
                    current = client.get("/api/state").json()["scene_runtime"]
                    if current.get("instance", {}).get("instance_id") == scene_id and current.get("active"):
                        client.post("/api/scenes/stop").raise_for_status()
                assert client.get("/api/health").json()["ok"]
    finally:
        auth.close()
        with sqlite3.connect(database) as cleanup:
            cleanup.execute("BEGIN IMMEDIATE")
            cleanup.execute("DELETE FROM sessions WHERE user_id=?", (operator["user_id"],))
            cleanup.execute("DELETE FROM users WHERE user_id=? AND username=? AND role='operator'", (operator["user_id"], username))
        print("Temporary operator removed. Artifacts:", run, flush=True)


if __name__ == "__main__":
    main()
