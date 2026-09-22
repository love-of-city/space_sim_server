"""Exercise the local scene API and video with an isolated authentication database."""

import json
import os
from pathlib import Path
import secrets
import subprocess
import time
from urllib.parse import urlencode

import httpx
from websockets.sync.client import connect


ROOT = Path(__file__).resolve().parents[1]
URL = "http://127.0.0.1:18000"


def verify_control(client, initial_observation, *, url=URL, access_key=None):
    observations = []
    sequences = set()
    cookie = "; ".join(f"{key}={value}" for key, value in client.cookies.items())
    operator_url = url.replace("https:", "wss:").replace("http:", "ws:") + "/ws/operator"
    if access_key:
        operator_url += "?" + urlencode({"access_key": access_key})
    with connect(operator_url, origin=url,
                 additional_headers={"Cookie": cookie}, proxy=None) as socket:
        def receive(message_type):
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                message = json.loads(socket.recv(timeout=10))
                if message["type"] == "observation":
                    observations.append(message["payload"])
                if message["type"] == "action_rejected":
                    raise RuntimeError(message)
                if message["type"] == message_type:
                    return message
            raise TimeoutError(message_type)

        receive("session")
        socket.send(json.dumps({"type": "activate_control"}))
        receive("control_granted")
        action = {
            "type": "operator_action", "deadman": True, "input_source": "keyboard",
            "end_effector_linear_speed_m_s": 0.01,
            "end_effector_linear_velocity": [0.2, 0, 0],
            "end_effector_angular_velocity": [0, 0, 0], "gripper_velocity": 0,
        }
        try:
            for sequence in range(1, 21):
                action.update(client_sequence=sequence, client_time_ns=str(time.time_ns()))
                socket.send(json.dumps(action))
                acknowledgement = receive("action_ack")
                assert acknowledgement["delivered_to_simulation"]
                sequences.add(acknowledgement["server_sequence"])
                time.sleep(0.05)
            receive("observation")
        finally:
            action.update(deadman=False, client_sequence=21, client_time_ns=str(time.time_ns()))
            socket.send(json.dumps(action))
            receive("action_ack")
    applied = [item for item in observations if item["applied_action_sequence"] in sequences]
    assert applied, "No simulator observation acknowledged the control sequence"
    baseline = initial_observation["target_arm_joint_position_rad"]
    delta = max(abs(actual - initial) for item in applied
                for actual, initial in zip(item["target_arm_joint_position_rad"], baseline))
    assert delta > 1e-7, "IK targets did not respond to the control pulse"
    return {"delivered_commands": len(sequences), "acknowledged_observations": len(applied),
            "max_target_joint_delta_rad": delta}


def main():
    run = ROOT / "run" / f"platform-smoke-{time.time_ns()}"
    run.mkdir(parents=True)
    environment = os.environ.copy()
    environment["SPACE_SIM_ADMIN_USERNAME"] = "smokeadmin"
    environment["SPACE_SIM_ADMIN_PASSWORD"] = secrets.token_urlsafe(36)
    environment["PIXEL_STREAMING_USERNAME"] = environment["SPACE_SIM_ADMIN_USERNAME"]
    environment["PIXEL_STREAMING_PASSWORD"] = environment["SPACE_SIM_ADMIN_PASSWORD"]
    environment["CHROME_PATH"] = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
    environment["PIXEL_STREAMING_WARMUP_MS"] = "25000"
    launcher = ROOT / "scripts/local_visualization.ps1"
    with (run / "launcher.log").open("w", encoding="utf-8") as output:
        process = subprocess.Popen(
            ["powershell.exe", "-NoProfile", "-File", str(launcher), "-NoBrowser",
             "-AuthDatabase", str(run / "auth.sqlite3")],
            cwd=ROOT, env=environment, stdout=output, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
        try:
            with httpx.Client(base_url=URL, headers={"Origin": URL}, timeout=20, trust_env=False) as client:
                deadline = time.monotonic() + 90
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f"Launcher exited; inspect {run}")
                    try:
                        if client.get("/api/health").status_code == 200:
                            break
                    except httpx.HTTPError:
                        pass
                    time.sleep(1)
                response = client.post("/api/auth/login", json={
                    "username": environment["SPACE_SIM_ADMIN_USERNAME"],
                    "password": environment["SPACE_SIM_ADMIN_PASSWORD"],
                })
                response.raise_for_status()
                state = client.get("/api/state").json()
                assert state["scene_runtime"]["enabled"], state
                response = client.post("/api/scenes/start", json={"seed": 42})
                response.raise_for_status()
                print("Scene requested:", response.json()["instance_id"], flush=True)
                deadline = time.monotonic() + 300
                while time.monotonic() < deadline:
                    state = client.get("/api/state").json()
                    phase = state["scene_runtime"].get("phase")
                    if phase == "failed":
                        raise RuntimeError(state["scene_runtime"].get("error"))
                    observation = state["simulation"].get("latest_observation")
                    if state["simulation"]["connected"] and observation:
                        break
                    time.sleep(2)
                else:
                    raise TimeoutError(f"Scene never connected: {state}")
                initial_time = int(observation["sim_time_ns"])
                time.sleep(3)
                state = client.get("/api/state").json()
                assert int(state["simulation"]["latest_observation"]["sim_time_ns"]) > initial_time
                (run / "connected-state.json").write_text(json.dumps(state, indent=2), encoding="utf-8")
                print("Simulation connected; observation time advances.", flush=True)
                control = verify_control(client, state["simulation"]["latest_observation"])
                (run / "control.json").write_text(json.dumps(control, indent=2), encoding="utf-8")
                print("Control verified:", control, flush=True)
                with (run / "video.log").open("w", encoding="utf-8") as video_output:
                    video = subprocess.run(
                        [r"C:\Program Files\nodejs\node.exe", "tools/verify_pixel_streaming.mjs", URL + "/"],
                        cwd=ROOT, env=environment, stdout=video_output, stderr=subprocess.STDOUT,
                        timeout=120, creationflags=subprocess.CREATE_NO_WINDOW,
                    )
                print((run / "video.log").read_text(encoding="utf-8"), flush=True)
                assert video.returncode == 0, f"Video validation failed; inspect {run}"
                client.post("/api/scenes/stop").raise_for_status()
                assert client.get("/api/health").json()["ok"]
                print("Scene stopped; website remains healthy.", flush=True)
        finally:
            if process.poll() is None:
                subprocess.run(["powershell.exe", "-NoProfile", "-File", str(launcher), "-Action", "Stop"],
                               cwd=ROOT, timeout=60, check=True, creationflags=subprocess.CREATE_NO_WINDOW)
            process.wait(timeout=30)
            print("Smoke artifacts:", run, flush=True)


if __name__ == "__main__":
    main()
