"""Isolated real UE + Basilisk capture regression (never touches production ports/data)."""
from __future__ import annotations
import argparse
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from space_arm_platform.capture_receiver import CaptureReceiver
from space_arm_platform.models import AppliedAction, EpisodeStart, EpisodeStop
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub


async def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    # The input may be a preview-only production scene. Never modify it: the
    # isolated recording check needs reliable rendering and authoritative RGB.
    scene = json.loads(args.scene.read_text(encoding="utf-8-sig"))
    scene["runtime"] = {**scene.get("runtime", {}), "dataset_capture": True,
                        "dynamics_rate_hz": 240, "ik_rate_hz": 120, "capture_rate_hz": 30}
    validation_scene = out / "validation.scene.json"
    validation_scene.write_text(json.dumps(scene, indent=2, ensure_ascii=False), encoding="utf-8")
    recorder = EpisodeRecorder(out / "episodes")
    hub = SimulationHub()
    latest_action = None
    async def observation(obs):
        await asyncio.to_thread(recorder.record_observation, obs, latest_action)
    hub.on_observation = observation
    hub.on_transport_error = recorder.fail
    receiver = CaptureReceiver("127.0.0.1", args.capture_port, recorder.record_authoritative_capture, recorder.fail)
    processes = []
    logs = []
    results = []
    gaps = []
    heart_running = True
    async def heartbeat():
        previous = time.monotonic()
        while heart_running:
            await asyncio.sleep(.02)
            now = time.monotonic(); gaps.append(now - previous); previous = now
    heart = asyncio.create_task(heartbeat())
    def launch(command, name, env=None):
        log = (out / (name + ".log")).open("w", encoding="utf-8")
        logs.append(log)
        p = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        processes.append(p)
        return p
    try:
        started = time.monotonic()
        await asyncio.to_thread(recorder.prepare)
        print(json.dumps({"stage":"writer_ready","cold_start_seconds":time.monotonic()-started}), flush=True)
        await hub.start("127.0.0.1", args.control_port)
        receiver.start()
        project = args.adapter / "Unreal/BskUnrealRenderer"
        editor = args.unreal / "Engine/Binaries/Win64/UnrealEditor.exe"
        env = os.environ.copy()
        env["UE-LocalDataCachePath"] = str(project / "Saved/DerivedDataCache")
        command = [str(editor),str(project / "BskUnrealRenderer.uproject"),"-game","-RenderOffscreen","-ForceRes",
                   "-ResX=640","-ResY=360",f"-BskPort={args.render_port}","-BskListen=127.0.0.1",
                   "-BskCaptureHost=127.0.0.1",f"-BskCapturePort={args.capture_port}","-BskCaptureRate=30",
                   "-BskCaptureProducts=rgb",'-ExecCmds=t.MaxFPS 90,r.VSync 0',
                   "-DDC=InstalledDerivedDataBackendGraph",f"-abslog={out / 'renderer.log'}",
                   "-PixelStreamingConnectionURL=ws://127.0.0.1:8888","-PixelStreamingID=BskValidation",
                   "-BskPixelStreamingURL=ws://127.0.0.1:8888","-BskPixelStreamingBaseId=BskValidation",
                   "-BskPixelStreamingCameras=teleop/camera/spacecraft_overview+teleop/camera/sarm_wrist_cam",
                   "-BskPixelStreamingCameraWidth=640","-BskPixelStreamingCameraHeight=360",
                   "-BskPixelStreamingCameraFps=90","-PixelStreamingWebRTCFps=90",
                   "-PixelStreamingUseMediaCapture=false","-PixelStreamingWebRTCDisableTransmitAudio=true"]
        renderer = launch(command, "renderer-process", env)
        deadline=time.monotonic()+240
        while True:
            if renderer.poll() is not None: raise RuntimeError("renderer exited during startup")
            try:
                s=socket.create_connection(("127.0.0.1",args.render_port),timeout=.1);s.close();break
            except OSError:
                if time.monotonic()>deadline:raise TimeoutError("renderer startup timeout")
                await asyncio.sleep(.25)
        command = [sys.executable,str(ROOT / "simulation/teleop_grasp_unreal.py"),
                   "--adapter-root",str(args.adapter),"--model-root",str(ROOT / "model/SARM/platform"),
                   "--catalog",str(project / "Saved/AssetImport/sarm_platform.catalog.json"),
                   "--scene-instance",str(validation_scene),"--control-port",str(args.control_port),
                   "--render-port",str(args.render_port),"--duration","0","--capture-rate","30","--ik-rate","120"]
        env["PYTHONPATH"] = os.pathsep.join([str(ROOT/'backend'),str(args.adapter/'Adapters')])
        simulation = launch(command,"simulation",env)
        deadline=time.monotonic()+180
        while hub.latest_observation is None or receiver.authoritative_count<4:
            if simulation.poll() is not None:raise RuntimeError("simulation exited during initialization")
            if time.monotonic()>deadline:raise TimeoutError("no state/RGB from simulation")
            await asyncio.sleep(.1)
        sequence=1
        for episode in range(args.episodes):
            info=await asyncio.to_thread(recorder.start,EpisodeStart(tags=["diagnostic","not-demonstration"],instruction="实时采集架构回归测试"))
            print(json.dumps({"stage":"recording","episode_id":info['episode_id']}),flush=True)
            start_sim=int(hub.latest_observation.sim_time_ns)
            interrupted=False
            last_report=0
            deadline=time.monotonic()+max(120,args.seconds*4)
            while True:
                current = hub.latest_observation
                if current is None:
                    await asyncio.sleep(.02)
                    continue
                if (int(current.sim_time_ns)-start_sim)/1e9 >= args.seconds:
                    break
                if simulation.poll() is not None or renderer.poll() is not None:raise RuntimeError("runtime exited during recording")
                if time.monotonic()>deadline:raise TimeoutError("recording did not advance")
                elapsed=(int(hub.latest_observation.sim_time_ns)-start_sim)/1e9
                # Exercise nonzero controls, not only a static scene. Keep motion small.
                linear = .006 if int(elapsed/3)%2==0 else -.006
                latest_action=AppliedAction(server_sequence=str(sequence),server_time_ns=str(time.time_ns()),
                    client_sequence=str(sequence),client_time_ns=str(time.time_ns()),deadman=True,
                    end_effector_linear_velocity_body_m_s=[linear,0.,0.],end_effector_angular_velocity_body_rad_s=[0.,0.,0.],
                    gripper_velocity_rad_s=0.,gripper_velocity_m_s=0.,input_source="keyboard")
                await hub.publish_action(latest_action);sequence+=1
                if args.reconnect and not interrupted and elapsed>3:
                    interrupted=True
                    hub._writer.close()
                status=recorder.sync_status()
                if status['dataset_error']:raise RuntimeError(status['dataset_error'])
                if elapsed-last_report>=5:
                    last_report=elapsed;print(json.dumps({"sim_elapsed":elapsed,"sync":status}),flush=True)
                await asyncio.sleep(.05)
            result=await asyncio.to_thread(recorder.stop,EpisodeStop(outcome="unknown",note="isolated architecture regression; not a grasp demonstration"))
            results.append(result)
            print(json.dumps({k:result[k] for k in ['episode_id','dataset_status','dataset_frame_count','capture_count','step_count','dataset_error']}),flush=True)
            assert result['dataset_status']=='complete',result
            assert result['capture_count']==2*result['dataset_frame_count']==2*result['step_count'],result
            assert result['incomplete_sample_count']==0 and result['rejected_capture_count']==0,result
            await asyncio.sleep(1)
        summary={"episodes":results,"max_event_loop_pause_seconds":max(gaps,default=0),
                 "capture_transport":receiver.status(),"observation_replays":hub.status()['observation_replays_deduplicated']}
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
    finally:
        if recorder.episode_id:
            try:await asyncio.to_thread(recorder.stop,EpisodeStop(outcome="aborted",note="validation interrupted"))
            except Exception:pass
        for p in reversed(processes):
            if p.poll() is None:
                p.terminate()
                try:await asyncio.to_thread(p.wait,10)
                except subprocess.TimeoutExpired:p.kill()
        await asyncio.to_thread(receiver.close)
        await hub.close()
        await asyncio.to_thread(recorder.close)
        heart_running=False;await heart
        for log in logs:log.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--adapter',type=Path,required=True)
    p.add_argument('--unreal',type=Path,required=True)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=20.)
    p.add_argument('--episodes',type=int,default=2)
    p.add_argument('--control-port',type=int,default=18766)
    p.add_argument('--capture-port',type=int,default=18767)
    p.add_argument('--render-port',type=int,default=15558)
    p.add_argument('--reconnect',action='store_true')
    args=p.parse_args()
    # Fail before launching anything if a chosen port is already in use.
    for port in [args.control_port,args.capture_port,args.render_port]:
        with socket.socket() as s:s.bind(('127.0.0.1',port))
    asyncio.run(run(args))

if __name__=='__main__':main()
