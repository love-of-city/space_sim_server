"""Headless native SARM -> real SimulationHub/Recorder and render-wire smoke.

No running platform or UE process is touched.  Uses private ephemeral ports and
an isolated Episode directory. The render peer records protocol frames, not GPU
images; this test deliberately does not claim Pixel Streaming validation.
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
from space_arm_platform.models import AppliedAction, EpisodeStart, EpisodeStop
from space_arm_platform.protocol import read_async
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub


async def run(args):
    args.output.mkdir(parents=True, exist_ok=True)
    hub = SimulationHub()
    recorder = EpisodeRecorder(args.output / 'episodes')
    episode = recorder.start(EpisodeStart(instruction='Isolated reaction-wheel integration validation'))
    observations, frames, manifests = [], [], []
    render_writers = set()
    sequence = 0

    async def on_observation(obs):
        nonlocal sequence
        observations.append(obs.model_dump(mode='json'))
        recorder.record_observation(obs, None)
        seconds = int(obs.sim_time_ns) / 1e9
        # Stop sending after 8 s: arm watchdog must expire, but onboard
        # attitude stabilization must continue without browser input.
        if seconds >= 8:
            return
        sequence += 1
        moving = 1 <= seconds < 4
        action = AppliedAction(
            server_sequence=str(sequence), server_time_ns=str(time.time_ns()),
            client_sequence=str(sequence), client_time_ns=str(time.time_ns()),
            deadman=True, input_source='integration-test',
            end_effector_linear_velocity_body_m_s=[.01 if moving else 0., 0., 0.],
            end_effector_angular_velocity_body_rad_s=[0., .04 if moving else 0., 0.],
            gripper_velocity_rad_s=0.,
            gripper_velocity_m_s=-.005 if 4 <= seconds < 6 else .005 if 6 <= seconds < 8 else 0.,
        )
        recorder.record_action(action)
        await hub.publish_action(action)

    async def render_peer(reader, writer):
        render_writers.add(writer)
        try:
            while True:
                message = await read_async(reader)
                if message.get('type') == 'frame':
                    frames.append(message)
                elif message.get('type') == 'scene_manifest':
                    manifests.append(message)
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            render_writers.discard(writer)
            writer.close()
            with contextlib.suppress(ConnectionError):
                await writer.wait_closed()

    hub.on_observation = on_observation
    await hub.start('127.0.0.1', 0)
    render_server = await asyncio.start_server(render_peer, '127.0.0.1', 0)
    render_port = render_server.sockets[0].getsockname()[1]
    catalog = args.catalog or args.adapter_root / 'Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_platform.catalog.json'
    command = [str(args.simulation_python), str(ROOT/'simulation/teleop_grasp_unreal.py'),
               '--adapter-root', str(args.adapter_root), '--model-root', str(ROOT/'model/SARM/platform'),
               '--catalog', str(catalog), '--control-port', str(hub.bound_port),
               '--render-port', str(render_port), '--duration', str(args.duration), '--simulation-rate', '1']
    process = None
    try:
        process = await asyncio.create_subprocess_exec(*command, cwd=ROOT,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=180)
        (args.output/'simulation.stdout.log').write_bytes(stdout)
        (args.output/'simulation.stderr.log').write_bytes(stderr)
        # Let queued final TCP frames reach the real Hub callback.
        await asyncio.sleep(.2)
        closed = recorder.stop(EpisodeStop(outcome='unknown'))
        (args.output/'observations.json').write_text(json.dumps(observations), encoding='utf-8')
        (args.output/'render.json').write_text(json.dumps({'manifests':manifests,'frames':frames}), encoding='utf-8')
        assert process.returncode == 0, stderr.decode(errors='replace')[-3000:]
        assert len(observations) >= int(args.duration * 28), f'Only {len(observations)} valid observations'
        assert closed['step_count'] == len(observations)
        assert len(frames) >= int(args.duration * 28)
        assert all(len(o['joint_position_rad']) == 8 for o in observations)
        assert all(o['attitude_control']['enabled'] for o in observations)
        assert observations[-1]['command_stale'], 'Onboard hold must survive stale teleoperation input'
        assert all(o['attitude_control']['reference_orientation_inertial_wxyz'] == observations[0]['attitude_control']['reference_orientation_inertial_wxyz'] for o in observations)
        arm_motion = max(abs(o['arm_joint_position_rad'][i]-observations[0]['arm_joint_position_rad'][i]) for o in observations for i in range(6))
        finger_motion = max(o['gripper_position_m'][0] for o in observations)-min(o['gripper_position_m'][0] for o in observations)
        assert arm_motion > .005, 'Control commands never moved the arm'
        assert finger_motion > .001, 'Control commands never moved the fingers'
        assert max(abs(v) for o in observations for v in o['reaction_wheels']['applied_motor_torque_nm']) <= .2000000001
        assert all(not any(o['reaction_wheels']['overspeed']) for o in observations)
        names = {o['object_id'] for o in manifests[-1]['objects']}
        assert {f'teleop/rw_{axis}' for axis in 'xyz'} <= names
        # Render and telemetry are sampled at different rates. Compare only
        # exactly equal physical timestamps, allowing quaternion sign reversal.
        by_time = {f['sim_time_ns']:f for f in frames}
        matched = 0
        for observation in observations:
            attitude = observation['attitude_control']
            frame = by_time.get(attitude['state_time_ns'])
            if frame is None:
                continue
            body = next(o for o in frame['objects'] if o['object_id']=='teleop/cubesat_bus')
            q, r = body['orientation_wxyz'], attitude['orientation_inertial_wxyz']
            error = min(sum((a-b)**2 for a,b in zip(q,r)),sum((a+b)**2 for a,b in zip(q,r)))
            assert error < 1e-14, (frame['sim_time_ns'], error)
            matched += 1
        assert matched > 10, f'No common-time authoritative pose comparisons: {matched}'
        summary = {'native_exit':process.returncode,'observations':len(observations),'render_frames':len(frames),
                   'recorded_steps':closed['step_count'],'matched_bus_attitudes':matched,
                   'arm_motion_rad':arm_motion,'finger_motion_m':finger_motion,
                   'max_attitude_error_deg':max(o['attitude_control']['attitude_error_angle_rad'] for o in observations)*180/math.pi,
                   'final_attitude_error_deg':observations[-1]['attitude_control']['attitude_error_angle_rad']*180/math.pi,
                   'final_command_stale':observations[-1]['command_stale'],
                   'gpu_video_tested':False,'episode_id':episode['episode_id']}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(json.dumps(summary,indent=2))
    finally:
        if process is not None and process.returncode is None:
            process.kill()  # Only the diagnostic subprocess created above.
            await process.wait()
        await hub.close()
        render_server.close()
        await render_server.wait_closed()
        for writer in list(render_writers):
            writer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adapter-root', required=True, type=Path)
    parser.add_argument('--simulation-python', required=True, type=Path)
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--duration', type=float, default=12.)
    parser.add_argument('--output', type=Path, default=ROOT/'run/attitude-control-validation')
    args = parser.parse_args()
    if args.duration < 10 or not math.isfinite(args.duration):
        parser.error('--duration must be finite and at least 10 seconds')
    asyncio.run(run(args))
