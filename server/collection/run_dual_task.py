"""Dual dynamics implementation used by the existing scripted collection entry."""
from contextlib import nullcontext
import json
import time
from pathlib import Path


def run(args):
    from simulation.scripted_tasks.dual_floating_runtime import DualFloatingRuntime
    from simulation.scripted_tasks.dual_capture_transport import DualCaptureTransport
    from space_arm_platform.sampling import capture_preroll_ns
    if args.model is None or args.scene is None:
        raise ValueError('Dual collection requires its prepared model and scene')
    scene=json.loads(args.scene.read_text())
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    runtime=capture=None;started=time.monotonic();samples=None
    report=dict(task_name=args.task,result='failed',reason='initialization_incomplete')
    try:
        runtime=DualFloatingRuntime(args.model,scene)
        if args.native_geometry:
            for task in runtime.task.tasks.values():task.chain.enable_native_geometry()
        if args.capture_control_port:
            capture=DualCaptureTransport(runtime,control_port=args.capture_control_port,
                render_port=args.capture_render_port,catalog=args.catalog)
        runtime.initialize()
        task=runtime.task
        report.update(model=str(runtime.model),model_sha256=scene['collection_config']['dual_floating']['model_sha256'],
            render_model_path=str(runtime.model),render_model_sha256=scene['collection_config']['dual_floating']['model_sha256'],
            collection_config=scene['collection_config'],initial_state=dict(qpos=runtime.stepper.qpos.tolist(),qvel=runtime.stepper.qvel.tolist()))
        print(json.dumps(dict(type='initialization_complete',seconds=time.monotonic()-started)),flush=True)
        epoch=None if capture else int(args.warmup_sim_seconds*1e9)
        preroll=capture_preroll_ns(args.warmup_sim_seconds);tick=0
        trace_context=(output/'trace.jsonl').open('w') if args.save_trace else nullcontext(None)
        with trace_context as trace:
            while True:
                if capture:
                    capture.flush()
                    if args.capture_stop_file and args.capture_stop_file.exists():break
                    if task.result is not None and not capture.enabled and not capture.render.bridge.last_capture_state.get('capture_episode_id'):
                        time.sleep(.01);continue
                    if not capture.can_advance():
                        time.sleep(args.capture_backpressure_poll_s);continue
                stamp=tick*5_000_000
                if capture and epoch is None and stamp>=preroll and not capture.enabled:
                    time.sleep(.01);continue
                if capture and capture.enabled and epoch is None:
                    epoch=stamp;capture.task_start_ns=stamp
                elapsed=0. if epoch is None else (stamp-epoch)*1e-9
                samples=runtime.samples(stamp,elapsed)
                if epoch is not None and elapsed>=0 and task.result is None:
                    if capture and not capture.enabled:
                        task.result,task.reason='failed','capture_stopped_before_task_completed'
                    elif elapsed>=args.max_sim_seconds:
                        task.result,task.reason='failed','total_simulation_timeout'
                    else:
                        task.step(samples,right_preflight=runtime.recheck_right)
                    if capture:
                        capture.sequence+=1;capture.task_elapsed_s=elapsed
                if tick%4==0 and trace is not None:
                    trace.write(json.dumps(dict(sim_time_ns=str(stamp),**task.metrics(samples)))+'\n')
                if tick%1000==0:
                    print(json.dumps(dict(type='task_progress',sim_seconds=elapsed,stage=task.stage,
                        arms={s:t.stage for s,t in task.tasks.items()})),flush=True)
                if not capture and task.result is not None:break
                runtime.sim.ConfigureStopTime(stamp+5_000_000);runtime.sim.ExecuteSimulation();tick+=1
        report.update(result=task.result or 'failed',reason=task.reason or 'capture_stopped_before_task_completed',
            simulated_seconds=elapsed,task_seconds=elapsed,events=task.events,arms={s:dict(events=t.events,**t.metrics(samples[s])) for s,t in task.tasks.items()},
            right_recheck=runtime.right_recheck)
    except BaseException as error:
        report.update(result='failed',reason=f'{type(error).__name__}: {error}')
        raise
    finally:
        report['wall_seconds']=time.monotonic()-started
        (output/'summary.json').write_text(json.dumps(report,indent=2))
        if capture:capture.close()
        if runtime:runtime.close()
    return 0 if report["result"]=="success" else 1
