"""Record fresh platform runtime windows, excluding previous/startup samples.

Start only after asset import and warm-up finish. This is simulation/render
dispatch throughput, not browser input-to-photon latency or decoded video FPS.
"""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--windows', type=int, default=6)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--warmup-sim-seconds', type=float, default=0)
    args = parser.parse_args()
    if args.windows < 1 or args.timeout <= 0:
        parser.error('windows and timeout must be positive')
    rows = []
    deadline = time.monotonic()+args.timeout
    with args.log.open(encoding='utf-8') as stream:
        stream.seek(0, 2)
        while len(rows) < args.windows:
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Only {len(rows)} fresh runtime windows received')
            before = stream.tell()
            line = stream.readline()
            if not line.endswith('\n'):
                stream.seek(before)
                time.sleep(.2)
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if (row.get('type') == 'runtime_performance'
                    and int(row['sim_time_ns'])*1e-9 >= args.warmup_sim_seconds):
                rows.append(row)
    wall = sum(r['window_wall_s'] for r in rows)
    simulated = sum(r['window_simulated_s'] for r in rows)
    result = dict(scope=__doc__, log=args.log.name, windows=rows,
        recorded_at_unix_s=time.time(), wall_s=wall, simulated_s=simulated,
        wall_real_time_factor=simulated/wall,
        processing_real_time_factor=simulated/sum(r['simulation_execute_s']+r['observation_send_s'] for r in rows),
        render_dispatch_hz=sum(r['outer_frames'] for r in rows)/wall,
        max_execute_frame_ms=max(r['max_execute_frame_ms'] for r in rows),
        max_render_backlog=max(r['render_backlog_frames'] for r in rows))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'windows'}, indent=2))


if __name__ == '__main__':
    main()
