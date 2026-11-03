"""Assemble the private persistent dynamics process used by collection.

The backend dynamics_worker owns requests, heartbeats and parent/process
management. This executable owns physics graph creation, reuse and final close;
its two names describe separate process boundaries rather than duplicate logic.
"""
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]

from space_arm_platform.dynamics_worker import serve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mailbox', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--parent-created', type=float, required=True)
    parser.add_argument('--max-episodes', type=int, default=4)
    parser.add_argument('--max-rss-mb', type=int, default=8192)
    args = parser.parse_args()
    # Basilisk creates a tqdm even with progress disabled. Its lazy monitor is
    # process-global and otherwise outlives the first episode. Batch workers
    # do not need terminal progress refresh; keep leak checks strict for every
    # actual session/transport thread instead of whitelisting daemon threads.
    from tqdm import tqdm
    tqdm.monitor_interval = 0
    # Optimized private collision queries and live separation proofs. Explicit
    # overrides support controlled baseline/shadow verification runs.
    for name in ('CYLINDER_FILTER_MODE', 'MESH_FILTER_MODE', 'THREAD_FILTER_MODE'):
        os.environ.setdefault(name, '2')
    from collection.run_scripted_task import main as run_episode
    from simulation.scripted_tasks.physics_graph import PhysicsGraphCache
    cache = PhysicsGraphCache() if os.environ.get('SPACE_SIM_REUSE_GRAPH', '1') == '1' else None
    try:
        serve(args.mailbox, dict(pid=args.parent_pid, created=args.parent_created),
              lambda argv: run_episode(argv=argv, graph_cache=cache),
              max_episodes=args.max_episodes, max_rss_mb=args.max_rss_mb)
    finally:
        if cache is not None:
            cache.close()



if __name__ == '__main__':
    main()
