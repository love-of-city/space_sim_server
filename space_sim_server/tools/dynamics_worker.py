"""Private persistent dynamics entry point; controlled by collect_scripted_tasks."""
import argparse
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
    # Imported once, but run() creates a new native scenario module and graph.
    from tools.run_scripted_task import main as run_episode
    serve(args.mailbox, dict(pid=args.parent_pid, created=args.parent_created),
          lambda argv: run_episode(argv=argv),
          max_episodes=args.max_episodes, max_rss_mb=args.max_rss_mb)


if __name__ == '__main__':
    main()
