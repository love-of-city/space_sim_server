"""Daily entry: collect enabled tasks sequentially on each shuffled layout."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]

from collection.sequence import main

if __name__ == '__main__':
    raise SystemExit(main())
