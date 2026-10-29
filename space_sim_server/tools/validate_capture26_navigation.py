"""Independent 400 Hz sensor/control diagnostic; never mission acceptance."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.capture26_runtime import main, run  # noqa: E402,F401

if __name__ == '__main__':
    main()
