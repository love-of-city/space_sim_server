"""Basilisk-only smoke check; run separately from standalone Python MuJoCo."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

from pathlib import Path
from tools.diagnostics.validate_satellite_mesh_collision import native_probe

if __name__ == '__main__':
    root = Path(__file__).resolve().parents[2]
    native_probe(root / 'model/SARM/platform', root / 'reports/task-box-native.json',
                 model_name='sarm_task_box_module.xml')
