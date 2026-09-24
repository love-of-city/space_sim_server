"""Basilisk-only smoke check; run separately from standalone Python MuJoCo."""
from pathlib import Path
from validate_satellite_mesh_collision import native_probe

if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    native_probe(root / 'model/SARM/platform', root / 'reports/task-box-native.json',
                 model_name='sarm_task_box_module.xml')
