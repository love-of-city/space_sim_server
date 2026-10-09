"""Separate-process native physics/controller comparison; no UE/browser timing."""
import argparse
from pathlib import Path
from validate_satellite_mesh_collision import native_probe

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=['sarm_ground_target_self_collision.xml',
        'sarm_task_box_module.xml', 'sarm_task_box_plugs.xml'], required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    moving = args.model == 'sarm_task_box_plugs.xml'
    native_probe(root/'model/SARM/platform', root/'reports'/('native-benchmark-'+args.model+'.json'),
        model_name=args.model, duration=2., expected_bodies=17 if moving else 15,
        expected_qpos=40 if moving else 26)
