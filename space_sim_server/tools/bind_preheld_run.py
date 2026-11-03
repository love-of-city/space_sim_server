"""Bind an already completed native run; never edit or rerate its evidence."""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.scene_library.manifest import digest, verify_files, write_json
from simulation.scene_library.status import phase_one_evidence


def bind(instance_path, native, hold_path, stdout, output):
    def read(path):
        return json.loads(Path(path).read_text(encoding='utf-8-sig'))
    instance = read(instance_path)
    verify_files(instance)
    fixture = instance['operation_fixture']
    if fixture['scope'] != 'dual_arm_preheld_operation':
        raise ValueError('Only dual-arm preheld operation receipts may be bound here')
    report_path = native / 'acceptance.json'
    report, summary, launch = read(report_path), read(native / 'run/summary.json'), read(native / 'launch.json')
    argv = launch['argv']
    scene = Path(argv[argv.index('--scene') + 1]).resolve()
    if (report.get('native_returncode') != 0 or report.get('accepted') is not True
            or summary['task_name'] != instance['recipe']['task']
            or summary['model_sha256'] != digest(instance['model'])
            or read(scene) != instance['initial_scene']
            or launch['source_sha256'].get(str(scene)) != digest(scene)):
        raise ValueError('Native run does not match the instance or did not pass')
    for key in ('transport_clearance_m', 'seating_overtravel_m', 'release_speed_rad_s'):
        if fixture.get(key) is not None and summary['config'].get(key) != fixture[key]:
            raise ValueError(f'Native controller option mismatch: {key}')
    receipt = dict(schema='space-arm-scene-run/1', scene_id=instance['recipe']['id'],
        source='bound_completed_native_run', status='operation_passed', accepted=False,
        operation_accepted=True, returncode=report['native_returncode'],
        evidence_scope=fixture['scope'], native_report=str(report_path.resolve()),
        native_report_sha256=digest(report_path), preheld_capture_validation=str(hold_path.resolve()),
        preheld_capture_validation_sha256=digest(hold_path), resolved_initial_scene=str(scene),
        physical_docking_accepted=False, friction_only_capture_accepted=False,
        ue_visual_verified=False, linux_verified=False)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / 'instance.json', instance)
    if not phase_one_evidence(output / 'receipt.json', receipt, {}, instance['recipe']):
        raise ValueError('The bound native, geometry and full-interval holding gates did not all pass')
    shutil.copyfile(native / 'launch.json', output / 'launch.json')
    shutil.copyfile(stdout, output / 'stdout.log')
    write_json(output / 'receipt.json', receipt)
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('instance', 'native', 'hold', 'stdout', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(bind(args.instance, args.native, args.hold, args.stdout, args.output)))
