"""Read-only comparison of the current V-bar and near23 capture contracts.

An accepted NearParked document is an optional input, not a capture permit.
No initial conditions, mission policies, navigation or physical state are edited.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]

from simulation.capture26_demo_geometry import load_geometry
from simulation.capture26_mission import MODEL_PATH, load_mission, load_vision
from simulation.end_stereo_pointing import EndStereoGeometry
from simulation.near23_capture import CONFIG
from space_arm_platform.visual_navigation import calibration_id


def inspect(root, handoff_path=None):
    root = Path(root).resolve()
    mission = load_mission(root)
    vision = load_vision(root)
    near = json.loads((root/CONFIG).read_text('utf-8-sig'))
    EndStereoGeometry(root/MODEL_PATH, near)  # Check both rendered/calibrated mounts.
    geometry = load_geometry(root, mission) if mission.is_demo else None
    accepted = None
    if handoff_path is not None:
        from simulation.capture26_handoff import load
        document = load(handoff_path, root)
        if document['phase'] != 'NearParked':
            raise ValueError('terminal capture requires a NearParked document, not FarReady')
        accepted = dict(phase=document['phase'], time_ns=document['time_ns'],
                        mission_id=document['mission_id'], accepted=document['accepted'],
                        source_sha256=document['source_sha256'])
    near_reference = np.asarray(near['marker_rig']['grasp_center_target_m'])
    return dict(
        scope='static integration inspection; not a flown handoff or capture acceptance',
        automatic_capture_connected=False, accepted_handoff_inspected=accepted,
        model_sha256=hashlib.sha256((root/MODEL_PATH).read_bytes()).hexdigest(),
        vbar=dict(acceptance_profile=mission.acceptance_profile, feedback_source=mission.feedback_source,
                  calibration_id=calibration_id(vision), target_attitude_control='passive',
                  terminal_gap_m=mission.terminal_gap_m,
                  surface_point_target_m=geometry['surface_point_target_m'] if geometry else None,
                  lateral_target_m=geometry['lateral_target_m'] if geometry else None),
        near23=dict(config=CONFIG, sha256=hashlib.sha256((root/CONFIG).read_bytes()).hexdigest(),
                    mount_parent=near['mount_parent_id'],
                    camera_positions_parent_m={eye:near['parent_from_optical'][eye]['position_parent_m']
                                               for eye in ('left', 'right')},
                    grasp_reference_target_m=near_reference.tolist(),
                    requires_held_target_prediction=True),
        surface_minus_grasp_reference_target_m=(
            (np.asarray(geometry['surface_point_target_m'])-near_reference).tolist() if geometry else None),
        existing_continuation='capture26_operations.run_scene -> capture26_runtime.run; restores physical/navigation/sensors',
        blockers=[
            'Operations runtime does not install the near23 automatic capture profile.',
            'NearParked is a folded, laterally displaced platform reference; deploy and align the arm through validated motion.',
            'Agree target attitude ownership: V-bar is passive; near23 prediction assumes a held target before contact.',
            'Keep the navigation body-camera calibration separate from the moving gripper stereo calibration.',
            'Validate image readiness, contact and force hold after an actual accepted NearParked continuation.',
        ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--handoff', type=Path, help='Optional actual accepted NearParked handoff')
    parser.add_argument('--output', type=Path, help='New JSON report; existing files are never overwritten')
    args = parser.parse_args()
    report = inspect(args.root, args.handoff)
    content = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            stream.write(content+'\n')
    print(content)


if __name__ == '__main__':
    main()
