"""Run the existing monocular ArUco + PnP with camera.json intrinsics."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'source'))
from space_arm_platform.fiducials import FiducialEstimator
from fiducial_center_metrics import point_in_gripper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path, help='Original 1920x1200 image')
    parser.add_argument('--output', type=Path, help='Output JSON; default: stdout')
    args = parser.parse_args()
    camera = json.loads((ROOT/'camera.json').read_text(encoding='utf-8'))
    # imdecode supports Windows paths containing Chinese characters.
    image = cv2.imdecode(np.frombuffer(args.image.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        parser.error('Cannot decode the supplied image')
    if list(image.shape[1::-1]) != camera['resolution_wh']:
        parser.error('Expected original 1920x1200 image matching camera.json; do not resize old camera images')
    result = FiducialEstimator().estimate(image, np.asarray(camera['K'], dtype=float))
    result.update(calibration_version=camera['intrinsics_version'],
                  calibration_status=camera['calibration_status'],
                  source_image=str(args.image), mode='monocular',
                  exposure_sim_time_ns=None, velocity_m_s=None,
                  navigation_for_control={'valid': False, 'control_ready': False,
                      'reason': 'navigation frame conversion, timestamps and filtering pending'})
    result['legacy_tool_measurement'] = None
    if result['pose'] is not None:
        reference = json.loads((ROOT/'legacy_reference.json').read_text(encoding='utf-8'))
        pose = result['pose']
        center = point_in_gripper(pose['rotation_camera_from_target'], pose['translation_m'], reference)
        result['legacy_tool_measurement'] = {
            'frame': 'current_sarm_ee', 'separation_center_in_gripper_m': center.tolist(),
            'distance_m': float(np.linalg.norm(center)),
            'scope': 'Existing SARM mount only; replace extrinsics/tool center for a new arm'}
    serialized = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(serialized, encoding='utf-8')
    else:
        print(serialized, end='')


if __name__ == '__main__':
    main()
