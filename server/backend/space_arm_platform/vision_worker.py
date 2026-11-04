"""Isolated OpenCV process. Its stdin contract has pixels, never simulator state."""
import argparse
import base64
import json
from pathlib import Path
import sys

from .stereo_pose import StereoPoseEstimator
import cv2


def fuse_provided_detections(estimator, provided):
    observations = []
    for eye in ('left', 'right'):
        detections = provided[eye].get('detections', [])
        ids = [d['id'] for d in detections]
        if len(set(ids)) != len(ids) or provided[eye].get('status') == 'duplicate_ids':
            result = estimator._result()
            result['reason'] = 'provided frontend observed duplicate marker IDs'
            return result
        observations.append({d['id']: d['corners'] for d in detections})
    return estimator.estimate_from_detections(*observations)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text('utf-8-sig'))
    cv2.setNumThreads(1)
    estimator = StereoPoseEstimator(config['calibration'], config['layout'])
    from .provided_mono import ProvidedMonoEstimator
    mono = ProvidedMonoEstimator(config['calibration'], config['layout'],
        args.config.parent / 'vendor' / 'aruco_pnp_basler_mono_v1')
    for line in sys.stdin.buffer:
        try:
            request = json.loads(line)
            if set(request) != {'left_png', 'right_png'}:
                raise ValueError('worker accepts only left_png and right_png')
            left = base64.b64decode(request['left_png'], validate=True)
            right = base64.b64decode(request['right_png'], validate=True)
            provided = mono.estimate(left, right)
            result = fuse_provided_detections(estimator, provided)
            result['algorithm'] = 'supplied monocular frontend per eye + stereo geometry using the same image corners'
            result['provided_monocular'] = provided
            result['input_camera_ids'] = config['camera_ids']
        except Exception as error:
            result = {'status': 'error', 'reason': str(error)}
        sys.stdout.write(json.dumps(result, allow_nan=False, separators=(',', ':'))+'\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
