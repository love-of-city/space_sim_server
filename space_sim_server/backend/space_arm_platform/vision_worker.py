"""Isolated OpenCV process. Its stdin contract has pixels, never simulator state."""
import argparse
import base64
import json
from pathlib import Path
import sys
import struct

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


def read_pairs(stream, binary=False):
    if not binary:
        for line in stream:
            request = json.loads(line)
            if set(request) != {'left_png', 'right_png'}:
                raise ValueError('worker accepts only left_png and right_png')
            yield (base64.b64decode(request['left_png'], validate=True),
                   base64.b64decode(request['right_png'], validate=True))
        return
    while True:
        header = stream.read(8)
        if not header:
            return
        if len(header) != 8:
            raise ValueError('truncated image packet header')
        sizes = struct.unpack('!II', header)
        if max(sizes) > 64*1024*1024:
            raise ValueError('image packet exceeds 64 MiB per eye')
        images = []
        for size in sizes:
            image = stream.read(size)
            if len(image) != size:
                raise ValueError('truncated image packet body')
            images.append(image)
        yield images


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--monocular-diagnostics', action='store_true')
    parser.add_argument('--binary-input', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text('utf-8-sig'))
    cv2.setNumThreads(1)
    estimator = StereoPoseEstimator(config['calibration'], config['layout'], **config.get('estimator', {}))
    mono = None
    if args.monocular_diagnostics:
        from .provided_mono import ProvidedMonoEstimator
        mono = ProvidedMonoEstimator(config['calibration'], config['layout'],
            args.config.parent / 'vendor' / 'aruco_pnp_basler_mono_v1')
    for left, right in read_pairs(sys.stdin.buffer, args.binary_input):
        try:
            if not left and not right:
                result = {'status': 'ready', 'reason': 'image solver and calibration loaded'}
            elif mono is None:
                result = estimator.estimate(left, right)
                result['algorithm'] = 'one ArUco detection per eye + joint calibrated stereo pose'
            else:
                provided = mono.estimate(left, right)
                result = fuse_provided_detections(estimator, provided)
                result['provided_monocular'] = provided
                result['algorithm'] = 'diagnostic supplied mono frontend + shared-corner stereo'
            result['input_camera_ids'] = config['camera_ids']
        except Exception as error:
            result = {'status': 'error', 'reason': str(error)}
        sys.stdout.write(json.dumps(result, allow_nan=False, separators=(',', ':'))+'\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
