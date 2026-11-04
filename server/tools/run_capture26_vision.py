"""Independent image ingress/solve process for native closed-loop validation.

Uses the production capture protocol and image worker. Control receives bounded
UDP pose packets only; image serialization never runs on the physics GIL.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'backend'))


def main():
    from space_arm_platform.capture_receiver import CaptureReceiver
    from space_arm_platform.vision_service import StereoVisionService
    from space_arm_platform.vision_channel import VisionResultSender

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--mission', required=True)
    p.add_argument('--session', required=True)
    p.add_argument('--result-port', type=int, required=True)
    p.add_argument('--capture-port', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--save-all-images', action='store_true')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    sender = VisionResultSender(args.result_port)
    vision = StereoVisionService(ROOT, config_path=args.config, on_result=sender)
    vision.bind_task(args.mission, args.session)
    vision.evidence_dir = args.output/'images'
    vision.evidence_all = args.save_all_images
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    capture = CaptureReceiver('127.0.0.1', args.capture_port, lambda *_: None,
                              on_vision_capture=vision.on_capture)
    try:
        vision.warmup()
        vision.start()
        capture.start()
        (args.output/'vision-ready.json').write_text(json.dumps(dict(ready=True)), encoding='utf-8')
        while not stop.wait(.5):
            if (args.output/'vision-stop').exists():
                break
            if (args.output/'vision-worker-stop').exists():
                vision.close()
            # A separate status file is diagnostic only, never a control input.
            destination = args.output/'vision-status.json'
            temporary = destination.with_suffix('.tmp')
            temporary.write_text(json.dumps(vision.status()), encoding='utf-8')
            temporary.replace(destination)
    finally:
        capture.close()
        vision.close()
        sender.close()


if __name__ == '__main__':
    main()
