"""Same image-only estimator over explicitly selected JSONL v2 transport."""
from space_arm_platform.benchmark_external import IMAGE_PROTOCOL, serve_method
from space_arm_platform.benchmark_vision import VisionPoseContext, VisionPoseObservation
from space_arm_platform.benchmark_vision_worker import ImagePoseMethod

if __name__ == '__main__':
    raise SystemExit(serve_method(ImagePoseMethod(), VisionPoseContext, VisionPoseObservation,
                                 protocol=IMAGE_PROTOCOL))
