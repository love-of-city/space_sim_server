"""Public image-only example. OpenCV runs outside the native dynamics process."""
from space_arm_platform.benchmark_external import serve_method
from space_arm_platform.benchmark_vision import VisionPoseContext, VisionPoseObservation
from space_arm_platform.benchmark_vision_worker import ImagePoseMethod

if __name__ == '__main__':
    raise SystemExit(serve_method(ImagePoseMethod(), VisionPoseContext, VisionPoseObservation))
