"""Example: estimate target spin from stereo images and own-ship gyro only."""
from space_arm_platform.benchmark_external import serve_method
from space_arm_platform.benchmark_vision_motion import VisionMotionContext, VisionMotionObservation
from space_arm_platform.benchmark_vision_motion_worker import ImageSpinMethod

raise SystemExit(serve_method(ImageSpinMethod(),VisionMotionContext,VisionMotionObservation))
