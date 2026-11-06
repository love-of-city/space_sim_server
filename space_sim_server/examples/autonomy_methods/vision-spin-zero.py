"""Negative control: real image pose estimates but zero target angular rate."""
from space_arm_platform.benchmark_external import serve_method
from space_arm_platform.benchmark_vision_motion import VisionMotionContext, VisionMotionObservation
from space_arm_platform.benchmark_vision_motion_worker import ImageSpinMethod

raise SystemExit(serve_method(ImageSpinMethod(zero_rate=True),VisionMotionContext,VisionMotionObservation))
