"""Example: drive actual body-axis motors using only current and past images."""
from space_arm_platform.benchmark_external import serve_method
from space_arm_platform.benchmark_vision_approach import VisionApproachContext,VisionApproachObservation
from space_arm_platform.benchmark_vision_approach_worker import ImageApproachMethod

raise SystemExit(serve_method(ImageApproachMethod(),VisionApproachContext,VisionApproachObservation))
