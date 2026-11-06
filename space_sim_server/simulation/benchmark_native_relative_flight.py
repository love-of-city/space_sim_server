"""Same real two-spacecraft scene, with explicit task-specific grading contracts."""
from simulation.benchmark_native_flight import NativeWaypointEnvironment
from space_arm_platform.benchmark_relative_flight import RelativeFlightContext
from space_arm_platform.benchmark_relative_flight_score import RelativeFlightScorer


class NativeRelativeFlightEnvironment(NativeWaypointEnvironment):
    context_type = RelativeFlightContext
    scorer_type = RelativeFlightScorer
