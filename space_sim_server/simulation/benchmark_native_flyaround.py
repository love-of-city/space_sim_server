"""Complete flyaround using the original two-spacecraft native flight scene."""
from simulation.benchmark_native_flight import NativeWaypointEnvironment
from space_arm_platform.benchmark_flyaround import FlyAroundContext
from space_arm_platform.benchmark_flyaround_score import FlyAroundScorer


class NativeFlyAroundEnvironment(NativeWaypointEnvironment):
    context_type = FlyAroundContext
    scorer_type = FlyAroundScorer
