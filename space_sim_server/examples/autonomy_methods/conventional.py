"""Replace the method class to evaluate your controller through the JSONL SDK."""
from pathlib import Path
import sys

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER / "backend"))
sys.path.insert(0, str(SERVER))

from space_arm_platform.benchmark_external import serve_method  # noqa: E402

if sys.argv[1:] == ["attitude"]:
    from simulation.benchmark_attitude_methods import AttitudeWheelPD
    from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation
    raise SystemExit(serve_method(AttitudeWheelPD(), AttitudeContext, AttitudeObservation))
if sys.argv[1:] == ["waypoint"]:
    from simulation.benchmark_methods import GNSSWaypointPD
    from space_arm_platform.benchmark_contract import MethodContext, NavigationObservation
    raise SystemExit(serve_method(GNSSWaypointPD(), MethodContext, NavigationObservation))
if sys.argv[1:] == ["orbit"]:
    from simulation.benchmark_orbit_methods import GNSSOrbitManeuver
    from space_arm_platform.benchmark_orbit import OrbitContext, OrbitObservation
    raise SystemExit(serve_method(GNSSOrbitManeuver(), OrbitContext, OrbitObservation))
if sys.argv[1:] == ["navigation"]:
    from simulation.benchmark_navigation_methods import GNSSOrbitEstimator
    from space_arm_platform.benchmark_navigation import NavigationContext, GNSSMeasurements
    raise SystemExit(serve_method(GNSSOrbitEstimator(), NavigationContext, GNSSMeasurements))
if sys.argv[1:] == ["relative-flight"]:
    from simulation.benchmark_relative_flight_methods import GNSSRelativeFlight
    from space_arm_platform.benchmark_relative_flight import RelativeFlightContext
    from space_arm_platform.benchmark_contract import NavigationObservation
    raise SystemExit(serve_method(GNSSRelativeFlight(), RelativeFlightContext, NavigationObservation))
raise SystemExit("expected one task-family argument: attitude, waypoint, orbit, navigation or relative-flight")
