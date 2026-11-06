"""Replace the method class to evaluate your controller through the JSONL SDK."""
from pathlib import Path
import sys

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER / "backend"))
sys.path.insert(0, str(SERVER))

from space_arm_platform.benchmark_external import serve_method  # noqa: E402

if sys.argv[1:] == ["debris-avoidance"]:
    from simulation.benchmark_avoidance_methods import CatalogAvoidance
    from space_arm_platform.benchmark_avoidance import AvoidanceContext, AvoidanceObservation
    raise SystemExit(serve_method(CatalogAvoidance(), AvoidanceContext, AvoidanceObservation))
if sys.argv[1:] == ["pointing"]:
    from simulation.benchmark_pointing_methods import PointingWheelPD
    from space_arm_platform.benchmark_pointing import PointingContext, PointingObservation
    raise SystemExit(serve_method(PointingWheelPD(), PointingContext, PointingObservation))
if sys.argv[1:] == ["attitude"]:
    from simulation.benchmark_attitude_methods import AttitudeWheelPD
    from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation
    raise SystemExit(serve_method(AttitudeWheelPD(), AttitudeContext, AttitudeObservation))
if sys.argv[1:] == ["attitude-sequence"]:
    from simulation.benchmark_attitude_methods import SequenceAttitudePD
    from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation
    raise SystemExit(serve_method(SequenceAttitudePD(), AttitudeContext, AttitudeObservation))
if sys.argv[1:] == ["wheel-recovery"]:
    from simulation.benchmark_wheel_recovery_methods import AdaptiveWheelRecovery
    from space_arm_platform.benchmark_wheel_recovery import WheelRecoveryContext, WheelRecoveryObservation
    raise SystemExit(serve_method(AdaptiveWheelRecovery(), WheelRecoveryContext, WheelRecoveryObservation))
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
if sys.argv[1:] == ["flyaround"]:
    from simulation.benchmark_flyaround_methods import GNSSFlyAround
    from space_arm_platform.benchmark_flyaround import FlyAroundContext
    from space_arm_platform.benchmark_contract import NavigationObservation
    raise SystemExit(serve_method(GNSSFlyAround(), FlyAroundContext, NavigationObservation))
if sys.argv[1:] == ["formation"]:
    from simulation.benchmark_formation_methods import GNSSFormation
    from space_arm_platform.benchmark_formation import FormationContext, FormationObservation
    raise SystemExit(serve_method(GNSSFormation(), FormationContext, FormationObservation))
raise SystemExit("expected one task-family argument: attitude, attitude-sequence, wheel-recovery, debris-avoidance, pointing, waypoint, orbit, navigation, relative-flight, flyaround or formation")
