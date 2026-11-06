"""Environment-side measurement model; no target telemetry or filtered state is emitted."""
import numpy as np
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_radar_navigation import RadarObservation


def measure_radar(sensor, target, observer, rotation_body_to_j2000, rng, *, episode_id, sequence, nanos):
    target, observer = np.asarray(target), np.asarray(observer)
    displacement = target[:3]-observer[:3]
    distance = float(np.linalg.norm(displacement))
    if not sensor.minimum_range_m <= distance <= sensor.maximum_range_m:
        raise ValueError('target is outside the declared ranging envelope')
    direction = displacement/distance
    rotation = np.asarray(rotation_body_to_j2000)
    own_noise = np.array([sensor.position_sigma_m]*3+[sensor.velocity_sigma_m_s]*3)
    own_state = observer+rng.normal(size=6)*own_noise
    orientation = Rotation.from_matrix(rotation)*Rotation.from_rotvec(rng.normal(size=3)*sensor.attitude_sigma_rad)
    bearing = Rotation.from_rotvec(rng.normal(size=3)*sensor.bearing_sigma_rad).apply(rotation.T@direction)
    return RadarObservation(episode_id=episode_id, sequence=sequence, sample_time_ns=nanos, delivery_time_ns=nanos,
        chaser_measurement_j2000=tuple(own_state), target_measurement_j2000=None,
        orientation_body_to_j2000_wxyz=tuple(orientation.as_quat(scalar_first=True)),
        direction_body_unit=tuple(bearing), range_m=distance+float(rng.normal())*sensor.range_sigma_m,
        range_rate_m_s=float(direction@(target[3:]-observer[3:]))+float(rng.normal())*sensor.range_rate_sigma_m_s)
