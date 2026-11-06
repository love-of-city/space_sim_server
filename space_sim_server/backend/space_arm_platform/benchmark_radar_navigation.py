"""Noncooperative target orbit estimation from simulated active ranging, without target telemetry."""
import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .benchmark_attitude import Quaternion, unit_quaternion
from .benchmark_contract import Contract, Number, Positive, Vector3, WaypointInitialState
from .benchmark_navigation import (
    GNSSMeasurements, NavigationContext, NavigationTask, RelativeNavigationSafety, validate_navigation_physics,
)


class RadarInitialState(WaypointInitialState):
    target_altitude_half_width_m: Positive = Field(le=10000)
    target_inclination_half_width_deg: Positive = Field(le=5)
    target_anomaly_half_width_deg: Positive = Field(le=10)


class RadarSensorModel(Contract):
    kind: Literal['idealized_monostatic_range_bearing_doppler']
    position_sigma_m: Positive
    velocity_sigma_m_s: Positive
    attitude_sigma_rad: Positive = Field(le=.01)
    range_sigma_m: Positive
    bearing_sigma_rad: Positive = Field(le=.01)
    range_rate_sigma_m_s: Positive
    minimum_range_m: Positive
    maximum_range_m: Positive
    delay_s: Literal[0.0]
    target_link: Literal['none']
    target_reference_point: Literal['idealized_target_assembly_com_scatterer']
    observer_reference_point: Literal['assembled_center_of_mass']
    bearing_frame: Literal['observer_body_axes']
    coverage: Literal['idealized_omnidirectional']

    @field_validator('delay_s', mode='before')
    @classmethod
    def numeric_delay(cls, value):
        if type(value) not in (float, int):
            raise ValueError('sensor delay must be numeric')
        return value


class RadarNavigationTask(NavigationTask):
    schema_version: Literal['spacesim-autonomy-radar-navigation/1']
    source_id: Literal['D02']
    requirement_id: Literal['ssb.relative-navigation.noncooperative']
    backend: Literal['native_team_radar_estimation_v1']
    spacecraft_count: Literal[2]
    target_cooperation: Literal['N']
    reference_frame: Literal['target_Earth_centered_J2000']
    initial: RadarInitialState
    safety: RelativeNavigationSafety
    observation: RadarSensorModel

    @model_validator(mode='after')
    def semantic_contract(self):
        validate_navigation_physics(self)
        initial, sensor = self.initial, self.observation
        if not 200000 <= initial.target_altitude_m-initial.target_altitude_half_width_m:
            raise ValueError('all target orbits must start above the modeled atmosphere floor')
        if not 0 < initial.inclination_deg-initial.target_inclination_half_width_deg < initial.inclination_deg+initial.target_inclination_half_width_deg < 180:
            raise ValueError('all target inclinations must lie strictly inside (0, 180) degrees')
        if not sensor.minimum_range_m < self.safety.minimum_range_m < self.safety.maximum_range_m < sensor.maximum_range_m:
            raise ValueError('the idealized ranging envelope must cover the entire safety shell')
        return self


class RadarObservation(GNSSMeasurements):
    schema_version: Literal['spacesim-autonomy-radar-observation/1'] = 'spacesim-autonomy-radar-observation/1'
    target_measurement_j2000: None
    orientation_body_to_j2000_wxyz: Quaternion
    range_m: Positive
    direction_body_unit: Vector3
    range_rate_m_s: Number

    _normalized = field_validator('orientation_body_to_j2000_wxyz')(unit_quaternion)

    @model_validator(mode='after')
    def valid_sensor_packet(self):
        if self.delivery_time_ns != self.sample_time_ns:
            raise ValueError('this ranging sensor declares zero delivery latency')
        if not math.isclose(math.dist(self.direction_body_unit, (0, 0, 0)), 1., abs_tol=1e-8, rel_tol=0):
            raise ValueError('range bearing must be a unit vector in observer body axes')
        return self


class RadarDelivery(Contract):
    record_type: Literal['radar_delivery'] = 'radar_delivery'
    observation: RadarObservation


class RadarNavigationContext(NavigationContext):
    task: RadarNavigationTask
