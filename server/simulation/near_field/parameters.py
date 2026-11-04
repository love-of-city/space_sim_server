"""Near-only engineering defaults; no distant-flight or shared GNSS overrides."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class NearParameters:
    kp_s2: float = .06
    kd_s: float = .6
    force_limit_n: float = 25.
    acceleration_m_s2: float = .002
    jerk_m_s3: float = .002
    braking_m_s2: float = .005
    dwell_s: float = 5.
    acquire_timeout_s: float = 600.
    capture_timeout_s: float = 300.
    node_timeout_s: float = 300.
    visual_timeout_s: float = 60.
    duration_s: float = 3600.
    visual_age_s: float = .35
    visual_samples: int = 10
    marker_px: float = 24.
    depth_sigma_m: float = .03
    visual_sigma_m: float = .04
    gnss_sigma_m: float = .05
    speed_m_s: float = .002
    acquire_speed_m_s: float = .02
    cruise_speed_m_s: float = .10
    close_speed_m_s: float = .02
    offset_speed_m_s: float = .0047
    offset_acceleration_m_s2: float = .0018
    acquire_transverse_m: float = .10
    vbar50_tolerance_m: float = .10
    optical17_tolerance_m: float = .05
    visual_tolerance_m: float = .02
    platform_tolerance_m: float = .006
    corridor_min_m: float = .10
    corridor_max_m: float = 1.
    corridor_slope: float = .01
    max_update_gap_s: float = .15
    boundary_margin_m: float = .05
    execution_delay_s: float = .05
    vbar50_m: float = 50.
    optical17_m: float = 17.4
    visual_depth_m: float = 5.5
    offset_end_depth_m: float = 2.
    vision_boundary_m: float = 12.
    ready_boundary_m: float = 5.

    def metadata(self):
        return asdict(self)


PARAMETERS = NearParameters()
