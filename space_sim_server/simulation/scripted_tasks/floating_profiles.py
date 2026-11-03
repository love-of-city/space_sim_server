"""Small, explicit collection conditions; task identity stays independent."""
from copy import deepcopy


FLOATING_PROFILES = {
    "left-static-red": dict(
        release=dict(min_distance_m=.25, max_distance_m=.40, view_x_range=[-.24, -.16],
                     min_tilt_deg=10., max_tilt_deg=15., tilt_axis="camera_y",
                     velocity_direction="stationary", max_speed_m_s=0., max_spin_rad_s=0.),
        target=dict(color_choices=["red"]), sunlight=.65, phase=45.),
    "right-incoming-blue": dict(
        release=dict(min_distance_m=.40, max_distance_m=.55, view_x_range=[.16, .24],
                     min_tilt_deg=10., max_tilt_deg=15., tilt_axis="camera_x",
                     velocity_direction="approaching", min_speed_m_s=.0015, max_speed_m_s=.002,
                     min_spin_rad_s=.004, max_spin_rad_s=.006, spin_axis="camera_z", arm_half_range_deg=3.),
        target=dict(color_choices=["blue"], mass_scale_range=[1.2, 1.2], friction_scale_range=[1.2, 1.2]),
        sunlight=1.35, phase=135.),
    "left-receding-green": dict(
        release=dict(min_distance_m=.35, max_distance_m=.50, view_x_range=[-.16, -.10],
                     min_tilt_deg=8., max_tilt_deg=12., velocity_direction="receding",
                     min_speed_m_s=.001, max_speed_m_s=.0015, min_spin_rad_s=.004,
                     max_spin_rad_s=.006, spin_axis="camera_x", spin_sign=-1, arm_half_range_deg=3.),
        target=dict(color_choices=["green"], mass_scale_range=[.8, .8], friction_scale_range=[.8, .8]),
        sunlight=.9, phase=225.),
    "right-lateral-orange": dict(
        release=dict(min_distance_m=.45, max_distance_m=.60, view_x_range=[.10, .20],
                     velocity_direction="lateral_right", min_speed_m_s=.001, max_speed_m_s=.002,
                     min_spin_rad_s=.004, max_spin_rad_s=.006, spin_axis="camera_y", arm_half_range_deg=3.),
        target=dict(color_choices=["orange"]), sunlight=1.1, phase=315.),
    "upper-drift-purple": dict(
        release=dict(min_distance_m=.30, max_distance_m=.50, view_y_range=[.12, .20],
                     velocity_direction="up", min_speed_m_s=.001, max_speed_m_s=.002,
                     min_spin_rad_s=.004, max_spin_rad_s=.006, spin_axis="camera_z", spin_sign=-1),
        target=dict(color_choices=["purple"]), sunlight=1., phase=90.),
    "lower-drift-yellow": dict(
        release=dict(min_distance_m=.30, max_distance_m=.50, view_y_range=[-.20, -.12],
                     velocity_direction="down", min_speed_m_s=.001, max_speed_m_s=.002,
                     min_spin_rad_s=.004, max_spin_rad_s=.006, spin_axis="camera_y", spin_sign=-1),
        target=dict(color_choices=["yellow"]), sunlight=1., phase=270.),
    "noisy-observations": dict(
        release=dict(min_distance_m=.30, max_distance_m=.50), target={}, sunlight=1., phase=0.,
        randomization=dict(joint_observation_noise=dict(enabled=True), joint_bias=dict(enabled=True),
                           wrist_wrench_noise=dict(enabled=True), observation_delay=dict(enabled=True, delay_ms=20))),
    "action-perturbation": dict(
        release=dict(min_distance_m=.30, max_distance_m=.50), target={}, sunlight=1., phase=180.,
        randomization=dict(action_noise=dict(enabled=True, std_deg=.01, correlation_s=.3),
                           lighting=dict(enabled=True, variation_pct=20.))),
}


def apply_floating_profile(source, name):
    """Apply a complete condition to a copied source scene before sampling."""
    profile = deepcopy(FLOATING_PROFILES[name])
    scene = deepcopy(source)
    scene["floating_capture"] = profile["release"]
    scene["floating_target"] = profile["target"]
    scene["collection_randomization"] = profile.get("randomization", {})
    scene["randomize_orbit_phase"] = False
    scene.setdefault("environment", {}).setdefault("orbit", {})["true_anomaly_deg"] = profile["phase"]
    scene["environment"].setdefault("lighting", {})["sunlight_intensity_scale"] = profile["sunlight"]
    scene["floating_profile"] = dict(schema="floating-profile/1", name=name, definition=profile)
    return scene
