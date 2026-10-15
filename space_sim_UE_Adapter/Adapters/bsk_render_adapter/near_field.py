"""Display near-field evidence labels through the existing UE mission HUD.

No physics, image processing, or capture authority is assigned to the renderer.
"""


def publish_near_field_status(bridge, row):
    messages = [
        f"NEAR {row['phase']} | {row['navigation_source']}",
        f"COM {row['center_distance_m']:.3f}m | bus-grasp {row['platform_to_grasp_m']:.3f}m",
        f"TCP gap {row['tcp_gap_m']:.4f}m | lateral {row['lateral_error_m']:.4f}m",
        f"v {row['relative_speed_m_s']:.4f}m/s | angle {row['alignment_error_deg']:.2f}deg | sigma {row['position_sigma_m']:.4f}m",
        f"{row['capture_state']} | PARAMETRIC STEREO / LOGICAL CAPTURE / VIEW FILL",
    ]
    for message in messages:
        bridge.publish_event("near_field_status", {"message": message, "severity": "info"})
