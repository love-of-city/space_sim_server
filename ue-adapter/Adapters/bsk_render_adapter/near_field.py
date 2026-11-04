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
    if "optical_depth_m" in row:
        checkpoint = "DONE" if row["optical_checkpoint_complete"] else "APPROACH / DWELL"
        messages.insert(2, f"Optical Z {row['optical_depth_m']:.2f}m | 17.4m checkpoint {checkpoint}")
    guidance = row.get('near_guidance')
    if guidance:
        node = guidance['nodes'][guidance['node_id']]
        # The existing UE HUD displays the last five events. Keep every new
        # release indicator inside that budget without changing UE assets.
        messages = [
            f"{row['phase']} | {guidance['subsegment']} | dwell {node['dwell_s']:.2f}s / wait {node['wait_s']:.2f}s",
            f"COM {row['center_distance_m']:.3f}m | Optical Z {row['optical_depth_m']:.2f}m | {guidance['node_id']}",
            f"GNSS + STEREO | VISUAL {'READY' if row.get('visual_ready') else 'NOT READY'} | v {row['relative_speed_m_s']:.4f}m/s | sigma {row['position_sigma_m']:.4f}m",
            f"TCP gap {row['tcp_gap_m']:.4f}m | lateral {row['lateral_error_m']:.4f}m | angle {row['alignment_error_deg']:.2f}deg",
            messages[-1],
        ]
    for message in messages:
        bridge.publish_event("near_field_status", {"message": message, "severity": "info"})
