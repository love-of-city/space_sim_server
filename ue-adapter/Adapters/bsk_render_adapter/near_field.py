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


def publish_capture26_status(bridge, status):
    """Image-navigation probe/flight HUD; values originate in control telemetry."""
    if status.get('acceptance_profile') == 'simulation_state_demo_v1':
        node = status.get('node', status['phase'])
        labels = dict(FarHandoff110m='110 m handoff', NearAcquire='Brake and align COM V-bar',
            Vbar50m='50 m COM V-bar', Optical17p4m='17.4 m optical parking',
            VisualReady5p5m='5.50 m optical parking', PlatformStandoff='TCP 0.5 m working position')
        working = node == 'PlatformStandoff'
        centre = status.get('center_distance_m')
        centre_text = '---' if centre is None else f'{centre:.3f}m'
        gap = status.get('tcp_gap_m')
        gap_text = '---' if gap is None else f'{gap:.3f}m'
        position = status.get('position_error_m')
        speed = status.get('velocity_error_m_s')
        p_text = '---' if position is None else f'{position:.4f}m'
        v_text = '---' if speed is None else f'{speed:.4f}m/s'
        messages = [
            f"CAPTURE26 {labels.get(node, node)} | {'PARKING' if status.get('at_node') else 'APPROACH'} | STATE FEEDBACK DEMO",
            f"COM distance {centre_text} | Optical depth {status['optical_depth_m']:.3f}m | TCP axial gap {gap_text}",
            f"POSITION {'OK' if status.get('position_qualified') else 'TRACKING'} {p_text} | VELOCITY {'OK' if status.get('velocity_qualified') else 'BRAKING'} {v_text}",
            f"CONTINUOUS DWELL {min(5., status.get('dwell_s', 0.)):.1f} / 5.0s | REMAINING {status.get('dwell_remaining_s', 5.):.1f}s",
            'SAFE HOLD: '+status.get('reason', 'approach stopped') if status.get('phase') == 'SafeHold'
            else ('WORKING POSITION HOLD' if working else 'FAR HANDOFF HOLD COMPLETE') if status.get('released')
            else 'COMPLETED: '+(', '.join(status.get('completed_nodes', [])) or 'none'),
        ]
        for message in messages:
            bridge.publish_event('near_field_status', dict(message=message, severity='info', capture26_working_view=working))
        return
    visual = status['visual']
    age = visual.get('exposure_age_s')
    age_text = 'none' if age is None else f'{age:.3f}s'
    gyros = '/'.join('OK' if value else 'LOST' for value in status['gyro_health'])
    rate = status.get('service_angular_rate_rad_s')
    rate_text = 'none' if rate is None else f'{rate:.5f}rad/s'
    messages = [
        f"CAPTURE26 {status['phase']} | UE RGB STEREO + GNSS + GYROS",
        f"OPTICAL Z {status['optical_depth_m']:.3f}m | GYROS {gyros} | SERVICE RATE {rate_text}",
        f"VISUAL {'READY' if visual['ready'] else 'WAIT'} {visual['count']}/10 | EXPOSURE AGE {age_text}",
        f"PLATFORM {'STABLE' if status.get('platform_stable') else 'WAIT'} {status.get('platform_stable_duration_s', 0.):.2f}s | RATE {'OK' if status.get('service_angular_rate_stable') else 'WAIT'} | RELEASE {bool(status.get('released'))}",
        f"{visual['reason']} | RTF {status['processing_real_time_factor']:.2f} | CONTACT CAPTURE DISABLED",
    ]
    for message in messages:
        bridge.publish_event('near_field_status', dict(message=message, severity='info'))
