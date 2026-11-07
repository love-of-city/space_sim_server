"""Reproducible stationary-spacecraft resets and capture-camera view checks.

Gap is TCP-to-interface displacement along the nominal gripper +Z axis.
Cone angles move the target center; tilt rotates the interface about that
center. These independent perturbations must each pass the physical trial.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
from simulation.serial_chain_kinematics import axis_angle_to_matrix
from .capture_grasp import INTERFACE_CENTER, TARGET_FROM_INTERFACE

from .capture_cameras import APPROACH_OFFSET as CAMERA_OFFSET, GRIP_FROM_OPTICAL, APPROACH_ID


@dataclass(frozen=True)
class CaptureInitialState:
    gap_m: float = .15
    bearing_x_deg: float = 0.
    bearing_y_deg: float = 0.
    tilt_x_deg: float = 0.
    tilt_y_deg: float = 0.
    roll_deg: float = 0.
    arm_joint_position_rad: tuple[float, ...] | None = None

    def __post_init__(self):
        if not np.isfinite([self.gap_m,self.bearing_x_deg,self.bearing_y_deg,
                            self.tilt_x_deg,self.tilt_y_deg,self.roll_deg]).all():
            raise ValueError('finite initial conditions required')
        if self.arm_joint_position_rad is not None:
            q = np.asarray(self.arm_joint_position_rad, dtype=float)
            if q.shape != (6,) or not np.isfinite(q).all():
                raise ValueError('initial capture arm must supply six finite joint angles')
            object.__setattr__(self, 'arm_joint_position_rad', tuple(q.tolist()))
        if not .025 <= self.gap_m <= .80:
            raise ValueError('candidate gap outside diagnostic guard [0.025,0.80] m')
        if max(abs(self.bearing_x_deg), abs(self.bearing_y_deg),
               abs(self.tilt_x_deg), abs(self.tilt_y_deg), abs(self.roll_deg)) > 30:
            raise ValueError('candidate angles exceed 30 degrees')

    def geometry(self, grip_position, grip_rotation):
        bearing = np.tan(np.deg2rad([self.bearing_x_deg, self.bearing_y_deg]))
        interface_position = grip_position + grip_rotation @ np.r_[self.gap_m*bearing, -self.gap_m]
        xyz = np.deg2rad([self.tilt_x_deg, self.tilt_y_deg, self.roll_deg])
        delta = np.eye(3)
        for axis, angle in zip(np.eye(3), xyz):
            delta = delta @ axis_angle_to_matrix(axis, angle)
        interface_rotation = grip_rotation @ delta
        target_rotation = interface_rotation @ TARGET_FROM_INTERFACE.T
        target_position = interface_position - target_rotation @ INTERFACE_CENTER
        return target_position, target_rotation


def camera_view(grip_position, grip_rotation, target_position, target_rotation,
                *, resolution=1024, margin_px=24):
    """Fixed approach camera; conservative 8 cm square around interface.

    Projection is separate from occlusion and rendered-image review.
    """
    center = target_position + target_rotation @ INTERFACE_CENTER
    ri = target_rotation @ TARGET_FROM_INTERFACE
    points = np.array([center + ri @ [x, y, 0.] for x in (-.04, 0., .04)
                       for y in (-.04, 0., .04)])
    cp = grip_position + grip_rotation @ CAMERA_OFFSET
    cr = grip_rotation @ GRIP_FROM_OPTICAL
    optical = (points-cp) @ cr
    focal = resolution/(2*math.tan(math.radians(100)/2))
    pixels = optical[:, :2]/optical[:, 2:]*focal + (resolution-1)/2
    facing_cos = float(ri[:, 2] @ (cp-center)/np.linalg.norm(cp-center))
    accepted = bool(np.all(optical[:, 2] > 0) and facing_cos > math.cos(math.radians(60))
                    and np.all(pixels >= margin_px) and np.all(pixels <= resolution-1-margin_px))
    return dict(projected_visible=accepted, pixel_bounds=[pixels.min(0).tolist(), pixels.max(0).tolist()],
                minimum_optical_depth_m=float(optical[:, 2].min()),
                face_angle_deg=float(np.rad2deg(np.arccos(np.clip(facing_cos, -1, 1)))),
                camera_id=APPROACH_ID,
                scope='projection; native occlusion and real images checked separately',
                camera_position_service_m=cp.tolist(), region_points_service_m=points.tolist())
