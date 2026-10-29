"""Pre-alignment from navigation, cooperative telemetry and measured arm joints.

View checks are geometric pinhole projections, not detections or occlusion
tests. No target state reader or renderer is used here. Nothing in this module
authorizes translation or grasp closure.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from simulation.cooperative_pointing import (
    _rotation, _rotation_vector, _vector, bounded_pointing_torque,
    camera_facing_reference, pointing_status,
)
from simulation.serial_chain_kinematics import SerialChainKinematics, quaternion_wxyz_to_matrix


class EndStereoGeometry:
    """Use grip_frame FK and the same optical mounts as the image estimator."""

    def __init__(self, model_path, config):
        self.config = config
        if config.get('mount_parent_id') != 'teleop/gripper':
            raise ValueError('pre-alignment requires the grasp-end stereo pair')
        root = ET.parse(model_path).getroot()
        gripper = root.find('.//body[@name="gripper"]')
        site = gripper.find('site[@name="grip_frame"]') if gripper is not None else None
        if (site is None or not np.allclose(np.fromstring(site.get('pos', '0 0 0'), sep=' '), 0)
                or not np.allclose(quaternion_wxyz_to_matrix(
                    np.fromstring(site.get('quat', '1 0 0 0'), sep=' ')), np.eye(3))):
            raise ValueError('grip_frame must coincide with the stereo parent gripper frame')
        self.chain = SerialChainKinematics.from_mjcf(model_path, base_body='cubesat_bus',
            joint_names=tuple(f'bottom_joint{i}' for i in range(1, 7)), tool_site='grip_frame')
        self.mounts = {}
        cal = config['calibration']
        # Render frame: X forward. Optical frame: Z forward, X right, Y down.
        optical_from_render = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        if set(config['camera_ids']) != {'left', 'right'} or len(config['cameras']) != 2:
            raise ValueError('exactly two named eyes required')
        for eye in ('left', 'right'):
            candidates = [c for c in config['cameras'] if c['camera_id'] == config['camera_ids'][eye]]
            if len(candidates) != 1:
                raise ValueError('stereo camera IDs must be unique')
            camera = candidates[0]
            mount = config['parent_from_optical'][eye]
            p = _vector(mount['position_parent_m'], 'optical mount')
            r = _rotation(mount['R_parent_from_optical'])
            rendered = quaternion_wxyz_to_matrix(camera['orientation_body_from_camera_wxyz']) @ optical_from_render.T
            if (camera['parent_id'] != 'teleop/gripper'
                    or not np.allclose(rendered, r, atol=1e-8, rtol=0)
                    or not np.allclose(camera['position_body_m'], p, atol=1e-8, rtol=0)):
                raise ValueError('render and optical calibration mounts differ')
            k = np.asarray(cal['K_'+eye], float)
            size = np.asarray(cal['image_size'], int)
            focal = size[0] / (2*np.tan(camera['field_of_view_rad']/2))
            if (k.shape != (3, 3) or not np.isfinite(k).all() or size.shape != (2,)
                    or np.any(size <= 0) or list(size) != camera['resolution']
                    or not np.allclose(k, [[focal,0,(size[0]-1)/2],
                                          [0,focal,(size[1]-1)/2],[0,0,1]], atol=1e-7)
                    or not np.allclose(cal['D_'+eye], 0)):
                raise ValueError('unsupported or inconsistent pinhole calibration')
            self.mounts[eye] = (p, r, k, size)
        pl, rl, _, _ = self.mounts['left']
        pr, rr, _, _ = self.mounts['right']
        if (not np.allclose(rr.T@rl, cal['R_right_from_left'], atol=1e-8)
                or not np.allclose(rr.T@(pl-pr), cal['t_right_from_left_m'], atol=1e-8)):
            raise ValueError('pair calibration differs from optical mounts')
        self.center = _vector(config['marker_rig']['grasp_center_target_m'], 'target reference')
        if config['marker_rig']['face'] != '+Y':
            raise ValueError('pre-alignment currently supports the target +Y main face')
        markers = config['marker_rig']['markers']
        largest = max(m['size_m'] for m in markers)
        self.region = np.asarray([config['layout']['markers'][str(m['id'])]
                                 for m in markers if np.isclose(m['size_m'], largest)], float)
        if self.region.ndim != 3 or self.region.shape[1:] != (4, 3) or not np.isfinite(self.region).all():
            raise ValueError('main-face region requires finite marker corners')

    @classmethod
    def from_files(cls, model_path, camera_config_path):
        return cls(model_path, json.loads(Path(camera_config_path).read_text('utf-8-sig')))

    def cameras_service(self, joint_position_rad):
        p, r = self.chain.forward(joint_position_rad)
        return {eye: (p+r@mount[0], r@mount[1]) for eye, mount in self.mounts.items()}

    def reference(self, relative_position_n_m, service_rotation_n_from_body, joint_position_rad):
        service = _rotation(service_rotation_n_from_body)
        cameras = self.cameras_service(joint_position_rad)
        midpoint = (cameras['left'][0]+cameras['right'][0])/2
        # Printed +Z is aligned to the projected image-up direction.
        up_n = -service@cameras['left'][1][:, 1]
        return camera_facing_reference(relative_position_n_m=relative_position_n_m,
            service_rotation_n_from_body=service, camera_midpoint_service_m=midpoint,
            marker_center_target_m=self.center, up_n=up_n)

    def reference_motion(self, *, relative_position_n_m, relative_velocity_n_m_s,
                         service_rotation_n_from_body, service_angular_rate_body_rad_s,
                         joint_position_rad, joint_velocity_rad_s):
        """Reference attitude and spatial angular rate at one prediction epoch.

        Relative velocity is the J2000 derivative of target-minus-service body
        origins, NOT rotating-frame V-bar velocity. Differentiate the same
        installation-aware reference using local constant-velocity motion over
        +/-1 ms; never difference noisy attitudes from consecutive samples.
        This needs estimated velocities, service gyro and encoder rates, not
        target translation truth. It is a local kinematic derivative, not an
        orbit predictor or arm deployment controller.
        """
        relative = _vector(relative_position_n_m, 'relative position')
        velocity = _vector(relative_velocity_n_m_s, 'relative inertial velocity')
        service = _rotation(service_rotation_n_from_body)
        omega = _vector(service_angular_rate_body_rad_s, 'service body angular rate')
        q, qdot = np.asarray(joint_position_rad, float), np.asarray(joint_velocity_rad_s, float)
        if (q.shape != (len(self.chain.joint_names),) or qdot.shape != q.shape
                or not np.isfinite(q).all() or not np.isfinite(qdot).all()):
            raise ValueError('finite encoder positions and rates matching the chain required')

        def at(dt):
            x, y, z = omega*dt
            skew = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
            angle = np.linalg.norm(omega*dt)
            increment = (np.eye(3) + np.sinc(angle/np.pi)*skew
                         + .5*np.sinc(angle/(2*np.pi))**2*(skew@skew))
            return self.reference(relative+velocity*dt, service@increment, q+qdot*dt)

        reference = self.reference(relative, service, q)
        step = .001
        rate_n = _rotation_vector(at(step)@at(-step).T)/(2*step)
        return reference, rate_n

    def view(self, relative_position_n_m, service_rotation_n_from_body,
             target_rotation_n_from_body, joint_position_rad):
        relative = _vector(relative_position_n_m, 'relative position')
        service = _rotation(service_rotation_n_from_body)
        target = _rotation(target_rotation_n_from_body)
        eyes = {}
        for eye, (p, r) in self.cameras_service(joint_position_rad).items():
            camera_n, rotation_n = service@p, service@r
            points = (self.region@target.T + relative - camera_n)@rotation_n
            front = bool(np.all(points[..., 2] > 0))
            k, size = self.mounts[eye][2:]
            pixel = (points@k.T)[..., :2] / np.where(np.abs(points[..., 2:]) > 1e-12,
                                                    points[..., 2:], np.nan)
            inside = bool(front and np.isfinite(pixel).all() and np.all(pixel >= 0)
                          and np.all(pixel <= size-1))
            sight = camera_n-relative-target@self.center
            cosine = float(target[:, 1]@sight / max(np.linalg.norm(sight), 1e-15))
            facing = cosine > 0
            eyes[eye] = dict(in_front=front, region_in_frame=inside, front_facing=facing,
                face_angle_deg=float(np.rad2deg(np.arccos(np.clip(cosine, -1, 1)))),
                largest_marker_edge_px=(float(np.linalg.norm(pixel-np.roll(pixel,1,axis=1),axis=2).max())
                                        if front else None))
        return dict(geometry_ok=all(e['region_in_frame'] and e['front_facing'] for e in eyes.values()),
                    eyes=eyes, scope='projection only; no detection or occlusion evidence')


@dataclass
class PrealignmentResult:
    torque_body_nm: np.ndarray
    aligned: bool
    reason: str
    dwell_s: float = 0.
    attitude_error_deg: float | None = None
    rate_error_rad_s: float | None = None
    view: dict | None = None


class PrealignmentController:
    """Five-second geometric/attitude handshake; never a mission-release permit.

    All inputs must refer to the same prediction epoch. Source acquisition
    timestamps remain explicit. The caller supplies the reference angular rate
    (zero only for stationary reference tests); orbital tracking needs its rate.
    """
    def __init__(self, geometry, *, dwell_s=5., max_update_gap_s=.15):
        if not np.isfinite([dwell_s, max_update_gap_s]).all() or min(dwell_s, max_update_gap_s) <= 0:
            raise ValueError('positive finite timing thresholds required')
        self.geometry = geometry
        self.required_dwell = dwell_s
        self.max_gap = max_update_gap_s
        self.since = None
        self.previous = None

    def update(self, *, now_s, input_stamps_s, relative_position_n_m,
               navigation_position_sigma_m, service_rotation_n_from_body,
               target_rotation_n_from_body, target_angular_rate_body_rad_s,
               reference_angular_rate_n_rad_s, joint_position_rad, enabled=True, inputs_valid=True,
               reference_rotation_n_from_body=None):
        if self.previous is not None and (now_s <= self.previous or now_s-self.previous > self.max_gap+1e-9):
            self.since = None
            if now_s <= self.previous:
                return PrealignmentResult(np.zeros(3), False, 'nonincreasing_control_time')
        self.previous = now_s
        if not enabled:
            self.since = None
            return PrealignmentResult(np.zeros(3), False, 'prealignment_disabled')
        try:
            # A close-approach coordinator may hold an already established
            # orbital attitude instead of slewing after the moving hand camera.
            # All freshness, angular, view and dwell gates still apply.
            desired = (self.geometry.reference(relative_position_n_m, service_rotation_n_from_body, joint_position_rad)
                       if reference_rotation_n_from_body is None else _rotation(reference_rotation_n_from_body))
            status = pointing_status(now_s=now_s, input_stamps_s=input_stamps_s,
                target_rotation_n_from_body=target_rotation_n_from_body,
                reference_rotation_n_from_body=desired,
                target_angular_rate_body_rad_s=target_angular_rate_body_rad_s,
                reference_angular_rate_n_rad_s=reference_angular_rate_n_rad_s,
                navigation_position_sigma_m=navigation_position_sigma_m, inputs_valid=inputs_valid)
            if status.attitude_error_deg is None:
                self.since = None
                return PrealignmentResult(np.zeros(3), False, status.reason)
            view = self.geometry.view(relative_position_n_m, service_rotation_n_from_body,
                                      target_rotation_n_from_body, joint_position_rad)
            torque = bounded_pointing_torque(target_rotation_n_from_body=target_rotation_n_from_body,
                reference_rotation_n_from_body=desired, target_angular_rate_body_rad_s=target_angular_rate_body_rad_s,
                reference_angular_rate_n_rad_s=reference_angular_rate_n_rad_s)
            eligible = status.ready and view['geometry_ok']
            if not eligible:
                self.since = None
            elif self.since is None:
                self.since = now_s
            dwell = 0. if self.since is None else now_s-self.since
            aligned = eligible and dwell >= self.required_dwell-1e-9
            reason = (status.reason if not status.ready else 'camera_geometry_not_ready'
                      if not view['geometry_ok'] else 'prealigned_not_visual_ready'
                      if aligned else 'alignment_dwell')
            return PrealignmentResult(torque, aligned, reason, dwell,
                                      status.attitude_error_deg, status.rate_error_rad_s, view)
        except (ValueError, KeyError, TypeError):
            self.since = None
            return PrealignmentResult(np.zeros(3), False, 'invalid_prealignment_input')
