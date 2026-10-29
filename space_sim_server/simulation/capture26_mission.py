"""Versioned capture26 flight geometry and acceptance configuration.

The motion demo freezes four physical parking nodes and a folded TCP reference.
Image-navigation candidates retain their separate certification rules.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

MODEL_ID = 'sarm-dual-capture26'
MODEL_PATH = Path('model/SARM/platform/sarm_dual_capture26.xml')
VISION_PATH = Path('config/vision/capture26_rendezvous_stereo.json')
MISSION_PATH = Path('config/missions/capture26-v2.json')
# Labels are mounted on the positive-Y face. Looking towards -Y keeps their
# decoded right/down directions (-X/-Z) in front of the camera.
OPTICAL_IN_TARGET = np.array([[-1., 0, 0], [0, 0, -1], [0, -1, 0]])
MARKER_REFERENCE_TARGET = np.array([.1755, .05475, .0420787001299893])


@dataclass(frozen=True)
class Capture26Mission:
    schema: str = 'space-sim-capture26-mission/2'
    physics_hz: int = 400
    attitude_hz: int = 100
    attitude_control_mode: str = 'body_rate_damping'
    service_angular_rate_limit_rad_s: float = .001
    gyro_hz: int = 50
    guidance_hz: int = 20
    gnss_hz: int = 10
    stereo_hz: int = 10
    stereo_delay_s: float = .20
    gyro_delay_s: float = 0.
    cooperative_delay_s: float = 0.
    vision_task_timeout_wall_s: float = 15.
    com_hold_m: float = 50.
    gnss_hold_depth_m: float = 17.4
    acquisition_depth_m: float = 4.
    readiness_depth_m: float = 2.
    terminal_depth_m: float | None = None
    dwell_s: float = 5.
    vision_timeout_s: float = 60.
    force_limit_n: float = 25.
    thrust_reserve: float = .4
    vision_nodes_validated: bool = False
    geometry_certificate_sha256: str | None = None
    acceptance_profile: str = 'image_navigation_v2'
    feedback_source: str = 'navigation_estimate'
    visual_hold_depth_m: float = 5.50
    position_tolerance_m: float = .05
    velocity_tolerance_m_s: float = .01
    far_position_tolerance_m: float = 2.
    terminal_gap_m: float = .5
    operating_geometry_file: str | None = None
    operating_geometry_sha256: str | None = None

    @property
    def is_demo(self):
        return self.acceptance_profile == 'simulation_state_demo_v1'

    def __post_init__(self):
        numeric = [value for value in asdict(self).values() if type(value) in (int, float)]
        if not np.isfinite(numeric).all():
            raise ValueError('capture26 mission numbers must be finite')
        if self.schema != 'space-sim-capture26-mission/2' or self.stereo_delay_s != .20:
            raise ValueError('capture26 requires version 2 fixed stereo delay')
        if self.attitude_control_mode != 'body_rate_damping' or self.service_angular_rate_limit_rad_s != .001:
            raise ValueError('capture26 requires body-rate damping and the 0.001 rad/s service rate limit')
        if (self.physics_hz, self.attitude_hz, self.gyro_hz, self.guidance_hz, self.gnss_hz, self.stereo_hz) != (400, 100, 50, 20, 10, 10):
            raise ValueError('capture26 mission rates are fixed by the physical model')
        if (not self.gnss_hold_depth_m > self.acquisition_depth_m >= self.readiness_depth_m > 0
                or self.terminal_depth_m is not None and not self.readiness_depth_m > self.terminal_depth_m > 0):
            raise ValueError('invalid optical node ordering')
        if (self.dwell_s != 5. or self.gyro_delay_s < 0
                or self.cooperative_delay_s < 0 or self.vision_task_timeout_wall_s <= 0
                or self.force_limit_n != 25. or not 0 < self.thrust_reserve < 1
                or self.com_hold_m != 50. or self.gnss_hold_depth_m != 17.4
                or self.vision_timeout_s <= 0):
            raise ValueError('invalid capture26 physical or health configuration')
        if (self.acceptance_profile not in ('image_navigation_v2', 'simulation_state_demo_v1')
                or self.feedback_source != ('authoritative_simulation' if self.is_demo else 'navigation_estimate')):
            raise ValueError('capture26 feedback and acceptance rules must agree')
        if self.is_demo and (self.visual_hold_depth_m != 5.5 or self.position_tolerance_m != .05
                or self.velocity_tolerance_m_s != .01 or self.far_position_tolerance_m != 2.
                or self.terminal_gap_m != .5 or not self.operating_geometry_file or not self.operating_geometry_sha256):
            raise ValueError('capture26 demo requires frozen geometry and explicit parking rules')

    def to_dict(self):
        return asdict(self)


def load_vision(root):
    config = json.loads((Path(root)/VISION_PATH).read_text('utf-8-sig'))
    return config


def load_mission(root):
    """The formal runner and operation scenes resolve the same frozen document."""
    document = json.loads((Path(root)/MISSION_PATH).read_text('utf-8-sig'))
    if document.get('schema') != 'space-sim-capture26-task/2':
        raise ValueError('capture26 frozen task schema mismatch')
    if not {'attitude_control_mode', 'service_angular_rate_limit_rad_s'} <= document['mission'].keys():
        raise ValueError('capture26 frozen task lacks explicit attitude rules')
    config = Capture26Mission(**document['mission'])
    if config.is_demo:
        from simulation.capture26_demo_geometry import load_geometry
        load_geometry(root, config)
    if config.vision_nodes_validated or config.geometry_certificate_sha256:
        validate_certificate(root, config, document.get('certificate'))
    return config


def model_files(root):
    """Bind the XML and all referenced meshes, including LFS geometry."""
    root = Path(root).resolve()
    model = root/MODEL_PATH
    files = {model}
    for mesh in ET.parse(model).findall('./asset/mesh'):
        if mesh.get('file'):
            path = (model.parent/mesh.get('file')).resolve()
            if not path.is_relative_to(root/'model'):
                raise ValueError('capture26 mesh escaped the versioned model tree')
            files.add(path)
    return {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(files)}


def validate_certificate(root, config, certificate):
    from space_arm_platform.visual_navigation import calibration_id
    if not isinstance(certificate, dict):
        raise ValueError('capture26 geometry certificate missing')
    digest = hashlib.sha256(json.dumps(certificate, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    required = {'dual_arm_reachability', 'self_collision', 'target_clearance', 'camera_occlusion',
                'braking', 'image_quality', 'covariance_consistency', 'normal_joint_preparation'}
    if (digest != config.geometry_certificate_sha256 or certificate.get('schema') != 'space-sim-capture26-certificate/1'
            or certificate.get('model_sha256') != hashlib.sha256((Path(root)/MODEL_PATH).read_bytes()).hexdigest()
            or certificate.get('model_files_sha256') != model_files(root)
            or certificate.get('calibration_id') != calibration_id(load_vision(root))
            or certificate.get('nodes_m') != [config.acquisition_depth_m, config.readiness_depth_m, config.terminal_depth_m]
            or not required <= set(certificate.get('checks', {}))
            or not all(value is True for value in certificate['checks'].values())
            or certificate.get('calibration_seed') != 7 or certificate.get('validation_seeds') != [19, 31]
            or not certificate.get('evidence_sha256')):
        raise ValueError('capture26 certificate binding or independent checks failed')
    evidence = certificate['evidence_sha256']
    if not isinstance(evidence, dict) or not evidence:
        raise ValueError('capture26 certificate requires evidence files and hashes')
    repository = Path(root).resolve().parent
    for name, expected in evidence.items():
        path = (repository/name).resolve()
        if (not path.is_relative_to(repository/'archive') or not path.is_file()
                or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise ValueError('capture26 certificate evidence missing or changed: '+str(name))
    frames = certificate.get('frames', {})
    for name in ('marker_reference_target_m', 'grasp_interface_target_m', 'grasp_interface_rotation_target'):
        value = np.asarray(frames.get(name), float)
        if value.shape != ((3, 3) if 'rotation' in name else (3,)) or not np.isfinite(value).all():
            raise ValueError('capture26 certificate lacks distinct model/interface/tool frames')
    if (frames.get('tool_site') != 'grip_frame' or not np.array_equal(frames['marker_reference_target_m'], MARKER_REFERENCE_TARGET)
            or not np.allclose(np.array(frames['grasp_interface_rotation_target']).T@frames['grasp_interface_rotation_target'], np.eye(3))
            or not np.isclose(np.linalg.det(frames['grasp_interface_rotation_target']), 1.)):
        raise ValueError('capture26 certificate frame binding failed')
    preparation = certificate.get('preparation', {})
    if preparation.get('evidence_file') not in evidence or not preparation.get('plan_sha256'):
        raise ValueError('capture26 certificate requires bound normal joint preparation')
    from simulation.capture26_preparation import JointPreparation
    plan = JointPreparation(json.loads((repository/preparation['evidence_file']).read_text('utf-8')), Path(root)/MODEL_PATH)
    if plan.digest != preparation['plan_sha256'] or not np.array_equal(plan.points[0], np.zeros(12)):
        raise ValueError('capture26 preparation plan hash or authored start changed')
    return digest


def load_certified_preparation(root, config):
    """Resolve only the evidence-bound path approved in the frozen task."""
    if not config.geometry_certificate_sha256:
        return None
    document = json.loads((Path(root)/MISSION_PATH).read_text('utf-8-sig'))
    validate_certificate(root, config, document.get('certificate'))
    from simulation.capture26_preparation import JointPreparation
    plan = Path(root).resolve().parent/document['certificate']['preparation']['evidence_file']
    return JointPreparation(json.loads(plan.read_text('utf-8')), Path(root)/MODEL_PATH)


def source_identity(root, adapter=None):
    root = Path(root)
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
    sources = list((root/'simulation').rglob('*.py'))
    sources += list((root/'backend/space_arm_platform').rglob('*.py'))
    sources += [root/'backend/space_arm_platform/stereo_pose.py',
                root/'backend/space_arm_platform/visual_navigation.py', root/'native/local_mujoco_stepper.cpp',
                root/'backend/space_arm_platform/capture_receiver.py',
                root/VISION_PATH, root/MISSION_PATH, root/'tools/validate_capture26_navigation.py', root/'tools/run_capture26_vision.py',
                root/'tools/score_capture26_navigation.py', root/'tools/scan_capture26_vision.py',
                root/'tools/score_capture26_platform.py', root/'tools/analyze_capture26_geometry.py',
                root/'tools/audit_capture26_attitude.py',
                root/'tools/check_capture26_continuity.py']
    adapter = (root.parent/'space_sim_UE_Adapter') if adapter is None else Path(adapter).resolve()
    adapter_sources = [adapter/'Adapters/bsk_render_adapter/bridge.py',
                       adapter/'Adapters/bsk_render_adapter/near_field.py',
                       adapter/'Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/BskUnrealRuntime.Build.cs',
                       adapter/'Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/Private/BskVisionPng.cpp',
                       adapter/'Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/Private/BskVisionPng.h',
                       adapter/'Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/Private/BskSceneController.cpp',
                       adapter/'Unreal/BskUnrealRenderer/scripts/start_renderer.ps1']
    return dict(model_id=MODEL_ID, source_model=MODEL_PATH.as_posix(), git_revision=revision,
                source_sha256=hashlib.sha256((root/MODEL_PATH).read_bytes()).hexdigest(),
                model_files_sha256=model_files(root),
                source_files_sha256={p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                adapter_files_sha256={p.relative_to(adapter).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in adapter_sources})


def stopping_distance(speed, sigma_m, latency_s, acceleration_m_s2, *, reserve=.4):
    values = np.array([speed, sigma_m, latency_s, acceleration_m_s2, reserve], float)
    if not np.isfinite(values).all() or min(speed, sigma_m, latency_s) < 0 or acceleration_m_s2 <= 0 or not 0 < reserve < 1:
        raise ValueError('invalid braking inputs')
    available = acceleration_m_s2*reserve
    # Includes one guidance period and the full measured sensor age. The
    # remaining thrust budget covers gravity, tracking and force uncertainty.
    return 3*sigma_m + speed*(latency_s+.05) + speed**2/(2*available)


def desired_geometry(depth, target_rotation, service_com, target_com, vision, *, service_rotation=None):
    """COM offset on the marker-normal lane, with independent body attitudes.

    Omitting service_rotation is reserved for nominal initialization/offline
    geometry. Runtime callers must pass the service navigation attitude.
    The second return value is a geometry diagnostic, never an attitude command.
    """
    mount = vision['body_from_optical']
    c = np.array(mount['left']['R_body_from_optical'])
    camera = .5*(np.array(mount['left']['position_body_m'])+np.array(mount['right']['position_body_m']))
    rs = target_rotation@OPTICAL_IN_TARGET@c.T if service_rotation is None else service_rotation
    forward = target_rotation@OPTICAL_IN_TARGET[:, 2]
    offset = target_rotation@(MARKER_REFERENCE_TARGET-target_com)-rs@(camera-service_com)-depth*forward
    return offset, rs


def translation_reference(depth, state, service_com, target_com, vision, *, depth_rate=0.,
                          com_distance_m=None, com_distance_rate_m_s=0.):
    """Relative COM reference and derivative from estimates and joint encoders.

    The COM node specifies radius, not a frozen optical depth. Differentiating
    that constraint includes the moving normal and both independent lever arms.
    """
    rs, rt = state['r']
    axis = rt@OPTICAL_IN_TARGET[:, 2]
    base, _ = desired_geometry(0., rt, service_com, target_com, vision, service_rotation=rs)
    if com_distance_m is not None:
        projection = float(base@axis)
        squared = com_distance_m**2-float(base@base)+projection**2
        if squared <= 0:
            raise ValueError('COM distance is incompatible with the marker-normal lane')
        depth = projection+float(np.sqrt(squared))
    offset = base-depth*axis
    mount = vision['body_from_optical']
    camera = .5*(np.array(mount['left']['position_body_m'])+np.array(mount['right']['position_body_m']))
    service_lever = rs@(camera-service_com)
    target_lever = rt@(MARKER_REFERENCE_TARGET-target_com-depth*OPTICAL_IN_TARGET[:, 2])
    omega_s, omega_t = (r@w for r, w in zip(state['r'], state['rates']-state['b']))
    dc = state.get('com_velocity_body', np.zeros((2, 3)))
    velocity = (np.cross(omega_t, target_lever)-np.cross(omega_s, service_lever)
                +rs@dc[0]-rt@dc[1])
    if com_distance_m is not None:
        depth_rate = float((offset@velocity-com_distance_m*com_distance_rate_m_s)/(offset@axis))
    return offset, velocity-depth_rate*axis, float(depth)
