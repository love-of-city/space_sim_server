"""Versioned capture26 flight geometry and acceptance configuration.

The two vision nodes are candidates until an actual UE sweep has certified
camera error and braking margins. This file never changes MJCF assets.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

MODEL_ID = 'sarm-dual-capture26'
MODEL_PATH = Path('model/SARM/platform/sarm_dual_capture26.xml')
VISION_PATH = Path('config/vision/capture26_rendezvous_stereo.json')
# Labels are mounted on the positive-Y face. Looking towards -Y keeps their
# decoded right/down directions (-X/-Z) in front of the camera.
OPTICAL_IN_TARGET = np.array([[-1., 0, 0], [0, 0, -1], [0, -1, 0]])
INTERFACE_TARGET = np.array([.1755, .05475, .0420787001299893])


@dataclass(frozen=True)
class Capture26Mission:
    schema: str = 'space-sim-capture26-mission/1'
    physics_hz: int = 400
    attitude_hz: int = 100
    gyro_hz: int = 50
    guidance_hz: int = 20
    gnss_hz: int = 10
    stereo_hz: int = 10
    com_hold_m: float = 50.
    gnss_hold_depth_m: float = 17.4
    acquisition_depth_m: float = 4.
    readiness_depth_m: float = 2.
    terminal_depth_m: float = .88
    dwell_s: float = 5.
    vision_timeout_s: float = 60.
    force_limit_n: float = 25.
    thrust_reserve: float = .4
    vision_nodes_validated: bool = False

    def __post_init__(self):
        if (self.physics_hz, self.attitude_hz, self.gyro_hz, self.guidance_hz, self.gnss_hz, self.stereo_hz) != (400, 100, 50, 20, 10, 10):
            raise ValueError('capture26 mission rates are fixed by the physical model')
        if not self.gnss_hold_depth_m > self.acquisition_depth_m >= self.readiness_depth_m > self.terminal_depth_m > 0:
            raise ValueError('invalid optical node ordering')

    def to_dict(self):
        return asdict(self)


def load_vision(root):
    config = json.loads((Path(root)/VISION_PATH).read_text('utf-8-sig'))
    return config


def source_identity(root, adapter=None):
    root = Path(root)
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
    sources = list((root/'simulation').rglob('*.py'))
    sources += list((root/'backend/space_arm_platform').rglob('*.py'))
    sources += [root/'backend/space_arm_platform/stereo_pose.py',
                root/'backend/space_arm_platform/visual_navigation.py', root/'native/local_mujoco_stepper.cpp',
                root/'backend/space_arm_platform/capture_receiver.py',
                root/VISION_PATH, root/'tools/validate_capture26_navigation.py', root/'tools/run_capture26_vision.py',
                root/'tools/score_capture26_navigation.py', root/'tools/scan_capture26_vision.py',
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


def desired_geometry(depth, target_rotation, service_com, target_com, vision):
    mount = vision['body_from_optical']
    c = np.array(mount['left']['R_body_from_optical'])
    camera = .5*(np.array(mount['left']['position_body_m'])+np.array(mount['right']['position_body_m']))
    rs = target_rotation@OPTICAL_IN_TARGET@c.T
    forward = target_rotation@OPTICAL_IN_TARGET[:, 2]
    offset = target_rotation@(INTERFACE_TARGET-target_com)-rs@(camera-service_com)-depth*forward
    return offset, rs
