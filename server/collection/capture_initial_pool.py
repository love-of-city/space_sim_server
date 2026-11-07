"""Freeze and validate physically tested capture initial conditions."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def validate_capture_payload_model(model):
    """The delivered workbench must belong to the captured spacecraft."""
    root = ET.parse(model).getroot()
    target = root.find('./worldbody/body[@name="capture_target"]')
    if target is None or target.find('.//body[@name="task_box_base"]') is None:
        raise ValueError('capture collection requires target-owned task_box_base; '
                         'a service-spacecraft workbench cannot be delivered with the target')


class CaptureInitialPool:
    def __init__(self,path):
        self.path=Path(path).resolve()
        self.document=json.loads(self.path.read_text())
        if self.document.get('schema') in ('capture-initial-range-pool/1', 'capture-initial-pool/1'):
            validate_capture_payload_model(self.document['model'])
        if self.document.get('schema')=='capture-initial-range-pool/1':
            from collection.capture_initial_domain import verify_calibration, contains_initial
            from simulation.scripted_tasks.capture_platform import platform_contract
            calibration=verify_calibration(self.document['domain'])
            if calibration.get('model_identity') and hashlib.sha256(Path(self.document['model']).read_bytes()).hexdigest()!=calibration['model_identity']['sha256']:
                raise ValueError('capture range pool model differs from its calibration')
            if self.document.get('capture_platform_contract') != platform_contract():
                raise ValueError('random capture pool belongs to a different platform')
            if not self.document.get('layouts') or not all(contains_initial(e['initial_state'],
                    self.document['domain']['sampling_config']) for e in self.document['layouts']):
                raise ValueError('random capture pool empty or outside calibrated bounds')
            self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
            return
        if self.document.get('schema')!='capture-initial-pool/1':
            raise ValueError('unsupported capture initial pool')
        self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
        if not self.document.get('layouts'): raise ValueError('capture initial pool is empty')
        for entry in self.document['layouts']:
            if entry.get('process_returncode') != 0:
                raise ValueError('initial pool requires a normally exited physical trial')
            evidence=Path(entry['physical_report'])
            if hashlib.sha256(evidence.read_bytes()).hexdigest()!=entry['report_sha256']:
                raise ValueError('physical feasibility evidence changed')
            report=json.loads(evidence.read_text())
            if report.get('source_model_sha256'):
                model=Path(self.document['model'])
                if hashlib.sha256(model.read_bytes()).hexdigest()!=report['source_model_sha256']:
                    raise ValueError('capture pool model differs from the physically tested source')
            if report.get('result')!='success' or report.get('preflight',{}).get('accepted') is not True:
                raise ValueError('initial pool requires geometric and physical success')
            from simulation.scripted_tasks.capture_platform import platform_contract
            if report.get('capture_platform_contract') != platform_contract():
                raise ValueError('initial pool predates the complete platform environment; recalibrate')
            if report['initial_state']!=entry['initial_state']:
                raise ValueError('pool pose differs from the physically tested pose')

    def write_snapshot(self,path):
        Path(path).write_text(json.dumps(self.document,indent=2))

    def select_index(self,index,seed,stream=None):
        entry=self.document['layouts'][index]
        if self.document['schema']=='capture-initial-range-pool/1':
            from collection.capture_initial_domain import sampling_bounds
            domain=self.document['domain']
            return Path(self.document['model']),Path(self.document['catalog']),None,dict(
                enabled=True,schema='capture-initial-selection/1',pool_sha256=self.sha256,
                index=index,id=entry['id'],initial_state=deepcopy(entry['initial_state']),
                sampling_mode='frozen_range_random_once',
                bounds=deepcopy(sampling_bounds(domain['sampling_config'])),
                calibration_sha256=domain['calibration_sha256'])
        return Path(self.document['model']),Path(self.document['catalog']),None,dict(
            enabled=True,schema='capture-initial-selection/1',pool_sha256=self.sha256,
            index=index,id=entry['id'],initial_state=deepcopy(entry['initial_state']),
            physical_report=entry['physical_report'],report_sha256=entry['report_sha256'])


def source_scene(seed=0):
    from space_arm_platform.models import SceneInstanceCreate
    from space_arm_platform.scene_runtime import _sample_instance, SCENE_RENDER_PROFILES
    from simulation.scripted_tasks.capture_platform import platform_contract
    # Use the platform's saved scene defaults, including explicit J2, ephemeris,
    # orbit, lighting and sensor defaults. Only the manipulation reset differs.
    scene = _sample_instance(SceneInstanceCreate(template_id='sarm-dual-capture26',
        randomization_profile='teleop-zero-prepare-v1',
        operating_arm_joint_position_deg=[90.,-60.,60.,0.,-90.,0.]), int(seed))
    scene.update(instance_id='capture-grasp-source', initialization_enabled=True,
                 arm_preparation_required=False, render_environment='platform_main')
    if scene.get('created_by') is None: scene.pop('created_by', None)
    profile = deepcopy(SCENE_RENDER_PROFILES['sarm-dual-capture26'])
    profile['visual_attachments'] = [a for a in profile.get('visual_attachments', [])
        if '/target_aruco_' not in a.get('geometry_file', '')]
    # Preserve the physical stereo cameras; suppress only fiducial processing.
    profile['vision_capture'].update(fiducial_calibration_valid=False, wrist_alignment_enabled=False)
    scene['render_profile'] = profile
    scene.setdefault('sensors', {}).setdefault('service_gyro', {})
    scene['runtime']['ik_rate_hz'] = 200
    scene['capture_platform_contract'] = platform_contract()
    scene['capture_initial_state'] = dict(gap_m=.30,bearing_x_deg=0.,bearing_y_deg=0.,
                                         tilt_x_deg=0.,tilt_y_deg=0.,roll_deg=0.)
    return scene
