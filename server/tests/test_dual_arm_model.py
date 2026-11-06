import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from simulation.sarm_model_profile import DUAL_ARMS
from simulation.serial_chain_kinematics import SerialChainKinematics
from tools.prepare_dual_arm_model import prepare

ROOT=Path(__file__).resolve().parents[1]/'model/dual_stowed_service'


def test_normalization_preserves_both_physical_arms_and_rewrites_references(tmp_path):
    source=ROOT/'camera_version_2/dual_stowed_service.xml'
    output=tmp_path/'dual.xml'
    result=prepare(source,output,ROOT/'workspace_pose.json')
    oldroot,newroot=ET.parse(source).getroot(),ET.parse(output).getroot()
    for side,arm in DUAL_ARMS.items():
        opposite='right' if side=='left' else 'left'
        arm.validate(output)
        original=SerialChainKinematics.from_mjcf(source,base_body='cubesat_bus',
            joint_names=tuple(f'{opposite}_joint{i}' for i in range(1,7)),tool_site=f'{opposite}_sarm_ee')
        normalized=SerialChainKinematics.from_mjcf(output,base_body='cubesat_bus',
            joint_names=arm.joint_names[:6],tool_site=arm.tool_site)
        q=np.array([result['workspace_joint_positions'][j][0] for j in arm.joint_names[:6]])
        for angles in (q,q+np.array([.01,0,0,0,0,0])):
            p,r=original.forward(angles);p2,r2=normalized.forward(angles)
            np.testing.assert_allclose(p,p2,atol=1e-12)
            np.testing.assert_allclose(r,r2,atol=1e-12)
        assert newroot.find(f'./sensor/force[@name="{side}_wrist_ft_force"]').get('site') == f'{side}_wrist_ft_site'
    assert [n.attrib for n in oldroot.findall('.//inertial')] == [n.attrib for n in newroot.findall('.//inertial')]
    assert newroot.find('./worldbody/body[@name="capture_target"]') is None
    assert result['left_body_axis']=='-X'


def test_target_instances_have_separate_donor_dependencies_and_zero_initial_speed(tmp_path):
    from simulation.scripted_tasks.dual_floating_model import prepare_dual_model
    source=ROOT/'camera_version_2/dual_stowed_service.xml'
    base=tmp_path/'base.xml'
    prepare(source,base,ROOT/'workspace_pose.json')
    # Dependencies retain paths to the delivered assets; local mesh pointers are not loaded.
    model=tmp_path/'scene.xml'
    scene=dict(seed=75,dual_floating=dict(left_task='floating_plug_capture',right_task='floating_plug_capture'))
    record=prepare_dual_model(base,ROOT.parent/'SARM/platform/sarm_20261012.xml',model,scene)
    assert record['instances'][0]['color'] != record['instances'][1]['color']
    root=ET.parse(model).getroot()
    assert root.find('./worldbody/body[@name="capture_target"]') is None
    for obj in record['instances']:
        body=root.find(f'./worldbody/body[@name="{obj["instance_id"]}"]')
        assert body.find('freejoint').get('name') == obj['free_joint']
        assert obj['linear_velocity_body_m_s']==obj['angular_velocity_body_rad_s']==[0,0,0]
        for geom in body.findall('geom[@mesh]'):
            assert root.find(f'./asset/mesh[@name="{geom.get("mesh")}"]') is not None
    assert root.find('keyframe') is None
