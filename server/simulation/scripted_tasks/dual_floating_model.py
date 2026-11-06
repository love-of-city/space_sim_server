"""Compose two independent target instances with the normalized dual-arm model."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from simulation.sarm_model_profile import DUAL_ARMS
from simulation.serial_chain_kinematics import SerialChainKinematics, matrix_to_quaternion_wxyz
from simulation.socket_colors import COLOR_KEYS
from .floating_assets import FLOATING_TASKS
from .floating_initialization import build_floating_body
from .floating_variations import apply_target_variations
from .motion import solve_pose
from .randomization import generator

DUAL_TASK = 'dual_floating_sequential_capture'


class DualFloatingConfig(BaseModel):
    model_config = ConfigDict(extra='forbid', frozen=True, allow_inf_nan=False)
    left_task: str = 'floating_plug_capture'
    right_task: str = 'floating_aviation_plug_capture'
    position_jitter_m: float = Field(default=.025, ge=0, le=.08)
    color_choices: tuple[str, ...] = COLOR_KEYS

    @model_validator(mode='after')
    def check(self):
        if any(t not in FLOATING_TASKS for t in (self.left_task,self.right_task)):
            raise ValueError('Both targets must be registered floating object types')
        if len(set(self.color_choices)) < 2 or any(c not in COLOR_KEYS for c in self.color_choices):
            raise ValueError('Two distinct registered colors are required')
        return self


def import_donor_body(root, donor, donor_path, asset, instance_id):
    """Namespace physical defaults and only the assets referenced by this part."""
    body=build_floating_body(donor,asset,instance_id)
    prefix='floating_donor_'
    if root.find(f'./default/default[@class="{prefix}root"]') is None:
        defaults=deepcopy(donor.find('default'))
        defaults.set('class',prefix+'root')
        for node in defaults.iter():
            for key in ('class','childclass'):
                if key in node.attrib and node.get(key)!=prefix+'root': node.set(key,prefix+node.get(key))
        root.find('default').append(defaults)
    if 'childclass' not in body.attrib: body.set('childclass','root')
    asset_root=root.find('asset')
    compiler=donor.find('compiler')
    def copy_asset(kind,name):
        target=prefix+name
        if asset_root.find(f'{kind}[@name="{target}"]') is not None: return target
        original=donor.find(f'./asset/{kind}[@name="{name}"]')
        if original is None: raise ValueError(f'Missing donor {kind} {name}')
        copied=deepcopy(original);copied.set('name',target)
        if 'file' in copied.attrib:
            folder=compiler.get('meshdir' if kind=='mesh' else 'texturedir','.')
            copied.set('file',str((donor_path.parent/folder/copied.get('file')).resolve()))
        if kind=='material' and copied.get('texture'): copied.set('texture',copy_asset('texture',copied.get('texture')))
        asset_root.append(copied)
        return target
    for node in body.iter():
        for key in ('class','childclass'):
            if key in node.attrib: node.set(key,prefix+node.get(key))
        for kind in ('mesh','material','hfield'):
            if kind in node.attrib: node.set(kind,copy_asset(kind,node.get(kind)))
    root.find('worldbody').append(body)
    return body


def prepare_dual_model(base, donor_path, output, scene):
    base,donor_path,output=map(lambda p:Path(p).resolve(),(base,donor_path,output))
    identity=json.loads(base.with_suffix('.json').read_text())
    if hashlib.sha256(base.read_bytes()).hexdigest()!=identity['model_sha256']:
        raise ValueError('Normalized dual-arm model hash mismatch')
    config=DualFloatingConfig.model_validate(scene.get('dual_floating',{}))
    root,donor=ET.parse(base).getroot(),ET.parse(donor_path).getroot()
    for node in root.findall('./asset/mesh[@file]'):
        node.set('file',str((base.parent/root.find('compiler').get('meshdir','.')/node.get('file')).resolve()))
    root.find('compiler').set('meshdir','.')
    # The source keyframes have no free object coordinates; initialization uses named joints.
    for keyframe in root.findall('keyframe'): root.remove(keyframe)
    rng=generator(scene['seed'],'dual-floating-instances-v1')
    colors=[str(c) for c in rng.choice(config.color_choices,2,replace=False)]
    initial=deepcopy(identity['workspace_joint_positions'])
    instances=[];arm_records={}
    for index,(side,arm) in enumerate(DUAL_ARMS.items()):
        kind=getattr(config,side+'_task');asset=FLOATING_TASKS[kind]
        sign=-1 if side=='left' else 1
        q=np.array([initial[n][0] for n in arm.joint_names[:6]])
        chain=SerialChainKinematics.from_mjcf(base,base_body='cubesat_bus',joint_names=arm.joint_names[:6],tool_site=arm.tool_site)
        _,rotation=chain.forward(q)
        lower,upper=np.array([np.fromstring(root.find(f'.//joint[@name="{n}"]').get('range'),sep=' ') for n in arm.joint_names[:6]]).T
        park=np.array([sign*.30,.05,.50])
        parked=solve_pose(chain,park,rotation,q,lower,upper)
        initial.update({n:[float(v)] for n,v in zip(arm.joint_names[:6],parked)})
        target= np.array([sign*.25,.15,.30])+rng.uniform(-config.position_jitter_m,config.position_jitter_m,3)
        grasp_basis=np.column_stack((asset.closing_axis,np.cross(asset.approach_axis,asset.closing_axis),asset.approach_axis))
        object_rotation=rotation@grasp_basis.T
        position=target-object_rotation@asset.grip_point
        instance_id=f'object_{side}'
        body=import_donor_body(root,donor,donor_path,asset,instance_id)
        quaternion=matrix_to_quaternion_wxyz(object_rotation)
        body.set('pos',' '.join(format(v,'.17g') for v in position))
        body.set('quat',' '.join(format(v,'.17g') for v in quaternion))
        variation_scene=dict(seed=scene['seed'],floating_target=dict(color_choices=[colors[index]],mass_scale_range=[1.,1.],friction_scale_range=[1.,1.]),collection_config={})
        variation=apply_target_variations(root,variation_scene,instance_id,instance_key=instance_id)
        instances.append(dict(instance_id=instance_id,arm=side,type_task=kind,free_joint=f'{instance_id}_free',
            color=colors[index],variation=variation,position_body_m=position.tolist(),orientation_wxyz=quaternion.tolist(),
            linear_velocity_body_m_s=[0.,0.,0.],angular_velocity_body_rad_s=[0.,0.,0.]))
        arm_records[side]=dict(initial_joint_position_rad=[*parked.tolist(),0.,0.],
            retract_position_body_m=[sign*.30,-.05,.40],target_instance_id=instance_id)
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root,space='  ');ET.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)
    record=dict(schema='dual-floating-scene/1',base_sha256=identity['model_sha256'],
        donor_sha256=hashlib.sha256(donor_path.read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),options=config.model_dump(mode='json'),
        instances=instances,arms=arm_records,initial_joint_positions=initial)
    scene.setdefault('collection_config',{})['dual_floating']=record
    return record


def randomized_dual_scene(source, seed):
    from .randomization import prepare_episode, options_for_scene
    scene=deepcopy(source)
    if scene.get('template_id')!='dual-service-v2-floating' or scene.get('collection_camera_profile')!='dual-arm-three-view-v1':
        raise ValueError('Dual task requires its dual model and three-camera profile')
    options=options_for_scene(scene)
    if any(enabled for name,enabled in options.flags().items() if name!='lighting'):
        raise ValueError('Dual collection currently supports lighting randomization; arm disturbances require a two-arm implementation')
    scene['seed']=int(seed)
    scene['collection_config']=dict(task=DUAL_TASK,initialization=dict(schema='dual-floating-initialization/1',seed=int(seed)))
    if scene.get('randomize_overview_camera'):
        raise ValueError('Dual camera mounts are fixed in the delivered model')
    if scene.get('randomize_orbit_phase'):
        from space_arm_platform.orbit_randomization import sample_orbit_phase
        from simulation.teleop_grasp_unreal import DEFAULT_ORBIT
        orbit={**DEFAULT_ORBIT,**scene.get('environment',{}).get('orbit',{})}
        orbit['true_anomaly_deg']=sample_orbit_phase(int(seed))
        scene.setdefault('environment',{})['orbit']=orbit
    prepare_episode(scene)
    return scene
