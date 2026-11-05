"""Independent bodies, colors, replay and target identity in cluttered scenes."""
from copy import deepcopy
from dataclasses import replace
import json
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from test_floating_capture import MODEL, source_scene, sample_at_grasp
from test_floating_options import parse
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.randomization import write_model_variant
from simulation.scripted_tasks.floating_capture import create_floating_task
from simulation.scripted_tasks.floating_initialization import floating_release_options, initialize_floating_body
from simulation.scripted_tasks.floating_objects import FloatingSceneConfig, validate_floating_object_recording
from simulation.scripted_tasks.floating_options import apply_floating_options
from simulation.scripted_tasks.native_state import FloatingState
from collection.record_scripted_task import task_instruction


def realize(tmp_path, count=2, composition='same-type-colors'):
    source=source_scene()
    source['floating_capture']=dict(min_distance_m=.35,max_distance_m=.55,max_tilt_deg=0.)
    source['floating_scene']=dict(object_count=count,composition=composition)
    if composition=='different-types':source['floating_scene']['distractor_types']=['floating_aviation_plug_capture','floating_guide_plug_capture']
    source['floating_target']=dict(color_choices=['blue'])
    scene=randomized_scene(source,75,0,'floating_plug_capture')
    model=write_model_variant(MODEL,tmp_path/'model.xml',scene)
    return source,scene,model


@pytest.mark.parametrize('count,composition',[(1,'different-types'),(2,'different-types'),(3,'different-types'),(2,'same-type-colors'),(3,'same-type-colors')])
def test_count_unique_free_bodies_and_replay(tmp_path,count,composition):
    source,scene,model=realize(tmp_path,count,composition)
    cfg=scene['collection_config'];record=cfg['floating_objects'];instances=record['instances']
    assert len(instances)==count and sum(i['is_target'] for i in instances)==1
    assert record['target_instance_id']=='cylindrical_plug'
    assert len({i['instance_id'] for i in instances})==count
    assert len({i['variation']['color'] for i in instances})==count
    if composition=='different-types':assert len({i['type_task'] for i in instances})==count
    xml=ET.parse(model).getroot();original=ET.parse(MODEL).getroot()
    for obj in instances:
        body=xml.find(f'./worldbody/body[@name="{obj["instance_id"]}"]')
        assert body is not None
        joint=next(c for c in body if c.tag=='freejoint' or (c.tag=='joint' and c.get('type')=='free'))
        assert obj['free_joint']==joint.get('name')
        for name in obj['variation']['colored_geoms']:
            assert body.find(f'geom[@name="{name}"]').get('material')==obj['variation']['material_name']
        if count>1:
            np.testing.assert_array_equal(obj['initial_state']['velocity_body_m_s'],np.zeros(3))
            np.testing.assert_array_equal(obj['initial_state']['angular_velocity_body_rad_s'],np.zeros(3))
    assert len({i['variation']['material_name'] for i in instances})==count
    # Distractor originals remain assembled and unchanged.
    for name in ('aviation_plug','guide_plug'):
        assert ET.tostring(xml.find(f'.//body[@name="{name}"]'))==ET.tostring(original.find(f'.//body[@name="{name}"]'))
    replay=json.loads(json.dumps(scene))
    assert write_model_variant(MODEL,tmp_path/'replay.xml',replay).read_bytes()==model.read_bytes()
    before=deepcopy(source)
    apply_floating_options(source,parse('--object-count','2'))
    assert source==before


@pytest.mark.parametrize('key',['target_instance_id','color','position','type'])
def test_saved_instance_tamper_rejected(tmp_path,key):
    _,scene,_=realize(tmp_path)
    record=scene['collection_config']['floating_objects'];obj=record['instances'][1]
    if key=='target_instance_id':record[key]=obj['instance_id']
    elif key=='color':obj['variation']['color']='blue'
    elif key=='position':obj['initial_state']['position_body_m'][0]+=.01
    else:obj['type_task']='floating_aviation_plug_capture'
    with pytest.raises(ValueError,match='Saved floating objects'):
        write_model_variant(MODEL,tmp_path/'tampered.xml',scene)


def test_all_objects_are_initialized(tmp_path):
    _,scene,_=realize(tmp_path,3)
    calls={}
    def body(name):
        calls[name]={}
        return SimpleNamespace(**{method:(lambda value,m=method: calls[name].update({m:np.asarray(value)})) for method in ('setPosition','setVelocity','setAttitude','setAttitudeRate')})
    native=SimpleNamespace(_quaternion_to_mrp=lambda q:q)
    orbit=dict(initial_position_m=[100,200,300],initial_velocity_m_s=[1,2,3])
    initialize_floating_body(SimpleNamespace(getBody=body),orbit,native,scene['collection_config'])
    for obj in scene['collection_config']['floating_objects']['instances']:
        np.testing.assert_allclose(calls[obj['instance_id']]['setPosition'],np.asarray(obj['initial_state']['position_body_m'])+orbit['initial_position_m'])
        np.testing.assert_allclose(calls[obj['instance_id']]['setVelocity'],orbit['initial_velocity_m_s'])
        assert len(calls[obj['instance_id']])==4


def test_non_target_contact_fails_even_without_deep_penetration(tmp_path):
    _,scene,model=realize(tmp_path)
    from simulation.scripted_tasks.floating_grasps import grasp_candidates
    scene['collection_config']['floating_grasp'] = {'candidate': grasp_candidates('floating_plug_capture')[0].snapshot()}
    task=create_floating_task('floating_plug_capture',model,scene)
    sample=sample_at_grasp(task,scene)
    task.step(replace(sample,non_target_contacts=[dict(bodies=['ee_finger1','floating_distractor_01'],normal_force_n=.1,distance_m=-.00001)]))
    assert task.result=='failed' and task.reason=='non_target_contact'


def test_default_zero_only_when_no_explicit_velocity():
    scene=dict(floating_scene=dict(object_count=2))
    assert floating_release_options(scene).max_speed_m_s==0.
    scene['floating_capture']=dict(min_speed_m_s=.0005,min_spin_rad_s=.001)
    options=floating_release_options(scene)
    assert options.min_speed_m_s==.0005 and options.max_speed_m_s==.001
    assert options.max_spin_rad_s==.003
    configured=apply_floating_options(scene,parse('--object-speed','.03','--object-spin','.3'))
    assert floating_release_options(configured).min_speed_m_s==.03
    assert floating_release_options(configured).max_spin_rad_s==.3


@pytest.mark.parametrize('options',[dict(object_count=0),dict(object_count=2.5),dict(object_count=True),dict(object_count=8,composition='same-type-colors'),dict(object_count=8,composition='different-types'),dict(distractor_types=['fake']),dict(composition='same-type-colors',distractor_types=['floating_plug_capture'])])
def test_invalid_compositions(options):
    with pytest.raises(ValueError):FloatingSceneConfig.model_validate(options)


def test_object_count_cli_and_prepared_rejection():
    args=parse('--object-count','3','--object-composition','same-type-colors')
    assert apply_floating_options({},args)['floating_scene']['object_count']==3
    with pytest.raises(ValueError,match='prepared'):apply_floating_options({},args,prepared=True)
    args.task='drawer_cycle'
    with pytest.raises(ValueError,match='floating capture'):apply_floating_options({},args)


def test_recording_identity_and_instruction_binding(tmp_path):
    _,scene,_=realize(tmp_path)
    cfg=scene['collection_config'];manifest=cfg['floating_objects']
    objects=[dict(instance_id=o['instance_id'],type_task=o['type_task'],color=o['variation']['color'],
        position_body_m=o['initial_state']['position_body_m'],orientation_wxyz=o['initial_state']['orientation_wxyz'],
        velocity_body_m_s=[0.]*3,angular_velocity_body_rad_s=[0.]*3,finger_contacts=[0,0],finger_normal_force_n=[0.,0.]) for o in manifest['instances']]
    task=dict(target_instance_id=manifest['target_instance_id'],target_body=manifest['target_instance_id'],non_target_contacts=[],floating_objects=objects)
    instruction=task_instruction(scene,'floating_plug_capture',None)
    validate_floating_object_recording(cfg,[dict(scripted_task=task)],instruction)
    with pytest.raises(AssertionError):validate_floating_object_recording(cfg,[dict(scripted_task=task)],instruction.replace('blue','red'))
    task['floating_objects'][1]['finger_contacts']=[1,0]
    with pytest.raises(AssertionError):validate_floating_object_recording(cfg,[dict(scripted_task=task)],instruction)


def test_native_reader_detects_shallow_distractor_contact_and_reads_all_instances(tmp_path):
    _,scene,model=realize(tmp_path)
    from simulation.scripted_tasks.floating_grasps import grasp_candidates
    scene['collection_config']['floating_grasp'] = {'candidate': grasp_candidates('floating_plug_capture')[0].snapshot()}
    task=create_floating_task('floating_plug_capture',model,scene)
    names=['cubesat_bus','cylindrical_plug','floating_distractor_01','ee_finger1']
    state=np.zeros(21);state[[3,10,17]]=1.
    state[7:10]=[.4,.3,.3];state[14:17]=[.2,.4,.3]
    contacts=np.array([[0,1,2,3,-.00001,.1]])
    stepper=SimpleNamespace(nbody=4,_name=lambda kind,i:names[i],qpos=state,qvel=np.zeros(18),
        contact_snapshot=lambda:contacts[:,:5],contact_force_snapshot=lambda:contacts)
    def get_body(name):
        index=names.index(name)
        return SimpleNamespace(getFreeJoint=lambda:SimpleNamespace(getQposAdr=lambda:index*7,getQvelAdr=lambda:index*6))
    reader=FloatingState(task,SimpleNamespace(getBody=get_body),stepper)
    wrench=dict(force_n=[0.]*3,torque_nm=[0.]*3,valid=True)
    sample=reader.sample(0.,np.zeros(8),np.zeros(8),wrench)
    assert len(sample.floating_objects)==2 and sample.non_target_contacts[0]['normal_force_n']==.1
    assert sample.floating_objects[1]['finger_contacts']==[1,0]
    np.testing.assert_array_equal(sample.floating_objects[1]['position_body_m'],[.2,.4,.3])
    contacts[0,5]=.0001
    assert not reader.sample(0.,np.zeros(8),np.zeros(8),wrench).non_target_contacts


@pytest.mark.parametrize("composition", ["different-types", "same-type-colors"])
def test_recording_composition_tamper_rejected(tmp_path, composition):
    _,scene,_=realize(tmp_path,2,composition)
    cfg=scene["collection_config"];objects=cfg["floating_objects"]["instances"]
    instruction=task_instruction(scene,"floating_plug_capture",None)
    validate_floating_object_recording(cfg,[],instruction)
    if composition=="different-types":
        objects[1]["type_task"]=objects[0]["type_task"]
        objects[1]["variation"]["color"]="red"
    else:
        objects[1]["type_task"]="floating_aviation_plug_capture"
    with pytest.raises(AssertionError):
        validate_floating_object_recording(cfg,[],instruction)


@pytest.mark.parametrize("composition", ["different-types", "same-type-colors"])
@pytest.mark.parametrize("color", ["blue", None])
def test_recording_rejects_repeated_or_missing_multi_object_color(tmp_path, composition, color):
    _,scene,_=realize(tmp_path,2,composition)
    cfg=scene["collection_config"]
    instruction=task_instruction(scene,"floating_plug_capture",None)
    cfg["floating_objects"]["instances"][1]["variation"]["color"]=color
    with pytest.raises(AssertionError):
        validate_floating_object_recording(cfg,[],instruction)
