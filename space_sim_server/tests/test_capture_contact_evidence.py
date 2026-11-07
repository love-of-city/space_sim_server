"""Independent capture force replay: no controller labels or aggregate shortcuts."""
import copy
import hashlib
import json

import numpy as np
import pytest

from simulation.capture_audit import CaptureAudit
from simulation.capture_contact_evidence import (
    CaptureContactScorer, geometry_from_model, physical_contact_metrics, score_capture_contacts,
)


@pytest.fixture
def model(tmp_path):
    path = tmp_path/'model.xml'
    jaws = ''.join(f'<body name="jaw_{i}"><geom name="jaw_geom_{i}"/></body>' for i in range(4))
    path.write_text('<mujoco><worldbody><body name="cubesat_bus"><geom name="bus"/>'
        '<body name="gripper"><geom name="collision_ring_convex_0"/>'+jaws+'</body></body>'
        '<body name="capture_target"><body name="target_child"><geom name="capture_x_piece_0"/>'
        '<geom name="wrong_target_surface"/></body></body></worldbody></mujoco>', encoding='utf-8')
    return path


def contact(jaw=0, normal=1.25, *, target='capture_x_piece_0', constraint=None):
    return dict(geoms=[f'jaw_geom_{jaw}',target], bodies=[f'jaw_{jaw}','target_child'],
        roots=['cubesat_bus','capture_target'],distance_m=-.0001,position_local_m=[0.,0.,0.],
        force_contact=[normal,0.,0.,0.,0.,0.],frame=[0.,0.,1., 1.,0.,0., 0.,1.,0.],
        efc_address=jaw if constraint is None else constraint,native_time_ns=0)


def row(tick, contacts=None):
    contacts = [contact(i) for i in range(4)] if contacts is None else contacts
    for item in contacts:
        item['native_time_ns'] = tick*2_500_000
    return dict(schema_version='space-sim-capture-contact-evidence/1',sequence=tick,
        time_ns=8_000_000_000_000+tick*2_500_000,time_s=8000.+tick*.0025,
        native_time_ns=tick*2_500_000,external_state_writes=0,gripper_axis_world=[0.,0.,1.],
        physics_time_ns=tick*2_500_000,physics_step_count=tick,physics_substeps=1,
        gripper_quaternion_wxyz=[1.,0.,0.,0.],
        native_contact_count=len(contacts),raw_contacts=contacts,
        # These fields intentionally lie. The independent grader must ignore
        # the controller's phase, feedback and claimed self-contact count.
        phase='abort',feedback={'jaw_support_count':0,'jaw_force_n':0.},self_contact_count=123)


def score_rows(model, modifier=None):
    scorer = CaptureContactScorer(geometry_from_model(model))
    for tick in range(8001):
        value = row(tick)
        if modifier:
            modifier(value,tick)
        scorer.add(value)
    return scorer.result()


def test_true_late_four_jaw_force_passes_independently_of_controller_phase_or_reported_feedback(model):
    result = score_rows(model)
    assert result['physical_hold_passed'] and result['longest_hold_s'] == 20.
    assert result['sample_count'] == 8001 and result['final_contact_metrics']['jaw_force_n'] == 5.
    assert not result['safety_violations'] and not result['evidence_errors']
    assert 'success' not in result  # A contact sub-audit is not mission acceptance.


@pytest.mark.parametrize('kind',['one_jaw_four_patches','pad_only','wrong_target_feature','inactive_constraint','force_excursion'])
def test_raw_contact_geometry_and_every_physics_tick_override_claimed_success(model,kind):
    def modify(value,tick):
        value.update(phase='hold_force',feedback={'jaw_support_count':4,'jaw_force_n':5.})
        if kind == 'one_jaw_four_patches':
            value['raw_contacts'] = [contact(0,constraint=i) for i in range(4)]
        elif kind == 'pad_only':
            for item in value['raw_contacts']:
                item['geoms'][0],item['bodies'][0] = 'collision_ring_convex_0','gripper'
        elif kind == 'wrong_target_feature':
            value['raw_contacts'] = [contact(i,target='wrong_target_surface') for i in range(4)]
        elif kind == 'inactive_constraint':
            value['raw_contacts'] = [contact(i,constraint=-1) for i in range(4)]
        elif tick == 4001:  # One 400 Hz excursion between 10 Hz image times.
            value['raw_contacts'] = [contact(i,normal=1.5) for i in range(4)]
        for item in value['raw_contacts']:
            item['native_time_ns'] = value['native_time_ns']
    result = score_rows(model,modify)
    assert not result['physical_hold_passed'] and result['longest_hold_s'] < 20.


@pytest.mark.parametrize('change',[
    {'sequence':True},{'sequence':0},{'time_ns':8_000_000_000_000},
    {'time_ns':8_000_005_000_000},{'native_time_ns':0},{'external_state_writes':1},
    {'gripper_axis_world':[0.,0.,2.]},{'native_contact_count':3},{'time_s':8001.},
    {'schema_version':'aggregate-only'},{'raw_contacts':None},
    {'physics_time_ns':0},{'physics_step_count':0},{'physics_substeps':2},
    {'gripper_quaternion_wxyz':[0.,1.,0.,0.]},
])
def test_corrupt_missing_duplicate_stale_and_overwritten_evidence_cannot_accumulate_hold(model,change):
    scorer = CaptureContactScorer(geometry_from_model(model))
    scorer.add(row(0))
    with pytest.raises(ValueError,match='invalid raw contact evidence'):
        scorer.add(row(1)|change)
    assert not scorer.result()['physical_hold_passed'] and scorer.result()['evidence_errors']


@pytest.mark.parametrize('key,value',[
    ('force_contact',[float('nan'),0.,0.,0.,0.,0.]),('force_contact',[-1.,0.,0.,0.,0.,0.]),
    ('geoms',['unknown','capture_x_piece_0']),('bodies',['jaw_3','target_child']),
    ('roots',['cubesat_bus','cubesat_bus']),('frame',[0.]*9),('efc_address',True),
    ('efc_address',1),('native_time_ns',2_500_000),('geoms',[{},'capture_x_piece_0']),
])
def test_contact_values_and_roles_are_bound_to_the_actual_model(model,key,value):
    sample = row(0)
    sample['raw_contacts'][0][key] = value
    scorer = CaptureContactScorer(geometry_from_model(model))
    with pytest.raises(ValueError):
        scorer.add(sample)
    assert not scorer.result()['physical_hold_passed']


@pytest.mark.parametrize('kind',['self_contact','peak_target_force'])
def test_single_safety_violation_vetoes_later_twenty_second_physical_hold(model,kind):
    scorer = CaptureContactScorer(geometry_from_model(model))
    for tick in range(8003):
        value = row(tick)
        if tick == 1:
            if kind == 'peak_target_force':
                value['raw_contacts'].append(contact(0,normal=11.,target='wrong_target_surface',constraint=4))
            else:
                item = contact(0,constraint=4)
                item.update(geoms=['jaw_geom_0','bus'],bodies=['jaw_0','cubesat_bus'],
                    roots=['cubesat_bus','cubesat_bus'],distance_m=-.0002)
                value['raw_contacts'].append(item)
            value['native_contact_count'] = len(value['raw_contacts'])
            for item in value['raw_contacts']:
                item['native_time_ns'] = value['native_time_ns']
        scorer.add(value)
    result = scorer.result()
    assert not result['physical_hold_passed'] and result['safety_violations']


def test_force_projection_uses_physical_contact_normal_and_distinct_jaw_bodies(model):
    value = row(0)
    for item in value['raw_contacts']:
        item['frame'] = [.6,0.,.8, 0.,1.,0., -.8,0.,.6]
    metrics = physical_contact_metrics(value,geometry_from_model(model))
    assert metrics['jaw_force_n'] == pytest.approx(4.) and metrics['jaw_support_count'] == 4
    assert metrics['jaw_force_by_body_n'] == {f'jaw_{i}':1. for i in range(4)}


def test_long_native_float_clock_retains_readback_without_adding_to_integer_hold_time(model):
    scorer = CaptureContactScorer(geometry_from_model(model))
    for tick in range(3):
        sample = row(tick)
        sample['physics_time_ns'] += 8_000_000_000_000
        sample['physics_step_count'] += 3_200_000
        # A late native clock has accumulated sub-microsecond rounding. It is
        # stored as read, and must not contribute any extra dwell credit.
        sample['native_time_ns'] = sample['physics_time_ns']+100
        for item in sample['raw_contacts']:
            item['native_time_ns'] = sample['native_time_ns']
        scorer.add(sample)
    assert scorer.result()['longest_hold_s'] == .005
    assert not scorer.result()['physical_hold_passed']
    stale = row(3)
    stale.update(physics_time_ns=8_000_007_500_000,physics_step_count=3_200_003,
        native_time_ns=8_000_005_000_100)
    with pytest.raises(ValueError,match='native contact clock'):
        scorer.add(stale)


@pytest.mark.parametrize('text',[
    '<body><geom name="g"/></body>',
    '<body name="b"><geom/></body>',
    '<body name="b"/><body name="b"/>',
    '<body name="b"><geom name="g"/><geom name="g"/></body>',
    '<include file="other.xml"/>',
    '<geom name="floor"/>',
])
def test_ambiguous_or_unexpanded_model_cannot_supply_contact_roles(tmp_path,text):
    path = tmp_path/'model.xml'
    path.write_text('<mujoco><worldbody>'+text+'</worldbody></mujoco>')
    with pytest.raises(ValueError):
        geometry_from_model(path)


def test_audit_preserves_raw_native_contacts_after_body_cap_and_does_not_change_feedback(model,tmp_path):
    class Stepper:
        external_writes = 0
        time = 0.
        substeps = 1
        step_count = 0
        _last_nanos = 0
        def body_detail(self,name):
            return np.array([0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,self.time])
        def _name(self,kind,index):
            return [*[f'jaw_geom_{i}' for i in range(4)],'capture_x_piece_0'][index]
        def contact_detail(self):
            result = np.zeros((4,23))
            for index,item in enumerate(result):
                item[:3]=[index,4,-.0001];item[6]=1.25;item[12:21]=[0.,0.,1.,1.,0.,0.,0.,1.,0.];item[21]=index
                item[22] = self.time
            return result
    stepper = Stepper()
    path = tmp_path/'contacts.jsonl'
    audit = CaptureAudit(model,stepper,output_path=path)
    try:
        for tick in range(3):
            stepper.time = tick*.0025
            stepper.step_count = tick
            stepper._last_nanos = tick*2_500_000
            audit.record(8000.+stepper.time,'not_a_success_label',np.zeros(12))
            assert audit.feedback['jaw_support_count'] == 4 and audit.feedback['jaw_force_n'] == 5.
    finally:
        audit.file.close();audit.force_file.close()
    raw = tmp_path/'contacts_force.jsonl'
    manifest = tmp_path/'contacts_manifest.json'
    before = {path:(hashlib.sha256(path.read_bytes()).hexdigest(),path.stat().st_mtime_ns) for path in (model,raw,manifest)}
    result = score_capture_contacts(raw,model,manifest)
    assert result['sample_count'] == 3 and result['longest_hold_s'] == .005 and not result['physical_hold_passed']
    assert result['final_contact_metrics']['jaw_force_n'] == audit.feedback['jaw_force_n']
    assert before == {path:(hashlib.sha256(path.read_bytes()).hexdigest(),path.stat().st_mtime_ns) for path in before}
    assert len(path.read_text().splitlines()) == 1  # Detailed body dump remains capped.
    document = json.loads(manifest.read_text())
    bad = copy.deepcopy(document);bad['geometry']['jaw_geom_0']['body']='jaw_1'
    manifest.write_text(json.dumps(bad))
    with pytest.raises(ValueError,match='actual model'):
        score_capture_contacts(raw,model,manifest)
    manifest.write_text('[]')
    with pytest.raises(ValueError,match='actual model'):
        score_capture_contacts(raw,model,manifest)
    manifest.write_text(json.dumps(document))
    raw.write_text(raw.read_text().rstrip('\n'))
    with pytest.raises(ValueError,match='truncated'):
        score_capture_contacts(raw,model,manifest)
