"""Condition semantics, source isolation and replay across JSON round trips."""
from copy import deepcopy
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from test_floating_capture import MODEL, source_scene
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.floating_profiles import FLOATING_PROFILES, apply_floating_profile
from simulation.scripted_tasks.floating_variations import apply_target_variations
from simulation.scripted_tasks.randomization import write_model_variant, prepare_episode


@pytest.mark.parametrize('profile', FLOATING_PROFILES)
def test_profile_replay_and_source_isolation(tmp_path, profile):
    source = source_scene()
    before = deepcopy(source)
    scene = randomized_scene(apply_floating_profile(source, profile), 75, 0, 'floating_aviation_plug_capture')
    path = write_model_variant(MODEL, tmp_path/'model.xml', scene)
    serialized = json.loads(json.dumps(scene))
    prepare_episode(serialized)
    replay = write_model_variant(MODEL, tmp_path/'replay.xml', serialized)
    assert path.read_bytes() == replay.read_bytes()
    assert source == before
    state = scene['collection_config']['floating_initial_state']
    assert state['schema'] == 'floating-release/3'
    assert state['options']['min_speed_m_s'] <= np.linalg.norm(state['velocity_body_m_s'])+1e-12
    root = ET.parse(path).getroot()
    original = ET.parse(MODEL).getroot()
    for name in ('cubesat_bus', 'task_box_base'):
        # Root attrs and all geometry stay identical, except intentionally detached targets.
        assert root.find(f'.//body[@name="{name}"]').attrib == original.find(f'.//body[@name="{name}"]').attrib
    color = scene['collection_config']['floating_target_variation']['color']
    if color:
        visual = root.find('.//body[@name="aviation_plug"]/geom[@group="2"]')
        assert visual.get('material') == 'floating_target_color_'+color
        for geom in root.findall('.//body[@name="aviation_plug"]/geom[@group="3"]'):
            old = original.find(f'.//geom[@name="{geom.get("name")}"]')
            assert geom.get('rgba') == old.get('rgba')


def test_initialization_disabled_wins():
    source = apply_floating_profile(source_scene(), 'right-incoming-blue')
    source['initialization_enabled'] = False
    scene = randomized_scene(source, 75, 30, 'floating_bolt_capture')
    assert scene['collection_config']['initialization']['offset_deg'] == [0.]*6


@pytest.mark.parametrize('key', ['velocity_body_m_s', 'angular_velocity_body_rad_s'])
def test_replay_rejects_reversed_direction(tmp_path, key):
    scene = randomized_scene(apply_floating_profile(source_scene(), 'right-incoming-blue'), 75, 0, 'floating_aviation_plug_capture')
    write_model_variant(MODEL, tmp_path/'a.xml', scene)
    scene['collection_config']['floating_initial_state'][key] = [-v for v in scene['collection_config']['floating_initial_state'][key]]
    with pytest.raises(ValueError):
        write_model_variant(MODEL, tmp_path/'b.xml', scene)


def test_inherited_mass_and_friction_do_not_change_other_bodies():
    xml = '''<mujoco><compiler inertiafromgeom="auto"/><default><geom density="800" friction="2 .02 .001"/><default class="nested"><geom mass="3"/></default></default><asset/><worldbody><body name="robot"><geom name="robot_geom"/></body><body name="target" childclass="nested"><geom name="target_geom"/><geom name="zero" mass="0" density="0"/></body></worldbody><contact><pair name="pair" geom1="target_geom" geom2="robot_geom" friction="3 3 .01 .001 .001"/></contact></mujoco>'''
    root = ET.fromstring(xml)
    before = ET.tostring(root.find('.//body[@name="robot"]'))
    scene = dict(seed=1, floating_target=dict(mass_scale_range=[1.2,1.2], friction_scale_range=[.8,.8]), collection_config={})
    apply_target_variations(root, scene, 'target')
    geom = root.find('.//geom[@name="target_geom"]')
    assert float(geom.get('mass')) == pytest.approx(3.6)
    assert root.find('.//geom[@name="zero"]').get('mass') == '0'
    assert np.fromstring(geom.get('friction'),sep=' ')[0] == pytest.approx(1.6)
    assert np.fromstring(root.find('./contact/pair').get('friction'),sep=' ')[0] == pytest.approx(2.4)
    assert ET.tostring(root.find('.//body[@name="robot"]')) == before


def test_explicit_inertia_scaling_and_saved_variation_tamper():
    xml = '<mujoco><asset/><worldbody><body name="target"><inertial mass="2" pos="1 2 3" diaginertia=".1 .2 .3"/><geom name="g" density="5"/></body></worldbody></mujoco>'
    scene = dict(seed=3,floating_target=dict(mass_scale_range=[.8,.8]),collection_config={})
    root = ET.fromstring(xml)
    apply_target_variations(root,scene,'target')
    inertia=root.find('.//inertial')
    assert inertia.get('pos') == '1 2 3'
    assert float(inertia.get('mass')) == 1.6
    np.testing.assert_allclose(np.fromstring(inertia.get('diaginertia'),sep=' '), [.08,.16,.24])
    assert root.find('.//geom').get('density') == '5'
    scene['collection_config']['floating_target_variation']['mass_scale']=1.1
    with pytest.raises(ValueError,match='Saved floating target'):
        apply_target_variations(ET.fromstring(xml),scene,'target')
