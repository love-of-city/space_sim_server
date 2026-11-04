"""Split skills keep physical reset, randomization and collision semantics."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.native_geometry_probe import NativePoseProbe
from simulation.scripted_tasks.task_initialization import randomized_scene
from simulation.scripted_tasks.native_state import DrawerState
from simulation.scripted_tasks.randomization import write_model_variant

MODEL = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_20261012.xml'
NEW_TASKS = ('drawer_open', 'drawer_close', 'locking_rod_fold', 'locking_rod_restore')


@pytest.mark.parametrize('task', NEW_TASKS)
def test_randomized_episode_keeps_direction_and_independent_physical_initial_state(tmp_path, task):
    source = {'template_id': 'sarm-task-box-20261012',
              'randomization': {'arm_joint_position_rad': [0.] * 8},
              'randomize_overview_camera': True}
    before = deepcopy(source)
    first = randomized_scene(source, 42, 0., task)
    assert first == randomized_scene(source, 42, 0., task)
    assert source == before
    cfg = first['collection_config']
    assert cfg['task'] == task and cfg['instruction_randomization']['task'] == task
    assert cfg['task_version'].startswith(task.replace('_', '-') + '/')
    original = MODEL.read_bytes()
    result = write_model_variant(MODEL, tmp_path / 'model.xml', first)
    root = ET.parse(result).getroot()
    def ref(name):
        return float(root.find(f'.//joint[@name="{name}"]').get('ref', '0'))
    assert ref('drawer_base_slide') == (.08 if task == 'drawer_close' else 0.)
    assert ref('locking_rod_hinge') == (-1.53 if task == 'locking_rod_restore' else 0.)
    assert MODEL.read_bytes() == original
    for state in cfg['model_randomization']['task_initialization'].values():
        assert state.get('runtime_state_writes', False) is False


def test_custom_return_opening_is_bound_to_preflight_and_execution_model(tmp_path):
    scene = {'scripted_task_name': 'drawer_close',
             'collection_config': {'drawer_initial_opening_m': .06}}
    result = write_model_variant(MODEL, tmp_path / 'model.xml', scene)
    assert float(ET.parse(result).find('.//joint[@name="drawer_base_slide"]').get('ref')) == .06


@pytest.mark.parametrize('task', ('drawer_cycle', 'drawer_lift'))
def test_legacy_drawer_start_resets_a_previously_generated_open_model(tmp_path, task):
    opened = write_model_variant(MODEL, tmp_path / 'open.xml', {'scripted_task_name': 'drawer_close'})
    reset = write_model_variant(opened, tmp_path / 'reset.xml', {'scripted_task_name': task})
    root = ET.parse(reset).getroot()
    assert float(root.find('.//joint[@name="drawer_base_slide"]').get('ref')) == 0.
    expected = ET.parse(MODEL).find('.//body[@name="drawer_base"]')
    np.testing.assert_allclose(np.fromstring(root.find('.//body[@name="drawer_base"]').get('pos'), sep=' '),
                               np.fromstring(expected.get('pos'), sep=' '), atol=1e-12)


@pytest.mark.parametrize('task,part,own', [
    ('locking_rod_fold', 'fastener', 'hook'),
    ('locking_rod_restore', 'locking_rod', 'limit_clip'),
    ('drawer_open', 'drawer_base', 'drawer_buckle'),
    ('drawer_close', 'drawer_base', 'drawer_buckle'),
])
def test_split_preflight_allows_own_mechanism_contact_but_rejects_neighbor_and_arm(task, part, own):
    probe = object.__new__(NativePoseProbe)
    probe.q = np.zeros(8)
    probe.home = probe.q.copy()
    probe.names = {0: part, 1: own, 2: 'neighbor_fixture', 3: 'wrist_ft_tool'}
    probe.contacts = lambda candidate: np.array([[0, 1, 0, 1, -.001]])
    assert probe.pose_collision(np.zeros(8), task, (part,)) is None
    probe.contacts = lambda candidate: np.array([[0, 1, 0, 2, -.001]])
    rejected = probe.pose_collision(np.zeros(8), task, (part,))
    assert rejected['collision_kind'] == 'mechanism_environment_collision'
    probe.contacts = lambda candidate: np.array([[0, 1, 0, 3, -.001]])
    rejected = probe.pose_collision(np.zeros(8), task, (part,))
    assert rejected['body2'] == 'wrist_ft_tool'


def test_native_drawer_metrics_override_diagnostic_aliases_with_measured_state():
    reader = object.__new__(DrawerState)
    reader.task = SimpleNamespace(_initial_drawer=.08,
        metrics=lambda sample: {'drawer_position_m': 100., 'drawer_target_m': 0.})
    sample = SimpleNamespace(drawer_m=.01, drawer_velocity_m_s=-.02,
        buckle_rad=.1, finger_drawer_contacts=(2, 2))
    values = reader.metrics(sample)
    assert values['drawer_position_m'] == .01
    assert values['drawer_displacement_m'] == pytest.approx(-.07)
    assert values['drawer_target_m'] == 0.
