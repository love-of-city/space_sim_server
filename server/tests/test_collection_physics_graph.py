"""Content changes must rebuild; repeated compatible tasks may reuse one graph."""
from types import SimpleNamespace

import pytest

from simulation.scripted_tasks.physics_graph import PhysicsGraphCache


def inputs(tmp_path):
    model = tmp_path / 'model.xml'
    model.write_text('<mujoco/>')
    return dict(model=model, args=SimpleNamespace(catalog=None, substeps=1,
                capture_control_port=None, capture_render_port=None),
                scene_instance={'environment': {'gravity': 'bsk'}, 'overview_camera': {'id': 1}},
                disturbances=SimpleNamespace(options=SimpleNamespace(
                    external_disturbance=SimpleNamespace(enabled=False))),
                orbital_physics=SimpleNamespace(drag=None))


@pytest.fixture
def cache_inputs(tmp_path, monkeypatch):
    from simulation import compiled_model_cache, mujoco_runtime
    monkeypatch.setattr(compiled_model_cache, 'model_cache_key', lambda p, runtime: p.read_text())
    monkeypatch.setattr(mujoco_runtime, 'basilisk_mujoco_runtime', lambda: tmp_path/'runtime')
    return inputs(tmp_path)


def retain(cache):
    closed = []
    cache.state = dict(key=cache.pending_key, capture=None,
                      sim=SimpleNamespace(local_stepper=SimpleNamespace(close=lambda: closed.append('core'))),
                      orbit=SimpleNamespace(gravity_factory=SimpleNamespace(
                          unloadSpiceKernels=lambda: closed.append('kernels'))))
    return closed


@pytest.mark.parametrize('change', ['model', 'camera', 'environment', 'ports', 'attitude', 'substeps'])
def test_changed_inputs_discard_the_old_graph(cache_inputs, change):
    values = cache_inputs
    cache = PhysicsGraphCache()
    assert not cache.compatible(values)
    closed = retain(cache)
    assert cache.compatible(values)
    if change == 'model':
        values['model'].write_text('<mujoco changed="true"/>')
    elif change == 'camera':
        values['scene_instance']['overview_camera']['id'] = 2
    elif change == 'environment':
        values['scene_instance']['environment']['gravity'] = 'other'
    elif change == 'ports':
        values['args'].capture_control_port = 12345
    elif change == 'attitude':
        values['model'].with_name('attitude_control.json').write_text('{"rate": 20}')
    else:
        values['args'].substeps = 2
    assert not cache.compatible(values)
    assert closed == ['core', 'kernels']
    assert cache.state is None
    cache.close()
    assert closed == ['core', 'kernels']


def test_new_task_seed_does_not_invalidate_compatible_graph(cache_inputs):
    cache = PhysicsGraphCache()
    assert not cache.compatible(cache_inputs)
    closed = retain(cache)
    cache_inputs['scene_instance'].update(seed=999, scripted_task_name='guide_plug_insert')
    assert cache.compatible(cache_inputs)
    assert not closed
    cache.close()


def test_episode_specific_force_wiring_disables_graph_reuse(cache_inputs):
    cache = PhysicsGraphCache()
    assert not cache.compatible(cache_inputs)
    closed = retain(cache)
    cache_inputs['disturbances'].options.external_disturbance.enabled = True
    assert not cache.compatible(cache_inputs)
    assert cache.pending_key is None
    assert closed == ['core', 'kernels']
