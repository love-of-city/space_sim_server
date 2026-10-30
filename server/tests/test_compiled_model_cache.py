from pathlib import Path

from simulation.compiled_model_cache import model_cache_key


def test_key_tracks_model_content_and_asset_content_across_episode_paths(tmp_path):
    runtime = tmp_path / 'runtime.so'
    runtime.write_bytes(b'runtime')
    mesh = tmp_path / 'mesh.obj'
    mesh.write_text('mesh one')
    xml = f'<mujoco><asset><mesh file="{mesh}"/></asset></mujoco>'
    first, second = tmp_path / 'a.xml', tmp_path / 'b.xml'
    first.write_text(xml)
    second.write_text(xml)
    key = model_cache_key(first, runtime)
    assert key == model_cache_key(second, runtime)
    mesh.write_text('mesh two')
    assert model_cache_key(first, runtime) != key
    changed = model_cache_key(first, runtime)
    first.write_text(xml.replace('<mujoco>', '<mujoco model="changed">'))
    assert model_cache_key(first, runtime) != changed
    runtime.write_bytes(b'new runtime')
    assert model_cache_key(second, runtime) != changed


def test_unhandled_includes_plugins_and_resource_providers_are_not_cached(tmp_path):
    model = tmp_path / 'model.xml'
    for content in ('<include file="other.xml"/>', '<asset><mesh file="provider://mesh"/></asset>',
                    '<extension><plugin plugin="unknown"/></extension>', '<compiler strippath="true"/>'):
        model.write_text('<mujoco>' + content + '</mujoco>')
        assert model_cache_key(model, Path('/unused/runtime')) is None


def test_cache_reconstructs_fresh_state_and_servos(tmp_path):
    import numpy as np
    from test_local_mujoco_stepper import MJCF, build, run_to

    model = tmp_path / 'model.xml'
    model.write_text(MJCF.replace('local_stepper_test', 'cache_independence_test'))
    sim, scene, core, keep = build(model, 'local')
    assert core.model_compilation['cache_hit'] is False
    initial = np.asarray(scene.dynManager.getStateObject("mujocoQpos").getState()).copy()
    run_to(sim, .2)
    expected = np.asarray(scene.dynManager.getStateObject("mujocoQpos").getState()).copy()
    assert not np.array_equal(expected, initial)
    second, second_scene, second_core, second_keep = build(model, 'local')
    assert second_core.model_compilation['cache_hit'] is True
    assert second_core._handle.value != core._handle.value
    np.testing.assert_array_equal(second_scene.dynManager.getStateObject("mujocoQpos").getState(), initial)
    run_to(second, .2)
    np.testing.assert_array_equal(second_scene.dynManager.getStateObject("mujocoQpos").getState(), expected)
    model.write_text(model.read_text().replace('mass="10"', 'mass="11"'))
    third, third_scene, third_core, third_keep = build(model, 'local')
    assert third_core.model_compilation['cache_hit'] is False
    assert third_scene.getBody('bus').getMass() == 11.
