"""Exercise the actual run entrypoint: realised colours must reach the renderer."""
import hashlib
import json
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from tools import run_scripted_task as runner


def layout_model(tmp_path):
    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', meshdir='meshes')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='surface', file='surface.obj')
    ET.SubElement(asset, 'material', name='old', rgba='.3 .3 .3 1')
    world = ET.SubElement(root, 'worldbody')
    names = ['bolt', 'layout_bolt_storage_hole', 'layout_bolt_assembly_hole']
    for kind in ('aviation', 'guide', 'cylindrical'):
        names += [kind + '_plug', 'layout_' + kind + '_pickup',
                  'layout_' + kind + '_insert_40p1', 'layout_' + kind + '_insert_40p5']
    for name in names:
        body = ET.SubElement(world, 'body', name=name)
        visual_name = name + ('_geom' if name in ('bolt', 'aviation_plug', 'guide_plug', 'cylindrical_plug') else '_visual')
        ET.SubElement(body, 'geom', name=visual_name, mesh='surface', material='old',
                      group='2', contype='0', conaffinity='0', mass='0')
        ET.SubElement(body, 'geom', name=name+'_collision', type='box', size='.01 .01 .01',
                      group='3', rgba='0 0 0 0', friction='.4 .001 .001')
    model = tmp_path / 'layout.xml'
    ET.ElementTree(root).write(model)
    return model


@pytest.mark.parametrize('task_name', ['drawer_cycle', 'guide_plug_insert'])
def test_run_passes_realized_model_to_capture_and_preserves_catalog_identity(tmp_path, monkeypatch, task_name):
    from simulation.scripted_tasks.drawer_lift import DrawerLiftConfig
    from simulation.scripted_tasks import native_state
    from bsk_render_adapter import mjcf_assets
    from bsk_render_adapter.bridge import _mj_mesh_geometry
    from simulation.socket_colors import COLOR_PALETTE

    model = layout_model(tmp_path)
    source_bytes = model.read_bytes()
    scene_path = tmp_path / 'scene.json'
    scene_path.write_text(json.dumps(dict(template_id='sarm-task-box-20261012', seed=42,
        randomization=dict(arm_joint_position_rad=[0.] * 8))))
    output = tmp_path / 'run'
    args = SimpleNamespace(task=task_name, model=model, scene=scene_path, output=output,
        initial_joints_deg=None, lift_m=.08, grip_offset_m=(0, 0, -.005), force_limit=80,
        arm_speed=None, linear_speed=None, target_site='guide_insert_40p5',
        plug_grasp_offset=(0, 0, -.003), substeps=1, capture_control_port=123,
        capture_render_port=124, catalog=tmp_path/'catalog.json')
    catalog_asset = '/Game/Test/Surface.Surface'
    args.catalog.write_text(json.dumps({'assets': {str(tmp_path/'meshes/surface.obj'): catalog_asset}}))
    task = SimpleNamespace(lower=np.full(6, -10.), upper=np.full(6, 10.),
                           config=DrawerLiftConfig(), events=[])
    monkeypatch.setattr('simulation.scripted_tasks.drawer_cycle.DrawerCycleTask', lambda *_: task)
    monkeypatch.setattr('simulation.scripted_tasks.plug_insert.PlugInsertTask', lambda *_: task)
    monkeypatch.setattr('simulation.sarm_model_profile.load_model_profile', lambda _: SimpleNamespace(
        joints=[], joint_names=['joint']*6, arm_actuators=[]))
    stepper = SimpleNamespace(publish_every=1, close=lambda: None)
    sim = SimpleNamespace(local_stepper=stepper)
    native = SimpleNamespace(__file__=runner.__file__)
    physical_models = []

    def build(**kwargs):
        physical_models.append(native.MODEL_PATH)
        return sim, object(), [], []

    native._build_simulation = build
    monkeypatch.setattr('scenario_spacecraft_arm_grasp_unreal.load_native_grasp_module', lambda _: native)
    monkeypatch.setattr('simulation.scripted_tasks.disturbances.EpisodeDisturbances',
                        lambda _: SimpleNamespace(install_external_force=lambda _: None))
    monkeypatch.setattr('simulation.assembly.AssemblyContext', lambda sim, scene, config:
                        SimpleNamespace(simulation=sim, scene=scene, config=config, install=lambda _: None, extra={}))
    for module, name in [('clock', 'ClockComponent'), ('dynamics_core', 'DynamicsCoreComponent'),
                         ('orbital_gravity', 'OrbitalGravityComponent'), ('arm_reference', 'JointReferenceComponent'),
                         ('attitude', 'AttitudeComponent')]:
        monkeypatch.setattr('simulation.components.'+module+'.'+name, lambda *_, **__: None)
    monkeypatch.setattr('simulation.components.orbit.OrbitComponent', lambda **kwargs:
                        SimpleNamespace(earth=None, epoch_utc=kwargs['epoch_utc'], gravity_factory=None))
    monkeypatch.setattr('simulation.components.plug_retention.install_configured_plug_retention', lambda _: None)
    monkeypatch.setattr('simulation.orbital_configuration.resolve_orbital_physics', lambda _: SimpleNamespace(drag=None))
    monkeypatch.setattr('simulation.orbital_configuration.environment_metadata', lambda *_, **__: {})
    monkeypatch.setitem(native_state.STATE_READERS, task_name, lambda *_: None)
    received = {}

    class CaptureReached(RuntimeError):
        pass

    def capture(ctx, task, profile, command, model_hash, scene, **kwargs):
        received.update(kwargs, model_hash=model_hash, scene=scene, physics_model=ctx.config.model_path)
        raise CaptureReached('transport boundary reached; no simulation or UE started')

    monkeypatch.setattr('simulation.scripted_tasks.capture_transport.ScriptedCaptureTransport', capture)
    with pytest.raises(CaptureReached):
        runner.run(args)
    variant = output / 'model.xml'
    assert physical_models == [variant]
    assert received['physics_model'] == received['render_model_path'] == variant
    assert received['catalog'] == args.catalog
    assert received['model_hash'] == hashlib.sha256(variant.read_bytes()).hexdigest()
    report = json.loads((output / 'summary.json').read_text())
    assert report['render_model_path'] == str(variant)
    assert report['render_model_sha256'] == report['model_sha256'] == received['model_hash']
    assert model.read_bytes() == source_bytes
    source_root, variant_root = ET.fromstring(source_bytes), ET.parse(variant).getroot()
    assert [ET.tostring(g) for g in source_root.findall('.//geom[@group="3"]')] == [
        ET.tostring(g) for g in variant_root.findall('.//geom[@group="3"]')]
    assert [(g.tag, g.get('name')) for g in source_root.findall('.//geom')] == [
        (g.tag, g.get('name')) for g in variant_root.findall('.//geom')]
    # Actual adapter parsing/catalog resolution uses the same mesh files and
    # geometry IDs after moving the colour variant to a different directory.
    monkeypatch.setattr(mjcf_assets, '_compiled_source_mesh_poses', lambda _: [])
    old, new = (mjcf_assets.parse_mjcf_geometry_metadata(p) for p in (model, variant))
    catalog = mjcf_assets.load_asset_catalog(args.catalog)
    assert [(m.body_name, m.mesh_name, m.source_path) for m in old] == [
        (m.body_name, m.mesh_name, m.source_path) for m in new]
    colors = received['scene']['collection_config']['color_randomization']
    checked = 0
    for index, metadata in enumerate(new):
        hole = metadata.body_name.removeprefix('layout_')
        if not metadata.mesh_name or hole not in colors['hole_colors']:
            continue
        geometry = _mj_mesh_geometry(SimpleNamespace(rgba=[.3, .3, .3, 1]),
            f'teleop/{metadata.body_name}/geom/{index}', metadata, catalog, {})
        assert geometry.asset_path == catalog_asset
        assert geometry.geometry_id == f'teleop/{old[index].body_name}/geom/{index}'
        hex_color = COLOR_PALETTE[colors['hole_colors'][hole]]['srgb_hex']
        srgb = [int(hex_color[i:i+2], 16) / 255 for i in (1, 3, 5)]
        expected = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4 for v in srgb] + [1.]
        np.testing.assert_allclose(geometry.color_rgba, expected, rtol=1e-8)
        checked += 1
    assert checked == 7
