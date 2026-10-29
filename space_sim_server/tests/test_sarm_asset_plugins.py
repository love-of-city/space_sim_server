"""Asset metadata extraction must not load a second MuJoCo/plugin runtime."""
import json
from pathlib import Path
import subprocess
import sys

import pytest


def test_catalog_generator_reads_geometry_without_compiling_plugin_physics(tmp_path):
    root = Path(__file__).resolve().parents[1]
    manifest = root / 'run/native_acceleration/sarm_thread.json'
    if not manifest.is_file():
        pytest.skip('Requires tools/assets/build_sarm_plugin.py on this machine')
    mujoco = pytest.importorskip('mujoco')
    record = json.loads(manifest.read_text(encoding='utf-8-sig'))
    from simulation.mujoco_plugins import validate_plugin_manifest
    library = root / record['library']
    validate_plugin_manifest(library, manifest, mujoco.mj_version())
    mesh = tmp_path / 'tetra.obj'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n'
                    'f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    model = tmp_path / 'model.xml'
    model.write_text('<mujoco><extension><plugin plugin="sarm.sdf.two_start_thread"/>'
                     '</extension><asset><mesh name="tetra" file="tetra.obj"/></asset>'
                     '<worldbody><body name="part"><geom type="mesh" mesh="tetra"/>'
                     '</body></worldbody></mujoco>')
    generator = root.parent / 'space_sim_UE_Adapter/Unreal/BskUnrealRenderer/scripts/generate_mjcf_asset_catalog.py'
    command = [sys.executable, str(generator), '--mjcf', str(model), '--destination', '/Game/Test',
               '--catalog', str(tmp_path / 'catalog.json'), '--import-settings', str(tmp_path / 'import.json')]
    missing = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert missing.returncode == 0, missing.stderr
    expected = json.loads((tmp_path / 'catalog.json').read_text())
    loaded = subprocess.run(command + ['--mujoco-plugin', str(library)], capture_output=True,
                            text=True, timeout=30)
    assert loaded.returncode == 0, loaded.stderr
    assert json.loads((tmp_path / 'catalog.json').read_text()) == expected
    assert len(json.loads((tmp_path / 'catalog.json').read_text())['assets']) == 1
