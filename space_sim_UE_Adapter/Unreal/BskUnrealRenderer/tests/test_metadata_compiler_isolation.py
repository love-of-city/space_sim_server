"""A standalone metadata compiler must not share a process with Basilisk."""
import json
from pathlib import Path
import subprocess
import sys

import pytest


def test_compiled_mesh_poses_match_without_importing_standalone_mujoco_in_parent(tmp_path):
    # A tetrahedron away from the origin exercises compiler recentering.
    model=tmp_path/'model.xml'
    model.write_text('''<mujoco><asset><mesh name="tetra" vertex="1 0 0  2 0 0  1 1 0  1 0 1"/></asset>
<worldbody><body name="body"><geom type="mesh" mesh="tetra" pos=".2 .3 .4" quat=".9238795325 0 0 .3826834324"/></body></worldbody></mujoco>''',encoding='utf-8')
    module=Path(__file__).resolve().parents[3]/'Adapters/bsk_render_adapter/mjcf_assets.py'
    script='''
import importlib.util, json, sys, types
from pathlib import Path
spec=importlib.util.spec_from_file_location('metadata_under_test',sys.argv[1])
module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
if sys.argv[3]=='isolated':
    sys.modules['Basilisk']=types.ModuleType('Basilisk')
poses=module._compiled_source_mesh_poses(Path(sys.argv[2]))
if sys.argv[3]=='isolated':
    assert 'mujoco' not in sys.modules
else:
    assert 'Basilisk' not in sys.modules
print(json.dumps(poses))
'''
    results=[]
    for mode in ('direct','isolated'):
        process=subprocess.run([sys.executable,'-c',script,str(module),str(model),mode],
            capture_output=True,text=True,timeout=150)
        assert process.returncode==0,process.stdout+process.stderr
        results.append(json.loads(process.stdout))
    assert results[0]==results[1]
    assert results[0][0][0]==pytest.approx([.2,.3,.4],abs=1e-10)
