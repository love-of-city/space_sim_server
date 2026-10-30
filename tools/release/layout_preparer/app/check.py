from pathlib import Path
import json
import subprocess
import sys
import tempfile

base = Path(__file__).resolve().parents[1]
root = base/'repo/server'
import numpy, bsk_render_adapter
from Basilisk.simulation import mujoco
from simulation.mujoco_plugins import ensure_model_plugins
from simulation.local_mujoco_stepper import _library
from simulation.collection_assets import prepare_base_catalog
ensure_model_plugins(root/'model/SARM/platform/sarm_20261012.xml')
_library()
with tempfile.TemporaryDirectory(prefix='layout-bundle-check-') as temp:
    catalog = prepare_base_catalog(Path(temp)/'base.catalog.json')
    count = len(json.loads(catalog.read_text())['assets'])
subprocess.run([str(base/'runtime/geometryenv/bin/python'), '-c',
    'import sys,numpy,scipy,trimesh,manifold3d,coacd,mujoco,bsk_render_adapter; print("geometry Python:", sys.executable, "MuJoCo:", mujoco.__version__)'], check=True)
editor = base/'runtime/UE_5.6/Engine/Binaries/Linux/UnrealEditor-Cmd'
result = subprocess.run(['ldd', str(editor)], text=True, capture_output=True, check=True)
if 'not found' in result.stdout:
    raise RuntimeError(result.stdout)
print(json.dumps(dict(passed=True,python=sys.executable,basilisk_mjscene=str(mujoco.MJScene),common_assets=count,thread_plugin=True,native_stepper=True,ue_dependencies=True,scope='Dependencies and assets only; use prepare for real import and certify for trajectories'),indent=2))
