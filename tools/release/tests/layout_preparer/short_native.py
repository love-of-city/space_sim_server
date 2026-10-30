from pathlib import Path
import subprocess,sys
import argparse
_parser=argparse.ArgumentParser(description='Explicit staged-runtime native smoke test.')
_parser.add_argument('--bundle',type=Path,required=True)
base=_parser.parse_args().bundle.resolve()
root=base/'repo/server'
sys.path[:0]=[str(root),str(root/'backend'),str(base/'repo/ue-adapter/Adapters')]
from tools.assets.prepare_collection_layout_pool import validation_scene
out=Path(__file__).resolve().parent
scene=validation_scene(None,out,202610288801)
model=out/'layout-smoke/layout-000-seed-202610288801/model/layout.xml'
cmd=[sys.executable,str(root/'tools/diagnostics/validate_task_trajectory_contacts.py'),'--model',str(model),'--scene',str(scene),'--task','drawer_cycle','--output',str(out/'native-short'),'--native-geometry','--ignore-free-bolt','--initial-joints-deg','90','-60','60','0','-90','0','--max-sim-seconds','0.1']
result=subprocess.run(cmd)
# A deliberately truncated trajectory must not be certified; the physics must start.
import json
report=json.loads((out/'native-short/acceptance.json').read_text())
assert result.returncode==1, result.returncode
assert report['execution_error'] is None, report
assert report['coverage_complete'] is True,report
assert report['operational_path_complete'] is False,report
print('PASS: native physics ran; truncated trajectory correctly rejected')
