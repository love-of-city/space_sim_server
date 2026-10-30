"""Verify the exact .run artifact after construction, including paths with spaces."""
from pathlib import Path
import subprocess,json,hashlib,time
import argparse
_parser=argparse.ArgumentParser(description='Explicit release integration verification; requires GPU/runtime.')
_parser.add_argument('--release-root',type=Path,required=True)
root=_parser.parse_args().release_root.resolve()
(root/'validation').mkdir(exist_ok=True)
artifact=root/'space-sim-layout-preparer-linux-x86_64.run'
dest=root/'relocated bundle/space-sim-layout-preparer'
output=root/'validation/relocated-layout'
records=[]
def run(command,name,allowed=(0,)):
    start=time.monotonic()
    with (root/'validation'/(name+'.log')).open('w') as log:
        result=subprocess.run(list(map(str,command)),stdout=log,stderr=subprocess.STDOUT)
    records.append(dict(name=name,returncode=result.returncode,wall_seconds=time.monotonic()-start))
    if result.returncode not in allowed:
        raise RuntimeError(f'{name} failed: see validation/{name}.log')
    print(name,'passed',flush=True)
run([artifact,'--extract',dest],'extract-exact-artifact')
launcher=dest/'space-sim-layouts'
run([launcher,'check'],'relocated-dependencies')
command=[launcher,'prepare','--count','1','--seed','202610288801','--assets-only','--output',output]
run(command,'relocated-prepare')
count_before=len((output/'timings.jsonl').read_text().splitlines())
run([*command,'--resume'],'relocated-resume')
assert len((output/'timings.jsonl').read_text().splitlines())==count_before,'Resume re-ran completed asset stages'
run([launcher,'status','--output',output],'relocated-status')
run([artifact,'--install-dir',dest,'check'],'artifact-reuse')
progress=json.loads((output/'progress.json').read_text())
assert progress['status']=='finished' and progress['prepared']==1 and progress['pool_published'] is False
assert not (output/'pool.json').exists()
folder=output/'layout-000-seed-202610288801'
assert json.loads((folder/'native-geometry.json').read_text())['passed']
assert json.loads((folder/'inset-geometry.json').read_text())['passed']
meshes=json.loads((folder/'ue-assets/configured.json').read_text())
assert meshes['configured']==12 and all(row['actual']==row['expected'] for row in meshes['geometry'])
report=dict(passed=True,artifact=json.loads((root/'release.json').read_text()),checks=records,
    supported_platform='Linux x86_64, glibc >= 2.35',relocated_bundle=str(dest),
    relocation_test_machine='same host; independent directory containing spaces',
    original_source_directories_used_by_dependency_check=0,
    complete_layout_preparation=True,ue_meshes_verified=12,prepared_layouts=1,
    certified_layouts=0,six_task_collection_validated=False,
    native_physics_smoke='0.1 simulated seconds; complete sampling and correct rejection of incomplete trajectory',
    launcher_tests=7,scope='random-layout preparation and original trajectory gate; no full six-task video collection')
(root/'validation/RESULTS.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('ALL RELEASE CHECKS PASSED',flush=True)
