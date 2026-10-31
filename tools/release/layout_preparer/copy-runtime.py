import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse

from runtime_metadata import collect, python_metadata

parser=argparse.ArgumentParser(description='Stage the current checkout and explicit local runtimes.')
parser.add_argument('--bundle',type=Path,required=True)
parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[3])
parser.add_argument('--python-runtime',type=Path,required=True)
parser.add_argument('--simenv',type=Path,required=True)
parser.add_argument('--geometryenv',type=Path,required=True)
parser.add_argument('--unreal',type=Path,required=True,help='UE root containing Engine/')
args=parser.parse_args()
base=args.bundle.resolve()
repo=args.repo.resolve()
if base.exists() and any(base.iterdir()):
    parser.error('Use an empty staging directory to avoid mixing releases')
for path in [repo,args.python_runtime,args.simenv,args.geometryenv,args.unreal/'Engine']:
    if not path.is_dir(): parser.error(f'Missing input directory: {path}')
python_candidates=[args.python_runtime/'bin/python3.13', args.python_runtime/'bin/python3', args.python_runtime/'bin/python']
python_executable=next((p for p in python_candidates if p.is_file()), None)
if python_executable is None: parser.error('Python runtime has no executable')
runtime_inputs=collect(python_executable,args.unreal)
python_version=runtime_inputs['python']['version']
python_minor='.'.join(python_version.split('.')[:2])
if python_minor!='3.13': parser.error('This bundle layout currently supports Python 3.13 only')
for label,environment in [('simulation',args.simenv),('geometry',args.geometryenv)]:
 metadata=python_metadata(environment/'bin/python')
 if metadata['version']!=python_version or metadata['abi']!=runtime_inputs['python']['abi']:
  parser.error(f'{label} interpreter does not match the supplied Python runtime')
 runtime_inputs[label]=metadata
for name,src in [('python',args.python_runtime),('simenv',args.simenv),('geometryenv',args.geometryenv)]:
 dest=base/'runtime'/name;dest.mkdir(parents=True,exist_ok=True)
 subprocess.run(['rsync','-a','--exclude=__pycache__/','--exclude=.git/',str(src)+'/',str(dest)+'/'],check=True)
 if name!='python':
  for p in (dest/'bin').glob('python*'):
   if p.is_symlink():p.unlink();p.symlink_to('../../python/bin/python3.13')
  site=dest/'lib/python3.13/site-packages'
  for p in site.glob('__editable__*'):p.unlink()
  (site/'spacesim_bundle.pth').write_text('../../../../../repo/server/backend\n../../../../../repo/ue-adapter/Adapters\n../../../../../repo/server\n')
  for p in site.glob('*.dist-info/direct_url.json'):p.unlink()
 print('copied',name,flush=True)
engine_dest=base/'runtime/UE_5.6'
engine_dest.mkdir(parents=True,exist_ok=True)
subprocess.run(['rsync','-a','--exclude=Saved/','--exclude=Intermediate/',str(args.unreal)+'/',str(engine_dest)+'/'],check=True)
paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=repo).decode().split('\0')
for rel in sorted(set(paths)):
 if not rel:continue
 p=Path(rel)
 if p.parts[0]=='server' and p.parts[1] not in ['backend','collection','config','simulation','tools','model','native','third_party','docs']:continue
 if p.parts[0]=='ue-adapter' and p.parts[1] not in ['Adapters','Unreal']:continue
 if p.parts[0] not in ['server','ue-adapter'] and p.name not in ['LICENSE','README.md','AGENTS.md','CONTRIBUTING.md']:continue
 src=repo/p
 if not src.is_file():continue
 dest=base/'repo'/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest)
 if dest.read_bytes()[:50].startswith(b'version https://git-lfs.github.com/spec/v1'):raise RuntimeError('Unresolved LFS '+str(p))
for rel in ['server/run/native_acceleration','ue-adapter/Unreal/BskUnrealRenderer/Binaries','ue-adapter/Unreal/BskUnrealRenderer/Plugins']:
 dest=base/'repo'/rel;dest.mkdir(parents=True,exist_ok=True)
 subprocess.run(['rsync','-a','--exclude=Intermediate/','--exclude=Saved/','--exclude=*.debug',str(repo/rel)+'/',str(dest)+'/'],check=True)
# Copy application launchers and notices from versioned sources.
shutil.copytree(Path(__file__).parent/'app',base/'app',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy2(Path(__file__).parent/'space-sim-layouts',base/'space-sim-layouts')
# Keep the working project's derived cache: it avoids the isolated-project shader cold start.
for suffix in ['DerivedDataCache','Saved/DerivedDataCache']:
 rel='ue-adapter/Unreal/BskUnrealRenderer/'+suffix
 dest=base/'repo'/rel;dest.mkdir(parents=True,exist_ok=True)
 subprocess.run(['rsync','-a',str(repo/rel)+'/',str(dest)+'/'],check=True)
from runtime_metadata import required_glibc

runtime_inputs['glibc']=required_glibc([base/'runtime',base/'repo'])
(base/'RUNTIME-INPUTS.json').write_text(json.dumps(runtime_inputs,indent=2)+'\n')
engine_version=runtime_inputs['engine']
build_info = {
    'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
    'source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True)),
    'platform': runtime_inputs['python']['platform'] + '-' + runtime_inputs['python']['machine'],
    'glibc_minimum': runtime_inputs['glibc']['minimum'],
    'python': python_version,
    'ue': '.'.join(str(engine_version[key]) for key in ('MajorVersion', 'MinorVersion', 'PatchVersion')),
    'runtime_inputs_sha256': hashlib.sha256((base/'RUNTIME-INPUTS.json').read_bytes()).hexdigest(),
    'parallel_workers': 1,
    'scope': 'Runtime staging; clean-install, relocation, GPU and data acceptance require separate validation',
}
(base/'BUILD.json').write_text(json.dumps(build_info, indent=2) + '\n')
print('repository and runtime copied',flush=True)

source_manifest={str(p.relative_to(base)):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ('repo','app') for p in sorted((base/folder).rglob('*')) if p.is_file()}
(base/'SOURCE-MANIFEST.json').write_text(json.dumps(source_manifest,indent=2)+'\n')
