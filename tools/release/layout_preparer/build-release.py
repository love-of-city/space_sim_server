from pathlib import Path
import subprocess,hashlib,shutil,json,time

def digest(stream):
    result=hashlib.sha256()
    for chunk in iter(lambda:stream.read(8*1024*1024),b''):
        result.update(chunk)
    return result.hexdigest()
import argparse
from datetime import datetime
parser=argparse.ArgumentParser(description='Package an explicitly staged SpaceSim runtime without modifying source.')
parser.add_argument('--bundle', type=Path, required=True, help='Prepared space-sim-layout-preparer directory')
parser.add_argument('--output', type=Path, required=True, help='New release output directory')
args=parser.parse_args()
bundle=args.bundle.resolve()
if bundle.name != 'space-sim-layout-preparer' or not bundle.is_dir():
    parser.error('--bundle must name an existing space-sim-layout-preparer directory')
root=args.output.resolve()
root.mkdir(parents=True, exist_ok=True)
if (root/'space-sim-layout-preparer-linux-x86_64.run').exists():
    parser.error('Refusing to overwrite an existing release')
payload=root/'payload.tar.gz'
excludes=['*/__pycache__','*/.pytest_cache','*/Intermediate','*/Saved/Logs','*/Saved/Autosaves','*/Saved/Config',
          '*/Saved/Collections','*/Saved/SourceControl','*/Saved/UnrealBuildTool','*.lock','*/.relocation.lock',
          '*/Content/BSK/Generated/Portable_*']
start=time.monotonic()
cmd=['tar',*[f'--exclude={p}' for p in excludes],'-C',str(bundle.parent),'-cf','-','space-sim-layout-preparer']
with payload.open('wb') as stream:
    tar=subprocess.Popen(cmd,stdout=subprocess.PIPE)
    compress=subprocess.Popen(([shutil.which('pigz'),'-1','-p','2'] if shutil.which('pigz') else ['gzip','-1']),stdin=tar.stdout,stdout=stream)
    tar.stdout.close()
    if compress.wait()!=0 or tar.wait()!=0:
        raise RuntimeError('Archive failed')
print('payload ready',payload.stat().st_size,'bytes',flush=True)
with payload.open('rb') as f:sha=digest(f)
release_id=datetime.now().strftime('%Y%m%d')+'-'+sha[:12]
header=(Path(__file__).resolve().parent/'self-extractor.template').read_text().replace('@SHA@',sha).replace('@ID@',release_id)
header=header.replace('@LINE@',str(header.count('\n')+1))
artifact=root/'space-sim-layout-preparer-linux-x86_64.run'
with artifact.open('wb') as out, payload.open('rb') as source:
    out.write(header.encode())
    shutil.copyfileobj(source,out,16*1024*1024)
artifact.chmod(0o755)
with artifact.open('rb') as f:artifact_sha=digest(f)
(root/(artifact.name+'.sha256')).write_text(artifact_sha+'  '+artifact.name+'\n')
(root/'release.json').write_text(json.dumps(dict(artifact=str(artifact),bytes=artifact.stat().st_size,sha256=artifact_sha,
    payload_sha256=sha,release_id=release_id,build_seconds=time.monotonic()-start),indent=2)+'\n')
print((root/'release.json').read_text(),flush=True)
