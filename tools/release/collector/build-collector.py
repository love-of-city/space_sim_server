"""Reuse the verified common runtime payload; append a collector overlay as a second gzip/tar member."""
from pathlib import Path
import hashlib,json,re,tarfile,subprocess,time
import argparse
parser=argparse.ArgumentParser(description='Build a collector from an explicit base release and overlay.')
parser.add_argument('--base-release', type=Path, required=True)
parser.add_argument('--overlay', type=Path, required=True, help='Prepared space-sim-layout-preparer overlay directory')
parser.add_argument('--output', type=Path, required=True)
args=parser.parse_args()
root=args.output.resolve()
old=args.base_release.resolve()
root.mkdir(parents=True, exist_ok=True)
if (root/'space-sim-collector-linux-x86_64.run').exists():
    parser.error('Refusing to overwrite an existing release')
if not args.overlay.is_dir():
    parser.error('Overlay directory is missing')
base=old/'space-sim-layout-preparer-linux-x86_64.run'
base_info=json.loads((old/'release.json').read_text())
delta=root/'collector-overlay.tar.gz'
with tarfile.open(delta,'w:gz',compresslevel=1) as archive:
    archive.add(args.overlay,arcname='space-sim-layout-preparer')
template=(Path(__file__).resolve().parent/'self-extractor.template').read_text()
template=template.replace('/spacesim-layout-preparer/','/spacesim-collector/')
template=template.replace('Self-contained SpaceSim layout preparer','Self-contained SpaceSim scripted data collector')
template=template.replace('space-sim-layout-preparer-linux-x86_64.run','space-sim-collector-linux-x86_64.run')
template=template.replace('prepare --count 150 --output DIRECTORY','run --task drawer_cycle --count 2 --layout-pool POOL --output DIRECTORY')
template=template.replace('tar -xzf - --no-same-owner','tar -xzf - --ignore-zeros --no-same-owner')
template=template.replace('/space-sim-layouts','/space-sim-collect')
(root/'self-extractor.template').write_text(template)
def header(sha):
    text=template.replace('@SHA@',sha).replace('@ID@',time.strftime('%Y%m%d')+'-collector-'+sha[:12])
    return text.replace('@LINE@',str(text.count('\n')+1)).encode()
first=header('0'*64)
artifact=root/'space-sim-collector-linux-x86_64.run'
temporary=artifact.with_suffix('.run.partial')
payload_hash=hashlib.sha256();base_hash=hashlib.sha256()
start=time.monotonic()
with base.open('rb') as source:
    first_lines=source.read(4096)
    line_number=int(re.search(rb'^PAYLOAD_LINE=(\d+)$',first_lines,re.M).group(1))
    source.seek(0)
    for _ in range(line_number-1):source.readline()
    with temporary.open('wb') as out:
        out.write(first)
        for chunk in iter(lambda:source.read(16*1024*1024),b''):
            base_hash.update(chunk);payload_hash.update(chunk);out.write(chunk)
        if base_hash.hexdigest()!=base_info['payload_sha256']:
            raise RuntimeError('Common runtime payload checksum mismatch')
        with delta.open('rb') as overlay:
            for chunk in iter(lambda:overlay.read(16*1024*1024),b''):
                payload_hash.update(chunk);out.write(chunk)
        final=header(payload_hash.hexdigest())
        assert len(final)==len(first)
        out.seek(0);out.write(final)
temporary.chmod(0o755)
full_hash=hashlib.sha256()
with temporary.open('rb') as f:
    for chunk in iter(lambda:f.read(16*1024*1024),b''):full_hash.update(chunk)
temporary.replace(artifact)
(root/(artifact.name+'.sha256')).write_text(full_hash.hexdigest()+'  '+artifact.name+'\n')
result=dict(artifact=str(artifact),bytes=artifact.stat().st_size,sha256=full_hash.hexdigest(),payload_sha256=payload_hash.hexdigest(),
    common_runtime_sha256=base_hash.hexdigest(),build_seconds=time.monotonic()-start,
    format='self-extracting bash; concatenated gzip/tar members; tar --ignore-zeros',
    release_id=time.strftime('%Y%m%d')+'-collector-'+payload_hash.hexdigest()[:12])
(root/'release.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2),flush=True)
