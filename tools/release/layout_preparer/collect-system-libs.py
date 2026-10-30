from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import subprocess,re,shutil,json
import argparse
parser=argparse.ArgumentParser(description='Collect redistributable system dependencies for an existing staged bundle.')
parser.add_argument('--bundle',type=Path,required=True)
args=parser.parse_args()
base=args.bundle.resolve()
if not (base/'runtime').is_dir(): parser.error('Staged runtime directory is missing')
excluded={'libc.so.6','libm.so.6','libdl.so.2','libpthread.so.0','librt.so.1','libresolv.so.2','libutil.so.1','libanl.so.1','libBrokenLocale.so.1','ld-linux-x86-64.so.2'}
files=[]
for p in (base/'runtime').rglob('*'):
 if p.is_file() and not p.is_symlink() and ('.so' in p.name or p.stat().st_mode & 0o111):
  with p.open('rb') as f:h=f.read(20)
  if h[:4]==b'\x7fELF' and h[18:20]==b'\x3e\x00':files.append(p)
for p in (base/'repo').rglob('*.so'):
 files.append(p)
def dependencies(p):
 r=subprocess.run(['ldd',str(p)],text=True,capture_output=True)
 return [(name,Path(path)) for name,path in re.findall(r'^\s*(\S+) => (/\S+)',r.stdout,re.M) if path.startswith(('/lib/','/lib64/','/usr/lib/')) and name not in excluded]
found={}
with ThreadPoolExecutor(max_workers=4) as ex:
 for deps in ex.map(dependencies,files):
  for name,path in deps:
   if name in found and found[name]!=path:raise RuntimeError(name)
   found[name]=path
out=base/'runtime/system-libs';out.mkdir(exist_ok=True)
for name,path in found.items():shutil.copy2(path.resolve(),out/name)
# Preserve distribution notices for copied libraries.
licenses=base/'licenses/system-libs';licenses.mkdir(parents=True,exist_ok=True)
packages=set()
for p in found.values():
 result=subprocess.run(['dpkg-query','-S',str(p)],text=True,capture_output=True)
 for line in result.stdout.splitlines():
  package=line.split(': ')[0].split(':')[0]
  packages.add(package)
for package in packages:
 src=Path('/usr/share/doc')/package/'copyright'
 if src.exists():shutil.copy2(src,licenses/(package+'.copyright'))
(base/'system-libraries.json').write_text(json.dumps(dict(scanned_elf_files=len(files),libraries={name:str(path) for name,path in sorted(found.items())},packages=sorted(packages)),indent=2)+'\n')
print('scanned',len(files),'bundled',len(found),flush=True)
# Make incidental Python entrypoints relocatable as well.
for env in ['simenv','geometryenv']:
 for p in (base/'runtime'/env/'bin').iterdir():
  if p.is_symlink() or not p.is_file():continue
  with p.open('rb') as f:header=f.read(300)
  if header.startswith(b'#!') and b'python' in header.split(b'\n',1)[0]:
   data=p.read_bytes().split(b'\n',1)[1]
   p.write_bytes(b'#!/bin/sh\n\'\'\'exec\' "$(dirname -- "$0")/python" "$0" "$@"\n\' \'\'\'\n'+data)
