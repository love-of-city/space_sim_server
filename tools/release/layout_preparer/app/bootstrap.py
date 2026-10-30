"""Relocate the included venvs without installing or downloading anything."""
from pathlib import Path
import json
import platform
import os
import fcntl

base = Path(__file__).resolve().parents[1]
if platform.machine() != 'x86_64':
    raise SystemExit('This release requires x86_64 Linux.')
if tuple(map(int, platform.libc_ver()[1].split('.')[:2])) < (2, 35):
    raise SystemExit('This release requires glibc >= 2.35 (Ubuntu 22.04 or compatible).')
with (base / '.relocation.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    for name in ('simenv', 'geometryenv'):
        path = base / 'runtime' / name / 'pyvenv.cfg'
        text = f'home = {base / "runtime/python/bin"}\ninclude-system-site-packages = false\nversion = 3.13.16\n'
        if path.read_text() != text:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(text)
            temporary.replace(path)
    path = base / 'repo/server/run/native_acceleration/sarm_thread.json'
    record = json.loads(path.read_text())
    library = next((base/'runtime/simenv/lib/python3.13/site-packages/bsk.libs').glob('libmujoco*.so*'))
    if record.get('runtime_path') != str(library):
        record['runtime_path'] = str(library)
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(record, indent=2)+'\n')
        temp.replace(path)
