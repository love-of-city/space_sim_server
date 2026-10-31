"""Exercise real archive creation, overlay extraction and immutable outputs."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

RELEASE = Path(__file__).resolve().parents[1]


def execute(*args):
    return subprocess.run(list(map(str, args)), text=True, capture_output=True)


def test_base_and_collector_archives_are_relocatable(tmp_path):
    bundle = tmp_path / 'staging with spaces' / 'space-sim-layout-preparer'
    bundle.mkdir(parents=True)
    (bundle / 'base-marker').write_text('shared runtime')
    dependency_lock = bundle / 'repo/manifests/environments/requirements.lock'
    dependency_lock.parent.mkdir(parents=True)
    dependency_lock.write_text('locked dependency identity')
    base = tmp_path / 'base release'
    result = execute(sys.executable, RELEASE / 'layout_preparer/build-release.py',
                     '--bundle', bundle, '--output', base)
    assert result.returncode == 0, result.stderr
    info = json.loads((base / 'release.json').read_text())
    artifact = Path(info['artifact'])
    assert hashlib.sha256(artifact.read_bytes()).hexdigest() == info['sha256']

    overlay = tmp_path / 'overlay'
    overlay.mkdir()
    (overlay / 'base-marker').write_text('overridden by collector')
    (overlay / 'collector-marker').write_text('collector')
    collector = tmp_path / 'collector release'
    result = execute(sys.executable, RELEASE / 'collector/build-collector.py',
                     '--base-release', base, '--overlay', overlay, '--output', collector)
    assert result.returncode == 0, result.stderr
    info = json.loads((collector / 'release.json').read_text())
    artifact = Path(info['artifact'])
    assert hashlib.sha256(artifact.read_bytes()).hexdigest() == info['sha256']
    destination = tmp_path / 'relocated with spaces'
    result = execute(artifact, '--extract', destination)
    assert result.returncode == 0, result.stderr
    assert (destination / 'base-marker').read_text() == 'overridden by collector'
    assert (destination / 'collector-marker').read_text() == 'collector'
    assert (destination / 'repo/manifests/environments/requirements.lock').read_text() == 'locked dependency identity'

    before = artifact.read_bytes()
    result = execute(sys.executable, RELEASE / 'collector/build-collector.py',
                     '--base-release', base, '--overlay', overlay, '--output', collector)
    assert result.returncode != 0
    assert artifact.read_bytes() == before


def test_collector_rejects_corrupt_common_runtime(tmp_path):
    base = tmp_path / 'base'
    base.mkdir()
    (base / 'space-sim-layout-preparer-linux-x86_64.run').write_bytes(
        b'PAYLOAD_LINE=2\ncorrupted payload')
    (base / 'release.json').write_text(json.dumps({'payload_sha256': '0' * 64}))
    overlay = tmp_path / 'overlay'
    overlay.mkdir()
    output = tmp_path / 'output'
    result = execute(sys.executable, RELEASE / 'collector/build-collector.py',
                     '--base-release', base, '--overlay', overlay, '--output', output)
    assert result.returncode != 0
    assert 'checksum mismatch' in result.stderr
    assert not (output / 'space-sim-collector-linux-x86_64.run').exists()
