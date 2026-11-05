import json
from pathlib import Path
import stat

from tools.policy import runtime_worker


def test_worker_separates_mutable_ue_files_and_keeps_shared_assets(tmp_path, monkeypatch):
    server = tmp_path / 'source/server'
    (server / 'config').mkdir(parents=True)
    (server / 'config/example.json').write_text('{}')
    for name in ('simulation', 'tools', 'backend', 'model', 'frontend', 'scripts'):
        (server / name).mkdir()
    adapter = tmp_path / 'source/ue-adapter'
    project = adapter / 'Unreal/BskUnrealRenderer'
    for name in ('Content', 'Config', 'Saved/AssetImport', 'Intermediate'):
        (project / name).mkdir(parents=True)
    (project / 'Content/asset').write_text('original')
    (project / 'Config/Default.ini').write_text('original')
    monkeypatch.setattr(runtime_worker, 'ROOT', server)
    workers = []
    for index in range(2):
        root = tmp_path / f'worker{index}'
        _, overlay, credentials = runtime_worker.prepare_worker(root, adapter)
        workers.append(overlay / 'Unreal/BskUnrealRenderer')
        assert credentials['password']
        assert stat.S_IMODE((root / 'credentials.json').stat().st_mode) == 0o600
        assert json.loads((root / 'credentials.json').read_text()) == credentials
    assert workers[0] / 'Saved' != workers[1] / 'Saved'
    (workers[0] / 'Config/Default.ini').write_text('worker0')
    assert (workers[1] / 'Config/Default.ini').read_text() == 'original'
    assert (project / 'Config/Default.ini').read_text() == 'original'
    assert (workers[0] / 'Content/asset').resolve() == project / 'Content/asset'
    assert (workers[0] / 'Saved/AssetImport').resolve() == project / 'Saved/AssetImport'
