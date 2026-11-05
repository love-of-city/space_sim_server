"""Run an isolated policy environment using shared, prebuilt UE assets."""
import argparse
import json
from pathlib import Path
import secrets
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]


def prepare_worker(root: Path, adapter: Path):
    root.mkdir(parents=True, exist_ok=True)
    server = root / 'server'
    server.mkdir(exist_ok=True)
    for name in ('simulation', 'tools', 'backend', 'model', 'frontend', 'scripts'):
        target = server / name
        if not target.exists():
            target.symlink_to(ROOT / name, target_is_directory=True)
    if not (server / 'config').exists():
        shutil.copytree(ROOT / 'config', server / 'config')
    for name in ('logs', 'run'):
        (server / name).mkdir(exist_ok=True)

    overlay = root / 'ue-adapter'
    overlay.mkdir(exist_ok=True)
    for item in adapter.iterdir():
        if item.name == 'Unreal':
            continue
        target = overlay / item.name
        if not target.exists():
            target.symlink_to(item, target_is_directory=item.is_dir())
    project = overlay / 'Unreal/BskUnrealRenderer'
    project.mkdir(parents=True, exist_ok=True)
    source = adapter / 'Unreal/BskUnrealRenderer'
    for item in source.iterdir():
        if item.name in {'Saved', 'Intermediate', 'Config', 'DerivedDataCache'}:
            continue
        target = project / item.name
        if not target.exists():
            target.symlink_to(item, target_is_directory=item.is_dir())
    if not (project / 'Config').exists():
        shutil.copytree(source / 'Config', project / 'Config')
    for name in ('Saved', 'Intermediate', 'DerivedDataCache'):
        (project / name).mkdir(exist_ok=True)
    catalog = project / 'Saved/AssetImport'
    if not catalog.exists():
        catalog.symlink_to(source / 'Saved/AssetImport', target_is_directory=True)

    secret = root / 'credentials.json'
    if not secret.exists():
        # Create privately rather than exposing the secret before chmod.
        import os
        descriptor = os.open(secret, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, 'w') as stream:
            json.dump({'username': 'policy', 'password': secrets.token_urlsafe(32)}, stream)
    return server, overlay, json.loads(secret.read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--adapter', type=Path, required=True)
    parser.add_argument('--unreal', type=Path, required=True)
    parser.add_argument('--episodes-root', type=Path, required=True)
    parser.add_argument('--port-base', type=int, required=True)
    args = parser.parse_args()
    args.root = args.root.resolve()
    server, adapter, credentials = prepare_worker(args.root, args.adapter.resolve())
    from space_arm_platform.app import PlatformConfig, create_app
    import uvicorn
    config = PlatformConfig(
        project_root=server, data_root=args.root / 'data/episodes',
        source_episodes_root=args.episodes_root.resolve(),
        auth_database=args.root / 'auth.sqlite3',
        bootstrap_admin_username=credentials['username'],
        bootstrap_admin_password=credentials['password'],
        simulation_port=args.port_base + 1, capture_port=args.port_base + 2,
        runtime_render_port=args.port_base + 3,
        runtime_pixel_streamer_port=args.port_base + 4,
        runtime_adapter_root=adapter, runtime_model_root=ROOT / 'model/SARM/platform',
        runtime_unreal_root=args.unreal.resolve(), runtime_pixel_streaming_camera_ids=(),
        runtime_renderer_ready_timeout=900, runtime_preview_rate=90,
    )
    uvicorn.run(create_app(config), host='127.0.0.1', port=args.port_base, access_log=False)


if __name__ == '__main__':
    main()
