"""Rendered runtime inputs, without loading Basilisk, OpenCV or Unreal."""
import hashlib
import json
import os
from pathlib import Path
import subprocess


def renderer_paths(server):
    project=Path(server).parent/'space_sim_UE_Adapter/Unreal/BskUnrealRenderer'
    unreal=Path(os.environ.get('SPACESIM_BENCHMARK_UNREAL_ROOT',r'C:\Program Files\Epic Games\UE_5.6'))
    editor=unreal/'Engine/Binaries/Win64/UnrealEditor.exe'
    if os.name!='nt' or not editor.is_file():
        raise RuntimeError('rendered benchmark requires Windows UE 5.6; configure SPACESIM_BENCHMARK_UNREAL_ROOT')
    if not (project/'BskUnrealRenderer.uproject').is_file():
        raise RuntimeError('paired SpaceSim UE project is missing')
    binary=project/'Plugins/BskUnrealRuntime/Binaries/Win64/UnrealEditor-BskUnrealRuntime.dll'
    if not binary.is_file():
        raise RuntimeError('build the paired UE runtime before rendered evaluation')
    for path in (project/'Plugins/BskUnrealRuntime/Source').rglob('*'):
        if path.suffix in {'.cpp','.h','.cs'} and path.stat().st_mtime_ns>binary.stat().st_mtime_ns:
            raise RuntimeError('UE runtime build predates a paired source file')
    return project.resolve(),editor.resolve()


def graphics_identity():
    """Stable hardware/driver metadata; never include clocks, load or process IDs."""
    if os.name!='nt':
        raise RuntimeError('rendered benchmark GPU identity requires Windows')
    try:
        result=subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-Command',
            '[Console]::OutputEncoding=[Text.UTF8Encoding]::new($false); Get-CimInstance Win32_VideoController | Select-Object Name,DriverVersion,AdapterCompatibility,VideoProcessor | ConvertTo-Json -Compress'],
            capture_output=True,text=True,encoding='utf-8',errors='strict',timeout=30,check=True,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    except subprocess.SubprocessError as error:
        raise RuntimeError('could not read graphics driver identity') from error
    value=json.loads(result.stdout)
    devices=value if isinstance(value,list) else [value]
    if not devices or any(not device.get('Name') or not device.get('DriverVersion') for device in devices):
        raise ValueError('GPU model and driver version must be available')
    return sorted(devices,key=lambda device:(device['Name'],device['DriverVersion']))


def receipt_dependencies(receipt,project,engine):
    """Resolve the actual built target's runtime closure, rejecting path escapes."""
    data=json.loads(receipt.read_text('utf-8'))
    for item in data['RuntimeDependencies']+data['BuildProducts']:
        if item['Type'] in {'SymbolFile','DebugNonUFS'}:
            continue
        value=item['Path'].replace('$(ProjectDir)',str(project)).replace('$(EngineDir)',str(engine))
        if '$(' in value:
            raise ValueError('unresolved UE build receipt path')
        path=Path(value).resolve()
        if not path.is_relative_to(project) and not path.is_relative_to(engine):
            raise ValueError('UE receipt dependency escapes project and engine')
        if not path.is_file():
            raise FileNotFoundError(path)
        yield path


def validate_catalog(server,project,catalog):
    data=json.loads(catalog.read_text('utf-8'))
    if data.get('schema')!='bsk-render-asset-catalog/1' or not data.get('assets'):
        raise ValueError('missing or unsupported rendered asset catalogue')
    sources=[]
    for name,asset in data['assets'].items():
        source=Path(name).resolve()
        if not source.is_relative_to(server):
            raise ValueError('render asset source belongs to another checkout')
        with source.open('rb') as stream:
            digest=hashlib.file_digest(stream,'sha256').hexdigest()
        if digest!=asset['source_sha256']:
            raise ValueError('render asset catalogue is stale: '+str(source))
        package=asset['asset_path'].split('.')[0]
        if not package.startswith('/Game/') or '..' in package.split('/'):
            raise ValueError('render asset is outside the paired project')
        if not (project/'Content'/package[6:]).with_suffix('.uasset').is_file():
            raise ValueError('imported UE asset is missing: '+package)
        sources.append(source)
    return sources


def rendered_resources(server):
    """Hash authored content and build inputs; generated episode caches are excluded.

    Each renderer uses a fresh -UserDir, so previous episodes cannot supply saved
    graphics settings. Shared DDC is only a derived cache, not an asset source.
    """
    from .benchmark_runtime import files_under
    server=Path(server).resolve()
    project,editor=renderer_paths(server)
    engine=editor.parents[2]
    version=engine/'Build/Build.version'
    data=json.loads(version.read_text('utf-8'))
    if (data['MajorVersion'],data['MinorVersion'])!=(5,6):
        raise ValueError('rendered runtime requires the declared UE 5.6 version')
    receipt=project/'Binaries/Win64/BskUnrealRendererEditor.target'
    catalog=project/'Saved/AssetImport/sarm_platform.catalog.json'
    paths={version,editor,receipt,catalog,project/'BskUnrealRenderer.uproject'}
    paths.update(receipt_dependencies(receipt,project,engine))
    paths.update(validate_catalog(server,project,catalog))
    for root in (project/'Content',project/'Config',project/'Source',engine/'Content',engine/'Config',engine/'Shaders'):
        paths.update(files_under(root))
    for plugin in (project/'Plugins').iterdir():
        if not plugin.is_dir():
            continue
        paths.update(plugin.glob('*.uplugin'))
        for folder in ('Source','Content','Config','Shaders','Binaries'):
            root=plugin/folder
            if root.is_dir():
                paths.update(path for path in files_under(root) if path.suffix!='.pdb')
    # Include the complete installed editor module set, including modules loaded
    # dynamically rather than explicitly listed by the project target receipt.
    paths.update((engine/'Binaries/Win64').glob('*.dll'))
    paths.update((engine/'Binaries/Win64').glob('*.modules'))
    for relative in ('config/visual_attachments/target_aruco_flush_5cm_v2',
                     'config/vision/vendor/aruco_pnp_basler_mono_v1'):
        paths.update(files_under(server/relative))
    files={}
    for path in sorted(paths):
        path=path.resolve()
        owner,label=next((root,name) for root,name in ((server,'repository'),(project,'ue_project'),(engine,'ue_engine'))
                        if path.is_relative_to(root))
        files[label+'/'+path.relative_to(owner).as_posix()]=path
    return files,dict(engine_version=data,graphics_devices=graphics_identity(),
        user_settings='fresh per-episode UserDir; no inherited project Saved settings',
        cache_policy='derived DDC only; original content and shaders are fingerprinted')
