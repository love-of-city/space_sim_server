"""Generate, import and photograph a configured target in UE (static inspection).

Run with the geometry Python, never the live Basilisk interpreter.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('profile','output','adapter','unreal'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--catalog',type=Path,action='append',required=True)
    parser.add_argument('--reuse-built',action='store_true')
    parser.add_argument('--native-python',type=Path,help='Optional separate Basilisk Python for a two-second hold audit')
    parser.add_argument('--install-index',type=Path,help='After verification, update this local family 2/4 scene index')
    args=parser.parse_args()
    if args.install_index and not args.native_python:
        parser.error('--install-index requires --native-python holding verification')
    from build_target_asset import load_profile
    profile=load_profile(args.profile)
    if args.native_python and profile['adapter']!='ground_validation_end_face':
        parser.error('This asset has no verified native dynamics adapter')
    out=args.output.resolve()
    project=args.adapter.resolve()/'Unreal/BskUnrealRenderer'
    editor=args.unreal.resolve()/('Engine/Binaries/Win64/UnrealEditor.exe' if os.name=='nt' else 'Engine/Binaries/Linux/UnrealEditor')
    report=dict(profile_id=profile['id'],profile_sha256=sha(args.profile),stages={},
                full_task_accepted=False,dynamics_executed=False)
    def run(stage,command,env=None):
        print('START',stage,flush=True)
        report['stages'][stage]='running';write(out/'pipeline.json',report)
        try:
            with (out/(stage+'.log')).open('w',encoding='utf-8') as log:
                subprocess.run(list(map(str,command)),cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        except Exception as error:
            report['stages'][stage]='failed';report['failure']=str(error);write(out/'pipeline.json',report)
            raise
        report['stages'][stage]='passed';write(out/'pipeline.json',report)
    if not args.reuse_built:
        subprocess.run([sys.executable,str(ROOT/'tools/build_target_asset.py'),'--profile',str(args.profile.resolve()),'--output',str(out)],check=True)
    generation=json.loads((out/'generation.json').read_text(encoding='utf-8'))
    if generation['profile_sha256']!=report['profile_sha256']:
        raise ValueError('Existing generation belongs to another profile')
    for name,expected in generation.get('artifacts',{}).items():
        if sha(out/name)!=expected:
            raise ValueError('Generated artifact changed: '+name)
    model=out/'preheld-dual.xml'
    report.update(model_sha256=sha(model),initial_scene_sha256=sha(out/'initial-scene.json'))
    run('assembly-audit',[sys.executable,ROOT/'tools/audit_target_asset.py','--profile',args.profile.resolve(),'--case',out])
    if profile['adapter']=='ground_validation_end_face':
        run('closure-clearance',[sys.executable,ROOT/'tools/audit_hull_clearance.py','--case',out])
        if args.native_python:
            run('native-hold',[args.native_python,ROOT/'tools/run_preheld_hold.py','--model',model,
                '--scene',out/'initial-scene.json','--output',out/'hold-native','--seconds','2'])
            run('native-hold-audit',[sys.executable,ROOT/'tools/verify_preheld_capture.py','--model',model,
                '--scene',out/'initial-scene.json','--native',out/'hold-native','--output',out/'hold-validation.json'])
            hold=json.loads((out/'hold-validation.json').read_text())
            if not hold['passed']:
                report['stages']['native-hold-audit']='failed';write(out/'pipeline.json',report)
                raise RuntimeError('Native holding validation failed')
            report['dynamics_executed']=True
    known={}
    for path in args.catalog:
        for item in json.loads(path.read_text(encoding='utf-8-sig'))['assets'].values():
            asset=project/'Content'/(item['asset_path'].split('.')[0][6:]+'.uasset')
            if asset.is_file():known[item['source_sha256']]=item
    tree=ET.parse(model).getroot()
    visual={g.get('mesh') for g in tree.iter('geom') if g.get('group')=='2'}
    imp=ET.Element('mujoco');assets=ET.SubElement(imp,'asset');body=ET.SubElement(ET.SubElement(imp,'worldbody'),'body',name='import')
    seen=set()
    for mesh in tree.findall('./asset/mesh'):
        path=mesh.get('file')
        if not path or mesh.get('name') not in visual or path in seen or sha(path) in known:continue
        seen.add(path);assets.append(mesh)
        ET.SubElement(body,'geom',type='mesh',mesh=mesh.get('name'),group='2',mass='0',contype='0',conaffinity='0')
    catalogs=list(args.catalog)
    report['input_catalogs']={str(p.resolve()):sha(p) for p in catalogs}
    # The engine's primitive cube may use Nanite even when imported CAD meshes
    # do not. Preserve the existing material graph and enable its required usage.
    material=project/'Content/BSK/M_BskPbrOpaque.uasset'
    if material.exists():
        import shutil
        shutil.copy2(material,out/'M_BskPbrOpaque-before.uasset')
    usage=out/'pbr-usage.json'
    env=dict(os.environ,BSK_PBR_USAGE_RESULT=str(usage))
    run('ue-material-usage',[editor,project/'BskUnrealRenderer.uproject',
        f'-ExecutePythonScript={ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/scripts/configure_pbr_geometry_usage.py"}',
        '-nullrhi','-unattended','-nop4','-nosplash'],env)
    if not json.loads(usage.read_text())['used_with_nanite']:
        raise RuntimeError('Required runtime material usage is unavailable')
    if seen:
        import_xml=out/'import.xml';ET.ElementTree(imp).write(import_xml,encoding='utf-8')
        catalog=out/'generated.catalog.json';settings=out/'import-settings.json'
        destination='/Game/BSK/Generated/TargetProfiles/'+profile['id']+'_'+report['model_sha256'][:8]
        env=dict(os.environ,PYTHONPATH=str(args.adapter/'Adapters')+os.pathsep+os.environ.get('PYTHONPATH',''))
        run('catalog',[sys.executable,project/'scripts/generate_mjcf_asset_catalog.py','--mjcf',import_xml,'--destination',destination,'--catalog',catalog,'--import-settings',settings,'--normal-mode','preserve'],env)
        run('ue-import',[editor,project/'BskUnrealRenderer.uproject','-run=ImportAssets',f'-importsettings={settings}',f'-dest={destination}','-replaceexisting','-nosourcecontrol','-unattended','-nop4','-nosplash','-RenderOffscreen'])
        cat=json.loads(catalog.read_text(encoding='utf-8'))
        for item in cat['assets'].values():
            if not (project/'Content'/(item['asset_path'].split('.')[0][6:]+'.uasset')).is_file():raise RuntimeError('UE failed to save asset')
        result=out/'mesh-configuration.json'
        env.update(BSK_MJCF_ASSET_CATALOG=str(catalog),BSK_MJCF_NORMAL_MODE='preserve',BSK_MJCF_RESULT_PATH=str(result))
        run('ue-configure',[editor,project/'BskUnrealRenderer.uproject',f'-ExecutePythonScript={project / "scripts/configure_imported_meshes.py"}','-nullrhi','-unattended','-nop4','-nosplash'],env)
        if json.loads(result.read_text())!={'configured':len(cat['assets']),'catalog_sha256':sha(catalog)}:raise RuntimeError('UE import receipt mismatch')
        catalogs.append(catalog)
    recording=out/'inspection.bskrec'
    view=profile.get('inspection_view',{})
    view_args=['--distance',str(view.get('distance_m',3.5))]
    if 'direction' in view:view_args+=['--view-direction',*map(str,view['direction'])]
    if not view.get('insets',True):view_args+=['--no-insets']
    run('export',[sys.executable,ROOT/'tools/export_asset_xml_preview.py','--model',model,'--scene',out/'initial-scene.json','--project',project,'--output',recording,'--label',profile['id'],*view_args,*[v for p in catalogs for v in ('--catalog',p)]])
    screenshot=out/'UE-inspection.png'
    command=[str(editor),str(project/'BskUnrealRenderer.uproject'),'-game','-RenderOffscreen','-Windowed','-ForceRes','-ResX=1800','-ResY=1200','-BskPort=5589',f'-BskReplay={recording}',f'-BskScreenshot={screenshot}',f'-abslog={out / "UE-inspection.log"}','-DDC=InstalledDerivedDataBackendGraph']
    process=subprocess.Popen(command,creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
    try:
        deadline=time.monotonic()+600
        while not screenshot.exists():
            if process.poll() is not None:raise RuntimeError('UE exited before screenshot')
            if time.monotonic()>deadline:raise TimeoutError('UE screenshot timeout')
            time.sleep(2)
        time.sleep(2)
        if screenshot.read_bytes()[:8]!=b'\x89PNG\r\n\x1a\n':raise RuntimeError('Invalid UE screenshot')
        engine_log=(out/'UE-inspection.log').read_text(encoding='utf-8',errors='replace')
        if 'missing bUsedWithNanite=True' in engine_log or 'Default Material will be used in game' in engine_log:
            report['stages']['ue_screenshot']='failed';write(out/'pipeline.json',report)
            raise RuntimeError('UE used a fallback material; screenshot is not accepted')
        report['stages']['ue_screenshot']='passed';report['screenshot_sha256']=sha(screenshot)
        write(out/'pipeline.json',report)
        if args.install_index:
            run('platform-registration',[sys.executable,ROOT/'tools/register_target_asset.py','--case',out,'--index',args.install_index.resolve()])
        print(json.dumps(report),flush=True)
    finally:
        process.terminate()
        process.wait(timeout=30)


if __name__=='__main__':main()
