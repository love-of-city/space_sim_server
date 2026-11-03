"""Install a verified local target candidate into existing family 2/4 entries."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'tools')]
from simulation.scene_library.manifest import digest, model_dependencies, write_json
from space_arm_platform.imported_scene_library import verified_source
from import_platform_scene_library import cameras


def register(case,index_path):
    model=case/'preheld-dual.xml';scene=case/'initial-scene.json';model_hash=digest(model)
    proofs={}
    for name in ('asset-audit','closure-clearance','hold-validation'):
        path=case/(name+'.json');value=json.loads(path.read_text(encoding='utf-8'))
        if not value['passed'] or value['model_sha256']!=model_hash:
            raise ValueError('Missing or mismatched acceptance: '+name)
        proofs[name]=dict(path=str(path),sha256=digest(path))
    if json.loads((case/'hold-validation.json').read_text())['scene_sha256']!=digest(scene):
        raise ValueError('Hold initial scene mismatch')
    pipeline=json.loads((case/'pipeline.json').read_text())
    if pipeline['model_sha256']!=model_hash or pipeline['stages'].get('ue_screenshot')!='passed':
        raise ValueError('UE inspection is required before registration')
    dest=case/'platform-registration';dest.mkdir(exist_ok=False)
    shutil.copy2(index_path,dest/'previous-index.json')
    index=json.loads(index_path.read_text(encoding='utf-8-sig'))
    new_scene=json.loads(scene.read_text(encoding='utf-8'))
    extra=json.loads((case/'inspection.catalog.json').read_text(encoding='utf-8'))
    for path,item in extra['assets'].items():
        if digest(path)!=item['source_sha256']:raise ValueError('UE catalog source changed: '+path)
    deps=model_dependencies(model);count=0
    for entry in index['entries']:
        if not entry['scene_id'].startswith(('02_','04_')):continue
        if cameras(model)!=entry['camera_contract']:raise ValueError('Installed camera contract changed')
        previous=deepcopy(entry);source=deepcopy(verified_source(previous))
        folder=dest/entry['scene_id'].replace('/','--');folder.mkdir()
        source.update(model=str(model),dependencies=deps,initial_scene=deepcopy(new_scene),
            accepted=False,assembly_status='continuous_hull_local_preview')
        fixture=source['operation_fixture']
        for key,name in [('assembly','assembly.json'),('mount_inspection','asset-audit.json'),('initial_scene_source','initial-scene.json')]:
            fixture[key]=str(case/name);fixture[key+'_sha256']=digest(case/name)
        write_json(folder/'instance.json',source)
        catalog=json.loads(Path(previous['catalog']).read_text(encoding='utf-8'))
        catalog['assets'].update(extra['assets']);catalog['source_mjcf']=str(model)
        write_json(folder/'catalog.json',catalog)
        write_json(folder/'receipt.json',dict(schema='continuous-hull-registration/1',proofs=proofs,
            model_sha256=model_hash,scene_sha256=digest(scene),previous_entry=previous,full_task_accepted=False))
        for key,name in [('source_instance','instance.json'),('catalog','catalog.json'),('receipt','receipt.json')]:
            entry[key]=str(folder/name);entry[key+'_sha256']=digest(folder/name)
        entry.update(layout_revision='continuous-hull-'+model_hash[:12],full_task_accepted=False,
            description='连续封闭箱体，按实际任务盒边框局部让位；太阳翼与接口保持约束；短时保持通过，完整任务未验收。')
        verified_source(entry);count+=1
    if not count:raise ValueError('No family 2/4 entries found')
    index['layout_revision']='continuous-hull-'+model_hash[:12]
    write_json(dest/'installed-index.json',index)
    index_path.write_text(json.dumps(index,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return dict(registered=count,index=str(index_path),backup=str(dest/'previous-index.json'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',type=Path,required=True);p.add_argument('--index',type=Path,required=True)
    a=p.parse_args();print(json.dumps(register(a.case.resolve(),a.index.resolve())))
