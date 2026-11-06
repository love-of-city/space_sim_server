"""Combine shipped V2 meshes with the existing SARM object asset catalog."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(model,base_catalog,adapter,output):
    model,base_catalog,adapter,output=map(lambda p:Path(p).resolve(),(model,base_catalog,adapter,output))
    identity=json.loads(model.with_suffix('.json').read_text())
    if hashlib.sha256(model.read_bytes()).hexdigest()!=identity['model_sha256']:
        raise ValueError('Normalized dual model differs from identity')
    document=json.loads(base_catalog.read_text())
    root=ET.parse(model).getroot()
    visual_names={g.get('mesh') for g in root.findall('.//geom') if g.get('group')=='2'}
    for mesh in root.findall('./asset/mesh'):
        if mesh.get('name') not in visual_names:continue
        source=(model.parent/root.find('compiler').get('meshdir','.')/mesh.get('file')).resolve()
        name=source.stem
        package=adapter/'Unreal/BskUnrealRenderer/Content/Generated/DualServiceV2'/f'{name}.uasset'
        for path in (source,package):
            if not path.is_file() or path.read_bytes().startswith(b'version https://git-lfs'):
                raise ValueError(f'Missing hydrated dual asset: {path}')
        document['assets'][str(source)]=dict(asset_type='static_mesh',asset_path=f'/Game/Generated/DualServiceV2/{name}.{name}',
            component_scale=[1.,1.,1.],build_scale=[100.,100.,100.],source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    document.update(source_mjcf=str(model),dual_model_sha256=identity['model_sha256'])
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(document,indent=2)+'\n')
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('model','base-catalog','adapter','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();print(prepare(a.model,a.base_catalog,a.adapter,a.output))
