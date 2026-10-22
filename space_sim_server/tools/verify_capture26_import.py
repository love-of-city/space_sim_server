"""Check capture arm import while preserving the upstream visual asset mappings."""
from pathlib import Path
import argparse,json,hashlib,xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
def verify(adapter):
    p=ROOT/'model/SARM/platform/sarm_dual_capture26.xml'
    manifest=json.loads(p.with_suffix('.manifest.json').read_text(encoding='utf-8'))
    assert hashlib.sha256(p.read_bytes()).hexdigest()==manifest['model_sha256']
    base=E.parse(ROOT/manifest['base_model']).getroot();added=E.parse(p).getroot()
    old={b.get('name'):b for b in base.iter('body')}
    new={b.get('name'):b for b in added.iter('body')}
    # Changes to the capture camera must never alter existing fixed cameras.
    original_cameras={c.get('name'):c.attrib for c in base.iter('camera')}
    actual_cameras={c.get('name'):c.attrib for c in added.iter('camera')}
    for name, attributes in original_cameras.items():
        assert actual_cameras.get(name)==attributes, ('Protected upstream camera changed', name)
    changed={x['geom'] for x in manifest.get('mount_clearance_changes',[])}
    if manifest.get('real_window_collision'):
        target = new['capture_target']
        original_hull = old['capture_target'].find("geom[@name='capture_target_collision']")
        hull = target.find("geom[@name='capture_target_collision']")
        assert hull.attrib == {**original_hull.attrib, 'contype':'0', 'conaffinity':'0'}
        pieces = [g for g in target.findall('geom') if g.get('name','').startswith('capture_x_piece_')]
        bus = [g for g in target.findall('geom') if g.get('name','').startswith('capture_bus_')]
        expected_pieces = manifest['real_window_collision']['convex_pieces']
        assert expected_pieces in (91, 898)
        assert len(pieces)==expected_pieces and len(bus)==5
        for g in pieces+bus:
            assert g.get('mass')=='0' and g.get('group')=='3'
            assert g.get('contype')=='2' and g.get('conaffinity')=='1'
        changed.update(g.get('name') for g in pieces+bus)
        changed.add('capture_target_collision')
    for name,b in old.items():
        assert new[name].attrib==b.attrib,name
        for tag in ['inertial','geom','joint','freejoint','camera','site']:
            assert [E.canonicalize(E.tostring(x,encoding='unicode'),strip_text=True) for x in b.findall(tag) if x.get('name') not in changed]==[E.canonicalize(E.tostring(x,encoding='unicode'),strip_text=True) for x in new[name].findall(tag) if x.get('name') not in changed],(name,tag)
    project=adapter/'Unreal/BskUnrealRenderer';folder=project/'Saved/AssetImport'
    profile=json.loads((ROOT/'config/scene_render_profiles.json').read_text(encoding='utf-8'))['sarm-dual-capture26']
    for camera_name, mount in profile.get('camera_mounts', {}).items():
        camera=added.find(f".//body[@name='{mount['body']}']/camera[@name='{camera_name}']")
        assert camera is not None, ('Missing mounted camera', camera_name)
        for attribute, value in mount.items():
            if attribute != 'body':
                assert camera.get(attribute) == value, ('Camera mount differs from scene profile', camera_name, attribute)
    base_catalog, capture_catalog, trim_catalog = profile['source_catalogs']
    catalog=json.loads((folder/base_catalog).read_text(encoding='utf-8-sig'))
    extra=json.loads((folder/capture_catalog).read_text(encoding='utf-8-sig'))
    catalog['assets'].update(extra['assets']);
    trim=folder/trim_catalog
    if trim.exists():catalog['assets'].update(json.loads(trim.read_text(encoding='utf-8-sig'))['assets'])
    # The upstream task-box arm does not render link6, but the duplicated
    # capture arm does. Reuse its byte-identical, already imported mesh.
    link6 = ROOT/'model/SARM_mjcf_20261012/meshes/link6.STL'
    capture_link6 = ROOT/'model/dual_arm_capture26/meshes/capture26_link6.STL'
    assert link6.read_bytes() == capture_link6.read_bytes()
    catalog['assets'][str(link6)] = dict(extra['assets'][str(capture_link6)])
    mapped = {str(Path(k).resolve()).casefold() for k in catalog['assets']}
    meshes = {m.get('name'): m for m in added.findall('asset/mesh')}
    for geom in added.iter('geom'):
        if not geom.get('mesh') or geom.get('group') == '3':
            continue
        if float(geom.get('rgba', '1 1 1 1').split()[-1]) == 0:
            continue
        source = (p.parent / meshes[geom.get('mesh')].get('file')).resolve()
        assert str(source).casefold() in mapped, ('Unmapped visible mesh', geom.get('name'), str(source))
    catalog['source_mjcf']=str(p)
    for entry in catalog['assets'].values():
        uasset=project/'Content'/(entry['asset_path'].split('.')[0].removeprefix('/Game/')+'.uasset')
        assert uasset.is_file(),uasset
    (folder/profile['asset_catalog']).write_text(json.dumps(catalog,indent=2),encoding='utf-8')
    report={'upstream_bodies_preserved':len(old),'additional_bodies':len(new)-len(old),'all_catalog_assets_present':True,'original_material_asset_paths_preserved_except_trimmed_base':True,'trimmed_base_geoms':len(manifest.get('mount_clearance_changes',[])), 'real_window_convex_pieces':manifest.get('real_window_collision', {}).get('convex_pieces',0)}
    print(json.dumps(report))
    return report
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--adapter-root',type=Path,required=True);args=ap.parse_args();verify(args.adapter_root)
