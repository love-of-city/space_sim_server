"""Verify per-scene provenance and camera invariance of actual platform images."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--index',type=Path,help='Frozen installed index matching this image batch')
    parser.add_argument('--only',nargs='*',help='Verify selected task IDs')
    args=parser.parse_args()
    project=Path(__file__).resolve().parents[2]
    index=read(args.index or project/'run/imported_scene_library.json')
    entries=[e for e in index['entries'] if not args.only or e['id'] in args.only]
    results=[]
    camera_groups={}
    for entry in entries:
        directory=args.input/entry['scene_id']
        checks={}
        try:
            complete=read(directory/'complete.json')
            report=read(directory/'inspection.json')
            streams=[s for s in report['streams'] if s.get('available')]
            dual=entry['base_template_id']=='sarm-dual-capture26'
            checks['expected_stream_count']=len(streams)==(4 if dual else 3)
            checks['fresh_decoded_streams']=all(s['decoded_frames']>0 for s in streams)
            files=[]
            for stream in streams:
                for key in ('image','workbench'):
                    file=directory/stream[key]
                    data=file.read_bytes()
                    assert hashlib.sha256(data).hexdigest()==complete['images'][file.name]
                    assert data[:8]==b'\x89PNG\r\n\x1a\n'
                    width,height=struct.unpack('!II',data[16:24])
                    if key=='image':
                        assert (width,height)==(stream['width'],stream['height'])
                    files.append(file.name)
            checks['image_hashes_and_dimensions']=True
            if entry['kind']!='rendezvous':
                snapshots=[report['state']]+[s['state'] for s in streams]
                checks['current_scene_identity']=all(
                    s['scene_runtime']['instance']['library_scene']['id']==entry['id']
                    and s['simulation']['latest_observation']['scene_instance_id']==s['scene_runtime']['instance']['instance_id']
                    and s['simulation']['connected'] for s in snapshots)
                runtime=read(directory/'camera-configuration.json')
                assert len(runtime)==1
                source=read(Path(entry['source_instance']))
                checks['exact_model_loaded']=Path(runtime[0]['model'])==Path(source['model'])
                cameras=runtime[0]['cameras']
                checks['camera_count']=len(cameras)==(7 if dual else 2)
                canonical=json.dumps(cameras,sort_keys=True)
                group=camera_groups.setdefault(entry['base_template_id'],canonical)
                checks['same_system_camera_configuration']=group==canonical
                baseline=Path(entry['camera_baseline'])
                checks['baseline_unchanged']=hashlib.sha256(baseline.read_bytes()).hexdigest()==entry['camera_baseline_sha256']
                session=report['state']['simulation']['latest_observation']['render_session_id']
                renderer=(directory/'renderer.log').read_text(encoding='utf-8',errors='replace')
                checks['ue_manifest_applied']=bool(re.search(r'Applied BSK scene manifest session='+re.escape(session)+r' .*objects=[1-9]',renderer))
                if dual and report.get('docking_stereo_required', True):
                    vision=report.get('vision_images',[])
                    checks['stereo_current_session']=len(vision)==2 and all(v['session_id']==session for v in vision) and not report['vision']['stale']
                    for item in vision:
                        file=directory/item['file']
                        assert hashlib.sha256(file.read_bytes()).hexdigest()==complete['images'][file.name]
                        files.append(file.name)
            else:
                checks['live_flight']=report['state']['mission']['stage']=='flight' and report['state']['mission']['telemetry']['connected']
            results.append(dict(id=entry['id'],loaded_and_captured=all(checks.values()),checks=checks,images=files,
                camera_coverage='needs_layout_review' if dual else 'user_review_pending'))
        except Exception as error:
            results.append(dict(id=entry['id'],loaded_and_captured=False,checks=checks,error=str(error)))
    output=dict(schema='platform-scene-visual-inspection/1',scope='real system import and original-camera screenshots; not a new full-task physical acceptance',
        camera_mounts_changed=False,all_views_satisfy_operation_requirements=False,
        loaded_and_captured=sum(r['loaded_and_captured'] for r in results),expected=len(entries),scenes=results)
    (args.input/'verification.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'loaded_and_captured':output['loaded_and_captured'],'expected':len(entries),'failures':[r for r in results if not r['loaded_and_captured']]},ensure_ascii=False))


if __name__=='__main__':
    main()
