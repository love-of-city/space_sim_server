"""Build a target asset from a versioned installation profile, locally only."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_profile(path):
    profile = json.loads(path.read_text(encoding='utf-8'))
    if profile.get('schema') != 'target-taskbox-profile/1':
        raise ValueError('Unsupported target asset profile')
    source = ROOT/profile['source_model']
    if hashlib.sha256(source.read_bytes()).hexdigest() != profile['source_sha256']:
        raise ValueError('Source asset changed: re-review the installation profile')
    scale = profile['scale']
    if not 0 < scale['value'] <= scale['max_value']:
        raise ValueError('Scale exceeds the reviewed limit')
    if profile['constraints']['taskbox_world_pose'] != 'fixed':
        raise ValueError('This generator requires fixed task hardware placement')
    return profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    profile = load_profile(args.profile)
    if profile['adapter'] == 'ground_validation_end_face':
        from build_end_face_scene import build
        if (profile['mount']['face_axis'],profile['mount']['face_sign']) != (2,-1):
            raise ValueError('Ground adapter supports only the reviewed -Z face')
        build(SimpleNamespace(source=ROOT/profile['source_model'],
            baseline=ROOT/profile['baseline'], output=args.output,
            scale=profile['scale']['value'], mount=profile['mount'],
            capture_joint_seed=profile['capture_joint_seed']))
    elif profile['adapter'] == 'static_visual_target':
        from build_visual_target import build
        build(profile, ROOT, args.output)
    else:
        raise ValueError('Unsupported asset adapter: '+profile['adapter'])
    from audit_target_asset import audit
    assembly_audit = audit(args.profile, args.output)
    record = dict(profile=profile, profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
                  generated=True, assembly_verified=assembly_audit['passed'],
                  native_verified=False, ue_verified=False, full_task_accepted=False)
    record['artifacts'] = {name: hashlib.sha256((args.output/name).read_bytes()).hexdigest()
                           for name in ('preheld-dual.xml', 'initial-scene.json')}
    (args.output/'generation.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
