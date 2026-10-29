"""Portable opt-in capture fixture and reversible checkout-local runtime lease."""
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class PreparedTrial:
    output: Path
    scene: Path
    vision: Path
    recipe: dict


def _resolve(value, substitutions):
    if isinstance(value,dict):return {k:_resolve(v,substitutions) for k,v in value.items()}
    if isinstance(value,list):return [_resolve(v,substitutions) for v in value]
    if isinstance(value,str):
        for key,replacement in substitutions.items():value=value.replace(key,replacement)
    return value


def prepare_trial(output,vision_python,*,root=ROOT):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    folder=Path(root)/'config/experiments/visual_gyro_capture'
    recipe=json.loads((folder/'recipe.json').read_text('utf-8'))
    substitutions={'$OUTPUT':output.as_posix(),'$VISION_PYTHON':str(vision_python)}
    scene=_resolve(json.loads((folder/recipe['scene']).read_text('utf-8')),substitutions)
    runtime=_resolve(json.loads((folder/recipe['vision_runtime']).read_text('utf-8')),substitutions)
    random=scene['randomization'];visual=random['capture_visual_test']
    if (scene['template_id']!='sarm-dual-capture26' or not random['capture_window_trial']
            or not visual['enabled'] or visual['mode']!='predictive_grasp' or not visual['held_target_pose']
            or random['formation_hold']['enabled'] or not random['coorbital_initialization']['enabled']
            or random['coorbital_initialization']['reference_point']!='assembly_com'
            or random['coorbital_initialization']['grip_frame_standoff_m']!=recipe['initial_gap_m']):
        raise ValueError('capture recipe must use free coorbital visual/gyro capture')
    scene['instance_id']='visual-gyro-capture-'+uuid.uuid4().hex[:12]
    scene_path=output/'scene.json';runtime_path=output/'vision_runtime.json'
    for path,value in ((scene_path,scene),(runtime_path,runtime),(output/'recipe.json',recipe)):
        path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    (output/'capture').mkdir()
    return PreparedTrial(output,scene_path,runtime_path,recipe)


@contextmanager
def temporary_vision_runtime(root,supplied,*,cleanup=None):
    """No shared persistent config change, including on exceptions or nested runs."""
    run=Path(root)/'run';run.mkdir(parents=True,exist_ok=True)
    lock=run/'capture_regression.lock'
    try:descriptor=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY)
    except FileExistsError as error:raise RuntimeError('capture regression already owns this checkout; inspect its lock') from error
    with os.fdopen(descriptor,'w') as file:file.write(str(os.getpid()))
    paths=[run/'vision_runtime.json',run/'wrist_alignment_observation.json']
    previous={p:p.read_bytes() if p.exists() else None for p in paths}
    try:
        paths[0].write_bytes(Path(supplied).read_bytes())
        paths[1].unlink(missing_ok=True)
        yield
    finally:
        try:
            if cleanup is not None:cleanup()
        finally:
            for path,data in previous.items():
                if data is None:path.unlink(missing_ok=True)
                else:path.write_bytes(data)
            lock.unlink(missing_ok=True)


def physical_fingerprint(root=ROOT):
    model=Path(root)/'model/SARM/platform/sarm_dual_capture26.xml'
    xml=ET.parse(model).getroot()
    target=xml.find(".//body[@name='capture_target']")
    fixed=[]
    for tag in ('weld','connect'):
        for node in xml.findall(f'./equality/{tag}'):
            bodies={node.get('body1',''),node.get('body2','')}
            if 'capture_target' in bodies:fixed.append(dict(type=tag,**node.attrib))
    if fixed or (target.find('freejoint') is None and target.find("joint[@type='free']") is None):
        raise ValueError('capture target must be free with no fixed inter-spacecraft connection')
    inertials={body.get('name'):dict(body.find('inertial').attrib)
        for body in xml.iter('body') if body.find('inertial') is not None}
    # Includes every XML whose compiler/default/inertial declarations could be reused.
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted((Path(root)/'model').rglob('*.xml'))}
    return dict(model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        xml_sha256=hashes,inertials=inertials,target_has_freejoint=True,
        target_assembly_mass_kg=sum(float(i.get('mass')) for i in target.iter('inertial')),
        inter_satellite_fixed_constraints=fixed)
