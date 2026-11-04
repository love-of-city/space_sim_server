"""Read-only projection and same-plane footprint checks for outer markers.

This does not render images, demonstrate decode range, alter marker geometry,
or validate thin-film structure / the grasping contact envelope.
"""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path

import numpy as np


def overlaps(a,b):
    # Current main-face marker axes are target -X and -Z. Include white quiet
    # zone (8 modules) around the six-module outer black square.
    if abs(a['center_m'][1]-b['center_m'][1])>1e-6:
        return False
    displacement=np.abs(np.array(a['center_m'])[[0,2]]-np.array(b['center_m'])[[0,2]])
    extent=(a['size_m']+b['size_m'])*2/3
    return bool(np.all(displacement<extent-1e-10))


def inspect(config_path,estimator_path):
    config=json.loads(config_path.read_text('utf-8-sig'))
    source=ast.parse(estimator_path.read_text('utf-8-sig'))
    init=next(n for c in source.body if isinstance(c,ast.ClassDef) and c.name=='MultiCameraPoseEstimator'
              for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    defaults={arg.arg:ast.literal_eval(value) for arg,value in zip(init.args.kwonlyargs,init.args.kw_defaults)}
    minimum=defaults['minimum_marker_side_px']
    focal=config['calibration']['K_left'][0][0]
    original=config['marker_rig']['markers']
    big=[m['id'] for m in original if np.isclose(m['size_m'],.05)]
    if len(big)!=4 or len(original)!=23:
        raise ValueError('expected original 23-marker rig with four 5 cm outer markers')
    results=[]
    for size in (.05,.055,.06,.07,.08,.10):
        markers=copy.deepcopy(original)
        for marker in markers:
            if marker['id'] in big:
                marker['size_m']=size
        pairs=[[a['id'],b['id']] for i,a in enumerate(markers) for b in markers[i+1:]
               if (a['id'] in big or b['id'] in big) and overlaps(a,b)]
        results.append(dict(black_side_cm=size*100,white_side_cm=size*4/3*100,
            edge_px_at_depth={str(z):focal*size/z for z in (17.4,12,10,5.5,3.36,.5)},
            ideal_frontal_depth_at_current_side_gate_m=focal*size/minimum,
            ideal_frontal_depth_at_24px_m=focal*size/24,
            ideal_depth_at_45deg_current_gate_m=focal*size*np.cos(np.deg2rad(45))/minimum,
            same_plane_white_footprint_overlap_pairs=pairs))
    selective=[]
    for identifier in big:
        for size in (.06,.07,.08):
            candidate=copy.deepcopy(next(m for m in original if m['id']==identifier))
            candidate['size_m']=size
            conflicts=[m['id'] for m in original if m['id']!=identifier and overlaps(candidate,m)]
            selective.append(dict(id=identifier,black_side_cm=size*100,white_side_cm=size*4/3*100,
                overlap_other_ids=conflicts))
    return dict(config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        detector_source_sha256=hashlib.sha256(estimator_path.read_bytes()).hexdigest(),
        actual_minimum_side_px=minimum,actual_min_markers_per_eye=defaults['min_markers_per_eye'],
        focal_px=focal,outer_marker_ids=big,near_marker_ids_preserved=[m['id'] for m in original if m['id'] not in big],
        candidates=results,one_marker_only_candidates=selective,requirements={str(z):dict(black_side_cm_for_current_gate=z*minimum/focal*100,
            white_side_cm_for_current_gate=z*minimum/focal*4/3*100) for z in (5.5,10,17.4)},
        actual_image_detection_tested=False,grasp_error_revalidated=False,
        limitations=['all sizes keep original marker centres; candidates are not installed',
            'f*s/z and tilted approximation are sampling limits, not measured detection ranges',
            'quiet-zone overlap can interfere with decoding and must be resolved before any larger-marker trial',
            'no panel mesh-support, contact-envelope or real film thickness claim'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--estimator-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=inspect(args.config,args.estimator_source)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
