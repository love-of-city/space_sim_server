"""Generate a candidate flush marker rig, retaining the supplied original.

Surface-only and user-authorized thin-film hole-bridging variants are separate.
"""
import argparse
import copy
import json
from pathlib import Path
import math


def generate(source_rig, black_side_m=.020, *, bridge_holes=False):
    if not 0 < black_side_m <= (.050 if bridge_holes else .020):
        raise ValueError('size exceeds the selected surface placement range')
    dx,dz = (.060,.150) if bridge_holes else (.06225,.12895)
    rig = copy.deepcopy(source_rig)
    rig.pop('view_joint_deg', None)
    rig['mounting'] = dict(mounting_plate=False, plate_thickness_m=0,
        physical_film_thickness_m=0, surface_y_m=.05475,
        render_depth_bias_m=.00001,
        layout_version='flush_5cm_v2' if bridge_holes else 'surface_20mm_candidate')
    rig.update(schema='space-arm-fiducials/2', black_side_m=black_side_m,
        white_side_m=black_side_m*4/3, surface_y_m=.05475,
        render_depth_bias_m=.00001, physical_thickness_m=0,
        grasp_center_target_m=[.1755,.05475,.0420787001299893],
        symmetric_offsets_m=[dx,dz],
        attachment='zero-thickness visual film; no mounting boards',
        hole_bridging_authorized=bridge_holes,
        physical_support=('partial, film bridges unused holes; no film structural model'
                          if bridge_holes else 'full flat support within mesh numerical tolerance'),
        numerical_surface_tolerance_m=1e-7)
    rows = []
    for marker in rig['markers']:
        identifier = marker['id']
        sx = -1 if identifier in (0,2) else 1
        sz = 1 if identifier in (0,1) else -1
        marker['center_m'] = [.1755+sx*dx, .05476, .0420787001299893+sz*dz]
        marker['surface_center_m'] = [marker['center_m'][0], .05475, marker['center_m'][2]]
        marker['size_m'] = black_side_m
        marker['right'] = [-1,0,0]
        marker['down'] = [0,0,-1]
        module = black_side_m/6
        for r in range(8):
            for c in range(8):
                value = 1 if r in (0,7) or c in (0,7) else int(marker['bits'][r-1][c-1])
                p = [marker['center_m'][0]-(c-3.5)*module, .05476,
                     marker['center_m'][2]-(r-3.5)*module]
                rows.append(dict(geometry_id=f'teleop/capture_target/aruco_flush/{identifier}/{r}_{c}',
                    shape='plane', dimensions_m=[module,module,0], position_body_m=p,
                    orientation_body_from_geometry_wxyz=[0,0,math.sqrt(.5),math.sqrt(.5)],
                    color_rgba=[float(value)]*3+[1.0], render_role='visual',
                    material_name='aruco_flush_matte', material_specular=0,
                    material_shininess=0, material_reflectance=0, material_emission=0))
    return rig, rows


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--source-rig', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--black-side-mm', type=float, default=20)
    p.add_argument('--bridge-holes', action='store_true')
    args = p.parse_args()
    rig, rows = generate(json.loads(args.source_rig.read_text('utf-8-sig')), args.black_side_mm/1000,
                         bridge_holes=args.bridge_holes)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, data in [('aruco_rig.json',rig), ('geometry_metres.json',rows)]:
        (args.output_dir/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8',newline='\n')
