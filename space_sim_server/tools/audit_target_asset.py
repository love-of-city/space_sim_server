"""Independent XML assembly invariants; does not claim collision or task success."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from simulation.scene_library.taskbox_mount import _world_pose
from simulation.scene_library.preserved_target import INTERFACE_GEOMS
from simulation.scene_library.manifest import digest


def audit(profile_path,case):
    profile=json.loads(profile_path.read_text(encoding='utf-8'))
    baseline=ROOT/profile['baseline']
    a=ET.parse(baseline/'preheld-dual.xml').getroot();b=ET.parse(case/'preheld-dual.xml').getroot()
    def tree(n):return n.tag,dict(n.attrib),[tree(c) for c in n]
    checks={'service_arms_and_service_taskbox_unchanged':tree(a.find('.//body[@name="cubesat_bus"]'))==tree(b.find('.//body[@name="cubesat_bus"]')),
            'original_camera_attributes_unchanged':[n.attrib for n in a.iter('camera')]==[n.attrib for n in b.iter('camera')]}
    errors=[]
    for name in ('task_box_base','drawer_base','aviation_plug','guide_plug','cylindrical_plug','bolt'):
        x=a.find(f'.//body[@name="{name}"]');y=b.find(f'.//body[@name="{name}"]')
        for node in x.iter('body'):
            match=b.find(f'.//body[@name="{node.get("name")}"]')
            if match is None:errors.append(node.get('name'));continue
            p,r=_world_pose(a,node);q,s=_world_pose(b,match)
            if not np.allclose(p,q,atol=1e-10,rtol=0) or not np.allclose(r,s,atol=1e-10,rtol=0):errors.append(node.get('name'))
        for key in ('pos','quat'):x.attrib.pop(key,None);y.attrib.pop(key,None)
        if tree(x)!=tree(y):errors.append(name+':mechanism')
    checks['complete_task_modules_and_world_poses_preserved']=not errors
    sa=json.loads((baseline/'initial-scene.json').read_text(encoding='utf-8'))
    sb=json.loads((case/'initial-scene.json').read_text(encoding='utf-8'))
    checks['operating_arm_initial_joints_unchanged']=sa['randomization']['arm_joint_position_rad']==sb['randomization']['arm_joint_position_rad']
    # Reparse because the structural comparison above removed mount attributes.
    b=ET.parse(case/'preheld-dual.xml').getroot()
    _,r=_world_pose(b,b.find('.//body[@name="task_box_base"]'));_,s=_world_pose(b,b.find('.//body[@name="service_task_box_base"]'))
    checks['L_shape_preserved']=abs(float(r[:,2]@s[:,2]))<1e-10
    if profile['adapter']=='ground_validation_end_face':
        original=ET.parse(ROOT/profile['source_model']).getroot()
        checks['original_interface_metric_geometry_preserved']=all(b.find(f'.//geom[@name="{n}"]').attrib==original.find(f'.//geom[@name="{n}"]').attrib for n in INTERFACE_GEOMS)
        expected=original.find('.//body[@name="satellite_inner_panel"]')
        actual=b.find('.//body[@name="satellite_inner_panel"]')
        checks['complete_solar_hierarchy_present']=[n.get('name') for n in expected.iter()]==[n.get('name') for n in actual.iter()]
        from simulation.scene_library.preserved_target import DEFAULT_PIVOT
        scale=profile['scale']['value'];solar_errors=[]
        actual_nodes={n.get('name'):n for n in actual.iter() if n.get('name')}
        for node in expected.iter():
            if not node.get('name'):continue
            match=actual_nodes.get(node.get('name'))
            if match is None:solar_errors.append(node.get('name'));continue
            for key,value in node.attrib.items():
                if key=='mesh':
                    am=original.find(f'./asset/mesh[@name="{value}"]');bm=b.find(f'./asset/mesh[@name="{match.get(key)}"]')
                    good=np.allclose(np.fromstring(am.get('scale','1 1 1'),sep=' ')*scale,np.fromstring(bm.get('scale','1 1 1'),sep=' '),atol=1e-10,rtol=1e-10)
                elif key in ('pos','size','fromto','margin','gap','diaginertia','fullinertia'):
                    wanted=np.fromstring(value,sep=' ')*(scale**2 if key in ('diaginertia','fullinertia') else scale)
                    if node is expected and key=='pos':wanted+=(1-scale)*np.asarray(DEFAULT_PIVOT)
                    good=np.allclose(wanted,np.fromstring(match.get(key,''),sep=' '),atol=1e-10,rtol=1e-10)
                else:good=match.get(key)==value
                if not good:solar_errors.append(node.get('name')+':'+key)
        checks['solar_uniform_scale_structure_and_hinge_angles_preserved']=not solar_errors
        errors.extend(solar_errors)
        checks['fixed_visual_attachments_preserved']=sa.get('render_profile')==sb.get('render_profile')
        from simulation.scene_library.hull_closure import audit_hull_closure
        closure=json.loads((case/'hull-closure.json').read_text(encoding='utf-8'))
        closure_result=audit_hull_closure(b,closure)
        checks['all_hull_faces_closed_except_task_aperture']=closure_result['passed']
        (case/'hull-closure-audit.json').write_text(json.dumps(closure_result,indent=2)+'\n',encoding='utf-8')
    result=dict(schema='target-asset-assembly-audit/1',model_sha256=digest(case/'preheld-dual.xml'),
        checks=checks,passed=all(checks.values()),errors=errors,collision_verified=False,native_dynamics_verified=False,full_task_accepted=False)
    (case/'asset-audit.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if not result['passed']:raise ValueError(result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',type=Path,required=True);p.add_argument('--case',type=Path,required=True)
    args=p.parse_args();print(json.dumps(audit(args.profile,args.case)))
