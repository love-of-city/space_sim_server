"""Independent initial preserved-face geometry audit in Python MuJoCo only.

Checks explicit collision pairs, actual visual triangles against the integral
enclosure and reserved rear volume, and original render-only fiducial geometry.
This does not run native dynamics or establish physical grip/task acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.scene_library.taskbox_mount import _world_pose, _pose

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(base,output):
    model_path=base/'preheld-dual.xml'; scene_path=base/'initial-scene.json'
    assembly=json.loads((base/'assembly.json').read_text(encoding='utf-8-sig'))
    if not assembly.get('capture_pose_found'):raise ValueError('Capture preparation incomplete')
    root=ET.parse(model_path).getroot();scene=json.loads(scene_path.read_text(encoding='utf-8-sig'));init=scene['randomization']
    m=load_geometry_model(model_path);d=mj.MjData(m)
    for (_,n,_),v in zip(INITIAL_JOINTS,init['preheld_capture_joint_position']):d.qpos[m.joint(n).qposadr[0]]=v
    for n,v in zip([*(f'joint{i}' for i in range(1,7)),'ee_link11_joint','ee_link12_joint'],init['arm_joint_position_rad']):d.qpos[m.joint(n).qposadr[0]]=v
    adr=int(m.joint('capture_target_free').qposadr[0]);d.qpos[adr:adr+3]=init['target_position_m'];d.qpos[adr+3:adr+7]=init['target_orientation_wxyz'];mj.mj_forward(m,d)
    def descendants(n):return {b.get('name') for b in root.find(f'.//body[@name="{n}"]').iter('body')}
    target=descendants('capture_target');hardware=descendants('task_box_base')|descendants('drawer_base')
    loose={'aviation_plug','guide_plug','cylindrical_plug','bolt'};hardware|=loose
    capture=descendants('bottom_mount');operating=descendants('link1');service=descendants('cubesat_bus')-capture-operating
    body=[m.body(int(m.geom_bodyid[g])).name for g in range(m.ngeom)]
    active=np.asarray(m.geom_contype|m.geom_conaffinity,dtype=bool)
    groups={
        'target_structural':[g for g,b in enumerate(body) if b in target-hardware and active[g]],
        'hardware':[g for g,b in enumerate(body) if b in hardware and active[g]],
        'target_all':[g for g,b in enumerate(body) if b in target|loose and active[g]],
        'service':[g for g,b in enumerate(body) if b in service and active[g]],
        'interface':[g for g in range(m.ngeom) if m.geom(g).name.startswith('capture_x_piece_')],
        'gripper_interface':[g for g in range(m.ngeom) if m.geom(g).name.startswith(('collision_ring_convex_','collision_jaw_'))],
    }
    def bounds():
        R=d.geom_xmat.reshape(-1,3,3);c=d.geom_xpos+np.einsum('nij,nj->ni',R,m.geom_aabb[:,:3]);e=np.einsum('nij,nj->ni',np.abs(R),m.geom_aabb[:,3:]);return c-e,c+e
    def pair_check(left,right,limit=1e-6):
        lo,hi=bounds();tested=separated=unsupported=0;hits=[];near=[];minval=.005
        for a in left:
            ids=np.asarray(right,dtype=int);mask=np.all(hi[a]+.005>=lo[ids],axis=1)&np.all(hi[ids]+.005>=lo[a],axis=1)
            separated+=int((~mask).sum())
            for b in ids[mask]:
                if int(m.geom_type[a])==int(mj.mjtGeom.mjGEOM_SDF) or int(m.geom_type[b])==int(mj.mjtGeom.mjGEOM_SDF):unsupported+=1;continue
                dist=float(mj.mj_geomDistance(m,d,a,int(b),.005,None));tested+=1;minval=min(minval,dist)
                if dist < -limit:hits.append(dict(first=m.geom(a).name,second=m.geom(int(b)).name,separation_m=dist))
                elif dist<1e-7:near.append(dict(first=m.geom(a).name,second=m.geom(int(b)).name,separation_m=dist))
        return dict(total_pairs=len(left)*len(right),aabb_separated_over_5mm=separated,distance_pairs=tested,unsupported_pairs=unsupported,minimum_distance_capped_5mm=minval,penetrations=hits,near_zero_pair_count=len(near),near_zero_examples=near[:20],penetration_tolerance_m=limit)
    checks={}
    for a,b in [('target_structural','hardware'),('target_all','service'),('interface','gripper_interface')]:
        print('Checking '+a+' vs '+b,flush=True);checks[a+'__'+b]=pair_check(groups[a],groups[b],.001 if a=='interface' else 1e-6)
    # Collect all actual interface contact depths, including ones inside the
    # allowed 1 mm soft-contact bound; do not hide them as intentional.
    interface_contacts=[];other_arm_contacts=[];all_other_arm_contacts=[]
    for c in d.contact:
        names=[m.geom(int(g)).name for g in c.geom];bb=[body[int(g)] for g in c.geom]
        expected=any(n.startswith('capture_x_piece_') for n in names) and any(n.startswith(('collision_ring_convex_','collision_jaw_')) for n in names)
        row=dict(geoms=names,bodies=bb,distance_m=float(c.dist))
        if expected:interface_contacts.append(row)
        elif any(b in capture|operating for b in bb):
            all_other_arm_contacts.append(row)
            if c.dist < -1e-6:other_arm_contacts.append(row)
    tp=d.body('capture_target').xpos.copy();tr=d.body('capture_target').xmat.reshape(3,3).copy()
    bp=d.body('task_box_base').xpos.copy();br=d.body('task_box_base').xmat.reshape(3,3).copy()
    source_root=ET.parse(assembly['source_model']).getroot()
    source_parents={child:parent for parent in source_root.iter() for child in parent}
    loose_pose=[]
    for name in sorted(loose):
        src=source_root.find(f'.//body[@name="{name}"]');source_p,source_r=_world_pose(source_root,src)
        if name=='bolt':
            anchor_name='task_box_base';anchor_type='body'
            ap,ar=_world_pose(source_root,source_root.find('.//body[@name="task_box_base"]'))
            current_ap,current_ar=bp,br
        else:
            anchor_name=name.removesuffix('_plug')+'_pickup';anchor_type='site'
            site=source_root.find(f'.//site[@name="{anchor_name}"]');owner=source_parents[site]
            while owner.tag!='body':owner=source_parents[owner]
            op,orr=_world_pose(source_root,owner);sp,sr=_pose(site);ap,ar=op+orr@sp,orr@sr
            sid=m.site(anchor_name).id;current_ap=d.site_xpos[sid];current_ar=d.site_xmat[sid].reshape(3,3)
        relative_p=ar.T@(source_p-ap);relative_r=ar.T@source_r
        expected_p=current_ap+current_ar@relative_p;expected_r=current_ar@relative_r
        actual=d.body(name);actual_r=actual.xmat.reshape(3,3)
        angle=float(np.arccos(np.clip((np.trace(expected_r.T@actual_r)-1)/2,-1.,1.)))
        loose_pose.append(dict(body=name,anchor=anchor_name,anchor_type=anchor_type,source_relative_position_m=relative_p.tolist(),source_relative_rotation=relative_r.tolist(),anchor_world_position_m=current_ap.tolist(),anchor_world_rotation=current_ar.tolist(),expected_world_position_m=expected_p.tolist(),actual_world_position_m=actual.xpos.tolist(),expected_world_rotation=expected_r.tolist(),actual_world_rotation=actual_r.tolist(),position_error_m=float(np.linalg.norm(actual.xpos-expected_p)),angle_error_rad=angle))
    visual=[g for g,b in enumerate(body) if b in target-hardware and int(m.geom_group[g])==2]
    protected=set(assembly['housing']['protected_geometry'])
    mesh_cache={}
    def mesh_world(g):
        mid=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mid]);n=int(m.mesh_vertnum[mid]);fa=int(m.mesh_faceadr[mid]);fn=int(m.mesh_facenum[mid])
        v=m.mesh_vert[a:a+n].astype(float)@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        return v,m.mesh_face[fa:fa+fn]
    # SAT triangle/AABB test; strict inward tolerance excludes a shared flush
    # attachment plane while retaining actual triangle penetration evidence.
    def intersects(tri,low,high,tol=1e-7):
        low=np.asarray(low)+tol;high=np.asarray(high)-tol
        if np.any(high<=low):return np.zeros(len(tri),dtype=bool)
        keep=np.all(tri.max(1)>=low,1)&np.all(tri.min(1)<=high,1)
        idx=np.flatnonzero(keep)
        if not len(idx):return keep
        t=tri[idx]-(low+high)/2;ext=(high-low)/2
        edges=[t[:,1]-t[:,0],t[:,2]-t[:,1],t[:,0]-t[:,2]]
        axes=[np.cross(edges[0],edges[1])]
        for edge in edges:
            for basis in np.eye(3):axes.append(np.cross(edge,basis))
        hit=np.ones(len(idx),dtype=bool)
        for ax in axes:
            pro=np.einsum('nji,ni->nj',t,ax);radius=np.abs(ax)@ext
            hit&=(pro.min(1)<=radius+1e-12)&(pro.max(1)>=-radius-1e-12)
        keep[idx]=hit;return keep
    shells=[]
    for g,b in enumerate(body):
        if m.geom(g).name.startswith('taskbox_enclosure_'):
            center=(d.geom_xpos[g]-bp)@br;rotation=br.T@d.geom_xmat[g].reshape(3,3);extent=np.abs(rotation)@m.geom_size[g,:3]
            shells.append((m.geom(g).name,center-extent,center+extent))
    visual_hits=[];reserved=[];visual_pair_count=0;nonmesh=[]
    for g in visual:
        name=m.geom(g).name
        if int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_MESH):
            world,faces=mesh_world(g);local=(world-bp)@br;target_local=(world-tp)@tr
            mesh_cache[g]=(target_local,faces)
            tris=local[faces]
            for shell,low,high in shells:
                mask=intersects(tris,low,high);visual_pair_count+=1
                if np.any(mask):visual_hits.append(dict(geom=name,body=body[g],enclosure=shell,intersecting_triangles=int(mask.sum()),protected=name in protected,task_frame_min=local.min(0).tolist(),task_frame_max=local.max(0).tolist()))
            mask=intersects(tris,[0.,0.,-.102],[.45,.55,-.025],tol=1e-6)
            if np.any(mask):reserved.append(dict(geom=name,body=body[g],intersecting_triangles=int(mask.sum()),protected=name in protected,task_frame_min=local.min(0).tolist(),task_frame_max=local.max(0).tolist()))
        elif int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_BOX):
            signs=np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])
            faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
            world=(signs*m.geom_size[g,:3])@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
            local=(world-bp)@br;tris=local[faces]
            for shell,low,high in shells:
                mask=intersects(tris,low,high);visual_pair_count+=1
                if np.any(mask):visual_hits.append(dict(geom=name,body=body[g],enclosure=shell,intersecting_triangles=int(mask.sum()),protected=name in protected,source_primitive='box'))
            mask=intersects(tris,[0.,0.,-.102],[.45,.55,-.025],tol=1e-6)
            if np.any(mask):reserved.append(dict(geom=name,body=body[g],intersecting_triangles=int(mask.sum()),protected=name in protected,source_primitive='box'))
        else:nonmesh.append(name)
    print('Visual triangle checks complete',flush=True)
    # All 260 original fiducial primitives are render attachments, not MuJoCo
    # geoms. Test actual target visual triangles against their original boxes.
    attachment=scene['render_profile']['visual_attachments'][0];tagpath=ROOT/attachment['geometry_file'];tags=json.loads(tagpath.read_text(encoding='utf-8'))
    tag_hits=[];new_tag_hits=[]
    for tag in tags:
        center=np.asarray(tag['position_body_m']);half=np.asarray(tag['dimensions_m'])/2;low=center-half;high=center+half
        if not np.allclose(tag['orientation_body_from_geometry_wxyz'],[1,0,0,0]):raise ValueError('Tag OBB rotation needs explicit audit')
        for g,(v,f) in mesh_cache.items():
            if np.any(v.max(0)<low) or np.any(v.min(0)>high):continue
            mask=intersects(v[f],low,high,tol=1e-8)
            if np.any(mask):
                row=dict(primitive=tag['geometry_id'],geom=m.geom(g).name,protected=m.geom(g).name in protected,triangles=int(mask.sum()))
                tag_hits.append(row)
                if not row['protected']:new_tag_hits.append(row)
    # Axis-aligned non-mesh front extensions can be tested directly.
    for g in visual:
        if int(m.geom_type[g])!=int(mj.mjtGeom.mjGEOM_BOX):continue
        c=(d.geom_xpos[g]-tp)@tr;R=tr.T@d.geom_xmat[g].reshape(3,3);e=np.abs(R)@m.geom_size[g,:3];lo=c-e;hi=c+e
        for tag in tags:
            c2=np.array(tag['position_body_m']);e2=np.array(tag['dimensions_m'])/2;overlap=np.minimum(hi,c2+e2)-np.maximum(lo,c2-e2)
            if np.all(overlap>1e-8):
                row=dict(primitive=tag['geometry_id'],geom=m.geom(g).name,protected=m.geom(g).name in protected,overlap_axis_m=overlap.tolist());tag_hits.append(row)
                if not row['protected']:new_tag_hits.append(row)
    original_groups=m.geom_group.copy()
    for g,b in enumerate(body):
        if b not in target-hardware:m.geom_group[g]=5
    raygroups=np.array([0,0,1,0,0,0],dtype=np.uint8);rays=[]
    rig=json.loads((tagpath.parent/'aruco_rig.json').read_text(encoding='utf-8'))
    for marker in rig['markers']:
        c=np.array(marker['center_m']);right=np.array(marker['right']);down=np.array(marker['down']);size=marker['size_m']
        for rr,cc in np.ndindex(6,6):
            q=c+right*((cc+.5)/6-.5)*size+down*((rr+.5)/6-.5)*size
            origin=tp+tr@(q+np.array([0,.2,0]));direction=tr@np.array([0.,-1.,0.]);gid=np.array([-1],dtype=np.int32)
            dist=float(mj.mj_ray(m,d,origin,direction,raygroups,1,-1,gid))
            if 0<=dist<.2-1e-6:rays.append(dict(marker=marker['id'],row=rr,col=cc,occluder=m.geom(int(gid[0])).name,occlusion_depth_m=.2-dist,protected=m.geom(int(gid[0])).name in protected))
    m.geom_group[:]=original_groups
    # A clear actual-triangle / complete hardware-envelope test
    # proves no structural visual intersects any hardware inside that envelope.
    # It also covers protruding workpieces and loose donor parts, not just the
    # hollow rear enclosure. Positive hits remain conservative blockers.
    all_hardware=[g for g,b in enumerate(body) if b in hardware]
    hlo=[];hhi=[]
    for g in all_hardware:
        if int(m.geom_type[g]) in (int(mj.mjtGeom.mjGEOM_MESH),int(mj.mjtGeom.mjGEOM_SDF)) and 0<=int(m.geom_dataid[g])<m.nmesh:
            vertices=(mesh_world(g)[0]-bp)@br;hlo.append(vertices.min(0));hhi.append(vertices.max(0))
        else:
            R=d.geom_xmat[g].reshape(3,3);c=(d.geom_xpos[g]+R@m.geom_aabb[g,:3]-bp)@br;e=np.abs(br.T@R)@m.geom_aabb[g,3:]
            hlo.append(c-e);hhi.append(c+e)
    hlow=np.min(hlo,0);hhigh=np.max(hhi,0);full_hits=[]
    for g in visual:
        if g in mesh_cache:
            v,f=mesh_cache[g];local=(v@tr.T+tp-bp)@br
        elif int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_BOX):
            signs=np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])
            f=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
            world=(signs*m.geom_size[g,:3])@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];local=(world-bp)@br
        else:continue
        mask=intersects(local[f],hlow,hhigh,tol=1e-6)
        if np.any(mask):full_hits.append(dict(geom=m.geom(g).name,body=body[g],intersecting_triangles=int(mask.sum()),protected=m.geom(g).name in protected))
    # Verify physical back placement and complete in-plane footprint, then
    # push the drawer to its authored inward limit without editing the XML.
    rear=m.geom('taskbox_enclosure_back').id;rear_center=(d.geom_xpos[rear]-tp)@tr;rear_R=tr.T@d.geom_xmat[rear].reshape(3,3);rear_ext=np.abs(rear_R)@m.geom_size[rear,:3]
    rear_bounds=np.array([rear_center-rear_ext,rear_center+rear_ext]);housing=np.asarray(assembly['housing']['housing_bounds_m'])
    back_mount=assembly.get('mount_face')=='back'
    wall_id=m.geom('target_instance_0018_part_009_color_00' if back_mount else 'target_instance_0024_part_011_color_00').id
    wall_vertices=mesh_cache[wall_id][0];wall_bounds=np.array([wall_vertices.min(0),wall_vertices.max(0)])
    dq=m.joint('drawer_base_slide');d.qpos[dq.qposadr[0]]=float(m.jnt_range[dq.id,0]);mj.mj_forward(m,d)
    drawer=descendants('drawer_base');dg=[g for g,b in enumerate(body) if b in drawer and active[g]];sg=[m.geom(n).id for n,_,_ in shells]
    drawer_check=pair_check(dg,sg,1e-6)
    vertices=[]
    for g in dg:
        if int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_MESH):vertices.append((mesh_world(g)[0]-bp)@br)
    drawer_min=float(np.concatenate(vertices)[:,2].min());rear_inner=-.100
    normal_axis,back_index,wall_index,plane_axes=(1,1,0,(0,2)) if back_mount else (0,0,1,(1,2))
    flush_error=float(rear_bounds[back_index,normal_axis]-housing[wall_index,normal_axis])
    face_fit=all(rear_bounds[0,a]>=wall_bounds[0,a]-1e-7 and rear_bounds[1,a]<=wall_bounds[1,a]+1e-7 for a in plane_axes)
    gates=dict(collision_groups_complete=all(groups.values()) and all(c['unsupported_pairs']==0 for c in checks.values()),structural_vs_hardware_clear=not checks['target_structural__hardware']['penetrations'],target_vs_service_clear=not checks['target_all__service']['penetrations'],original_interface_penetration_under_1mm=not checks['interface__gripper_interface']['penetrations'],no_other_arm_initial_penetration=not other_arm_contacts,no_visual_mesh_enclosure_intersection=not visual_hits,no_structural_visual_intrusion_in_taskbox_rear=not reserved,no_new_fiducial_primitive_intersection=not new_tag_hits,black_frame_grid_front_clear=not rays,tag_count_260=len(tags)==260,tag_geometry_hash_unchanged=sha(tagpath)==attachment['sha256'],tag_transform_fixed=attachment['transform_mode']=='fixed_body_geometry',back_flush_to_hull=abs(flush_error)<1e-6,back_footprint_on_side_wall=face_fit,drawer_inward_clear=not drawer_check['penetrations'],drawer_rear_gap_at_least_4p99mm=drawer_min-rear_inner>=.00499)
    report=dict(schema='preserved-face-initial-audit/1',model_sha256=sha(model_path),scene_sha256=sha(scene_path),validator_sha256=sha(__file__),scope='initial explicit collisions, actual visual triangle enclosure/tag tests, 144 front-normal tag rays; not native dynamics',geometry_counts={k:len(v) for k,v in groups.items()},collision_checks=checks,interface_contact_count=len(interface_contacts),maximum_interface_contact_penetration_m=max([max(0.,-c['distance_m']) for c in interface_contacts],default=0.),interface_contacts=interface_contacts,other_arm_contacts=other_arm_contacts,visual_enclosure_triangle_checks=visual_pair_count,visual_enclosure_intersections=visual_hits,structural_visual_in_taskbox_rear_volume=reserved,visual_nonmesh_checked_for_tags=nonmesh,tag_primitive_count=len(tags),tag_primitive_intersections=tag_hits,new_tag_primitive_intersections=new_tag_hits,tag_black_frame_ray_count=144,tag_black_frame_occluded_rays=rays,historical_fiducial_limitation='Original white-margin/rib overlap around 0.11 mm was documented in the historical plate asset; protected-geometry intersections are reported separately, not silently attributed to new housing.',mount=dict(rear_back_bounds_target_m=rear_bounds.tolist(),housing_bounds_target_m=housing.tolist(),flush_error_m=flush_error,full_footprint_on_wall=face_fit),drawer_inward=dict(q_m=float(m.jnt_range[dq.id,0]),minimum_source_z_m=drawer_min,rear_inner_source_z_m=rear_inner,rear_gap_m=drawer_min-rear_inner,collisions=drawer_check),checks=gates,passed=all(gates.values()),task_accepted=False,limitations=['Collision mesh distances may report zero for separated mesh pairs; clearance is not inferred from zero.','Visual triangle tests cover integral shell and reserved rear box volume; they do not establish all visual-visual pair clearance elsewhere.','Front-normal fiducial rays are a geometric mounting check, not a calibrated real-camera visibility or PnP acceptance.','Protected historical fiducial intersections require separate interpretation; positive new intersections fail.'])
    report['mount']['actual_X_side_wall_bounds_target_m']=wall_bounds.tolist()
    if assembly.get('recessed_taskbox'):
        pocket_path=base/'recess-contract.json';pocket=json.loads(pocket_path.read_text(encoding='utf-8'))
        surface=float(((bp+br@np.array([.225,.275,0.])-tp)@tr)[1])
        # Work face, rather than the internal liner rear, is now flush.
        report['mount'].update(mode='recessed_work_surface',work_surface_y_target_m=surface,
            internal_depth_m=float(rear_bounds[1,1]-surface),pocket_bounds_target_m=pocket['pocket_bounds_target_m'],
            recess_contract_sha256=sha(pocket_path),flush_error_m=surface-float(housing[0,1]))
        report['checks'].pop('back_flush_to_hull')
        report['checks']['work_surface_flush_to_hull']=bool(abs(surface-housing[0,1])<1e-6)
        pl,ph=np.asarray(pocket['pocket_bounds_target_m']);intrusions=[]
        checked=0
        for g in visual:
            if g in mesh_cache:
                vertices,faces=mesh_cache[g]
            elif int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_BOX):
                vertices=((signs*m.geom_size[g,:3])@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]-tp)@tr
                faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
            else:continue
            checked+=1
            if np.any(intersects(vertices[faces],pl,ph,tol=1e-6)):intrusions.append(m.geom(g).name)
        report['recess_visual_pocket_checks']=dict(checked=checked,intrusions=intrusions)
        report['checks']['recess_visual_pocket_clear']=not intrusions
    if back_mount:
        report['mount'].pop('actual_X_side_wall_bounds_target_m')
        report['mount']['face']='-Y'
        report['mount']['actual_back_wall_bounds_target_m']=wall_bounds.tolist()
    report['other_arm_penetration_tolerance_m']=1e-6
    report['other_arm_contact_count']=len(all_other_arm_contacts)
    report['minimum_other_arm_contact_distance_m']=min((c['distance_m'] for c in all_other_arm_contacts),default=None)
    report['all_other_arm_initial_contacts']=all_other_arm_contacts
    report['mount']['back_footprint_Y_edge_clearances_m']=[float(rear_bounds[0,1]-wall_bounds[0,1]),float(wall_bounds[1,1]-rear_bounds[1,1])]
    if back_mount:
        report['mount'].pop('back_footprint_Y_edge_clearances_m')
        report['mount']['back_footprint_X_edge_clearances_m']=[float(rear_bounds[0,0]-wall_bounds[0,0]),float(wall_bounds[1,0]-rear_bounds[1,0])]
    report['checks']['all_target_visual_primitive_types_supported']=not nonmesh
    report['visual_primitive_count_supported']=len(visual)-len(nonmesh)
    report['enclosure_geom_count']=len(shells)
    report['complete_hardware_envelope_task_frame_m']=[hlow.tolist(),hhigh.tolist()]
    report['complete_hardware_geom_count']=len(all_hardware)
    report['complete_hardware_envelope_method']='Exact mesh/SDF-render-mesh vertices; conservative support bounds only for primitives'
    report['visual_complete_hardware_envelope_intersections']=full_hits
    report['visual_complete_hardware_envelope_check_count']=len(visual)-len(nonmesh)
    report['checks']['no_structural_visual_intrusion_in_complete_hardware_envelope']=not full_hits
    report['loose_source_pose_contract']=dict(source_model_sha256=sha(assembly['source_model']),position_tolerance_m=1e-6,angle_tolerance_rad=1e-6,bodies=loose_pose)
    report['checks']['loose_source_slot_relative_poses_preserved']=all(r['position_error_m']<=1e-6 and r['angle_error_rad']<=1e-6 for r in loose_pose)
    report['tag_geometry_sha256']=sha(tagpath)
    report['tag_transform_mode']=attachment['transform_mode']
    report['tag_attachment_scale']=attachment.get('scale',[1.,1.,1.])
    report['tag_markers']=[dict(id=t['id'],center_m=t['center_m'],size_m=t['size_m'],right=t['right'],down=t['down']) for t in rig['markers']]
    report['drawer_inward']['geometry_counts']=dict(drawer=len(dg),enclosure=len(sg))
    preserved_path=base/'preserved-geometry.json'
    if preserved_path.is_file():
        pre=json.loads(preserved_path.read_text(encoding='utf-8-sig'))
        report['preserved_geometry']=dict(report_sha256=sha(preserved_path),model_sha256=sha(pre['candidate']),scale_mode=pre['scale_mode'],protected_interface_geometry_count=pre['protected_interface_geometry_count'],standard_adapter_present=pre['standard_adapter_present'],differences=pre['differences'],checks=pre['checks'])
        if pre.get('authorized_capture_mount') is not None:
            report['preserved_geometry']['authorized_capture_mount']=pre['authorized_capture_mount']
            report['preserved_geometry']['arm_root_owners']=pre['arm_root_owners']
        if pre.get('authorized_operating_mount') is not None:
            report['preserved_geometry']['authorized_operating_mount']=pre['authorized_operating_mount']
            report['preserved_geometry']['arm_root_owners']=pre['arm_root_owners']
        report['checks']['preserved_geometry_contract_valid']=pre['scale_mode']=='housing_only_preserved_face' and pre['protected_interface_geometry_count']==899 and not pre['standard_adapter_present'] and not pre['differences'] and all(pre['checks'].values()) and sha(pre['candidate'])==sha(model_path)
    else:
        report['preserved_geometry']=None
        report['checks']['preserved_geometry_contract_valid']=False
    report['passed']=all(report['checks'].values())
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(output=str(output),passed=report['passed'],checks=gates,visual_intersections=len(visual_hits),reserved_intrusions=len(reserved),new_tag_intersections=len(new_tag_hits),black_frame_occluded=len(rays),max_interface_penetration=report['maximum_interface_contact_penetration_m'],drawer_gap=drawer_min-rear_inner)),flush=True)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scene-directory',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();report=run(a.scene_directory,a.output);raise SystemExit(0 if report['passed'] else 1)
