"""Static XML forward kinematics for visual assets; never loads a physics plugin."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT.parent/'space_sim_UE_Adapter/Adapters')]
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix as qm, matrix_to_quaternion_wxyz as mq
from simulation.capture_arm_target import INITIAL_JOINTS
from bsk_render_adapter.mjcf_assets import parse_mjcf_geometry_metadata, load_asset_catalog, _quat_multiply
from bsk_render_adapter.bridge import _mj_geometry, _mj_mesh_geometry
from bsk_render_adapter.descriptors import CameraVisual,SceneSettings
from bsk_render_adapter.recording import BskRecordingWriter


def numbers(node,key,default):return np.fromstring(node.get(key,default),sep=' ')


def ordered_geometries(root, metadata):
    """Pair XML attributes with the body-grouped MuJoCo metadata order."""
    if root.find('.//frame') is not None or root.find('.//include') is not None:
        raise ValueError('Static inspector requires expanded bodies without MJCF frames')
    entries=[(body,geom) for body in root.findall('./worldbody//body') for geom in body.findall('geom')]
    if len(entries)!=len(metadata):
        raise ValueError('Geometry metadata count mismatch')
    for (body,geom),meta in zip(entries,metadata):
        if body.get('name')!=meta.body_name or geom.get('mesh','')!=meta.mesh_name:
            raise ValueError('Geometry metadata identity mismatch: '+geom.get('name','unnamed'))
    return [geom for _,geom in entries]


def kinematics(root,initial):
    values=dict(zip([*(f'joint{i}' for i in range(1,7)),'ee_link11_joint','ee_link12_joint'],initial['arm_joint_position_rad']))
    values.update({n:v for (_,n,_),v in zip(INITIAL_JOINTS,initial.get('preheld_capture_joint_position',[]))})
    poses={}
    def visit(body,pp,pr):
        p=numbers(body,'pos','0 0 0');r=qm(numbers(body,'quat','1 0 0 0'))
        if any(k in body.attrib for k in ('euler','axisangle','xyaxes')):raise ValueError('Unsupported body frame encoding')
        if body.get('name')=='capture_target':
            p=np.asarray(initial['target_position_m']);r=qm(initial['target_orientation_wxyz'])
        else:
            for joint in body.findall('joint'):
                kind=joint.get('type','hinge')
                if kind=='free':continue
                ref=float(joint.get('ref','0'));value=values.get(joint.get('name'),ref)-ref
                axis=numbers(joint,'axis','0 0 1');axis/=np.linalg.norm(axis)
                if kind=='slide':p=p+r@axis*value
                elif kind=='hinge':
                    dr=Rotation.from_rotvec(axis*value).as_matrix();anchor=numbers(joint,'pos','0 0 0')
                    p=p+r@(anchor-dr@anchor);r=r@dr
                else:raise ValueError('Unsupported joint in static inspector: '+kind)
        p,r=pp+pr@p,pr@r;poses[body.get('name')]=(p,r)
        for child in body.findall('body'):visit(child,p,r)
    for body in root.findall('./worldbody/body'):visit(body,np.zeros(3),np.eye(3))
    return poses


def export(model,scene,catalogs,project,output,label,*,view_direction=None,distance=3.5,insets=True):
    if output.exists():raise FileExistsError(output)
    root=ET.parse(model).getroot();initial=json.loads(scene.read_text(encoding='utf-8'))['randomization']
    poses=kinematics(root,initial);metadata=parse_mjcf_geometry_metadata(model,compile_mesh_poses=False)
    by_hash={}
    for path in catalogs:
        for asset in json.loads(path.read_text(encoding='utf-8'))['assets'].values():
            if (project/'Content'/(asset['asset_path'].split('.')[0][6:]+'.uasset')).is_file():by_hash[asset['source_sha256']]=asset
    rebound={}
    for meta in metadata:
        if meta.visible and meta.source_path:
            h=hashlib.sha256(Path(meta.source_path).read_bytes()).hexdigest()
            if h not in by_hash:raise ValueError('Missing UE mesh: '+meta.source_path)
            rebound[meta.source_path]=by_hash[h]
    cp=output.with_suffix('.catalog.json');cp.write_text(json.dumps(dict(schema='bsk-render-asset-catalog/1',source_mjcf=str(model),assets=rebound)),encoding='utf-8')
    catalog=load_asset_catalog(cp)
    objects={n:dict(object_id=n,display_name=n,parent_id='',transform_space='world',semantic_label='static_asset_inspection',asset_path='',geometries=[]) for n in poses}
    geoms=ordered_geometries(root,metadata)
    types={'sphere':2,'capsule':3,'ellipsoid':4,'cylinder':5,'box':6}
    for i,(node,meta) in enumerate(zip(geoms,metadata)):
        if not meta.visible or meta.role=='collision':continue
        size=numbers(node,'size','0 0 0');size=np.pad(size,(0,max(0,3-len(size))))
        geom=SimpleNamespace(size=size,type=types.get(node.get('type','sphere'),-1),pos=meta.position_body_m,quat=meta.orientation_body_from_geometry_wxyz,rgba=numbers(node,'rgba','0.64 0.68 0.73 1'))
        visual=_mj_mesh_geometry(geom,'geom/'+str(i),meta,catalog,{}) if meta.mesh_name else _mj_geometry(geom,'geom/'+str(i))
        if visual:objects[meta.body_name]['geometries'].append(visual.to_payload())
    parents={c:p for p in root.iter() for c in p};cameras=[]
    for cam in root.iter('camera'):
        owner=parents[cam];q=numbers(cam,'quat','1 0 0 0')
        if cam.get('xyaxes'):
            xy=numbers(cam,'xyaxes','');x=xy[:3]/np.linalg.norm(xy[:3]);y=xy[3:]-x*np.dot(x,xy[3:]);y/=np.linalg.norm(y);q=mq(np.column_stack((x,y,np.cross(x,y))))
        res=[int(v) for v in cam.get('resolution','1920 1080').split()];fy=math.radians(float(cam.get('fovy','45')))
        cameras.append(CameraVisual(camera_id='camera/'+cam.get('name'),display_name=cam.get('name'),parent_id=owner.get('name'),position_body_m=numbers(cam,'pos','0 0 0').tolist(),orientation_body_from_camera_wxyz=_quat_multiply(q,(.5,-.5,.5,.5)),field_of_view_rad=2*math.atan(math.tan(fy/2)*res[0]/res[1]),resolution=tuple(res),semantic_label='original_mjcf_camera',picture_in_picture=True,picture_in_picture_slot=len(cameras)+1,capture_rate_hz=2,capture_products=('rgb',)).to_payload())
    h=hashlib.sha256(model.read_bytes()).hexdigest();common=dict(protocol='bsk-render/2',session_id='static-xml-'+h[:12]);origin=poses['cubesat_bus'][0]
    # Rotate the inspection coordinate basis uniformly, equivalent to orbiting
    # the spectator camera. Never edit model poses or mounted camera extrinsics.
    basis=np.eye(3)
    if view_direction is not None:
        direction=np.asarray(view_direction,dtype=float)
        if direction.shape!=(3,) or not np.isfinite(direction).all() or np.linalg.norm(direction)<1e-9:
            raise ValueError('Invalid inspection view direction')
        direction/=np.linalg.norm(direction)
        default=np.array([-.7697511313,-.5389855447,-.3420201433])
        basis=Rotation.align_vectors([default/np.linalg.norm(default)],[direction])[0].as_matrix()
    if not math.isfinite(distance) or distance<=0:raise ValueError('Invalid inspection distance')
    if not insets:
        for camera in cameras:camera['picture_in_picture']=False
    settings=SceneSettings(origin_object_id='',default_camera_target='capture_target',default_camera_distance_m=distance,orbit_lines=False,trajectory_history=False,fill_light_intensity_lux=100000.,headlight_enabled=True,headlight_diffuse_rgb=(10000.,)*3).to_payload()
    with BskRecordingWriter(output) as writer:
        writer.write(dict(common,type='hello',capabilities=['scene_manifest','recording','events'],required_capabilities=['scene_manifest'],coordinates=dict(length_unit='m',handedness='right',position_frame='L',quaternion_order='wxyz',rotation_semantics='active_parent_from_child')))
        writer.write(dict(common,type='scene_manifest',revision='1',objects=list(objects.values()),celestial_bodies=[],visuals=[],cameras=cameras,settings=settings))
        writer.write(dict(common,type='event',sequence='1',event_kind='inspection_notice',payload=dict(severity='info',message='STATIC XML ASSET INSPECTION | '+label)))
        for tick in range(301):writer.write(dict(common,type='frame',manifest_revision='1',frame_id=str(tick),sim_time_ns=str(tick*100000000),wall_time_ns='0',origin_N_m=origin.tolist(),c_LN=np.eye(3).reshape(-1).tolist(),celestial_bodies=[],visual_states=[],objects=[dict(object_id=n,position_m=(basis@(p-origin)).tolist(),orientation_wxyz=mq(basis@r).tolist()) for n,(p,r) in poses.items()]))
    report=dict(mode='static_xml_forward_kinematics',dynamics_executed=False,model_sha256=h,cameras=len(cameras),bodies=len(poses),source_meshes=len(rebound),full_task_accepted=False,inspection_basis=basis.tolist(),inspection_distance_m=distance,installed_camera_extrinsics_changed=False)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8');return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('model','scene','project','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--catalog',type=Path,action='append',required=True);p.add_argument('--label',default='Visual asset')
    p.add_argument('--view-direction',type=float,nargs=3);p.add_argument('--distance',type=float,default=3.5);p.add_argument('--no-insets',action='store_true')
    a=p.parse_args();print(json.dumps(export(a.model,a.scene,a.catalog,a.project,a.output,a.label,view_direction=a.view_direction,distance=a.distance,insets=not a.no_insets)))
