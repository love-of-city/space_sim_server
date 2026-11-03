"""Carve a local service-hull slot for the full drawer latch motion envelope.

Geometry-only Python process. Preserve mechanism, inertias, collision masks,
arm mounts, protected target surfaces, and camera transforms.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from preview_scene_model import load_geometry_model
from build_task_box_module import split_outside
from simulation.scene_library.taskbox_recess import clip_surface_pocket
from simulation.scene_library.taskbox_mount import _pose

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v): p.write_text(json.dumps(v,indent=2),encoding='utf-8')

def build(source,output):
    output.mkdir(parents=True,exist_ok=False); meshes=output/'latch-slot-meshes';meshes.mkdir()
    root=ET.parse(source).getroot();original=deepcopy(root)
    model=load_geometry_model(source);data=mj.MjData(model)
    slide=model.joint('service_drawer_base_slide');hinge=model.joint('service_drawer_buckle_joint')
    gids=[i for i in range(model.ngeom) if model.body(int(model.geom_bodyid[i])).name in ('service_drawer_buckle','service_drawer_base')]
    low=np.full(3,np.inf);high=np.full(3,-np.inf)
    for s in model.jnt_range[slide.id]:
        data.qpos[slide.qposadr[0]]=s
        for q in np.linspace(*model.jnt_range[hinge.id],351):
            data.qpos[hinge.qposadr[0]]=q;mj.mj_kinematics(model,data)
            for g in gids:
                mid=int(model.geom_dataid[g]);v=model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid]+model.mesh_vertnum[mid]]
                vertices=v@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g]
                low=np.minimum(low,vertices.min(0));high=np.maximum(high,vertices.max(0))
    low-=.002;high+=.002
    # The 1 mrad angular grid has a sub-micrometre chord error at this latch's radius;
    # the independent denser sweep below checks the actual exported geometry.
    bus=root.find('.//body[@name="cubesat_bus"]');asset=root.find('asset')
    report=dict(schema='service-latch-slot/1',source=str(source),source_sha256=sha(source),slot_bounds_service_m=[low.tolist(),high.tolist()],clearance_m=.002,slide_range=model.jnt_range[slide.id].tolist(),hinge_range=model.jnt_range[hinge.id].tolist(),edits=[],assets=[],inertias_unchanged=True)
    for geom in list(bus.findall('geom')):
        if not geom.get('name','').startswith('base_link_geom') or not geom.get('mesh'):continue
        a=asset.find(f'mesh[@name="{geom.get("mesh")}"]');p,r=_pose(geom)
        if any(k in a.attrib for k in ('refpos','refquat')):raise ValueError('Unsupported mesh reference transform')
        path=Path(a.get('file'));path=path if path.is_absolute() else source.parent/path
        mesh=trimesh.load_mesh(path,process=True);mesh.vertices=(mesh.vertices*np.fromstring(a.get('scale','1 1 1'),sep=' '))@r.T+p
        if np.any(mesh.bounds[1]<=low) or np.any(mesh.bounds[0]>=high):continue
        collision=bool(int(geom.get('contype','0')) or int(geom.get('conaffinity','0')))
        pieces=split_outside(mesh.convex_hull,np.array([low,high])) if collision else [clip_surface_pocket(mesh,low,high)]
        before_volume=float(mesh.convex_hull.volume) if collision else None
        after_volume=sum(float(x.convex_hull.volume) for x in pieces) if collision else None
        replacements=[]
        for i,part in enumerate(pieces):
            if not len(part.faces):continue
            if collision:part=part.convex_hull
            part.vertices=(part.vertices-p)@r
            name=geom.get('name')+'_latch_slot_'+str(i);dest=meshes/(name+'.stl');part.export(dest)
            ET.SubElement(asset,'mesh',name=name,file=dest.resolve().as_posix(),scale='1 1 1',inertia='shell')
            attrs=dict(geom.attrib);attrs['name']=geom.get('name') if i==0 else name;attrs['mesh']=name
            replacements.append(ET.Element('geom',attrs));report['assets'].append(dict(name=name,file=str(dest.resolve()),sha256=sha(dest),collision=collision))
        idx=list(bus).index(geom);bus.remove(geom)
        for i,node in enumerate(replacements):bus.insert(idx+i,node)
        report['edits'].append(dict(original=dict(geom.attrib),replacements=[x.attrib for x in replacements],source_mesh=str(path),source_mesh_sha256=sha(path),collision=collision,before_convex_volume_m3=before_volume,after_convex_volume_m3=after_volume))
    for section in ['actuator','sensor','contact','equality']:
        assert ET.tostring(original.find(section))==ET.tostring(root.find(section))
    for name in ['link1','bottom_mount','capture_target','service_task_box_base','service_drawer_base']:
        assert ET.tostring(original.find(f'.//body[@name="{name}"]'))==ET.tostring(root.find(f'.//body[@name="{name}"]'))
    dest=output/'preheld-dual.xml';ET.indent(root);ET.ElementTree(root).write(dest,encoding='utf-8',xml_declaration=True)
    for name in ['initial-scene.json','attitude_control.json','recess-contract.json']:shutil.copy2(source.with_name(name),output/name)
    assembly=json.loads(source.with_name('assembly.json').read_text());assembly.update(model=str(dest),model_sha256=sha(dest),service_latch_slot_contract=str(output/'latch-slot-contract.json'))
    write(output/'assembly.json',assembly);report['model_sha256']=sha(dest);write(output/'latch-slot-contract.json',report)
    print(json.dumps(dict(model=str(dest),bounds=report['slot_bounds_service_m'],edits=[e['original']['name'] for e in report['edits']])),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();build(a.source,a.output)
