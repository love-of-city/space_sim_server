from pathlib import Path
import sys,json,xml.etree.ElementTree as E
import numpy as np
import mujoco as mj
s=Path(__file__).resolve().parents[1];sys.path[:0]=[str(s),str(s/'tools')]
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.scene_library.manifest import digest
import argparse
parser=argparse.ArgumentParser(description="Explicit closure-panel clearance, including filtered contacts")
parser.add_argument("--case",type=Path,required=True)
case=parser.parse_args().case;model=case/'preheld-dual.xml';r=E.parse(model).getroot();scene=json.loads((case/'initial-scene.json').read_text());ini=scene['randomization']
m=load_geometry_model(model);d=mj.MjData(m)
for name,v in zip([*(f'joint{i}' for i in range(1,7)),'ee_link11_joint','ee_link12_joint'],ini['arm_joint_position_rad']):d.qpos[m.joint(name).qposadr[0]]=v
for (_,name,_),v in zip(INITIAL_JOINTS,ini['preheld_capture_joint_position']):d.qpos[m.joint(name).qposadr[0]]=v
adr=m.joint('capture_target_free').qposadr[0];d.qpos[adr:adr+3]=ini['target_position_m'];d.qpos[adr+3:adr+7]=ini['target_orientation_wxyz'];mj.mj_forward(m,d)
active=[g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
body=[m.body(int(m.geom_bodyid[g])).name for g in range(m.ngeom)];rot=d.geom_xmat.reshape(-1,3,3);centers=d.geom_xpos+np.einsum('nij,nj->ni',rot,m.geom_aabb[:,:3]);ext=np.einsum('nij,nj->ni',abs(rot),m.geom_aabb[:,3:]);lo,hi=centers-ext,centers+ext
closure=[g for g in active if m.geom(g).name.startswith('hull_closure_')];hits=[];unsupported=[];queries=0;minimum=.005
for g in closure:
 for h in active:
  if body[h]=='capture_target':continue
  if np.any(hi[g]+.005<lo[h]) or np.any(hi[h]+.005<lo[g]):continue
  if int(m.geom_type[h])==int(mj.mjtGeom.mjGEOM_SDF):unsupported.append([m.geom(g).name,m.geom(h).name]);continue
  dist=float(mj.mj_geomDistance(m,d,g,h,.005,None));queries+=1;minimum=min(minimum,dist)
  if dist< -1e-6:hits.append(dict(geoms=[m.geom(g).name,m.geom(h).name],bodies=[body[g],body[h]],distance_m=dist))
result=dict(schema='closure-hardware-clearance/1',model_sha256=digest(model),closure_collision_geoms=len(closure),distance_queries=queries,minimum_distance_capped_5mm=minimum,forbidden_penetrations=hits,unsupported_near_pairs=unsupported,passed=not hits and not unsupported)
(case/'closure-clearance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
raise SystemExit(0 if result["passed"] else 1)
