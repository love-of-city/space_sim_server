"""Independent full-range service drawer/latch versus service hull distance sweep."""
import argparse,json,hashlib,sys
from pathlib import Path
import numpy as np
import mujoco as mj
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from preview_scene_model import load_geometry_model

def verify(path,output):
    m=load_geometry_model(path);d=mj.MjData(m)
    lhs=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name in ('service_drawer_buckle','service_drawer_base') and m.geom_contype[g]|m.geom_conaffinity[g]]
    rhs=[g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name=='cubesat_bus' and m.geom_contype[g]|m.geom_conaffinity[g]]
    slide=m.joint('service_drawer_base_slide');hinge=m.joint('service_drawer_buckle_joint')
    hits=[];closest=.005;closest_pair=None;queries=0;poses=0
    for s in np.linspace(*m.jnt_range[slide.id],32):
        for q in np.linspace(*m.jnt_range[hinge.id],72):
            d.qpos[slide.qposadr[0]]=s;d.qpos[hinge.qposadr[0]]=q;mj.mj_kinematics(m,d);poses+=1
            R=d.geom_xmat.reshape(-1,3,3);c=d.geom_xpos+np.einsum('nij,nj->ni',R,m.geom_aabb[:,:3]);e=np.einsum('nij,nj->ni',np.abs(R),m.geom_aabb[:,3:]);lo,hi=c-e,c+e
            for a in lhs:
                nearby=[b for b in rhs if np.all(hi[a]+.005>=lo[b]) and np.all(hi[b]+.005>=lo[a])]
                for b in nearby:
                    dist=float(mj.mj_geomDistance(m,d,a,b,.005,None));queries+=1
                    if dist<closest:closest=dist;closest_pair=[m.geom(a).name,m.geom(b).name]
                    if dist < -1e-7:hits.append(dict(slide=float(s),hinge=float(q),geoms=[m.geom(a).name,m.geom(b).name],separation_m=dist))
    report=dict(schema='service-latch-slot-audit/1',model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),poses=poses,distance_queries=queries,minimum_separation_m=closest,closest_pair=closest_pair,penetration_count=len(hits),penetrations=hits[:100],passed=not hits,scope='Independent finite joint-range sweep, explicit geometry pairs including normally filtered pairs; no continuous collision or full task success claim')
    output.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report),flush=True)
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();verify(a.model,a.output)
