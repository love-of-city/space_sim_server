"""Read-only 400 Hz capture diagnostics, including every target descendant."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

class CaptureAudit:
    def __init__(self, model, stepper, *, output_path=None):
        self.stepper=stepper
        self.feedback={}
        self.filtered_pad_force=0.
        self.last_t=None
        self.geom_body={};self.body_root={}
        def walk(body, root):
            name=body.get('name'); self.body_root[name]=root
            for g in body.findall('geom'):
                if g.get('name'):self.geom_body[g.get('name')]=name
            for child in body.findall('body'):walk(child,root)
        for body in ET.parse(model).findall('./worldbody/body'):walk(body,body.get('name'))
        self.path=(Path(output_path) if output_path is not None else
                   Path(__file__).resolve().parents[1]/'run/capture_contact_audit.jsonl')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file=self.path.open('w',encoding='utf-8');self.count=0
        # Explicit sequential trials may reach grasp after the bounded body
        # dump ends at 300 s. Retain compact feedback at every physical step.
        self.force_file=(self.path.with_name(self.path.stem+'_force.jsonl').open('w',encoding='utf-8',buffering=1)
                         if output_path is not None else None)
        self.file.write(json.dumps(dict(type='schema', rate_hz=400, body_fields='position_local_m[3],quaternion_wxyz[4],omega_world[3],velocity_world[3],native_time_s',contact_force_frame='contact frame; normal first',native_pose_note='post-step kinematic cache; sampled before next step, no extra forward'))+'\n')
    def record(self,t,phase,q):
        bodies={name:self.stepper.body_detail(name).tolist() for name in ('cubesat_bus','capture_target','gripper')}
        contacts=[];self_hits=[]
        for row in self.stepper.contact_detail():
            names=[self.stepper._name(5,int(i)) for i in row[:2]]
            bn=[self.geom_body.get(n,'') for n in names]
            roots=[self.body_root.get(n,'') for n in bn]
            arm=[n.startswith(('bottom_','jaw_','gripper','axial_','cam')) for n in bn]
            if not ('capture_target' in roots or any(arm)):continue
            c=dict(geoms=names,bodies=bn,roots=roots,distance_m=row[2],position_local_m=row[3:6].tolist(),force_contact=row[6:12].tolist(),frame=row[12:21].tolist(),efc_address=int(row[21]))
            contacts.append(c)
            if any(arm) and not all(arm) and roots==['cubesat_bus','cubesat_bus'] and row[2]<-.0001:self_hits.append(c)
        quat=np.asarray(bodies['gripper'][3:7]);w,x,y,z=quat
        axis=np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
        force=0.;compression=0.;jaws={};peak_target_force=0.
        for c in contacts:
            if set(c['roots'])!={'capture_target','cubesat_bus'}:continue
            if c['efc_address']>=0:
                peak_target_force=max(peak_target_force,abs(float(c['force_contact'][0])))
            if not any(n.startswith('capture_x_piece_') for n in c['geoms']):continue
            ispad=any(n.startswith('collision_ring_convex_') for n in c['geoms'])
            if ispad:compression=max(compression,max(0.,-c['distance_m']))
            if c['efc_address']<0:continue
            axial=max(0.,c['force_contact'][0])*abs(np.dot(c['frame'][:3],axis))
            if ispad:force+=axial
            for b in c['bodies']:
                if b.startswith('jaw_'):jaws[b]=jaws.get(b,0.)+axial
        dt=.0025 if self.last_t is None else max(0.,t-self.last_t);self.last_t=t
        self.filtered_pad_force+=(force-self.filtered_pad_force)*(1-np.exp(-dt/.05))
        self.feedback=dict(pad_force_n=float(self.filtered_pad_force),pad_force_raw_n=float(force),pad_compression_m=float(compression),jaw_force_n=float(sum(jaws.values())),jaw_support_count=int(sum(v>.1 for v in jaws.values())),peak_target_contact_force_n=float(peak_target_force))
        if self.force_file is not None:
            self.force_file.write(json.dumps(dict(time_s=t,phase=phase,feedback=self.feedback,
                                                 self_contact_count=len(self_hits)))+'\n')
        if t<=300:
            self.file.write(json.dumps(dict(type='step',t=t,phase=phase,q=q.tolist(),bodies=bodies,contacts=contacts,clamp=self.feedback))+'\n')
        self.count+=1
        if self.count%200==0:self.file.flush()
        return self_hits
