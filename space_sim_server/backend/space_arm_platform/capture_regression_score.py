"""Independent contact/vision/gyro qualification. No feedback to the controller."""
import json
from pathlib import Path
import numpy as np


def records(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        for line in stream:
            if line.strip():yield json.loads(line)


def phase_evidence(controls):
    expected=['open','extend','approach_visual','approach_encoder','close','tighten','hold_force']
    cursor=0;transitions=[];aborts=[]
    for row in controls:
        status=row['status']
        if not transitions or transitions[-1]['phase']!=status:transitions.append(dict(t=row['t'],phase=status))
        if 'abort' in status:aborts.append(row)
        if cursor<len(expected) and status=='predictive_'+expected[cursor]:cursor+=1
    return dict(complete=cursor==len(expected) and not aborts,transitions=transitions,aborts=aborts)


def score_hold(rows):
    stable=None;longest=0.;self_hit=None;first_contact=None;hold_forces=[];last=None;previous_t=None
    for row in rows:
        if row.get('type')!='step':continue
        t=float(row['t']);clamp=row['clamp'];last=row
        if previous_t is not None and (t<=previous_t or t-previous_t>.01):stable=None
        previous_t=t
        good=row['phase']=='hold_force' and clamp['jaw_support_count']>=4 and 4.5<=clamp['pad_force_n']<=5.5
        if good:
            if stable is None:stable=t
            longest=max(longest,t-stable)
            hold_forces.append(clamp['pad_force_n'])
        else:stable=None
        for contact in row['contacts']:
            roots=contact['roots'];arm=[name.startswith(('bottom_','jaw_','gripper','axial_','cam')) for name in contact['bodies']]
            if roots==['cubesat_bus','cubesat_bus'] and any(arm) and not all(arm) and contact['distance_m']<-.0001:
                if self_hit is None:self_hit=dict(t=t,contact=contact)
            if (first_contact is None and contact['efc_address']>=0
                    and set(roots)=={'cubesat_bus','capture_target'} and abs(contact['force_contact'][0])>1e-5):
                first_contact=dict(t=t,phase=row['phase'],contact=contact)
    return dict(continuous_hold_s=longest,self_contact=self_hit,first_target_contact=first_contact,
        hold_force_range_n=[min(hold_forces),max(hold_forces)] if hold_forces else None,last=last)


def rotation(q):
    w,x,y,z=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def angle(matrix):return float(np.degrees(np.arccos(np.clip((np.trace(matrix)-1)/2,-1,1))))


def target_motion(audit_path,reference_path,first_contact_t):
    reference=list(records(reference_path))
    rt=np.array([r['t'] for r in reference]);ref=np.array([r['target'] for r in reference])
    differences=[]
    for index,row in enumerate(records(audit_path)):
        if row.get('type')!='step' or index%8:continue
        t=row['t'];a=np.array(row['bodies']['capture_target'])
        if not rt[0]<=t<=rt[-1]:raise ValueError('free reference does not cover actual grasp')
        b=np.array([np.interp(t,rt,ref[:,i]) for i in range(13)])
        differences.append(dict(t=t,translation_mm=float(np.linalg.norm(a[:3]-b[:3])*1000),
            rotation_deg=angle(rotation(a[3:7])@rotation(b[3:7]).T)))
    pre=[r for r in differences if r['t']<first_contact_t-.05]
    if not pre:raise ValueError('missing precontact free-reference comparison')
    return dict(precontact_translation_mm=max(r['translation_mm'] for r in pre),
        precontact_rotation_deg=max(r['rotation_deg'] for r in pre),
        max_translation_mm=max(r['translation_mm'] for r in differences),
        max_rotation_deg=max(r['rotation_deg'] for r in differences),last=differences[-1])
