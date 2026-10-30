"""Independent truth scoring for actual formal-route parking intervals."""
import json
from pathlib import Path

import numpy as np

from simulation.near_field.orbital_guidance import rotating_relative_state


def score_near(output, route):
    nodes = {}
    sequence = route.sequence
    names = ('Vbar50m','Optical17p4m','GNSS5p5m','PlatformStandoff')
    for name in names:
        done = None if sequence is None else sequence.vbar.gates[name].completed_at_s
        end = None if done is None else done+sequence.epoch
        reference = (sequence.vbar.targets[sequence.vbar.node_ids.index(name)].tolist()
                     if sequence is not None and sequence.vbar.targets is not None else None)
        nodes[name] = dict(end_s=end,reference_com_m=reference,count=0,first_s=None,last_s=None,
            max_position_m=0.,max_velocity_m_s=0.,max_tool_error_m=0.,continuous=True)
    with (Path(output)/'score.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line); t=row['time_s']
            for name,node in nodes.items():
                end=node['end_s']
                if end is None or t<end-5.-1e-7 or t>end+1e-7:
                    continue
                p,v=np.asarray(row['truth_com_position_n_m']),np.asarray(row['truth_com_velocity_n_m_s'])
                x=rotating_relative_state(np.r_[p[1],v[1]],np.r_[p[1]-p[0],v[1]-v[0]])
                reference=np.asarray(node['reference_com_m'])
                node['max_position_m']=max(node['max_position_m'],float(np.linalg.norm(x[:3]-reference)))
                node['max_velocity_m_s']=max(node['max_velocity_m_s'],float(np.linalg.norm(x[3:])))
                if name=='PlatformStandoff':
                    rs,rt=np.asarray(row['truth_rotation_n_from_body'])
                    cs,ct=np.asarray(row['truth_com_body_m'])
                    site=row['encoder_geometry']['cubesat_bus']['sites']['grip_frame']
                    pg,rg=np.asarray(site['position_body_m']),np.asarray(site['rotation_body_from_site'])
                    tool=rg.T@(rs.T@(p[1]-p[0]-rt@ct+rs@cs+rt@route.geometry.center)-pg)
                    node['max_tool_error_m']=max(node['max_tool_error_m'],float(np.linalg.norm(tool-[0.,0.,-.5])))
                if node['last_s'] is not None and not 0<t-node['last_s']<=.050001:
                    node['continuous']=False
                if node['first_s'] is None:node['first_s']=t
                node['last_s']=t;node['count']+=1
    for node in nodes.values():
        node['passed']=bool(node['count']>=101 and node['last_s']-node['first_s']>=5.-1e-7
            and node['continuous'] and node['max_position_m']<=.05 and node['max_velocity_m_s']<=.01 and node['max_tool_error_m']<=.05)
    streak_start=last=None; longest=0.; count=0
    force_file=Path(output)/'contacts_400hz_force.jsonl'
    if force_file.exists():
        with force_file.open(encoding='utf-8') as stream:
            for line in stream:
                row=json.loads(line);t=row['time_s'];f=row['feedback']
                valid=(row['phase']=='hold_force' and f.get('jaw_support_count',0)>=4
                    and 4.5<=f.get('jaw_force_n',0)<=5.5 and f.get('peak_target_contact_force_n',0)<=10.
                    and row.get('self_contact_count',0)==0)
                if not valid or last is not None and not 0<t-last<=.002501:
                    streak_start=None;count=0
                if valid:
                    if streak_start is None:streak_start=t
                    count+=1
                    if count>=8001:longest=max(longest,t-streak_start)
                last=t
    result=dict(passed=all(n['passed'] for n in nodes.values()),nodes=nodes,
        force_hold=dict(passed=longest>=20.-1e-7,longest_s=longest))
    (Path(output)/'parking-audit.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result
