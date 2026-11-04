"""Audit used-marker image sizes at a proposed passive-channel overlap.

Static independently rendered pairs assess optical feasibility; they cannot
prove a simultaneous channel transfer or a 10-sample live readiness streak.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();results=[]
    for folder in a.runs:
        groups=defaultdict(list)
        for line in (folder/'observations.jsonl').read_text().splitlines():
            row=json.loads(line); m=row['metrics'];chosen=row['result']['methods']['joint_multicamera']
            sizes={}
            if chosen['status']=='ok':
                for e in ('left','right'):
                    detected=row['result']['detected_corners_px'][e]
                    edges=[np.min(np.linalg.norm(np.array(detected[str(i)])-
                         np.roll(np.array(detected[str(i)]),1,axis=0),axis=1)) for i in chosen['used_ids'][e]]
                    sizes[e]=float(max(edges))
            record=dict(accepted=chosen['status']=='ok',sides_px=sizes,
                        quality_at_24px=bool(sizes and min(sizes.values())>=24),
                        error_mm=m.get('position_error_mm'))
            groups[(m['gap_m'],m['angle_deg'])].append(record)
        for (depth,tilt),rows in groups.items():
            good=[r for r in rows if r['accepted']]
            results.append(dict(run=folder.name,depth_m=depth,tilt_deg=tilt,pairs=len(rows),
                accepted=len(good),quality_at_24px=sum(r['quality_at_24px'] for r in rows),
                minimum_largest_used_marker_edge_px=min(min(r['sides_px'].values()) for r in good) if good else None,
                maximum_position_error_mm=max(r['error_mm'] for r in good) if good else None))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(scope=__doc__,groups=results,live_transfer_validated=False),indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
