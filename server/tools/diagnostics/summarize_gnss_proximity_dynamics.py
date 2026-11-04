"""Independently audit GNSS dynamics traces against their same-epoch truth."""
import argparse
import json
from pathlib import Path

import numpy as np


def stats(rows, key):
    values=np.asarray([row[key] for row in rows if row.get(key) is not None])
    return dict(samples=len(values),rmse_mm=float(np.sqrt(np.mean(values**2))),
                p95_mm=float(np.percentile(values,95)),max_mm=float(values.max())) if len(values) else None


def audit(path):
    summary=json.loads((path/'summary.json').read_text())
    rows=[json.loads(line) for line in (path/'trajectory.jsonl').read_text().splitlines()]
    times=np.array([row['time_s'] for row in rows])
    if not np.allclose(np.diff(times),.05,atol=1e-10,rtol=0):
        raise ValueError(f'{path}: broken 20 Hz trace')
    for row in rows:
        if 'truth_relative_origin_eci_m' in row:
            error=(np.array(row['estimated_relative_origin_eci_m'])-row['truth_relative_origin_eci_m'])*1000
            if not np.allclose(error,row['filtered_position_error_xyz_mm'],atol=1e-9,rtol=0):
                raise ValueError(f'{path}: signed error mismatch')
            if not np.isclose(np.linalg.norm(error),row['filtered_position_error_mm'],atol=1e-9,rtol=0):
                raise ValueError(f'{path}: norm mismatch')
    warm=[r for r in rows if r['time_s']>=20]
    terminal=[r for r in rows if r['time_s']>=times[-1]-20]
    checked=stats(warm,'filtered_position_error_mm')
    if not np.isclose(checked['rmse_mm'],summary['after20s']['filtered_position_error_mm']['rmse'],atol=1e-9):
        raise ValueError(f'{path}: published RMSE mismatch')
    boundary=next((r for r in rows if r['policy']['subsegment']=='OpticalBoundaryHold'),None)
    maximum=max(warm,key=lambda r:r['filtered_position_error_mm'])
    terminal_depth=np.array([r['true_optical_depth_m'] for r in terminal])
    physical_valid=summary.get('physical_run_valid',summary.get('passed',False))
    result=dict(run=path.name,mode=summary['mode'],seed=summary['seed'],duration_s=times[-1],
        initial_optical_depth_m=summary['initial_optical_depth_m'],
        min_true_optical_depth_m=min(r['true_optical_depth_m'] for r in rows),
        min_true_com_distance_m=min(r['true_com_distance_m'] for r in rows),
        all_position=stats(rows,'filtered_position_error_mm'),after20s_position=checked,
        after20s_raw_gnss=stats(warm,'raw_position_error_mm'),
        after20s_true_tracking=stats(warm,'true_tracking_error_mm'),
        after20s_estimated_tracking=stats(warm,'estimated_tracking_error_mm'),
        terminal20s_position=stats(terminal,'filtered_position_error_mm'),
        terminal20s_true_optical_depth_m=dict(min=float(terminal_depth.min()),mean=float(terminal_depth.mean()),max=float(terminal_depth.max())),
        terminal_true_com_distance_m=rows[-1]['true_com_distance_m'],
        first_boundary=None if boundary is None else {k:boundary[k] for k in ('time_s','true_optical_depth_m','estimated_optical_depth_m')},
        terminal_failure=rows[-1]['policy']['failure_reason'],
        physical_run_valid=physical_valid,
        accuracy_3mm=dict(all_frames=bool(all(r['filtered_position_error_mm']<3 for r in rows)),
            after20s_all_frames=bool(all(r['filtered_position_error_mm']<3 for r in warm)),
            after20s_fraction=float(np.mean([r['filtered_position_error_mm']<3 for r in warm]))),
        after20s_worst_example=maximum,
        raw_position_samples=sum(r['raw_position_error_mm'] is not None for r in rows),
        signed_vector_audit_samples=sum('truth_relative_origin_eci_m' in r for r in rows),
        external_state_writes=summary['stepper']['external_state_writes'],vision_updates=summary['vision_updates'])
    result['distance_bands']={}
    for low,high in ((15,17.5),(13,15),(12,13),(10,12),(8,10),(6,8),(5,6),(4,5),(3.2,4)):
        band=[r for r in warm if low<=r['true_optical_depth_m']<high]
        if band:
            result['distance_bands'][f'{low}-{high}m']=stats(band,'filtered_position_error_mm')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    results=[audit(p) for p in args.runs]
    args.output.write_text(json.dumps(dict(results=results),indent=2),encoding='utf-8')
    for result in results:
        print(json.dumps({k:result[k] for k in ('run','min_true_optical_depth_m','all_position','after20s_position','physical_run_valid','accuracy_3mm','signed_vector_audit_samples')},indent=2))
