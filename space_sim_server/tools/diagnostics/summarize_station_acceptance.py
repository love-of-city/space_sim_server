"""Score stationary COM arrival windows, without declaring whole-node success.

The fixed station error is distinct from the moving-reference tracking error
and the estimator-vs-truth navigation error. Only the position component of
independent short-leg experiments is assessed here.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def audit(node_id, path, tolerance_mm):
    summary=json.loads((path/'summary.json').read_text())
    handoff=summary['pointing_handoff']
    events=handoff['events']
    start=next(e['time_s'] for e in events if e['phase']=='StationSettle')
    end=next(e['time_s'] for e in events if e['phase']=='TargetPointing')
    rows=[json.loads(line) for line in (path/'trajectory.jsonl').read_text().splitlines()]
    all_times=np.array([r['time_s'] for r in rows])
    assert np.allclose(np.diff(all_times),.05,rtol=0,atol=1e-10)
    selected=[r for r in rows if start-1e-9<=r['time_s']<=end+1e-9]
    station=np.array(handoff['station_com_reference_m'])
    errors=np.array([(np.array(r['true_com_position_m'])-station)*1000 for r in selected])
    norms=np.linalg.norm(errors,axis=1)
    assert len(selected)>=101 and selected[-1]['time_s']-selected[0]['time_s']>=5-1e-9
    assert all(np.allclose(r['reference_com_m'],station,rtol=0,atol=1e-12) for r in selected)
    assert summary['stepper']['external_state_writes']==0
    assert not any(r['target_contact_pairs'] for r in rows)
    for row in selected:
        expected=(np.array(row['estimated_relative_origin_eci_m'])-row['truth_relative_origin_eci_m'])*1000
        assert abs(np.linalg.norm(expected)-row['filtered_position_error_mm'])<1e-9
    return dict(node_id=node_id,run=str(path.resolve()),
        stationary_window_s=[selected[0]['time_s'],selected[-1]['time_s']],samples=len(selected),
        fixed_station_com_m=station.tolist(),tolerance_mm=tolerance_mm,
        true_fixed_station_error_mm=dict(rmse=float(np.sqrt(np.mean(norms**2))),
            maximum=float(norms.max()),maximum_abs_xyz=np.abs(errors).max(axis=0).tolist()),
        navigation_error_mm=dict(rmse=float(np.sqrt(np.mean([r['filtered_position_error_mm']**2 for r in selected]))),
            maximum=max(r['filtered_position_error_mm'] for r in selected)),
        maximum_estimated_rotating_speed_m_s=max(r['estimated_rotating_speed_m_s'] for r in selected),
        maximum_navigation_position_sigma_m=max(r['nav_position_sigma_m'] for r in selected),
        fixed_station_position_component_pass=bool(np.all(norms<tolerance_mm)),
        actual_image_updates=summary['vision_updates'],
        full_node_validated=False,continuous_four_node_mission_validated=False,
        scope='independently initialized short GNSS leg; position component only; original mission common gates not certified')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station',nargs=3,action='append',required=True,
                        metavar=('NODE','RUN_DIRECTORY','TOLERANCE_MM'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    results=[audit(node,Path(folder),float(tolerance)) for node,folder,tolerance in args.station]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(results=results),indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))
