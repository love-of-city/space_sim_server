"""Recheck a completed 7 km -> near23 grasp using both stages' raw evidence.

Does not launch a flight, write physical state or infer HTTP/deployment success.
An independent 110 m initialization cannot pass this audit.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from simulation.capture26_handoff import load
from simulation.capture26_mission import NEAR23_VISION_PATH
from simulation.capture26_near23_score import score_near
from simulation.near_field.orbital_guidance import rotating_relative_state
from simulation.orbital_rendezvous.reference import FlightProfile
from tools.check_capture26_continuity import check

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text('utf-8-sig'))


def identity_checks(far, near, received):
    checks = {key: far.get(key) == near.get(key) for key in (
        'mission_id', 'mission_options', 'mission_configuration', 'calibration_id',
        'source_sha256', 'ephemeris_epoch_utc')}
    checks.update(accepted_far=far.get('accepted') is True and far.get('phase') == 'FarReady',
        accepted_near=near.get('accepted') is True and near.get('phase') == 'GraspHold',
        actual_sent_checkpoint=far == received,
        absolute_clock=far.get('time_ns', -1) < near.get('time_ns', -1))
    return checks


def score_far(folder, handoff):
    profile = FlightProfile(scenario=handoff['mission_options'].get('scenario', 'in-orbit'),
        handoff_distance_m=handoff['mission_options']['handoff_distance_m'])
    end = handoff['time_ns']*1e-9
    start = end-5.
    first = previous = None
    count = 0
    continuous = True
    max_position = max_velocity = 0.
    initial = None
    with (Path(folder)/'score.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if initial is None:
                initial = dict(time_s=row['time_s'], distance_m=float(np.linalg.norm(row['truth_relative_position_m'])))
            t = row['time_s']
            if not start-1e-7 <= t <= end+1e-7:
                continue
            pt, vt, _ = profile.target(t)
            ps, vs, _ = profile.chaser(t)
            p, v = (np.asarray(row[key]) for key in ('truth_com_position_n_m', 'truth_com_velocity_n_m_s'))
            max_position = max(max_position, float(np.linalg.norm(p[0]-p[1]-(ps-pt))))
            max_velocity = max(max_velocity, float(np.linalg.norm(v[0]-v[1]-(vs-vt))))
            if previous is not None and not 0 < t-previous <= .050001:
                continuous = False
            if first is None:
                first = t
            previous = t
            count += 1
    result = dict(first_s=first, last_s=previous, samples=count, continuous=continuous,
        max_position_m=max_position, max_velocity_m_s=max_velocity, initial=initial)
    result['passed'] = bool(count >= 101 and previous-first >= 5.-1e-7 and continuous
        and max_position <= 2. and max_velocity <= .01
        and start >= profile.start_approach_s+profile.approach_duration_s-1e-7
        and initial and initial['time_s'] == 0 and initial['distance_m'] >= 6000.)
    return result


def recorded_route(near, vision):
    """Rebuild scoring metadata, without running a guidance/control algorithm."""
    snapshot = near['control']['approach']
    nodes = snapshot['telemetry']['vbar_nodes']
    names = tuple(nodes)
    return SimpleNamespace(sequence=SimpleNamespace(epoch=snapshot['epoch_ns']*1e-9,
        vbar=SimpleNamespace(node_ids=names,
            targets=[np.asarray(nodes[name]['reference_com_m']) for name in names],
            gates={name: SimpleNamespace(completed_at_s=nodes[name]['completed_at_s']) for name in names})),
        geometry=SimpleNamespace(center=np.asarray(vision['marker_rig']['grasp_center_target_m'])))


def score_activation(folder, near):
    activation = near['control']['stereo_activation']
    stamp = activation['time_ns']
    route = near['control']['approach']
    dwell = route['telemetry']['vbar_nodes']['GNSS5p5m']['completed_at_s']
    expected = route['epoch_ns']+round(dwell*1e9)
    events = read(Path(folder)/'delivery.json')
    starts = [e for e in events if e['kind']=='stereo_activation']
    frames = [e for e in events if e['kind']=='delivery']
    grid = [e['exposure_ns'] for e in frames]
    # The runner finishes computing pending images before saving handoff.
    # Future delivery epochs remain staged; they are not applied early to nav.
    last_requested_exposure = near['time_ns']//100_000_000*100_000_000
    checks = dict(qualified_dwell_activation=type(stamp) is int and stamp==expected
        and activation['policy']=='after_qualified_GNSS5p5m_dwell' and activation['capture_rate_hz']==10,
        one_recorded_activation=len(starts)==1 and starts[0]['time_ns']==stamp
            and starts[0]['policy']==activation['policy'] and starts[0]['rate_hz']==10,
        original_grid=bool(grid) and all(type(t) is int and t%100_000_000==0 for t in grid)
            and all(b-a==100_000_000 for a,b in zip(grid,grid[1:])),
        first_real_exposure=bool(grid) and type(stamp) is int and stamp<=grid[0]<=stamp+100_000_000,
        complete_delivery_interval=bool(grid) and grid[-1]==last_requested_exposure,
        all_requested_frames_processed=bool(frames) and all(e['result_available'] is True and e['process_healthy'] is True
            and e['due_ns']==e['exposure_ns']+200_000_000 for e in frames),
        no_delivery_faults=all(e['kind'] in ('stereo_activation','delivery') for e in events))
    return dict(passed=all(checks.values()),checks=checks,activation_ns=stamp,
        processed_pairs=len(frames),first_exposure_ns=grid[0] if grid else None,
        last_exposure_ns=grid[-1] if grid else None)


def score_alignment_recovery(folder, near):
    route = near['control']['approach']
    epoch = route['epoch_ns']*1e-9
    events = route['telemetry']['target_realignment']['events']
    intervals = []
    pending = None
    valid_events = True
    for event in events:
        if event['event'] == 'brake_for_target_realign' and pending is None:
            pending = event
        elif event['event'] == 'resume_after_target_realign' and pending is not None:
            end = epoch+event['time_s']
            dwell = event['dwell']
            valid_events &= (event['node']==pending['node'] and dwell['samples']>=101 and dwell['dwell_s']>=5.-1e-7
                and pending['time_s'] < event['time_s'] <= pending['time_s']+180.+1e-7)
            intervals.append(dict(start_s=epoch+pending['time_s'],end_s=end,reference_com_m=pending['stop_com_m'],
                samples=0,first_s=None,last_s=None,continuous=True,qualified=True,max_position_m=0.,max_velocity_m_s=0.))
            pending = None
        else:
            valid_events = False
    valid_events &= pending is None
    observed_starts = []
    previous_phase = None
    with (Path(folder)/'score.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            t, phase = row['time_s'], row['guidance']['phase']
            if phase == 'TargetRealignment' and previous_phase != phase:
                observed_starts.append(t)
            previous_phase = phase
            for interval in intervals:
                if not interval['end_s']-5.-1e-7 <= t <= interval['end_s']+1e-7:
                    continue
                p, v = (np.asarray(row[key]) for key in ('truth_com_position_n_m','truth_com_velocity_n_m_s'))
                state = rotating_relative_state(np.r_[p[1],v[1]],np.r_[p[1]-p[0],v[1]-v[0]])
                interval['max_position_m'] = max(interval['max_position_m'],float(np.linalg.norm(state[:3]-interval['reference_com_m'])))
                interval['max_velocity_m_s'] = max(interval['max_velocity_m_s'],float(np.linalg.norm(state[3:])))
                interval['qualified'] &= row['guidance']['vbar']['prealigned'] is True
                if interval['last_s'] is not None and not 0 < t-interval['last_s'] <= .050001:
                    interval['continuous'] = False
                if interval['first_s'] is None:
                    interval['first_s'] = t
                interval['last_s'] = t
                interval['samples'] += 1
    observed = (len(observed_starts)==len(intervals)
        and all(abs(t-i['start_s'])<1e-7 for t,i in zip(observed_starts,intervals)))
    for interval in intervals:
        interval['passed'] = bool(interval['samples']>=101 and interval['last_s']-interval['first_s']>=5.-1e-7
            and interval['continuous'] and interval['qualified'] and interval['max_position_m']<=.05
            and interval['max_velocity_m_s']<=.01)
    return dict(passed=bool(valid_events and observed and all(i['passed'] for i in intervals)),
        valid_event_pairs=bool(valid_events),observed_phases_match=observed,intervals=intervals)


def audit(far_folder, near_folder):
    far_folder, near_folder = Path(far_folder), Path(near_folder)
    far, near = (load(folder/'handoff.json', ROOT) for folder in (far_folder, near_folder))
    if far['mission_options'].get('capture26_profile') != 'near23_vbar_v1':
        raise ValueError('this audit requires the explicit near23 route')
    far_report, near_report = (read(folder/'mission_report.json') for folder in (far_folder, near_folder))
    if far_report.get('scenario') != 'in-orbit':
        raise ValueError('this audit covers the 7 km in-orbit start, not orbit-raise')
    identities = identity_checks(far, near, read(near_folder/'input_handoff.json'))
    continuity = check(far_folder/'handoff.json', near_folder)
    far_score = score_far(far_folder, far)
    parking = score_near(near_folder, recorded_route(near, read(ROOT/NEAR23_VISION_PATH)))
    activation = score_activation(near_folder,near)
    recovery = score_alignment_recovery(near_folder,near)
    checks = dict(same_continuous_task=all(identities.values()),
        both_stage_reports_passed=all(r.get('basic_nominal_passed') is True for r in (far_report, near_report)),
        native_first_epoch_continuity=continuity['continuity_passed'],
        real_7km_and_110m_five_second_dwell=far_score['passed'],
        all_near_dwells=parking['passed'], physical_four_jaw_hold_20s=parking['force_hold']['passed'],
        original_sampling_after_phase_activation=activation['passed'],
        real_stationary_alignment_recovery=recovery['passed'],
        original_work_lights_recorded=(near_folder/'native_work_lights_ready.json').is_file())
    return dict(schema='space-sim-capture26-continuous-audit/1', accepted=all(checks.values()),
        scope='recorded 7 km physical flight through near23 grasp; HTTP launch and shared deployment are separate',
        mission_id=far['mission_id'], checks=checks, identity_checks=identities,
        far=far_score, near=parking, continuity=continuity,stereo_activation=activation,alignment_recovery=recovery,
        source=dict(far=str(far_folder.resolve()), near=str(near_folder.resolve())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('far', type=Path)
    parser.add_argument('near', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = audit(args.far, args.near)
    except (OSError, ValueError, KeyError, TypeError) as error:
        result = dict(schema='space-sim-capture26-continuous-audit/1', accepted=False, error=str(error))
    destination = args.output or args.near/'continuous-audit.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, indent=2)+'\n', 'utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
