"""Recompute the marker report's mono/stereo comparison from saved frame scores.

No detection, rendering, pose fitting, or source experiment files are changed.
Each comparison uses exactly the same accepted frame IDs in all three methods.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


METHODS = ('mono_left', 'mono_right', 'joint_multicamera')
LABELS = ('左眼单目', '右眼单目', '联合双目')


def metrics(rows):
    position = np.array([float(r['position_error_mm']) for r in rows])
    orientation = np.array([float(r['orientation_error_deg']) for r in rows])
    return dict(samples=len(rows), position_rmse_mm=float(np.sqrt(np.mean(position**2))),
                position_p95_mm=float(np.percentile(position, 95)),
                position_max_mm=float(position.max()), within_3mm=int(np.sum(position <= 3)),
                orientation_rmse_deg=float(np.sqrt(np.mean(orientation**2))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--direction-run', type=Path)
    args = parser.parse_args()
    source, output = args.source_run.resolve(), args.output.resolve()
    if not (source/'completed.json').exists():
        raise ValueError('completed source run required')
    with (source/'dynamic_per_frame.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    indexed = {m: {r['source_frame_id']: r for r in rows if r['method'] == m} for m in METHODS}
    ids = set(indexed[METHODS[0]])
    assert all(set(indexed[m]) == ids for m in METHODS)
    common = set.intersection(*[{k for k, r in indexed[m].items() if r['status'] == 'ok'}
                                for m in METHODS])
    near = {k for k, r in indexed['joint_multicamera'].items()
            if .05 <= np.hypot(float(r['actual_axial_gap_m']), float(r['actual_lateral_mm'])/1000) <= .501}
    summary = dict(source_run=str(source), new_render_or_pose_fit=False,
                   all_three_methods_use_identical_accepted_ids=True,
                   rejected_frames_never_scored_as_zero=True, source_pairs=len(ids),
                   near_distance_definition='gripper origin to mechanism center Euclidean distance; 0.05–0.501m',
                   source_hashes={name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                                  for name in ('dynamic_per_frame.csv', 'calibration.json', 'estimator_policy.json',
                                               'completed.json', 'native_work_lights_ready.json')},
                   batches={}, availability={})
    records = []
    for name, selection in (('full_225s', ids), ('near_0p5_to_0p05m', near)):
        accepted = common & selection
        summary['batches'][name] = dict(total_pairs=len(selection), common_valid=len(accepted),
            methods={m: metrics([indexed[m][k] for k in sorted(accepted)]) for m in METHODS})
        for method in METHODS:
            available = [r for k, r in indexed[method].items() if k in selection and r['status'] == 'ok']
            summary['availability'].setdefault(name, {})[method] = dict(
                accepted=len(available), total=len(selection),
                accepted_within_3mm=sum(float(r['position_error_mm']) <= 3 for r in available))
            records.append(dict(batch=name, method=method, **summary['batches'][name]['methods'][method]))
    # Check the old publication against the recomputed source values.
    old = json.loads((source/'dynamic_summary.json').read_text(encoding='utf-8'))
    for mono in METHODS[:2]:
        assert old['paired'][mono]['common_valid'] == len(common)
        expected = old['paired'][mono]['mono']['position_rmse_mm']
        assert np.isclose(expected, summary['batches']['full_225s']['methods'][mono]['position_rmse_mm'],
                          atol=1e-12, rtol=0)
    old_near = json.loads((source/'requested_range_summary.json').read_text(encoding='utf-8'))
    assert old_near['n'] == len(near)
    assert old_near['valid'] == summary['batches']['near_0p5_to_0p05m']['common_valid']
    assert np.isclose(old_near['position_rmse_mm'],
                      summary['batches']['near_0p5_to_0p05m']['methods']['joint_multicamera']['position_rmse_mm'],
                      atol=1e-12, rtol=0)
    if args.direction_run is not None:
        direction = args.direction_run.resolve()
        assert (direction/'completed.json').exists()
        with (direction/'all_pairs_metrics.csv').open(encoding='utf-8-sig', newline='') as stream:
            direction_rows = list(csv.DictReader(stream))
        common_direction = [r for r in direction_rows if all(r[m+'_status'] == 'ok' for m in METHODS)]
        direction_metrics = {m: metrics([dict(position_error_mm=r[m+'_error_mm'],
            orientation_error_deg=r[m+'_orientation_error_deg']) for r in common_direction]) for m in METHODS}
        old_direction = json.loads((direction/'coverage_analysis.json').read_text(encoding='utf-8'))
        for method in METHODS:
            assert len(common_direction) == old_direction['common_mono_joint'][method]['total']
            assert np.isclose(direction_metrics[method]['position_rmse_mm'],
                old_direction['common_mono_joint'][method]['rmse_mm'], atol=1e-12, rtol=0)
        summary['direction_scan'] = dict(source_run=str(direction), total_pairs=len(direction_rows),
            common_valid=len(common_direction), methods=direction_metrics,
            source_sha256=hashlib.sha256((direction/'all_pairs_metrics.csv').read_bytes()).hexdigest(),
            joint_all_frames=old_direction['overall'], static_images_not_dynamic_grasp=True)
    output.mkdir(parents=True, exist_ok=True)
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    with (output/'comparison.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
    for ax, (batch, data), title in zip(axes, summary['batches'].items(),
                                      ('完整225秒：共同有效2219帧', '约0.5m至5cm：共同有效1420帧')):
        values = [data['methods'][m]['position_rmse_mm'] for m in METHODS]
        ax.bar(LABELS, values, color=('#7599c7', '#edb15c', '#39a58f'), width=.62)
        ax.set_ylim(0, .56)
        ax.set_title(title, fontproperties=font, fontsize=12)
        ax.set_ylabel('机构中心三维位置 RMSE / mm', fontproperties=font)
        ax.set_xticks(range(3), LABELS, fontproperties=font)
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
        for i, value in enumerate(values):
            ax.text(i, value+.012, f'{value:.3f}', ha='center')
    fig.suptitle('同一条原双目仿真轨迹：单目与双目同帧对比', fontproperties=font, fontsize=15)
    fig.savefig(output/'mono_stereo_comparison.png', dpi=170)
    plt.close(fig)
    print(json.dumps(dict(source_pairs=len(ids), common_valid=len(common), near_pairs=len(near),
                          near_common_valid=len(near & common), output=str(output)), ensure_ascii=False))


if __name__ == '__main__':
    main()
