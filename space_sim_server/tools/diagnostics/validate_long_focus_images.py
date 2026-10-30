"""Use the saved original-light UE fixture to test the original stereo optics.

The legacy filename is retained for old review scripts; additional cameras
are withdrawn and cannot be requested by this entry point.

Requires --source-server containing the successful 2026-10-24 fixture and its
capture runner. Uses a separate hidden UE process and private native-light
preview tabs. Never writes the source scene_runtime or project configuration.
This is an optical pose sweep, not an integrated flight or physical camera test.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-server', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--channel', choices=['wide'], default='wide',
                        help='Original stereo only; extra-camera candidates withdrawn')
    parser.add_argument('--depths', type=float, nargs='+', default=[17.4, 12., 5.5, 3., 2.])
    parser.add_argument('--repeats', type=int, default=2)
    args = parser.parse_args()
    source = args.source_server.resolve()
    fixture = source/'run/end_stereo_20261023'
    output = args.output.resolve()
    if output.parent.name != 'results':
        raise ValueError('--output must have the form <audit-root>/results/<new-run-name>')
    if output.exists() or args.repeats < 1 or min(args.depths) <= .05:
        raise ValueError('new output directory, positive repeats and optical depths required')
    output.parent.mkdir(parents=True, exist_ok=True)
    sys.path[:0] = [str(fixture), str(source/'backend'),
                    str(source.parent/'space_sim_UE_Adapter/Adapters')]
    spec = importlib.util.spec_from_file_location('native_optical_fixture', fixture/'validate_far_candidates_20261027.py')
    audit = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(audit)
    audit.HERE = output.parent.parent  # ROOT/SOURCE retain explicit original fixture.
    audit.CANDIDATES = ['passive_long_focus']

    def apply(setup, manifest, name):
        far, scene = copy.deepcopy(setup), copy.deepcopy(manifest)
        record = dict(channel='original_wide', original_camera_definitions_retained=True,
                      original_markers_retained=True)
        setup.clear(); setup.update(far)
        manifest.clear(); manifest.update(scene)
        record.update(candidate=name, mounting_validated=False,
                      original_23_markers_retained=True, lighting_modified=False,
                      live_configuration_modified=False)
        return record

    audit.apply_candidate = apply
    tilts = [(0, 0), (0, 15), (90, 15), (0, 30), (90, 30)]
    audit.plans = lambda smoke: [dict(kind='surface_tilt', gap_m=z, azimuth_deg=az,
                                     angle_deg=a, lateral_m=list(xy))
        for z in args.depths for az, a in tilts
        for xy in [(0, 0), (.003, .002), (-.003, -.002)]]
    provenance = {}
    for path in [Path(__file__), SERVER/'simulation/dual_range_vision.py',
                 fixture/'validate_far_candidates_20261027.py',
                 source/'backend/space_arm_platform/multicamera_pose.py',
                 source/'backend/space_arm_platform/stereo_pose.py']:
        provenance[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    previous = sys.argv
    sys.argv = [str(__file__), '--name', output.name, '--candidate', 'passive_long_focus',
                '--repeats', str(args.repeats)]
    try:
        audit.main()
    finally:
        sys.argv = previous
        if output.exists():
            (output/'long_focus_sources.json').write_text(json.dumps(dict(
                hashes=provenance, original_source_server=str(source),
                native_preview_cameras='original main and original right; no added camera lights',
                scope='static actual UE images only; extra housing not validated'), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
