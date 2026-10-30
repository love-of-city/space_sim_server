"""Additional axis and moving-reference tests in the reduced native fixture.

Uses actual camera FK, estimated GNSS relative position/velocity, ideal service
attitude/gyro and target cooperative telemetry. Gravity, arm deployment,
occlusion and image decoding are not modeled. This is not an orbital mission.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SERVER),str(SERVER/'backend')]
from simulation.end_stereo_pointing import EndStereoGeometry
from simulation.orbital_rendezvous.model import assembly_properties
from tools.diagnostics.validate_cooperative_pointing import fixture
from tools.diagnostics.validate_end_stereo_pointing import run_case


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-model',type=Path,required=True)
    parser.add_argument('--camera-config',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=SERVER/'run/end_stereo_tracking')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    geometry = EndStereoGeometry.from_files(args.scene_model,args.camera_config)
    properties = assembly_properties(ET.parse(args.scene_model).find('./worldbody/body[@name="capture_target"]'))
    model = output/'target_rigid_fixture.xml'
    fixture(model,properties)
    q = np.array([-np.pi/2,0.,0.,0.,0.,0.])
    cases = []
    for name,axis in zip('xyz',np.eye(3)):
        for sign in (-1,1):
            cases.append(dict(label=f'{name}_{"plus" if sign>0 else "minus"}_90deg',
                              angle=90.,initial_axis=axis*sign))
        cases.append(dict(label=f'{name}_180deg',angle=180.,initial_axis=axis))
    cases += [
        dict(label='service_roll_and_initial_target_spin',angle=90.,service_spin=[.0011,0,0],
             target_spin=[.01,-.015,.005],motion_reference=True,tracking_hold_s=10.),
        dict(label='transverse_translation_rotating_service',angle=90.,service_spin=[0,0,.0011],
             target_com_velocity=[0,.02,0],motion_reference=True,tracking_hold_s=10.),
    ]
    results = []
    for case in cases:
        result = run_case(model,geometry,output,depth=17.4,q=q,expect_alignment=True,
                          duration=120.,**case)
        results.append(result)
        print(json.dumps(dict(label=result['label'],passed=result['passed'],
            completed_s=result['completed_s'],error_deg=result['final_error_deg'],
            rate_error_rad_s=result['final_rate_rad_s'])),flush=True)
    summary = dict(scope=__doc__,sources={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (args.scene_model,args.camera_config)},
        target_properties=properties,assumptions=['fixed measured arm posture; no arm motion dynamics',
            'ideal cooperative and service attitude/gyro telemetry','GNSS receiver-solution noise and EKF',
            'zero-gravity rigid target and freely rotating service; no orbit model',
            'motion cases use reference rate from navigation velocity, service gyro and zero encoder rates',
            'projection only; no image detection or occlusion evidence'],
        full_mission_validated=False,results=results,passed=all(r['passed'] for r in results))
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=summary['passed'],cases=len(results))))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
