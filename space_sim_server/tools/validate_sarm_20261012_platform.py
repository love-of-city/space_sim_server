"""Offline native integration probe using the same Basilisk DLL as the platform.

Run with the platform Python. No Python MuJoCo package is imported. An integration
warning, automatic reset or non-finite state fails the run in LocalMujocoStepper.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent/'space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples'))

def validate_cycles(samples):
    """Functional bounds, not a claim of microradian physical accuracy.

    Require >=95% stroke each way, <=0.025 rad tracking error (one .02-rad
    command increment plus .005 rad), and <1e-6 rad opposed-link closure error.
    """
    if max(abs(s['positions'][6]-s['target_rad']) for s in samples) > .025:
        raise AssertionError('tendon target tracking failed')
    if max(s['coupling_error_rad'] for s in samples) > 1e-6:
        raise AssertionError('opposed linkage equality failed')
    for start in (10,20,30):
        opening=[s['positions'][6] for s in samples if start < s['time_s'] <= start+5]
        closing=[s['positions'][6] for s in samples if start+5 < s['time_s'] <= start+10]
        if not opening or not closing or max(opening) < .95 or closing[-1] > .05:
            raise AssertionError('full gripper stroke not observed')

def verify_sdf_equivalence(output: Path, reference_commit: str) -> dict:
    """Compare the compiled SDF and gradient with a frozen Git reference."""
    import re
    import subprocess
    from tools.build_native_acceleration import compiler_environment, VENDORED_MUJOCO_INCLUDE
    if not re.fullmatch(r"[0-9a-fA-F]{7,40}", reference_commit):
        raise ValueError("reference-commit must be a Git commit hash")
    output = output.resolve()
    if output.exists():
        raise ValueError("use a new output path; preserve previous evidence")
    output.parent.mkdir(parents=True, exist_ok=True)
    repo = ROOT.parent
    out = output.parent
    path = "space_sim_server/model/SARM_mjcf_20261012/thread_sdf.cpp"
    original = subprocess.check_output(["git", "-C", str(repo), "show", f"{reference_commit}:{path}"], encoding="utf-8")
    candidate = (repo / path).read_text(encoding="utf-8")
    def functions(text, name):
        text = text.split('const char* const kAttributes[]')[0]
        text = re.sub(r'^#include[^\n]*\n', '', text, flags=re.M)
        return f"namespace {name} {{\n{text}\n}}\n}}\n"
    program = '''#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <mujoco/mujoco.h>
#include <iostream>
#include <thread>
#include <atomic>
'''
    program += functions(original, "reference") + functions(candidate, "optimized")
    program += r'''
std::atomic<int> failures{0};
std::atomic<int> checks{0};
void check(const double p[3], int mode) {
  const double expected = reference::distance(p,mode);
  for(int repeat=0;repeat<2;++repeat) {
    double actual = optimized::distance(p,mode);
    double a[3],b[3]; reference::gradient(a,p,mode); optimized::gradient(b,p,mode);
    if(std::memcmp(&actual,&expected,sizeof(double)) || std::memcmp(a,b,sizeof(a))) ++failures;
    ++checks;
  }
}
void run(int seed, int count) {
  std::uint64_t state=seed;
  auto number=[&]() { state=state*6364136223846793005ULL+1442695040888963407ULL;
    return (state>>11)*0x1.0p-53; };
  for(int i=0;i<count;++i) {
    double p[3]={.16*number()-.08,.16*number()-.08,.18*number()-.16};
    for(int mode=-1;mode<=5;++mode) check(p,mode);
  }
  for(double x:{-0.,0.,.0135,.015,.0105,.012,.023,.013,.01265,.020})
    for(double z:{-0.,0.,-.020,-.060,-.065,-.110})
      for(double delta:{0.,-2e-6,2e-6,-1e-15,1e-15}) {
        double p[3]={x+delta,0.,z+delta};
        for(int mode=-1;mode<=5;++mode) check(p,mode);
      }
}
int main() {
  run(42,50000);
  std::thread a([]{run(71,5000);}),b([]{run(81,5000);}),c([]{run(91,5000);});
  a.join();b.join();c.join();
  std::cout << "checks=" << checks << " mismatches=" << failures << std::endl;
  return failures ? 1 : 0;
}
'''
    source = output.with_suffix(".cpp")
    executable = output.with_suffix(".exe")
    source.write_text(program, encoding="utf-8")
    env = compiler_environment()
    compiler = Path(env["VCToolsInstallDir"]) / "bin/Hostx64/x64/cl.exe"
    subprocess.run([str(compiler), "/nologo", "/O2", "/fp:strict", "/EHsc", "/std:c++17", "/MD",
                    str(source), f"/I{VENDORED_MUJOCO_INCLUDE}", f"/Fo:{out/'sdf-equivalence.obj'}",
                    f"/Fe:{executable}"], cwd=out, env=env, check=True)
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    report = {"reference_commit": reference_commit, "reference_sha256": hashlib.sha256(original.encode()).hexdigest(),
              "candidate_sha256": hashlib.sha256(candidate.encode()).hexdigest(), "passed": result.returncode == 0,
              "result": result.stdout.strip(),
              "comparison": "bitwise signed distances and finite-difference gradients, all modes, boundaries, eviction and concurrent threads"}
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report), flush=True)
    if result.returncode:
        raise RuntimeError(f"SDF equivalence failed: {result.stdout.strip()} {result.stderr.strip()}")
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'run/sarm_20261012_validation.json')
    parser.add_argument('--sdf-equivalence', action='store_true', help='Compile a bitwise distance/gradient comparison; no simulation')
    parser.add_argument('--reference-commit', default='9011b97c9e19939cc984f98c36085f183a27f885')
    args=parser.parse_args()
    if args.sdf_equivalence:
        verify_sdf_equivalence(args.output, args.reference_commit)
        return
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    n=load_native_grasp_module(ROOT/'model/SARM/platform')
    from simulation.sarm_model_profile import load_model_profile
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from Basilisk.utilities import macros
    n.MODEL_PATH=ROOT/'model/SARM/platform/sarm_20261012.xml'
    p=load_model_profile(n.MODEL_PATH)
    n.JOINTS=p.joints; n.ACTUATORS=p.arm_actuators
    n.KP=np.full(6,500.); n.KD=np.full(6,50.);n.TORQUE_LIMITS=np.full(6,100.);n.TIME_STEP=1/240
    q=np.zeros(8)
    ref=HeldJointReferencePublisher(lambda t:(q.copy(),np.zeros(8)),8)
    simulation,scene,keep,_=n._build_simulation(external_reference=ref,dynamics_backend='local',local_substeps=5,
        record_history=False,attitude_control_enabled=False)
    simulation.AddModelToTask('graspTask',ref,9000)
    simulation.InitializeSimulation()
    # Exercise Galilean common drift, which exposed source free-joint world
    # damping during the real platform startup (zero velocity concealed it).
    from space_arm_platform.task_box import initialize_free_plugs
    initialize_free_plugs(scene,n.MODEL_PATH,np.zeros(3),n.COMMON_VELOCITY)
    for name in ('cubesat_bus','capture_target'):
        scene.getBody(name).setVelocity(n.COMMON_VELOCITY)
    reads=[scene.getBody(b).getScalarJoint(j).stateOutMsg.addSubscriber() for b,j in p.joints]
    samples=[]
    for i in range(1,401):
        seconds=i/10
        # Ten seconds at rest, then three full 0..1..0 cycles at 0.2 rad/s.
        phase=max(0.,seconds-10)%10
        q[6]=.2*min(phase,10-phase);q[7]=-q[6]
        simulation.ConfigureStopTime(macros.sec2nano(seconds));simulation.ExecuteSimulation()
        stepper=simulation.local_stepper
        if not np.isfinite(stepper.qpos).all() or not np.isfinite(stepper.qvel).all():
            raise RuntimeError('Non-finite physical state')
        positions=[float(read().state) for read in reads]
        samples.append(dict(time_s=seconds,target_rad=float(q[6]),positions=positions,
            coupling_error_rad=abs(positions[6]+positions[7]),max_qacc=float(np.abs(stepper.qacc).max())))
        if i%10==0: print(json.dumps(samples[-1]),flush=True)
    validate_cycles(samples)
    report=dict(model_sha256=hashlib.sha256(n.MODEL_PATH.read_bytes()).hexdigest(),runtime='Basilisk MuJoCo 3.7.0',
        substeps=5,nq=stepper.nq,nv=stepper.nv,samples=samples,
        max_coupling_error_rad=max(s['coupling_error_rad'] for s in samples),
        scope='zero arm pose with common free-body drift, 10 seconds idle and three full tendon cycles; no grasp/plug insertion acceptance')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f'PASS {args.output}',flush=True)
    _=keep

if __name__=='__main__': main()
