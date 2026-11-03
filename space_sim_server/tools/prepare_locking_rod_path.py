"""Private native geometry search for the delivered captive rod, not dynamics acceptance."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend'),
    str(ROOT.parent / 'space_sim_UE_Adapter/Adapters'),
    str(ROOT.parent / 'space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--trials', type=int, default=16)
    args = parser.parse_args()
    if not 1 <= args.trials <= 80:
        parser.error('trials must be between 1 and 80')
    args.output.mkdir(parents=True, exist_ok=False)
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    load_native_grasp_module(ROOT / 'model/SARM/platform')
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe
    from simulation.scripted_tasks.motion import solve_pose
    from simulation.serial_chain_kinematics import axis_angle_to_matrix
    from simulation.scene_library.manifest import digest, write_json

    scene = json.loads(args.scene.read_text(encoding='utf-8-sig'))
    task = DrawerCycleTask(args.model)
    task.chain.enable_native_geometry()
    probe = NativePoseProbe(args.model, clearance_m=.002)
    probe.apply_scene_initial(scene)
    rod = probe.address('locking_rod_hinge')
    spin = probe.address('fastener_spin')
    slide = probe.address('fastener_slide')
    rng = np.random.default_rng(args.seed)
    trials, successful = [], None

    def pose(angle, turn, yaw, retreat=0.):
        probe.q[:] = probe.home
        probe.q[rod], probe.q[spin], probe.q[slide] = angle, turn, turn * .016 / (2*np.pi)
        probe.contacts(np.r_[np.zeros(6), 1., -1.])
        position, rotation = probe.body_pose('fastener')
        # TCP offset follows the delivered connector grasp calibration. Its
        # suitability for the nut is tested by actual bilateral surface contact.
        return (position + rotation[:, 2] * (-.003-retreat),
                rotation @ axis_angle_to_matrix(np.array([0., 0., 1.]), yaw))

    try:
        for trial in range(args.trials):
            yaw = (trial % 8) * np.pi/4
            seed = (np.asarray(scene['randomization']['arm_joint_position_rad'][:6])
                    if trial < 8 else rng.uniform(task.lower, task.upper))
            entry = dict(trial=trial, yaw_rad=yaw, seed_rad=seed.tolist())
            trials.append(entry)
            path = []
            try:
                p, r = pose(0., 0., yaw, .15)
                q = solve_pose(task.chain, p, r, seed, task.lower, task.upper)
                initial_q = np.r_[q, 1., -1.]
                hit = probe.pose_collision(initial_q, 'locking_rod_cycle',
                                           arm_penetration_m=0., clearance_m=.002)
                if hit:
                    raise ValueError('hover collision: ' + str(hit))
                # First establish the actual jaw separation at the grasp pose.
                grasp_q = solve_pose(task.chain, *pose(0., 0., yaw), q, task.lower, task.upper)
                opening = probe.grasp_opening(np.r_[grasp_q, 1., -1.], 'fastener')
                entry['contact_opening_rad'] = opening
                phases = [
                    ('approach', np.linspace(.15, 0., 61), lambda x: (0., 0., x, 1.)),
                    ('grip', np.linspace(1., opening, 25), lambda x: (0., 0., 0., x)),
                    ('fold', np.linspace(0., -.5, 81), lambda x: (x, 0., 0., opening)),
                    ('restore', np.linspace(-.5, 0., 81), lambda x: (x, 0., 0., opening)),
                    ('tighten', np.linspace(0., 2.2, 121), lambda x: (0., x, 0., opening)),
                    ('release', np.linspace(opening, 1., 25), lambda x: (0., 2.2, 0., x)),
                    ('retract', np.linspace(0., .15, 61), lambda x: (0., 2.2, x, 1.)),
                ]
                for phase, values, settings in phases:
                    entry['during'] = phase
                    for value in values:
                        angle, turn, retreat, jaws = settings(float(value))
                        p, r = pose(angle, turn, yaw, retreat)
                        next_q = solve_pose(task.chain, p, r, q, task.lower, task.upper)
                        if np.max(np.abs(next_q-q)) > .3:
                            raise ValueError('discontinuous IK branch')
                        q = next_q
                        candidate = np.r_[q, jaws, -jaws]
                        hit = probe.pose_collision(candidate, 'locking_rod_cycle',
                            ('fastener',) if phase != 'retract' else (),
                            joint_values={'locking_rod_hinge': angle, 'fastener_spin': turn,
                                          'fastener_slide': turn*.016/(2*np.pi)},
                            arm_penetration_m=0., clearance_m=0., mechanism_clearance_m=0.)
                        path.append(dict(stage=phase, joint_reference=candidate.tolist(),
                            rod_angle_rad=angle, fastener_spin_rad=turn,
                            fastener_slide_m=turn*.016/(2*np.pi)))
                        if hit:
                            entry['contact'] = hit
                            raise ValueError('path collision')
                entry['geometry_path_clear'] = True
                successful = dict(schema='space-arm-locking-rod-path/1', model_sha256=digest(args.model),
                    initial_joint_position_rad=initial_q.tolist(), samples=path)
                break
            except ValueError as error:
                entry.update(error=str(error), checked_samples=len(path), geometry_path_clear=False)
                print(json.dumps(entry), flush=True)
    finally:
        probe.close()
    report = dict(schema='space-arm-locking-rod-preflight/1', model=str(args.model.resolve()),
        model_sha256=digest(args.model), scene_sha256=digest(args.scene), seed=args.seed,
        trials=trials, geometry_path_clear=successful is not None, task_accepted=False,
        scope='private_static_geometry_only_no_physical_grasp_or_lock_acceptance')
    write_json(args.output / 'preflight.json', report)
    if successful is not None:
        write_json(args.output / 'path.json', successful)
        scene['randomization']['arm_joint_position_rad'] = successful['initial_joint_position_rad']
        write_json(args.output / 'initial-scene.json', scene)
    print(json.dumps(dict(geometry_path_clear=successful is not None, trials=len(trials))))
    return 0 if successful is not None else 1


if __name__ == '__main__':
    raise SystemExit(main())
