"""Find a collision-free drawer start in a private native planning model."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend"),
    str(ROOT.parent / "space_sim_UE_Adapter/Adapters"),
    str(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples")]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    load_native_grasp_module(ROOT / "model/SARM/platform")
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe
    from simulation.scripted_tasks.motion import solve_pose
    from simulation.scene_library.manifest import digest, write_json
    task = DrawerCycleTask(args.model)
    task.chain.enable_native_geometry()
    goal = task.grip_origin + task.axis * task.config.clearance_m
    down = -task.axis
    scene = json.loads(args.scene.read_text(encoding="utf-8-sig"))
    original = np.asarray(scene["randomization"]["arm_joint_position_rad"][:6])
    rng = np.random.default_rng(args.seed)
    trials = []
    probe = NativePoseProbe(args.model, clearance_m=.002)
    probe.apply_scene_initial(scene)
    solution = None
    try:
        for trial in range(80):
            seed = original if trial == 0 else rng.uniform(task.lower, task.upper)
            closing = task.drawer_rotation[:, 1] * (1 if trial % 2 == 0 else -1)
            rotation = np.column_stack([closing, np.cross(down, closing), down])
            try:
                q = solve_pose(task.chain, goal, rotation, seed, task.lower, task.upper)
                collisions = []
                # Check both initial closed fingers and the opening corridor.
                for opening in np.linspace(0., 1., 17):
                    hit = probe.pose_collision(np.r_[q, opening, -opening], "free_motion",
                                               arm_penetration_m=0., clearance_m=.002)
                    if hit:
                        collisions.append(str(hit))
                        break
                trials.append(dict(trial=trial, seed_rad=seed.tolist(), q_rad=q.tolist(), collisions=collisions))
                if not collisions:
                    solution = np.r_[q, 0., 0.].tolist()
                    break
            except ValueError as error:
                trials.append(dict(trial=trial, seed_rad=seed.tolist(), error=str(error)))
    finally:
        probe.close()
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "preflight.json", dict(model_sha256=digest(args.model), seed=args.seed,
               target_position_m=goal.tolist(), trials=trials, pose_found=solution is not None,
               full_task_accepted=False, runtime_state_overridden=False))
    if solution is None:
        return 1
    scene["randomization"]["arm_joint_position_rad"] = solution
    write_json(args.output / "initial-scene.json", scene)
    print(json.dumps(dict(initial_joint_deg=np.rad2deg(solution[:6]).tolist(), trials=len(trials))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
