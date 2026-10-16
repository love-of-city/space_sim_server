"""Make review figures from real recorded drawer episodes (never dataset images)."""

from pathlib import Path
import argparse
import json
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch", type=Path)
    args = parser.parse_args()
    report = json.loads((args.batch / "summary.json").read_text(encoding="utf-8"))
    paths = [Path(p) for p in report["accepted_episodes"]]
    if not paths:
        raise ValueError("No accepted episodes")
    fig, axes = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True, constrained_layout=True
    )
    table = []
    for index, episode in enumerate(paths, 1):
        observations = [
            json.loads(line)["observation"]
            for line in (episode / "steps.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        times = np.array([int(o["sim_time_ns"]) for o in observations]) * 1e-9
        times -= times[0]
        drawer = (
            np.array(
                [o["scripted_task"]["drawer_displacement_m"] for o in observations]
            )
            * 1000
        )
        opening = np.array([o["gripper_opening_normalized"] for o in observations])
        axes[0].plot(times, drawer, label=f"Episode {index}")
        axes[1].plot(times, opening, label=f"Episode {index}")
        meta = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))
        entry = next(
            a for a in report["attempts"] if a.get("episode_id") == episode.name
        )
        table.append(
            dict(
                episode_id=episode.name,
                seed=entry["seed"],
                frames=len(observations),
                simulated_seconds=float(times[-1]),
                recording_wall_seconds=entry["recording_wall_seconds"],
                maximum_drawer_mm=float(drawer.max()),
                final_drawer_mm=float(drawer[-1]),
                final_buckle_rad=observations[-1]["scripted_task"]["buckle_rad"],
                initial_offset_deg=entry["initial_state"]["offset_deg"],
                outcome=meta["outcome"],
            )
        )
        if index == 1:
            first_episode, first_obs = episode, observations
    axes[0].set_ylabel("Drawer displacement (mm)")
    axes[1].set_ylabel("Gripper opening (0 closed, 1 open)")
    axes[1].set_xlabel("Simulation time from first recorded frame (s)")
    axes[0].axhline(80, color="gray", linestyle=":", linewidth=0.8)
    axes[0].axhline(0, color="gray", linestyle=":", linewidth=0.8)
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend(loc="upper right", ncol=3)
    fig.suptitle("Recorded drawer cycles: open, return, release, retreat")
    fig.savefig(args.batch / "drawer-cycles.png", dpi=150)
    plt.close(fig)
    captures = [
        json.loads(line)
        for line in (first_episode / "captures.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    images = {
        (c["metadata"]["camera_id"], str(c["source_frame_id"])): first_episode
        / c["files"]["rgb"]
        for c in captures
    }
    stages = [
        ("descend", "下降抓取"),
        ("hold", "抽出并保持"),
        ("return", "放回途中"),
        ("release", "归位后松爪"),
        ("verify", "撤离后检查"),
    ]
    width = 400
    sheet = Image.new("RGB", (width * len(stages), 875), "#111827")
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
    for col, (stage, label) in enumerate(stages):
        group = [o for o in first_obs if o["scripted_task"]["stage"] == stage]
        obs = group[len(group) // 2]
        draw.text((width * col + 10, 10), label, font=font, fill="white")
        for row, camera in enumerate(
            ("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam")
        ):
            with Image.open(images[(camera, obs["render_frame_id"])]) as image:
                sheet.paste(
                    image.convert("RGB").resize((width, width)),
                    (width * col, 44 + row * width),
                )
        draw.text(
            (width * col + 10, 848),
            f"{int(obs['sim_time_ns']) / 1e9:.2f}s | {obs['scripted_task']['drawer_displacement_m'] * 1000:.2f} mm",
            font=font,
            fill="white",
        )
    sheet.save(args.batch / "drawer-cycle-preview.jpg", quality=94)
    result = dict(
        batch_status=report["status"],
        successes=len(paths),
        attempts=len(report["attempts"]),
        wall_seconds=report["wall_seconds"],
        episodes=table,
    )
    (args.batch / "review.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
