"""Render a native model in a separate Python-MuJoCo process, without Basilisk."""
import argparse
import gzip
import json
from pathlib import Path
import struct
import sys
import zlib

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def load_geometry_model(path):
    if "Basilisk" in sys.modules:
        raise RuntimeError("Geometry inspection must not share a process with Basilisk")
    from simulation.mujoco_plugins import validate_plugin_manifest
    manifest = ROOT / "run/native_acceleration/sarm_thread.json"
    record = json.loads(manifest.read_text(encoding="utf-8"))
    library = ROOT / record["library"]
    validate_plugin_manifest(library, manifest, mujoco.mj_version())
    mujoco.mj_loadPluginLibrary(str(library))
    return mujoco.MjModel.from_xml_path(str(Path(path).resolve()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--scene", type=Path)
    source.add_argument("--state-trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--azimuth", type=float, default=140.)
    parser.add_argument("--elevation", type=float, default=-25.)
    parser.add_argument('--focus-body', help='Optional named body for a diagnostic close view')
    parser.add_argument('--distance', type=float, default=2.7, help='Diagnostic camera distance in metres')
    parser.add_argument("--role-colors", action="store_true", help="Color ownership roles in this diagnostic image only")
    parser.add_argument("--diagnostic-lighting", action="store_true", help="Use bounded MuJoCo preview lighting; source UE lamp intensity stays unchanged")
    parser.add_argument("--time-s", type=float, help="Select first trace sample at or after this time")
    args = parser.parse_args()
    if not np.isfinite(args.distance) or args.distance <= 0:
        parser.error('distance must be finite and positive')
    if args.time_s is not None and (not args.state_trace or not np.isfinite(args.time_s) or args.time_s < 0):
        parser.error("--time-s requires a trace and a finite nonnegative time")
    from simulation.scene_library.manifest import digest, write_json
    model = load_geometry_model(args.model)
    data = mujoco.MjData(model)
    if args.diagnostic_lighting:
        # The delivered UE wrist lamp uses intensity 1000, which saturates the
        # MuJoCo rasterizer. This is a labelled preview-only lighting override.
        model.light_diffuse[:] = np.clip(model.light_diffuse, 0., .8)
        model.light_specular[:] = np.clip(model.light_specular, 0., .2)
    if args.role_colors:
        colors = {"mounted_capture_target": [.2, .5, .85, 1.],
                  "service_receiver_mount": [1., .55, .12, 1.],
                  "task_box_base": [.6, .65, .7, 1.], "drawer_base": [.4, .7, .5, 1.]}
        for gid in range(model.ngeom):
            bid = int(model.geom_bodyid[gid])
            while bid:
                if model.body(bid).name in colors:
                    model.geom_rgba[gid] = colors[model.body(bid).name]
                    model.geom_matid[gid] = -1
                    break
                bid = int(model.body_parentid[bid])
    if args.state_trace:
        summary_path = args.state_trace.parent / "run/summary.json"
        if not summary_path.is_file():
            raise ValueError("Trace requires its native run/summary.json provenance")
        summary = json.loads(summary_path.read_text(encoding="utf-8-sig"))
        if summary.get("model_sha256") != digest(args.model):
            raise ValueError("Trace model hash does not match the preview model")
        with gzip.open(args.state_trace, "rt") as stream:
            last = None
            for line in stream:
                last = json.loads(line)
                if args.time_s is not None and last["sim_time_ns"] >= args.time_s * 1e9:
                    break
        if last is None or len(last["qpos"]) != model.nq:
            raise ValueError("Trace does not match this model state dimension")
        if args.time_s is not None and last["sim_time_ns"] < args.time_s * 1e9:
            raise ValueError("Requested time is beyond the trace")
        data.qpos[:] = last["qpos"]
        state_source = dict(trace=str(args.state_trace), trace_sha256=digest(args.state_trace),
                            time_ns=last["sim_time_ns"])
    else:
        scene = json.loads(args.scene.read_text(encoding="utf-8-sig"))
        q = scene["randomization"]["arm_joint_position_rad"]
        for name, value in zip([*(f"joint{i}" for i in range(1, 7)), "ee_link11_joint", "ee_link12_joint"], q):
            data.qpos[model.joint(name).qposadr[0]] = value
        target_joint = model.joint("capture_target_free")
        target_address = int(target_joint.qposadr[0])
        randomization = scene["randomization"]
        data.qpos[target_address:target_address+3] = randomization["target_position_m"]
        quaternion = np.asarray(randomization["target_orientation_wxyz"], dtype=float)
        if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-12:
            raise ValueError("Invalid initial target quaternion")
        data.qpos[target_address+3:target_address+7] = quaternion / np.linalg.norm(quaternion)
        state_source = dict(scene=str(args.scene), source_sha256=digest(args.scene))
    mujoco.mj_forward(model, data)
    contacts = [dict(first=model.geom(int(c.geom[0])).name, second=model.geom(int(c.geom[1])).name,
                     separation_m=float(c.dist)) for c in data.contact]
    origin = data.body("cubesat_bus").xpos
    orientation = data.body("cubesat_bus").xmat.reshape(3, 3)
    body_positions = {name: (orientation.T @ (data.body(name).xpos-origin)).tolist()
                      for name in ("bolt", "aviation_plug", "guide_plug", "cylindrical_plug", "link5")}
    width, height = 1200, 900
    model.vis.global_.offwidth, model.vis.global_.offheight = width, height
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (data.body(args.focus_body).xpos if args.focus_body
                       else origin + orientation @ np.array([.3, .25, .2]))
    camera.distance, camera.azimuth, camera.elevation = args.distance, args.azimuth, args.elevation
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # Hide collision-only meshes in the picture, not physics.
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera, scene_option=option)
        rgb = renderer.render()
    if not np.isfinite(data.qpos).all():
        raise ValueError("Non-finite model state")
    def chunk(kind, payload):
        return struct.pack("!I", len(payload)) + kind + payload + struct.pack("!I", zlib.crc32(kind + payload) & 0xffffffff)
    raw = b"".join(b"\0" + row.tobytes() for row in rgb)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "preview.png").write_bytes(png)
    write_json(args.output / "preview.json", dict(model_sha256=digest(args.model), source=state_source,
               camera=dict(lookat_world_m=camera.lookat.tolist(), distance_m=args.distance,
                           focus_body=args.focus_body,
                           azimuth_deg=args.azimuth, elevation_deg=args.elevation),
               diagnostic_role_colors=args.role_colors,
               diagnostic_lighting=args.diagnostic_lighting,
               body_positions_in_service_frame_m=body_positions,
               mujoco_version=mujoco.__version__, contacts=contacts, task_accepted=False,
               limitation="Image and discrete contact snapshot do not establish full-path or rigid-assembly clearance"))
    print(str(args.output / "preview.png"))


if __name__ == "__main__":
    main()
