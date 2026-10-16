"""Render an actual torque-driven tighten/release cycle, not a pose animation.

Offline only: requires MuJoCo 3.7.0, Pillow and an OpenGL context. Same plugin
setup as validate_captive_fastener.py. The driver is a torque fixture, not a robot.
"""
import argparse
import json
from pathlib import Path

import mujoco as mj
from PIL import Image, ImageDraw

from validate_captive_fastener import ROOT, address, checked_step, hook_contact_force


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    record = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text())
    mj.mj_loadPluginLibrary(str(ROOT/record['library']))
    model = mj.MjModel.from_xml_path(str(ROOT/'model/SARM/platform/sarm_20261012.xml'))
    model.opt.timestep = 1/240
    model.opt.disableflags |= mj.mjtDisableBit.mjDSBL_AUTORESET
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    z, _ = address(model, 'fastener_slide')
    _, td = address(model, 'fastener_spin')
    camera = mj.MjvCamera()
    camera.lookat[:] = (data.body('locking_rod').xpos+data.body('fastener').xpos)/2
    camera.distance, camera.azimuth, camera.elevation = .26, 100, -10
    frames = []
    with mj.Renderer(model, height=480, width=640) as renderer:
        for label, torque, steps in [('Tighten', .06, 960), ('Hold', 0., 240),
                                      ('Loosen', -.15, 1440), ('Released', 0., 240)]:
            for i in range(steps):
                data.qfrc_applied[td] = torque*min(1., (i+1)/48)
                checked_step(model, data)
                if i % 16 == 0:
                    renderer.update_scene(data, camera=camera)
                    frame = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((0, 0, 640, 46), fill=(15, 20, 30))
                    draw.text((10, 5), f'{label} | t={data.time:.2f}s | applied torque {data.qfrc_applied[td]:+.3f} N m', fill='white')
                    draw.text((10, 25), f'Travel {data.qpos[z]*1000:.3f} mm | hook contact {hook_contact_force(model, data):.1f} N', fill='white')
                    frames.append(frame)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=67, loop=0)
    print(args.output)


if __name__ == '__main__':
    main()
