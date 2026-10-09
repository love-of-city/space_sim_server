"""Render a STEP-converted MJCF using CPU or the system's OpenGL renderer."""
from __future__ import annotations
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_directory',type=Path)
    parser.add_argument('--backend',choices=('cpu','opengl'),default='cpu')
    parser.add_argument('--interactive',action='store_true',help='Open MuJoCo viewer (requires working system OpenGL)')
    parser.add_argument('--free-preview',action='store_true',help='View the synthetic free-body model (interactive only)')
    args=parser.parse_args()
    folder=args.model_directory.resolve()
    manifest_path=folder/'conversion_manifest.json'
    manifest=json.loads(manifest_path.read_text('utf-8'))
    if args.interactive:
        import mujoco.viewer
        filename='satellite_free_preview.xml' if args.free_preview else 'ground_validation_satellite.xml'
        mujoco.viewer.launch_from_path(str(folder/filename))
    else:
        if args.backend=='cpu':
            from cpu_mjcf_preview import render_cpu_preview
            report=render_cpu_preview(folder,manifest['bounds']['dimensions_m'])
        else:
            from convert_step_to_mjcf import render_preview
            name=render_preview(folder,manifest['bounds']['dimensions_m'])
            report=dict(overview='preview/'+name,renderer='MuJoCo system OpenGL renderer',mujoco_opengl_renderer=True,ai_generated=False)
        manifest['preview']=report
        manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        print(folder/'preview'/'overview.png',flush=True)


if __name__=='__main__':
    main()
