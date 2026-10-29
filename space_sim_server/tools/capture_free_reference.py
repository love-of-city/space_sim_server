"""Independent free-flight scorer. Changes render transport only, not physics/control.

The saved reference scene disables the grasp and vision. The real control client
remains connected to the isolated backend. Its neutral default holds the arms.
No reference data is ever opened by the grasp process.
"""
from pathlib import Path
import argparse
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--adapter-root',type=Path,required=True)
    parser.add_argument('--control-port',type=int,required=True)
    parser.add_argument('--duration',type=float,required=True)
    args=parser.parse_args()
    from space_arm_platform.capture_regression import free_reference_scene
    scene=free_reference_scene(json.loads(args.scene.read_text('utf-8')))
    path=args.output/'free_reference_scene.json'
    path.write_text(json.dumps(scene,indent=2),encoding='utf-8')
    from simulation import teleop_grasp_unreal as app
    from simulation.components.render import RenderComponent
    from simulation.assembly import Slot
    from Basilisk.architecture import sysModel
    import bsk_render_adapter
    from bsk_render_adapter.protocol import RecordingOnlyPublisher
    bridge_class=bsk_render_adapter.BasiliskRenderBridge
    # Keep actual scene registration, output cadence, observations and backend
    # handshake. Only the network rendering destination is a discard sink.
    def bridge_without_ue(**kwargs):
        return bridge_class(**kwargs,publisher=RecordingOnlyPublisher())
    bsk_render_adapter.BasiliskRenderBridge=bridge_without_ue
    stream=(args.output/'free_reference.jsonl').open('w',encoding='utf-8')
    class Recorder(sysModel.SysModel):
        def __init__(self,stepper):
            super().__init__();self.ModelTag='freeTargetScorer';self.stepper=stepper
        def UpdateState(self,ns):
            stream.write(json.dumps(dict(t=ns/1e9,target=self.stepper.body_detail('capture_target').tolist()))+'\n')
    original_install=RenderComponent.install
    def install(self,ctx):
        original_install(self,ctx)
        recorder=Recorder(ctx.simulation.local_stepper)
        ctx.add(recorder,Slot.OUTPUT,every=8);ctx.keep_alive(recorder)
    RenderComponent.install=install
    sys.argv=['free-reference','--scene-instance',str(path),'--duration',str(args.duration),
        '--model-root',str(ROOT/'model/SARM/platform'),'--adapter-root',str(args.adapter_root),
        '--catalog',str(args.adapter_root/'Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_dual_capture26.catalog.json'),
        '--control-port',str(args.control_port),'--dynamics-backend','local','--orbital-mode','bsk']
    try:app.main()
    finally:stream.close()


if __name__=='__main__':main()
