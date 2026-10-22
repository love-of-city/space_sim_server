"""Isolated image-only single-camera worker. Stdin accepts one PNG, no truth."""
import argparse
import base64
import json
from pathlib import Path
import sys
import cv2
import numpy as np
from .provided_mono import _load_original
from .grasp_alignment import alignment_from_pose


class WristEstimator:
    def __init__(self,config):
        self.config=config
        original=_load_original(Path(config['vendor_root']))
        self.estimator=original.FiducialEstimator()
        self.estimator.rig=config['rig']
        self.estimator.markers={m['id']:m for m in config['rig']['markers']}

    def __call__(self,png):
        image=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_COLOR)
        if image is None or list(image.shape[1::-1])!=self.config['image_size']:
            raise ValueError('wrist image dimensions do not match actual camera calibration')
        result=self.estimator.estimate(image,np.array(self.config['K']))
        result['camera_id']=self.config['camera_id']
        result['input_camera_id']=self.config['input_camera_id']
        count=len(result.get('detections',[]))
        if count<self.config['minimum_markers']:
            result.update(status='insufficient_markers' if count else 'lost',pose=None)
        result['alignment']=alignment_from_pose(result,self.config['mount'],self.config['standoff_m'])
        result['image_size']=self.config['image_size']
        if result['alignment']['valid']:
            mount=self.config['mount'];pose=result['pose'];K=np.array(self.config['K'])
            R_GC=np.array(mount['R_grip_from_camera_optical']);p_GC=np.array(mount['camera_position_grip_m'])
            p_GI=np.array(result['alignment']['interface_position_grip_m'])
            measured=np.array(pose['rotation_camera_from_target'])@np.array(mount['interface_origin_target_m'])+np.array(pose['translation_m'])
            axis=R_GC.T@(np.array([0.,0.,p_GI[2]])-p_GC)
            if measured[2]>0 and axis[2]>0:
                project=lambda p:((K@p)[:2]/p[2]).tolist()
                result['overlay_pixels']={'interface_center':project(measured),'grip_axis_at_estimated_depth':project(axis)}
        return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,required=True)
    config=json.loads(parser.parse_args().config.read_text('utf-8'))
    cv2.setNumThreads(1)
    estimator=WristEstimator(config)
    for line in sys.stdin.buffer:
        try:
            request=json.loads(line)
            if set(request)!={'png'}:raise ValueError('worker accepts only png')
            result=estimator(base64.b64decode(request['png'],validate=True))
        except Exception as error:
            result={'status':'error','reason':str(error),'alignment':{'valid':False,'control_enabled':False}}
        print(json.dumps(result,allow_nan=False,separators=(',',':')),flush=True)


if __name__=='__main__':main()
