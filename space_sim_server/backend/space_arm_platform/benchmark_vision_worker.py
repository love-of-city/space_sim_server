"""Image-only baselines; launch in a separate process from Basilisk/MJScene."""
from pathlib import Path
import json
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .benchmark_external import serve_method
from .benchmark_vision import TargetPose, VisionPoseAction, VisionPoseContext, VisionPoseObservation
from .provided_mono import _current_rig, _load_original
from .stereo_pose import StereoPoseEstimator


class ImagePoseMethod:
    def reset(self,context):
        self.context=context
        cameras=context.task.cameras
        self.layout=dict(dictionary=context.marker_dictionary,
            markers={str(marker.marker_id):marker.corners_target_m for marker in context.markers})
        self.k=[np.asarray(camera.intrinsics) for camera in cameras]
        self.mono=len(cameras)==1
        cv2.setNumThreads(1)
        if self.mono:
            original=_load_original(Path(__file__).resolve().parents[2]/'config/vision/vendor/aruco_pnp_basler_mono_v1')
            self.estimator=original.FiducialEstimator()
            self.estimator.rig=_current_rig(self.layout)
            self.estimator.markers={marker['id']:marker for marker in self.estimator.rig['markers']}
        else:
            rotations=[Rotation.from_quat(np.asarray(camera.orientation_body_from_optical_wxyz)[[1,2,3,0]]).as_matrix() for camera in cameras]
            left,right=rotations
            self.estimator=StereoPoseEstimator(dict(K_left=self.k[0],K_right=self.k[1],D_left=[],D_right=[],
                image_size=[cameras[0].width,cameras[0].height],R_right_from_left=right.T@left,
                t_right_from_left_m=right.T@(np.asarray(cameras[0].position_body_m)-cameras[1].position_body_m)),self.layout)

    def act(self,observation):
        images=observation.images
        if tuple(image.camera_id for image in images)!=tuple(camera.camera_id for camera in self.context.task.cameras):
            raise ValueError('method image cameras differ from reset calibration')
        pose=None
        if self.mono:
            image=cv2.imdecode(np.frombuffer(images[0].png_bytes(),np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape[:2]!=(images[0].height,images[0].width):
                raise ValueError('image decoding differs from its validated dimensions')
            result=self.estimator.estimate(image,self.k[0])
            if result['status']=='tracking' and result.get('marker_count',0)>=2:
                raw=result['pose']
                pose=TargetPose(position_target_origin_optical_m=tuple(raw['translation_m']),
                    orientation_optical_from_target_wxyz=tuple(Rotation.from_matrix(raw['rotation_camera_from_target']).as_quat()[[3,0,1,2]]))
        else:
            result=self.estimator.estimate(images[0].png_bytes(),images[1].png_bytes())
            if result['status']=='ok':
                raw=result['pose_target_to_left']
                pose=TargetPose(position_target_origin_optical_m=tuple(raw['t_m']),
                    orientation_optical_from_target_wxyz=tuple(raw['quaternion_wxyz']))
        print(json.dumps(dict(sequence=observation.sequence,status=result['status'],
            detected_markers=len(result.get('detections',[])) if self.mono else result.get('counts'),
            reprojection_error_px=result.get('reprojection_error_px')),allow_nan=False),file=sys.stderr,flush=True)
        return VisionPoseAction(episode_id=observation.episode_id,observation_sequence=observation.sequence,
            estimate_time_ns=observation.sample_time_ns,reference_frame=self.context.task.reference_frame,pose=pose)

    def close(self):
        pass


if __name__=='__main__':
    raise SystemExit(serve_method(ImagePoseMethod(),VisionPoseContext,VisionPoseObservation))
