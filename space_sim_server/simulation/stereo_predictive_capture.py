"""23-marker stereo observations feeding the shared gyro/encoder predictor."""
import json
from pathlib import Path
import time

from simulation.visual_predictive_capture import VisualPredictiveCapture
from space_arm_platform.stereo_capture_packet import configuration_id


class StereoPredictiveCapture(VisualPredictiveCapture):
    def __init__(self, path, session_id, *, config_file, held_pose=True,
                 wall_now_ns=time.time_ns):
        if not held_pose:
            raise ValueError('near23 stereo gyro capture requires held_target_pose')
        super().__init__(path, session_id, held_pose=True, wall_now_ns=wall_now_ns)
        self.config_file = Path(config_file)
        self.config = json.loads(self.config_file.read_text('utf-8-sig'))
        self.configuration_id = configuration_id(self.config)

    def _pose(self, t):
        p, r = super()._pose(t)
        packet = self.last_packet
        if (packet.get('schema') != 'near23-stereo-capture/1'
                or packet.get('configuration_id') != self.configuration_id
                or packet.get('camera_ids') != self.config['camera_ids']):
            raise ValueError('stereo_configuration_mismatch')
        age = t-int(packet['sim_time_ns'])/1e9
        if age > .25 or age < -.025:
            raise ValueError('stale_stereo_exposure')
        return p, r

    def _complete_observation(self):
        m = self.last_packet['measurement']
        return (m.get('status') == 'ok' and m.get('route') == 'joint_multicamera'
                and m.get('positive_depth') is True
                and all(len(m.get('used_ids', {}).get(eye, [])) >= 1
                        and 0 <= m.get('reprojection_error_px', {}).get(eye, float('inf')) <= 2.5
                        and 0 <= m.get('maximum_corner_error_px', {}).get(eye, float('inf')) <= 5
                        and m.get('minimum_marker_side_px', {}).get(eye, 0) >= 12
                        for eye in ('left', 'right')))
