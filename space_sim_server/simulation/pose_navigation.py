"""Two-spacecraft error-state EKF driven by GNSS, gyros and image poses.

Nominal states are assembly COM position/velocity in J2000, body-to-J2000
rotation, and body gyro bias. Each spacecraft has [dp,dv,dtheta_N,db_B]
errors. Full 24x24 covariance preserves common GNSS errors. Delayed updates
rewind and replay sensor/control events; truth is not an input to this module.
"""
from __future__ import annotations

from copy import deepcopy
from functools import lru_cache
import hashlib
import json
import math

import numpy as np

from space_arm_platform.visual_navigation import (
    covariance, image_clock, POSE_ERROR_ORDER, stereo_quality,
    FIXED_STEREO_PROFILE, NEAR23_BODY_PROFILE,
)
from simulation.navigation_gravity import ReplayGravity

_I3 = np.eye(3)


@lru_cache(maxsize=64)
def _propagation_matrices(dt, acceleration_sigma, gyro_bias_walk):
    """Owned templates for constant process terms on repeated replay steps."""
    f, q = np.eye(24), np.zeros((24, 24))
    g = np.vstack([.5*dt**2*_I3, dt*_I3])
    translation_noise = g@g.T*acceleration_sigma**2
    for j in (0, 12):
        f[j:j+3, j+3:j+6] = _I3*dt
        q[j:j+6, j:j+6] = translation_noise
        q[j+9:j+12, j+9:j+12] = _I3*gyro_bias_walk**2*dt
    f.setflags(write=False)
    q.setflags(write=False)
    return f, q


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp(v):
    v = np.asarray(v, float)
    angle = np.linalg.norm(v)
    k = skew(v)
    if angle < 1e-6:
        return np.eye(3) + k + .5*k@k
    return np.eye(3) + math.sin(angle)/angle*k + (1-math.cos(angle))/angle**2*k@k


def log(r):
    angle = math.acos(float(np.clip((np.trace(r)-1)/2, -1, 1)))
    vee = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    if angle < 1e-6:
        return .5*vee
    if math.pi-angle < 1e-5:
        _, vectors = np.linalg.eigh((r+r.T)/2)
        axis = vectors[:, -1]
        return axis*angle * (-1 if axis@vee < 0 else 1)
    return angle/(2*math.sin(angle))*vee


def rotation(value):
    r = np.asarray(value, float)
    if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T@r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1):
        raise ValueError('expected proper body-to-inertial rotation')
    return r.copy()


def _copy_state(state):
    # State is deliberately flat: arrays, two scalar lists, and an integer
    # epoch. Replay creates hundreds of these per second; deepcopy's recursive
    # dispatch adds no isolation beyond copying these owned containers.
    return {key: value.copy() if isinstance(value, (np.ndarray, list)) else value
            for key, value in state.items()}


class PoseNavigation:
    lag_ns = 1_000_000_000

    def __init__(self, position, velocity, rotations, initial_covariance, *, time_ns=0,
                 gravity=None, camera_rotation=None, camera_position=None,
                 service_com=None, target_com=None, gyro_white_sigma=1e-4,
                 gyro_bias_walk=1e-6, acceleration_sigma=.003, vision_identity=None,
                 visual_profile=FIXED_STEREO_PROFILE):
        if visual_profile not in (FIXED_STEREO_PROFILE, NEAR23_BODY_PROFILE):
            raise ValueError('unknown visual measurement profile')
        self.visual_profile = visual_profile
        self.gravity = None if gravity is None else ReplayGravity(gravity)
        self.camera_rotation = rotation(np.eye(3) if camera_rotation is None else camera_rotation)
        self.camera_position = np.asarray(np.zeros(3) if camera_position is None else camera_position, float).copy()
        if self.camera_position.shape != (3,) or not np.isfinite(self.camera_position).all():
            raise ValueError('invalid camera position')
        self.camera_com_offset = self.camera_position - np.asarray(np.zeros(3) if service_com is None else service_com, float)
        self.target_com = np.asarray(np.zeros(3) if target_com is None else target_com, float)
        self.gyro_white_sigma, self.gyro_bias_walk = float(gyro_white_sigma), float(gyro_bias_walk)
        self.acceleration_sigma = float(acceleration_sigma)
        p, v = np.asarray(position, float), np.asarray(velocity, float)
        if p.shape != (2, 3) or v.shape != (2, 3) or not np.isfinite(np.r_[p, v]).all():
            raise ValueError('explicit two-body position/velocity prior required')
        r = np.array([rotation(x) for x in rotations])
        if r.shape != (2, 3, 3):
            raise ValueError('two explicit attitude priors required')
        self.state = dict(time_ns=int(time_ns), p=p.copy(), v=v.copy(), r=r,
                          b=np.zeros((2, 3)), covariance=covariance(initial_covariance, 24).copy(),
                          rates=np.zeros((2, 3)), acceleration=np.zeros((2, 3)),
                          camera_com_offset=self.camera_com_offset.copy(), target_com=self.target_com.copy(),
                          com_velocity_body=np.zeros((2, 3)),
                          gyro_time=[None, None], gyro_valid=[False, False])
        self.base = _copy_state(self.state)
        self.base_committed = False
        self.events, self.snapshots = [], []
        self._seen = set()
        self.accepted = {}
        self.last_reason = 'initial prior'
        self.vision_identity = tuple(vision_identity) if vision_identity is not None else None
        self.last_visual_epoch = None
        self.handoff_visual_packets = set()

    @property
    def time_ns(self):
        return self.state['time_ns']

    def bind_vision_session(self, mission_id, session_id, calibration_id, *, pending_visual_results=()):
        """Rotate the image stream after restoring a continuous mission.

        Keep the posterior and delayed history, including old image corrections.
        Only explicitly inherited in-flight packets retain the retired identity.
        Other late packets from the retired renderer are rejected. Changing calibration or
        mission requires constructing a new estimator with an explicit prior.
        """
        identity = (mission_id, session_id, calibration_id)
        if any(not isinstance(v, str) or not v for v in identity):
            raise ValueError('nonempty vision mission/session/calibration required')
        if self.vision_identity is not None and (identity[0], identity[2]) != (self.vision_identity[0], self.vision_identity[2]):
            raise ValueError('vision mission or calibration changed across handoff')
        inherited = set()
        for packet in pending_visual_results:
            packet_identity = tuple(packet.get(k) for k in ('mission_id', 'session_id', 'calibration_id'))
            if (packet_identity != self.vision_identity
                    or not 0 <= int(packet['sim_time_ns']) <= self.time_ns):
                raise ValueError('inherited image identity or exposure changed')
            inherited.add(self.visual_packet_digest(packet))
        self.handoff_visual_packets.update(inherited)
        self.vision_identity = identity

    @staticmethod
    def visual_packet_digest(packet):
        return hashlib.sha256(json.dumps(packet, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    def _propagate(self, state, now_ns):
        remaining = (now_ns-state['time_ns'])*1e-9
        if remaining < -1e-12:
            raise ValueError('navigation cannot propagate backwards')
        while remaining > 1e-10:
            dt = min(.02, remaining)
            f0, q0 = _propagation_matrices(dt, self.acceleration_sigma, self.gyro_bias_walk)
            f, q = f0.copy(), q0.copy()
            for body in range(2):
                j = body*12
                p, v, r = state['p'][body], state['v'][body], state['r'][body]
                epoch = state['time_ns']*1e-9
                a = state['acceleration'][body].copy()
                gradient = np.zeros((3, 3))
                if self.gravity is not None:
                    a += self.gravity.acceleration(p, epoch)
                    gradient = self.gravity.gradient(p, epoch)
                pnew = p + v*dt + .5*a*dt**2
                anew = state['acceleration'][body].copy()
                if self.gravity is not None:
                    anew += self.gravity.acceleration(pnew, epoch+dt)
                state['p'][body], state['v'][body] = pnew, v+.5*(a+anew)*dt
                state['r'][body] = r @ exp((state['rates'][body]-state['b'][body])*dt)
                f[j:j+3, j:j+3] += .5*gradient*dt**2
                f[j+3:j+6, j:j+3] = gradient*dt
                f[j+6:j+9, j+9:j+12] = -r*dt
                last = state['gyro_time'][body]
                step_end_ns = state['time_ns']+round(dt*1e9)
                fresh = last is not None and step_end_ns-last <= 60_000_000 and state['gyro_valid'][body]
                # Missing/saturated rates cannot be treated as precise held
                # observations; unobservable attitude directions keep growing.
                rate_sigma = self.gyro_white_sigma if fresh else .5
                q[j+6:j+9, j+6:j+9] = _I3*rate_sigma**2*.02*dt
            state['covariance'] = f@state['covariance']@f.T+q
            state['camera_com_offset'] -= state['com_velocity_body'][0]*dt
            state['target_com'] += state['com_velocity_body'][1]*dt
            state['time_ns'] += int(round(dt*1e9))
            remaining = (now_ns-state['time_ns'])*1e-9
        state['time_ns'] = int(now_ns)

    @staticmethod
    def _correct(state, innovation, h, noise, gate):
        p = state['covariance']
        s = h@p@h.T + noise
        solved = np.linalg.solve(s, np.column_stack((innovation, h@p)))
        nis = float(innovation @ solved[:, 0])
        if not np.isfinite(nis) or nis > gate:
            return False, nis
        k = solved[:, 1:].T
        delta = k@innovation
        left = np.eye(24)-k@h
        posterior = left@p@left.T+k@noise@k.T
        reset = np.eye(24)
        for body in range(2):
            j = body*12
            state['p'][body] += delta[j:j+3]
            state['v'][body] += delta[j+3:j+6]
            state['r'][body] = exp(delta[j+6:j+9])@state['r'][body]
            state['b'][body] += delta[j+9:j+12]
            reset[j+6:j+9, j+6:j+9] += .5*skew(delta[j+6:j+9])
        p = reset@posterior@reset.T
        state['covariance'] = .5*(p+p.T)
        return True, nis

    def predict_pose(self, state=None, *, frame='left_optical'):
        state = self.state if state is None else state
        service, target = state['r']
        if frame == 'service_body':
            # The image bridge already used the exposure-time arm FK. This
            # measurement is between BODY ORIGINS; the state remains COM.
            # Encoder geometry in replay supplies the current assembly levers.
            camera_rotation = np.eye(3)
            camera_com_offset = state['camera_com_offset']-self.camera_position
        elif frame == 'left_optical':
            camera_rotation = self.camera_rotation
            camera_com_offset = state['camera_com_offset']
        else:
            raise ValueError('unsupported visual measurement frame')
        to_camera = camera_rotation.T@service.T
        offset = target@state['target_com']
        dp = state['p'][1]-state['p'][0]
        t = to_camera@(dp-offset)-camera_rotation.T@camera_com_offset
        r = to_camera@target
        h = np.zeros((6, 24))
        h[:3, :3], h[:3, 12:15] = -to_camera, to_camera
        h[:3, 6:9] = to_camera@skew(dp-offset)
        h[:3, 18:21] = to_camera@skew(offset)
        h[3:, 6:9], h[3:, 18:21] = -to_camera, to_camera
        return t, r, h

    def _apply(self, state, event):
        kind, data = event['kind'], event['data']
        if kind == 'geometry':
            for key in ('camera_com_offset', 'target_com', 'com_velocity_body'):
                state[key] = np.array(data[key])
            return True
        if kind == 'gyro':
            body = data['body']
            state['gyro_time'][body] = event['time_ns']
            state['gyro_valid'][body] = data['valid']
            if data['valid']:
                state['rates'][body] = np.array(data['rate'])
            return True
        if kind == 'command':
            state['acceleration'] = np.array(data['acceleration'])
            return True
        if kind == 'gnss':
            h = np.zeros((12, 24))
            h[:6, :6] = h[6:, 12:18] = np.eye(6)
            prediction = np.concatenate([np.r_[p, v] for p, v in zip(state['p'], state['v'])])
            return self._correct(state, np.array(data['measurement']).reshape(12)-prediction,
                                 h, np.array(data['covariance']), 50.)[0]
        t, r, h = self.predict_pose(state, frame=data.get('frame', 'left_optical'))
        innovation = np.r_[np.array(data['translation'])-t, log(np.array(data['rotation'])@r.T)]
        return self._correct(state, innovation, h, np.array(data['covariance']), 30.)[0]

    def advance(self, now_ns):
        self._propagate(self.state, int(now_ns))
        # Prune only at stored events so delayed replay has an exact base.
        while self.events and self.events[0]['time_ns'] < self.time_ns-self.lag_ns:
            self.base = self.snapshots.pop(0)
            self.base_committed = True
            old = self.events.pop(0)
            self._seen.discard(old['id'])
            self.accepted.pop(old['id'], None)

    def _insert(self, event):
        if event['id'] in self._seen:
            self.last_reason = 'duplicate sensor event'
            return False
        if (event['time_ns'] < self.base['time_ns'] or event['time_ns'] > self.time_ns
                or (self.base_committed and event['time_ns'] == self.base['time_ns'])):
            self.last_reason = 'sensor epoch outside replay history'
            return False
        self._seen.add(event['id'])
        self.events.append(deepcopy(event))
        self.events.sort(key=lambda e: (e['time_ns'], e['priority'], e['id']))
        index = next(i for i, e in enumerate(self.events) if e['id'] == event['id'])
        # Most sensor/control events arrive at the current epoch in order.
        # advance() has already propagated this state; replaying that interval
        # again duplicates native gravity queries on every 100 Hz command.
        in_order_now = index == len(self.events)-1 and event['time_ns'] == self.time_ns
        if in_order_now:
            accepted = self._apply(self.state, self.events[-1])
            self.accepted[event['id']] = accepted
            self.snapshots.append(_copy_state(self.state))
            self.last_reason = 'accepted' if accepted else 'innovation rejected'
            return accepted
        state = _copy_state(self.base if index == 0 else self.snapshots[index-1])
        self.snapshots = self.snapshots[:index]
        current = self.time_ns
        for e in self.events[index:]:
            self._propagate(state, e['time_ns'])
            self.accepted[e['id']] = self._apply(state, e)
            self.snapshots.append(_copy_state(state))
        self._propagate(state, current)
        self.state = state
        accepted = self.accepted[event['id']]
        self.last_reason = 'accepted' if accepted else 'innovation rejected'
        return accepted

    def gyro(self, body, observation):
        if body not in (0, 1):
            raise ValueError('unknown gyro body')
        rate = np.asarray(observation['angular_velocity_body_rad_s'], float)
        if rate.shape != (3,) or not np.isfinite(rate).all():
            raise ValueError('invalid gyro rate')
        epoch = int(observation['sample_time_ns'])
        return self._insert(dict(id=f'gyro/{body}/{epoch}', kind='gyro', priority=0, time_ns=epoch,
            data=dict(body=body, rate=rate.tolist(), valid=bool(observation['valid']) and not any(observation['saturated_axes']))))

    def command(self, acceleration, *, time_ns=None):
        a = np.asarray(acceleration, float)
        if a.shape != (2, 3) or not np.isfinite(a).all():
            raise ValueError('invalid commanded accelerations')
        epoch = self.time_ns if time_ns is None else int(time_ns)
        return self._insert(dict(id=f'command/{epoch}', kind='command', priority=1, time_ns=epoch,
                                 data=dict(acceleration=a.tolist())))

    def gnss(self, measurement, measurement_covariance, exposure_ns, packet_id):
        z = np.asarray(measurement, float)
        if z.shape != (2, 6) or not np.isfinite(z).all():
            raise ValueError('invalid paired GNSS observation')
        c = covariance(measurement_covariance, 12)
        return self._insert(dict(id='gnss/'+str(packet_id), kind='gnss', priority=2, time_ns=int(exposure_ns),
                                 data=dict(measurement=z.tolist(), covariance=c.tolist())))

    def encoder_geometry(self, now_ns, camera_com_offset, target_com, com_velocity_body):
        data = {}
        for key, value, shape in (('camera_com_offset', camera_com_offset, (3,)),
                                  ('target_com', target_com, (3,)), ('com_velocity_body', com_velocity_body, (2, 3))):
            a = np.asarray(value, float)
            if a.shape != shape or not np.isfinite(a).all():
                raise ValueError('invalid encoder geometry')
            data[key] = a.tolist()
        return self._insert(dict(id=f'geometry/{now_ns}', kind='geometry', priority=-1,
                                 time_ns=int(now_ns), data=data))

    def visual(self, result, *, received_wall_time_ns=None):
        if result.get('status') != 'ok' or result.get('ambiguous', True):
            self.last_reason = result.get('reason', 'invalid image solution')
            return False

        try:
            identity = tuple(result[k] for k in ('mission_id', 'session_id', 'calibration_id'))
            if any(not isinstance(k, str) or not k for k in identity):
                raise ValueError('missing visual identity')
            if self.vision_identity is not None and identity != self.vision_identity:
                digest = self.visual_packet_digest(result)
                if digest not in self.handoff_visual_packets:
                    raise ValueError('visual task/session/calibration mismatch')
                self.handoff_visual_packets.remove(digest)
            epoch, age, _, _ = image_clock(result, self.time_ns, received_wall_time_ns)
            if age > 350_000_000:
                raise ValueError('visual exposure outside freshness interval')
            if self.last_visual_epoch is not None and epoch <= self.last_visual_epoch:
                raise ValueError('duplicate or out-of-order visual exposure')
            stereo_quality(result, self.visual_profile)
            body_pose = self.visual_profile == NEAR23_BODY_PROFILE
            frame = 'service_body' if body_pose else 'left_optical'
            if body_pose:
                geometry = result['exposure_geometry']
                if int(geometry['sample_time_ns']) != epoch or geometry['calibration_id'] != identity[2]:
                    raise ValueError('image/encoder epoch or calibration mismatch')
            pose = result['pose_target_to_service' if body_pose else 'pose_target_to_left']
            t, r = np.asarray(pose['t_m'], float), rotation(pose['R'])
            c = result['pose_covariance']
            if (c['frame'] != frame or c['rotation_error'] != 'left_multiplicative'
                    or c.get('order') != POSE_ERROR_ORDER):
                raise ValueError('unsupported pose covariance convention')
            noise = covariance(c['matrix'], 6)
            if t.shape != (3,) or not np.isfinite(t).all():
                raise ValueError('invalid image position')
            self.last_visual_epoch = epoch
            return self._insert(dict(id=self.visual_event_id(result), kind='visual', priority=3,
                time_ns=epoch, data=dict(translation=t.tolist(), rotation=r.tolist(), covariance=noise.tolist(), frame=frame)))
        except (ValueError, KeyError, TypeError, OverflowError, np.linalg.LinAlgError) as exc:
            self.last_reason = str(exc)
            return False

    @staticmethod
    def visual_event_id(result):
        return 'visual/'+'/'.join(str(result[k]) for k in ('mission_id', 'session_id', 'source_frame_id', 'calibration_id'))

    def relative_covariances(self):
        h = np.zeros((6, 24))
        h[:3, :3], h[:3, 12:15] = -np.eye(3), np.eye(3)
        h[3:, 6:9], h[3:, 18:21] = -np.eye(3), np.eye(3)
        c = h@self.state['covariance']@h.T
        return c[:3, :3], c[3:, 3:]

    def gyro_health(self, now_ns):
        return [bool(valid and epoch is not None and 0 <= now_ns-epoch <= 60_000_000)
                for valid, epoch in zip(self.state['gyro_valid'], self.state['gyro_time'])]

    def snapshot(self):
        def encode(s):
            return {k: v.tolist() if isinstance(v, np.ndarray) else deepcopy(v) for k, v in s.items()}
        return dict(schema='space-sim-pose-navigation/1', state=encode(self.state), base=encode(self.base), base_committed=self.base_committed,
                    events=deepcopy(self.events), snapshots=[encode(s) for s in self.snapshots],
                    accepted=deepcopy(self.accepted), configuration=self.configuration(),
                    last_visual_epoch=self.last_visual_epoch,
                    handoff_visual_packets=sorted(self.handoff_visual_packets))

    def configuration(self):
        result = dict(camera_rotation=self.camera_rotation.tolist(), camera_com_offset=self.camera_com_offset.tolist(),
                    target_com=self.target_com.tolist(), gyro_white_sigma=self.gyro_white_sigma,
                    gyro_bias_walk=self.gyro_bias_walk, acceleration_sigma=self.acceleration_sigma,
                    vision_identity=None if self.vision_identity is None else list(self.vision_identity))
        if self.visual_profile != FIXED_STEREO_PROFILE:
            result.update(visual_profile=self.visual_profile, camera_position=self.camera_position.tolist())
        return result

    def restore(self, document):
        if document.get('schema') != 'space-sim-pose-navigation/1' or document.get('configuration') != self.configuration():
            raise ValueError('navigation restart configuration mismatch')
        def decode(s):
            out = deepcopy(s)
            shapes = dict(p=(2, 3), v=(2, 3), r=(2, 3, 3), b=(2, 3), covariance=(24, 24),
                          rates=(2, 3), acceleration=(2, 3), camera_com_offset=(3,), target_com=(3,), com_velocity_body=(2, 3))
            for key, shape in shapes.items():
                out[key] = np.array(s.get(key, self.state.get(key)), float)
                if out[key].shape != shape or not np.isfinite(out[key]).all():
                    raise ValueError('invalid navigation restart state')
            if type(out['time_ns']) is not int or out['time_ns'] < 0:
                raise ValueError('invalid navigation restart epoch')
            if len(out['gyro_time']) != 2 or len(out['gyro_valid']) != 2:
                raise ValueError('invalid gyro restart history')
            if any(t is not None and (type(t) is not int or not 0 <= t <= out['time_ns']) for t in out['gyro_time']):
                raise ValueError('invalid gyro restart epoch')
            covariance(out['covariance'], 24)
            for r in out['r']:
                rotation(r)
            return out
        state, base = decode(document['state']), decode(document['base'])
        snapshots = [decode(s) for s in document['snapshots']]
        events = deepcopy(document['events'])
        if len(events) != len(snapshots) or base['time_ns'] > state['time_ns']:
            raise ValueError('invalid replay history')
        order = [(e['time_ns'], e['priority'], e['id']) for e in events]
        if (order != sorted(order) or len({e['id'] for e in events}) != len(events)
                or any(not base['time_ns'] <= e['time_ns'] <= state['time_ns']
                       or e['time_ns'] != s['time_ns'] for e, s in zip(events, snapshots))):
            raise ValueError('invalid replay event ordering')
        last_visual = document['last_visual_epoch']
        if last_visual is not None and (type(last_visual) is not int or not 0 <= last_visual <= state['time_ns']):
            raise ValueError('invalid visual restart epoch')
        self.state, self.base, self.snapshots, self.events = state, base, snapshots, events
        self.base_committed = bool(document['base_committed'])
        self._seen, self.accepted = {e['id'] for e in events}, deepcopy(document['accepted'])
        self.last_visual_epoch = last_visual
        digests = document.get('handoff_visual_packets', [])
        if (not isinstance(digests, list) or any(not isinstance(d, str) or len(d) != 64
                or any(c not in '0123456789abcdef' for c in d) for d in digests)):
            raise ValueError('invalid inherited image packet binding')
        self.handoff_visual_packets = set(digests)
