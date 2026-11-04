"""Encoder-supervised joint reference paths; never assigns physical coordinates."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


class JointPreparation:
    def __init__(self, document, model):
        self.document = deepcopy(document)
        self.digest = hashlib.sha256(json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if (document.get('schema') != 'space-sim-capture26-preparation/1'
                or document.get('model_sha256') != hashlib.sha256(Path(model).read_bytes()).hexdigest()):
            raise ValueError('capture26 preparation model binding failed')
        self.names = [f'joint{i}' for i in range(1, 7)]+[f'bottom_joint{i}' for i in range(1, 7)]
        joints = {j.get('name'): j for j in ET.parse(model).iter('joint')}
        bounds = np.array([np.fromstring(joints[name].get('range'), sep=' ') for name in self.names])
        self.points = np.array(document['waypoints_rad'], float)
        self.speed, self.acceleration = float(document['maximum_rate_rad_s']), float(document['maximum_acceleration_rad_s2'])
        if (self.points.ndim != 2 or self.points.shape[1] != 12 or len(self.points) < 2
                or not np.isfinite(self.points).all() or np.any(self.points < bounds[:, 0])
                or np.any(self.points > bounds[:, 1]) or not 0 < self.speed <= .12
                or not 0 < self.acceleration <= .04):
            raise ValueError('capture26 preparation violates model limits')
        self.segment, self.elapsed_s, self.last_ns, self.settled_ns = 0, 0., None, None
        self.complete, self.failed = False, ''
        self.reference = self.points[0].copy()

    def update(self, now_ns, measured, rates):
        measured, rates = np.asarray(measured, float), np.asarray(rates, float)
        if (measured.shape != (12,) or rates.shape != (12,) or not np.isfinite(np.r_[measured, rates]).all()
                or self.last_ns is not None and now_ns < self.last_ns):
            raise ValueError('invalid preparation encoders or clock')
        dt = 0. if self.last_ns is None else (now_ns-self.last_ns)*1e-9
        self.last_ns = int(now_ns)
        if self.failed or self.complete:
            return self.reference.copy()
        if np.max(np.abs(measured-self.reference)) > .05:
            self.failed = 'preparation encoder tracking error exceeded'
            return self.reference.copy()
        start, end = self.points[self.segment:self.segment+2]
        travel = float(np.max(np.abs(end-start)))
        duration = max(.5, 1.875*travel/self.speed, np.sqrt(5.774*travel/self.acceleration))
        # Keep the servo reference within 0.01 rad of encoders. A blocked arm
        # cannot integrate its command through an obstacle or skip a waypoint.
        if np.max(np.abs(measured-self.reference)) < .01:
            self.elapsed_s = min(duration, self.elapsed_s+dt)
        u = self.elapsed_s/duration
        self.reference = start+(end-start)*(10*u**3-15*u**4+6*u**5)
        settled = (self.elapsed_s == duration and np.max(np.abs(measured-end)) < .003
                   and np.max(np.abs(rates)) < .003)
        if not settled:
            self.settled_ns = None
        elif self.settled_ns is None:
            self.settled_ns = int(now_ns)
        elif now_ns-self.settled_ns >= 500_000_000:
            if self.segment == len(self.points)-2:
                self.complete = True
            else:
                self.segment += 1
                self.elapsed_s, self.settled_ns = 0., None
        return self.reference.copy()

    def status(self):
        return dict(plan_sha256=self.digest, segment=self.segment, complete=self.complete, failure=self.failed,
                    reference_rad=self.reference.tolist())

    def snapshot(self):
        return deepcopy(dict(schema='space-sim-capture26-preparation-state/1', **self.status(),
                             elapsed_s=self.elapsed_s, last_ns=self.last_ns, settled_ns=self.settled_ns))

    def restore(self, document):
        reference = np.asarray(document.get('reference_rad'), float)
        segment = document.get('segment')
        elapsed, last, settled = (document.get(k) for k in ('elapsed_s', 'last_ns', 'settled_ns'))
        if (document.get('schema') != 'space-sim-capture26-preparation-state/1'
                or document.get('plan_sha256') != self.digest or type(segment) is not int
                or not 0 <= segment < len(self.points)-1
                or type(document.get('complete')) is not bool or not isinstance(document.get('failure'), str)
                or reference.shape != (12,) or not np.isfinite(reference).all()
                or type(elapsed) not in (int, float) or not np.isfinite(elapsed) or elapsed < 0
                or last is not None and (type(last) is not int or last < 0)
                or settled is not None and (type(settled) is not int or last is None or not 0 <= settled <= last)
                or last is None and (segment != 0 or elapsed != 0 or document['complete'])):
            raise ValueError('capture26 preparation state changed')
        start, end = self.points[segment:segment+2]
        travel = float(np.max(np.abs(end-start)))
        duration = max(.5, 1.875*travel/self.speed, np.sqrt(5.774*travel/self.acceleration))
        u = elapsed/duration
        expected = start+(end-start)*(10*u**3-15*u**4+6*u**5)
        if (elapsed > duration or not np.allclose(reference, expected, atol=1e-12, rtol=0)
                or document['complete'] and (segment != len(self.points)-2 or elapsed != duration
                    or settled is None or last-settled < 500_000_000 or document['failure'])):
            raise ValueError('capture26 preparation progress or reference changed')
        self.segment, self.complete, self.failed = document['segment'], document['complete'], document['failure']
        self.reference = reference.copy()
        self.elapsed_s, self.last_ns, self.settled_ns = document['elapsed_s'], document['last_ns'], document['settled_ns']
