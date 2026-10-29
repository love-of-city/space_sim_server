"""Fixed simulation delivery; all waiting belongs to the outer runner.

The native callbacks only drain staged packets. Host speed never changes an
exposure, its delivery epoch or the order presented to navigation.
"""
from collections import deque
from copy import deepcopy
import math
import time


class FixedStereoDelivery:
    def __init__(self, delay_ns=200_000_000, timeout_s=15., *, clock=time.monotonic, pause=time.sleep):
        if type(delay_ns) is not int or delay_ns < 0 or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('invalid fixed delivery configuration')
        self.delay_ns, self.timeout_s = delay_ns, timeout_s
        self.clock, self.pause = clock, pause
        self.pending = deque()
        self.completed = {}
        self.staged = deque()
        self.healthy, self.failure = True, ''
        self.events = []
        self.last_exposure = None
        self.wait_s = 0.
        self.received = set()

    def _receive(self, packets, now_ns):
        expected = {(p['exposure_ns'], p['frame_id']) for p in self.pending}
        for packet in packets:
            if packet.get('transport_rejection'):
                self.staged.append((now_ns, packet))
                continue
            key = (int(packet['sim_time_ns']), str(packet['source_frame_id']))
            if key in self.received or key not in expected:
                reason = 'duplicate delivery result' if key in self.received else 'unexpected delivery result'
                self.staged.append((now_ns, dict(packet, status='rejected', reason=reason)))
            else:
                self.completed[key] = packet
                self.received.add(key)

    def expect(self, exposure_ns, frame_id):
        if self.last_exposure is not None and exposure_ns <= self.last_exposure:
            raise ValueError('stereo exposures must increase')
        self.last_exposure = int(exposure_ns)
        self.pending.append(dict(exposure_ns=int(exposure_ns), frame_id=str(frame_id),
                                 due_ns=int(exposure_ns)+self.delay_ns, started=self.clock()))

    def fail(self, reason, now_ns):
        if self.healthy:
            self.events.append(dict(kind='process_fault', time_ns=now_ns, reason=str(reason)))
        self.healthy, self.failure = False, str(reason)

    def prepare(self, next_ns, poll, *, alive=lambda: True):
        """Call before ExecuteSimulation can reach a delivery epoch."""
        self._receive(poll(), next_ns)
        if not alive():
            self.fail('vision process exited', next_ns)
        while self.pending and self.pending[0]['due_ns'] <= next_ns:
            expected = self.pending[0]
            key = (expected['exposure_ns'], expected['frame_id'])
            started_wait = self.clock()
            while key not in self.completed and self.healthy:
                if not alive():
                    self.fail('vision process exited', expected['due_ns'])
                    break
                if self.clock()-expected['started'] >= self.timeout_s:
                    self.fail('vision task wall timeout', expected['due_ns'])
                    break
                self.pause(.005)
                self._receive(poll(), expected['due_ns'])
            waited = self.clock()-started_wait
            self.wait_s += waited
            self.pending.popleft()
            packet = self.completed.pop(key, None)
            # Failure health is independent; do not invent an over-age image.
            if packet is not None:
                if packet.get('process_healthy') is False:
                    self.fail(packet.get('reason', 'vision worker failed'), expected['due_ns'])
                self.staged.append((expected['due_ns'], packet))
            self.events.append(dict(kind='delivery', exposure_ns=expected['exposure_ns'],
                due_ns=expected['due_ns'], waiting_wall_s=waited,
                task_wall_s=self.clock()-expected['started'], result_available=packet is not None,
                backlog=len(self.pending), process_healthy=self.healthy))

    def take_due(self, now_ns):
        self.staged = deque(sorted(self.staged, key=lambda p: p[0]))
        packets = []
        while self.staged and self.staged[0][0] <= now_ns:
            _, packet = self.staged.popleft()
            packets.append(packet)
        return packets

    def prepare_diagnostic(self, next_ns, poll, *, alive=lambda: True):
        """Fixed-deadline diagnostics that never delay ideal-state control.

        Results absent at their 0.20 s delivery deadline are recorded missing;
        later arrivals remain diagnostic rejections with their original payload.
        The image-navigation runner retains prepare() and its strict scheduler.
        """
        self._receive(poll(), next_ns)
        if not alive():
            self.fail('vision process exited', next_ns)
        while self.pending and self.pending[0]['due_ns'] <= next_ns:
            expected = self.pending.popleft()
            key = (expected['exposure_ns'], expected['frame_id'])
            packet = self.completed.pop(key, None)
            if packet is not None:
                self.staged.append((expected['due_ns'], packet))
            self.events.append(dict(kind='diagnostic_delivery', exposure_ns=expected['exposure_ns'],
                due_ns=expected['due_ns'], waiting_wall_s=0., result_available=packet is not None,
                backlog=len(self.pending), process_healthy=self.healthy))

    def status(self):
        return dict(healthy=self.healthy, failure=self.failure, pending=len(self.pending),
                    waiting_wall_s=self.wait_s, fixed_delay_s=self.delay_ns*1e-9,
                    task_timeout_wall_s=self.timeout_s)

    def snapshot(self):
        pending = [dict(p, age_wall_s=self.clock()-p['started']) for p in self.pending]
        for packet in pending:
            packet.pop('started')
        return deepcopy(dict(schema='space-sim-fixed-stereo-delivery/1', delay_ns=self.delay_ns,
            timeout_s=self.timeout_s, pending=pending, staged=list(self.staged),
            completed=[dict(exposure_ns=k[0], frame_id=k[1], result=v) for k, v in self.completed.items()],
            last_exposure=self.last_exposure, healthy=self.healthy, failure=self.failure))

    def restore(self, document):
        d = deepcopy(document)
        if (d.get('schema') != 'space-sim-fixed-stereo-delivery/1' or d.get('delay_ns') != self.delay_ns
                or d.get('timeout_s') != self.timeout_s):
            raise ValueError('fixed sensor delivery configuration changed')
        pending = d['pending']
        if (any(type(p['exposure_ns']) is not int or p['exposure_ns'] < 0
                or type(p['due_ns']) is not int or p['due_ns'] != p['exposure_ns']+self.delay_ns
                or not isinstance(p['frame_id'], str) or not p['frame_id']
                or not math.isfinite(p['age_wall_s']) or p['age_wall_s'] < 0 for p in pending)
                or any(a['exposure_ns'] >= b['exposure_ns'] for a, b in zip(pending, pending[1:]))
                or type(d['healthy']) is not bool or not isinstance(d['failure'], str)
                or d['last_exposure'] is not None and (type(d['last_exposure']) is not int or d['last_exposure'] < 0)
                or pending and pending[-1]['exposure_ns'] != d['last_exposure']):
            raise ValueError('fixed sensor delivery epoch changed')
        expected = {(p['exposure_ns'], p['frame_id']) for p in pending}
        completed_keys = [(p['exposure_ns'], p['frame_id']) for p in d['completed']]
        if (len(set(completed_keys)) != len(completed_keys) or not set(completed_keys) <= expected
                or any(not isinstance(p['result'], dict) or int(p['result']['sim_time_ns']) != p['exposure_ns']
                       or str(p['result']['source_frame_id']) != p['frame_id'] for p in d['completed'])
                or any(type(due) is not int or due < 0 or not isinstance(result, dict) for due, result in d['staged'])):
            raise ValueError('fixed sensor delivery packet identity changed')
        self.pending = deque(dict(p, started=self.clock()-p['age_wall_s']) for p in pending)
        for packet in self.pending:
            packet.pop('age_wall_s')
        self.completed = {(p['exposure_ns'], p['frame_id']): p['result'] for p in d['completed']}
        self.received = set(self.completed)
        self.staged = deque(d['staged'])
        self.last_exposure, self.healthy, self.failure = d['last_exposure'], d['healthy'], d['failure']
