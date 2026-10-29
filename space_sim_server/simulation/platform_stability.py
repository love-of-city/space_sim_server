"""Platform-only dwell, evaluated after navigation updates in simulation time."""
from copy import deepcopy


class PlatformStability:
    def __init__(self, duration_s=5.):
        self.duration_ns = round(duration_s*1e9)
        self.start_ns = self.last_ns = None
        self.reference = None
        self.failures = []

    def update(self, now_ns, reference, conditions):
        if self.last_ns is not None and now_ns < self.last_ns:
            raise ValueError('platform stability clock regressed')
        failures = [name for name, valid in conditions.items() if not valid]
        if self.last_ns is not None and now_ns-self.last_ns > 150_000_000:
            failures.append('platform updates interrupted')
        if reference != self.reference:
            self.start_ns = None
        self.reference, self.last_ns, self.failures = deepcopy(reference), int(now_ns), failures
        if failures:
            self.start_ns = None
        elif self.start_ns is None:
            self.start_ns = int(now_ns)
        return self.status(now_ns)

    def status(self, now_ns):
        elapsed = 0 if self.start_ns is None else max(0, self.last_ns-self.start_ns)
        return dict(platform_stable_duration_s=elapsed*1e-9,
                    platform_stable=self.start_ns is not None and elapsed >= self.duration_ns,
                    platform_evaluation_age_s=None if self.last_ns is None else (now_ns-self.last_ns)*1e-9,
                    platform_failures=list(self.failures))

    def snapshot(self):
        return deepcopy(dict(schema='space-sim-platform-stability/1', duration_ns=self.duration_ns,
                             start_ns=self.start_ns, last_ns=self.last_ns, reference=self.reference,
                             failures=self.failures))

    def restore(self, document):
        d = deepcopy(document)
        if (d.get('schema') != 'space-sim-platform-stability/1' or d.get('duration_ns') != self.duration_ns
                or type(d.get('last_ns')) is not int or d['last_ns'] < 0
                or d.get('start_ns') is not None and (type(d['start_ns']) is not int or not 0 <= d['start_ns'] <= d['last_ns'])
                or not isinstance(d.get('failures'), list)):
            raise ValueError('invalid platform stability record')
        self.start_ns, self.last_ns, self.reference, self.failures = d['start_ns'], d['last_ns'], d['reference'], d['failures']
