"""Lossless auxiliary stream for sensor samples on a different clock than RGB."""
import json
from pathlib import Path


class WrenchStreamWriter:
    relative_path = "high_frequency/wrist_wrench.jsonl"

    def __init__(self, root: Path):
        self.root = root
        self.first_observation_ns = None
        self.last_end_ns = None
        self.count = 0
        self.rate = None
        self.initialized = False
        self.identity = None

    def append(self, observation):
        rate = observation.wrist_wrench_rate_hz
        if self.initialized and rate != self.rate:
            raise ValueError("high-rate wrench availability changed within episode")
        self.initialized, self.rate = True, rate
        if rate is None:
            return
        stamp = int(observation.sim_time_ns)
        if self.first_observation_ns is None:
            self.first_observation_ns = stamp
        rows = []
        previous = self.last_end_ns
        identity = self.identity
        for sample in observation.wrist_wrench_samples:
            end = int(sample.interval_end_time_ns)
            if end < self.first_observation_ns:
                continue  # Trim pre-roll, never assign pre-episode force to its first frame.
            if previous is not None and end != previous + 5_000_000:
                raise ValueError("high-rate wrench stream has a gap or duplicate")
            metadata = sample.model_dump(exclude={"force_n", "torque_nm", "sample_time_ns", "interval_end_time_ns", "valid"})
            if identity is not None and metadata != identity:
                raise ValueError("high-rate wrench sensor identity changed")
            identity = metadata
            rows.append(dict(sample.model_dump(), sim_time_ns=str(end),
                             timestamp=(end-self.first_observation_ns)*1e-9))
            previous = end
        if stamp > 0 and previous != stamp:
            raise ValueError("high-rate wrench batch does not reach the recorded frame")
        if rows:
            path = self.root / self.relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.writelines(json.dumps(row, ensure_ascii=False)+"\n" for row in rows)
        self.last_end_ns, self.identity = previous, identity
        self.count += len(rows)

    def metadata(self):
        if self.rate is None:
            return None
        return dict(schema="space-arm-wrench-stream/1", path=self.relative_path,
                    rate_hz=self.rate, samples=self.count, sensor=self.identity,
                    first_observation_time_ns=str(self.first_observation_ns),
                    last_interval_end_time_ns=str(self.last_end_ns) if self.last_end_ns is not None else None,
                    time_semantics="sim_time_ns is completed physics interval end; sample_time_ns is native last-substep start; timestamp relative to first RGB/state frame",
                    components=["Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm"])
