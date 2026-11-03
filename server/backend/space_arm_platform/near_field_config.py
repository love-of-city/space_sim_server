"""Public reproducible near-field controls, separate from distant-flight GNSS."""
from dataclasses import asdict, dataclass, fields
import math


@dataclass(frozen=True)
class NearFieldOptions:
    seed: int = 7
    vision_loss_phase: str = ""
    communication_loss_s: float | None = None

    def __post_init__(self):
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2**32)")
        if self.vision_loss_phase not in ("", "VisualApproach", "FinalApproach", "Grasp"):
            raise ValueError("unsupported vision loss phase")
        value = self.communication_loss_s
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 10000):
            raise ValueError("communication_loss_s must be finite in (0, 10000]")

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict) or set(values) - {f.name for f in fields(cls)}:
            raise ValueError("unknown near-field options")
        return cls(**values)

    def to_dict(self):
        return asdict(self)
