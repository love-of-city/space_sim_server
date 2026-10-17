"""Shared receiver-solution error model for distant and proximity navigation.

PODRIX absolute bounds are 3D RMS (not per-axis). Differential performance and
spatial correlation are simulation assumptions; this is not a carrier decoder.
"""
from dataclasses import asdict, dataclass, fields
import math


@dataclass(frozen=True)
class GNSSConfig:
    absolute_position_sigma_m: float = 1.0 / math.sqrt(3)
    absolute_velocity_sigma_m_s: float = 0.002 / math.sqrt(3)
    relative_position_sigma_m: float = 0.03
    relative_velocity_sigma_m_s: float = 0.0015
    correlation_length_m: float = 1_000_000.0
    sample_rate_hz: float = 10.0
    delay_s: float = 0.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key} must be a finite number")
            if value < 0 or (key != "delay_s" and value == 0):
                raise ValueError(f"{key} must be positive (delay may be zero)")
        if self.sample_rate_hz not in (1, 2, 5, 10, 20) or self.delay_s > .25:
            raise ValueError("GNSS sampling must divide 20 Hz and transport delay must be at most 0.25 s")
        for component in ("position", "velocity"):
            suffix = "m" if component == "position" else "m_s"
            absolute = getattr(self, f"absolute_{component}_sigma_{suffix}")
            relative = getattr(self, f"relative_{component}_sigma_{suffix}")
            if relative > math.sqrt(2) * absolute:
                raise ValueError("differential residual exceeds the two receivers' variance budget")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict) or set(values) - {field.name for field in fields(cls)}:
            raise ValueError("GNSS configuration must be an object with known fields")
        return cls(**values)
