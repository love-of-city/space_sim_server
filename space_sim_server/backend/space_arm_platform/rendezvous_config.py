"""Validated, reproducible mission inputs shared by HTTP and the flight process."""
from dataclasses import asdict, dataclass, fields
import math


@dataclass(frozen=True)
class FlightOptions:
    seed: int = 7
    gnss_position_sigma_m: float = 3.0
    gnss_velocity_sigma_m_s: float = 0.5
    initial_position_sigma_m: float = 10.0
    initial_velocity_sigma_m_s: float = 0.02
    orbit_acceleration_sigma_m_s2: float = 0.00002
    orbit_correlation_s: float = 60.0
    drag_enabled: bool = True
    srp_enabled: bool = True
    density_300km_kg_m3: float = 2.418e-11
    scale_height_m: float = 60000.0
    drag_coefficient: float = 2.2
    reflection_coefficient: float = 1.3
    chaser_area_m2: float = 1.0
    target_area_m2: float = 4.0
    handoff_distance_m: float = 110.0

    def __post_init__(self):
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("seed must be an integer in [0, 2^32)")
        for name in ("drag_enabled", "srp_enabled"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be boolean")
        bounds = {
            "gnss_position_sigma_m": (0.001, 100),
            "gnss_velocity_sigma_m_s": (0.00001, 10),
            "initial_position_sigma_m": (0, 1000),
            "initial_velocity_sigma_m_s": (0, 1),
            "orbit_acceleration_sigma_m_s2": (0, 0.001),
            "orbit_correlation_s": (0.5, 3600),
            "density_300km_kg_m3": (0, 1e-8),
            "scale_height_m": (10000, 200000),
            "drag_coefficient": (0.1, 5), "reflection_coefficient": (1, 2),
            "chaser_area_m2": (0.001, 100), "target_area_m2": (0.001, 100),
            "handoff_distance_m": (100.5, 1000),
        }
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be finite in [{low}, {high}]")

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict):
            raise ValueError("mission options must be an object")
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown mission options: {sorted(unknown)}")
        return cls(**values)

    def to_dict(self):
        return asdict(self)
