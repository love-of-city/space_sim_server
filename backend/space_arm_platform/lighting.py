"""Scene-local rendering controls; never used by the dynamics models."""
import math

DEFAULT_SUNLIGHT_INTENSITY_SCALE = 1.0  # [-], preserve renderer calibration
MAX_SUNLIGHT_INTENSITY_SCALE = 20_000.0  # [-], operator-facing safety bound


def validate_sunlight_intensity_scale(value: object) -> float:
    """Require a finite JSON number in the supported range, including zero."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("sunlight_intensity_scale must be a number in [0, 20000]")
    scale = float(value)
    if not math.isfinite(scale) or not 0.0 <= scale <= MAX_SUNLIGHT_INTENSITY_SCALE:
        raise ValueError("sunlight_intensity_scale must be finite and in [0, 20000]")
    return scale
