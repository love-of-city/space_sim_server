"""Dependency-free local physics stepping and publication settings."""
import os

from space_arm_platform.sampling import DYNAMICS_HZ, RENDER_HZ

DEFAULT_SUBSTEPS = 1
MAX_SUBSTEPS = 16
DEFAULT_PUBLISH_STRIDE = DYNAMICS_HZ // RENDER_HZ
ALLOWED_PUBLISH_STRIDES = (1, 2, 4, 8)


def local_substeps(value: int | None = None) -> int:
    """Fixed MuJoCo substeps per physics step (SPACE_SIM_LOCAL_SUBSTEPS)."""
    raw = value if value is not None else os.environ.get("SPACE_SIM_LOCAL_SUBSTEPS", DEFAULT_SUBSTEPS)
    count = int(raw)
    if not 1 <= count <= MAX_SUBSTEPS:
        raise ValueError(f"local substeps must be in [1, {MAX_SUBSTEPS}]")
    return count


def publish_stride(value: int | None = None) -> int:
    """Body/site kinematics messages every Nth physics step (SPACE_SIM_LOCAL_PUBLISH_STRIDE)."""
    raw = value if value is not None else os.environ.get("SPACE_SIM_LOCAL_PUBLISH_STRIDE", DEFAULT_PUBLISH_STRIDE)
    count = int(raw)
    if count not in ALLOWED_PUBLISH_STRIDES:
        raise ValueError(f"publish stride must be one of {ALLOWED_PUBLISH_STRIDES}")
    return count


