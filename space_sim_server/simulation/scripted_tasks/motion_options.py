"""Shared command-line contract for drawer collection motion settings."""

import math


def add_motion_options(parser):
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=None,
        help="Drawer-cycle cruise speed in m/s (default 0.1, maximum 0.1)",
    )


def motion_arguments(task, linear_speed):
    if linear_speed is None:
        return []
    if task != "drawer_cycle":
        raise ValueError("--linear-speed is supported only for drawer_cycle")
    if not math.isfinite(linear_speed) or not 0 < linear_speed <= 0.1:
        raise ValueError("Drawer cruise speed must be finite and in (0, 0.1] m/s")
    return ["--linear-speed", str(linear_speed)]
