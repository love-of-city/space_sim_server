"""Shared command-line contract for drawer collection motion settings."""

import math


def add_motion_options(parser):
    parser.add_argument(
        "--linear-speed",
        type=float,
        default=None,
        help="Drawer-cycle cruise speed in m/s (default 0.1, maximum 0.1)",
    )
    parser.add_argument(
        "--threaded-speed-profile",
        choices=("default", "fast_collection"),
        default="default",
        help="Speed profile for threaded collection; default preserves the validated slow profile",
    )


def motion_arguments(task, linear_speed, threaded_speed_profile="default"):
    if task != "drawer_cycle" and linear_speed is not None:
        raise ValueError("--linear-speed is supported only for drawer_cycle")
    if linear_speed is not None and (not math.isfinite(linear_speed) or not 0 < linear_speed <= 0.1):
        raise ValueError("Drawer cruise speed must be finite and in (0, 0.1] m/s")
    if threaded_speed_profile not in ("default", "fast_collection"):
        raise ValueError(f"Unknown threaded speed profile: {threaded_speed_profile}")
    if task not in ("aviation_bolt_insert", "locking_rod_cycle") and threaded_speed_profile != "default":
        raise ValueError("--threaded-speed-profile is supported only for threaded tasks")
    result = []
    if task == "drawer_cycle" and linear_speed is not None:
        result += ["--linear-speed", str(linear_speed)]
    if task in ("aviation_bolt_insert", "locking_rod_cycle") and threaded_speed_profile != "default":
        result += ["--threaded-speed-profile", threaded_speed_profile]
    return result
