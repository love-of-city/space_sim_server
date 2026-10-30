"""Shared collection identities for the two threaded manipulation tasks.

Controllers are imported lazily so listing the web collection catalog never
loads a native dynamics library.
"""

THREADED_TASKS = ("aviation_bolt_insert", "locking_rod_cycle")


def task_default_duration_s(name):
    return 900.0 if name in THREADED_TASKS else 360.0

THREADED_INSTRUCTIONS = {
    "aviation_bolt_insert": (
        "Unscrew the aviation bolt from its holder, then screw it fully "
        "into the corresponding hole."
    ),
    "locking_rod_cycle": (
        "Loosen the locking rod's fastener, fold the rod into its clip, "
        "then raise it and tighten the fastener."
    ),
}


def threaded_task_class(name):
    if name == "aviation_bolt_insert":
        from .aviation_bolt import AviationBoltTask

        return AviationBoltTask
    if name == "locking_rod_cycle":
        from .locking_rod import LockingRodTask

        return LockingRodTask
    raise ValueError(f"Unknown threaded task: {name}")


def create_threaded_task(name, model, **options):
    speed_profile = options.pop("speed_profile", "default")
    from .threaded_motion import threaded_speed_profile_options
    profile_options = threaded_speed_profile_options(speed_profile)
    # Explicit options are useful for diagnostics and take precedence over a
    # named profile.  The selected profile is still recorded in task metadata.
    profile_options.update(options)
    profile_options["speed_profile"] = speed_profile
    if name == "aviation_bolt_insert":
        from .aviation_bolt import AviationBoltConfig

        config = AviationBoltConfig(**profile_options)
    elif name == "locking_rod_cycle":
        from .locking_rod import LockingRodConfig

        config = LockingRodConfig(**profile_options)
    else:
        raise ValueError(f"Unknown threaded task: {name}")
    return threaded_task_class(name)(model, config)
