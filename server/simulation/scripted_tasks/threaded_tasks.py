"""Shared collection identities for threaded manipulation tasks.

Controllers are imported lazily so listing the web collection catalog never
loads a native dynamics library.
"""

from space_arm_platform.task_catalog import LOCKING_ROD_TASKS, THREADED_TASKS


def threaded_task_class(name):
    if name == "aviation_bolt_insert":
        from .aviation_bolt import AviationBoltTask

        return AviationBoltTask
    if name in LOCKING_ROD_TASKS:
        from .locking_rod import LockingRodTask, LockingRodFoldTask, LockingRodRestoreTask

        return {"locking_rod_cycle": LockingRodTask, "locking_rod_fold": LockingRodFoldTask,
                "locking_rod_restore": LockingRodRestoreTask}[name]
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
    elif name in LOCKING_ROD_TASKS:
        config = threaded_task_class(name).config_class(**profile_options)
    else:
        raise ValueError(f"Unknown threaded task: {name}")
    return threaded_task_class(name)(model, config)
