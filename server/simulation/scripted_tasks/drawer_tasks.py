"""Lazy identities and factories for full-cycle and independent drawer tasks."""

from space_arm_platform.task_catalog import DRAWER_TASKS


def drawer_task_class(name):
    if name == "drawer_cycle":
        from .drawer_cycle import DrawerCycleTask
        return DrawerCycleTask
    if name == "drawer_open":
        from .drawer_open import DrawerOpenTask
        return DrawerOpenTask
    if name == "drawer_close":
        from .drawer_close import DrawerCloseTask
        return DrawerCloseTask
    raise ValueError(f"Unknown drawer task: {name}")


def create_drawer_task(name, model, **options):
    task_class = drawer_task_class(name)
    if name == "drawer_open":
        from .drawer_open import DrawerOpenConfig as Config
    elif name == "drawer_close":
        from .drawer_close import DrawerCloseConfig as Config
    else:
        from .drawer_cycle import DrawerCycleConfig as Config
    return task_class(model, Config(**options))
