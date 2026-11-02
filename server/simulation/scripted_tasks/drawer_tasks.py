"""Lazy identities and factories for full-cycle and independent drawer tasks."""

DRAWER_TASKS = ("drawer_cycle", "drawer_open", "drawer_close")
DRAWER_INSTRUCTIONS = {
    "drawer_cycle": "Open the drawer approximately 8 cm, then close it fully and release it.",
    "drawer_open": "Open the drawer approximately 8 cm, then release it and withdraw the gripper.",
    "drawer_close": "Close the open drawer fully, then release it and withdraw the gripper.",
}


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
