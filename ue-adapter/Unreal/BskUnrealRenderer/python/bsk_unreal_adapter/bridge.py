"""UE-specific compatibility name for the renderer-neutral bridge."""

from bsk_render_adapter import BasiliskRenderBridge


class BasiliskUnrealBridge(BasiliskRenderBridge):
    """Backward-compatible UE class name that emits ``bsk-render/2``."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ModelTag = "BasiliskUnrealBridge"
