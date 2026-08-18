"""Space manipulator teleoperation and dataset collection backend.

The simulation-side protocol client runs in the lean ``mujoco-dev`` Conda
environment, so importing this package must not eagerly require FastAPI.
"""


def create_app(*args, **kwargs):
    """Import the web stack only when the backend application is requested."""

    from .app import create_app as factory

    return factory(*args, **kwargs)


__all__ = ["create_app"]
