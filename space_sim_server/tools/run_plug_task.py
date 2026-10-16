"""Compatibility entry point for the shared scripted task runtime."""

if __package__:
    from .run_scripted_task import main
else:
    from run_scripted_task import main

if __name__ == "__main__":
    raise SystemExit(main("plug_insert"))
