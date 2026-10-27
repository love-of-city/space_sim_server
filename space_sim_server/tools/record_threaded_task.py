"""Record one live threaded manipulation task with both production cameras."""

if __package__:
    from .record_drawer_task import main
else:
    from record_drawer_task import main


if __name__ == "__main__":
    raise SystemExit(main("aviation_bolt_insert"))
