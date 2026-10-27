"""Collect threaded manipulation tasks through the shared production pipeline."""

if __package__:
    from .collect_scripted_tasks import main
else:
    from collect_scripted_tasks import main


if __name__ == "__main__":
    raise SystemExit(main("aviation_bolt_insert"))
