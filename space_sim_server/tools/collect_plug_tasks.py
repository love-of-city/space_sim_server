"""Collect one selected independent plug task per episode."""

if __package__:
    from .collect_scripted_tasks import main
else:
    from collect_scripted_tasks import main

if __name__ == "__main__":
    raise SystemExit(main("cylindrical_plug_insert"))
