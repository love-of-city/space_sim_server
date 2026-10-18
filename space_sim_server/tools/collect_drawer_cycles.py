"""Compatibility entry point for drawer batch collection."""

if __package__:
    from .collect_scripted_tasks import main, validate_recording
else:
    from collect_scripted_tasks import main, validate_recording

if __name__ == "__main__":
    raise SystemExit(main())
