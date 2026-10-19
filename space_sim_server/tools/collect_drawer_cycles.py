"""Compatibility entry point for drawer batch collection."""

if __package__:
    from .collect_scripted_tasks import main, validate_recording, validate_capture_manifest
else:
    from collect_scripted_tasks import main, validate_recording, validate_capture_manifest

if __name__ == "__main__":
    raise SystemExit(main())
