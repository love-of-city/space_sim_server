"""Preserve the local accounts when switching to the standard deployment database."""

import argparse
from contextlib import closing
from pathlib import Path
import sqlite3


def migrate(source: Path, destination: Path) -> bool:
    if destination.exists() or not source.is_file():
        return False
    destination.parent.mkdir(parents=True, exist_ok=True)
    with closing(sqlite3.connect(source.resolve().as_uri() + "?mode=ro", uri=True)) as original:
        original.execute("SELECT user_id FROM users LIMIT 1").fetchall()
        with destination.open("xb"):
            pass
        try:
            with closing(sqlite3.connect(destination)) as target:
                original.backup(target)
                if target.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
                    raise RuntimeError("Migrated authentication database did not pass integrity checking")
        except Exception:
            destination.unlink(missing_ok=True)
            raise
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    changed = migrate(args.source, args.destination)
    print("Existing accounts copied using SQLite backup." if changed else "Existing deployment database preserved; no accounts overwritten.")


if __name__ == "__main__":
    main()
