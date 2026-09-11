"""Read-only preflight; fail before stopping services if a default admin remains."""
from __future__ import annotations

import argparse
import hashlib
import hmac
from pathlib import Path
import sqlite3


def check_database(path: Path) -> None:
    if not path.exists():
        return  # A new database will be bootstrapped using the validated secret.
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        rows = db.execute(
            "SELECT password_hash,password_salt,password_iterations FROM users WHERE role='admin' AND active=1"
        )
        for expected, salt, iterations in rows:
            actual = hashlib.pbkdf2_hmac("sha256", b"ChangeMe123!", bytes.fromhex(salt), int(iterations)).hex()
            if hmac.compare_digest(actual, expected):
                raise ValueError("Change the existing default admin password in the local UI before deployment; bootstrap settings do not update existing accounts.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    args = parser.parse_args()
    try:
        check_database(args.database)
    except (ValueError, sqlite3.Error, OSError) as error:
        parser.exit(1, f"Deployment authentication preflight failed: {error}\n")
    print("Deployment authentication preflight passed (read-only).")


if __name__ == "__main__":
    main()
