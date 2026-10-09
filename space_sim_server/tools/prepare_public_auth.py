"""Prepare authentication for zero-input public launch, without resetting custom passwords.

Default invocation is read-only. --apply is used only AFTER the platform stops and
BEFORE the public gateway opens. Secrets come from the environment, never argv.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import sqlite3

DEFAULT_PASSWORD = "ChangeMe123!"
ITERATIONS = 310_000


def matches(row: sqlite3.Row, password: str) -> bool:
    actual = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(row["password_salt"]),
                                 int(row["password_iterations"])).hex()
    return hmac.compare_digest(actual, row["password_hash"])


def plan(path: Path, username: str, password: str) -> dict:
    if not 12 <= len(password) <= 256 or password == DEFAULT_PASSWORD:
        raise ValueError("A generated non-default administrator secret is required.")
    result = {"generated_password_users": [], "existing_admin_users": [], "default_admin_users": []}
    if not path.exists():
        result["generated_password_users"] = [username]
        return result
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        admins = list(db.execute("SELECT * FROM users WHERE role='admin'"))
        if not admins:
            if db.execute("SELECT 1 FROM users WHERE username=? COLLATE NOCASE", (username,)).fetchone():
                raise ValueError("Bootstrap username is already occupied by a non-admin account.")
            result["generated_password_users"] = [username]
            return result
        active = [row for row in admins if row["active"]]
        if not active:
            raise ValueError("No active administrator; public setup will not reactivate disabled accounts.")
        for row in active:
            if matches(row, DEFAULT_PASSWORD):
                result["default_admin_users"].append(row["username"])
                result["generated_password_users"].append(row["username"])
            elif matches(row, password):
                result["generated_password_users"].append(row["username"])
            else:
                result["existing_admin_users"].append(row["username"])
    return result


def prepare(path: Path, username: str, password: str, *, apply: bool = False,
            backup_directory: Path | None = None) -> dict:
    result = plan(path, username, password)
    if not apply or not result["default_admin_users"]:
        return result
    if backup_directory is None:
        raise ValueError("An explicit backup directory is required before rotating default passwords.")
    backup_directory.mkdir(parents=True, exist_ok=True)
    backup = backup_directory / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + secrets.token_hex(8) + ".sqlite3")
    # Use SQLite backup, not a raw copy of a potentially WAL-backed database.
    with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as source, sqlite3.connect(backup) as target:
        source.backup(target)
    rotated = []
    with sqlite3.connect(path.resolve().as_uri() + "?mode=rw", uri=True, timeout=10) as db:
        db.row_factory = sqlite3.Row
        db.execute("BEGIN IMMEDIATE")
        for row in db.execute("SELECT * FROM users WHERE role='admin' AND active=1").fetchall():
            # Recheck under the write lock: never overwrite a newly customized password.
            if not matches(row, DEFAULT_PASSWORD):
                continue
            salt = secrets.token_bytes(16)
            digest = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, ITERATIONS).hex()
            db.execute("UPDATE users SET password_hash=?,password_salt=?,password_iterations=? WHERE user_id=?",
                       (digest, salt.hex(), ITERATIONS, row["user_id"]))
            db.execute("DELETE FROM sessions WHERE user_id=?", (row["user_id"],))
            rotated.append(row["username"])
    result = plan(path, username, password)
    result["rotated_default_users"] = rotated
    result["backup_file"] = str(backup)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--username", default="admin")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--backup-directory", type=Path)
    args = parser.parse_args()
    try:
        result = prepare(args.database, args.username, os.environ.get("SPACE_SIM_ADMIN_PASSWORD", ""),
                         apply=args.apply, backup_directory=args.backup_directory)
    except (ValueError, sqlite3.Error, OSError) as error:
        parser.exit(1, f"Public authentication preparation failed: {error}\n")
    print(json.dumps(result))  # Metadata only; never include passwords/hashes/session tokens.


if __name__ == "__main__":
    main()
