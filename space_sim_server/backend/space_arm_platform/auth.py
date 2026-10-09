"""Persistent two-role authentication and browser sessions."""

from __future__ import annotations

import hashlib
import hmac
import secrets
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any


SESSION_COOKIE = "space_sim_session"
_PASSWORD_ITERATIONS = 310_000
_SESSION_SECONDS = 12 * 60 * 60


class AuthError(RuntimeError):
    pass


class AuthStore:
    def __init__(self, path: Path, bootstrap_username: str, bootstrap_password: str) -> None:
        self.path = path.resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(self.path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._lock:
            self._db.executescript(
                """
                PRAGMA journal_mode=WAL;
                PRAGMA foreign_keys=ON;
                CREATE TABLE IF NOT EXISTS users (
                    user_id TEXT PRIMARY KEY,
                    username TEXT NOT NULL UNIQUE COLLATE NOCASE,
                    role TEXT NOT NULL CHECK(role IN ('admin', 'operator')),
                    password_hash TEXT NOT NULL,
                    password_salt TEXT NOT NULL,
                    password_iterations INTEGER NOT NULL,
                    active INTEGER NOT NULL DEFAULT 1,
                    created_at_ns TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS sessions (
                    token_hash TEXT PRIMARY KEY,
                    user_id TEXT NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
                    created_at_ns TEXT NOT NULL,
                    expires_at_ns TEXT NOT NULL
                );
                """
            )
            self._db.commit()
        self.bootstrap_admin_created = self._bootstrap_admin(bootstrap_username, bootstrap_password)

    @staticmethod
    def _validate_username(username: str) -> str:
        normalized = username.strip()
        if not 3 <= len(normalized) <= 64:
            raise AuthError("用户名长度必须为 3～64 个字符")
        if not all(ch.isalnum() or ch in "._-" for ch in normalized):
            raise AuthError("用户名只能包含字母、数字、点、下划线和连字符")
        return normalized

    @staticmethod
    def _validate_password(password: str) -> None:
        if len(password) < 8 or len(password) > 256:
            raise AuthError("密码长度必须为 8～256 个字符")

    @staticmethod
    def _password_record(password: str, salt_hex: str | None = None) -> tuple[str, str, int]:
        salt = bytes.fromhex(salt_hex) if salt_hex else secrets.token_bytes(16)
        digest = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, _PASSWORD_ITERATIONS)
        return digest.hex(), salt.hex(), _PASSWORD_ITERATIONS

    @staticmethod
    def _public_user(row: sqlite3.Row) -> dict[str, Any]:
        return {
            "user_id": row["user_id"],
            "username": row["username"],
            "role": row["role"],
            "active": bool(row["active"]),
            "created_at_ns": row["created_at_ns"],
        }

    def _bootstrap_admin(self, username: str, password: str) -> bool:
        username = self._validate_username(username)
        self._validate_password(password)
        with self._lock:
            exists = self._db.execute("SELECT 1 FROM users WHERE role='admin' LIMIT 1").fetchone()
            if exists:
                return False
            password_hash, salt, iterations = self._password_record(password)
            self._db.execute(
                "INSERT INTO users VALUES (?, ?, 'admin', ?, ?, ?, 1, ?)",
                (f"user-{uuid.uuid4().hex}", username, password_hash, salt, iterations, str(time.time_ns())),
            )
            self._db.commit()
            return True

    def authenticate(self, username: str, password: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._db.execute("SELECT * FROM users WHERE username=? COLLATE NOCASE", (username.strip(),)).fetchone()
        if not row or not row["active"]:
            return None
        digest = hashlib.pbkdf2_hmac(
            "sha256", password.encode("utf-8"), bytes.fromhex(row["password_salt"]), int(row["password_iterations"])
        ).hex()
        return self._public_user(row) if hmac.compare_digest(digest, row["password_hash"]) else None

    def create_session(self, user_id: str) -> tuple[str, int]:
        token = secrets.token_urlsafe(32)
        now = time.time_ns()
        expires = now + _SESSION_SECONDS * 1_000_000_000
        with self._lock:
            self._db.execute("DELETE FROM sessions WHERE CAST(expires_at_ns AS INTEGER) <= ?", (now,))
            self._db.execute(
                "INSERT INTO sessions VALUES (?, ?, ?, ?)",
                (hashlib.sha256(token.encode()).hexdigest(), user_id, str(now), str(expires)),
            )
            self._db.commit()
        return token, _SESSION_SECONDS

    def session_user(self, token: str | None) -> dict[str, Any] | None:
        if not token:
            return None
        token_hash = hashlib.sha256(token.encode()).hexdigest()
        now = time.time_ns()
        with self._lock:
            row = self._db.execute(
                """SELECT users.* FROM sessions JOIN users USING(user_id)
                   WHERE token_hash=? AND CAST(expires_at_ns AS INTEGER)>? AND users.active=1""",
                (token_hash, now),
            ).fetchone()
        return self._public_user(row) if row else None

    def delete_session(self, token: str | None) -> None:
        if not token:
            return
        with self._lock:
            self._db.execute("DELETE FROM sessions WHERE token_hash=?", (hashlib.sha256(token.encode()).hexdigest(),))
            self._db.commit()

    def list_users(self) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute("SELECT * FROM users ORDER BY role, username COLLATE NOCASE").fetchall()
        return [self._public_user(row) for row in rows]

    def create_operator(self, username: str, password: str) -> dict[str, Any]:
        username = self._validate_username(username)
        self._validate_password(password)
        password_hash, salt, iterations = self._password_record(password)
        user_id = f"user-{uuid.uuid4().hex}"
        try:
            with self._lock:
                self._db.execute(
                    "INSERT INTO users VALUES (?, ?, 'operator', ?, ?, ?, 1, ?)",
                    (user_id, username, password_hash, salt, iterations, str(time.time_ns())),
                )
                self._db.commit()
                row = self._db.execute("SELECT * FROM users WHERE user_id=?", (user_id,)).fetchone()
        except sqlite3.IntegrityError as error:
            raise AuthError("用户名已存在") from error
        return self._public_user(row)

    def delete_operator(self, user_id: str) -> None:
        with self._lock:
            row = self._db.execute("SELECT role FROM users WHERE user_id=?", (user_id,)).fetchone()
            if not row:
                raise AuthError("操作员不存在")
            if row["role"] != "operator":
                raise AuthError("不能删除管理员账号")
            self._db.execute("DELETE FROM users WHERE user_id=?", (user_id,))
            self._db.commit()

    def change_password(self, user_id: str, current_password: str, new_password: str) -> None:
        self._validate_password(new_password)
        with self._lock:
            row = self._db.execute("SELECT * FROM users WHERE user_id=?", (user_id,)).fetchone()
            if not row:
                raise AuthError("用户不存在")
            current_hash = hashlib.pbkdf2_hmac(
                "sha256", current_password.encode("utf-8"), bytes.fromhex(row["password_salt"]),
                int(row["password_iterations"]),
            ).hex()
            if not hmac.compare_digest(current_hash, row["password_hash"]):
                raise AuthError("当前密码错误")
            password_hash, salt, iterations = self._password_record(new_password)
            self._db.execute(
                "UPDATE users SET password_hash=?, password_salt=?, password_iterations=? WHERE user_id=?",
                (password_hash, salt, iterations, user_id),
            )
            self._db.execute("DELETE FROM sessions WHERE user_id=?", (user_id,))
            self._db.commit()

    def reset_operator_password(self, user_id: str, password: str) -> None:
        self._validate_password(password)
        password_hash, salt, iterations = self._password_record(password)
        with self._lock:
            row = self._db.execute("SELECT role FROM users WHERE user_id=?", (user_id,)).fetchone()
            if not row or row["role"] != "operator":
                raise AuthError("操作员不存在")
            self._db.execute(
                "UPDATE users SET password_hash=?, password_salt=?, password_iterations=? WHERE user_id=?",
                (password_hash, salt, iterations, user_id),
            )
            self._db.execute("DELETE FROM sessions WHERE user_id=?", (user_id,))
            self._db.commit()

    def close(self) -> None:
        with self._lock:
            self._db.close()
