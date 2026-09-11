"""Default-password migration uses throwaway SQLite databases, never the live auth store."""
import importlib.util
import json
from pathlib import Path

import pytest

from space_arm_platform.auth import AuthStore

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("public_auth", ROOT / "tools/prepare_public_auth.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
PASSWORD = "Generated-Public-Test-Password!"


def test_public_auth_readonly_new_database_does_not_create_files(tmp_path):
    db = tmp_path / "auth.sqlite3"
    assert module.prepare(db, "admin", PASSWORD)["generated_password_users"] == ["admin"]
    assert not db.exists()


def test_only_default_password_rotated_and_sessions_revoked_with_backup(tmp_path):
    db = tmp_path / "auth.sqlite3"
    store = AuthStore(db, "admin", "ChangeMe123!")
    user = store.authenticate("admin", "ChangeMe123!")
    session, _ = store.create_session(user["user_id"])
    operator = store.create_operator("operator1", "Operator-Password!")
    original = (user.copy(), operator.copy())
    plan = module.prepare(db, "admin", PASSWORD)
    assert plan["default_admin_users"] == ["admin"]
    assert store.authenticate("admin", "ChangeMe123!") is not None
    assert store.session_user(session) is not None
    result = module.prepare(db, "admin", PASSWORD, apply=True, backup_directory=tmp_path / "backups")
    assert result["rotated_default_users"] == ["admin"]
    assert store.authenticate("admin", "ChangeMe123!") is None
    assert store.authenticate("admin", PASSWORD)["user_id"] == original[0]["user_id"]
    assert store.authenticate("operator1", "Operator-Password!")["user_id"] == original[1]["user_id"]
    assert store.session_user(session) is None
    assert Path(result["backup_file"]).is_file()
    assert module.plan(Path(result["backup_file"]), "admin", PASSWORD)["default_admin_users"] == ["admin"]
    assert PASSWORD not in json.dumps(result)
    second = module.prepare(db, "admin", PASSWORD, apply=True, backup_directory=tmp_path / "backups")
    assert second["generated_password_users"] == ["admin"]
    assert len(list((tmp_path / "backups").glob("*.sqlite3"))) == 1
    store.close()


def test_custom_password_and_existing_session_are_never_changed(tmp_path):
    db = tmp_path / "auth.sqlite3"
    store = AuthStore(db, "custom-admin", "Customized-Password!")
    user = store.authenticate("custom-admin", "Customized-Password!")
    token, _ = store.create_session(user["user_id"])
    result = module.prepare(db, "admin", PASSWORD, apply=True, backup_directory=tmp_path / "backups")
    assert result["generated_password_users"] == []
    assert result["existing_admin_users"] == ["custom-admin"]
    assert store.authenticate("custom-admin", "Customized-Password!")
    assert store.session_user(token)
    assert not (tmp_path / "backups").exists()
    store.close()


def test_apply_requires_backup_and_rejects_weak_secret(tmp_path):
    db = tmp_path / "auth.sqlite3"
    store = AuthStore(db, "admin", "ChangeMe123!")
    with pytest.raises(ValueError, match="backup"):
        module.prepare(db, "admin", PASSWORD, apply=True)
    for password in ("", "short", "ChangeMe123!"):
        with pytest.raises(ValueError):
            module.prepare(db, "admin", password, apply=True, backup_directory=tmp_path / "backups")
    assert store.authenticate("admin", "ChangeMe123!")
    store.close()
