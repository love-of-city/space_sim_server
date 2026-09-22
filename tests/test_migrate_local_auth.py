import sqlite3

import pytest

from tools.migrate_local_auth import migrate


def test_preserves_accounts_including_wal(tmp_path):
    source = tmp_path / "source.sqlite3"
    destination = tmp_path / "new" / "auth.sqlite3"
    with sqlite3.connect(source) as database:
        database.execute("PRAGMA journal_mode=WAL")
        database.execute("CREATE TABLE users (user_id TEXT, password_hash TEXT)")
        database.execute("INSERT INTO users VALUES ('existing-user', 'unchanged-hash')")
        database.commit()
        assert migrate(source, destination)
        with sqlite3.connect(destination) as copied:
            assert copied.execute("SELECT * FROM users").fetchall() == [('existing-user', 'unchanged-hash')]
        assert source.is_file()


def test_never_overwrites_destination(tmp_path):
    destination = tmp_path / "auth.sqlite3"
    destination.write_bytes(b"existing")
    assert not migrate(tmp_path / "absent", destination)
    assert destination.read_bytes() == b"existing"


def test_missing_source_does_not_create_database(tmp_path):
    destination = tmp_path / "auth.sqlite3"
    assert not migrate(tmp_path / "absent", destination)
    assert not destination.exists()


def test_invalid_source_leaves_destination_absent(tmp_path):
    source = tmp_path / "bad.sqlite3"
    source.write_bytes(b"invalid")
    destination = tmp_path / "auth.sqlite3"
    with pytest.raises(sqlite3.DatabaseError):
        migrate(source, destination)
    assert not destination.exists()
