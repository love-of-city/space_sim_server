from pathlib import Path
import sys

import pytest

from space_arm_platform.benchmark_paths import evidence_path, windows_extended_path


@pytest.mark.parametrize(("original", "expected"), [
    ("C:\\results\\trial", "\\\\?\\C:\\results\\trial"),
    ("\\\\server\\share\\trial", "\\\\?\\UNC\\server\\share\\trial"),
    ("\\\\?\\C:\\results\\trial", "\\\\?\\C:\\results\\trial"),
    ("\\\\?\\UNC\\server\\share\\trial", "\\\\?\\UNC\\server\\share\\trial"),
])
def test_windows_prefix_preserves_drive_unc_and_already_extended_paths(original, expected):
    assert windows_extended_path(original) == expected


def test_evidence_path_resolves_before_adding_prefix_and_preserves_target(tmp_path):
    source = tmp_path/'directory'/'..'/'result.json'
    destination = evidence_path(source)
    assert destination == evidence_path(destination)
    destination.write_text('evidence', encoding='utf-8')
    assert (tmp_path/'result.json').read_text(encoding='utf-8') == 'evidence'
    assert destination.is_absolute()
    if sys.platform == 'win32':
        assert str(destination).startswith('\\\\?\\')
    else:
        assert destination == Path(source).resolve()
