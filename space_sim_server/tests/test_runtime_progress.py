import json

import pytest

from simulation.runtime_progress import runtime_stage


def test_runtime_stage_records_completion(capsys):
    with runtime_stage("test_build"):
        pass
    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert [event["status"] for event in events] == ["started", "completed"]
    assert events[-1]["elapsed_s"] >= 0


def test_runtime_stage_preserves_exception(capsys):
    with pytest.raises(ValueError, match="failure"):
        with runtime_stage("test_build"):
            raise ValueError("failure")
    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert events[-1]["status"] == "failed"
