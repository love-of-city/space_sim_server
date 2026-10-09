import pytest

from space_arm_platform.models import TaskCreate
from space_arm_platform.tasks import TaskStore


def test_task_lifecycle_is_persistent_and_strict(tmp_path) -> None:
    store = TaskStore(tmp_path)
    created = store.create(TaskCreate(instruction="抓取目标", seed=7))
    running = store.transition(created["task_id"], {"queued"}, "running", episode_id="episode-1")
    assert running["episode_id"] == "episode-1"
    with pytest.raises(RuntimeError):
        store.transition(created["task_id"], {"queued"}, "running")
    reloaded = TaskStore(tmp_path)
    assert reloaded.get(created["task_id"])["status"] == "running"
