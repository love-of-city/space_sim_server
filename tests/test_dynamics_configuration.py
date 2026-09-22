import pytest
from pydantic import ValidationError

from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager


@pytest.mark.parametrize("step", [1.0 / 240])
def test_scene_preserves_explicit_dynamics_step(tmp_path, step):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=17, dynamics_step_s=step))
    assert instance["runtime"]["dynamics_step_s"] == step


def test_new_scenes_default_to_rational_240hz(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=17))
    assert instance["runtime"]["dynamics_step_s"] == 1.0 / 240


@pytest.mark.parametrize("step", [0, -0.001, 0.00025, 0.0005, 0.001, 0.002, float("inf"), float("nan")])
def test_scene_rejects_unvalidated_dynamics_steps(step):
    with pytest.raises(ValidationError):
        SceneInstanceCreate(dynamics_step_s=step)
