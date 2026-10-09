"""240/120/30 share one exact integer time grid, including long-run timestamps."""
import pytest
from space_arm_platform.sampling import (
    DYNAMICS_HZ, DEFAULT_IK_HZ, DEFAULT_CAPTURE_HZ, SUPPORTED_FPS,
    tick_time_ns, sample_tick, ik_step_stride,
)
from space_arm_platform.models import EpisodeStart, SceneInstanceCreate


def test_defaults_and_integer_dividers():
    scene = SceneInstanceCreate()
    assert (DYNAMICS_HZ, scene.ik_rate_hz, scene.capture_rate_hz) == (240, 120, 30)
    assert (DEFAULT_IK_HZ, DEFAULT_CAPTURE_HZ, EpisodeStart().fps) == (120, 30, 30)
    assert ik_step_stride(scene.ik_rate_hz) == 2
    for rate in (100, 119, 121, 120.5, 0, 241, 500):
        with pytest.raises(ValueError):
            ik_step_stride(rate)
        with pytest.raises(ValueError):
            SceneInstanceCreate(ik_rate_hz=rate)


@pytest.mark.parametrize("seconds", [0, 1, 3600, 86400, 365 * 86400])
def test_absolute_grid_has_no_accumulated_rounding_error(seconds):
    base = seconds * DYNAMICS_HZ
    stamps = [tick_time_ns(base + n, DYNAMICS_HZ) for n in range(241)]
    assert stamps[-1] - stamps[0] == 1_000_000_000
    assert set(b - a for a, b in zip(stamps, stamps[1:])) == {4_166_666, 4_166_667}
    for n, stamp in enumerate(stamps):
        # Error <= half a nanosecond, measured without floats.
        assert abs(stamp * 240 - (base + n) * 1_000_000_000) <= 120
        tick = sample_tick(stamp, 30)
        if n % 8 == 0:
            assert tick == seconds * 30 + n // 8
            assert stamp == tick_time_ns(seconds * 30 + n // 8, 30)
        else:
            assert tick is None
        if n % 2 == 0:
            assert stamp == tick_time_ns(seconds * 120 + n // 2, 120)


@pytest.mark.parametrize("fps", SUPPORTED_FPS)
def test_dataset_grid_rejects_off_grid_and_unsupported_rates(fps):
    for tick in (0, 1, 2, fps * 86400 + 1):
        stamp = tick_time_ns(tick, fps)
        assert sample_tick(stamp, fps) == tick
        assert sample_tick(stamp + 1, fps) is None
    assert sample_tick(-1, fps) is None
    assert sample_tick(0, 24) is None


def test_scene_metadata_records_physics_clock(tmp_path):
    from space_arm_platform.scene_runtime import SceneRuntimeManager
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=7))
    assert instance["runtime"]["dynamics_rate_hz"] == 240
    assert instance["runtime"]["clock"] == "absolute-rational-nanoseconds"
    assert instance["runtime"]["ik_rate_hz"] == 120
    assert instance["runtime"]["capture_rate_hz"] == 30


def test_backend_defaults_and_legacy_rate_validation(tmp_path):
    from space_arm_platform.app import PlatformConfig
    config = PlatformConfig(project_root=tmp_path, data_root=tmp_path / "episodes")
    assert config.runtime_ik_rate == 120
    assert config.runtime_capture_rate == 30
    with pytest.raises(ValueError, match="divisor"):
        PlatformConfig(project_root=tmp_path, data_root=tmp_path, runtime_ik_rate=100)
    with pytest.raises(ValueError, match="capture rate"):
        PlatformConfig(project_root=tmp_path, data_root=tmp_path,
                       runtime_default_dataset_capture=True, runtime_capture_rate=24)
