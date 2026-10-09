from types import SimpleNamespace

import pytest

from simulation.runtime_performance import RuntimePerformanceMonitor, render_transport_status


def test_monitor_separates_pacing_work_and_transport():
    monitor = RuntimePerformanceMonitor(1., wall_start=100., interval_s=2.)
    assert monitor.record(1_000_000_000, .2, .01, now=101.) is None
    event = monitor.record(2_000_000_000, .2, .01, now=102.)
    assert event["wall_real_time_factor"] == 1.
    assert event["processing_real_time_factor"] == pytest.approx(2 / .42)
    assert event["schedule_lag_s"] == 0.
    assert event["outer_frames"] == 2
    assert event["simulation_execute_s"] == pytest.approx(.4)
    assert event["observation_send_s"] == pytest.approx(.02)
    event = monitor.record(2_100_000_000, .1, 10., now=112.1)
    assert event["window_simulated_s"] == pytest.approx(.1)
    assert event["wall_real_time_factor"] == pytest.approx(.1 / 10.1)
    assert event["schedule_lag_s"] == pytest.approx(10.)
    assert event["outer_frames"] == 1


def test_monitor_understands_slow_requested_rate():
    monitor = RuntimePerformanceMonitor(.5, wall_start=0., interval_s=2.)
    event = monitor.record(1_000_000_000, .7, .1, now=2.)
    assert event["wall_real_time_factor"] == .5
    assert event["schedule_lag_s"] == 0.


@pytest.mark.parametrize("value", [0., -1., float("nan"), float("inf")])
def test_monitor_rejects_invalid_target(value):
    with pytest.raises(ValueError):
        RuntimePerformanceMonitor(value, wall_start=0.)


def test_transport_status_only_observes_counters():
    stats = SimpleNamespace(frames_queued=100, frames_sent=60, frames_dropped=10, last_error="timeout")
    publisher = SimpleNamespace(stats=stats)
    assert render_transport_status(publisher)["render_backlog_frames"] == 30
    assert render_transport_status(publisher)["render_transport_error"] == "timeout"
    assert stats.frames_queued == 100
    assert render_transport_status(object()) == {}
