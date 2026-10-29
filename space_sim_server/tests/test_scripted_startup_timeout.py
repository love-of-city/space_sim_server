"""Native model initialization and simulation pre-roll have independent budgets."""
from types import SimpleNamespace

import pytest

from collection import collect_scripted_tasks as collector
from collection import record_scripted_task as recorder
from collection.collect_parallel import CollectionConfig


def finish_immediate(coroutine):
    # The virtual clock's sleeps complete synchronously; no real sockets or
    # elapsed wall time are needed for the deadline regression.
    try:
        coroutine.send(None)
    except StopIteration as result:
        return result.value
    finally:
        coroutine.close()
    pytest.fail("Virtual-clock coroutine unexpectedly yielded")


class StartupClock:
    def __init__(self, first_state_at, preroll_at, *, acknowledgement=True):
        self.now = 0.
        self.first_state_at = first_state_at
        self.preroll_at = preroll_at
        self.hub = SimpleNamespace(latest_observation=None,
                                   capture_pair_ack_supported=acknowledgement)
        self.emitted = []
        self.checks = 0

    def check(self):
        self.checks += 1

    async def sleep(self, seconds):
        self.now += seconds
        if self.now >= self.first_state_at:
            self.hub.latest_observation = SimpleNamespace(
                sim_time_ns="20000000" if self.now >= self.preroll_at else "0")

    def wait(self, monkeypatch, timeout):
        monkeypatch.setattr(recorder, "time", SimpleNamespace(monotonic=lambda: self.now))
        monkeypatch.setattr(recorder, "asyncio", SimpleNamespace(sleep=self.sleep))
        return recorder.wait_for_simulation_startup(
            self.hub, 20_000_000, timeout, self.check,
            lambda stage: self.emitted.append((stage, self.now)))


def test_late_first_state_keeps_a_fresh_full_preroll_budget(monkeypatch):
    clock = StartupClock(first_state_at=179., preroll_at=350.)
    finish_immediate(clock.wait(monkeypatch, 180.))
    assert clock.now >= 350.
    assert clock.emitted[0][0] == "first_observation"
    assert 179 <= clock.emitted[0][1] < 180
    assert clock.checks > 3000


@pytest.mark.parametrize("budget,passes", [(180., False), (600., True)])
def test_configured_budget_permits_slow_native_initialization(monkeypatch, budget, passes):
    clock = StartupClock(first_state_at=300., preroll_at=310.)
    waiting = clock.wait(monkeypatch, budget)
    if passes:
        finish_immediate(waiting)
        assert clock.now >= 310.
    else:
        with pytest.raises(TimeoutError, match="No authoritative simulation state within 180 seconds"):
            finish_immediate(waiting)
        assert not clock.emitted


def test_preroll_timeout_is_still_enforced_after_a_slow_first_state(monkeypatch):
    clock = StartupClock(first_state_at=170., preroll_at=1000.)
    with pytest.raises(TimeoutError, match="pre-roll did not complete"):
        finish_immediate(clock.wait(monkeypatch, 180.))
    assert 350 <= clock.now < 351


def test_missing_capture_acknowledgements_still_fail_before_preroll(monkeypatch):
    clock = StartupClock(first_state_at=1., preroll_at=2., acknowledgement=False)
    with pytest.raises(RuntimeError, match="capture acknowledgements"):
        finish_immediate(clock.wait(monkeypatch, 600.))


@pytest.mark.parametrize("first_state_at", [0., 1000.])
def test_cancellation_is_checked_during_both_startup_phases(monkeypatch, first_state_at):
    clock = StartupClock(first_state_at=first_state_at, preroll_at=1000.)

    def cancel():
        if clock.now > 2.:
            raise RuntimeError("cancelled by operator")

    clock.check = cancel
    with pytest.raises(RuntimeError, match="cancelled by operator"):
        finish_immediate(clock.wait(monkeypatch, 600.))
    assert clock.now < 3.


@pytest.mark.parametrize("entry", [collector, recorder])
@pytest.mark.parametrize("value", ["0", "-1", "7200.1", "nan", "inf"])
def test_invalid_startup_budget_is_rejected_before_runtime(tmp_path, monkeypatch, capsys, entry, value):
    output = tmp_path / "recording"
    monkeypatch.setattr("sys.argv", [
        "collect", "--scene", str(tmp_path / "missing.json"), "--output", str(output),
        "--simulation-startup-timeout-seconds", value,
    ])
    with pytest.raises(SystemExit) as error:
        entry.main()
    assert error.value.code == 2
    assert "simulation-startup-timeout-seconds must be finite and in (0,7200]" in capsys.readouterr().err
    assert not output.exists()


@pytest.mark.parametrize("arguments,expected", [([], 600.), (["--simulation-startup-timeout-seconds", "180"], 180.), (["--startup-timeout-seconds", "180"], 180.)])
def test_recording_cli_uses_default_or_explicit_startup_budget(tmp_path, monkeypatch, arguments, expected):
    async def capture(args):
        return args.simulation_startup_timeout_seconds

    monkeypatch.setattr(recorder, "run", capture)
    monkeypatch.setattr(recorder, "asyncio", SimpleNamespace(run=finish_immediate))
    monkeypatch.setattr("sys.argv", [
        "record", "--scene", str(tmp_path / "scene.json"), "--output", str(tmp_path / "out"),
        *arguments,
    ])
    assert recorder.main() == expected


def test_parallel_startup_timeout_uses_current_600_second_budget():
    assert CollectionConfig(workers=[{}]).simulation_startup_timeout_seconds == 600.
