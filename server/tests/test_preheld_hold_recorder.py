"""Hold adapter tests exercise safety and reference semantics without native dynamics."""
from types import SimpleNamespace
import numpy as np
import pytest

from tools.diagnostics.run_preheld_hold import HoldMonitor


def sample(index,wrench=None,valid=True):
    return SimpleNamespace(time_s=index*.005,joints=np.full(8,.3),
                           wrench=np.zeros(6) if wrench is None else np.array(wrench),wrench_valid=valid)


def test_servo_deflection_never_changes_fixed_reference():
    initial=np.linspace(-.2,.2,8);task=HoldMonitor(initial,.1)
    for i in range(21):
        command,velocity=task.step(sample(i))
        np.testing.assert_array_equal(command,initial)
        np.testing.assert_array_equal(velocity,np.zeros(8))
    assert task.result=='success' and task.stage=='hold_completed'
    assert task.metrics(sample(20))['full_task_accepted'] is False


@pytest.mark.parametrize('wrench',[[40.1,0,0,0,0,0],[0,0,0,4.01,0,0]])
def test_continuous_original_wrist_limits_stop_before_duration(wrench):
    task=HoldMonitor(np.zeros(8),2.)
    for i in range(21):task.step(sample(i,wrench))
    assert task.result=='failed' and task.reason=='wrist_load_limit'
    np.testing.assert_array_equal(task.reference,np.zeros(8))


def test_transient_overload_does_not_accumulate_across_clear_interval():
    task=HoldMonitor(np.zeros(8),.2)
    for i in range(41):
        task.step(sample(i,[41,0,0,0,0,0] if i<10 or 20<=i<30 else None))
    assert task.result=='success'


def test_only_time_zero_sensor_bootstrap_may_be_invalid():
    task=HoldMonitor(np.zeros(8),.1)
    task.step(sample(0,valid=False));assert task.result is None
    task.step(sample(1,valid=False));assert task.reason=='wrist_sensor_unavailable'


def test_missing_200hz_evidence_is_rejected():
    task=HoldMonitor(np.zeros(8),.1)
    task.step(sample(0));task.step(sample(2))
    assert task.result=='failed' and '200hz' in task.reason


@pytest.mark.parametrize('duration',[0.,-.1,.001,float('nan'),float('inf')])
def test_hold_duration_must_end_on_a_real_200hz_sample(duration):
    with pytest.raises(ValueError):HoldMonitor(np.zeros(8),duration)
