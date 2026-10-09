import numpy as np
import pytest
from simulation.motion_diagnostics import MotionSpeedMonitor
from space_arm_platform.models import MotionSpeedDiagnostics


def feed(m, start, stop, actual, expected=(.05,0,0,0,0,0), **kw):
    result = None
    for i in range(start,stop):
        result = m.update(i*.01,np.array(expected),np.array(actual),np.array(expected),enabled=True,**kw)
    return result


def test_80_percent_projection_debounce_and_85_percent_recovery():
    m = MotionSpeedMonitor()
    r = feed(m,0,20,[.03,.2,0,0,0,0])
    assert not r["linear"]["warning"]
    r = feed(m,20,100,[.03,.2,0,0,0,0])
    assert r["linear"]["warning"]
    assert r["linear"]["ratio"] == pytest.approx(.6)
    assert r["linear"]["lateral_speed"] == pytest.approx(.2)
    assert r["linear"]["reasons"][0]["code"] == "cause_unconfirmed"
    r = feed(m,100,150,[.041,0,0,0,0,0])
    assert r["linear"]["warning"]
    r = feed(m,150,180,[.045,0,0,0,0,0])
    assert not r["linear"]["warning"]
    MotionSpeedDiagnostics.model_validate(r)


def test_translation_rotation_are_independent_and_reversal_rebaselines():
    m=MotionSpeedMonitor()
    r=feed(m,0,100,[.05,0,0,0,.02,0],expected=(.05,0,0,0,.1,0))
    assert not r["linear"]["warning"]
    assert r["angular"]["warning"]
    r=feed(m,100,105,[.05,0,0,0,.02,0],expected=(-.05,0,0,0,-.1,0))
    assert not r["angular"]["warning"]
    assert r["linear"]["ratio"] == -1


def test_zero_stale_invalid_and_reset_never_use_prediction_as_measurement():
    m=MotionSpeedMonitor()
    r=feed(m,0,100,[0]*6)
    assert r["linear"]["warning"]
    r=feed(m,100,110,[0]*6,stale=True)
    assert not r["linear"]["warning"] and r["command_stale"]
    r=feed(m,110,120,[0]*6,expected=(0,0,0,0,0,0))
    assert r["linear"]["ratio"] is None
    r=feed(m,120,130,[float("nan")]*6)
    assert not r["measurement_valid"] and not r["linear"]["warning"]
    m.reset()
    assert m.last_time is None


def test_slow_plant_gets_tracking_evidence_not_invented_contact_or_singularity():
    m=MotionSpeedMonitor()
    r=feed(m,0,100,[0]*6,measured_joints=np.zeros(6),target_joints=np.full(6,.02))
    codes=[r["code"] for r in r["linear"]["reasons"]]
    assert codes == ["joint_tracking_error"]


def test_actual_threshold_not_scaled_ik_denominator():
    m=MotionSpeedMonitor()
    for i in range(100):
        r=m.update(i*.01,[.05,0,0,0,0,0],[.02,0,0,0,0,0],[.02,0,0,0,0,0],enabled=True,
            solver_reasons=[{"code":"joint_position_limit","joints":[4]}])
    assert r["linear"]["ratio"] == pytest.approx(.4)
    assert r["linear"]["reasons"][0]["joints"] == [4]


def test_saturation_requires_requested_and_applied_torque_evidence():
    m=MotionSpeedMonitor()
    r=feed(m,0,100,[0]*6,actuator_evidence={
        "requested_torque_nm":[3,0,0,0,0,0],"applied_torque_nm":[2,0,0,0,0,0],
        "torque_limits_nm":[2,2,2,1,1,.35]})
    assert r["linear"]["reasons"] == [{"code":"torque_saturation","joints":[1]}]


def test_exactly_eighty_percent_does_not_trigger():
    m=MotionSpeedMonitor()
    r=feed(m,0,100,[.04,0,0,0,0,0])
    assert not r["linear"]["warning"]
