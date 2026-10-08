"""Equivalence to the retained Python policy, not only valid/finite outputs."""
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest

from simulation.online_elbow_ik import (
    OnlineElbowPreference, ElbowDeviationState, _apply_online_elbow_preference_python,
    apply_online_elbow_preference,
)
from simulation.native_posture import available, apply_native
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.joint_limits import load_joint_limits

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model/SARM/platform/sarm_platform.xml"
NAMES = tuple(f"joint{i}" for i in range(1, 7))


@pytest.fixture
def model():
    if not available():
        pytest.skip("Build native posture helper to run equivalence tests")
    chain = SerialChainKinematics.from_mjcf(MODEL, base_body="cubesat_bus", joint_names=NAMES, tool_site="sarm_ee")
    limits = load_joint_limits(MODEL, NAMES)
    options = dict(joint_velocity_limits=np.array([.7,.7,.7,.9,1.,1.]),
                   joint_position_min=np.array(limits.lower), joint_position_max=np.array(limits.upper), dt=1/120)
    return chain, options


def assert_equivalent(py, native, state_py, state_native, base=None):
    left, ld = py
    right, rd = native
    assert ld.status == rd.status
    for field in ("joint_velocity_rad_s", "achieved_twist", "residual_twist"):
        np.testing.assert_allclose(getattr(left, field), getattr(right, field), rtol=2e-9, atol=2e-11)
    for key, value in asdict(ld).items():
        other = getattr(rd, key)
        if isinstance(value, (str, bool)):
            assert value == other, key
        else:
            assert value == pytest.approx(other, rel=2e-8, abs=2e-10), key
    if base is not None:
        assert (left is base) == (right is base)
    assert (state_py.position is None) == (state_native.position is None)
    if state_py.position is not None:
        np.testing.assert_allclose(state_py.position, state_native.position, atol=2e-10, rtol=2e-9)
        np.testing.assert_allclose(state_py.rotation, state_native.rotation, atol=2e-10, rtol=2e-9)


@pytest.mark.parametrize("wrist,joint3", [(False,False),(False,True),(True,False),(True,True)])
def test_random_single_step_equivalence(model, wrist, joint3):
    chain, options = model
    rng = np.random.default_rng(20260928)
    pref = OnlineElbowPreference(wrist_enabled=wrist, joint3_enabled=joint3, up_axis=(1.,2.,3.))
    for index in range(250):
        q = rng.uniform(options["joint_position_min"]+.01, options["joint_position_max"]-.01)
        if index < 2:
            q = np.zeros(6) if index == 0 else np.deg2rad([0.,-67.6,-86.6,143.2,-85.5,0.])
        cmd = rng.uniform(-1.,1.,6) * [0.05,0.05,0.05,.5,.5,.5]
        if index % 11 == 0:
            cmd[:] = 0
        base = chain.inverse_velocity_ik_pose(q, cmd, **options)
        a,b = ElbowDeviationState(),ElbowDeviationState()
        left = _apply_online_elbow_preference_python(chain,q,cmd,base,**options,preference=pref,deviation_state=a)
        right = apply_native(chain,q,cmd,base,**options,preference=pref,deviation_state=b)
        assert_equivalent(left,right,a,b,base)


@pytest.mark.parametrize("tight", [False, True])
@pytest.mark.parametrize("shared_geometry", [False, True])
def test_stateful_budget_release_reset_and_trajectory_equivalence(model,tight,shared_geometry):
    chain, options = model
    other = SerialChainKinematics.from_mjcf(MODEL,base_body="cubesat_bus",joint_names=NAMES,tool_site="sarm_ee")
    if shared_geometry:
        other.enable_native_geometry()
    pref = OnlineElbowPreference()
    if tight:
        pref = replace(pref,maximum_position_offset_m=1e-5,maximum_orientation_offset_rad=1e-4)
    qa,qb = np.zeros(6),np.zeros(6)
    a,b = ElbowDeviationState(),ElbowDeviationState()
    for index in range(700):
        command = np.array([.02,.01,0.,0.,0.,.04])
        if 200 <= index < 260:
            command[:] = 0
        if index == 400:
            a.reset();b.reset()
        ba = chain.inverse_velocity_ik_pose(qa,command,**options)
        bb = other.inverse_velocity_ik_pose(qb,command,**options)
        left = _apply_online_elbow_preference_python(chain,qa,command,ba,**options,preference=pref,deviation_state=a)
        right = apply_native(other,qb,command,bb,**options,preference=pref,deviation_state=b)
        assert_equivalent(left,right,a,b)
        qa += options["dt"] * left[0].joint_velocity_rad_s
        qb += options["dt"] * right[0].joint_velocity_rad_s
        np.testing.assert_allclose(qa,qb,rtol=2e-9,atol=2e-11)


@pytest.mark.parametrize("bad", ["q_nan", "speed_zero", "limits_nan", "outside", "dt_zero", "base_nan"])
def test_native_rejects_invalid_inputs_like_reference(model,bad):
    chain, opts = model
    q=np.zeros(6);cmd=np.array([.02,0.,0.,0.,0.,0.]);pref=OnlineElbowPreference()
    base=chain.inverse_velocity_ik_pose(q,cmd,**opts)
    if bad=="q_nan":q[0]=np.nan
    elif bad=="speed_zero":opts["joint_velocity_limits"][0]=0
    elif bad=="limits_nan":opts["joint_position_min"][0]=np.nan
    elif bad=="outside":q[0]=100
    elif bad=="dt_zero":opts["dt"]=0
    elif bad=="base_nan":base=replace(base,joint_velocity_rad_s=np.full(6,np.nan))
    for function in (_apply_online_elbow_preference_python,apply_native):
        with pytest.raises(ValueError):
            function(chain,q,cmd,base,**opts,preference=pref)


def test_dispatch_is_opt_in_and_invalid_mode_fails(model,monkeypatch):
    chain,opts=model;q=np.zeros(6);cmd=np.array([.02,0.,0.,0.,0.,0.])
    base=chain.inverse_velocity_ik_pose(q,cmd,**opts)
    monkeypatch.setenv("SPACE_SIM_POSTURE_BACKEND","invalid")
    with pytest.raises(ValueError,match="SPACE_SIM_POSTURE_BACKEND"):
        apply_online_elbow_preference(chain,q,cmd,base,**opts)
    monkeypatch.setenv("SPACE_SIM_POSTURE_BACKEND","native")
    actual=apply_online_elbow_preference(chain,q,cmd,base,**opts,preference=OnlineElbowPreference(enabled=False))
    assert actual[0] is base and actual[1].status=="off"


def test_list_up_axis_matches_public_python_configuration(model):
    chain, opts = model
    q=np.zeros(6);cmd=np.array([.02,0.,0.,0.,0.,0.])
    pref=OnlineElbowPreference(up_axis=[1.,2.,3.])
    base=chain.inverse_velocity_ik_pose(q,cmd,**opts)
    a,b=ElbowDeviationState(),ElbowDeviationState()
    left=_apply_online_elbow_preference_python(chain,q,cmd,base,**opts,preference=pref,deviation_state=a)
    right=apply_native(chain,q,cmd,base,**opts,preference=pref,deviation_state=b)
    assert_equivalent(left,right,a,b,base)


def test_native_shared_geometry_matches_independent_python_chain(model):
    python, options = model
    native = SerialChainKinematics.from_mjcf(MODEL,base_body="cubesat_bus",joint_names=NAMES,tool_site="sarm_ee")
    native.enable_native_geometry()
    rng=np.random.default_rng(20260928)
    for _ in range(500):
        q=rng.uniform(options["joint_position_min"],options["joint_position_max"])
        for expected,actual in zip(python._geometry(q),native._geometry(q),strict=True):
            np.testing.assert_allclose(actual,expected,atol=3e-14,rtol=3e-13)
        command=rng.uniform(-.05,.05,6)
        a=python.inverse_velocity_ik_pose(q,command,**options)
        b=native.inverse_velocity_ik_pose(q,command,**options)
        np.testing.assert_allclose(a.joint_velocity_rad_s,b.joint_velocity_rad_s,atol=2e-11,rtol=2e-9)
    # Public arrays remain independent from the shared cached geometry.
    p,r=native.forward(q);p[:]=999;r[:]=999
    expected=python.forward(q)
    actual=native.forward(q)
    np.testing.assert_allclose(actual[0],expected[0],atol=3e-14)
    np.testing.assert_allclose(actual[1],expected[1],atol=3e-14)
