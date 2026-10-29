import numpy as np
import pytest

from simulation.pointing_handoff import PointingHandoff,PointingHandoffConfig


def scenario(*,diagnostic=False):
    return PointingHandoff([18,0,0],[17.4,0,0],
        PointingHandoffConfig(dwell_s=.1,visual_wait_timeout_s=.3),end_com=[17.,0,0],
        gnss_diagnostic_continuation=diagnostic)


def tick(sequence,t,p=17.4,v=0.,**kwargs):
    return sequence.update(t,[p,0,0,v,0,0],position_sigma_m=.01,
        inputs_valid=True,service_ready=True,corridor_ready=True,**kwargs)


def reach_pointing(sequence):
    for t in (0.,.05,.1):
        tick(sequence,t,p=18)
    assert sequence.phase=='StationApproach'
    t=.15
    while sequence.phase=='StationApproach':
        sample=sequence.leg.sample(t-sequence.phase_start)
        tick(sequence,t,p=sample[0],v=sample[1]);t+=.05
    for _ in range(3):
        tick(sequence,t);t+=.05
    assert sequence.phase=='TargetPointing'
    return t


def test_target_rotation_only_starts_after_arrival_and_stationary_dwell():
    seq=scenario();t=reach_pointing(seq)
    assert seq.target_pointing_enabled
    assert [e['phase'] for e in seq.events]==['NavigationAcquire','StationApproach','StationSettle','TargetPointing']
    for _ in range(10):
        reference,velocity,_=tick(seq,t,prealigned=False);t+=.05
        assert seq.phase=='TargetPointing'
        np.testing.assert_array_equal(reference,[17.4,0,0])
        np.testing.assert_array_equal(velocity,0)


def test_prealigned_is_not_visual_ready_and_never_authorizes_translation():
    seq=scenario();t=reach_pointing(seq)
    for _ in range(3):
        tick(seq,t,prealigned=True);t+=.05
    assert seq.phase=='VisualWait'
    for _ in range(9):
        ref,vel,_=tick(seq,t,prealigned=True);t+=.05
        assert not seq.telemetry['formal_visual_release']
        assert vel[0]>=-1e-10
    assert seq.phase=='SafeHold' and seq.failure=='visual_not_ready_after_pointing'
    tick(seq,t,prealigned=True,visual_ready=True,visual_stamp_s=t)
    assert seq.phase=='SafeHold'


def test_alignment_interruption_resets_continuous_release_dwell():
    seq=scenario();t=reach_pointing(seq)
    tick(seq,t,prealigned=True);t+=.05
    tick(seq,t,prealigned=False);t+=.05
    tick(seq,t,prealigned=True);t+=.05
    tick(seq,t,prealigned=True);t+=.05
    assert seq.phase=='TargetPointing'
    tick(seq,t,prealigned=True)
    assert seq.phase=='VisualWait'


@pytest.mark.parametrize('diagnostic',[False,True])
def test_release_requires_real_visual_readiness_or_explicit_diagnostic(diagnostic):
    seq=scenario(diagnostic=diagnostic);t=reach_pointing(seq)
    for _ in range(3):
        tick(seq,t,prealigned=True);t+=.05
    if not diagnostic:
        assert seq.phase=='VisualWait'
        tick(seq,t,prealigned=True,visual_ready=True,visual_stamp_s=t-1);t+=.05
        assert seq.phase=='VisualWait'
    tick(seq,t,prealigned=True,visual_ready=not diagnostic,visual_stamp_s=t)
    assert seq.phase==('GNSSDiagnosticContinue' if diagnostic else 'VisualContinue')
    assert seq.telemetry['formal_visual_release'] is (not diagnostic)
    tick(seq,t+.05,prealigned=False)
    assert seq.phase=='SafeHold'


def test_visual_and_station_joint_dwell_restarts_on_readiness_loss():
    seq=PointingHandoff([18,0,0],[17.4,0,0],
        PointingHandoffConfig(dwell_s=.1,visual_dwell_s=.2,visual_wait_timeout_s=2),end_com=[17.,0,0])
    t=reach_pointing(seq)
    for _ in range(3):tick(seq,t,prealigned=True);t+=.05
    assert seq.phase=='VisualWait'
    for _ in range(3):
        tick(seq,t,prealigned=True,visual_ready=True,visual_stamp_s=t);t+=.05
        assert seq.phase=='VisualWait'
    tick(seq,t,prealigned=True,visual_ready=False);t+=.05
    for _ in range(4):
        tick(seq,t,prealigned=True,visual_ready=True,visual_stamp_s=t);t+=.05
        assert seq.phase=='VisualWait'
    tick(seq,t,prealigned=True,visual_ready=True,visual_stamp_s=t)
    assert seq.phase=='VisualContinue'


@pytest.mark.parametrize('problem',['time_gap','link','navigation','service','corridor'])
def test_invalid_inputs_latch_safe_hold(problem):
    seq=scenario();t=reach_pointing(seq)
    data=dict(position_sigma_m=.01,inputs_valid=True,service_ready=True,corridor_ready=True)
    if problem=='time_gap':t+=1
    if problem=='link':data['inputs_valid']=False
    if problem=='navigation':data['position_sigma_m']=.2
    if problem=='service':data['service_ready']=False
    if problem=='corridor':data['corridor_ready']=False
    seq.update(t,[17.4,0,0,0,0,0],**data)
    assert seq.phase=='SafeHold'
    tick(seq,t+.05,prealigned=True,visual_ready=True,visual_stamp_s=t+.05)
    assert seq.phase=='SafeHold'


def test_large_translation_error_blocks_pointing_completion():
    seq=scenario();t=reach_pointing(seq)
    for _ in range(6):
        tick(seq,t,p=17.5,prealigned=True);t+=.05
    assert seq.phase=='TargetPointing'


def test_initial_uncertainty_waits_at_start_without_permitting_approach():
    seq=scenario()
    for t in (0.,.05,.1):
        ref,vel,_=seq.update(t,[18,0,0,0,0,0],position_sigma_m=.03,
            inputs_valid=True,service_ready=True,corridor_ready=False)
        assert seq.phase=='NavigationAcquire'
        assert not seq.target_pointing_enabled
        np.testing.assert_array_equal(ref,[18,0,0])
        np.testing.assert_array_equal(vel,[0,0,0])
    tick(seq,.15,p=18)
    tick(seq,.2,p=18)
    assert seq.phase=='NavigationAcquire'
    tick(seq,.25,p=18)
    assert seq.phase=='StationApproach'


def test_initial_acquisition_has_a_bounded_wait():
    seq=PointingHandoff([18,0,0],[17.4,0,0],
        PointingHandoffConfig(settle_timeout_s=.1))
    for t in (0.,.05,.1,.15):
        seq.update(t,[18,0,0,0,0,0],position_sigma_m=.2,
            inputs_valid=True,service_ready=True,corridor_ready=False)
    assert seq.phase=='SafeHold'
    assert seq.failure=='navigation_acquire_timeout'
    assert not seq.target_pointing_enabled


def test_nonfinite_limits_rejected():
    with pytest.raises(ValueError):PointingHandoffConfig(dwell_s=np.nan)
