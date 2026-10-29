"""Consumer clock/identity safety, independent of target truth and graphics."""
import copy
import importlib.util

import numpy as np
import pytest

from simulation.own_motion import OwnMotionFeedback
from simulation.serial_chain_kinematics import axis_angle_to_matrix

EPOCH = ('render-a', 'reset-a')


def packet(stamp, rate=(0., 0., .001)):
    return dict(sensor_name='service_gyro', body_name='cubesat_bus', frame='spacecraft_body',
                reference_frame='J2000', measurement='simulated_body_angular_rate',
                sample_rate_hz=50, sample_time_ns=str(stamp), angular_velocity_body_rad_s=list(rate),
                error_variance_rad2_s2=[0., 0., 0.], saturated_axes=[False]*3, valid=True)


def test_upstream_sensor_and_wire_type_are_available():
    assert importlib.util.find_spec('simulation.gyroscope'), 'missing upstream service_gyro'
    from space_arm_platform import models
    assert hasattr(models, 'GyroObservation'), 'missing gyro wire validation'


def run_observer(control_stride):
    observer=OwnMotionFeedback();com=np.array([.2, 0, 0]);reading=None;result=None
    for tick in range(1201):
        now=tick*2_500_000
        if tick%8==0:
            reading=packet(now, (0, 0, .001 if now<2_000_000_000 else .101))
        if tick%control_stride==0:
            result=observer.update(now, reading, com, epoch=EPOCH)
    return observer, result


def test_50hz_samples_match_400hz_reads_and_calibration_is_not_reweighted():
    coarse,(pc,rc)=run_observer(8)
    dense,(pd,rd)=run_observer(1)
    expected=axis_angle_to_matrix(np.array([0,0,1]), .1)
    assert np.allclose(rd, expected, atol=1e-10)
    assert np.allclose(rc,rd,atol=1e-14) and np.allclose(pc,pd,atol=1e-14)
    assert dense.diagnostics()['unique_samples']==151
    assert dense.diagnostics()['calibration_samples']==coarse.diagnostics()['calibration_samples']==76
    assert np.allclose(pd+rd@np.array([.2,0,0]),[.2,0,0])


def test_predicts_between_samples_without_relabelling_or_advancing_anchor():
    observer,_=run_observer(8)
    old=packet(3_000_000_000,(0,0,.101));before=copy.deepcopy(old)
    _,rotation=observer.update(3_002_500_000,old,np.zeros(3),epoch=EPOCH)
    assert np.allclose(rotation,axis_angle_to_matrix(np.array([0,0,1]),.10025),atol=1e-10)
    assert observer.diagnostics()['sample_time_ns']=='3000000000'
    assert observer.diagnostics()['sample_age_ns']==2_500_000
    assert observer.diagnostics()['unique_samples']==151 and old==before
    _,same=observer.update(3_002_500_000,old,np.zeros(3),epoch=EPOCH)
    assert np.array_equal(rotation,same)


@pytest.mark.parametrize('fault,reason',[
    ('missing','missing'),('saturated','invalid'),('invalid','invalid'),('future','future'),
    ('gap','gap'),('reversed','reversed'),('frame','frame'),('nonfinite','invalid'),
    ('epoch','epoch'),('old','stale'),('changed','duplicate'),('truth','invalid')])
def test_faults_are_explicit_without_truth_fallback(fault,reason):
    observer=OwnMotionFeedback();reading=packet(0)
    observer.update(0,reading,np.zeros(3),epoch=EPOCH)
    now=20_000_000;reading=packet(now);epoch=EPOCH
    if fault=='missing':reading=None
    elif fault=='saturated':reading.update(valid=False,saturated_axes=[True,False,False])
    elif fault=='invalid':reading['valid']=False
    elif fault=='future':reading=packet(40_000_000)
    elif fault=='gap':now=40_000_000;reading=packet(now)
    elif fault=='reversed':observer.update(now,reading,np.zeros(3),epoch=EPOCH);now=22_500_000;reading=packet(0)
    elif fault=='frame':reading['frame']='camera'
    elif fault=='nonfinite':reading['angular_velocity_body_rad_s'][0]=float('nan')
    elif fault=='epoch':epoch=('render-b','reset-b')
    elif fault=='old':now=42_500_000;reading=packet(0)
    elif fault=='changed':reading=packet(0,(0,0,.2))
    elif fault=='truth':reading['true_orientation']=[1,0,0,0]
    with pytest.raises(ValueError,match=reason):observer.update(now,reading,np.zeros(3),epoch=epoch)


def test_explicit_reset_clears_integral_and_requires_new_zero_sample():
    observer,_=run_observer(8)
    observer.reset()
    p,r=observer.update(0,packet(0,(0,0,.01)),np.array([.5,0,0]),epoch=('b','b'))
    assert np.array_equal(r,np.eye(3)) and np.array_equal(p,np.zeros(3))
    assert observer.diagnostics()['unique_samples']==1
    assert observer.diagnostics()['reference_spin_estimate_rad_s']==[0.,0.,.01]


def test_gyro_fault_stops_approach_but_does_not_release_force_hold(tmp_path):
    from simulation.visual_predictive_capture import VisualPredictiveCapture
    controller=VisualPredictiveCapture(tmp_path/'missing.json','s',held_pose=True)
    controller.sequence.phase='approach'
    command,status=controller.gyro_unavailable('stale',1.,np.zeros(12),{})
    assert not command['deadman'] and status.startswith('predictive_abort_service_gyro')
    held=VisualPredictiveCapture(tmp_path/'missing.json','s',held_pose=True)
    held.sequence.phase='hold_force';held.sequence.stable_since=0
    feedback=dict(pad_force_n=5.,pad_compression_m=.0003,jaw_force_n=5.,jaw_support_count=4)
    command,status=held.gyro_unavailable('stale',2.,np.zeros(12),feedback)
    assert command['deadman'] and command['capture_radial_velocity_m_s']==0
    assert command['end_effector_linear_velocity_body_m_s']==[0.,0.,0.]
    assert held.sequence.phase=='hold_force' and 'gyro_unavailable' in status


@pytest.mark.parametrize('phase',['tighten','hold_force'])
@pytest.mark.parametrize('fault,reason',[
    ({'peak_target_contact_force_n':11.},'contact_overload'),
    ({'jaw_force_n':21.},'contact_overload'),
    ({'pad_compression_m':.0021},'contact_overload'),
    ({'pad_force_n':float('nan')},'invalid_force_feedback'),
])
def test_gyro_fault_force_hold_retains_all_contact_safety_guards(tmp_path,phase,fault,reason):
    from simulation.visual_predictive_capture import VisualPredictiveCapture
    controller=VisualPredictiveCapture(tmp_path/'missing.json','s',held_pose=True)
    controller.sequence.phase=phase
    feedback=dict(pad_force_n=5.,pad_compression_m=.0003,jaw_force_n=5.,
                  jaw_support_count=4,peak_target_contact_force_n=5.)
    feedback.update(fault)
    command,status=controller.gyro_unavailable('stale',2.,np.zeros(12),feedback)
    assert status=='predictive_abort_'+reason
    assert not command['deadman']
    assert command['capture_radial_velocity_m_s']==0
    assert command['capture_axial_velocity_m_s']==0
