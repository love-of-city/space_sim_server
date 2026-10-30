import numpy as np
import pytest

from simulation.near_field.navigation import RelativeEKF


@pytest.mark.parametrize('time_s', [4.3, 7.5, 8.6, 9.1, 14.5, 17.2])
def test_same_nanosecond_image_epoch_is_not_rejected_as_future(time_s):
    stamp_ns = round(time_s*1e9)
    nav = RelativeEKF(np.zeros(6), np.eye(6)*.001)
    packet = dict(source='stereo', stamp=stamp_ns*1e-9, z=np.zeros(3), R=np.eye(3)*1e-6)
    assert nav.update(packet, time_s)
    assert nav.accepted['stereo'] == 1 and nav.rejection_reasons['age'] == 0


@pytest.mark.parametrize('offset_s', [1e-6, .01, -.350001, float('nan')])
def test_real_future_expired_and_invalid_stamps_still_reject(offset_s):
    nav = RelativeEKF(np.zeros(6), np.eye(6))
    packet = dict(source='stereo', stamp=10.+offset_s, z=np.zeros(3), R=np.eye(3)*1e-6)
    before = nav.x.copy(), nav.P.copy()
    assert not nav.update(packet, 10.)
    np.testing.assert_array_equal(nav.x, before[0])
    np.testing.assert_array_equal(nav.P, before[1])
