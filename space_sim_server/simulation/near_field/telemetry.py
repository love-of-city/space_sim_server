"""Stable near-only serialization in independent target/relative coordinates."""
import numpy as np

from simulation.relative_navigation import navigation_solution, osculating_roe, _wrap


def near_navigation_solution(target, relative, target_covariance, relative_covariance, epoch_s):
    """Same uncertainty as [[Pt,Pt],[Pt,Pt+Pr]], without subtracting large Pt.

    Propagated absolute uncertainty may be many orders above tightly observed
    relative uncertainty after a 3600 s approach. Constructing that 12x12 block
    loses PSD to roundoff. Transform the independent Pt/Pr coordinates directly;
    neither covariance, posterior nor shared navigation implementation changes.
    """
    Pt, Pr = np.asarray(target_covariance), np.asarray(relative_covariance)
    for covariance in (Pt, Pr):
        if (covariance.shape != (6, 6) or not np.isfinite(covariance).all()
                or not np.allclose(covariance, covariance.T, rtol=1e-10, atol=1e-12)
                or np.linalg.eigvalsh(covariance).min() < -1e-10):
            asymmetry = float(np.max(np.abs(covariance-covariance.T))) if covariance.shape == (6, 6) else None
            eigenvalue = float(np.linalg.eigvalsh(covariance).min()) if covariance.shape == (6, 6) and np.isfinite(covariance).all() else None
            raise ValueError(f'invalid near navigation covariance: min_eigenvalue={eigenvalue}, asymmetry={asymmetry}')
    states = np.array([target, np.asarray(target)-relative])
    result = navigation_solution(states, np.zeros((12, 12)), epoch_s, reference_point='body_origin',
        source='propagated target estimate and GNSS/stereo relative EKF',
        covariance_model='target/relative independence approximation used by near-phase handoff')
    result['covariance'] = Pr.tolist()
    if result['roe']['available']:
        step = np.tile([.1, .1, .1, .0001, .0001, .0001], 2)
        offsets = np.diag(step).reshape(12, 2, 6)
        plus, _ = osculating_roe(states+offsets)
        minus, _ = osculating_roe(states-offsets)
        delta = plus-minus
        delta[:, 1] = _wrap(delta[:, 1])
        J = (delta/(2*step[:, None])).T
        common, relative_J = J[:, :6]+J[:, 6:], -J[:, 6:]
        covariance = common@Pt@common.T+relative_J@Pr@relative_J.T
        result['roe']['covariance'] = (.5*(covariance+covariance.T)).tolist()
        result['roe']['covariance_method'] = 'equivalent first-order independent target/relative covariance transformation'
    return result
