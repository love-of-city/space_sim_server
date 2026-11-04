"""Estimated Cartesian navigation and osculating quasi-nonsingular ROE.

ROE convention: Koenig, Guffanti and D'Amico (2016), equation (2):
https://slab.sites.stanford.edu/sites/g/files/sbiybj25201/files/media/file/space2016_koenigguffantidamico.pdf
The chaser is chief, target is deputy, matching target-minus-chaser navigation.
This is a coordinate/uncertainty transformation of a navigation posterior, not
raw carrier processing, a mean-element filter, an attitude sensor or guidance.
"""
import numpy as np

from simulation.orbital_rendezvous.reference import MU


ROE_COMPONENTS = ("delta_a", "delta_lambda", "delta_ex", "delta_ey", "delta_ix", "delta_iy")


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _elements(states, mu):
    """Vectorized (a, mean u, e cos omega, e sin omega, i, RAAN).

    Eccentric longitude avoids dividing by eccentricity at circular orbits.
    The ascending-node basis still requires a non-equatorial elliptic orbit.
    """
    r, v = states[..., :3], states[..., 3:]
    radius = np.linalg.norm(r, axis=-1)
    h = np.cross(r, v)
    hnorm = np.linalg.norm(h, axis=-1)
    if np.any(radius <= 0) or np.any(hnorm <= 0):
        raise ValueError("ROE requires a nondegenerate elliptic orbit")
    normal = h / hnorm[..., None]
    sin_i = np.linalg.norm(normal[..., :2], axis=-1)
    if np.any(sin_i < 1e-5):
        raise ValueError("quasi-nonsingular ROE is undefined near equatorial orbits")
    inverse_a = 2 / radius - np.sum(v*v, axis=-1) / mu
    evec = np.cross(v, h) / mu - r / radius[..., None]
    e2 = np.sum(evec*evec, axis=-1)
    if np.any(inverse_a <= 0) or np.any(e2 >= 1 - 1e-10):
        raise ValueError("ROE requires a bound elliptic orbit")
    a = 1 / inverse_a
    node = np.stack((-normal[..., 1], normal[..., 0], np.zeros_like(sin_i)), axis=-1) / sin_i[..., None]
    transverse = np.cross(normal, node)
    ex = np.sum(evec * node, axis=-1)
    ey = np.sum(evec * transverse, axis=-1)
    eta = np.sqrt(1 - e2)
    beta = 1 / (1 + eta)
    x = np.sum(r * node, axis=-1) / a + ex
    y = np.sum(r * transverse, axis=-1) / a + ey
    cos_f = ((1 - beta*ex*ex)*x - beta*ex*ey*y) / eta
    sin_f = ((1 - beta*ey*ey)*y - beta*ex*ey*x) / eta
    mean_u = np.arctan2(sin_f, cos_f) - ex*sin_f + ey*cos_f
    inclination = np.arctan2(sin_i, normal[..., 2])
    raan = np.arctan2(node[..., 1], node[..., 0])
    return np.stack((a, mean_u, ex, ey, inclination, raan), axis=-1)


def osculating_roe(estimates, *, mu=MU):
    """Return dimensionless ROE and chief a; inputs (..., 2, 6), J2000 m/m/s."""
    states = np.asarray(estimates, dtype=float)
    if states.shape[-2:] != (2, 6) or not np.isfinite(states).all():
        raise ValueError("ROE requires simultaneous target and chaser state estimates")
    if not np.isfinite(mu) or mu <= 0:
        raise ValueError("ROE gravitational parameter must be finite and positive")
    elements = _elements(states, mu)
    deputy, chief = elements[..., 0, :], elements[..., 1, :]
    delta_node = _wrap(deputy[..., 5] - chief[..., 5])
    values = np.stack(((deputy[..., 0] - chief[..., 0]) / chief[..., 0],
                       _wrap(deputy[..., 1] - chief[..., 1] + delta_node*np.cos(chief[..., 4])),
                       deputy[..., 2] - chief[..., 2], deputy[..., 3] - chief[..., 3],
                       deputy[..., 4] - chief[..., 4], delta_node*np.sin(chief[..., 4])), axis=-1)
    return values, chief[..., 0]


def navigation_solution(estimates, covariance, epoch_s, *, reference_point, source,
                        covariance_model="joint posterior with cross covariance", mu=MU):
    """Serialize estimated target-minus-chaser navigation without reading truth.

    Covariance is ordered [target r/v, chaser r/v]. ROE covariance is J P J.T
    from a central-difference Jacobian (0.1 m, 0.1 mm/s); angle differences wrap.
    Invalid orbital geometry leaves Cartesian navigation available and labels
    ROE unavailable. Malformed estimates/covariance are rejected.
    """
    states, P = np.asarray(estimates, dtype=float), np.asarray(covariance, dtype=float)
    if (states.shape != (2, 6) or P.shape != (12, 12)
            or not np.isfinite(states).all() or not np.isfinite(P).all()
            or not np.isfinite(epoch_s) or epoch_s < 0
            or not np.allclose(P, P.T, rtol=1e-10, atol=1e-12)
            or np.linalg.eigvalsh(P).min() < -1e-10):
        raise ValueError("invalid navigation estimate, epoch or joint covariance")
    difference = np.hstack((np.eye(6), -np.eye(6)))
    relative = states[0] - states[1]
    result = {"schema": "spacesim-relative-navigation/1", "epoch_s": float(epoch_s),
              "source": source, "frame": "Earth J2000", "reference_point": reference_point,
              "relative_to": "chaser", "object": "target", "position_m": relative[:3].tolist(),
              "velocity_m_s": relative[3:].tolist(), "velocity_definition": "inertial difference; not rotating-frame derivative",
              "covariance_order": ["x", "y", "z", "vx", "vy", "vz"],
              "covariance": (difference @ P @ difference.T).tolist(), "covariance_model": covariance_model,
              "attitude": {"available": False, "reason": "GNSS position/velocity solutions do not observe spacecraft attitude"}}
    try:
        values, chief_a = osculating_roe(states, mu=mu)
        step = np.tile([.1, .1, .1, .0001, .0001, .0001], 2)
        offsets = np.diag(step).reshape(12, 2, 6)
        plus, _ = osculating_roe(states + offsets, mu=mu)
        minus, _ = osculating_roe(states - offsets, mu=mu)
        delta = plus - minus
        delta[:, 1] = _wrap(delta[:, 1])
        jacobian = (delta / (2*step[:, None])).T
        roe_covariance = jacobian @ P @ jacobian.T
        result["roe"] = {"available": True, "kind": "osculating quasi-nonsingular",
                         "chief": "chaser", "deputy": "target", "components": list(ROE_COMPONENTS),
                         "units": ["1", "rad", "1", "1", "rad", "rad"], "values": values.tolist(),
                         "chief_semimajor_axis_m": float(chief_a), "mu_m3_s2": float(mu),
                         "covariance": (.5*(roe_covariance + roe_covariance.T)).tolist(),
                         "covariance_method": "first-order full joint covariance transformation"}
    except ValueError as error:
        result["roe"] = {"available": False, "reason": str(error)}
    return result


def near_navigation_solution(target, relative, target_covariance, relative_covariance, epoch_s):
    """Use the near-phase estimate and the same uncertainty model as its handoff."""
    target_P, relative_P = np.asarray(target_covariance), np.asarray(relative_covariance)
    states = np.array([target, np.asarray(target)-relative])
    joint = np.block([[target_P, target_P], [target_P, target_P+relative_P]])
    return navigation_solution(states, joint, epoch_s, reference_point="body_origin",
                               source="propagated target estimate and GNSS/stereo relative EKF",
                               covariance_model="target/relative independence approximation used by near-phase handoff")
