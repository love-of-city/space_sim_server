"""Shared pose IK for task planners; never changes simulation state."""

import numpy as np
from simulation.serial_chain_kinematics import rotation_matrix_to_vector


def solve_pose(chain, position, rotation, seed, lower, upper):
    """Small damped pose IK used only to plan actuator references."""
    q = np.clip(np.asarray(seed, dtype=float), lower + 1e-7, upper - 1e-7)
    for _ in range(240):
        p, r = chain.forward(q)
        e = np.r_[position - p, 0.2 * rotation_matrix_to_vector(rotation @ r.T)]
        if np.linalg.norm(e[:3]) < 2e-6 and np.linalg.norm(e[3:]) < 2e-6:
            return q
        jac = chain.jacobian(q).copy()
        jac[3:] *= 0.2
        delta = jac.T @ np.linalg.solve(jac @ jac.T + 1e-8 * np.eye(6), e)
        delta *= min(1.0, 0.15 / max(np.max(np.abs(delta)), 1e-12))
        for scale in (1.0, 0.5, 0.25, 0.1, 0.02):
            candidate = np.clip(q + scale * delta, lower + 1e-7, upper - 1e-7)
            cp, cr = chain.forward(candidate)
            ce = np.r_[position - cp, 0.2 * rotation_matrix_to_vector(rotation @ cr.T)]
            if np.linalg.norm(ce) < np.linalg.norm(e):
                q = candidate
                break
        else:
            break
    raise ValueError(
        f"No continuous IK solution for target {np.round(position, 5).tolist()}"
    )
