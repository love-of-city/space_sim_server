"""Paired, correlated GNSS solution measurements and a joint orbit filter.

The simulator is the only truth consumer. It represents common navigation errors
and receiver residuals at solution level, not raw RF, carrier ambiguities or RTK
fix acquisition. The filter uses measurements/covariance and commanded forces.
"""
import numpy as np

from space_arm_platform.gnss_config import GNSSConfig
from simulation.orbital_rendezvous.reference import MU, gravity


class DifferentialGNSS:
    def __init__(self, config: GNSSConfig, seed=7):
        self.config = config
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, 4821]))
        self.last_time = None
        self.sequence = 0

    def measure(self, truth, time_s):
        states = np.asarray(truth, dtype=float)
        if states.shape != (2, 6) or not np.isfinite(states).all() or not np.isfinite(time_s):
            raise ValueError("GNSS requires two finite, simultaneous position/velocity states")
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError("GNSS observation time must advance")
        cfg = self.config
        absolute = np.array([cfg.absolute_position_sigma_m] * 3 + [cfg.absolute_velocity_sigma_m_s] * 3)
        residual = np.array([cfg.relative_position_sigma_m] * 3 + [cfg.relative_velocity_sigma_m_s] * 3) / np.sqrt(2)
        common = np.sqrt(np.maximum(0, absolute**2 - residual**2))
        baseline = np.linalg.norm(states[0, :3] - states[1, :3])
        rho = float(np.exp(-0.5 * (baseline / cfg.correlation_length_m)**2))
        independent = self.rng.normal(size=(4, 6))
        errors = np.array([
            common * independent[0] + residual * independent[2],
            common * (rho * independent[0] + np.sqrt(max(0, 1-rho*rho)) * independent[1]) + residual * independent[3],
        ])
        observations = states + errors
        marginal = np.diag(absolute**2)
        cross = np.diag(rho * common**2)
        joint = np.block([[marginal, cross], [cross, marginal]])
        self.sequence += 1
        self.last_time = float(time_s)
        return {"stamp": float(time_s), "due": float(time_s + cfg.delay_s),
                "sequence": self.sequence, "frame": "J2000", "observations": observations,
                "covariance": joint, "relative": observations[0] - observations[1],
                "relative_covariance": 2 * (marginal - cross), "correlation": rho,
                "model": "correlated receiver solutions; no raw carrier processing"}

    def snapshot(self):
        return {"config": self.config.to_dict(), "rng_state": self.rng.bit_generator.state,
                "last_time": self.last_time, "sequence": self.sequence}

    def restore(self, saved):
        if saved.get("config") != self.config.to_dict():
            raise ValueError("GNSS configuration changed across the handoff")
        self.rng.bit_generator.state = saved["rng_state"]
        self.last_time, self.sequence = saved["last_time"], saved["sequence"]


class JointOrbitFilter:
    """12-state filter retaining inter-spacecraft covariance for relative flight."""
    def __init__(self):
        self.state = None
        self.covariance = None
        self.last_stamp = None

    def update(self, packet, accelerations):
        z = np.asarray(packet["observations"], dtype=float)
        R = np.asarray(packet["covariance"], dtype=float)
        acceleration = np.asarray(accelerations, dtype=float)
        stamp = float(packet["stamp"])
        if (z.shape != (2, 6) or R.shape != (12, 12) or acceleration.shape != (2, 3)
                or not all(np.isfinite(v).all() for v in (z, R, acceleration)) or not np.isfinite(stamp)
                or not np.allclose(R, R.T) or np.linalg.eigvalsh(R).min() <= 0):
            raise ValueError("invalid paired GNSS observation or covariance")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError("duplicate or out-of-order GNSS observation")
        if self.state is None:
            self.state, self.covariance = z.copy(), R.copy()
        else:
            dt = stamp - self.last_stamp
            F, Q = np.eye(12), np.zeros((12, 12))
            for i, a_control in enumerate(acceleration):
                r, v = self.state[i, :3].copy(), self.state[i, 3:].copy()
                a = gravity(r) + a_control
                rnew = r + dt*v + 0.5*dt*dt*a
                self.state[i] = np.r_[rnew, v + 0.5*dt*(a + gravity(rnew) + a_control)]
                radius = np.linalg.norm(r)
                G = MU * (3*np.outer(r, r)/radius**5 - np.eye(3)/radius**3)
                T = np.eye(6)
                T[:3, 3:] = dt*np.eye(3)
                T[3:, :3] = dt*G
                T[:3, :3] += 0.5*dt*dt*G
                B = np.vstack((0.5*dt*dt*np.eye(3), dt*np.eye(3)))
                j = slice(i*6, (i+1)*6)
                F[j, j], Q[j, j] = T, (B @ B.T) * 0.003**2
            prior = F @ self.covariance @ F.T + Q
            K = np.linalg.solve((prior + R).T, prior.T).T
            self.state = (self.state.reshape(12) + K @ (z - self.state).reshape(12)).reshape(2, 6)
            left = np.eye(12) - K
            P = left @ prior @ left.T + K @ R @ K.T
            self.covariance = 0.5*(P + P.T)
        self.last_stamp = stamp
        return self.state.copy()

    @property
    def relative_covariance(self):
        H = np.hstack((np.eye(6), -np.eye(6)))
        return H @ self.covariance @ H.T

