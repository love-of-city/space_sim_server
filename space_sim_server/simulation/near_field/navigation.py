"""Relative GNSS/stereo measurement simulation, timestamps and a six-state EKF.

This module never consumes images or performs carrier ambiguity resolution.
Truth belongs only to SensorSimulator. Controller code receives delayed packets.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np
from space_arm_platform.gnss_config import GNSSConfig
from simulation.differential_gnss import DifferentialGNSS
from .orbital_guidance import guidance_frame, rotating_relative_state
from .parameters import PARAMETERS


class VisualReadiness:
    """Current accepted distinct-sample quality, never an information weight."""
    def __init__(self):
        self.streak = 0
        self.last_stamp = None
        self.last_receive = None
        self.geometry = {}

    def observe(self, packet, accepted, sigma, now=None):
        stamp = packet['stamp']
        different = self.last_stamp is None or stamp > self.last_stamp+1e-9
        geometry = packet.get('geometry', {})
        quality = (accepted and geometry.get('valid', False) and geometry.get('common_fov', False)
                   and geometry.get('marker_px', 0) >= PARAMETERS.marker_px
                   and geometry.get('depth_sigma_m', np.inf) < PARAMETERS.depth_sigma_m
                   and sigma < PARAMETERS.visual_sigma_m)
        if not quality or not different:
            self.streak = 0
        else:
            uninterrupted = self.last_stamp is None or stamp-self.last_stamp <= PARAMETERS.max_update_gap_s+1e-9
            self.streak = self.streak+1 if uninterrupted else 1
        if different:
            self.last_stamp = stamp
            self.geometry = dict(geometry) if accepted else {}
            self.last_receive = now

    def current(self, now, sigma):
        fresh = self.last_stamp is not None and 0 <= now-self.last_stamp <= PARAMETERS.visual_age_s+1e-9
        if self.last_receive is not None:
            fresh = fresh and 0 <= now-self.last_receive <= PARAMETERS.max_update_gap_s+1e-9
        quality = (fresh and sigma < PARAMETERS.visual_sigma_m and self.geometry.get('valid', False)
                   and self.geometry.get('common_fov', False)
                   and self.geometry.get('marker_px', 0) >= PARAMETERS.marker_px
                   and self.geometry.get('depth_sigma_m', np.inf) < PARAMETERS.depth_sigma_m)
        if not quality:
            self.streak = 0
        return bool(quality and self.streak >= PARAMETERS.visual_samples)


def origin_navigation(handoff):
    """Restore COM estimates at body origins; never reset inherited uncertainty.

    Attitude and lever arms are assumed exact. Distant filters are independent,
    hence P_relative = P_target + P_chaser; correlated inputs need cross terms.
    """
    estimates = np.array(handoff["gnss_estimates_eci"], dtype=float).copy()
    if handoff.get("navigation_reference") != "body_origin" and not handoff.get("source_kind", "").startswith("synthetic"):
        bodies = {b["body_id"]: b for b in handoff["state"]["bodies"]}
        for i, (role, name) in enumerate((("target", "capture_target"), ("chaser", "cubesat_bus"))):
            body = bodies[f"rendezvous/{role}"]
            w, x, y, z = body["attitude_wxyz"]
            R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                          [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                          [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
            center = np.array(handoff["body_properties"][name]["center_body_m"])
            estimates[i, :3] -= R @ center
            estimates[i, 3:] -= R @ np.cross(body["angular_velocity_rad_s"], center)
    marginals = np.asarray(handoff["covariances"], dtype=float)
    if marginals.shape != (2, 6, 6) or not np.isfinite(marginals).all():
        raise ValueError("invalid navigation covariance in handoff")
    cross = handoff.get("gnss_cross_covariance_eci")
    covariance = marginals.sum(axis=0)
    if cross is not None:
        cross = np.asarray(cross, dtype=float)
        if cross.shape != (6, 6) or not np.isfinite(cross).all():
            raise ValueError("invalid cross covariance in handoff")
        joint = np.block([[marginals[0], cross], [cross.T, marginals[1]]])
        if not np.allclose(joint, joint.T) or np.linalg.eigvalsh(joint).min() < -1e-10:
            raise ValueError("handoff joint covariance must be positive semidefinite")
        covariance -= cross + cross.T
    return estimates, covariance


def admit_handoff(handoff):
    """Admit orbital geometry; target/chaser pointing converges under control."""
    estimates, covariance = origin_navigation(handoff)
    RelativeEKF(estimates[0] - estimates[1], covariance)
    bodies = {b["body_id"]: b for b in handoff["state"]["bodies"]}
    target, chaser = [bodies[f"rendezvous/{r}"] for r in ("target", "chaser")]
    delta = np.array(target["position_m"]) - chaser["position_m"]
    if guidance_frame(handoff) == "orbital-vbar":
        relative = np.r_[delta, np.array(target["velocity_m_s"]) - chaser["velocity_m_s"]]
        orbital = rotating_relative_state(np.r_[target["position_m"], target["velocity_m_s"]], relative)
        if not (100 <= orbital[0] <= 125 and np.linalg.norm(orbital[1:3]) <= 5):
            raise ValueError("Near-field requires the 100-125 m V-bar corridor with transverse offset at most 5 m")
        if np.linalg.norm(orbital[3:]) > .2:
            raise ValueError("Handoff rotating-frame speed exceeds 0.2 m/s")
        if any(np.linalg.norm(body["angular_velocity_rad_s"]) > .05 for body in (target, chaser)):
            raise ValueError("Handoff angular rate exceeds the cooperative despin envelope")
        if any(abs(j["position"]) > 1e-8 for j in handoff["stowed_joints"]):
            raise ValueError("Only authored-zero articulated handoff is supported")
        return
    if not (100 <= delta[0] <= 125 and np.linalg.norm(delta[1:]) < 0.6):
        raise ValueError("Near-field autonomy requires the validated 100-125 m +X corridor with transverse offset <0.6 m")
    if np.linalg.norm(np.array(target["velocity_m_s"]) - chaser["velocity_m_s"]) > 0.05:
        raise ValueError("Handoff relative speed exceeds 0.05 m/s")
    for body in (target, chaser):
        if (2*np.arccos(np.clip(abs(body["attitude_wxyz"][0]), 0, 1)) > np.deg2rad(1)
                or np.linalg.norm(body["angular_velocity_rad_s"]) > 0.001):
            raise ValueError("Handoff requires cooperative identity attitude within 1 deg and rate <0.001 rad/s")
    if any(abs(j["position"]) > 1e-8 for j in handoff["stowed_joints"]):
        raise ValueError("Only the validated authored-zero articulated handoff is supported")


@dataclass(frozen=True)
class SensorConfig:
    seed: int = 7
    gnss_sigma_m: float = 0.03  # relative observation, each axis 1 sigma
    gnss_velocity_sigma_m_s: float = 0.002
    gnss_delay_s: float = 0.1
    vision_delay_s: float = 0.15
    loss_after_phase: str = ""
    communication_loss_s: float | None = None
    focal_px: float = 6.0 / 0.00345
    baseline_m: float = 0.12
    marker_size_m: float = 0.08
    disparity_sigma_px: float = 0.2
    receiver_config: GNSSConfig | None = None


class RelativeEKF:
    def __init__(self, state, covariance, *, acceleration_sigma_m_s2=0.0003):
        self.x = np.array(state, dtype=float).copy()
        self.P = np.array(covariance, dtype=float).copy()
        if (self.x.shape != (6,) or self.P.shape != (6, 6)
                or not np.isfinite(self.x).all() or not np.isfinite(self.P).all()
                or not np.allclose(self.P, self.P.T)
                or np.linalg.eigvalsh(self.P).min() < -1e-10):
            raise ValueError("invalid relative navigation handoff")
        self.accepted = {"gnss": 0, "stereo": 0}
        self.rejected = 0
        if not np.isfinite(acceleration_sigma_m_s2) or acceleration_sigma_m_s2 <= 0:
            raise ValueError("process acceleration sigma must be positive")
        self.acceleration_sigma_m_s2 = float(acceleration_sigma_m_s2)
        self.last_nis = None
        self.rejection_reasons = {"age": 0, "innovation": 0}

    def predict(self, dt, acceleration, gradient):
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        F[3:, :3] = gradient * dt
        F[:3, :3] += 0.5 * dt**2 * gradient
        self.x = F @ self.x + np.r_[0.5 * dt**2 * acceleration, dt * acceleration]
        B = np.vstack((np.eye(3) * dt**2 / 2, np.eye(3) * dt))
        self.P = F @ self.P @ F.T + B @ B.T * self.acceleration_sigma_m_s2**2

    def update(self, packet, now):
        age = now - packet["stamp"]
        # Nanosecond image epochs can round a few ulps ahead in float seconds.
        # Allow at most 1 ns representation error, retaining the original gate.
        if not np.isfinite(age) or not -1e-9 <= age <= 0.35+1e-9:
            self.rejected += 1
            self.rejection_reasons["age"] += 1
            return False
        age = float(np.clip(age, 0., .35))
        full = packet["source"] == "gnss"
        H = np.eye(6) if full else np.eye(6)[:3]
        z = packet["z"].copy()
        # Time-tagged first-order transport, with conservative unmodeled
        # acceleration uncertainty. No current truth is used to erase latency.
        z[:3] += age * (z[3:] if full else self.x[3:])
        R = packet["R"].copy()
        R[:3, :3] += np.eye(3) * ((age * 0.003)**2 + (0.5 * age**2 * 0.02)**2)
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        self.last_nis = float(innovation @ np.linalg.solve(S, innovation))
        if self.last_nis > (30 if full else 20):
            self.rejected += 1
            self.rejection_reasons["innovation"] += 1
            return False
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ innovation
        L = np.eye(6) - K @ H
        self.P = L @ self.P @ L.T + K @ R @ K.T
        self.P = (self.P + self.P.T) / 2
        self.accepted[packet["source"]] += 1
        return True

    @property
    def sigma_m(self):
        return float(np.sqrt(np.linalg.eigvalsh(self.P[:3, :3]).max()))


def stereo_geometry(grasp_in_tool, config=SensorConfig()):
    """Project all virtual marker corners into both rectified cameras.

    Stereo mount +Z is bus +X. In this frame, cameras are at
    (0.40 +/- b/2, 0.10, -0.10). They stay fixed on the bus.
    Marker center offset is (0.10, 0.08, 0) in the cooperative grasp frame.
    """
    g = np.asarray(grasp_in_tool)
    center = g + [0.10, 0.08, 0]
    half = config.marker_size_m / 2
    corners = center + np.array([[x, y, 0] for x in (-half, half) for y in (-half, half)])
    common = True
    for side in (-1, 1):
        camera = np.array([0.40 + side * config.baseline_m / 2, 0.10, -0.10])
        rays = corners - camera
        common &= bool(np.all(rays[:, 2] > 0.1))
        uv = config.focal_px * rays[:, :2] / np.maximum(rays[:, 2:], 1e-12)
        common &= bool(np.all(np.abs(uv) < [960 - 16, 600 - 16]))
    depth = max(0.001, float(g[2] + 0.10))
    ranges = [float(np.linalg.norm(center - [0.40 + side * config.baseline_m / 2, 0.10, -0.10])) for side in (-1, 1)]
    pixels = config.focal_px * config.marker_size_m / depth
    disparity = config.focal_px * config.baseline_m / depth
    sigma = np.hypot(0.0005, depth**2 * config.disparity_sigma_px / (config.focal_px * config.baseline_m))
    return {"common_fov": common, "marker_px": float(pixels), "disparity_px": float(disparity),
            "depth_sigma_m": float(sigma), "camera_depth_m": depth, "camera_ranges_m": ranges,
            "valid": common and pixels >= 8 and disparity >= 2}


class SensorSimulator:
    def __init__(self, config, *, receiver_state=None, epoch_s=0.0, transport_state=None):
        self.config = config
        streams = np.random.SeedSequence(config.seed).spawn(2)
        self.gnss_rng, self.vision_rng = [np.random.default_rng(s) for s in streams]
        self.pending = deque()
        self.failed = False
        self.geometry = {}
        self.epoch_s = float(epoch_s)
        self.receiver = DifferentialGNSS(config.receiver_config, config.seed) if config.receiver_config else None
        if receiver_state is not None:
            if self.receiver is None:
                raise ValueError("paired receiver state requires the shared receiver configuration")
            self.receiver.restore(receiver_state)
        self.last_gnss_acquisition = -float('inf')
        self.last_vision_acquisition = -float('inf')
        self.last_delivery_stamp = None
        self.last_delivery_time = None
        self.last_delivery_sequence = None
        self.delivery_count = 0
        self.now = 0.
        self.last_source_time = None
        if transport_state is not None:
            self._restore_transport(transport_state)

    def _restore_transport(self, saved):
        if (self.receiver is None or saved.get("schema") != "spacesim-gnss-transport/1"
                or not np.isfinite(saved.get("epoch_s", float('nan')))
                or abs(saved["epoch_s"]-self.epoch_s) > 1e-8
                or saved.get("reference") not in {"assembled_center_of_mass", "body_origin"}):
            raise ValueError("GNSS transport handoff has an incompatible epoch or reference")
        pending = saved.get("pending")
        if not isinstance(pending, list) or len(pending) > np.ceil(self.receiver.config.delay_s*self.receiver.config.sample_rate_hz)+1:
            raise ValueError("invalid in-flight GNSS packet count")
        previous_sequence = saved.get("gnss_delivered_sequence") or 0
        previous_stamp = saved.get("gnss_sample_time_s")
        restored = []
        for packet in pending:
            stamp, due = float(packet["stamp"]), float(packet["due"])
            sequence = packet["sequence"]
            if (not np.isfinite([stamp, due]).all() or stamp > self.epoch_s+1e-8
                    or due <= self.epoch_s+1e-9 or due < stamp
                    or abs(due-stamp-self.receiver.config.delay_s) > 1e-8
                    or type(sequence) is not int or not previous_sequence < sequence <= self.receiver.sequence
                    or (previous_stamp is not None and stamp <= previous_stamp)
                    or self.receiver.last_time is None or stamp > self.receiver.last_time+1e-8):
                raise ValueError("invalid in-flight GNSS timestamp or sequence")
            if saved["reference"] == "assembled_center_of_mass":
                observations = np.asarray(packet["observations"], dtype=float)
                offsets = np.asarray(packet["origin_offsets"], dtype=float)
                if observations.shape != (2, 6) or offsets.shape != (2, 6):
                    raise ValueError("invalid in-flight GNSS reference-point geometry")
                origins = observations-offsets
                measurement = origins[0]-origins[1]
                covariance = np.asarray(packet["relative_covariance"], dtype=float)
            else:
                measurement = np.asarray(packet["z"], dtype=float)
                covariance = np.asarray(packet["R"], dtype=float)
            cooperative = packet["cooperative"]
            rotation = np.asarray(cooperative["rotation"], dtype=float)
            hinge = float(cooperative["hinge"])
            if (measurement.shape != (6,) or covariance.shape != (6, 6) or rotation.shape != (3, 3)
                    or not all(np.isfinite(value).all() for value in (measurement, covariance, rotation))
                    or not np.isfinite(hinge) or not np.allclose(covariance, covariance.T)
                    or np.linalg.eigvalsh(covariance).min() <= 0
                    or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7)
                    or not np.isclose(np.linalg.det(rotation), 1., atol=1e-7)):
                raise ValueError("invalid in-flight GNSS measurement or cooperative telemetry")
            extra = {}
            if 'angular_rate' in cooperative and 'hinge_rate' in cooperative:
                angular_rate = np.asarray(cooperative['angular_rate'], dtype=float)
                hinge_rate = float(cooperative['hinge_rate'])
                if angular_rate.shape != (3,) or not np.isfinite(angular_rate).all() or not np.isfinite(hinge_rate):
                    raise ValueError('invalid cooperative rate telemetry')
                extra = {'angular_rate': angular_rate.tolist(), 'hinge_rate': hinge_rate}
            restored.append({"source": "gnss", "stamp": stamp-self.epoch_s, "due": due-self.epoch_s,
                             "sequence": sequence, "z": measurement.copy(), "R": covariance.copy(),
                             "cooperative": {"rotation": rotation.copy(), "hinge": hinge, **extra}})
            previous_sequence, previous_stamp = sequence, stamp
        self.pending.extend(restored)
        self.last_gnss_acquisition = self.receiver.last_time-self.epoch_s
        self.last_delivery_stamp = saved.get("gnss_sample_time_s")
        self.last_delivery_time = saved.get("gnss_receive_time_s")
        self.last_delivery_sequence = saved.get("gnss_delivered_sequence")
        self.delivery_count = saved.get("gnss_delivered_count", 0)

    def transport_snapshot(self):
        if self.receiver is None:
            return None
        pending = []
        for packet in self.pending:
            if packet["source"] != "gnss":
                continue
            pending.append({"stamp": self.epoch_s+packet["stamp"], "due": self.epoch_s+packet["due"],
                            "sequence": packet["sequence"], "z": packet["z"].tolist(), "R": packet["R"].tolist(),
                            "cooperative": {**packet['cooperative'],
                                            "rotation": packet["cooperative"]["rotation"].tolist()}})
        return {"schema": "spacesim-gnss-transport/1", "epoch_s": self.epoch_s+self.now,
                "reference": "body_origin", "pending": pending,
                **self.transport_telemetry()}

    def transport_telemetry(self):
        return {
                "gnss_sample_time_s": self.last_delivery_stamp,
                "gnss_receive_time_s": self.last_delivery_time,
                "gnss_measurement_age_s": (self.epoch_s+self.now-self.last_delivery_stamp
                                           if self.last_delivery_stamp is not None else None),
                "gnss_delivered_sequence": self.last_delivery_sequence,
                "gnss_acquired_sequence": self.receiver.sequence if self.receiver else 0,
                "gnss_delivered_count": self.delivery_count,
                "gnss_pending_count": sum(packet["source"] == "gnss" for packet in self.pending),
                "gnss_requested_delay_s": self.receiver.config.delay_s if self.receiver else self.config.gnss_delay_s}

    def acquire(self, now, phase, relative_truth, grasp_in_tool, tool_to_inertial, cooperative=None, *, absolute_truth=None,
                acquisition_time_s=None):
        cfg = self.config
        self.now = float(now)
        captured = self.now if acquisition_time_s is None else float(acquisition_time_s)
        if (not np.isfinite([self.now, captured]).all() or captured < 0 or captured > self.now+1e-9
                or (self.last_source_time is not None and captured < self.last_source_time-1e-9)):
            raise ValueError("sensor source time must be finite, monotonic and no later than delivery time")
        self.last_source_time = captured
        link = cfg.communication_loss_s is None or now < cfg.communication_loss_s
        if phase == cfg.loss_after_phase:
            self.failed = True
        period = 1 / cfg.receiver_config.sample_rate_hz if self.receiver else .1
        if link and captured - self.last_gnss_acquisition >= period - 1e-9:
            self.last_gnss_acquisition = captured
            sigmas = np.array([cfg.gnss_sigma_m] * 3 + [cfg.gnss_velocity_sigma_m_s] * 3)
            if self.receiver:
                if absolute_truth is None:
                    raise ValueError("differential observations require both receiver states")
                stamp = self.epoch_s + captured
                # The first near-field tick is the already-measured terminal
                # far-field epoch. Preserve it instead of generating it twice.
                if self.receiver.last_time is None or stamp > self.receiver.last_time + 1e-9:
                    pair = self.receiver.measure(absolute_truth, stamp)
                    self.pending.append({"source": "gnss", "stamp": captured, "due": captured + cfg.receiver_config.delay_s,
                                         "sequence": pair["sequence"], "z": pair["relative"],
                                         "R": pair["relative_covariance"], "cooperative": cooperative})
            else:
                self.pending.append({"source": "gnss", "stamp": captured, "due": captured + cfg.gnss_delay_s,
                                     "z": relative_truth + self.gnss_rng.normal(size=6) * sigmas,
                                     "R": np.diag(sigmas**2), "cooperative": cooperative})
        geom = stereo_geometry(grasp_in_tool, cfg)
        self.geometry = geom
        if geom["valid"] and not self.failed and captured - self.last_vision_acquisition >= .1 - 1e-9:
            self.last_vision_acquisition = captured
            sigmas = np.array([max(0.0003, geom["camera_depth_m"] * 0.2 / cfg.focal_px)] * 2 + [geom["depth_sigma_m"]])
            covariance = tool_to_inertial @ np.diag(sigmas**2) @ tool_to_inertial.T
            # Equivalent relative-origin observation after calibrated transforms
            # and exact encoder/attitude navigation, an explicit platform assumption.
            z = relative_truth[:3] + tool_to_inertial @ (self.vision_rng.normal(size=3) * sigmas)
            self.pending.append({"source": "stereo", "stamp": captured, "due": captured + cfg.vision_delay_s,
                                 "z": z, "R": covariance, "geometry": geom,
                                 "rotation_noise": self.vision_rng.normal(0, np.deg2rad(0.05), 3)})
        ready, waiting = [], deque()
        while self.pending:
            packet = self.pending.popleft()
            (ready if packet["due"] <= now + 1e-8 else waiting).append(packet)
        self.pending = waiting
        for packet in ready:
            if packet["source"] == "gnss" and self.receiver:
                self.last_delivery_stamp = self.epoch_s+packet["stamp"]
                self.last_delivery_time = self.epoch_s+now
                self.last_delivery_sequence = packet["sequence"]
                self.delivery_count += 1
        return ready, link
