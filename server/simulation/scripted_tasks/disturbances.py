"""Explicit physical-command and measured-observation injection boundaries."""

from collections import deque
from copy import deepcopy
import math
import numpy as np
from .randomization import generator, options_for_scene


class EpisodeDisturbances:
    def __init__(self, scene):
        self.options = options_for_scene(scene)
        self.seed = int(scene.get("seed", 0))
        self.realized = scene.get("collection_config", {}).get(
            "realized_randomization", {}
        )
        self.bias = np.deg2rad(self.realized.get("joint_bias_deg", [0.0] * 6))
        if not self.options.joint_bias.enabled:
            self.bias[:] = 0
        self.samples = deque(maxlen=64)  # > 200 ms at 200 Hz
        self.action_state = np.zeros(6)
        self.action_filtered = np.zeros(6)
        self.action_rng = generator(self.seed, "action_noise")
        self.last_action_time = None
        self.force_message = self.torque_message = None

    def ingest(self, q, v, stamp):
        stamp = int(stamp)
        if self.samples and stamp <= self.samples[-1][0]:
            if stamp == self.samples[-1][0]:
                return
            raise ValueError("Observation samples must be monotonic")
        q, v = np.array(q, copy=True), np.array(v, copy=True)
        q[:6] += self.bias
        cfg = self.options.joint_observation_noise
        if cfg.enabled:
            rng = generator(self.seed, "joint_observation_noise", stamp)
            q[:6] += rng.normal(0, math.radians(cfg.position_std_deg), 6)
            v[:6] += rng.normal(0, math.radians(cfg.velocity_std_deg_s), 6)
        self.samples.append((stamp, q, v))

    def observe(self, q, v, stamp):
        self.ingest(q, v, stamp)
        delay = self.options.observation_delay
        target = int(stamp) - (delay.delay_ms * 1_000_000 if delay.enabled else 0)
        selected = self.samples[0]
        for sample in self.samples:
            if sample[0] > target:
                break
            selected = sample
        return selected[1].copy(), selected[2].copy(), selected[0]

    def wrench(self, reading):
        cfg = self.options.wrist_wrench_noise
        if not cfg.enabled or reading is None or not reading["valid"]:
            return reading
        result = deepcopy(reading)
        rng = generator(
            self.seed, "wrist_wrench_noise", int(reading["interval_end_time_ns"])
        )
        result.update(
            measurement="simulated_noisy_interface_wrench",
            ground_truth_force_n=list(reading["force_n"]),
            ground_truth_torque_nm=list(reading["torque_nm"]),
        )
        result["force_n"] = (
            np.array(reading["force_n"]) + rng.normal(0, cfg.force_std_n, 3)
        ).tolist()
        result["torque_nm"] = (
            np.array(reading["torque_nm"]) + rng.normal(0, cfg.torque_std_nm, 3)
        ).tolist()
        return result

    def action(self, q, v, time_s, lower, upper):
        cfg = self.options.action_noise
        q, v = np.array(q, copy=True), np.array(v, copy=True)
        if not cfg.enabled:
            return q, v
        dt = 0 if self.last_action_time is None else time_s - self.last_action_time
        self.last_action_time = time_s
        if dt < 0 or dt > 0.1:
            raise ValueError("Action disturbance requires monotonic 200 Hz updates")
        if not dt:
            return q, v
        alpha = math.exp(-dt / cfg.correlation_s)
        sigma = math.radians(cfg.std_deg)
        self.action_state = alpha * self.action_state + math.sqrt(
            1 - alpha * alpha
        ) * self.action_rng.normal(0, sigma, 6)
        old = self.action_filtered.copy()
        self.action_filtered = alpha * old + (1 - alpha) * np.clip(
            self.action_state, -3 * sigma, 3 * sigma
        )
        proposed = q[:6] + self.action_filtered
        q[:6] = np.clip(proposed, lower, upper)
        v[:6] += (self.action_filtered - old) / dt
        v[:6] = np.where(proposed != q[:6], 0.0, v[:6])
        return q, v

    def install_external_force(self, stepper):
        if not self.options.external_disturbance.enabled:
            return
        from Basilisk.architecture import messaging

        self.force_message = messaging.CmdForceInertialMsg()
        self.torque_message = messaging.CmdTorqueBodyMsg()
        self.write_external_force(0.0, active=False)
        stepper.add_body_wrench_input(
            self.options.external_disturbance.body,
            self.force_message,
            self.torque_message,
            source="collection_random_disturbance",
        )

    def external_wrench(self, time_s, active=True):
        cfg = self.options.external_disturbance
        if not cfg.enabled or not active:
            return np.zeros(6)
        elapsed = max(0.0, float(time_s)) / cfg.interval_s
        tick = int(elapsed)
        u = elapsed - tick
        blend = u * u * u * (10 + u * (-15 + 6 * u))
        scale = np.array([cfg.force_std_n] * 3 + [cfg.torque_std_nm] * 3)

        def sample(index):
            if index == 0:
                return np.zeros(6)
            return (
                np.clip(
                    generator(self.seed, "external_disturbance", index).normal(size=6),
                    -3,
                    3,
                )
                * scale
            )

        return (1 - blend) * sample(tick) + blend * sample(tick + 1)

    def write_external_force(self, time_s, active=True):
        values = self.external_wrench(time_s, active)
        if self.force_message is not None:
            from Basilisk.architecture import messaging

            f, t = (
                messaging.CmdForceInertialMsgPayload(),
                messaging.CmdTorqueBodyMsgPayload(),
            )
            f.forceRequestInertial = values[:3].tolist()
            t.torqueRequestBody = values[3:].tolist()
            self.force_message.write(f)
            self.torque_message.write(t)
        return values
