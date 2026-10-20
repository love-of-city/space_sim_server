"""20 Hz observation-driven approach/PBVS and explicitly logical capture FSM."""
import numpy as np
import xml.etree.ElementTree as ET
from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk

from simulation.navigation_gravity import PointMassNavigationGravity
from simulation.orbital_rendezvous.model import assembly_properties
from simulation.pose_ik import _iterate, _rotation_error, ElbowPreference

from .model import JOINTS
from .approach_plan import approach_plan, OpticalCheckpoint
from .navigation import RelativeEKF, SensorSimulator, VisualReadiness, origin_navigation
from .orbital_guidance import guidance_frame, vbar_frame, rotating_acceleration
from .center_kinematics import CenterKinematics, center_relative, skew
from .parameters import PARAMETERS
from .vbar_approach import VbarApproach


def bounded(v, limit):
    return v * min(1., limit / max(float(np.linalg.norm(v)), 1e-12))


def config_rate(config):
    return config.receiver_config.sample_rate_hz if config.receiver_config else 10


class NearController(sysModel.SysModel):
    def __init__(self, scene, ports, chain, mass, handoff, config, model_path, *, gravity_model=None):
        super().__init__()
        self.ModelTag = "nearFieldEstimatedControl"
        self.scene, self.chain, self.mass = scene, chain, mass
        self.gravity = gravity_model or PointMassNavigationGravity()
        root = ET.parse(model_path).getroot()
        self.centers = [CenterKinematics(root.find(f'./worldbody/body[@name="{name}"]'))
                        for name in ('capture_target', 'cubesat_bus')]
        self.vbar = VbarApproach()
        self.visual_quality = VisualReadiness()
        self.pointing_stable_since = None
        self.cooperative_stamp = None
        self.cooperative_rate = None
        self.cooperative_hinge_rate = None
        self.cooperative_rotation = None
        self.cooperative_hinge = 0.
        self.assembly_com_readers = [[(float(b.find("inertial").get("mass")), ports.body_com_state(b.get("name")).addSubscriber())
                                      for b in top.iter("body")] for top in root.findall("./worldbody/body")]
        # COM publishers are primed by the first physics step, after t=0
        # guidance. Admission requires authored-zero joints, so the initial
        # articulated COM is exactly the stowed offset transformed by handoff.
        initial_com = []
        for name, role in (("capture_target", "target"), ("cubesat_bus", "chaser")):
            body = next(b for b in handoff["state"]["bodies"] if b["body_id"] == f"rendezvous/{role}")
            offset = assembly_properties(root.find(f'./worldbody/body[@name="{name}"]'))["center_body_m"]
            initial_com.append(np.array(body["position_m"]) + rbk.EP2C(np.array(body["attitude_wxyz"])).T @ offset)
        self.initial_center_distance = float(np.linalg.norm(initial_com[0] - initial_com[1]))
        self.handoff_time = float(handoff["state"]["sim_time_s"])
        if (config.receiver_config is not None and config.receiver_config.delay_s > 0
                and handoff.get("gnss_transport") is None):
            raise ValueError("nonzero GNSS latency requires an explicit in-flight transport handoff")
        estimates, covariance = origin_navigation(handoff)
        self.target_orbit = estimates[0].copy()
        self.target_covariance = np.asarray(handoff["covariances"][0], dtype=float).copy()
        self.orbital_guidance = guidance_frame(handoff) == "orbital-vbar"
        # Navigation is at body origins, which accelerate around the assembly
        # COM during wheel slew and cooperative target despin. The old quiet
        # fixture's process noise falsely rejected valid measurements here.
        self.nav = RelativeEKF(estimates[0] - estimates[1], covariance,
                               acceleration_sigma_m_s2=.05 if self.orbital_guidance else .0003)
        self.initial_navigation = {"state": self.nav.x.tolist(), "covariance": self.nav.P.tolist()}
        self.initial_hold_frame = vbar_frame(self.target_orbit)[0].T @ self.nav.x[:3] if self.orbital_guidance else None
        self.sensors = SensorSimulator(config, receiver_state=handoff.get("gnss_receiver_state"), epoch_s=self.handoff_time,
                                       transport_state=handoff.get("gnss_transport"))
        self.config = config
        self.readers = [scene.getBody(n).getOrigin().stateOutMsg.addSubscriber() for n in ("capture_target", "cubesat_bus")]
        self.grasp_reader = scene.getSite("capture_target_grasp").stateOutMsg.addSubscriber()
        self.joint_readers = [ports.joint_state(j).addSubscriber() for j in JOINTS]
        self.joint_rate_readers = [ports.joint_rate(j).addSubscriber() for j in JOINTS]
        self.hinge_reader = ports.joint_state("outer_panel_hinge").addSubscriber()
        self.hinge_rate_reader = ports.joint_rate("outer_panel_hinge").addSubscriber()
        # Cooperative target holds its existing articulated panel. Equal and
        # opposite bounded torques model a servo, without editing the joint.
        self.panel_torque = messaging.CmdTorqueBodyMsg()
        self.panel_reaction = messaging.CmdTorqueBodyMsg()
        for msg in (self.panel_torque, self.panel_reaction):
            msg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., 0., 0.]))
        ports.add_body_wrench("satellite_outer_panel", source="cooperative_hinge_hold", torque_body=self.panel_torque)
        ports.add_body_wrench("capture_target", source="cooperative_hinge_reaction", torque_body=self.panel_reaction)
        self.force_messages = [messaging.SingleActuatorMsg() for _ in range(3)]
        self.target_torque = messaging.CmdTorqueBodyMsg()
        self.target_torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., 0., 0.]))
        ports.add_body_wrench("capture_target", source="cooperative_target_aocs", torque_body=self.target_torque)
        target_initial = next(b for b in handoff["state"]["bodies"] if b["body_id"] == "rendezvous/target")
        self.target_reference_rotation = rbk.EP2C(np.array(target_initial["attitude_wxyz"])).T
        for axis, message in enumerate(self.force_messages):
            message.write(messaging.SingleActuatorMsgPayload(input=0.))
            ports.drive_actuator(f"orbital_force_{axis}", message, owner="near_translation")
        self.qref, self.dqref = np.zeros(8), np.zeros(8)
        self.phase, self.phase_start = "FarHandoff", 0.
        self.transitions = [{"time_s": self.handoff_time, "phase": self.phase}]
        self.force = np.zeros(3)
        self.previous = None
        self.last_gnss, self.last_vision = -1e6, -1e6
        self.visual_streak, self.visual_dominant = 0, False
        self.visual_ready = False
        self.optical = {}
        self.rotation_noise = np.zeros(3)
        self.dwell = 0.
        self.hold_delta = None
        self.deploy_q = None
        self.target_rotation = self.target_reference_rotation.copy()
        self.cooperative_rotation = self.target_rotation.copy()
        self.cooperative_rate = list(target_initial['angular_velocity_rad_s'])
        self.cooperative_hinge_rate = 0.
        self.cooperative_stamp = 0.
        self.target_hinge_observation = 0.
        self.arm_goal_gap = 0.15
        self.latch = False
        self.close_inhibited = False
        self.rows = []
        self.zero_tool_rotation = chain.forward(np.zeros(6))[1]
        self.plan = approach_plan(chain)
        self.nominal_grasp = np.array(self.plan["nominal_grasp_bus_m"])
        self.optical_checkpoint = OpticalCheckpoint()
        self.platform_stable_since = None

    def fail(self, reason, now, state=None):
        if self.phase in {'SafeHold', 'CaptureHold'}:
            return
        self.close_inhibited = True
        self.pending_failure_reason = reason
        self.hold_delta = self.nav.x[:3].copy()
        if self.orbital_guidance and state is not None:
            self.vbar.fail(reason, state)
        self.transition('SafeHold', now)

    def reference(self, _t):
        return self.qref.copy(), self.dqref.copy()

    def attitude_reference(self, _t):
        rotation, omega = vbar_frame(self.target_orbit)
        return rotation, omega, np.zeros(3)

    def transition(self, phase, now):
        if self.phase != phase:
            self.phase, self.phase_start, self.dwell = phase, now, 0.
            self.transitions.append({"time_s": self.handoff_time + now, "phase": phase})
            if phase == "SafeHold" and self.orbital_guidance:
                self.safe_hold_frame = vbar_frame(self.target_orbit)[0].T @ self.nav.x[:3]

    def UpdateState(self, nanos):
        now = nanos * 1e-9
        dt = 0. if self.previous is None else now - self.previous
        self.previous = now
        payloads = [r() for r in self.readers]
        hinge = float(self.hinge_reader().state)
        hinge_rate = float(self.hinge_rate_reader().state)
        hinge_torque = float(np.clip(-0.10 * hinge - 0.10 * hinge_rate, -0.02, 0.02))
        self.panel_torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., 0., hinge_torque]), nanos)
        self.panel_reaction.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., 0., -hinge_torque]), nanos)
        target_r = rbk.MRP2C(payloads[0].sigma_BN).T
        orbital_rotation, orbital_rate = (vbar_frame(self.target_orbit) if self.orbital_guidance else (np.eye(3), np.zeros(3)))
        if self.orbital_guidance:
            self.target_reference_rotation = orbital_rotation
        target_error_body = target_r.T @ _rotation_error(self.target_reference_rotation @ target_r.T)
        target_rate_error = np.array(payloads[0].omega_BN_B) - target_r.T @ orbital_rate
        target_torque = bounded(0.02 * target_error_body - 0.12 * target_rate_error, 0.02)
        self.target_torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=target_torque.tolist()), nanos)
        truth = np.array([np.r_[p.r_BN_N, p.v_BN_N] for p in payloads])
        q = np.array([r().state for r in self.joint_readers])
        dq = np.array([r().state for r in self.joint_rate_readers])
        rb = rbk.MRP2C(payloads[1].sigma_BN).T  # existing perfect attitude measurement assumption
        ptcp, rtool = self.chain.forward(q[:6])  # existing exact encoder assumption
        rtool_n = rb @ rtool
        grasp = np.array(self.grasp_reader().r_BN_N)
        tcp = truth[1, :3] + rb @ ptcp
        truth_relative = truth[0] - truth[1]
        stereo_rotation = rb @ self.zero_tool_rotation
        grasp_tool = stereo_rotation.T @ (grasp - truth[1, :3])
        gradient = self.gravity.gradient(self.target_orbit[:3], now-dt)
        # Despin and arm deployment accelerate body origins relative to their
        # assembly COMs. Once pointing is acquired, use the settled-flight
        # disturbance model; retaining slew noise makes velocity estimates too
        # noisy to certify a five-second stationary hold. Never reset P or x.
        if self.orbital_guidance:
            settled = self.pointing_stable_since is not None and now-self.pointing_stable_since >= 2.
            self.nav.acceleration_sigma_m_s2 = .05 if self.phase == 'ArmDeploy' or (self.phase == 'FarHandoff' and not settled) else .003
        absolute_transition = np.eye(6)
        absolute_transition[:3, 3:] = dt*np.eye(3)
        absolute_transition[3:, :3] = dt*gradient
        absolute_transition[:3, :3] += .5*dt*dt*gradient
        acceleration_noise = np.vstack((.5*dt*dt*np.eye(3), dt*np.eye(3)))
        self.target_covariance = (absolute_transition @ self.target_covariance @ absolute_transition.T
            + acceleration_noise @ acceleration_noise.T * self.nav.acceleration_sigma_m_s2**2)
        self.target_covariance = .5*(self.target_covariance+self.target_covariance.T)
        self.nav.predict(dt, -self.force / self.mass, gradient)
        orbit_acceleration = self.gravity.acceleration(self.target_orbit[:3], now-dt)
        self.target_orbit[:3] += self.target_orbit[3:] * dt + 0.5 * orbit_acceleration * dt**2
        self.target_orbit[3:] += 0.5 * dt * (orbit_acceleration + self.gravity.acceleration(self.target_orbit[:3], now))
        # Read the physical publisher's epoch, not this pre-physics callback's
        # clock. They differ by one outer step after initialization. Poll every
        # control tick; SensorSimulator gates acquisition at the receiver rate.
        message_epochs = [int(reader.timeWritten()) for reader in self.readers]
        if message_epochs[0] != message_epochs[1]:
            raise ValueError("paired GNSS body messages have different physical epochs")
        acquisition_time = message_epochs[0]*1e-9
        if acquisition_time <= now+1e-9:
            # Ideal onboard attitude/encoder measurements are time-tagged and
            # delayed with the cooperative state packet, not read by guidance.
            cooperative = {"rotation": target_r.copy(), "hinge": hinge,
                           "angular_rate": list(payloads[0].omega_BN_B), "hinge_rate": hinge_rate}
            packets, link = self.sensors.acquire(now, self.phase, truth_relative, grasp_tool, stereo_rotation, cooperative,
                                                 absolute_truth=truth, acquisition_time_s=acquisition_time)
            vision_updates = []
            for packet in packets:
                accepted = self.nav.update(packet, now)
                if packet['source'] == 'stereo':
                    self.visual_quality.observe(packet, accepted, self.nav.sigma_m, now)
                    vision_updates.append({'stamp': packet['stamp'], 'accepted': bool(accepted),
                                           'sigma_m': self.nav.sigma_m, 'geometry': packet['geometry'],
                                           'nis': self.nav.last_nis})
                if accepted:
                    if packet["source"] == "gnss":
                        self.last_gnss = packet["stamp"]
                        # Cooperative attitude telemetry; perfect measurement
                        # assumption is shared with platform AOCS, not optical validation.
                        self.target_rotation = packet["cooperative"]["rotation"]
                        self.target_hinge_observation = packet["cooperative"]["hinge"]
                        self.cooperative_stamp = packet['stamp']
                        self.cooperative_rotation = self.target_rotation.copy()
                        self.cooperative_hinge = self.target_hinge_observation
                        # Old in-flight packets are not completed from current
                        # truth. Wait for timestamped rate-bearing telemetry.
                        self.cooperative_rate = packet['cooperative'].get('angular_rate')
                        self.cooperative_hinge_rate = packet['cooperative'].get('hinge_rate')
                    else:
                        self.last_vision = packet["stamp"]
                        self.optical = packet["geometry"]
                        self.rotation_noise = packet["rotation_noise"]
        else:
            raise ValueError("sensor body messages are from a future physical epoch")
        # All guidance below this boundary uses estimated state, accepted
        # observations and platform attitude/encoders. Truth above is sensor/evaluation only.
        visual_valid = now - self.last_vision <= 0.35
        link_age_limit = (max(.5, 1.5 / config_rate(self.config) + self.config.receiver_config.delay_s)
                          if self.config.receiver_config else .5)
        link_valid = now - self.last_gnss <= link_age_limit
        self.visual_ready = self.visual_quality.current(now, self.nav.sigma_m)
        self.visual_streak = self.visual_quality.streak
        self.visual_dominant = self.visual_ready  # legacy fixture compatibility; never a fusion weight
        geometry_ready = self.cooperative_rate is not None and self.cooperative_hinge_rate is not None
        if geometry_ready:
            age = now-self.cooperative_stamp
            spin = np.asarray(self.cooperative_rate)*age
            magnitude = np.linalg.norm(spin)
            K = skew(spin/max(magnitude, 1e-15))
            self.target_rotation = self.cooperative_rotation@(np.eye(3)+np.sin(magnitude)*K+(1-np.cos(magnitude))*(K@K))
            self.target_hinge_observation = self.cooperative_hinge+age*self.cooperative_hinge_rate
        # Cooperative target encoder telemetry assumption.
        c, s = np.cos(self.target_hinge_observation), np.sin(self.target_hinge_observation)
        panel_r = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        grasp_local = np.array([-0.0145, -0.07702, 0.04707870013]) + panel_r @ [-0.36, 0.0046, 0]
        estimated_grasp = self.nav.x[:3] + self.target_rotation @ grasp_local
        error_n = estimated_grasp - rb @ ptcp
        goal_rotation = self.target_rotation @ panel_r @ self.zero_tool_rotation
        axis = goal_rotation[:, 2]
        gap = float(axis @ error_n)
        lateral = float(np.linalg.norm(error_n - axis * gap))
        orientation_error = _rotation_error(goal_rotation @ rtool_n.T) + self.rotation_noise
        angle = float(np.linalg.norm(orientation_error))
        final_delta = rb @ self.nominal_grasp - self.target_rotation @ grasp_local
        platform_error = self.nav.x[:3] - final_delta
        optical_axis = stereo_rotation[:, 2]
        optical_depth = float(optical_axis @ estimated_grasp + .10)
        checkpoint_delta = final_delta + optical_axis * (
            self.plan["optical_checkpoint_depth_m"] - (optical_axis @ (rb @ self.nominal_grasp) + .10))
        checkpoint_error = self.nav.x[:3] - checkpoint_delta
        checkpoint_complete = self.optical_checkpoint.completed_at_s is not None
        rotating_velocity = self.nav.x[3:] - np.cross(orbital_rate, self.nav.x[:3])
        speed = float(np.linalg.norm(rotating_velocity))
        if now >= PARAMETERS.duration_s:
            self.fail('near_mission_timeout', now)
        if self.phase == 'PlatformHold' and now-self.phase_start >= PARAMETERS.node_timeout_s:
            self.fail('platform_residence_timeout', now)
        late = self.phase in {"PlatformHold", "ArmDeploy", "ArmAlign", "FinalApproach", "ContactProxy", "Grasp", "CaptureConfirm"}
        if ((late and (not visual_valid or (self.orbital_guidance and not self.visual_ready))) or (now > 0.6 and not link_valid)
                or (self.phase != "FarHandoff" and self.nav.sigma_m > 2.0)):
            if self.phase not in {"SafeHold", "CaptureHold"}:
                reason = ('visual_stale_or_not_ready' if late and not self.visual_ready
                          else 'link_age' if not link_valid else 'navigation_uncertainty')
                self.fail(reason, now)
        chaser_angle = np.linalg.norm(_rotation_error(orbital_rotation @ rb.T))
        target_angle = np.linalg.norm(target_error_body)
        pointing_ready = (not self.orbital_guidance or
                          (chaser_angle < np.deg2rad(1) and target_angle < np.deg2rad(1)
                           and np.linalg.norm(target_rate_error) < .001))
        chaser_rate_error = np.array(payloads[1].omega_BN_B)-rb.T@orbital_rate
        pointing_ready = pointing_ready and (not self.orbital_guidance or np.linalg.norm(chaser_rate_error) < .001)
        if pointing_ready:
            if self.pointing_stable_since is None:
                self.pointing_stable_since = now
        else:
            self.pointing_stable_since = None
        vbar_acceleration = None
        near_guidance = {}
        if self.orbital_guidance:
            target_center = self.centers[0].evaluate({'outer_panel_hinge': self.target_hinge_observation},
                {'outer_panel_hinge': self.cooperative_hinge_rate or 0.})
            chaser_center = self.centers[1].evaluate(dict(zip(JOINTS, q)), dict(zip(JOINTS, dq)))
            com_state, com_covariance = center_relative(self.nav.x, self.nav.P, orbital_rotation, orbital_rate,
                self.target_rotation, rb, np.asarray(self.cooperative_rate or [0., 0., 0.]),
                np.array(payloads[1].omega_BN_B), target_center, chaser_center)
            early = self.phase in {'FarHandoff', 'GNSSApproach', 'VisualFusion', 'VisualApproach', 'SafeHold'}
            if self.phase == 'SafeHold' and self.vbar.failure_reason is None:
                self.vbar.fail(getattr(self, 'pending_failure_reason', 'navigation_or_link_failure'), com_state)
            if early:
                # Freeze the authored stowed geometry in TNR. Runtime COM
                # measurements still use current encoders and timestamped rates.
                ct, cb = target_center[0], chaser_center[0]
                hold_com = self.nominal_grasp-grasp_local+ct-cb
                depth_origin = float(self.zero_tool_rotation[:, 2]@(grasp_local-ct+cb)+.10)
                optical_s = lambda depth: (depth-depth_origin)/self.zero_tool_rotation[0, 2]
                platform_ready = (abs(gap-.5) < .006 and np.linalg.norm(platform_error) < .006)
                common = (pointing_ready and link_valid and np.max(np.abs(q[:6])) < .005
                          and abs(self.target_hinge_observation) < .005
                          and geometry_ready and abs(self.cooperative_hinge_rate or 0.) < .001)
                if self.phase == 'SafeHold':
                    common = pointing_ready and link_valid and geometry_ready
                reserve = np.linalg.norm(gradient@self.nav.x[:3]
                    - rotating_acceleration(orbital_rate, self.nav.x[:3], rotating_velocity))
                old_phase = self.phase
                reference, ref_velocity, ref_acceleration = self.vbar.update(now, com_state, com_covariance,
                    common=common, sigma=self.nav.sigma_m, visual_valid=visual_valid,
                    visual_ready=self.visual_ready, optical_depth=optical_depth, optical_s=optical_s,
                    hold=hold_com, offset_end_s=optical_s(PARAMETERS.offset_end_depth_m),
                    geometry_ready=geometry_ready, available_acceleration=PARAMETERS.force_limit_n/self.mass-reserve,
                    observation_age=max(0., now-self.last_gnss), platform_ready=platform_ready)
                near_guidance = self.vbar.telemetry
                near_guidance['covariance'] = com_covariance.tolist()
                if self.vbar.failure_reason:
                    self.fail(self.vbar.failure_reason, now, com_state)
                elif old_phase == 'FarHandoff' and self.vbar.acquire.completed_at_s is not None:
                    self.transition('GNSSApproach', now)
                elif old_phase == 'GNSSApproach' and self.vbar.gates['Optical17p4m'].completed_at_s is not None:
                    self.transition('VisualFusion', now)
                elif old_phase == 'VisualFusion' and self.visual_ready:
                    self.transition('VisualApproach', now)
                elif old_phase == 'VisualApproach' and self.vbar.gates['PlatformStandoff'].completed_at_s is not None:
                    self.transition('PlatformHold', now)
                self.optical_checkpoint.completed_at_s = self.vbar.gates['Optical17p4m'].completed_at_s
                self.optical_checkpoint.dwell_s = self.vbar.gates['Optical17p4m'].dwell_s
                checkpoint_complete = self.optical_checkpoint.completed_at_s is not None
                # Relative sign is target minus chaser: chaser thrust is the
                # negative requested relative acceleration. Include all rotating
                # terms once and evaluate gravity at COM, not the body origin.
                commanded_relative = (ref_acceleration-PARAMETERS.kp_s2*(com_state[:3]-reference)
                                      -PARAMETERS.kd_s*(com_state[3:]-ref_velocity))
                com_delta_n = orbital_rotation@com_state[:3]
                com_velocity_n = orbital_rotation@com_state[3:]
                orbital_accel = self.gravity.acceleration(self.target_orbit[:3], now)
                omega_dot = (np.cross(self.target_orbit[:3], orbital_accel)/(self.target_orbit[:3]@self.target_orbit[:3])
                             -2*(self.target_orbit[:3]@self.target_orbit[3:])/(self.target_orbit[:3]@self.target_orbit[:3])*orbital_rate)
                vbar_acceleration = (gradient@com_delta_n-orbital_rotation@commanded_relative
                    -rotating_acceleration(orbital_rate, com_delta_n, com_velocity_n)-np.cross(omega_dot, com_delta_n))
            else:
                near_guidance = dict(self.vbar.telemetry, subsegment=self.phase,
                    algorithm='original platform origin hold and arm PBVS/IK', state=com_state.tolist(),
                    covariance=com_covariance.tolist())
            speed = float(np.linalg.norm(com_state[3:])) if early else speed
            desired_velocity = bounded(-.1*platform_error, .005)
        else:
            if self.phase == "FarHandoff" and now >= 5 and self.nav.sigma_m < 0.15 and link_valid and pointing_ready:
                self.transition("GNSSApproach", now)
            if self.phase == "GNSSApproach":
                self.optical_checkpoint.update(error_m=checkpoint_error, speed_m_s=speed,
                    ready=pointing_ready and link_valid, now=now)
            checkpoint_complete = self.optical_checkpoint.completed_at_s is not None
            if self.phase == "GNSSApproach" and checkpoint_complete and visual_valid:
                self.transition("VisualFusion", now)
            if self.phase == "VisualFusion" and self.visual_dominant:
                self.transition("VisualApproach", now)
            if self.phase in {"GNSSApproach", "VisualFusion", "VisualApproach"}:
                approach_error = platform_error if checkpoint_complete else checkpoint_error
                vmax = 0.10 if np.linalg.norm(platform_error) > 10 else 0.02 if np.linalg.norm(platform_error) > 1 else 0.005
                # Without accepted vision, stop with 12 m extra standoff. A lost
                # dominant visual source latches a hold before further closing.
                if self.visual_dominant and not visual_valid:
                    self.hold_delta = self.nav.x[:3].copy()
                    self.close_inhibited = True
                    self.transition("SafeHold", now)
                elif not visual_valid and np.linalg.norm(platform_error) < 12:
                    vmax = 0.
                desired_velocity = bounded(-0.10 * approach_error, vmax)
                platform_ready = (self.visual_dominant and checkpoint_complete and pointing_ready
                    and abs(gap-self.plan["stowed_tcp_gap_m"]) < .006
                    and np.linalg.norm(platform_error) < .006 and speed < .002)
                if platform_ready:
                    self.dwell += dt
                    if self.dwell >= 5:
                        self.transition("PlatformHold", now)
                else:
                    self.dwell = 0.
            else:
                delta = self.hold_delta if self.phase == "SafeHold" else final_delta
                if self.phase == "SafeHold" and self.orbital_guidance:
                    delta = orbital_rotation @ self.safe_hold_frame
                if self.phase == "FarHandoff":
                    if self.hold_delta is None:
                        self.hold_delta = self.nav.x[:3].copy()
                    delta = orbital_rotation @ self.initial_hold_frame if self.orbital_guidance else self.hold_delta
                desired_velocity = bounded(-0.1 * (self.nav.x[:3] - delta), 0.005)
        if self.phase == "PlatformHold":
            platform_ready = (visual_valid and self.visual_ready and link_valid and pointing_ready
                and abs(gap-self.plan["stowed_tcp_gap_m"]) < .006
                and np.linalg.norm(platform_error) < .006 and speed < .002)
            if not platform_ready:
                self.platform_stable_since = None
            elif self.platform_stable_since is None:
                self.platform_stable_since = now
            self.dwell = 0. if self.platform_stable_since is None else now-self.platform_stable_since
        if self.phase == "PlatformHold" and self.dwell >= self.plan["platform_dwell_s"]:
            target_p = rb.T @ (estimated_grasp - axis * 0.15)
            ik_target_r = rb.T @ goal_rotation
            self.deploy_q = _iterate(self.chain, np.deg2rad([76.15, -64.33, 44.38, -70.05, 76.15, -90]), target_p, ik_target_r,
                                     np.array([-3.1416] * 5 + [-6.2832]), np.array([3.1416] * 5 + [6.2832]), ElbowPreference())
            if self.deploy_q is None:
                self.fail('deployment_ik_failed', now)
            else:
                self.transition("ArmDeploy", now)
        self.dqref[:] = 0.
        if self.phase == "ArmDeploy":
            self.dqref[:6] = 0.5 * (self.deploy_q - q[:6])
            self.dqref[:6] *= min(1., 0.05 / max(np.max(np.abs(self.dqref[:6])), 1e-12))
            if np.max(np.abs(self.deploy_q - q[:6])) < 0.005:
                self.transition("ArmAlign", now)
        if self.phase in {"ArmAlign", "FinalApproach", "ContactProxy", "Grasp", "CaptureConfirm", "CaptureHold"}:
            if self.phase == "FinalApproach":
                self.arm_goal_gap = max(0.018, self.arm_goal_gap - 0.001 * dt)
            twist_n = np.r_[bounded(0.8 * (error_n - axis * self.arm_goal_gap), 0.005 if self.phase == "ArmAlign" else 0.0015),
                              bounded(0.8 * orientation_error, 0.02)]
            twist_b = np.r_[rb.T @ twist_n[:3], rb.T @ twist_n[3:]]
            result = self.chain.inverse_velocity_ik_pose(q[:6], twist_b, joint_velocity_limits=np.full(6, 0.08),
                       joint_position_min=np.array([-3.1416] * 5 + [-6.2832]),
                       joint_position_max=np.array([3.1416] * 5 + [6.2832]), dt=max(dt, 0.05))
            self.dqref[:6] = result.joint_velocity_rad_s
            if self.phase == "ArmAlign" and abs(gap - 0.15) < 0.004 and lateral < 0.004 and angle < np.deg2rad(1):
                self.dwell += dt
                if self.dwell > 2:
                    self.transition("FinalApproach", now)
            elif self.phase == "ArmAlign":
                self.dwell = 0.
            contact = 0.012 < gap < 0.021 and lateral < 0.005 and angle < np.deg2rad(1) and speed < 0.003 and visual_valid
            if self.phase == "FinalApproach" and contact and self.arm_goal_gap <= 0.019:
                self.transition("ContactProxy", now)
            if self.phase == "ContactProxy" and contact and now - self.phase_start > 1:
                self.transition("Grasp", now)
            if self.phase == "Grasp" and not contact:
                self.fail('grasp_contact_window_lost', now)
            if self.phase == "Grasp" and np.max(q[6:]) < 0.006 and now - self.phase_start > 2:
                self.transition("CaptureConfirm", now)
            if self.phase == "CaptureConfirm":
                if not contact or np.max(q[6:]) > 0.008:
                    self.fail('capture_confirmation_failed', now)
                elif now - self.phase_start >= 3:
                    self.latch = True
                    self.transition("CaptureHold", now)
        closing = self.phase in {"Grasp", "CaptureConfirm", "CaptureHold"} and not self.close_inhibited
        finger_goal = 0.002 if closing else 0.0375
        # Once closing is inhibited, even a small measured overshoot of the
        # open limit must not command a closing correction.
        self.dqref[6:] = np.clip(2 * (finger_goal - q[6:]), 0. if self.close_inhibited else -0.005, 0.005)
        self.qref += self.dqref * dt
        self.qref = np.clip(self.qref, q - 0.03, q + 0.03)
        self.qref[:6] = np.clip(self.qref[:6], [-3.1416] * 5 + [-6.2832], [3.1416] * 5 + [6.2832])
        self.qref[6:] = np.clip(self.qref[6:], 0, 0.0375)
        acceleration = (gradient @ self.nav.x[:3] + 0.6 * (rotating_velocity - desired_velocity)
                        - rotating_acceleration(orbital_rate, self.nav.x[:3], rotating_velocity))
        if vbar_acceleration is not None:
            acceleration = vbar_acceleration
        self.force = bounded(self.mass * acceleration, PARAMETERS.force_limit_n)
        body_force = rb.T @ self.force
        for message, value in zip(self.force_messages, body_force):
            message.write(messaging.SingleActuatorMsgPayload(input=float(value)), nanos)
        true_error = grasp - tcp
        ct, st = np.cos(hinge), np.sin(hinge)
        true_goal_rotation = target_r @ np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]]) @ self.zero_tool_rotation
        true_axis = true_goal_rotation[:, 2]
        assembly_com = [sum(m * np.array(read().r_BN_N) for m, read in group) / sum(m for m, _ in group)
                        for group in self.assembly_com_readers]
        self.rows.append({"t": self.handoff_time + now, "elapsed_s": now, "phase": self.phase,
            "physical_observation_time_s": self.handoff_time+acquisition_time,
            "navigation_prediction_time_s": self.handoff_time+now,
            "truth": truth.tolist(), "estimate": self.nav.x.tolist(), "covariance": self.nav.P.tolist(),
            "center_distance_m": self.initial_center_distance if now == 0 else float(np.linalg.norm(assembly_com[1] - assembly_com[0])),
            "platform_to_grasp_m": float(np.linalg.norm(estimated_grasp)), "tcp_gap_m": gap,
            "optical_depth_m": optical_depth,
            "truth_optical_depth_m": float(optical_axis @ (grasp-truth[1, :3]) + .10),
            "optical_checkpoint_complete": checkpoint_complete,
            "optical_checkpoint_dwell_s": self.optical_checkpoint.dwell_s,
            "lateral_error_m": lateral, "alignment_error_deg": float(np.rad2deg(angle)),
            "relative_speed_m_s": speed, "position_sigma_m": self.nav.sigma_m,
            "guidance_frame": "orbital-vbar" if self.orbital_guidance else "inertial-fixture",
            "navigation_last_nis": self.nav.last_nis,
            "navigation_acceleration_sigma_m_s2": self.nav.acceleration_sigma_m_s2,
            "navigation_rejection_reasons": dict(self.nav.rejection_reasons),
            "chaser_pointing_error_deg": float(np.rad2deg(chaser_angle)) if self.orbital_guidance else None,
            "target_pointing_error_deg": float(np.rad2deg(target_angle)),
            "vision_valid": visual_valid, "communication_valid": link_valid,
            "navigation_source": "GNSS + stereo fusion; visual ready" if self.visual_ready else "GNSS + stereo fusion; visual not ready" if visual_valid else "relative GNSS" if link_valid else "prediction only",
            "near_guidance": near_guidance, "visual_ready": self.visual_ready,
            "visual_streak": self.visual_streak, "vision_updates": vision_updates,
            "last_vision_stamp_s": self.last_vision, "last_gnss_stamp_s": self.last_gnss,
            "chaser_rate_error_rad_s": float(np.linalg.norm(chaser_rate_error)),
            "target_rate_error_rad_s": float(np.linalg.norm(target_rate_error)),
            "truth_rotations": [target_r.tolist(), rb.tolist()],
            "truth_angular_rates": [list(payloads[0].omega_BN_B), list(payloads[1].omega_BN_B)],
            "joint_rates": dq.tolist(), "target_hinge_rate_rad_s": hinge_rate,
            "optical": self.optical, "capture_state": "LOGICAL_PROXY_HOLD" if self.latch else self.phase,
            "close_inhibited": self.close_inhibited, "joints": q.tolist(), "joint_reference": self.qref.tolist(),
            "force_n": self.force.tolist(), "truth_tcp_gap_m": float(true_axis @ true_error),
            "truth_relative_speed_m_s": float(np.linalg.norm(truth_relative[3:])),
            "truth_alignment_error_deg": float(np.rad2deg(np.linalg.norm(_rotation_error(true_goal_rotation @ rtool_n.T)))),
            "target_aocs_torque_nm": target_torque.tolist(),
            "target_hinge_angle_rad": hinge, "target_hinge_torque_nm": hinge_torque,
            "truth_lateral_error_m": float(np.linalg.norm(true_error - true_axis * (true_axis @ true_error))),
            "navigation_error_m": float(np.linalg.norm(self.nav.x[:3] - truth_relative[:3])),
            **(self.sensors.transport_telemetry() if self.config.receiver_config is not None
               and self.config.receiver_config.delay_s > 0 else {})})
