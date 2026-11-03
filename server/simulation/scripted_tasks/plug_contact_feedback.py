"""Bounded joint references for plug contact, with separate friction preload.

The nominal reference tracks the requested tool pose. A finite servo offset
compensates the configured axial dry friction; it is never integrated into the
nominal pose. This module only produces actuator references.
"""

import math

import numpy as np

from simulation.serial_chain_kinematics import rotation_matrix_to_vector


class PlugContactFeedback:
    reference_limit_rad = .06
    preload_ramp_m = .004
    preload_filter_s = .1

    def __init__(self, *, joint_stiffness_nm_rad=None, friction_force_n=0.,
                 friction_travel_m=0., extraction_lateral_gain_s_inv=3.,
                 extraction_velocity_gain=8.):
        if (not math.isfinite(friction_force_n) or friction_force_n < 0.
                or not math.isfinite(friction_travel_m) or friction_travel_m < 0.):
            raise ValueError("Contact friction force and travel must be finite and nonnegative")
        self.joint_stiffness_nm_rad = (
            None if joint_stiffness_nm_rad is None
            else np.asarray(joint_stiffness_nm_rad, dtype=float).copy()
        )
        if self.joint_stiffness_nm_rad is not None and (
            self.joint_stiffness_nm_rad.shape != (6,)
            or not np.isfinite(self.joint_stiffness_nm_rad).all()
            or np.any(self.joint_stiffness_nm_rad <= 0.)
        ):
            raise ValueError("Contact feedback requires six finite positive joint stiffnesses")
        if friction_force_n > 0. and (
            self.joint_stiffness_nm_rad is None or friction_travel_m <= 0.
        ):
            raise ValueError("Friction preload requires the actual servo stiffness and friction travel")
        if (not math.isfinite(extraction_lateral_gain_s_inv)
                or not 0. < extraction_lateral_gain_s_inv <= 6.):
            raise ValueError("Extraction lateral gain must be finite and in (0, 6]")
        self.extraction_lateral_gain_s_inv = float(extraction_lateral_gain_s_inv)
        if not math.isfinite(extraction_velocity_gain) or not 0. < extraction_velocity_gain <= 8.:
            raise ValueError("Extraction velocity gain must be finite and in (0, 8]")
        self.extraction_velocity_gain = float(extraction_velocity_gain)
        self.friction_force_n = float(friction_force_n)
        self.friction_travel_m = float(friction_travel_m)
        self.nominal_reference = None
        self.applied_reference = None
        self.preload_weight = 0.
        self.preload_force_n = 0.
        self.measured_axial_velocity_m_s = 0.

    def configuration(self):
        return dict(
            method="nominal-pose-with-bounded-friction-preload/1",
            joint_stiffness_nm_rad=(None if self.joint_stiffness_nm_rad is None
                                    else self.joint_stiffness_nm_rad.tolist()),
            friction_force_n=self.friction_force_n,
            friction_travel_m=self.friction_travel_m,
            reference_limit_rad=self.reference_limit_rad,
            preload_ramp_m=self.preload_ramp_m,
            preload_filter_s=self.preload_filter_s,
            extraction_velocity_gain=self.extraction_velocity_gain,
            extraction_velocity_filter_s=.005,
            extraction_lateral_gain_s_inv=self.extraction_lateral_gain_s_inv,
        )

    def reset(self, reference):
        self.nominal_reference = np.asarray(reference, dtype=float).copy()
        self.applied_reference = self.nominal_reference.copy()
        self.preload_weight = self.preload_force_n = 0.
        self.measured_axial_velocity_m_s = 0.

    def step(self, chain, measured_joints, target_position, target_rotation, *,
             axial_velocity_m_s, seat_gap_m, dt, max_speed_rad_s, lower, upper,
             motion="insert", extraction_travel_m=0., measured_joint_velocities=None,
             preload_allowed=True):
        if self.nominal_reference is None:
            raise RuntimeError("Reset contact feedback at the stage handoff")
        if not math.isfinite(dt) or dt < 0.:
            raise ValueError("Contact feedback dt must be finite and nonnegative")
        if motion not in ("insert", "extract"):
            raise ValueError("Contact feedback motion must be insert or extract")
        position, rotation = chain.forward(self.nominal_reference)
        error = np.r_[target_position-position,
                      .2*rotation_matrix_to_vector(target_rotation @ rotation.T)]
        jacobian = chain.jacobian(self.nominal_reference).copy()
        jacobian[3:] *= .2
        twist = 3.*error
        twist[2] += axial_velocity_m_s
        if motion == "extract":
            if measured_joint_velocities is None:
                raise ValueError("Extraction feedback requires measured joint velocities")
            actual_jacobian = chain.jacobian(measured_joints)
            measured_velocity = float((actual_jacobian[:3] @ measured_joint_velocities)[2])
            self.measured_axial_velocity_m_s += (1.-math.exp(-dt/.005)) * (
                measured_velocity-self.measured_axial_velocity_m_s)
            # Keep the existing extraction damping and measured XY correction
            # while removing accumulated servo preload from the nominal path.
            twist[2] += self.extraction_velocity_gain*(
                axial_velocity_m_s-self.measured_axial_velocity_m_s)
            actual_position, _ = chain.forward(measured_joints)
            twist[:2] += self.extraction_lateral_gain_s_inv*(target_position-actual_position)[:2]
        velocity = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-6*np.eye(6), twist)
        velocity *= min(1., max_speed_rad_s / max(float(np.max(np.abs(velocity))), 1e-12))

        bias = np.zeros(6)
        if self.friction_force_n:
            if motion == "extract":
                # Keep compensation throughout the friction zone; unloading
                # before breakaway can strand the part below static friction.
                u = float(np.clip((extraction_travel_m-self.friction_travel_m)
                                  / self.preload_ramp_m, 0., 1.))
                desired_weight = 1.-u*u*(3.-2.*u)
            else:
                # Start loading before the configured friction zone. The
                # finite preload unloads through the existing release stage.
                u = float(np.clip((self.friction_travel_m + self.preload_ramp_m - seat_gap_m)
                                  / self.preload_ramp_m, 0., 1.))
                desired_weight = u*u*(3.-2.*u)
            if not preload_allowed:
                desired_weight = 0.
            self.preload_weight += (1.-math.exp(-dt/self.preload_filter_s)) * (
                desired_weight-self.preload_weight)
            self.preload_force_n = self.friction_force_n*self.preload_weight
            force = np.array([0., 0., (1. if motion == "extract" else -1.)*self.preload_force_n])
            bias = chain.jacobian(measured_joints)[:3].T @ force / self.joint_stiffness_nm_rad

        candidate = self.nominal_reference + dt*velocity + bias
        step = candidate-self.applied_reference
        step *= min(1., max_speed_rad_s*dt / max(float(np.max(np.abs(step))), 1e-12))
        candidate = self.applied_reference+step
        delta = candidate-measured_joints
        # Scale the complete offset: clipping joints independently changes the
        # requested Cartesian direction and can pitch the wrist into the bore.
        delta *= min(1., self.reference_limit_rad / max(float(np.max(np.abs(delta))), 1e-12))
        reference = np.clip(measured_joints+delta, lower, upper)
        # Back-calculate limit interventions, excluding the instantaneous bias.
        self.nominal_reference = reference-bias
        self.applied_reference = reference.copy()
        return reference
