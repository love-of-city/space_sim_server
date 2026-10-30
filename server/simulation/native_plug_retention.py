"""Native arithmetic for the unchanged, message-connected plug guidance law."""
from __future__ import annotations

import ctypes
import math

import numpy as np

from simulation.native_acceleration import load_native_library
from simulation.plug_retention import PlugFriction


class NativePlugFriction(PlugFriction):
    """Same socket state machine and forces; axial friction stays in the core."""

    def __init__(self, sockets, parameters):
        self._state = np.zeros(4, dtype=np.float64)
        super().__init__(sockets, parameters)
        p = parameters
        self._parameters = np.array([
            p.friction_travel_m, p.lateral_stiffness_n_m, p.lateral_damping_ns_m,
            p.lateral_force_limit_n, p.rotational_stiffness_nm_rad,
            p.rotational_damping_nms_rad, p.torsional_damping_nms_rad,
            p.torque_limit_nm, p.backstop_stiffness_n_m, p.backstop_force_limit_n,
            p.capture_radius_m, p.seated_tolerance_m, math.radians(p.capture_angle_deg),
            p.lateral_release_m, math.radians(p.angular_release_deg),
        ], dtype=np.float64)
        self._sockets = np.array([
            [*s.position_body_m, *s.rotation_body.reshape(-1),
             *s.plug_rotation_body.reshape(-1), float(s.initial), float(s.keyed)]
            for s in sockets
        ], dtype=np.float64)
        self._input = np.empty(18, dtype=np.float64)
        self._output = np.empty(6, dtype=np.float64)
        self._library = load_native_library("local_mujoco_stepper")
        self._evaluate = self._library.lms_plug_guidance
        pointer = ctypes.POINTER(ctypes.c_double)
        self._evaluate.argtypes = [pointer, pointer, ctypes.c_int, pointer, pointer, pointer]
        self._evaluate.restype = ctypes.c_int
        self._arguments = (self._input.ctypes.data_as(pointer), self._sockets.ctypes.data_as(pointer),
                           len(sockets), self._parameters.ctypes.data_as(pointer),
                           self._state.ctypes.data_as(pointer), self._output.ctypes.data_as(pointer))
        self._sample = np.empty(26, dtype=np.float64)
        self._coms = np.empty(6, dtype=np.float64)
        self._wrenches = np.empty((4, 3), dtype=np.float64)
        self._evaluate_wrenches = self._library.lms_plug_guidance_wrenches
        self._evaluate_wrenches.argtypes = [pointer, pointer, pointer, ctypes.c_int, pointer, pointer, pointer]
        self._evaluate_wrenches.restype = ctypes.c_int
        self._wrench_arguments = (self._sample.ctypes.data_as(pointer), self._coms.ctypes.data_as(pointer),
                                  *self._arguments[1:5], self._wrenches.ctypes.data_as(pointer))

    def reset(self):
        super().reset()
        self._state[:] = (-1, 1, 0, 0)

    def wrench(self, relative_position, relative_rotation, relative_velocity, relative_omega):
        self._input[:3] = relative_position
        self._input[3:12] = np.asarray(relative_rotation).reshape(9)
        self._input[12:15] = relative_velocity
        self._input[15:18] = relative_omega
        if self._evaluate(*self._arguments) != 0:
            raise ValueError("retention requires finite relative pose and velocity")
        self._update_status()
        # Callers may retain values beyond the next update.
        return self._output[:3].copy(), self._output[3:].copy()

    def paired_wrenches(self, q, v, base_layout, plug_layout):
        for index, (qa, va, com) in enumerate((base_layout, plug_layout)):
            self._sample[13*index:13*index+7] = q[qa:qa+7]
            self._sample[13*index+7:13*index+13] = v[va:va+6]
            self._coms[3*index:3*index+3] = com
        if self._evaluate_wrenches(*self._wrench_arguments) != 0:
            raise ValueError("retention requires finite body poses and velocities")
        self._update_status()
        return self._wrenches.copy()

    def _update_status(self):
        active = int(self._state[0])
        self.active = self.sockets[active] if active >= 0 else None
        self.first_sample = False
        self.status = ("free", "retained", "held", "extracting", "inserting")[int(self._state[2])]
        self.displacement_m = float(self._state[3]) if active >= 0 else None
