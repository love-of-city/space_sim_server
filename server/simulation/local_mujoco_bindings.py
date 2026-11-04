"""C ABI bindings for the local MuJoCo helper, independent of BSK scheduling.

Loading verifies the source-matched native build and its ABI/header version.
This module never imports Python MuJoCo or loads a separate MuJoCo runtime.
"""
from __future__ import annotations

import ctypes
import os

import numpy as np

from simulation.native_acceleration import load_native_library

DOUBLE_P = ctypes.POINTER(ctypes.c_double)
INT64_P = ctypes.POINTER(ctypes.c_int64)


def load_stepper_library():
    lib = load_native_library("local_mujoco_stepper")
    if getattr(lib, "_lms_bound", False):
        return lib
    lib.lms_abi.restype = ctypes.c_int
    lib.lms_header_version.restype = ctypes.c_int
    lib.lms_create.restype = ctypes.c_void_p
    runtime_path_type = ctypes.c_wchar_p if os.name == "nt" else ctypes.c_char_p
    lib.lms_create.argtypes = [runtime_path_type, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.lms_create_cached.restype = ctypes.c_void_p
    lib.lms_create_cached.argtypes = [runtime_path_type, ctypes.c_char_p, ctypes.c_char_p,
                                     INT64_P, ctypes.c_char_p, ctypes.c_int]
    lib.lms_destroy.argtypes = [ctypes.c_void_p]
    lib.lms_dims.argtypes = [ctypes.c_void_p, INT64_P]
    lib.lms_name2id.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
    lib.lms_id2name.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    lib.lms_joint.argtypes = [ctypes.c_void_p, ctypes.c_int, INT64_P]
    lib.lms_body_mass.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P]
    lib.lms_body_inertial.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P]
    lib.lms_body_geometry_pose.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P]
    lib.lms_free_joints.argtypes = [ctypes.c_void_p, INT64_P, ctypes.c_int]
    lib.lms_set_servo.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.lms_set_state.argtypes = [ctypes.c_void_p, DOUBLE_P, DOUBLE_P]
    lib.lms_get_state.argtypes = [ctypes.c_void_p, DOUBLE_P, DOUBLE_P]
    lib.lms_step.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_double, ctypes.c_int, DOUBLE_P, ctypes.c_double,
                             DOUBLE_P, DOUBLE_P, DOUBLE_P, DOUBLE_P, DOUBLE_P]
    lib.lms_body_coms.argtypes = [ctypes.c_void_p, DOUBLE_P, DOUBLE_P]
    lib.lms_set_orbital_forces.argtypes = [ctypes.c_void_p, DOUBLE_P]
    lib.lms_contacts.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_int]
    lib.lms_probe_set_state.argtypes = [ctypes.c_void_p, DOUBLE_P, DOUBLE_P]
    lib.lms_probe_set_state.restype = ctypes.c_int
    if hasattr(lib, "lms_probe_relevance_begin"):
        lib.lms_probe_relevance_begin.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
        lib.lms_probe_relevance_end.argtypes = [ctypes.c_void_p]
        lib.lms_probe_relevance_skipped.restype = ctypes.c_ulonglong
    lib.lms_probe_clearance.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.lms_contact_forces.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_int]
    lib.lms_contact_materials.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_int]
    lib.lms_contact_geometry.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_int]
    lib.lms_contact_detail.argtypes = [ctypes.c_void_p, DOUBLE_P, ctypes.c_int]
    lib.lms_contact_detail.restype = ctypes.c_int
    lib.lms_body_detail.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P]
    lib.lms_body_detail.restype = ctypes.c_int
    lib.lms_read_wrench.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, DOUBLE_P]
    lib.lms_momentum.argtypes = [ctypes.c_void_p, DOUBLE_P]
    lib.lms_set_body_wrench.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P, DOUBLE_P]
    lib.lms_add_axial_friction.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
                                          ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.lms_set_axial_friction.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, DOUBLE_P, DOUBLE_P]
    lib.lms_axial_friction_state.argtypes = [ctypes.c_void_p, ctypes.c_int, DOUBLE_P]
    lib.lms_friction_diagnostics.argtypes = [ctypes.c_void_p, INT64_P]
    lib.lms_friction_copy_diagnostics.argtypes = [ctypes.c_void_p, INT64_P]
    lib.lms_reset_episode.argtypes = [ctypes.c_void_p]
    lib.lms_reset_episode.restype = ctypes.c_int
    lib.lms_finger_contact_counts.argtypes = [ctypes.c_void_p, *([ctypes.c_int] * 4), ctypes.c_double, ctypes.POINTER(ctypes.c_int)]
    lib.lms_finger_contact_counts.restype = ctypes.c_int
    lib.lms_sum_body_wrenches.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), *([ctypes.POINTER(DOUBLE_P)] * 4)]
    lib.lms_sum_body_wrenches.restype = ctypes.c_int
    lib.lms_plug_guidance.argtypes = [DOUBLE_P, DOUBLE_P, ctypes.c_int, DOUBLE_P, DOUBLE_P, DOUBLE_P]
    lib.lms_plug_guidance.restype = ctypes.c_int
    lib.lms_plug_guidance_wrenches.argtypes = [DOUBLE_P, DOUBLE_P, DOUBLE_P, ctypes.c_int,
                                             DOUBLE_P, DOUBLE_P, DOUBLE_P]
    lib.lms_plug_guidance_wrenches.restype = ctypes.c_int
    if lib.lms_abi() != 1 or lib.lms_header_version() != 3007000:
        raise RuntimeError("local MuJoCo stepper ABI/header mismatch; rebuild it")
    lib._lms_bound = True
    return lib


def double_pointer(array: np.ndarray):
    """Borrow a float64 array's storage for a synchronous native call."""
    return array.ctypes.data_as(DOUBLE_P)
