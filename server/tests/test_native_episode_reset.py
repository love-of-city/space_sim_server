"""The worker's reset must remove previous coordinates and velocities."""
import ctypes
import os

import numpy as np
import pytest


def test_native_reset_restores_xml_state_and_clears_episode_velocity(tmp_path):
    pytest.importorskip('Basilisk')
    from simulation.local_mujoco_bindings import load_stepper_library, double_pointer
    from simulation.mujoco_runtime import basilisk_mujoco_runtime
    model = tmp_path/'reset.xml'
    model.write_text('''<mujoco><option gravity="0 0 0"/><worldbody>
      <body name="moving" pos="1 2 3"><freejoint/>
      <geom type="sphere" size=".02" mass="1"/></body>
    </worldbody></mujoco>''')
    lib = load_stepper_library()
    error = ctypes.create_string_buffer(1024)
    runtime = str(basilisk_mujoco_runtime())
    raw = lib.lms_create(runtime if os.name == 'nt' else os.fsencode(runtime),
                         os.fsencode(model), error, len(error))
    assert raw, error.value.decode()
    handle = ctypes.c_void_p(raw)
    try:
        q, v = np.zeros(7), np.zeros(6)
        assert lib.lms_get_state(handle, double_pointer(q), double_pointer(v)) == 0
        initial = q.copy()
        q[:3] += [4., 5., 6.]
        v[:] = np.arange(6) + 1
        assert lib.lms_set_state(handle, double_pointer(q), double_pointer(v)) == 0
        assert lib.lms_reset_episode(handle) == 0
        assert lib.lms_get_state(handle, double_pointer(q), double_pointer(v)) == 0
        np.testing.assert_array_equal(q, initial)
        np.testing.assert_array_equal(v, np.zeros(6))
        assert lib.lms_reset_episode(None) < 0
    finally:
        lib.lms_destroy(handle)


def test_reset_restores_xml_timestep_for_initial_contact_force(tmp_path):
    """Reset must give the same initial contact sample as a fresh native model."""
    pytest.importorskip('Basilisk')
    from simulation.local_mujoco_bindings import load_stepper_library, double_pointer
    from simulation.mujoco_runtime import basilisk_mujoco_runtime

    model = tmp_path / 'reset_contact.xml'
    model.write_text('''<mujoco><option timestep=".001" gravity="0 0 0"/><worldbody>
      <geom type="plane" size="1 1 .1" solref=".002 1"/>
      <body pos="0 0 .009"><freejoint/>
        <geom type="sphere" size=".01" mass="1" solref=".002 1"/>
      </body></worldbody></mujoco>''')
    lib = load_stepper_library()
    error = ctypes.create_string_buffer(1024)
    runtime = str(basilisk_mujoco_runtime())
    raw = lib.lms_create(runtime if os.name == 'nt' else os.fsencode(runtime),
                         os.fsencode(model), error, len(error))
    assert raw, error.value.decode()
    handle = ctypes.c_void_p(raw)
    try:
        q, v = np.zeros(7), np.zeros(6)
        assert lib.lms_get_state(handle, double_pointer(q), double_pointer(v)) == 0
        initial_q, initial_v = q.copy(), v.copy()
        assert lib.lms_set_state(handle, double_pointer(q), double_pointer(v)) == 0

        def contact_forces():
            count = lib.lms_contact_forces(handle, None, 0)
            assert count > 0
            rows = np.zeros((count, 11))
            assert lib.lms_contact_forces(handle, double_pointer(rows), count) >= 0
            return rows

        cold = contact_forces()
        controls, origin = np.zeros(0), np.zeros(3)
        qacc, actuator_force, stats = np.zeros(6), np.zeros(0), np.zeros(6)
        assert lib.lms_step(handle, double_pointer(controls), .0025, 1, double_pointer(origin), 0.,
                            double_pointer(q), double_pointer(v), double_pointer(qacc),
                            double_pointer(actuator_force), double_pointer(stats)) == 0
        assert lib.lms_reset_episode(handle) == 0
        assert lib.lms_set_state(handle, double_pointer(initial_q), double_pointer(initial_v)) == 0
        np.testing.assert_array_equal(contact_forces(), cold)
    finally:
        lib.lms_destroy(handle)
