"""The worker's reset must remove previous coordinates and velocities."""
import ctypes
import os

import numpy as np
import pytest


def test_native_reset_restores_xml_state_and_clears_episode_velocity(tmp_path):
    pytest.importorskip('Basilisk')
    from simulation.local_mujoco_stepper import _library, _basilisk_mujoco_runtime, _pointer
    model = tmp_path/'reset.xml'
    model.write_text('''<mujoco><option gravity="0 0 0"/><worldbody>
      <body name="moving" pos="1 2 3"><freejoint/>
      <geom type="sphere" size=".02" mass="1"/></body>
    </worldbody></mujoco>''')
    lib = _library()
    error = ctypes.create_string_buffer(1024)
    runtime = str(_basilisk_mujoco_runtime())
    raw = lib.lms_create(runtime if os.name == 'nt' else os.fsencode(runtime),
                         os.fsencode(model), error, len(error))
    assert raw, error.value.decode()
    handle = ctypes.c_void_p(raw)
    try:
        q, v = np.zeros(7), np.zeros(6)
        assert lib.lms_get_state(handle, _pointer(q), _pointer(v)) == 0
        initial = q.copy()
        q[:3] += [4., 5., 6.]
        v[:] = np.arange(6) + 1
        assert lib.lms_set_state(handle, _pointer(q), _pointer(v)) == 0
        assert lib.lms_reset_episode(handle) == 0
        assert lib.lms_get_state(handle, _pointer(q), _pointer(v)) == 0
        np.testing.assert_array_equal(q, initial)
        np.testing.assert_array_equal(v, np.zeros(6))
        assert lib.lms_reset_episode(None) < 0
    finally:
        lib.lms_destroy(handle)
