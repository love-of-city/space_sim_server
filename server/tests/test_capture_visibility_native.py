"""Exercise the real Basilisk MuJoCo 3.7 ray ABI in a small known scene."""
import ctypes
import gc
import os

import numpy as np


def test_visual_ray_first_hit_and_miss_survive_collection(tmp_path):
    from simulation.mujoco_runtime import basilisk_mujoco_runtime
    from simulation.local_mujoco_bindings import load_stepper_library, double_pointer
    runtime = str(basilisk_mujoco_runtime())
    model = tmp_path/'visibility.xml'
    model.write_text('''<mujoco><worldbody>
      <geom name="floor" type="plane" size="5 5 .1" group="0"/>
      <body pos="0 0 1"><freejoint/>
        <geom name="visible_box" type="box" size=".5 .5 .5" group="2"/>
      </body></worldbody></mujoco>''')
    lib = load_stepper_library()
    error = ctypes.create_string_buffer(1024)
    pointer = lib.lms_create(runtime if os.name == 'nt' else os.fsencode(runtime),
                             str(model).encode(), error, len(error))
    assert pointer, error.value
    handle = ctypes.c_void_p(pointer)
    try:
        start = np.array([0., 0., 3.])
        down, up = np.array([0., 0., -1.]), np.array([0., 0., 1.])
        expected = lib.lms_name2id(handle, 5, b'visible_box')
        for _ in range(20):
            result = np.empty(2)
            assert lib.lms_visual_ray(handle, double_pointer(start), double_pointer(down), double_pointer(result)) == 0
            np.testing.assert_allclose(result, [1.5, expected])
            assert lib.lms_visual_ray(handle, double_pointer(start), double_pointer(up), double_pointer(result)) == 0
            np.testing.assert_allclose(result, [-1., -1.])
            gc.collect()
    finally:
        lib.lms_destroy(handle)


def test_native_contacts_store_separation_after_both_body_ids(tmp_path):
    from simulation.mujoco_runtime import basilisk_mujoco_runtime
    from simulation.local_mujoco_bindings import load_stepper_library, double_pointer
    runtime = str(basilisk_mujoco_runtime())
    model = tmp_path / 'contacts.xml'
    model.write_text('''<mujoco><worldbody>
      <geom name="floor" type="plane" size="5 5 .1"/>
      <body name="box" pos="0 0 .4"><freejoint/>
        <geom name="box_geom" type="box" size=".5 .5 .5"/>
      </body></worldbody></mujoco>''')
    lib = load_stepper_library()
    error = ctypes.create_string_buffer(1024)
    pointer = lib.lms_create(runtime if os.name == 'nt' else os.fsencode(runtime),
                             str(model).encode(), error, len(error))
    assert pointer, error.value
    handle = ctypes.c_void_p(pointer)
    try:
        count = lib.lms_contacts(handle, None, 0)
        assert count > 0
        contacts = np.empty((count, 5))
        assert lib.lms_contacts(handle, double_pointer(contacts), count) == count
        assert np.all(contacts[:, 2:4] >= 0)
        np.testing.assert_allclose(contacts[:, 4], -.1, atol=1e-12)
    finally:
        lib.lms_destroy(handle)
