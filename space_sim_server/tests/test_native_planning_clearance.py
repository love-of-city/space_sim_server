"""Positive separation queries must not inflate the production physics model."""
import ctypes
import os
import numpy as np
import pytest


def test_clearance_is_local_to_private_model_and_reports_positive_separation(tmp_path):
    pytest.importorskip("Basilisk")
    from simulation.local_mujoco_stepper import _library, _basilisk_mujoco_runtime, _pointer
    model=tmp_path/'gap.xml'
    model.write_text('''<mujoco><option gravity="0 0 0"/><worldbody>
    <body name="fixed"><geom type="sphere" size=".02"/></body>
    <body name="moving" pos=".041 0 0"><freejoint/><geom type="sphere" size=".02" mass="1"/></body>
    </worldbody></mujoco>''')
    lib=_library();runtime=str(_basilisk_mujoco_runtime())
    def create():
        error=ctypes.create_string_buffer(1024)
        h=lib.lms_create(runtime if os.name=='nt' else os.fsencode(runtime),os.fsencode(model),error,len(error))
        assert h,error.value.decode()
        return ctypes.c_void_p(h)
    live,probe=create(),create()
    try:
        q,v=np.zeros(7),np.zeros(6)
        assert lib.lms_get_state(live,_pointer(q),_pointer(v))==0
        before=q.copy()
        assert lib.lms_contacts(live,None,0)==0
        assert lib.lms_probe_clearance(probe,.002)==0
        assert lib.lms_set_state(probe,_pointer(q),_pointer(v))==0
        rows=np.zeros((1,5))
        assert lib.lms_contacts(probe,_pointer(rows),1)==1
        assert rows[0,4]==pytest.approx(.001,abs=1e-10)
        assert lib.lms_contacts(live,None,0)==0
        assert lib.lms_get_state(live,_pointer(q),_pointer(v))==0
        np.testing.assert_array_equal(q,before)
    finally:
        lib.lms_destroy(probe);lib.lms_destroy(live)
