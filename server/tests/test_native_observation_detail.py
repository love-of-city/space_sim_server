"""Detailed native observations are passive; one plug has one friction owner."""
import numpy as np
import pytest

from test_local_mujoco_stepper import MJCF, build, run_to


def test_detailed_observations_preserve_native_state(tmp_path):
    model = tmp_path / 'contact.xml'
    model.write_text(MJCF.replace('name="ball" pos="1.5 0 0"', 'name="ball" pos="0.24 0 0"'))
    sim, scene, core, keep = build(model, 'local')
    try:
        run_to(sim, 0)
        q, v = core.qpos.copy(), core.qvel.copy()
        details, basic = core.contact_detail(), core.contact_snapshot()
        assert details.shape == (len(basic), 23) and len(details)
        np.testing.assert_array_equal(details[:, :2], basic[:, :2])
        np.testing.assert_allclose(details[:, 2], basic[:, 4])
        pose = core.body_detail('ball')
        assert pose.shape == (14,) and np.isfinite(pose).all()
        address = scene.getBody('ball').getFreeJoint().getQposAdr()
        np.testing.assert_allclose(pose[:7], q[address:address+7])
        np.testing.assert_array_equal(q, core.qpos)
        np.testing.assert_array_equal(v, core.qvel)
    finally:
        core.close()


def test_friction_transfer_rejects_simultaneous_owners(tmp_path):
    model = tmp_path / 'owners.xml'
    model.write_text(MJCF.replace('</worldbody>', '<body name="receiver" pos="0 2 0">'
        '<freejoint/><geom type="sphere" size=".1" mass="2"/></body></worldbody>'))
    sim, scene, core, keep = build(model, 'local')
    try:
        first = core.add_axial_friction('ball', 'bus', static_n=5, kinetic_n=4, travel_m=.01)
        second = core.add_axial_friction('ball', 'receiver', static_n=5, kinetic_n=4, travel_m=.01)
        first.configure(enabled=True)
        with pytest.raises(RuntimeError):
            second.configure(enabled=True)
        first.configure(enabled=False)
        second.configure(enabled=True)
        with pytest.raises(RuntimeError):
            first.configure(enabled=True)
        second.configure(enabled=False)
        run_to(sim, .01)
        assert first.telemetry()['friction_mode'] == second.telemetry()['friction_mode'] == 'free'
    finally:
        core.close()
