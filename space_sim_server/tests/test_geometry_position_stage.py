import numpy as np
import pytest


def test_position_stage_preserves_geometry_and_contact_results():
    mj = pytest.importorskip('mujoco')
    model = mj.MjModel.from_xml_string('''<mujoco><worldbody>
    <geom type="plane" size="1 1 .1"/>
    <body pos="0 0 .09"><freejoint/><geom type="sphere" size=".1" mass="1"/>
    <site name="tip" pos=".02 0 0"/></body></worldbody></mujoco>''')
    full, position = mj.MjData(model), mj.MjData(model)
    for z in (.09, .12):
        full.qpos[2] = position.qpos[2] = z
        mj.mj_forward(model, full)
        mj.mj_fwdPosition(model, position)
        np.testing.assert_array_equal(position.geom_xpos, full.geom_xpos)
        np.testing.assert_array_equal(position.site_xmat, full.site_xmat)
        assert [(c.geom1, c.geom2, c.dist) for c in position.contact] == [(c.geom1, c.geom2, c.dist) for c in full.contact]
