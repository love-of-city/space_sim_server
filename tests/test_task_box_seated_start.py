"""Physics regression for unlatched plugs starting inside their storage bores."""
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_seated_plugs_start_quietly_and_can_be_withdrawn():
    mj = pytest.importorskip('mujoco')
    np = pytest.importorskip('numpy')
    path = ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml'
    model = mj.MjModel.from_xml_path(str(path))
    metadata = json.loads(path.with_suffix('.manifest.json').read_text())['plugs']
    data = mj.MjData(model)
    for item in metadata:
        a = model.jnt_qposadr[model.joint(item['name'] + '_free').id]
        offset = data.qpos[a:a+3] - item['bus_center_m']
        assert np.linalg.norm(offset[:2]) < 1e-8
        assert 0 <= offset[2] <= 0.0001, 'Plug must start inside its bore, not above it'
    mj.mj_forward(model, data)
    assert data.ncon == 0
    initial = data.qpos.copy()
    mj.mj_step(model, data, nstep=1000)
    for item in metadata:
        j = model.joint(item['name'] + '_free').id
        a, v = model.jnt_qposadr[j], model.jnt_dofadr[j]
        assert np.linalg.norm(data.qpos[a:a+3] - initial[a:a+3]) < 0.0001
    for item in metadata:
        j = model.joint(item['name'] + '_free').id
        a, v = model.jnt_qposadr[j], model.jnt_dofadr[j]
        # A free outward velocity must withdraw the plug without pose overrides.
        mj.mj_resetData(model, data)
        data.qvel[v+2] = 0.03
        mj.mj_step(model, data, nstep=1500)
        assert data.qpos[a+2] - initial[a+2] > 0.08
        assert np.isfinite(data.qpos).all() and not data.warning.number.any()
        mj.mj_resetData(model, data)
        # The storage bore has clearance, but its wall must still block the plug.
        data.qpos[a] += 0.008
        mj.mj_forward(model, data)
        number = 9 if item['name'] == 'guide_plug' else 13
        flex = mj.mj_name2id(model, mj.mjtObj.mjOBJ_FLEX, f'task_box_{number:03}_contact')
        forces = []
        for index, contact in enumerate(data.contact):
            if flex in contact.flex:
                force = np.zeros(6)
                mj.mj_contactForce(model, data, index, force)
                forces.append(force[0])
        assert forces and max(forces) > 0
        mj.mj_resetData(model, data)
