"""Offline MuJoCo 3.7 contact acceptance. Never run in a Basilisk process."""
import argparse
import copy
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'model/SARM/platform/sarm_task_box_module.xml'


def probe_model():
    root = ET.parse(SCENE).getroot()
    bus = copy.deepcopy(root.find('.//body[@name="cubesat_bus"]'))
    bus.set('pos', '0 0 0')
    bus.set('quat', '1 0 0 0')
    for child in list(bus):
        if child.tag not in ('geom', 'flexcomp'):
            bus.remove(child)
    world = root.find('worldbody')
    world.clear()
    world.append(bus)
    for tag in ('actuator', 'contact', 'sensor', 'equality', 'keyframe'):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    probe = ET.SubElement(world, 'body', name='contact_probe', pos='0 0 1')
    ET.SubElement(probe, 'freejoint', name='probe_free')
    ET.SubElement(probe, 'geom', name='probe', type='sphere', size='.0005', mass='.01',
                  contype='1', conaffinity='1', friction='.6 .005 .0001')
    used = {g.get('mesh') for g in root.findall('.//geom')}
    asset = root.find('asset')
    for element in list(asset):
        if element.tag == 'mesh' and element.get('name') not in used:
            asset.remove(element)
    for element in list(asset.findall('mesh')) + root.findall('.//flexcomp'):
        if element.get('file'):
            element.set('file', str((SCENE.parent / element.get('file')).resolve()))
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def main(output):
    assert mj.__version__ == '3.7.0', 'Use runtime-matched, standalone MuJoCo 3.7.0'
    start = time.perf_counter()
    model = mj.MjModel.from_xml_path(str(SCENE))
    compile_seconds = time.perf_counter() - start
    print('Compiled', compile_seconds, 's', flush=True)
    old = mj.MjModel.from_xml_path(str(SCENE.with_name('sarm_ground_target_self_collision.xml')))
    assert (model.nq, model.nv, model.nu) == (old.nq, old.nv, old.nu)
    assert np.allclose(model.body_mass, old.body_mass)
    assert np.allclose(model.body_inertia, old.body_inertia)
    xml = ET.parse(SCENE).getroot()
    xml_meshes = [g.get('mesh', '') for g in xml.find('worldbody').iter('geom')]
    compiled_meshes = [model.mesh(int(model.geom_dataid[i])).name
                       if model.geom_type[i] == mj.mjtGeom.mjGEOM_MESH else ''
                       for i in range(model.ngeom)]
    assert xml_meshes == compiled_meshes, 'UE XML/compiled geometry index mapping differs'
    assert not model.opt.disableflags & int(mj.mjtDisableBit.mjDSBL_CONTACT | mj.mjtDisableBit.mjDSBL_CONSTRAINT)
    assert not model.opt.enableflags & int(mj.mjtEnableBit.mjENBL_OVERRIDE)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    initial_contacts = data.ncon
    print('Initial contacts', initial_contacts, flush=True)
    assert initial_contacts == 0, 'Unexpected assembly/mount contact at zero pose'
    start = time.perf_counter()
    mj.mj_step(model, data, nstep=1000)
    step_seconds = time.perf_counter() - start
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert not data.warning.number.any()
    probe = probe_model()
    pd = mj.MjData(probe)
    pid = probe.geom('probe').id
    # Positions in satellite bus coordinates, converted from measured local CAD.
    cases = {
        'table_top': ([.5, .32, -.0001], True),
        'box_side_wall': ([.575, .32, -.03], True),
        'guide_bore_clear': ([.535, .275, -.02], False),
        'guide_slot_clear': ([.535, .251, -.02], False),
        'guide_wrong_angle_wall': ([.561, .275, -.0001], True),
        'guide_bore_wall': ([.5553, .275, -.02], True),
        'aviation_bore_clear': ([.535, .195, -.02], False),
        'aviation_pin_hole_clear': ([.5425, .2025, -.05], False),
        'aviation_wrong_pin_position': ([.535, .195, -.0401], True),
    }
    results = {}
    for name, (position, expected) in cases.items():
        mj.mj_resetData(probe, pd)
        pd.qpos[:3] = position
        mj.mj_forward(probe, pd)
        contacts = [i for i in range(pd.ncon) if pid in pd.contact[i].geom]
        force = []
        for i in contacts:
            wrench = np.zeros(6)
            mj.mj_contactForce(probe, pd, i, wrench)
            force.append(float(wrench[0]))
        results[name] = dict(position_m=position, expected_contact=expected,
                             contacts=len(contacts), normal_force_N=max(force, default=0.))
        print(name, results[name], flush=True)
    mj.mj_resetData(probe, pd)
    pd.qpos[:3] = [.5, .32, .003]
    pd.qvel[2] = -.01
    minimum_z, peak_force = float('inf'), 0.
    for _ in range(500):
        mj.mj_step(probe, pd)
        minimum_z = min(minimum_z, float(pd.qpos[2]))
        for i in range(pd.ncon):
            if pid in pd.contact[i].geom:
                wrench = np.zeros(6)
                mj.mj_contactForce(probe, pd, i, wrench)
                peak_force = max(peak_force, float(wrench[0]))
    assert peak_force > 0 and minimum_z > -.0005, 'Probe tunneled through tabletop'
    # The upstream capture target uses bit 2, while the arm uses bit 1.
    probe.geom_contype[pid] = 2
    mj.mj_resetData(probe, pd)
    pd.qpos[:3] = [.5, .32, -.0001]
    mj.mj_forward(probe, pd)
    assert any(pid in c.geom for c in pd.contact), 'Upstream target mask cannot contact task box'
    report = dict(mujoco_version=mj.__version__, compile_seconds=compile_seconds,
                  simulated_seconds=1000 * model.opt.timestep, step_seconds=step_seconds,
                  initial_contacts=initial_contacts, warnings=data.warning.number.tolist(),
                  nq=model.nq, nv=model.nv, nu=model.nu, probes=results,
                  dynamic_table_test=dict(minimum_z_m=minimum_z, peak_force_N=peak_force,
                                          approach_speed_m_s=.01, simulated_seconds=1.),
                  scope='Static clearance/contact-force probes and 1000-step smoke; not physical plug insertion or UE acceptance.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    for name, result in results.items():
        assert bool(result['contacts']) == result['expected_contact'], name
        if result['expected_contact']:
            assert result['normal_force_N'] > 0, name
    print(json.dumps({k: v for k, v in report.items() if k != 'probes'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/task-box-contact.json')
    main(parser.parse_args().output)
