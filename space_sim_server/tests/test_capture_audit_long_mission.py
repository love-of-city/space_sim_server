"""A late grasp must not lose its per-step force evidence after the body cap."""
import json

import numpy as np

from simulation.capture_audit import CaptureAudit


def test_separate_trials_keep_force_epochs_across_the_300_second_boundary(tmp_path):
    model = tmp_path/'model.xml'
    model.write_text('<mujoco><worldbody><body name="cubesat_bus"><body name="gripper"/>'
                     '</body><body name="capture_target"/></worldbody></mujoco>', encoding='utf-8')

    class Stepper:
        external_writes = 0
        substeps = 1
        step_count = 0
        _last_nanos = 0
        def body_detail(self, name):
            return np.array([0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        def contact_detail(self):
            return []

    for trial, epochs in (('late', [299.9975, 300., 300.0025, 1000.]), ('other', [1.])):
        audit = CaptureAudit(model, Stepper(), output_path=tmp_path/trial/'contacts.jsonl')
        try:
            for epoch in epochs:
                audit.record(epoch, 'hold_force', np.zeros(12))
        finally:
            audit.file.close()
            audit.force_file.close()
    force = [json.loads(line) for line in (tmp_path/'late/contacts_force.jsonl').read_text().splitlines()]
    assert [row['time_s'] for row in force] == [299.9975, 300., 300.0025, 1000.]
    assert all(row['feedback']['jaw_support_count'] == 0 for row in force)
    assert all(row['schema_version'] == 'space-sim-capture-contact-evidence/1' and row['raw_contacts'] == [] for row in force)
    assert [row['sequence'] for row in force] == list(range(4))
    manifest = json.loads((tmp_path/'late/contacts_manifest.json').read_text())
    assert manifest['force_file'] == 'contacts_force.jsonl'
    bodies = [json.loads(line) for line in (tmp_path/'late/contacts.jsonl').read_text().splitlines()]
    assert [row['t'] for row in bodies if row['type'] == 'step'] == [299.9975, 300.]
    other = [json.loads(line) for line in (tmp_path/'other/contacts_force.jsonl').read_text().splitlines()]
    assert [row['time_s'] for row in other] == [1.]
