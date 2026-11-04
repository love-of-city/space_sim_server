"""The collection preflight/contact gate reads the authoritative model without stepping."""

import numpy as np
from test_local_mujoco_stepper import MJCF, build, run_to


def test_contact_snapshot_reports_body_pairs_and_preserves_state(tmp_path):
    path = tmp_path / "contacts.xml"
    path.write_text(
        MJCF.replace('name="ball" pos="1.5 0 0"', 'name="ball" pos="0.24 0 0"'),
        encoding="utf-8",
    )
    sim, scene, stepper, keep = build(path, "local")
    try:
        run_to(sim, 0)
        before_q, before_v = stepper.qpos.copy(), stepper.qvel.copy()
        contacts = stepper.contact_snapshot()
        assert contacts.shape[1] == 5 and len(contacts) > 0
        pairs = [
            {stepper._name(1, int(row[2])), stepper._name(1, int(row[3]))}
            for row in contacts
        ]
        assert {"bus", "ball"} in pairs
        assert contacts[:, 4].min() < 0
        np.testing.assert_array_equal(contacts, stepper.contact_snapshot())
        np.testing.assert_array_equal(before_q, stepper.qpos)
        np.testing.assert_array_equal(before_v, stepper.qvel)
    finally:
        stepper.close()
