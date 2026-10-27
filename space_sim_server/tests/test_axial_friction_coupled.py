"""Three physical friction interfaces sharing one floating, articulated base."""

import ctypes

import numpy as np
import pytest

from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


MJCF = """<mujoco>
<option gravity="0 0 0" integrator="implicitfast" tolerance="1e-10"/>
<default><geom contype="0" conaffinity="0"/></default>
<worldbody>
  <body name="bus"><freejoint name="bus_free"/>
    <geom type="box" size=".35 .35 .2" pos=".01 .02 .03" mass="100"/>
    <body name="link" pos=".1 0 0">
      <joint name="hinge" axis="0 1 0" damping=".01"/>
      <geom type="capsule" fromto="0 0 0 .1 0 0" size=".02" mass=".4"/>
    </body>
  </body>
  <body name="plug0" pos="-.15 -.1 .304"><freejoint name="plug0_free"/>
    <geom type="sphere" size=".01" mass=".2"/>
  </body>
  <body name="plug1" pos=".15 -.1 .304"><freejoint name="plug1_free"/>
    <geom type="sphere" size=".01" mass=".3"/>
  </body>
  <body name="plug2" pos="0 .15 .304"><freejoint name="plug2_free"/>
    <geom type="sphere" size=".01" mass=".4"/>
  </body>
</worldbody>
<actuator><motor joint="hinge"/></actuator>
</mujoco>"""


class _NativeTriple:
    """Direct native stepping: state is set once, before any applied loads."""

    def __init__(self, path):
        from simulation.local_mujoco_stepper import (
            _basilisk_mujoco_runtime, _library, _pointer,
        )

        self.lib, self.ptr = _library(), _pointer
        error = ctypes.create_string_buffer(4096)
        self.handle = self.lib.lms_create(
            str(_basilisk_mujoco_runtime()).encode(), str(path).encode(), error, len(error)
        )
        assert self.handle, error.value.decode()
        dims = np.zeros(6, dtype=np.int64)
        self.check(self.lib.lms_dims(self.handle, dims.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))))
        nq, nv, nu, *_ = map(int, dims)
        self.q, self.v, self.a = np.zeros(nq), np.zeros(nv), np.zeros(nv)
        self.ctrl, self.effort, self.stats = np.zeros(nu), np.zeros(nu), np.zeros(6)
        self.origin, self.zero = np.array([6.9e6, 0., 0.]), np.zeros(3)
        self.check(self.lib.lms_get_state(self.handle, self.ptr(self.q), self.ptr(self.v)))
        self.check(self.lib.lms_set_state(self.handle, self.ptr(self.q), self.ptr(self.v)))
        self.base = self.lib.lms_name2id(self.handle, 1, b"bus")
        self.base_q = self.joint_address("bus_free")
        self.plugs = []
        for index, xy in enumerate(((-.15, -.1), (.15, -.1), (0., .15))):
            body = self.lib.lms_name2id(self.handle, 1, f"plug{index}".encode())
            channel = self.lib.lms_add_axial_friction(self.handle, body, self.base, 20., 20., .008)
            self.check(channel)
            axis, seat = np.array([0., 0., 1.]), np.array([*xy, .3])
            self.check(self.lib.lms_set_axial_friction(
                self.handle, channel, 1, self.ptr(axis), self.ptr(seat)
            ))
            self.plugs.append((body, self.joint_address(f"plug{index}_free"), channel))

    @staticmethod
    def check(status):
        assert status >= 0, f"native stepper error {status}"

    def joint_address(self, name):
        joint = self.lib.lms_name2id(self.handle, 3, name.encode())
        info = np.zeros(3, dtype=np.int64)
        self.check(self.lib.lms_joint(
            self.handle, joint, info.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))
        ))
        return int(info[0])

    def depths(self):
        base = self.q[self.base_q:self.base_q+3]
        rotation = quaternion_wxyz_to_matrix(self.q[self.base_q+3:self.base_q+7])
        return np.array([(rotation.T @ (self.q[address:address+3]-base))[2]-.3
                         for _, address, _ in self.plugs])

    def step(self, loads):
        # Loads act at the actual plug COM; only friction supplies the reaction
        # to the free base. Its translation/rotation couples all three channels.
        for (body, _, _), load in zip(self.plugs, loads):
            force = np.array([0., 0., load])
            self.check(self.lib.lms_set_body_wrench(
                self.handle, body, self.ptr(force), self.ptr(self.zero)
            ))
        self.check(self.lib.lms_step(
            self.handle, self.ptr(self.ctrl), .0025, 1, self.ptr(self.origin), 0.,
            self.ptr(self.q), self.ptr(self.v), self.ptr(self.a),
            self.ptr(self.effort), self.ptr(self.stats)
        ))
        assert self.stats[5] == 1
        telemetry = np.zeros((3, 6))
        for row, (_, _, channel) in zip(telemetry, self.plugs):
            self.check(self.lib.lms_axial_friction_state(self.handle, channel, self.ptr(row)))
        return telemetry

    def close(self):
        self.lib.lms_destroy(self.handle)


@pytest.mark.parametrize("stateful_actuator", [False, True])
def test_three_interfaces_hold_break_away_stop_and_repeat_deterministically(tmp_path, stateful_actuator):
    pytest.importorskip("Basilisk")
    path = tmp_path / "coupled_friction.xml"
    xml = MJCF
    if stateful_actuator:
        xml = xml.replace('<motor joint="hinge"/>',
                          '<general joint="hinge" dyntype="filter" dynprm=".1"/>')
    path.write_text(xml, encoding="utf-8")
    first, second = _NativeTriple(path), _NativeTriple(path)
    # Twenty millisecond overloads move short distances before braking; every
    # plug stays inside its 8 mm physical friction interval for the whole run.
    profile = [(40, (-19., 19., 0.)), (8, (22., -22., 19.)),
               (40, (0., 0., -19.)), (8, (-22., 22., -19.)),
               (40, (19., -19., 0.)), (8, (0., 19., 22.)),
               (40, (-19., 0., 0.)), (8, (0., -19., -22.)),
               (80, (0., 0., 0.))]
    directions = [set() for _ in range(3)]
    try:
        for ticks, loads in profile:
            for _ in range(ticks):
                previous = first.depths()
                rows, duplicate = first.step(loads), second.step(loads)
                np.testing.assert_array_equal(first.q, second.q)
                np.testing.assert_array_equal(first.v, second.v)
                np.testing.assert_array_equal(first.a, second.a)
                np.testing.assert_array_equal(rows, duplicate)
                depths = first.depths()
                assert np.all((depths > 0.) & (depths < .008))
                # Check the solver's actual integrated slip against independent
                # relative poses, rather than a requested velocity or force.
                np.testing.assert_allclose(rows[:, 4], (depths-previous)/.0025, atol=1e-11)
                assert np.max(np.abs(rows[:, 3])) <= 20.+1e-10
                for index, row in enumerate(rows):
                    if row[5] == 1:
                        assert abs(row[4]) <= 1e-6
                    else:
                        assert row[5] == 2
                        assert abs(row[3]) == pytest.approx(20., abs=1e-8)
                        assert row[3]*row[4] < 0.
                        directions[index].add(int(np.sign(row[4])))
        assert directions == [{-1, 1}, {-1, 1}, {-1, 1}]
        np.testing.assert_array_equal(rows[:, 5], np.ones(3))
        assert np.max(np.abs(rows[:, 4])) <= 1e-6
        # The base actually reacts; this is not three independently pinned
        # sockets. No state resets are used to return the plugs after a pulse.
        assert np.linalg.norm(first.q[first.base_q:first.base_q+3]) > 1e-6
        counts = np.zeros(4, dtype=np.int64)
        first.check(first.lib.lms_friction_diagnostics(
            first.handle, counts.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))
        ))
        assert counts[1] == (0 if stateful_actuator else sum(ticks for ticks, _ in profile))
        assert counts[0] >= counts[1]
        copies = np.zeros(2, dtype=np.int64)
        first.check(first.lib.lms_friction_copy_diagnostics(
            first.handle, copies.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))
        ))
        if stateful_actuator:
            assert copies[0] == counts[0] and copies[1] == 0
        else:
            assert copies[0] == 0 and copies[1] == counts[0]
    finally:
        first.close()
        second.close()


def test_enabled_channels_past_travel_do_not_reuse_an_old_prediction(tmp_path):
    pytest.importorskip("Basilisk")
    path = tmp_path / "past_travel.xml"
    path.write_text(MJCF, encoding="utf-8")
    enabled, disabled = _NativeTriple(path), _NativeTriple(path)
    try:
        # Populate a valid retained prediction first, then move all interfaces
        # outside travel. A stale prediction would rewind/freeze the next step.
        for core in (enabled, disabled):
            core.step((1., -1., 0.))
            for _, address, channel in core.plugs:
                core.q[address+2] += .03
                if core is disabled:
                    core.check(core.lib.lms_set_axial_friction(
                        core.handle, channel, 0, core.ptr(np.array([0., 0., 1.])), core.ptr(core.zero)
                    ))
            core.check(core.lib.lms_set_state(core.handle, core.ptr(core.q), core.ptr(core.v)))
        initial = enabled.q.copy()
        for _ in range(20):
            enabled.step((1., 2., 3.))
            disabled.step((1., 2., 3.))
            np.testing.assert_array_equal(enabled.q, disabled.q)
            np.testing.assert_array_equal(enabled.v, disabled.v)
            np.testing.assert_array_equal(enabled.a, disabled.a)
        assert np.linalg.norm(enabled.q-initial) > 1e-5
        counts = np.zeros(4, dtype=np.int64)
        enabled.check(enabled.lib.lms_friction_diagnostics(
            enabled.handle, counts.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))
        ))
        np.testing.assert_array_equal(counts[1:3], [1, 20])
    finally:
        enabled.close()
        disabled.close()
