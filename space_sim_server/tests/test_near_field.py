"""Sensor, handoff and admission boundaries; native acceptance is a separate run."""
import numpy as np
import pytest

from simulation.near_field.navigation import RelativeEKF, SensorConfig, SensorSimulator, stereo_geometry, origin_navigation, admit_handoff
from space_arm_platform.near_field_config import NearFieldOptions
from simulation.near_field.approach_plan import OpticalCheckpoint, approach_plan


def test_optical_checkpoint_requires_continuous_position_speed_and_link_dwell():
    checkpoint = OpticalCheckpoint()
    tick = 0
    def step(**overrides):
        nonlocal tick
        tick += 1
        return checkpoint.update(**(dict(error_m=[.01, 0, 0], speed_m_s=.001,
                                          ready=True, now=tick*.05) | overrides))
    for violation in ({"error_m": [.06, 0, 0]}, {"speed_m_s": .003}, {"ready": False}):
        for _ in range(90):
            assert not step()
        assert not step(**violation)
        assert checkpoint.dwell_s == 0.
    for _ in range(102):
        completed = step()
    assert completed and checkpoint.dwell_s >= 5.
    assert step(ready=False)  # completed waypoint remains in mission history


def test_checkpoint_does_not_count_interval_before_first_qualified_observation():
    checkpoint = OpticalCheckpoint()
    values = dict(error_m=np.zeros(3), speed_m_s=0., ready=True)
    assert not checkpoint.update(**values, now=10.)
    assert not checkpoint.update(**values, now=14.95)
    assert checkpoint.update(**values, now=15.)
    assert checkpoint.dwell_s == 5.


def test_standoff_is_projected_tcp_gap_not_spacecraft_origin_distance():
    class Chain:
        def forward(self, _):
            return np.array([.395, -.025, .095]), np.array([[0., 0, 1], [0, -1, 0], [1, 0, 0]])
    plan = approach_plan(Chain())
    grasp = np.array(plan["nominal_grasp_bus_m"])
    stowed, rotation = Chain().forward(None)
    assert rotation[:, 2] @ (grasp-stowed) == pytest.approx(.5)
    assert np.linalg.norm(grasp) > .9
    assert stereo_geometry(rotation.T @ grasp)["valid"]


def test_stereo_has_gnss_overlap_and_retains_both_images_at_hold():
    assert not stereo_geometry([.3, -.1, 110])["valid"]
    assert stereo_geometry([.3, -.1, 10])["valid"]
    hold = stereo_geometry([.3, -.1, .6])
    assert hold["common_fov"] and hold["valid"]
    assert hold["camera_depth_m"] == pytest.approx(.7)
    assert hold["depth_sigma_m"] < .001
    assert not stereo_geometry([10, 0, 1])["common_fov"]


def test_sensor_delays_failure_and_seed_are_reproducible():
    cfg = SensorConfig(loss_after_phase="FinalApproach")
    sensors = [SensorSimulator(cfg), SensorSimulator(cfg)]
    delivered = []
    for t in np.arange(0, 1.1, .1):
        phase = "ArmAlign" if t < .5 else "FinalApproach"
        outputs = [s.acquire(float(t), phase, np.array([.97, .17, .25, 0, 0, 0]), [.3, -.1, .6], np.eye(3))[0] for s in sensors]
        assert len(outputs[0]) == len(outputs[1])
        for a, b in zip(*outputs):
            np.testing.assert_array_equal(a["z"], b["z"])
            assert t >= a["due"] - 1e-8
            delivered.append(a)
    assert any(p["source"] == "stereo" for p in delivered)
    assert all(p["stamp"] < .5 for p in delivered if p["source"] == "stereo")
    assert sensors[0].failed


def test_filter_rejects_stale_and_outliers_without_resetting_covariance():
    P = np.diag([9.] * 3 + [.25] * 3)
    f = RelativeEKF(np.zeros(6), P)
    np.testing.assert_array_equal(f.P, P)
    stale = {"source": "gnss", "stamp": 0., "z": np.ones(6), "R": np.eye(6) * .01}
    assert not f.update(stale, 1.)
    assert not f.update({**stale, "stamp": 1., "z": np.full(6, 1000.)}, 1.1)
    assert f.update({**stale, "stamp": 1.}, 1.1)
    assert np.linalg.eigvalsh(f.P).min() > 0
    assert f.sigma_m < 3
    assert f.rejected == 2


@pytest.mark.parametrize("value", [float("nan"), -1., True])
def test_invalid_link_loss_is_rejected(value):
    with pytest.raises(ValueError):
        NearFieldOptions(communication_loss_s=value)


def test_options_do_not_relabel_distant_independent_gnss():
    with pytest.raises(ValueError):
        NearFieldOptions.from_dict({"gnss_position_sigma_m": 3})
    with pytest.raises(ValueError):
        NearFieldOptions(vision_loss_phase="anything")
    with pytest.raises(ValueError):
        RelativeEKF(np.zeros(6), -np.eye(6))


def test_handoff_com_conversion_inherits_covariance_and_time_without_mutation():
    import copy
    # Common nonzero attitude and angular rate exercise both position and velocity lever arms.
    angle = .01
    ep = [np.cos(angle/2), 0, 0, np.sin(angle/2)]
    R = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
    bodies, props, estimates = [], {}, []
    for role, name, p, c in (("target", "capture_target", [110, .3, .3], [.1, 0, 0]),
                             ("chaser", "cubesat_bus", [0, 0, 0], [0, .2, 0])):
        omega = np.array([0, 0, .0005])
        bodies.append(dict(body_id=f"rendezvous/{role}", position_m=p, velocity_m_s=[0, 10, 0],
                           attitude_wxyz=ep, angular_velocity_rad_s=omega.tolist()))
        props[name] = {"center_body_m": c}
        estimates.append(np.r_[np.array(p)+R@c, np.array([0, 10, 0])+R@np.cross(omega, c)].tolist())
    handoff = dict(guidance_frame="inertial-fixture", state={"sim_time_s": 123.45, "bodies": bodies}, body_properties=props,
                   gnss_estimates_eci=estimates, covariances=[np.eye(6).tolist(), (2*np.eye(6)).tolist()], stowed_joints=[])
    before = copy.deepcopy(handoff)
    states, P = origin_navigation(handoff)
    np.testing.assert_allclose(states, [[110, .3, .3, 0, 10, 0], [0, 0, 0, 0, 10, 0]], atol=1e-14)
    np.testing.assert_array_equal(P, 3*np.eye(6))
    assert handoff == before and handoff["state"]["sim_time_s"] == 123.45
    admit_handoff(handoff)
    handoff["state"]["bodies"][0]["position_m"] = [-110, 0, 0]
    with pytest.raises(ValueError, match="corridor"):
        admit_handoff(handoff)


def test_cooperative_telemetry_keeps_acquisition_timestamp_and_delay():
    sensor = SensorSimulator(SensorConfig())
    observation = {"rotation": np.eye(3), "hinge": .12}
    sensor.acquire(0., "GNSSApproach", np.zeros(6), [0, 0, 110], np.eye(3), observation)
    packets, _ = sensor.acquire(.1, "GNSSApproach", np.zeros(6), [0, 0, 110], np.eye(3), {"hinge": .2})
    packet = next(p for p in packets if p["source"] == "gnss")
    assert packet["stamp"] == 0. and packet["cooperative"]["hinge"] == .12
