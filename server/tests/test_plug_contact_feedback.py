"""Bounded insert feedback keeps friction preload separate from nominal motion."""

import numpy as np
import pytest

from simulation.scripted_tasks.plug_contact_feedback import PlugContactFeedback


class LinearChain:
    """Decouple translation and joints so preload accumulation is observable."""

    def forward(self, joints):
        return np.asarray(joints[:3], dtype=float), np.eye(3)

    def jacobian(self, _joints):
        return np.eye(6)


def step(controller, *, target=None, gap=0., dt=.02, measured=None, speed=100., lower=None, upper=None,
         motion="insert", extraction_travel=0., measured_velocities=None, preload_allowed=True):
    return controller.step(
        LinearChain(), np.zeros(6) if measured is None else measured,
        np.zeros(3) if target is None else np.asarray(target), np.eye(3),
        axial_velocity_m_s=0., seat_gap_m=gap, dt=dt,
        max_speed_rad_s=speed,
        lower=np.full(6, -10.) if lower is None else lower,
        upper=np.full(6, 10.) if upper is None else upper,
        motion=motion, extraction_travel_m=extraction_travel,
        measured_joint_velocities=measured_velocities,
        preload_allowed=preload_allowed,
    )


def preload_controller():
    controller = PlugContactFeedback(joint_stiffness_nm_rad=np.full(6, 500.),
                                     friction_force_n=20., friction_travel_m=.008)
    controller.reset(np.zeros(6))
    return controller


def test_stationary_contact_preload_converges_without_accumulating_nominal_motion():
    controller = preload_controller()
    first = step(controller)
    assert -.04 < first[2] < 0.
    for _ in range(500):
        reference = step(controller)
    np.testing.assert_allclose(reference, [0., 0., -.04, 0., 0., 0.], atol=1e-10)
    np.testing.assert_allclose(controller.nominal_reference, 0., atol=1e-12)
    assert controller.preload_weight == pytest.approx(1.)


def test_friction_preload_uses_each_joint_actual_stiffness():
    controller = PlugContactFeedback(joint_stiffness_nm_rad=np.array([500., 500., 1000., 500., 500., 500.]),
                                     friction_force_n=20., friction_travel_m=.008)
    controller.reset(np.zeros(6))
    for _ in range(500):
        reference = step(controller)
    assert reference[2] == pytest.approx(-.02, abs=1e-10)
    np.testing.assert_allclose(controller.nominal_reference, 0., atol=1e-12)


def test_uniform_reference_limit_preserves_motion_direction():
    controller = PlugContactFeedback()
    controller.reset(np.zeros(6))
    reference = step(controller, target=[1., 2., 3.], dt=1.)
    np.testing.assert_allclose(reference[:3], [.02, .04, .06], atol=1e-12)
    np.testing.assert_allclose(reference[3:], 0., atol=1e-12)


def test_saturation_back_calculates_nominal_and_reverses_without_windup():
    controller = preload_controller()
    for _ in range(100):
        reference = step(controller, target=[1., 2., 3.], dt=.1)
        assert np.max(np.abs(reference)) <= .06 + 1e-12
        assert np.max(np.abs(controller.nominal_reference)) <= .10 + 1e-12
    expected_bias = np.array([0., 0., -.04*controller.preload_weight, 0., 0., 0.])
    np.testing.assert_allclose(controller.nominal_reference+expected_bias, reference, atol=1e-12)
    reversed_reference = step(controller, target=[-1., -2., -3.], dt=.1)
    assert np.all(reversed_reference[:3] < 0.)
    assert np.max(np.abs(reversed_reference)) <= .06 + 1e-12


def test_reset_restarts_preload_and_keeps_new_stage_reference():
    controller = preload_controller()
    for _ in range(20):
        step(controller)
    entered = np.array([.1, -.1, .03, 0., 0., 0.])
    controller.reset(entered)
    assert controller.preload_weight == 0.
    np.testing.assert_allclose(controller.nominal_reference, entered)
    np.testing.assert_allclose(step(controller, dt=0., measured=entered), entered)


def test_zero_time_does_not_advance_preload_or_joint_reference():
    controller = preload_controller()
    reference = step(controller, target=[1., 2., 3.], dt=0.)
    np.testing.assert_array_equal(reference, np.zeros(6))
    np.testing.assert_array_equal(controller.nominal_reference, np.zeros(6))
    assert controller.preload_weight == 0.


def test_disabled_retention_has_no_preload_without_stiffness_configuration():
    controller = PlugContactFeedback(friction_force_n=0., friction_travel_m=0.)
    controller.reset(np.zeros(6))
    for _ in range(50):
        reference = step(controller, gap=-.001)
    np.testing.assert_array_equal(reference, np.zeros(6))
    assert controller.preload_weight == 0.


def test_output_respects_joint_bounds_and_nominal_is_back_calculated():
    controller = preload_controller()
    lower, upper = np.full(6, -.01), np.full(6, .01)
    reference = step(controller, target=[1., 2., 3.], dt=1., lower=lower, upper=upper)
    assert np.all(reference >= lower) and np.all(reference <= upper)
    expected_bias = np.array([0., 0., -.04*controller.preload_weight, 0., 0., 0.])
    np.testing.assert_allclose(controller.nominal_reference+expected_bias, reference, atol=1e-12)


def test_speed_limit_covers_preload_ramp_and_nominal_motion_together():
    controller = preload_controller()
    previous = np.zeros(6)
    dt, speed = .02, .1
    # Nominal motion and the rising contact bias both request negative Z.
    # Limiting only nominal velocity leaves the applied servo target too fast.
    for tick in range(100):
        reference = step(controller, target=[0., 0., -.2],
                         gap=0. if tick < 50 else .02, dt=dt, speed=speed)
        assert np.max(np.abs(reference-previous)) <= speed*dt + 1e-12
        previous = reference.copy()
    bias = np.array([0., 0., -.04*controller.preload_weight, 0., 0., 0.])
    np.testing.assert_allclose(controller.nominal_reference+bias, reference, atol=1e-12)


def test_release_handoff_preserves_applied_reference_and_unloads_continuously():
    from test_independent_plug_tasks import make_task

    task, sample = make_task("cylindrical_plug_insert")
    measured = sample.joints[:6]
    position, rotation = task.chain.forward(measured)
    task._contact_feedback = PlugContactFeedback(
        joint_stiffness_nm_rad=np.full(6, 500.),
        friction_force_n=20., friction_travel_m=.008,
    )
    task._contact_feedback.reset(measured)
    for _ in range(100):
        task.reference[:6] = task._contact_feedback.step(
            task.chain, measured, position, rotation,
            axial_velocity_m_s=0., seat_gap_m=0., dt=.005,
            max_speed_rad_s=task.config.arm_speed_rad_s,
            lower=task.lower, upper=task.upper,
        )
    applied = task.reference[:6].copy()
    assert np.linalg.norm(applied-task._contact_feedback.nominal_reference) > 1e-5
    task.stage = "insert"
    task._enter("release", sample)
    np.testing.assert_array_equal(task.reference[:6], applied)
    np.testing.assert_array_equal(task._release_hold_joints, measured)
    assert np.linalg.norm(task.reference[:6]-measured) > 1e-5
    task._hold_release_pose(sample, .005)
    # The release controller starts at the loaded output, then unwinds it.
    assert np.max(np.abs(task.reference[:6]-applied)) <= .08*.005 + 1e-12
    assert 0. < np.linalg.norm(task.reference[:6]-measured) < np.linalg.norm(applied-measured)


@pytest.mark.parametrize("motion,direction", [("insert", -1.), ("extract", 1.)])
def test_contact_preload_opposes_friction_in_the_motion_direction(motion, direction):
    controller = preload_controller()
    for _ in range(100):
        reference = step(controller, motion=motion, measured_velocities=np.zeros(6))
    assert reference[2] == pytest.approx(direction*.04, abs=1e-9)
    np.testing.assert_allclose(controller.nominal_reference, 0., atol=1e-12)


def test_extraction_holds_compensation_through_friction_zone_then_unloads_smoothly():
    controller = preload_controller()
    for _ in range(200):
        reference = step(controller, motion="extract", extraction_travel=.008,
                         measured_velocities=np.zeros(6))
    assert controller.preload_weight == pytest.approx(1., abs=1e-9)
    assert reference[2] == pytest.approx(.04, abs=1e-9)
    previous_weight = controller.preload_weight
    reference = step(controller, motion="extract", extraction_travel=.012,
                     measured_velocities=np.zeros(6))
    assert controller.preload_weight == pytest.approx(previous_weight*np.exp(-.02/.1))
    assert 0. < reference[2] < .04
    for _ in range(150):
        reference = step(controller, motion="extract", extraction_travel=.012,
                         measured_velocities=np.zeros(6))
    assert reference[2] == pytest.approx(0., abs=1e-12)
    np.testing.assert_allclose(controller.nominal_reference, 0., atol=1e-12)


def test_recovery_unloads_existing_preload_instead_of_loading_backoff_direction():
    controller = preload_controller()
    for _ in range(100):
        step(controller)
    previous_force = controller.preload_force_n
    reference = step(controller, preload_allowed=False)
    assert controller.preload_force_n == pytest.approx(previous_force*np.exp(-.02/.1))
    assert -.04 < reference[2] < 0.
    for _ in range(150):
        reference = step(controller, preload_allowed=False)
    assert reference[2] == pytest.approx(0., abs=1e-12)


def test_extraction_uses_filtered_measured_axial_velocity_for_damping():
    controller = PlugContactFeedback()
    controller.reset(np.zeros(6))
    measured_velocities = np.array([0., 0., .01, 0., 0., 0.])
    reference = step(controller, dt=.005, motion="extract",
                     measured_velocities=measured_velocities)
    expected_velocity = (1.-np.exp(-1.))*.01
    assert controller.measured_axial_velocity_m_s == pytest.approx(expected_velocity)
    assert reference[2] == pytest.approx(-8.*expected_velocity*.005/(1.+1e-6))


def test_extraction_retains_measured_lateral_recentering():
    controller = PlugContactFeedback()
    controller.reset(np.zeros(6))
    reference = step(controller, target=[.01, 0., 0.], motion="extract",
                     measured_velocities=np.zeros(6))
    assert reference[0] == pytest.approx(6.*.01*.02/(1.+1e-6))


def test_configured_extraction_gains_control_lateral_correction_and_velocity_damping():
    controller = PlugContactFeedback(extraction_lateral_gain_s_inv=4., extraction_velocity_gain=6.)
    controller.reset(np.zeros(6))
    reference = step(controller, target=[.01, 0., 0.], dt=.005, motion="extract",
                     measured_velocities=np.array([0., 0., .01, 0., 0., 0.]))
    filtered_velocity = (1.-np.exp(-1.))*.01
    assert reference[0] == pytest.approx((3.+4.)*.01*.005/(1.+1e-6))
    assert reference[2] == pytest.approx(-6.*filtered_velocity*.005/(1.+1e-6))


@pytest.mark.parametrize("name,maximum", [("extraction_lateral_gain_s_inv", 6.), ("extraction_velocity_gain", 8.)])
@pytest.mark.parametrize("value", [0., -1., float("nan"), float("inf")])
def test_contact_feedback_rejects_invalid_extraction_gains(name, maximum, value):
    with pytest.raises(ValueError, match="Extraction"):
        PlugContactFeedback(**{name: value})
    with pytest.raises(ValueError, match="Extraction"):
        PlugContactFeedback(**{name: maximum+.001})


def test_zero_time_preserves_loaded_applied_reference():
    controller = preload_controller()
    for _ in range(100):
        applied = step(controller)
    force = controller.preload_force_n
    reference = step(controller, target=[1., 2., 3.], gap=.02, dt=0.)
    np.testing.assert_array_equal(reference, applied)
    assert controller.preload_force_n == force


def test_servo_offset_safety_limit_takes_priority_over_speed_when_measurement_jumps():
    controller = PlugContactFeedback()
    controller.reset(np.zeros(6))
    measured = np.array([.2, 0., 0., 0., 0., 0.])
    reference = step(controller, measured=measured, speed=.1)
    assert np.max(np.abs(reference-measured)) <= controller.reference_limit_rad+1e-12
    assert reference[0] == pytest.approx(.14)
    np.testing.assert_array_equal(controller.applied_reference, reference)


def test_extraction_requires_measured_joint_velocities():
    controller = preload_controller()
    with pytest.raises(ValueError, match="measured joint velocities"):
        step(controller, motion="extract")


@pytest.mark.parametrize("stiffness", [
    np.zeros(6), np.full(6, -1.), np.full(6, np.nan),
    np.full(6, np.inf), np.ones(5),
])
def test_invalid_joint_stiffness_is_rejected(stiffness):
    with pytest.raises(ValueError):
        PlugContactFeedback(joint_stiffness_nm_rad=stiffness,
                            friction_force_n=20., friction_travel_m=.008)


@pytest.mark.parametrize("force", [-1., float("nan"), float("inf")])
def test_invalid_friction_force_is_rejected(force):
    with pytest.raises(ValueError):
        PlugContactFeedback(joint_stiffness_nm_rad=np.full(6, 500.),
                            friction_force_n=force, friction_travel_m=.008)


@pytest.mark.parametrize("travel", [-.001, float("nan"), float("inf")])
def test_invalid_friction_travel_is_rejected(travel):
    with pytest.raises(ValueError):
        PlugContactFeedback(joint_stiffness_nm_rad=np.full(6, 500.),
                            friction_force_n=20., friction_travel_m=travel)
