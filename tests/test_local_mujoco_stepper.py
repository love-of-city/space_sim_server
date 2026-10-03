"""Local fixed-step MuJoCo dynamics backend against Basilisk MJScene and analytics.

Native: needs Basilisk (MJScene) and the built helper
(tools/build_native_acceleration.py --component local_mujoco_stepper ...).
Never imports Python MuJoCo into this process.
"""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("Basilisk")
from Basilisk.architecture import messaging, sysModel  # noqa: E402
from Basilisk.simulation import MJJointPIDController, mujoco, saturationSingleActuator  # noqa: E402
from Basilisk.utilities import SimulationBaseClass, macros  # noqa: E402

from simulation.local_mujoco_stepper import (  # noqa: E402
    LocalMujocoStepper, OrbitOrigin, dynamics_backend, local_substeps, publish_stride, servo_specs,
)
from simulation.native_integration import configure_scene_integrator  # noqa: E402

DT = 1.0 / 240.0
MJCF = """<mujoco model="local_stepper_test">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="bus" pos="0 0 0">
      <freejoint name="bus_free"/>
      <geom name="bus_geom" type="box" size=".2 .2 .2" mass="10"/>
      <body name="link" pos="0.3 0 0">
        <joint name="arm" type="hinge" axis="0 0 1" range="-3 3" limited="true" damping="0.5"/>
        <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" mass="1"/>
      </body>
      <body name="wheel" pos="0 0 0.3">
        <joint name="wheel_spin" type="hinge" axis="0 0 1"/>
        <geom type="cylinder" size=".1 .02" mass="1" contype="0" conaffinity="0"/>
      </body>
    </body>
    <body name="ball" pos="1.5 0 0">
      <freejoint name="ball_free"/>
      <geom type="sphere" size="0.05" mass="0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="arm_motor" joint="arm" ctrllimited="true" ctrlrange="-100 100"/>
    <motor name="wheel_motor" joint="wheel_spin" ctrllimited="true" ctrlrange="-0.2 0.2"/>
  </actuator>
</mujoco>
"""
KP, KD, LIMIT = 20.0, 1.0, 1.5


def STEPS_IN(seconds):
    """Steps after t=0 on this fixed (rounded 4166667 ns) test task; the live
    runtime uses the rational 240 Hz clock instead."""
    return int(round(seconds * 1e9)) // macros.sec2nano(DT)


class Reference(sysModel.SysModel):
    """Arm reference: smooth 0.6 rad move over 1 s, then hold."""

    def __init__(self):
        super().__init__()
        self.positionOutMsgs = [messaging.ScalarJointStateMsg()]
        self.velocityOutMsgs = [messaging.ScalarJointStateMsg()]

    def Reset(self, nanos):
        self.UpdateState(nanos)

    def UpdateState(self, nanos):
        t = min(nanos * 1e-9, 1.0)
        q = 0.6 * (10 * t**3 - 15 * t**4 + 6 * t**5)
        qd = 0.6 * (30 * t**2 - 60 * t**3 + 30 * t**4) if nanos * 1e-9 < 1.0 else 0.0
        self.positionOutMsgs[0].write(messaging.ScalarJointStateMsgPayload(state=q), nanos, self.moduleID)
        self.velocityOutMsgs[0].write(messaging.ScalarJointStateMsgPayload(state=qd), nanos, self.moduleID)


class WheelTorque(sysModel.SysModel):
    def __init__(self, torque):
        super().__init__()
        self.torque = torque
        self.actuatorOutMsg = messaging.SingleActuatorMsg()

    def UpdateState(self, nanos):
        self.actuatorOutMsg.write(messaging.SingleActuatorMsgPayload(input=self.torque), nanos, self.moduleID)


@pytest.fixture
def model_path(tmp_path: Path) -> Path:
    path = tmp_path / "local_stepper_test.xml"
    path.write_text(MJCF, encoding="utf-8")
    return path


def build(model_path, backend, *, substeps=1, wheel_torque=0.0, origin=None, rebase_distance_m=50.0,
          publish_every=1):
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("p")
    process.addTask(simulation.CreateNewTask("t", macros.sec2nano(DT)))
    scene = mujoco.MJScene.fromFile(str(model_path))
    if backend == "local":  # as scenario_sarm_grasp: the stepper publishes the scene
        process.addTask(simulation.CreateNewTask("scene_init", 10**18))
        simulation.AddModelToTask("scene_init", scene)
    else:
        simulation.AddModelToTask("t", scene)
    reference, wheel = Reference(), WheelTorque(wheel_torque)
    keep = [reference, wheel]
    scene.getSingleActuator("wheel_motor").actuatorInMsg.subscribeTo(wheel.actuatorOutMsg)
    if backend == "local":
        simulation.AddModelToTask("t", reference, 9000)
        simulation.AddModelToTask("t", wheel, 8000)
        stepper = LocalMujocoStepper(scene, model_path, servo_specs(["arm_motor"], ["arm"], [KP], [KD], [LIMIT], reference),
                                     origin=origin, substeps=substeps, publish_every=publish_every,
                                     rebase_distance_m=rebase_distance_m)
        simulation.AddModelToTask("t", stepper, 1000)
        keep.append(stepper)
    else:
        # Held per physics tick on both backends, as the live reference is.
        simulation.AddModelToTask("t", reference, 9000)
        scene.AddModelToDynamicsTask(wheel, 8500)
        joint = scene.getBody("link").getScalarJoint("arm")
        pid = MJJointPIDController.JointPIDController()
        pid.setProportionalGain(KP)
        pid.setDerivativeGain(KD)
        pid.setIntegralGain(0.0)
        pid.desiredPosInMsg.subscribeTo(reference.positionOutMsgs[0])
        pid.desiredVelInMsg.subscribeTo(reference.velocityOutMsgs[0])
        pid.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
        pid.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)
        limiter = saturationSingleActuator.SaturationSingleActuator()
        limiter.setMinInput(-LIMIT)
        limiter.setMaxInput(LIMIT)
        limiter.actuatorInMsg.subscribeTo(pid.outputOutMsg)
        scene.getSingleActuator("arm_motor").actuatorInMsg.subscribeTo(limiter.actuatorOutMsg)
        scene.AddModelToDynamicsTask(pid, 8000)
        scene.AddModelToDynamicsTask(limiter, 7000)
        keep += [pid, limiter]
        configure_scene_integrator(scene)
        stepper = None
    simulation.InitializeSimulation()
    return simulation, scene, stepper, keep


def run_to(simulation, seconds):
    simulation.ConfigureStopTime(int(round(seconds * 1e9)))
    simulation.ExecuteSimulation()


def arm_history(model_path, backend, substeps=1):
    simulation, scene, stepper, keep = build(model_path, backend, substeps=substeps)
    reader = scene.getBody("link").getScalarJoint("arm").stateOutMsg.addSubscriber()
    samples = []
    for k in range(1, 61):
        run_to(simulation, k / 30.0)
        samples.append(float(reader().state))
    return np.array(samples), stepper, keep


def test_backend_and_substep_selection(monkeypatch):
    monkeypatch.delenv("SPACE_SIM_DYNAMICS_BACKEND", raising=False)
    assert dynamics_backend() == "local"          # platform default: fixed-step MuJoCo
    assert dynamics_backend("basilisk") == "basilisk"  # explicit reference path
    monkeypatch.setenv("SPACE_SIM_DYNAMICS_BACKEND", "Basilisk")
    assert dynamics_backend() == "basilisk"
    with pytest.raises(ValueError):
        dynamics_backend("rk4")
    assert local_substeps(3) == 3
    with pytest.raises(ValueError):
        local_substeps(0)
    with pytest.raises(ValueError):
        local_substeps(17)
    assert publish_stride(4) == 4
    for bad in (0, 3, 16):
        with pytest.raises(ValueError):
            publish_stride(bad)


def test_publish_stride_only_thins_messages_not_physics(model_path):
    def final_state(stride):
        simulation, scene, stepper, _keep = build(model_path, "local", publish_every=stride)
        joint = scene.getBody("link").getScalarJoint("arm").stateOutMsg.addSubscriber()
        run_to(simulation, 1.0)
        # 240 steps are a multiple of every allowed stride: the last one is published.
        return stepper.qpos.copy(), float(joint().state), stepper.step_count

    full_q, full_joint, steps = final_state(1)
    for stride in (2, 4, 8):
        q, joint, count = final_state(stride)
        assert count == steps
        np.testing.assert_array_equal(q, full_q)  # the integrated state is bit-identical
        assert joint == pytest.approx(full_joint, abs=1e-2)  # message may lag by < stride steps


def test_joint_messages_are_fresh_every_step_and_kinematics_follow_the_stride(model_path):
    simulation, scene, stepper, keep = build(model_path, "local", publish_every=2)
    joint = scene.getBody("link").getScalarJoint("arm").stateOutMsg.addSubscriber()
    site = scene.getBody("bus").getOrigin().stateOutMsg.addSubscriber()
    period = macros.sec2nano(DT)
    before, after = [], []

    class Probe(sysModel.SysModel):
        def __init__(self, log):
            super().__init__()
            self.log = log

        def UpdateState(self, nanos):
            self.log.append((nanos // period, joint.timeWritten() // period, site.timeWritten() // period))

    keep += [Probe(before), Probe(after)]
    simulation.AddModelToTask("t", keep[-2], 5000)   # like IK: before the stepper
    simulation.AddModelToTask("t", keep[-1], -5000)  # like the render bridge: after it
    run_to(simulation, 0.5)
    # Joint messages: the previous step before the stepper, the current one
    # after it, on every tick, exactly as when MJScene integrates in the task.
    assert all(j == tick - 1 for tick, j, _ in before if tick > 0)
    assert all(j == tick for tick, j, _ in after)
    # Body/site kinematics: every 2nd step and always on render ticks; the
    # message time reveals the one-step age in between.
    assert all(s == tick for tick, _, s in after if tick % 2 == 0)
    assert all(s == tick - 1 for tick, _, s in after if tick % 2 == 1)
    assert stepper.kinematics_publish_count < stepper.step_count


def test_servo_tracks_like_the_basilisk_pid_limiter_chain(model_path):
    reference, _, _keep = arm_history(model_path, "basilisk")
    for substeps, tolerance in ((1, 4e-3), (4, 1.2e-3)):
        local, stepper, _keep_local = arm_history(model_path, "local", substeps)
        assert stepper.step_count == STEPS_IN(2.0)
        # First-order fixed-step discretization of the same continuous system.
        assert np.max(np.abs(local - reference)) < tolerance
        assert abs(local[-1] - reference[-1]) < 1e-3  # both settle on the 0.6 rad reference
    assert abs(reference[-1] - 0.6) < 5e-3


def test_scene_publishes_the_local_state_without_integrating(model_path):
    simulation, scene, stepper, _keep = build(model_path, "local")
    scene.getBody("bus").setVelocity([0.1, 0.0, 0.0])
    joint = scene.getBody("link").getScalarJoint("arm").stateOutMsg.addSubscriber()
    bus = scene.getBody("bus").getOrigin().stateOutMsg.addSubscriber()
    applied = stepper.appliedOutMsgs[0].addSubscriber()
    run_to(simulation, 1.0)
    assert scene.isDynamicsSynced
    assert stepper.step_count == STEPS_IN(1.0)
    assert joint().state == pytest.approx(stepper.qpos[stepper.servo_qpos[0]], abs=0.0)
    # Momentum exchange with the arm moves the bus, but the imposed drift dominates.
    assert bus().r_BN_N[0] == pytest.approx(0.1, abs=0.02)
    assert joint.timeWritten() == bus.timeWritten() == STEPS_IN(1.0) * macros.sec2nano(DT)
    # The applied effort seen through the MJScene actuator equals the servo force.
    assert scene.getSingleActuator("arm_motor").actuatorInMsg().input == applied().input
    assert abs(applied().input) <= LIMIT


def test_momentum_is_restored_after_internal_motion_stops(model_path):
    simulation, scene, stepper, _keep = build(model_path, "local")
    run_to(simulation, DT)
    start = stepper.momentum()
    peak = 0.0
    for k in range(2, 721):
        run_to(simulation, k * DT)
        if k % 24 == 0:
            peak = max(peak, float(np.linalg.norm(stepper.momentum()["linear_kg_m_s"] - start["linear_kg_m_s"])))
    end = stepper.momentum()
    # Internal servo/wheel forces cannot change total momentum. The implicit
    # scheme conserves it only to first order while the arm moves, and the
    # error does not accumulate once motion stops.
    assert peak < 5e-3
    assert np.linalg.norm(end["linear_kg_m_s"] - start["linear_kg_m_s"]) < 2e-4
    assert np.linalg.norm(end["angular_about_com_kg_m2_s"] - start["angular_about_com_kg_m2_s"]) < 2e-4


def test_command_actuators_follow_their_scene_messages(model_path):
    speeds = {}
    for backend in ("basilisk", "local"):
        simulation, scene, _stepper, _keep = build(model_path, backend, wheel_torque=0.05)
        reader = scene.getBody("wheel").getScalarJoint("wheel_spin").stateDotOutMsg.addSubscriber()
        run_to(simulation, 1.0)
        speeds[backend] = float(reader().state)
    assert speeds["basilisk"] > 1.0
    assert speeds["local"] == pytest.approx(speeds["basilisk"], rel=0.02)


def test_external_state_writes_are_adopted(model_path):
    simulation, scene, stepper, _keep = build(model_path, "local")
    ball = scene.getBody("ball")
    reader = ball.getOrigin().stateOutMsg.addSubscriber()
    run_to(simulation, 0.5)
    ball.setPosition([3.0, 1.0, 0.0])
    ball.setVelocity([0.0, 0.2, 0.0])
    run_to(simulation, 1.0)
    assert stepper.external_writes == 1
    assert reader().r_BN_N[0] == pytest.approx(3.0, abs=1e-9)
    assert reader().r_BN_N[1] == pytest.approx(1.0 + 0.2 * 0.5, abs=1e-6)


def test_orbit_origin_offsets_and_earth_tidal_acceleration(model_path):
    mu, radius = 3.986004418e14, 6_878_136.6
    r_o, v_o = np.array([radius, 0.0, 0.0]), np.array([0.0, 7612.6, 0.0])

    def orbit(now):  # O on a straight line: tidal term only, no uniform field here
        return r_o + v_o * now, v_o

    origin = OrbitOrigin(orbit, mu=mu)
    simulation, scene, stepper, _keep = build(model_path, "local", origin=origin)
    ball = scene.getBody("ball")
    ball.setPosition(r_o + [1.5, 0.0, 0.0])
    ball.setVelocity(v_o)
    scene.getBody("bus").setPosition(r_o)
    scene.getBody("bus").setVelocity(v_o)
    reader = ball.getOrigin().stateOutMsg.addSubscriber()
    run_to(simulation, 10.0)
    # Radial tidal stretch: a = 2 mu / r^3 * x (evaluated at the local offset).
    expected_velocity = 2.0 * mu / radius**3 * 1.5 * 10.0
    relative_velocity = np.asarray(reader().v_BN_N) - v_o
    assert relative_velocity[0] == pytest.approx(expected_velocity, rel=0.02)
    # O moves ~76 km along +y, rotating the radial direction by ~0.011 rad:
    # the resulting cross-track tidal component is real but about 1% of radial.
    assert abs(relative_velocity[1]) < 0.02 * expected_velocity
    assert np.asarray(reader().r_BN_N)[1] == pytest.approx(v_o[1] * reader.timeWritten() * 1e-9, abs=1e-5)


def test_rebase_keeps_inertial_states_continuous(model_path):
    origin = OrbitOrigin()  # fixed, gravity-free origin with an internal offset
    simulation, scene, stepper, _keep = build(model_path, "local", origin=origin, rebase_distance_m=0.5)
    for name in ("bus", "ball"):
        scene.getBody(name).setVelocity([1.0, 0.0, 0.0])
    ball = scene.getBody("ball").getOrigin().stateOutMsg.addSubscriber()
    run_to(simulation, 3.0)
    assert stepper.rebase_count >= 1
    # Inertial motion is unaffected by moving O: uniform drift of the free ball.
    assert ball().r_BN_N[0] == pytest.approx(1.5 + ball.timeWritten() * 1e-9, abs=1e-6)
    assert float(np.linalg.norm(stepper.momentum()["com_m"])) < 1.5


def test_body_wrench_input_applies_held_inertial_force_and_body_torque(model_path):
    simulation, scene, stepper, keep = build(model_path, "local")
    force, torque = messaging.CmdForceInertialMsg(), messaging.CmdTorqueBodyMsg()
    force.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.02, 0.0, 0.0]))
    torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0.0, 0.0, 1.0e-4]))
    keep += [force, torque]
    stepper.add_body_wrench_input("ball", force, torque, source="drag")
    with pytest.raises(ValueError, match="already registered"):
        stepper.add_body_wrench_input("ball", force, source="drag")
    with pytest.raises(ValueError, match="unknown"):
        stepper.add_body_wrench_input("no_such_body", force, source="drag")
    ball = scene.getBody("ball").getOrigin().stateOutMsg.addSubscriber()
    run_to(simulation, 1.0)
    elapsed = ball.timeWritten() * 1e-9
    mass, inertia = 0.2, 0.4 * 0.2 * 0.05**2  # solid sphere
    assert ball().v_BN_N[0] == pytest.approx(0.02 / mass * elapsed, rel=1e-3)
    assert ball().r_BN_N[0] == pytest.approx(1.5 + 0.5 * 0.02 / mass * elapsed**2, abs=2e-3)
    assert ball().omega_BN_B[2] == pytest.approx(1.0e-4 / inertia * elapsed, rel=1e-3)
    assert abs(ball().v_BN_N[1]) < 1e-9 and abs(ball().omega_BN_B[0]) < 1e-9


def test_several_wrench_sources_on_one_body_are_summed(model_path):
    """Environment and actuator forces add; no source needs to know the others."""
    simulation, scene, stepper, keep = build(model_path, "local")
    first, second = messaging.CmdForceInertialMsg(), messaging.CmdForceInertialMsg()
    first.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.02, 0.0, 0.0]))
    second.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.01, -0.03, 0.0]))
    keep += [first, second]
    stepper.add_body_wrench_input("ball", first, source="drag")
    stepper.add_body_wrench_input("ball", second, source="thruster")
    ball = scene.getBody("ball").getOrigin().stateOutMsg.addSubscriber()
    run_to(simulation, 1.0)
    elapsed = ball.timeWritten() * 1e-9
    mass = 0.2
    assert ball().v_BN_N[0] == pytest.approx(0.03 / mass * elapsed, rel=1e-3)
    assert ball().v_BN_N[1] == pytest.approx(-0.03 / mass * elapsed, rel=1e-3)


def test_servo_can_be_connected_after_construction(model_path):
    """The assembly layer wires references after building the core."""
    simulation, scene, stepper, keep = build(model_path, "local")
    reference = Reference()
    keep.append(reference)
    with pytest.raises(ValueError, match="already served"):
        stepper.connect_servo(servo_specs(["arm_motor"], ["arm"], [KP], [KD], [LIMIT], reference)[0])
    other = Reference()
    keep.append(other)
    stepper.connect_servo(servo_specs(["wheel_motor"], ["wheel_spin"], [1.0], [0.05], [0.2], other)[0])
    assert set(stepper.actuator_names[i] for i in stepper.command_index) == set()
    run_to(simulation, 0.5)
    assert stepper.servos_finalized
