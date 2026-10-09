# Local fixed-step MuJoCo dynamics backend

`local` is the platform default: every MJCF body is integrated by MuJoCo's own fixed-step
`implicitfast` at the 240 Hz physics tick. `--dynamics-backend basilisk` (or
`SPACE_SIM_DYNAMICS_BACKEND=basilisk`) selects the retired-from-default MJScene + adaptive RKF45 path,
which stays available as the accuracy reference. The component/assembly layer and the port table that
sits on top of this backend are described in [MUJOCO_CORE_ARCHITECTURE.md](MUJOCO_CORE_ARCHITECTURE.md).

## Why

* Under RKF45 every gripper/plug contact forces repeated step rejection: about 148 MuJoCo
  evaluations per 240 Hz step, against 1 for the local backend.
* MuJoCo collision detection is only reliable near the origin. With every body at orbital radius
  (6.9e6 m, as in MJScene's inertial frame) the same gripper/plug contact flickers (ncon 0-7); it is
  exact up to about 3e5 m (`run/diagnostics/local_stepper/orbital_offset_contact.py`). The Basilisk
  reference therefore shows a ~5 mm finger offset and a spurious reaction-wheel disturbance in
  contact that the local backend does not have. Accuracy comparisons must use contact-free runs.

## Architecture

* Basilisk propagates only the orbital reference point O (point-mass `Spacecraft`
  `localFrameOrigin`, Earth/Sun gravity, same SPICE ephemeris), in `graspTask` before IK.
* `LocalMujocoStepper` (`simulation/local_mujoco_stepper.py`, `graspTask` priority 1000) integrates
  all bodies in the local frame L (origin O, axes parallel to N, zero gravity) with an independent
  `mjModel` from the same MJCF (`native/local_mujoco_stepper.cpp`: a C ABI bound to the
  `mujoco.dll` Basilisk already loaded; the Python `mujoco` module is never imported). By default,
  shared BSK gravity models supply Earth/Sun differential COM forces at each substep through
  `xfrc_applied`. The old analytic Earth tide is an explicit regression mode only.
* `MJScene` stays as the state container and message publisher, so the render bridge, IK, attitude
  control and observations are unchanged. It never integrates in this mode (`isDynamicsSynced`),
  and it is not in `graspTask`; its publish-only copy has contact/constraints disabled and rigid
  flexcomps removed (same qpos layout, masses and inertias; checked at startup).
* Servo actuators are MuJoCo position servos: `force = clip(kp*(q_ref-q) + kd*(qd_ref-qd))`, with
  `ctrl = q_ref + kd/kp*qd_ref`, so damping is implicit. Requested/applied efforts are still
  published as `SingleActuatorMsg`. Other actuators (reaction wheels) read their `actuatorInMsg`
  each step (zero-order hold); the wheel drives run in `graspTask` at 240 Hz.
* External forces: `LocalMujocoStepper.add_body_wrench_input(body, CmdForceInertialMsg,
  CmdTorqueBodyMsg)` holds a per-body wrench for each step (thrusters, magnetorquers, drag, ...).
* State written to MJScene by init/reset code (`setPosition`/`setVelocity`, task-box plugs) is
  detected and adopted on the next step.
* O is re-based onto the system COM when it drifts more than 50 m (checked once per second).
* `simulation/mjscene_threadpool.py` refuses to probe a publish-only scene.

## Options

| Setting | Values | Meaning |
|---|---|---|
| `SPACE_SIM_DYNAMICS_BACKEND` / `--dynamics-backend` | `local` (default), `basilisk` | backend |
| `SPACE_SIM_ORBITAL_MODE` / `--orbital-mode` | `bsk` (default), `linear_tidal` | local orbital force provider |
| `SPACE_SIM_LOCAL_SUBSTEPS` / `--local-substeps` | 1-16 (default 1) | MuJoCo substeps per 240 Hz step |
| `SPACE_SIM_LOCAL_PUBLISH_STRIDE` | 1, 2, 4, 8 (default 2) | body/site kinematics publication stride |
| `SPACE_SIM_POSTURE_BACKEND` | `native` (default), `python` | IK geometry/posture implementation |

Publication after every physics step: joint state/rate messages, the scene state message, the bulk
states and servo efforts are always written (cheap; they read the bulk states). Body/site messages
(`MJScene.postIntegration`, ~0.6 ms in the live process) are written every `stride` steps and on
every 30 Hz render/observation step, so rendered and recorded frames are exact. The attitude
navigation now runs on the 120 Hz physics grid (a multiple of the stride, phase-aligned), so it reads
a body state exactly one physics step old; `timeWritten()` shows it. The integrated
state does not depend on the stride (tested).

## Measurements

Physics-only RTF (`tools/profile_simulation_runtime.py`: UE rendering, network and disk excluded),
contact scene `run/diagnostics/gripper-contact` unless noted, final code (2026-10-01). Frame time is
per 30 Hz frame (budget 33.3 ms). The machine was shared (browser etc., 30-40 % CPU load), so the
run-to-run spread is large. Results: `run/diagnostics/local_stepper/final3/`.

| Case | Backend | runs | RTF | frame p95 (ms) |
|---|---|---|---|---|
| grasp script (close on plug, drag, yaw), 6 s | basilisk RKF45 | 1 | 0.068 | 1879 |
| | local | 4 | 1.07-1.31 | 32-44 |
| | local + native posture | 4 | 1.15-1.49 | 29-39 |
| hold in contact, 3 s | basilisk RKF45 | 1 | 0.044 | 1453 |
| | local | 4 | 1.01-1.31 | 32-47 |
| | local + native posture | 4 | 1.27-1.58 | 27-35 |
| production plug model, six-axis sweep, 7 s | basilisk RKF45 | 1 | 0.107 | 1040 |
| | local | 4 | 0.88-0.93 | 46-50 |
| | local + native posture | 4 | 1.08-1.16 | 38-42 |
| contact-free sweep, 7 s | basilisk RKF45 | 2 | 0.30-0.32 | 235-259 |
| | local | 2 | 0.98-1.19 | 37-44 |

Cost split (profiled, production plug model sweep, local, python posture): stepper 2.6 s self of
9.4 s, of which native `mj_step` is about 1.1-1.8 ms per step (collision narrow phase of the 11
rigid task-box flexcomps alone is ~1.2 ms, independent of the contact count); IK + online posture
preference 3.0 s (posture 1.4 s, Python); render bridge 0.9 s; `postIntegration` 0.6 s; observation
snapshot 0.45 s. MuJoCo worker threads made the step slower (1-6 threads measured), so none are used.

## Accuracy (contact-free, local vs Basilisk)

* Teleop sweep, 7 s and 60 s (456 km of orbit): arm joints <= 6.2e-4 rad (J1-J5) and 2.5e-3 rad
  (J6, the stiffest), end effector 1.3e-4 m, bus attitude 9.6e-5 rad, finger 0.09 mm, inertial bus
  position 2e-5 m after 1 s and 5e-6 m after 60 s. Substeps 2/4 halve/quarter the arm errors.
* Native scripted grasp (gravity-free, 10 s) against RK4 at 2 ms: joints <= 6.2e-4 rad. Momentum is
  conserved only to first order while the arm moves (linear 1.9e-4, 9.3e-5, 4.7e-5 kg m/s at
  substeps 1/2/4; angular about COM 9.0e-5 / 4.5e-5 / 2.2e-5) and the error does not accumulate once
  motion stops. This exceeds the scenario's 5e-5 / 1e-5 acceptance thresholds, which were written
  for RK4; RKF45 conserves to 1e-9.

## Status and limits

* Contact cases (grasp, hold) run faster than real time with both posture backends. The production
  plug model sweep is at 0.88-0.93 with the Python posture backend and 1.08-1.16 with the native
  one; it is MuJoCo collision of the task-box flexcomps plus Python IK, not contact solving.
* 5-30 % of 30 Hz frames still exceed 33 ms on this shared machine; real time is an average, not a
  per-frame guarantee.
* `LocalMujocoStepper` is a `SysModel` in the Basilisk task and talks only through Basilisk
  messages. Session wiring goes through `simulation/physics_ports.py` and `simulation/assembly.py`.
