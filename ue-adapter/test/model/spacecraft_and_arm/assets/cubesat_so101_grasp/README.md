# CubeSat + SO-101 pure-contact grasp model

This package extends the generated `cubesat_so101` model with an independent
free-floating capture target. It is a calibrated regression case for approach,
contact, gripper closure, retained grasp, withdrawal, and system momentum
conservation.

The target has its own `capture_target_free` joint, mass `0.10 kg`, and a narrow
cylindrical handle. Capture uses only MuJoCo contact and friction. The model has
no equality constraint, weld, or adhesion. Its stored default target pose is
deliberately contact-free; the scenario sets the calibrated task pose after
Basilisk initialization.

## Optional model development and simulation

The checked-in XML is used by the adapter's optional grasp examples; it does
not need regeneration to start the platform. Do not edit the generated XML
directly. `build_model.py` supports generation and `--check`; use a dedicated
uv, Conda or standard venv environment with MuJoCo 3.11.0 for this historical generator.

Run `scenarios/scenario_cubesat_so101_grasp.py` from
`test/model/spacecraft_and_arm` using the explicitly selected Python with
Basilisk/MJScene installed. An ephemeral `uv run --with mujoco` environment
alone does not provide Basilisk. Environment selection and adapter startup
are documented in the repository README.

The scenario returns JSON metrics and fails when grasp/momentum criteria are
not met. `--no-assert` is for calibration only. Optional `--video` output
requires compatible native MuJoCo, imageio-ffmpeg, Pillow and working graphics;
it replays recorded states, rather than running a second dynamics simulation.

The controller is a six-channel Basilisk PID with torque saturation and a
10-second quintic joint-space sequence. Contact debug visuals can be enabled
with `--contact-debug`.

## Stable target interface

| Type | Name |
|---|---|
| body | `capture_target` |
| free joint | `capture_target_free` |
| handle geom | `capture_target_handle` |
| main geom | `capture_target_body_geom` |
| grasp site | `capture_target_grasp` |

The full model has `nq=20`, `nv=18`, `nu=6`, two free joints, and total mass
`24.744006 kg`.

Momentum is reconstructed from every recorded Basilisk `qpos/qvel` sample with
native MuJoCo `mj_subtreeVel`. Mechanical energy is reported but is not asserted
constant because the PID actuators, damping, friction, and inelastic contact do
work or dissipate energy.
