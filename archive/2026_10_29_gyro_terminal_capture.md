# Gyro-assisted terminal visual capture

When wrist markers leave view, holding their old pose in the moving service
frame does not account for arm-induced base motion. The new opt-in held-target
mode uses the existing shared 50 Hz service gyro and encoder mass-model COM to
propagate own motion, retaining original sample times at the 400 Hz control
rate. It passes the prediction to the existing approach/contact/force sequence.

The sensor, policy interfaces, physical model, masses/inertias, optics and
contact parameters are unchanged. No fixed inter-spacecraft connection is
introduced. Target truth remains outside visual navigation commands.

Safety checks reject bad sensor timing, epochs, frames and invalid/saturated
measurements. A sensor fault stops approach; an existing grasp stays in force
hold with the original finite/overload checks. Legacy visual-servo contact
guards are retained, and incompatible held-target configurations are rejected
before startup.

Tests include pure 50/400 Hz integration, visual loss/delay, explicit sensor
faults, native scheduler ordering and legacy-mode regressions. Native tests
are separate from basic CI because Basilisk and standalone Python MuJoCo must
not share a process. Local historical experiments completed five nominal/error
cases and one injected slow-bias case. The latter still grasped but did not
diagnose drift and had larger prediction error and target disturbance. Those
experiments are not a full latest-main scene requalification or a reliability
claim.

See [usage, timing and limitations](../space_sim_server/docs/GYRO_TERMINAL_CAPTURE.md).
Raw RGB, state/contact logs and machine configuration are intentionally not
committed. Normal startup does not enable a fault injector.
