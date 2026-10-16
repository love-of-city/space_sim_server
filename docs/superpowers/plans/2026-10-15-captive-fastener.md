# Captive fastener implementation plan

Goal: replace only the locking rod/fastener SDF pair with a captive helical joint,
preserving the rod hinge, visual bodies, grasp contacts and the original SDF model.
User approved this scope on 2026-10-15; implementation proceeds inline.

Geometry: rod-local +Y is the thread axis; fastener-local +Z points toward the
hook. Rod cap begins at Y=0.129 m, radius 0.0175 m. Fastener is 0.065 m long;
hook contact-envelope Y is 0.06112857 m. Envelope gap is 0.00287143 m, but it is NOT a seating limit.
This is a geometric envelope estimate, not a measured assembly tolerance.

Architecture: parent fastener to locking_rod while preserving its world pose.
Use co-axial slide and hinge, z=0.016/(2*pi)*theta, bounded slide, rotational
frictionloss and damping. Actual hook collision supplies seating; joint-limit compliance supplies retention
reaction; no kinematic pose teleport or external one-sided torque. Existing
MuJoCo solver supplies reactions to the rod and satellite. Keep SDF as an
explicit generated comparison. Source delivery bytes remain unchanged.

- [x] Add failing tests for pose preservation, finite stroke, SDF comparison,
  rejection of unrelated malformed bodies and attached-body orbit initialization.
- [x] Implement geometry-derived model transformation and provenance; make
  captive version the generated platform model, with an SDF build option.
- [x] Run focused tests and compile/load actual models in MuJoCo 3.7.0.
- [x] Exercise applied torque in both directions, stop retention, reversal,
  zero torque stiction, rod motion and mass/pose invariance; record trajectories.
- [x] Measure repeated no-render physics cost for both models with the same
  runtime, dt and state; distinguish these results from full gripper acceptance.
- [x] Run server checks, independent review, document measured limits and results.

Review focus: radians versus source degrees; geometry frames and source mass;
free-body orbit initialization; unintended SDF pairs remaining; force/torque
conservation and soft-limit penetration at the 240 Hz platform time step.

Friction/damping/compliance are explicit simulation parameters, not real hardware
calibration. No auto restart or replacement of the currently running main checkout.

Final correction: real seating occurs near 5.1 mm. Use 54 mm thread-runout safety range; never clamp at the projected 2.87 mm envelope gap. See space_sim_server/docs/CAPTIVE_FASTENER.md for measured results and scope.

