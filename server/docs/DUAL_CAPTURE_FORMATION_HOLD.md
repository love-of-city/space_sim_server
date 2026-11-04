# Dual capture formation preview

## Current preview: unforced co-orbital translation

The saved preview now sets `randomization.coorbital_initialization.enabled: true`
and `formation_hold.enabled: false`. The former uses `separation_m` only once,
to select the target's initial phase on the service spacecraft's circle.
Both free spacecraft then evolve under the existing Earth/Sun gravity model,
evaluated at their respective locations. No formation wrench sources are
registered, no target attitude actuator is added, and no relative-distance
constraint is installed. The service's existing wheel controller keeps its
orbital attitude reference via `OrbitAttitudeReferenceComponent`; this module
reads only the service state and has no target state or distance input.
The target starts with the corresponding orbit-frame angular rate and then
evolves freely. Masses, inertias, gravity settings and wheel gains are unchanged.
`coorbital_free_motion` logs measured relative states every ten simulation
seconds. This mode does not guarantee rigid relative pose under disturbances.

### Whole-spacecraft COM initialization

The adjacent-face ready scene uses
`coorbital_initialization.reference_point: assembly_com`. The local authoritative
model measures each spacecraft's compiled mass and complete attached-subtree COM
after all initial joint positions have been assigned. This includes both arms,
the gripper and attached task-box parts; independent world-level free objects
are separate assemblies. It does not substitute the main body's own inertial
offset for the complete spacecraft COM, and does not change any inertial data.

The configured circular orbit describes the service COM. `separation_m` is the
COM arc separation on that same circle. With body-to-inertial rotation `R` and
body-frame COM offset `c`, body-origin states are initialized as
`r_origin = r_COM - R*c` and
`v_origin = v_COM - cross(omega_orbit, R*c)`. All joint rates initially vanish.
Only initial states are assigned; no formation wrench or inter-spacecraft weld
is introduced. Omitting `reference_point` retains legacy body-origin semantics
for existing scene replays.

The current ready scene initializes the operating arm directly at the upstream
SARM operating pose `[90, -60, 60, 0, -90, 0]` degrees. The capture arm uses a
separately solved pose `[89.087993, -134.740217, 69.068578, 45.673918,
89.687932, 0.856473]` degrees, facing the target X interface along its insertion
axis with a 0.5 m standoff. Both arms start with zero joint rates and no
zero-start unfolding. It uses a 1.15 m COM arc separation; the service
orientation and all model masses/inertias are unchanged.
`grip_frame_standoff_m: 0.5` verifies the initial distance to
the X-interface centre at target-local `[0.1755, 0.05475, 0.0420787001299893]` m.
Startup fails if that distance differs by more than 1 mm. The UI body-origin
distance is about 1.0904 m; it is not the grip-to-interface distance. Body spin
and flexible/articulated motion may change the interface distance after startup.
`gravity_configuration` records measured assembly properties and both initial
COM states for orbit-element verification.

### Target visual markers

The ready scene's `render_profile.visual_attachments` attaches the supplied
5 cm ArUco surface-plate geometry to the rendered `teleop/capture_target` body.
The four marker boards and their black/white cells are body-local display
geometry only. They are added after the physics scene has been built, without
changing the MJCF, masses, inertias, contacts, camera mounts or initial states.
The configuration is saved with the scene so ordinary restart/reset retains it.
The source geometry and its SHA256 are stored under
`config/visual_attachments/target_aruco_5cm/`. No pose-estimation algorithm or
new-arm calibration is enabled by this visual import.

## Previous opt-in controlled experiment

Opt in on `sarm-dual-capture26` by setting `randomization.formation_hold` in a
saved scene: `enabled: true`, `separation_m: 1.5`, `force_limit_n: 0.2`,
`torque_limit_nm: 0.02`. Circular orbit and local dynamics backend are required.
Set `enabled: false` and restart to return to unforced flight; do this before
testing an uncontrolled target's response to capture contact.

The service origin starts on the saved circle and the target origin starts
1.5 metres farther along it. Each receives the velocity at its own orbital
phase. Their inertial velocity vectors are therefore slightly different;
zero relative velocity here means zero in the rotating orbital frame.
Service -Z faces forward along track; target +Y faces the service spacecraft.
The service's free task-box plugs inherit its initial rotation and velocity.

`randomization.capture_arm_joint_position_rad` optionally sets six capture-arm
startup angles. The preview points `grip_frame` at the source-CAD X interface
at target-local `[0.1755, 0.07825, 0.0420787]` m, with 0.25 m origin standoff.
These coordinates use the ground-validation model's recorded origin shift.
The old `capture_target_grasp` site is on a different panel and is not used.
This is a prepared initial pose, not an executed deployment trajectory.

`FormationHoldComponent` runs at the body's publication cadence (50 Hz).
It aligns orbital and body message timestamps before comparing positions.
Translation uses bounded ideal forces at each body's COM with acceleration
feedback `0.04 * position_error + 0.4 * velocity_error` and circular-offset
gravity-gradient feedforward. Mass is read separately from each existing
MJCF subtree. Target attitude uses bounded ideal torque. Service attitude
uses the existing reaction wheels/controller with an orbit-frame reference;
its gains, wheel hardware and model inertias are unchanged.

No weld or post-initialization pose/velocity setter is used. This is truth-state
feedback and ideal actuation, not a flight-qualified thruster implementation.
The existing coarse target collision model, passive panel hinge, and capture-arm
collision masks remain unchanged. Holding is not evidence of a successful grasp.

Telemetry `formation_hold` reports measured tracking errors, actual requested
forces/torque, rotating-frame relative speed and target attitude error. A sample
is logged every ten simulation seconds. The frontend shows hold status and
errors independently of the selected arm or camera. Existing camera transforms
are unmodified.
