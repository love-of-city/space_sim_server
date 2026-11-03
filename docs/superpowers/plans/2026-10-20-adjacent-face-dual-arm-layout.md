# Adjacent-face dual-arm layout implementation plan

> For agentic workers: Use superpowers:executing-plans in this session. Execute continuously and retain the existing dirty workspace; isolate generated variant assets and preserve byte-identical backups.

**Goal:** Mount the existing capture arm on the side adjacent to the operating arm, with a collision-free folded state and common access to the target interface vicinity / service-facing surface after capture.
**Architecture:** Generate a reversible mesh/MJCF variant from the current canonical model. Retain arm geometry, controller identities, explicit inertias, 898 target collision pieces, compliant pad and force controller. Select the capture pose based on collision-aware two-arm IK before activating the variant. Update the coorbital initialization only as required to express the chosen relative orientation and position; no formation forces, weld or target teleport during simulation.
**Tech Stack:** Python, NumPy/SciPy, VTK mesh clipping, MuJoCo native/Basilisk, UE existing asset pipeline.
**Spec:** User-approved adjacent-face layout in this conversation, 2026-10-20; preferred work region: interface vicinity and target surface facing the service spacecraft. Earlier 0.5 m start distance is measured grip_frame to interface center.

## Global constraints
- Baseline current XML, manifest, catalog, scene and changed source files must be backed up before mutation.
- Existing top operating arm, tool/task board and protected camera poses remain unchanged.
- Target mass/inertia and all explicit body inertials remain unchanged; relocating the capture arm necessarily changes the assembled spatial mass distribution.
- No new weld/fixed equality between spacecraft; preserve force-controlled compliant-pad capture.
- Preserve genuine serial-arm geometry; generated concept art is not a geometry source.

## Review focus
- Folded whole-arm visual envelope includes wrist and new cylindrical gripper.
- Collision filters must catch arm-versus-own-bus and arm-versus-other-arm, without counting intentional connected-joint overlap.
- IK position reachability alone is insufficient: check tool orientation, joint limits and sampled approach paths.
- Target access excludes occupied capture contact zone; do not claim full satellite surface coverage.
- UE visible meshes, thermal blanket geometry and authoritative collisions must follow the same changed bus layout.

## Tasks
- [x] 1. Snapshot baseline, measure the existing folded envelope / target mesh and establish explicit success checks.
- [x] 2. Build a sibling side-face mesh variant, clear only mounting-face protrusions, extend bus height as needed, fit folded capture arm and preserve inertials/old camera parameters. Files: outputs/gripper_platform_integration/adjacent_face_v1/* and unique model assets under model/dual_arm_capture26/adjacent_face_v1/.
- [x] 3. Solve capture and operating-arm access poses and collision-free sampled paths. Require position residual <1 mm, orientation residual <1 degree, valid joint ranges, no unintended penetrations. Save a map of tested target-face access points and clearance evidence; fail before activation if the selected layout cannot meet these.
- [x] 4. Activate the verified variant with a rollback selector; update only necessary runtime layout/initialization fields and UE asset mappings. Start separated at 0.5 m along approach where feasible; otherwise record reachability limitation rather than hiding it.
- [ ] 5. Run native dynamics/contact checks for capture and subsequent operating-arm approach. Record whether capture maintained force/contact support, maximum target disturbance and any arm/body contacts. Show true UE views of folded layout and operation approach; distinguish static/path, dynamics and rendering acceptance.

## Execution ledger
- Ruling: baseline workspace contains extensive authorized uncommitted meshes and renderer changes, so new work uses versioned variant assets plus byte backups, avoiding a clean checkout that would omit the user's current model.
- Ruling: full 360-degree target coverage is outside this arm/visibility geometry; validate the explicitly selected interface vicinity and service-facing surface.

- User scope reduction: 2026-10-20, only coarse partial target reach is required now. Retain the already found access-point IK evidence; defer full operating-arm path planning and post-capture dynamics demonstrations. Finish adjacent-face model and UE initialization with baseline preservation.

- Review fix: external staging XML uses absolute asset paths for analysis, while installed/selected repository XML now uses relative mesh paths so another checkout root can load the same model. Active manifest hashes refreshed.

- Completed reduced-scope acceptance: explicit inertials/equalities/camera parameters preserved; installed model relative paths validated; 53 tests passed; real UE folded and ready snapshots inspected. Task5 full post-capture dynamics is deferred at user request; final restart uses unconnected 0.5m ready scene.
