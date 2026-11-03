# Dual arm controls and companion viewport implementation plan

> Execute inline with executing-plans; no delegated agents.

Goal: matched arm visuals, initially stationary relative target, independently selectable arm teleoperation, and a main viewport companion that preserves the main viewing angle.
Spec: ../specs/2026-10-18-dual-arm-control-and-views-design.md

Constraints: retain mass/inertia/attitude parameters, upstream overview mount and four-L gripper. No weld or hidden target state overrides. Preserve all pre-existing local changes.

- [x] Model: reuse six original link visual mesh assets and materials, preserve wrist cut required by gripper; verify geometric frame correspondence. Refresh manifest only after preservation checks.
- [x] Initial state: explicitly initialize both spins to zero in the current zero-relative scene, retain shared inertial velocity, log relative-state evidence.
- [x] Control: add active_arm and capture tool rates to validated action path; gate original controller and add bounded capture-arm IK with current-pose holding. Publish separate capture-arm telemetry. Test routing, stale inputs, limits, release, and switching.
- [x] UI: select controlled arm independently from selected video; render selected-arm feedback; add explicit radial/axial tool controls.
- [x] Companion: add a read-only PixelStreaming main-view player only for capture-camera view. Never send initialization/reset orbit commands from it. Keep main-angle state across primary-player switches; let the main view own its offset.
- [x] Verification: targeted backend and frontend tests, model checks, native restart, exercise capture-arm motion and unchanged idle arm, inspect camera and companion videos. Record actual limitations.

Execution evidence 2026-10-18:
- Original and imported six link triangle buffers are byte-identical; original material asset paths now reused. All inertials, joints, free joints and cameras compared unchanged.
- Native first measured state at t=0 reports relative linear and angular velocity exactly zero.
- Frontend: 188 passed; backend targeted final regression: 43 passed; prior orbital/native suite: 53 passed including import checks. Vite build passed.
- Native live UI: 0.5 s D input moved capture J1 to 0.3 degrees, R opened jaws from -12.0 to about -10.8 mm; release held. Operating arm remained [90,-60,60,0,-90,0] degrees.
- Review findings fixed: latch all 12 references on release, correct radial button signs, tear down companion on full player disposal. Reviewer recheck clear.
- Ruling: companion read-only means no robot input; allow deliberate middle-button main-view orbit per approved spec, but never send orbit on connection.
- Final scene restart loads arm-only tool-hold correction; final live screenshot pending. No full automatic capture or comprehensive collision validation claimed.

Final live acceptance: latest native scene running; 0.4 s D moved J1 to 0.3 degrees, R moved jaws to -11.1 mm, 0.5 s axial button moved carriage from 20.0 to 19.0 mm. Escape stopped and held. Original arm still [90,-60,60,0,-90,0] degrees. Both videos unpaused (1024x1024 and 1280x720), time advancing. Screenshot: outputs/gripper_platform_integration/dual_arm_control_live.png. Companion-angle preservation is covered by shared-cache/routing tests and source review; automated physical middle-drag not exercised. No full collision/capture certification.
