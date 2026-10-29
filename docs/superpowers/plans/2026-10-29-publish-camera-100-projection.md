# Publish independent 100 by 100 degree capture-wrist optics

> **For agentic workers:** Use superpowers:executing-plans inline, with one final fresh review.

**Goal:** Publish the existing local capture-wrist calibration, matched PnP intrinsics and UE projection onto current main without regressing newer functionality.

**Architecture:** Selectively port the JSON calibration through startup, adapter manifest and UE capture projection/metadata. Keep all existing main camera modes, relative views, rendering cadence, policy/flight and gyro functionality. Existing cameras with no vertical FOV retain their projection.

**Tech Stack:** Python, pytest, UE 5.6 C++, PowerShell, GitHub required checks.

**Spec:** User request in this chat on 2026-10-29: merge local 100°×100° configuration, intrinsics synchronization and UE projection; preserve main features and mass/inertia; verify and publish to main.

## Global Constraints

- Capture-wrist camera is single-camera, 1920×1200, 100° horizontal and vertical, 10 Hz; fx=805.5356459301888, fy=503.459778706368, cx=959.5, cy=599.5, zero distortion.
- Preserve camera mount and other camera views. Optics are ideal virtual anisotropic pinhole optics, not validation of a real 6 mm lens or finite exposure blur.
- Do not change model, mass, inertia, collision, dynamics, controls, scene layout, or the dirty experimental worktree.
- Keep main rgb/fisheye/monochrome, relative views and policy/flight code. New independent pinhole projection does not silently modify fisheye semantics.
- Server and adapter changes ship in one PR; protected main merges through required checks.

## Review Focus

- Legacy cameras omit or set zero vertical FOV and keep square-pixel projection.
- RGB and monochrome preserve new independent projection; fisheye retains its existing equidistant model.
- Bad/nonfinite calibration or projection inputs fail clearly instead of silently mismatching PnP and rendering.
- Preview and native vision use the same optics/mount while retaining independent routing/rate.
- Capture metadata matches rendered pixels, including a non-square raster, without changing physical assets.

### Task 1: Calibration and startup synchronization

**Files:** config/vision/capture_wrist_camera.json, backend/space_arm_platform/wrist_vision_config.py, simulation/stereo_cameras.py, simulation/components/render.py, tests/test_wrist_camera_calibration.py.

- [ ] Port local regression tests first and observe failure on main.
- [ ] Load/validate calibration and apply identical optics to capture preview and native vision at startup.
- [ ] Test invalid optics, unchanged mount/other cameras and existing camera modes.

### Task 2: Adapter and UE projection

**Files:** adapter CameraVisual; BskProtocolTypes.h, BskFrameParser.cpp, BskCameraProjection.h, BskSceneController.cpp; adapter and UE tests.

- [ ] Add independent FOV and compatibility regression tests first; observe failure.
- [ ] Add optional vertical FOV to descriptor/parser and projection at both capture paths, retaining main code.
- [ ] Match metadata fy/FOV to the projection. Test legacy defaults, invalid FOV and camera-mode compatibility.
- [ ] Build Editor and execute UE automation; verify native GPU image/metadata against PnP calibration if available.

### Task 3: Verify and release

- [ ] Run declared basic Python suites, relevant native/image tests, web checks, lint, LFS and diff checks.
- [ ] Compare physical model/config hashes and original worktree hashes before/after.
- [ ] One fresh whole-branch review; fix important findings with regression tests.
- [ ] Document evidence/limits, commit, push, create/attach PR, wait checks, merge and verify remote main.
