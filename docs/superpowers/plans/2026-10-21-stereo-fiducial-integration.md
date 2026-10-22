# Stereo fiducial integration implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development or superpowers:executing-plans to implement the independently verifiable tasks below.

**Goal:** Attach four flush symmetric target markers and feed synchronized body-mounted stereo images to an image-only pose baseline.

**Architecture:** Keep physics and render attachments separate. Two new explicit body camera descriptors feed a 10 Hz authoritative vision channel independent of LeRobot recording. A bounded receiver pairs source-frame-matched PNGs; an isolated OpenCV worker receives only pixels, static calibration and known marker corners.

**Tech Stack:** Python, OpenCV ArUco/PnP, SciPy, UE 5.6, existing TCP capture protocol, FastAPI.

**Spec:** User request in the current task on 2026-10-21; the hardware list is nominal, not a measured calibration.

## Global constraints

- Preserve all existing masses, inertias, contacts, capture initialization and protected camera poses.
- Do not add a weld or fake capture episode IDs.
- Keep source handoff assets and original marker arrangement available for comparison.
- Camera resolution 1920x1200, nominal focal length 6 mm / 3.45 um = 1739.1304348 px; parallel baseline 0.12 m; trigger 10 Hz; requested exposure 0.002 s.
- Hardware sync tolerance <=100000 ns is a pairing requirement; simulated identical timestamps do not validate physical hardware synchronization.
- Algorithm input excludes simulator target pose, camera truth extrinsics, depth buffers and contact state.

## Review focus

- White marker footprint must be supported by actual flat unobstructed structure, not bridged across a hole.
- Image axes, marker corner order, optical camera axes and renderer axes must agree.
- Never pair different session/frame/time images; capture sequence is per image, not a stereo trigger.
- Dataset capture disabled must not disable vision capture or silently alter existing preview cameras.
- Reject insufficient/ambiguous detections; never fill missing estimates from truth.

### Task 1: Provenance and surface placement

Files: output-only model/surface audits; new versioned config/visual_attachments target layout, preserve original.

- [x] Fetch origin/main and compare target meshes/file hashes/body/inertia separately from test contacts.
- [x] Audit complete projected visual geometry for flat support and foreground occlusion.
- [x] Generate symmetric markers and 3D decoded corner layout using verified surface locations.
- [x] Verify all physical model XML bytes remain unchanged by marker changes.

### Task 2: Independent synchronous vision capture

Files: Adapter descriptors/bridge/transport; UE parser/protocol/controller; server render component and launch script.

- [x] Add optional vision consumer and integer-clock trigger; default off preserves current behavior.
- [x] Capture both cameras on the same authoritative frame, preserve reliable packets/ACKs.
- [x] Register new cameras explicitly; keep all existing camera values and preview settings unchanged.
- [x] Run scoped protocol, trigger and camera registration tests; build UE runtime.

### Task 3: Image-only pose baseline and service

Files: backend stereo_pose.py, vision_service.py, vision_worker.py, capture_receiver.py and app.py; focused tests.

- [x] Test detection/pose with independent projected marker images, noise, outliers and bad stereo.
- [x] Pair by session/source frame/simulation timestamp, reject time skew and stale sessions.
- [x] Bound worker queue and ACK before algorithm work; do not send vision packets to dataset recorder.
- [x] Expose authenticated status and latest left/right images without changing normal camera views.

### Task 4: Live verification

- [x] Restart via official launch flow, verify 1920x1200 PNG pairs and 10 Hz simulation trigger.
- [x] Inspect actual UE images and marker IDs, compare estimated pose only with independent truth evaluation.
- [x] Save configuration, sample input images, results and limitations; report what is verified versus nominal hardware configuration.

## Execution outcome

User kept50mm black codes and authorized thin film across unused holes. Source supports and raised-rib clearance were audited separately; the full-support20mm candidate is retained but not active. Runtime and image-only estimation are verified. Real wall-clock10Hz was NOT achieved; current preview is paced at0.2x, with timing/drop metrics exposed. Hardware calibration, finite exposure integration and capture/control acceptance remain outside this implementation.
