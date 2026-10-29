# Portable visual and gyro terminal capture regression

> **For agentic workers:** Use superpowers:executing-plans inline, with one final fresh review.

**Goal:** Deliver a portable successful-scene recipe, vision runtime template and one entry point; qualify the complete UE-image/gyro/force grasp on current main before publishing.

**Architecture:** Reuse current main backend, simulation entry, native helpers, UE renderer and PnP worker. A regression runner resolves machine arguments and output paths, builds missing runtime components and machine-local catalogs, starts isolated processes, captures evidence and restores temporary runtime configuration. The controller/physics/model remain unchanged. A separate no-contact run supplies target-motion scoring only, never guidance.

**Spec:** User request 2026-10-29 in this chat. Start from ac25b705f5aa070ad81830ff551dcb13d8131470; preserve masses/inertias; add no fixed inter-spacecraft constraints; verify then merge main.

## Global constraints

- Use the retained nominal seed26016 initial scene: adjacent-face dual arms, COM-coorbital initialization, 0.5m grip/interface gap, attitude control enabled, formation_hold disabled.
- 1920×1200,100°×100°,10Hz native single-camera PNGs and original 50Hz shared gyro; no truth navigation or injected fault.
- Preserve all model/physics/gyro/controller sources and physical parameters. No weld/equality between satellites.
- Default manual startup stays manual. Experiment opt-in applies only to this entry.
- Resolve repository paths from the entry itself; runtime/vision interpreter and UE installation are explicit machine parameters. Do not depend on the old experimental checkout or output folder.
- Preserve previous run/vision_runtime.json and observation bytes; refuse another scene/backend in the same checkout or occupied ports; terminate only owned processes; retain failure evidence.
- Nominal acceptance: real four-marker image, visual then encoder/gyro approach, close/tighten/hold,>=20s continuous four-support hold at4.5..5.5N, no service self-contact, unchanged physical fingerprint. Report target body-origin translation/rotation against matched free-flight reference.
- Regression limits selected for this nominal scenario: target disturbance<=10mm/2deg with precontact reference match<=0.05mm/0.005deg; these are engineering regression gates, not hardware accuracy or reliability claims.

## Review focus

- A clean checkout builds native binaries and derives catalogs from its own tracked assets without copying another machine's Saved files.
- Missing images, false/old sessions, gyro faults, early exits, timeout or missing evidence cannot yield success.
- Arbitrary cwd, spaces in paths, pre-existing outputs/config, occupied ports and failure cleanup preserve unrelated user work.
- The free reference records target truth only in a separate process; no truth/reference enters actual grasp control.
- Source hashes and process PYTHONPATH prove the backend, worker, adapter and native libraries belong to this checkout.

### Task1: Portable configuration and scoring

- [ ] Commit sanitized nominal scene and vision templates under config/experiments/visual_gyro_capture.
- [ ] Add failing tests for path resolution, safety/model invariants and evidence scorer failure categories.
- [ ] Implement recipe preparation and streaming scoring; tests green.

### Task2: Unified runner and independent reference

- [ ] Add failing lifecycle/ownership/config restoration tests.
- [ ] Implement native preparation, tracked-asset catalog preparation, isolated backend/UE/simulation, runtime export, monitoring and exact cleanup.
- [ ] Add independent free-flight recorder with neutral real backend and discarded render transport only; no control/physics patches.
- [ ] Update CI deployment filter and docs; execute meaningful unit/native checks.

### Task3: End-to-end qualification and publication

- [ ] Run one full native UE grasp from this main-based tree and independent free-flight reference.
- [ ] Recompute real-image PnP and score gyro/phase/force/contact/mass/constraint/target disturbance evidence; retain failed attempts.
- [ ] Run basic CI/web/native/UE checks appropriate to actual changes; one fresh whole-branch review and verified fixes.
- [ ] Push one PR, wait required checks, merge and verify remote source identity and main CI.
