# Fixed trajectory repeatability implementation plan

**Goal:** Evaluate drawer_cycle and the three plug tasks at fixed initial conditions, repeating identical time-indexed actuator references implementing a frozen spacecraft-relative TCP trajectory. No success-targeted resampling, online object tracking or corrective replanning.

**Architecture:** Existing scripted controllers generate one nominal trace per task. A separate exporter freezes reference TCP poses, eight actuator coordinates, model/dependency hashes and initial state. Playback uses the compiled reference table with existing servos and load protection. Independent ordered physical predicates score all fixed-count trials; capture/visual policy evaluation is outside this repeatability baseline.

**Tech stack:** Python, NumPy, existing Basilisk/MuJoCo production runtime, pytest.

## Constraints and rulings
- Work only on local feature/benchmark-asset-library; no push or merge.
- Existing source assets, model geometry, physics and adaptive collectors are preserved.
- No randomization in this protocol; a new native process per trial.
- Exactly N attempts per task, no rejected-trial replacement. Report success/N and success/(success+task_failure), plus initialization, infrastructure and unscorable counts.
- The canonical trajectory is generated once, then frozen even when its donor fails. Do not silently search for a successful donor. Report donor result and independent score.
- Fixed TCP path is compiled to frozen joint references using the same model FK; playback does not call the adaptive task's step() or online IK. Only the servo and load-stop are feedback.
- Predicates use physical measurements, never scripted stage/result/open_verified flags. Distance-only contact counts are proximity evidence, not measured grasp load.
- Candidate thresholds reuse current simulator engineering tolerances, not claimed LIBERO/RoboDojo tolerances. Freeze thresholds in artifacts.
- Preserve per-trial traces, exception details, hashes and results. A process failure must never become a successful or silently excluded trial.

## Tasks
- [x] Add tests first for immutable trajectory validation/playback, flag-independent ordered scoring, missing/gapped evidence, safety stop, denominator accounting.
- [x] Implement trajectory exporter, scorer and replay controller; add opt-in production runtime hook and audit rows.
- [x] Add fixed-count CLI with canonical generation, offline re-scoring and trial/suite summaries; make source/physics identity explicit.
- [x] Verify unit/regression checks and real native smoke tests; attempt four-task baseline runs. Distinguish code verification from full task success and report any environment blocker.
- [x] Obtain fresh independent review, fix important findings, document commands/results and leave local changes for user inspection.

## Review focus
Forged script success cannot pass; missing evidence cannot yield a task score; final stability uses simulation time; model/config changes reject playback; seeds never masquerade as diversity in deterministic repetition.

## Verified outcome
102 targeted controller/evaluation/collection regression tests passed. Four tasks each completed three fixed-count native trials: drawer 3/3; aviation, guide and cylindrical insert 0/3 due wrist overload. Final strict scorer offline audit matched all twelve original online reports; an additional final-code drawer native verification passed online/offline agreement. All evidence remains outside Git in D:/mingti/outputs/repeatability-20261029. Basic CI collection blocked by missing scipy; UE/visual-policy evaluation remains outside scope. Independent review findings (bootstrap wrench, resolved physics identity, malformed documents, evidence hashes, sample gaps and frozen velocities) addressed.
