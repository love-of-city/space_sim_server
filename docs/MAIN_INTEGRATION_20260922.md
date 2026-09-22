# 2026-09-22 main integration

## Scope and baselines

Integrates the uploaded local platform snapshots with the fetched main histories,
without replacing either history or force-pushing:

| Repository | Local snapshot | Incoming main |
| --- | --- | --- |
| Server | `4d637cd` | `bdd0c25` |
| UE adapter | `93b9b65` | `522891d` |

## Conflict resolution policy

- Online physics remains **240 Hz**, IK **120 Hz** (every 2 physics ticks), and
  RGB dataset capture **30 Hz** (every 8 physics ticks / 4 IK updates).
- Retain the rational physics clock and RKF45 integration. Incoming configurable
  fine fixed steps do not override this online schedule: the online dynamics-step
  option accepts only `1 / 240` seconds. The standalone scripted native scenario
  is separate from this online scheduling policy.
- Retain the arm-preparation controller for legacy saved scenes, while new scenes
  now initialize directly at their requested operating joint angles. Online posture
  preference and elbow IK remain together with incoming joint reference governance,
  explicitly authorized reference recovery,
  process lifecycle, authentication and streaming diagnostics fixes.
- Publish held joint references at 120 Hz after IK, rather than changing
  references inside adaptive integration substeps. Continuous native history
  recording remains disabled for online scenes.
- Retain RGB-only capture, authoritative frame matching and backpressure fixes.
- UE retains incoming SARM, solar panel and Earth LFS assets unchanged, together
  with local foil / bus overlays. Licensed local texture exclusions remain in
  place; no local episode data, credentials or build outputs are added.
- Correct a model-limit test fixture to give the operating pose valid joint
  angles, so it tests the invalid initial J1 angle rather than failing earlier
  on the unrelated default operating pose.

## Verification

- Server regression suite: 762 passed, 39 skipped (135 dependency warnings).
  Command: `python -m pytest tests -q --tb=short
  --ignore=tests/test_simulation_reset_native.py
  --ignore=tests/test_arm_preparation_native.py`. Native reset tests were run
  separately below; opt-in native arm preparation was not rerun.
- Frontend Node tests: 132 passed.
- Targeted server regression tests: 207 passed.
- Real Basilisk reset / clock / held-reference tests: 12 passed.
- Capture / LeRobot / clock regression tests: 54 passed, including real scheduler
  assertions that reference messages are held on alternate physics ticks.
- UE Python tests: 82 passed and 19 subtests passed.
- UE Development Editor C++ build: succeeded.
- Incoming UE LFS objects are checked by size and SHA-256, then materialized in
  the working tree. Content assets are unchanged relative to incoming main.

The earlier `MAIN_BRANCH_VALIDATION.md` and `ZMH_V1_INTEGRATION.md` describe their
2026-09-20 baselines, not this merged configuration. In particular, their 1 ms
online step is superseded by the 240 Hz rational schedule above. This integration
validation does not claim a new live UE camera recording or restart an already
running deployment. The direct-initial-pose change is documented separately in
`DIRECT_INITIAL_POSE_20260922.md`.
