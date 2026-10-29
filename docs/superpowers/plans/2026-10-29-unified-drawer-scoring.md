# Unified drawer physical success criteria

User decision: use the existing, more complete DrawerCycleSuccess logic as the drawer evaluation authority. Local-only change; preserve previous artifacts and do not alter the in-progress controller/planner work in this checkout.

- After the user's clarification, share physical task requirements, not expert choreography. Use a versioned DrawerBenchmarkCriteria independent of expert configuration. Require grip evidence, opening at least75 mm continuously2 s, held return, release, and at least90 mm retreat from the actual handle in any direction with closed/latched/no-contact state continuously2 s.
- Preserve existing drawer80 N/8 Nm/0.1 s overload checks and200 Hz complete-evidence checks in the evaluation wrapper. Plug geometry and40 N/4 Nm/0.15 s scoring remain unchanged.
- Adapt the shared monitor to explicit50 Hz or200 Hz timestamps. Freeze geometry, thresholds and sampling period in the scorer specification; no dependency on scripted result/stage flags or mutable controller targets.
- Expert grasp orientation, exact hover waypoint and arm stillness are diagnostic only in benchmark mode. Orientation/joint-velocity diagnostics may be absent; required physical evidence may not. Retain legacy calibrated mode explicitly. Reject old scorer specs rather than silently reinterpret them; old summaries and traces remain immutable.
- Keep the existing policy seating allowance for a pressed latch; final acceptance requires latch return. Preserve the evaluation's no-contact release/final requirement in the shared core.
- Test physical temporal requirements, independence from expert configuration/pose and50/200 Hz agreement, then integrate replay evidence and offline re-scoring. Re-score retained measurements with frozen model calibration; report revised outcomes without rewriting v1 results.
- Verify relevant regression tests, independent review and a native replay if the working runtime permits it. Do not tune the control trajectory or weaken the new criteria to reproduce the old success rate.

The fixed-reference runner is a regression baseline. A policy benchmark must run observation -> policy action -> environment step -> next observation, with an independent judge and fixed-count outcomes. This change does not claim a validated pi0.5 adapter, four-task policy suite or new native execution; concurrent controller/runtime changes remain out of scope.
