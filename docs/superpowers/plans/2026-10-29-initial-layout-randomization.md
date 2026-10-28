# Initial task-box layout randomization

Scope approved: randomize task objects and mating-interface installation XY/yaw; retain initial mechanism states and seated pickup pairs. Reuse existing 13-unit generator, manifests, catalog and read-only collection pool. Do not change arm initialization, physics parameters or the 40 N/4 Nm limits.

1. Audit API/CLI defaults and sampling/validation chain; distinguish continuous layout generation from finite pool selection.
2. Reproduce Windows geometry-probe/UE-command path failures with tests, then reuse portable runtime discovery.
3. Generate two actual layouts with bounded yaw, preserving original mechanisms and mating geometry. Validate with matching MuJoCo 3.7.0 and exported geometry checks.
4. Import generated visuals into the local isolated adapter project under fresh namespaces, validate acknowledgements, publish a content-pinned pool. No push.
5. Verify deterministic selection, pairing and state preservation; document exact paths, seeds, ranges, rejection/acceptance and remaining task/visual acceptance boundaries.

Generated assets and reports stay outside Git. Layout geometry acceptance does not prove all robot tasks are reachable or successful. Keep per-task preflight rejection records rather than silently substituting fixed layouts.

## Scope update after upstream comparison

The user authorized aligning with newer zmh_linux layout code, then confirmed the intended future scene is a compatible robot/tool with targets mounted on satellite exterior surfaces inside its usable task workspace. Selectively align operation-clearance contracts/sampling, layout-pool validation, builder/checker integration and tests while preserving main, existing Windows ports and unrelated local task fixes. Preserve the stopped old-layout prototype without publishing a pool. Generic asset-driven robot/satellite selection remains a design proposal in docs/ASSET_SCENE_GENERATION_DESIGN.md; do not claim its implementation or full robot-task acceptance.

Selective alignment and two review-driven axis corrections are implemented; 55 focused regressions pass. Actual compiled-source gripper bounds saved outside Git. Both real-model planning probes exhausted their sampling budgets (seeds2026102900/yaw30 and2026102901/yaw0). New layout pool generation/import remains incomplete; this validation update does not assert a usable randomized task scene. Next design stage must support asset-dependent capacity and target subsets without weakening clearance.
