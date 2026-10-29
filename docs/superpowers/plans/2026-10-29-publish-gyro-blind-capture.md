# Publish gyro-assisted terminal capture

User authorization: push the locally validated gyro/terminal-capture work to main.

Base: origin/main ddea83ce6a30386d4eeb8a08b96650901747590d.
Scope: own-motion observer, predictive capture component wiring and fault guards,
portable pure/native tests, usage and experiment limitations. Preserve upstream
gyro, policy/flight interfaces, masses/inertias, models, optics and contact settings.

1. Port only capture source/test changes into a clean main-based release tree.
   Prove new tests fail before the missing consumer is copied, then pass.
2. Run relevant pure/native checks and repository CI checks; classify native
   dependencies separately. Review the complete release diff and fix blockers.
3. Push a feature branch, create and attach a PR, wait for required checks,
   merge using a merge commit, fetch and verify main contains the release.
4. Preserve the original dirty experiment workspace and local raw evidence.

Use a clean release worktree, separate from the dirty experiment checkout.
Raw local evidence and the execution ledger stay outside the repository.

Rulings: native worktree tool rejected the nested repository sourceRoot because
the chat workspace is its non-Git parent; used git worktree fallback. The original
linked experiment worktree is too dirty and behind main to merge wholesale.
No production sensor or teleop entry copy is needed: main already installs and
publishes service_gyro, and Capture26 lazily resolves it after graph construction.

Review focus: 50 Hz unique samples vs 400 Hz control, stale/future/epoch faults,
force-hold safety, legacy servo behavior, no truth fallback, compatibility with
main's current policy/flight APIs, no model/parameter changes.
