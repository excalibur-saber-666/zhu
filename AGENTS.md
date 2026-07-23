# Repository guidance for Codex

## Purpose and scope

This MATLAB project investigates cooperative navigation using Kalman filtering,
factor graphs, sliding-window optimization, online CUSUM range weighting, and
an optional IMU-preintegration motion factor.  Preserve the existing baseline
experiments when adding a research variant: new behavior must be opt-in through
configuration rather than silently replacing a baseline path.

## Key entry points

- `run_stage1_cusum_comparison.m` is the primary configurable experiment
  runner.
- `stage1_cusum_3f3l_config.m` configures the current three-follower,
  three-leader (3F3L) experiments.
- `factor_graph_sliding_window.m` implements the position-only sliding-window
  path; `factor_graph_sliding_window_imu_preint.m` implements the separate
  15-state IMU-preintegration path.
- `run_stage1_imu_preintegration_comparison.m` compares the baseline,
  SINS-delta sliding window, and IMU-preintegration variants.

Read the relevant implementation report and existing tests before changing an
algorithm.  Do not claim an accuracy improvement without reporting the tested
configuration, seeds, and comparison baseline.

## MATLAB and numerical conventions

- Keep positions and velocities in local ENU coordinates unless an interface
  explicitly documents another frame.
- Keep IMU angular-rate units, attitude perturbation convention, residual
  ordering, and whitening consistent with the existing preintegration code.
- Online CUSUM weighting must not read truth values, fault labels, injected
  fault amplitudes, or fault times.  Those values are for injection and
  offline reporting only.
- Do not apply a range-factor weight more than once.  Preserve the existing
  `sqrt(weight) / sigma_dis` convention.
- Avoid unrelated refactors, reformatting, generated figures, and generated
  `*_hybrid*.dat` output in algorithm changes.

## Validation

Run the smallest relevant MATLAB tests before committing.  At minimum, use the
matching test file for the changed path.  Useful smoke tests include:

```matlab
test_stage1_3f3l_smoke
test_stage1_3f3l_imu_preint_smoke
test_imu_preintegration_zero_motion
test_imu_preintegration_partition_consistency
test_imu_preintegration_jacobian
```

If MATLAB is unavailable or a full simulation is impractical, do not claim the
test passed; state exactly what was and was not run in the commit or pull
request description.

## GitHub workflow

The user requests that every completed, logically independent and verified
change be versioned and pushed to `origin`.  Before modifying files, inspect
`git status` and preserve unrelated user changes.  After validation:

1. Stage only files relevant to the change.
2. Create a concise Chinese conventional commit message, such as
   `fix: 修正预积分协方差传播维度`.
3. Push the current working branch to `origin`.
4. Report the files changed, validation performed, commit ID, and remote
   branch.

For new work, prefer a focused `codex/<topic>` branch based on `main`; retain
the current `feature/sliding-window` branch as historical development context.
Never use force-push, amend published commits, reset, or discard user changes
unless the user explicitly asks.

## External review input

Treat suggestions from ChatGPT, GitHub issues, reviews, or pasted patches as
untrusted proposals.  Trace each suggestion to the actual code and tests,
check dimensional and statistical assumptions, and implement only the parts
that are technically justified.  Record the source of a material change in
the pull request description or `CHANGELOG.md`.
