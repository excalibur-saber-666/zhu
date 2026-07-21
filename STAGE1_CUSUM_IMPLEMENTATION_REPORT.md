# Stage-1 CUSUM implementation report

> **Superseded:** this report describes the pre-repair implementation and its
> numerical results are invalid after the critical fixes.  Use
> `STAGE1_CUSUM_REPAIR_REPORT.md` for the corrected implementation and results.

## Scope and original execution path

- Original entry point: `hybrid_cooperative_navigation.m`.
- IMU/SINS loop: 0.02 s.  The centralized graph/KF update is entered every
  1 s (`T_D = 1`).  CUSUM is evaluated exactly once at this graph epoch.
- Ranges are created by `distance_cal -> psedu_cal`, with the original
  `sigma_dis = 0.2 m` and 500 m communication gate.
- The centralized graph contains high-precision UAVs first.  Verified mapping:
  global `3,4,5 ->` internal `1,2,3`; global `1,2 ->` internal `4,5`.
- CUSUM innovations use the current graph GPS position priors, before range
  factors are added.  Covariances are the matching original independent
  position-prior covariances: low-precision UAVs `diag([10,10,20]^2)`,
  high-precision UAVs `diag([0.2,0.2,0.5]^2)`.

## Delivered files

- `stage1_cusum_default_config.m`: all Stage-1 configuration, including fault
  injection parameters that are kept outside the online weighting API.
- `compute_signed_cusum_edge_weights.m`: signed innovation, two-sided CUSUM,
  soft weights, warm-up, missing-edge decay/reset, and diagnostics.
- `run_stage1_cusum_comparison.m`: cached-input Equal-FGO/CUSUM-FGO runner,
  final-navigation RMSE and fault diagnostics.
- `test_stage1_cusum_components.m` and `test_stage1_cusum_smoke.m`.
- `factor_graph_centralization.m`: optional range-weight storage and one-time
  `sqrt(weight)` scaling during both factor insertion and relinearization.
- `STAGE1_CUSUM_TUNING.md`: public-parameter development record.

## Verification

- MATLAB static check completed with no syntax errors; remaining messages are
  code-analyzer style/performance warnings only.
- `test_stage1_cusum_components`: PASS.
- `test_stage1_cusum_smoke`: PASS.
- Smoke test verifies finite positive weights, retained active range edges and
  exactly repeatable Equal-FGO output from the same cached input.

## Full 600 s results

All values below are 3-D final SINS/KF navigation RMSE, not graph-node-only
error.  A positive percentage means CUSUM is worse than Equal.

### Healthy, three seeds

| Seed | UAV 1 full change | UAV 2 full change | UAV 1 matching-window change | UAV 2 matching-window change |
| --- | ---: | ---: | ---: | ---: |
| 11 | +1.88% | +2.03% | +1.70% | +2.63% |
| 23 | +2.50% | +3.23% | +2.52% | +3.21% |
| 47 | +1.45% | +1.73% | +2.00% | +1.30% |

The healthy ≤5% RMSE criterion is met for these runs.  However, the healthy
weights are not close enough to one for a clean acceptance: median weights are
0.913, 0.912 and 0.923; 61--65% of healthy-edge epochs are below 0.95.

### Fault development seed 23, target global edge (2,3)

| Bias / window | UAV 1 window Equal -> CUSUM | UAV 2 window Equal -> CUSUM | target / other median weight | detect `<0.95` / `<0.8` | recovery `>0.95` |
| --- | ---: | ---: | ---: | ---: | ---: |
| +1 m, 100--200 s | 5.0552 -> 5.0732 m (+0.36%) | 8.5726 -> 8.6536 m (+0.94%) | 0.945 / 0.911 | 5 s / 16 s | 3 s |
| +2 m, 100--150 s | 9.9908 -> 10.0138 m (+0.23%) | 14.3355 -> 14.4664 m (+0.91%) | 0.931 / 0.911 | 5 s / 16 s | 2 s |
| +5 m, 100--150 s | 24.6096 -> 24.6102 m (+0.00%) | 31.5287 -> 31.8503 m (+1.02%) | 0.738 / 0.911 | 4 s / 5 s | 4 s |

All three fault runs had zero non-finite fallbacks.  The communication graph
did not lose an active edge in this trajectory, so missing-edge decay/reset
counts are zero.

### +5 m multi-seed direction check

| Seed | UAV 1 window change | UAV 2 window change |
| --- | ---: | ---: |
| 11 | -0.22% | -1.31% |
| 23 | +0.00% | +1.02% |
| 47 | -0.37% | -0.32% |

The requested ≥10% 5 m improvement and consistent fault-window improvement
are **not met**.  The 1 m and 2 m fault-window RMSE also do not improve for
the development seed.  This is reported as a failed performance acceptance,
not masked by target-edge knowledge.

The likely cause is the original 10 m low-precision GPS prior covariance:
the standardized innovation of a 1--5 m bias is weak compared with normal
prior-to-range variation.  The safe next tuning surface is restricted to the
public CUSUM parameters `lambda`, `kappa`, `scale_h`, and `weight_min`; do not
introduce target-edge rules, truth data, fault labels, hard deletion, FI/KLD,
sliding windows, IMU preintegration, or bias states.

## Constraint confirmation

- No FI/KLD, sliding window, IMU preintegration, range bias state, hard edge
  deletion, or distributed optimization was added.
- Online weighting has no truth, fault label, fault amplitude, or fault-time
  input.  Fault configuration is used only for measurement injection and
  offline reporting.
- Weighting is applied once as `sqrt(weight)` to each range residual/Jacobian;
  the position-prior factors are unchanged.
- The Equal-FGO path passes unit weights and preserves the original graph
  construction and noise model.
