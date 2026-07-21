# Stage-1 CUSUM tuning record

> **Historical only:** this record predates the critical predictor, gyro, and
> Gauss--Newton fixes. Do not use its numerical conclusions; see
> `STAGE1_CUSUM_REPAIR_REPORT.md`.

Development seed: `23`.  All trials used the same cached IMU, GPS, range
noise, trajectory, and offline fault injection.  The online CUSUM API never
received a fault edge, fault flag, fault time, fault magnitude, or truth data.

| Trial | Change from baseline | Healthy result | 5 m / 100--150 s result | Decision |
| --- | --- | --- | --- | --- |
| Baseline | GPS graph priors, `lambda=0.90`, `kappa=0.25`, `h=4.0`, `w_min=0.05` | UAV 1/2 full RMSE change: +2.50% / +3.23% | window change: +0.00% / +1.02%; target median weight 0.738 | Retained: healthy requirement is met. |
| A | `h: 4.0 -> 1.0` | +26.62% / +29.11% | -1.85% / +2.68%; target median weight 0.150 | Rejected: excessive healthy false down-weighting. |
| B | Use KF/SINS state and 18-state KF covariance as the CUSUM predictor | +48.82% / +73.83%; healthy signed innovation mean -0.492 | -8.42% / -7.27% in the two fault-window RMSEs | Rejected: the healthy predictor has a persistent non-zero innovation mean and violates the health requirement. |

The final implementation therefore restores the original factor-graph GPS
priors as the pre-range position/covariance source and keeps the supplied
Stage-1 default CUSUM parameters.  No parameter was selected from an online
fault label or from a method-specific random realization.
