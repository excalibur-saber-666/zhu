function cfg = stage1_cusum_default_config(profile)
%STAGE1_CUSUM_DEFAULT_CONFIG Centralized single-epoch CUSUM experiment settings.
%   The values in this file are the only Stage-1 CUSUM tuning parameters.

if nargin < 1 || isempty(profile)
    profile = 'full';
end
cfg.mode = lower(profile);

cfg.dt = 0.02;
cfg.graph_interval = 1.0;
cfg.t_stop = 600;
cfg.uav_num = 5;
cfg.high_num = 3;
cfg.communication_range = 500;
cfg.sigma_dis = 0.2;
% The first three leaders are initialized from these columns of the supplied
% position data.  Keeping this mapping explicit lets companion experiments
% change the follower/leader split without editing the original data files.
cfg.base_leader_source_indices = [3, 4, 5];
% Optional high-precision leaders beyond the three leaders contained in the
% original position data.  Each column is an initial local [E; N; U] point
% in metres and follows the same prescribed manoeuvre thereafter.
cfg.additional_leader_positions_xyz = zeros(3, 0);

cfg.seeds = 1;
cfg.cusum_apply = true;
% Keep the paired healthy replay by default for regression testing.  A
% publication-style fault-only run can disable it because each fault
% trajectory already contains nominal intervals before and after faults.
cfg.include_healthy_scenario = true;

% Graph controls.  Ordinary FGO is single-epoch; CUSUM-FGO uses the
% established position-only SINS-delta sliding window.
cfg.graph_mode = 'single_epoch';
cfg.sliding_window_length = 10;
cfg.sliding_window_motion_std = [2; 2; 4];
cfg.sliding_window_motion_enable = true;

% Optional research variant.  The default four-method comparison keeps this
% false and uses the established position-only SINS-delta window.  Set it to
% true to replace only the CUSUM-FGO motion model with 15-state IMU
% preintegration; the other three methods remain unchanged.
cfg.imu_preintegration_enable = false;
cfg.imu_preint_window_length = 3;
cfg.imu_preint_prior_mode = 'first_frame_full';
cfg.imu_preint_feedback_mode = 'state_carryover';
cfg.imu_preint_covariance_mode = 'current_frame_only';
cfg.imu_preint_repropagate_enable = true;
cfg.imu_preint_gyro_bias_repropagate_threshold = 5e-5;
cfg.imu_preint_acc_bias_repropagate_threshold = 5e-3;
cfg.imu_preint_gyro_noise_std = 10 * pi / (3600 * 180) / sqrt(cfg.dt);
cfg.imu_preint_acc_noise_std = 0;
cfg.imu_preint_gyro_bias_rw_std = sqrt(2 / 3600) * 10 * pi / (3600 * 180);
cfg.imu_preint_acc_bias_rw_std = sqrt(2 / 1800) * 1e-3 * 9.7803698;
cfg.imu_preint_covariance_regularization = 1e-12;
cfg.imu_preint_gn_max_iterations = 30;
cfg.imu_preint_gn_step_tolerance = 1e-5;
cfg.imu_preint_position_eps = 1e-5;
cfg.imu_preint_velocity_eps = 1e-5;
cfg.imu_preint_rotation_eps = 1e-7;
cfg.imu_preint_gyro_bias_eps = 1e-8;
cfg.imu_preint_acc_bias_eps = 1e-6;
cfg.imu_preint_position_prior_std = [10; 10; 20];
cfg.imu_preint_velocity_prior_std = [5; 5; 8];
cfg.imu_preint_rotation_prior_std = [5; 5; 10] * pi / 180;
cfg.imu_preint_gyro_bias_prior_std = ...
    sqrt(2) * 10 * pi / (3600 * 180) * ones(3, 1);
cfg.imu_preint_acc_bias_prior_std = 1e-3 * 9.7803698 * ones(3, 1);

% Optional singular-value diagnostics for the final linearized graph.
cfg.graph_condition_diagnostics = false;
% Once the online CUSUM alarm is active, do not admit that range factor into
% new sliding-window frames.  Existing factors age out normally with the
% window; this avoids using an offline fault label or retrospective edits.
cfg.sliding_window_exclude_alarmed_edges = true;
% 'all' stops every alarmed edge.  'per_follower_max' stops only the largest
% CUSUM leader edge for each follower at a key frame, preserving the other
% leader ranges when one bad edge is assumed.
cfg.sliding_window_alarm_exclusion_mode = 'all';
% In the five-UAV sliding-window comparison, each follower has only two
% alternate leader edges.  Their two-value median can include the faulty
% edge and mislocalize a single fault, so this comparison uses the calibrated
% per-edge CUSUM statistic by default.  The original Stage-1 path keeps its
% cfg.cusum_consensus_enable setting above.
cfg.sliding_window_cusum_consensus_enable = false;
% CUSUM-FGO detector input.  'prior_innovation' preserves the established
% graph-prior residual detector.  'range_predictor' is an opt-in robust
% variant that reuses the independent online alpha-beta range predictor,
% while retaining FGO soft weighting and confirmed-alarm edge isolation.
cfg.fgo_cusum_detector_mode = 'prior_innovation';
% When the FGO reuses the independent range predictor, this mode determines
% whether all currently suspicious leader ranges are softened or only the
% largest-CUSUM range for each follower is softened.  Confirmed isolation is
% controlled separately by sliding_window_alarm_exclusion_mode.
cfg.fgo_cusum_soft_weight_mode = 'per_follower_max';
% Optional FGO-only recovery for a range-predictor CUSUM alarm.  When a
% confirmed alarm is followed by a persistent, strong innovation in the
% opposite direction, the previous step fault has likely cleared or changed
% sign.  The detector may reinitialize its online predictor instead of
% retaining a stale alarm and withholding a healthy geometry constraint.
% Disabled by default so established profiles remain reproducible.
cfg.fgo_cusum_opposite_step_reset_enable = false;
cfg.fgo_cusum_opposite_step_reset_threshold = 2.5;
cfg.fgo_cusum_opposite_step_reset_epochs = 2;
% Methods selected by main_stage1; any non-empty subset is also supported.
cfg.methods = {'ekf', 'fgo', 'cusum_ekf', 'cusum_fgo'};
cfg.plot_follower_indices = [];
% Healthy-protection tuning: innovation persistence is discounted more
% quickly and a larger deadzone leaves weak, shared predictor mismatch at
% full range weight.  These values were checked on independent seeds; they
% are still used only by the online innovation statistic.
cfg.cusum_lambda = 0.80;
cfg.cusum_kappa = 0.35;
% Continuous soft-weight mapping.  CUSUM values inside the deadzone keep
% their range factors at exactly full weight; values above it are softened.
cfg.cusum_scale_h = 8.0;
cfg.cusum_weight_deadzone = 2.00;
cfg.cusum_alarm_weight_gain = 1.00;
cfg.cusum_weight_min = 0.05;
cfg.cusum_weight_max = 1.00;
cfg.cusum_warmup_time = 10;
cfg.cusum_missing_decay = 0.85;
cfg.cusum_reset_after = 20;
cfg.cusum_eps = 1e-12;

% Detector confirmation, hysteresis, and fault-clear release.  These fields
% are independent from the continuous weight and are never driven by offline
% fault metadata.
cfg.cusum_alarm_on_threshold = 6.00;
cfg.cusum_alarm_off_threshold = 0.75;
cfg.cusum_alarm_confirm_epochs = 2;
cfg.cusum_alarm_release_epochs = 2;
cfg.cusum_release_lambda = 0.45;
cfg.cusum_release_innovation_gate = 0.75;

% Compare an edge only against simultaneous, trusted edges that share the
% same follower.  This removes common SINS/KF predictor mismatch without
% using truth, fault metadata, or an oracle label.
cfg.cusum_consensus_enable = true;
cfg.cusum_consensus_min_neighbors = 2;
cfg.cusum_consensus_leader_only = false;

% EKF-specific CUSUM protection uses the same complete two-stage policy as
% the proposed method: warning evidence softly downweights at most one
% leader edge per follower, then a confirmed alarm selectively isolates that
% edge.  Follower-to-follower ranges stay available at full weight.

% The local EKF maintains one 18-state covariance per follower rather than
% a joint covariance for all followers.  Retain a conservative position-
% uncertainty floor after each cooperative batch so unmodelled inter-follower
% correlation cannot make the local filter overconfident and suppress its
% subsequent GPS correction.  Both EKF variants use the same floor; optional
% leader-only fusion remains available for a geometry ablation.
cfg.ekf_range_leader_only = false;
cfg.ekf_range_position_std_floor = [2; 2; 3];

% EKF-CUSUM is intentionally independent from the FGO detector.  It monitors
% the signed range innovation relative to a per-edge online baseline, uses a
% two-sided discounted CUSUM for persistence, and enlarges R only after a
% confirmed alarm.  No field below refers to truth or fault metadata.
cfg.ekf_cusum_calibration_samples = 15;
% Keep the post-calibration reference fixed.  A continuously adapting mean
% would absorb the small, persistent range bias that CUSUM is meant to flag.
cfg.ekf_cusum_baseline_adapt_gain = 0.00;
cfg.ekf_cusum_baseline_variance_floor = cfg.sigma_dis^2;
cfg.ekf_cusum_lambda = 0.90;
cfg.ekf_cusum_kappa = 0.50;
cfg.ekf_cusum_alarm_on_threshold = 5.0;
% Keep hysteresis below the predictor-freeze boundary, but allow three
% consecutive near-nominal innovations to release an alarm before frozen
% constant-velocity prediction error accumulates into the opposite CUSUM.
cfg.ekf_cusum_alarm_off_threshold = 1.2;
cfg.ekf_cusum_alarm_confirm_epochs = 2;
cfg.ekf_cusum_alarm_release_epochs = 3;
cfg.ekf_cusum_innovation_gate = 4.0;
cfg.ekf_cusum_release_innovation_gate = 1.0;
cfg.ekf_cusum_weight_deadzone = 1.50;
cfg.ekf_cusum_weight_gain = 0.50;
cfg.ekf_cusum_weight_min = 0.10;
cfg.ekf_cusum_soft_weight_mode = 'per_follower_max';
% Optional EKF-only recovery constraint.  A confirmed bad raw leader range
% remains isolated; the independent online range predictor may contribute a
% low-weight surrogate range to avoid a sudden loss of three-dimensional
% geometry.  The surrogate never reads truth or injected-fault metadata.
cfg.ekf_cusum_alarm_surrogate_enable = false;
cfg.ekf_cusum_alarm_surrogate_weight = 0.05;
% Range-domain alpha-beta predictor used only by EKF-CUSUM.  It predicts the
% next range from prior online ranges, so an already fused bad range cannot
% cancel the detector innovation through EKF state feedback.
cfg.ekf_cusum_range_predictor_alpha = 0.30;
cfg.ekf_cusum_range_predictor_beta = 0.05;
cfg.ekf_cusum_range_predictor_std = 0.80;
cfg.ekf_cusum_predictor_freeze_threshold = 1.00;
% During an alarm, preserve the trusted range level but track smooth range
% rate changes from consecutive measurements.  A constant additive fault
% cancels in the difference; onset/clear jumps are rejected by the gate.
cfg.ekf_cusum_frozen_rate_adapt_gain = 0.20;
cfg.ekf_cusum_frozen_rate_gate = 1.00;
% Generic predictor-recovery fields consumed by the shared range-domain
% CUSUM implementation.  The FGO detector receives its opt-in values via
% local_cusum_detector_config; the EKF retains these defaults.
cfg.range_predictor_opposite_step_reset_enable = false;
cfg.range_predictor_opposite_step_reset_threshold = 2.5;
cfg.range_predictor_opposite_step_reset_epochs = 2;

% Healthy-edge innovation calibration.  The predictor is intentionally kept
% independent from the graph range update, but its residual can contain a
% persistent nominal offset that is not represented by PK.  Each edge learns
% that offset during warm-up and only adapts it while its CUSUM is healthy.
cfg.cusum_baseline_enable = true;
cfg.cusum_baseline_min_samples = 2;
cfg.cusum_edge_calibration_samples = 10;
cfg.cusum_calibration_innovation_gate = 3.00;
cfg.cusum_baseline_adapt_gain = 0.05;
cfg.cusum_baseline_innovation_gate = 3.50;
cfg.cusum_baseline_cusum_gate = 6.00;
cfg.cusum_baseline_change_gate = 5.00;
cfg.cusum_baseline_release_cusum_gate = 0.75;
cfg.cusum_baseline_release_innovation_gate = 1.00;
cfg.cusum_baseline_release_epochs = 2;
cfg.cusum_baseline_variance_floor = cfg.sigma_dis^2;

% These fields are used only to inject and report an offline fault.  The
% online CUSUM function deliberately does not accept this configuration.
cfg.fault_enable = false;
cfg.fault_edge = [2, 3];
cfg.fault_start = 100;
cfg.fault_end = 150;
cfg.fault_bias = 2;
% Fault injection profiles are offline experiment settings only.  'single'
% preserves the original one-edge/one-window baseline.  The opt-in
% 'segmented_random_edge' uses each row of fault_segments as
% [start_time, end_time, additive_bias_m] and resolves one distinct random
% follower-leader edge per row from the configured seed.  The opt-in
% 'segmented_explicit_edges' mode uses five columns by profile convention:
% [start_time, end_time, additive_bias_m, follower, leader_global].
cfg.fault_mode = 'single';
cfg.fault_segments = zeros(0, 3);

cfg.verbose = true;
cfg.assert_node_mapping = true;

switch lower(profile)
    case {'full', 'default'}
        % Keep the original 600 s duration.
    case 'quick'
        % Intended for smoke tests only; the navigation/noise models are not
        % changed, only the duration and offline fault window are shortened.
        cfg.t_stop = 20;
        cfg.cusum_warmup_time = 2;
        cfg.cusum_edge_calibration_samples = 2;
        cfg.ekf_cusum_calibration_samples = 2;
        cfg.fault_start = 5;
        cfg.fault_end = 12;
        cfg.fault_bias = 5;
    otherwise
        error('stage1_cusum_default_config:UnknownProfile', ...
            'Unknown profile "%s". Use "full" or "quick".', profile);
end
end
