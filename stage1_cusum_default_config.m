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

% Graph-selection controls.  The default preserves the published Stage-1
% equal-weight versus CUSUM comparison.  The sliding-window comparison is
% enabled only by run_stage1_sliding_window_cusum_comparison.
cfg.graph_mode = 'single_epoch';
cfg.sliding_window_length = 10;
cfg.sliding_window_motion_std = [2; 2; 4];
% This is an empirical SINS relative-position constraint, not a full IMU
% preintegration factor. Keep the switch explicit for ablation studies.
cfg.sliding_window_motion_enable = true;
% Cross-frame model: 'none', 'sins_delta' (the established position-only
% path), or 'imu_preint'.  The default deliberately preserves sins_delta.
cfg.sliding_window_motion_model = 'sins_delta';
cfg.imu_preint_window_length = 3;
cfg.imu_preint_use_bias_states = true;
cfg.imu_preint_use_covariance_propagation = true;
cfg.imu_preint_repropagate_enable = true;
cfg.imu_preint_gyro_bias_repropagate_threshold = 5e-5;
cfg.imu_preint_acc_bias_repropagate_threshold = 5e-3;
% Values are SI continuous-noise densities mapped from imu_err_random.m:
% gyro white noise is 10 deg/h per dt sample; accelerometer Markov noise has
% stationary sigma 0.001 g and time constant 1800 s.
cfg.imu_preint_gyro_noise_std = 10 * pi / (3600 * 180) / sqrt(cfg.dt);
cfg.imu_preint_acc_noise_std = 1e-3 * 9.7803698 / sqrt(cfg.dt);
cfg.imu_preint_gyro_bias_rw_std = sqrt(2 / 3600) * 10 * pi / (3600 * 180);
cfg.imu_preint_acc_bias_rw_std = sqrt(2 / 1800) * 1e-3 * 9.7803698;
cfg.imu_preint_gn_max_iterations = 30;
cfg.imu_preint_gn_step_tolerance = 1e-5;
cfg.imu_preint_numeric_jacobian_enable = true;
cfg.imu_preint_position_eps = 1e-5;
cfg.imu_preint_velocity_eps = 1e-5;
cfg.imu_preint_rotation_eps = 1e-7;
cfg.imu_preint_gyro_bias_eps = 1e-8;
cfg.imu_preint_acc_bias_eps = 1e-6;
% Follower priors for the new 15-state graph.  Position units are m,
% velocity m/s, rotation rad, gyro bias rad/s, and accel bias m/s^2.
cfg.imu_preint_position_prior_std = [10; 10; 20];
cfg.imu_preint_velocity_prior_std = [5; 5; 8];
cfg.imu_preint_rotation_prior_std = [5; 5; 10] * pi / 180;
cfg.imu_preint_gyro_bias_prior_std = 10 * 10 * pi / (3600 * 180) * ones(3, 1);
cfg.imu_preint_acc_bias_prior_std = 5 * 1e-3 * 9.7803698 * ones(3, 1);
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
% 对比模式切换说明（运行时只需修改下一行 cfg.comparison_mode）：
%   'equal_vs_cusum'              : 单历元 Equal-FGO 对比单历元 CUSUM-FGO。
%   'original_vs_sliding_cusum'   : 原始单历元 Equal-FGO 对比当前
%                                    滑动窗口 + CUSUM + 在线隔离方法（主对比）。
%   'original_vs_sliding_equal'   : 仅用于审计/消融：原始 Equal-FGO 对比
%                                    滑动窗口等权 FGO（不使用 CUSUM）。
% 将 cfg.plot_component_comparison 设为 true 可绘制三轴误差对比图。
% 需要自由切换模式时，请使用 run_stage1_cusum_comparison(cfg)，而不要
% 使用 run_stage1_sliding_window_cusum_comparison(cfg)；后者固定为第二种模式。
cfg.comparison_mode = 'equal_vs_cusum';
cfg.plot_component_comparison = false;
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

cfg.verbose = true;
cfg.assert_node_mapping = true;
cfg.enable_original_baseline_regression = false;

switch lower(profile)
    case {'full', 'default'}
        % Keep the original 600 s duration.
    case 'quick'
        % Intended for smoke tests only; the navigation/noise models are not
        % changed, only the duration and offline fault window are shortened.
        cfg.t_stop = 20;
        cfg.cusum_warmup_time = 2;
        cfg.cusum_edge_calibration_samples = 2;
        cfg.fault_start = 5;
        cfg.fault_end = 12;
        cfg.fault_bias = 5;
    otherwise
        error('stage1_cusum_default_config:UnknownProfile', ...
            'Unknown profile "%s". Use "full" or "quick".', profile);
end
end
