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

cfg.seeds = 1;
cfg.cusum_apply = true;
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
