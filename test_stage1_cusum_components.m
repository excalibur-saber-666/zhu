function test_stage1_cusum_components()
%TEST_STAGE1_CUSUM_COMPONENTS Deterministic Stage-1 CUSUM state tests.

cfg = stage1_cusum_default_config('full');
cfg.cusum_edge_calibration_samples = 2;
cfg.cusum_calibration_innovation_gate = 3;
cfg.cusum_alarm_on_threshold = 2;
positions = zeros(3, 5);
positions(:, 1) = [0; 0; 0];
positions(:, 2) = [10; 0; 0];
positions(:, 3) = [0; 10; 0];
covariances = zeros(3, 3, 5);
zero_edge = local_edge(1, 2, 10);

% Per-edge calibration establishes a nominal reference and keeps full weight.
state = [];
for time = 1:2
    [weights, detail, state] = compute_signed_cusum_edge_weights( ...
        zero_edge, positions, covariances, cfg, state, time);
end
assert(weights == 1 && detail.baseline_ready && ~detail.calibrating && ...
    ~detail.baseline_unverified && ...
    abs(detail.baseline_mean) < 1e-12, 'Independent edge calibration failed.');

% Healthy deadzone: small alternating innovations cannot reduce the weight.
for time = 3:18
    edge = local_edge(1, 2, 10 + 0.1 * (-1)^time * cfg.sigma_dis);
    [weights, detail, state] = compute_signed_cusum_edge_weights( ...
        edge, positions, covariances, cfg, state, time);
    assert(weights == 1 && ~detail.alarm_active, ...
        'Healthy deadzone or alternating-innovation handling failed.');
end

% Positive and negative persistent steps drive the corresponding CUSUM side.
[positive_weight, positive_detail, positive_state] = local_calibrate_then_step( ...
    cfg, positions, covariances, +3, 4);
assert(positive_detail.cusum_positive > 0 && positive_detail.cusum_negative < eps && ...
    positive_weight < 1 && positive_detail.alarm_active && positive_detail.baseline_frozen, ...
    'Positive step did not produce a confirmed positive alarm.');
[negative_weight, negative_detail, ~] = local_calibrate_then_step( ...
    cfg, positions, covariances, -3, 4);
assert(negative_detail.cusum_negative > 0 && negative_detail.cusum_positive < eps && ...
    negative_weight < 1 && negative_detail.alarm_active, ...
    'Negative step did not produce a confirmed negative alarm.');

% A larger single-epoch step is always mapped to a lower soft weight.
[weak_weight, ~, ~] = local_calibrate_then_step(cfg, positions, covariances, +1, 1);
[medium_weight, ~, ~] = local_calibrate_then_step(cfg, positions, covariances, +3, 1);
[strong_weight, ~, ~] = local_calibrate_then_step(cfg, positions, covariances, +8, 1);
assert(weak_weight == 1 && medium_weight < weak_weight && strong_weight < medium_weight, ...
    'Soft-weight mapping is not monotonic with step amplitude.');

% A frozen baseline is never adapted by a persistent same-sign fault.
baseline_before_fault = positive_state.baseline_mean;
[~, frozen_detail, frozen_state] = compute_signed_cusum_edge_weights( ...
    local_edge(1, 2, 10 + 3 * cfg.sigma_dis), positions, covariances, cfg, positive_state, 6);
assert(frozen_detail.baseline_frozen && ...
    abs(frozen_state.baseline_mean - baseline_before_fault) < 1e-12, ...
    'Persistent alarm innovation was incorrectly absorbed into the baseline.');

% Fast release only begins after the innovation returns to the healthy band.
for time = 7:16
    [recovered_weight, recovered_detail, frozen_state] = compute_signed_cusum_edge_weights( ...
        zero_edge, positions, covariances, cfg, frozen_state, time);
end
assert(recovered_weight > 0.95 && ~recovered_detail.alarm_active && ...
    ~recovered_detail.baseline_frozen, 'Alarm did not release after healthy innovations.');

% A trusted baseline survives a long link gap; only detector state is reset.
trusted_mean = frozen_state.baseline_mean;
[~, reset_detail, gap_state] = compute_signed_cusum_edge_weights( ...
    repmat(zero_edge, 0, 1), positions, covariances, cfg, frozen_state, 50);
assert(reset_detail.reset_count == 1 && gap_state.baseline_valid && ...
    abs(gap_state.baseline_mean - trusted_mean) < 1e-12, ...
    'Long-missing edge discarded a trusted baseline.');
[gap_weight, gap_detail, ~] = compute_signed_cusum_edge_weights( ...
    local_edge(1, 2, 10 + 5 * cfg.sigma_dis), positions, covariances, cfg, gap_state, 51);
assert(gap_detail.cusum_value > 0 && gap_weight < 1, ...
    'Reappearing edge did not reuse its trusted baseline for detection.');

% A new edge calibrates independently of global time and of existing edges.
state = [];
for time = 1:2
    [~, ~, state] = compute_signed_cusum_edge_weights( ...
        local_edge(1, 2, 10), positions, covariances, cfg, state, time);
end
for time = 3:4
    [~, new_detail, state] = compute_signed_cusum_edge_weights( ...
        local_edge(1, 3, 10), positions, covariances, cfg, state, time);
end
assert(size(state.keys, 1) == 2 && new_detail.baseline_ready && ...
    ~new_detail.calibrating, 'Late-appearing edge did not calibrate independently.');

% A first-appearance extreme fault is explicitly rejected, not silently used
% as a healthy baseline.  Constant weak initial bias remains unidentifiable
% without a prior/reference and is recorded as baseline_unverified.
[first_weight, first_detail, first_state] = compute_signed_cusum_edge_weights( ...
    local_edge(1, 2, 10 + 8 * cfg.sigma_dis), positions, covariances, cfg, [], 1);
assert(first_weight == 1 && first_detail.calibrating && ...
    first_detail.calibration_rejected_count == 1 && ...
    first_state.baseline_unverified, ...
    'Initial extreme fault was silently accepted as a baseline sample.');

% Edge ordering, finite output, disabled mode, and input validation.
edge_a = local_edge(1, 2, 10 + 2 * cfg.sigma_dis);
edge_b = local_edge(1, 3, 10 + 1 * cfg.sigma_dis);
[ordered_weights, ~, ~] = compute_signed_cusum_edge_weights([edge_a; edge_b], ...
    positions, covariances, cfg, [], 1);
[reordered_weights, ~, reversed_state] = compute_signed_cusum_edge_weights([edge_b; edge_a], ...
    positions, covariances, cfg, [], 1);
assert(size(reversed_state.keys, 1) == 2 && ...
    abs(ordered_weights(1) - reordered_weights(2)) < eps && ...
    abs(ordered_weights(2) - reordered_weights(1)) < eps && ...
    all(isfinite(ordered_weights)) && isreal(ordered_weights) && ...
    all(ordered_weights >= cfg.cusum_weight_min) && all(ordered_weights <= 1));
disabled_cfg = cfg;
disabled_cfg.cusum_apply = false;
[disabled_weight, ~, ~] = compute_signed_cusum_edge_weights( ...
    edge_a, positions, covariances, disabled_cfg, [], 1);
assert(disabled_weight == 1 && nargin('compute_signed_cusum_edge_weights') == 6, ...
    'Disabled mode or online API contract changed.');
invalid_covariances = covariances;
invalid_covariances(:, :, 1) = diag([1, 1, -1]);
local_expect_error(@() compute_signed_cusum_edge_weights( ...
    edge_a, positions, invalid_covariances, cfg, [], 1), ...
    'Non-PSD covariance was accepted.');

% EKF-CUSUM uses its own online range-domain predictor.  A persistent step
% must be confirmed and softened without consulting any offline fault fields.
ekf_cfg = cfg;
ekf_cfg.ekf_cusum_calibration_samples = 3;
ekf_cfg.ekf_cusum_range_predictor_std = 0.8;
ekf_cfg.ekf_cusum_predictor_freeze_threshold = 1.5;
ekf_state_a = [];
ekf_state_b = [];
ekf_cfg_with_offline_fault = ekf_cfg;
ekf_cfg_with_offline_fault.fault_enable = true;
ekf_cfg_with_offline_fault.fault_edge = [2, 4];
ekf_cfg_with_offline_fault.fault_start = 999;
ekf_cfg_with_offline_fault.fault_end = 1000;
ekf_cfg_with_offline_fault.fault_bias = 99;
for time = 1:10
    measurement = 10 + 0.1 * time;
    if time >= 7
        measurement = measurement + 3;
    end
    ekf_edge = local_edge(1, 3, measurement);
    [ekf_weight_a, ekf_detail_a, ekf_state_a] = compute_ekf_cusum_range_weights( ...
        ekf_edge, ekf_cfg, ekf_state_a, time);
    [ekf_weight_b, ekf_detail_b, ekf_state_b] = compute_ekf_cusum_range_weights( ...
        ekf_edge, ekf_cfg_with_offline_fault, ekf_state_b, time);
end
assert(ekf_detail_a.alarm_active && ekf_weight_a < 1 && ...
    ekf_detail_a.cusum_value > ekf_cfg.ekf_cusum_alarm_on_threshold, ...
    'EKF-CUSUM did not confirm and soften a persistent range step.');
assert(isequal(ekf_weight_a, ekf_weight_b) && ...
    isequal(ekf_detail_a.cusum_value, ekf_detail_b.cusum_value) && ...
    isequal(ekf_state_a, ekf_state_b), ...
    'EKF-CUSUM read offline fault metadata.');
for time = 11:16
    ekf_edge = local_edge(1, 3, 10 + 0.1 * time);
    [ekf_weight_a, ekf_detail_a, ekf_state_a] = compute_ekf_cusum_range_weights( ...
        ekf_edge, ekf_cfg, ekf_state_a, time);
end
assert(~ekf_detail_a.alarm_active && ekf_weight_a == 1, ...
    'EKF-CUSUM did not release a confirmed alarm after nominal innovations returned.');

state_example = (1:18)';
gyro_correction = state_example(10:12) + state_example(13:15);
assert(isequal(gyro_correction, [23; 25; 27]), 'Three-axis gyro correction indices are incorrect.');
graph = factor_graph_centralization(zeros(3, 3), [0.2; 0.2; 0.5]);
assert(~graph.is_step_converged(-1e-3 * ones(3, 1), 1e-5) && ...
    graph.is_step_converged(-1e-6 * ones(3, 1), 1e-5), ...
    'GN step convergence check regressed.');
fprintf('test_stage1_cusum_components: PASS\n');
end

function [weight, detail, state] = local_calibrate_then_step( ...
        cfg, positions, covariances, amplitude_sigma, repeat_count)
state = [];
for time = 1:cfg.cusum_edge_calibration_samples
    [~, ~, state] = compute_signed_cusum_edge_weights( ...
        local_edge(1, 2, 10), positions, covariances, cfg, state, time);
end
for index = 1:repeat_count
    [weight, detail, state] = compute_signed_cusum_edge_weights( ...
        local_edge(1, 2, 10 + amplitude_sigma * cfg.sigma_dis), ...
        positions, covariances, cfg, state, cfg.cusum_edge_calibration_samples + index);
end
end

function edge = local_edge(global_i, global_j, measurement)
edge = struct('global_i', global_i, 'global_j', global_j, 'measurement', measurement);
end

function local_expect_error(action, message)
did_error = false;
try
    action();
catch
    did_error = true;
end
assert(did_error, message);
end
