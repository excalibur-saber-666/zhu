function [weights, detail, state] = compute_ekf_cusum_range_weights( ...
        edges, cfg, state, current_time)
%COMPUTE_EKF_CUSUM_RANGE_WEIGHTS Online CUSUM protection for local EKF ranges.
%   Each edge predicts its next range from its own prior online ranges using
%   a constant-velocity alpha-beta model.  Its two-sided CUSUM then monitors
%   this range-domain innovation, independently of the EKF state feedback.
%   Warning evidence continuously enlarges the effective covariance before
%   the runner selectively isolates a leader edge after alarm confirmation.

local_validate_inputs(edges, cfg);
if nargin < 3 || isempty(state)
    state = local_empty_state();
end
detail = local_empty_detail(edges);
candidate_weights = ones(numel(edges), 1);
for edge_index = 1:numel(edges)
    pair = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
    state_index = local_find_or_add_state(state, pair);
    if isempty(state_index)
        state = local_add_state(state, pair);
        state_index = size(state.keys, 1);
    end
    [state, predicted_range, innovation, initialized] = local_predict_range( ...
        state, state_index, edges(edge_index).measurement, current_time);
    innovation_variance = max(cfg.sigma_dis^2 + cfg.ekf_cusum_range_predictor_std^2, ...
        cfg.ekf_cusum_baseline_variance_floor);

    detail.global_pairs(edge_index, :) = pair;
    detail.measurement(edge_index) = edges(edge_index).measurement;
    detail.predicted_range(edge_index) = predicted_range;
    detail.innovation(edge_index) = innovation;
    detail.innovation_variance(edge_index) = innovation_variance;

    if ~initialized || ~state.baseline_valid(state_index)
        [state, accepted] = local_calibrate(state, state_index, innovation, innovation_variance, cfg);
        detail.calibration_accepted(edge_index) = accepted;
        detail = local_write_detail(detail, edge_index, state, state_index, ...
            innovation, innovation_variance, 0, 0, ...
            detail.calibration_accepted(edge_index), 1);
        state = local_update_range_predictor(state, state_index, predicted_range, ...
            innovation, edges(edge_index).measurement, current_time, cfg);
        continue;
    end
    baseline_variance = max([state.baseline_variance(state_index), innovation_variance, ...
        cfg.ekf_cusum_baseline_variance_floor]);
    z = (innovation - state.baseline_mean(state_index)) / sqrt(baseline_variance);
    old_positive = state.cusum_positive(state_index);
    old_negative = state.cusum_negative(state_index);
    c_positive = max(0, cfg.ekf_cusum_lambda * old_positive + z - cfg.ekf_cusum_kappa);
    c_negative = max(0, cfg.ekf_cusum_lambda * old_negative - z - cfg.ekf_cusum_kappa);
    c_value = max(c_positive, c_negative);
    state.cusum_positive(state_index) = c_positive;
    state.cusum_negative(state_index) = c_negative;
    [state, predictor_reset] = local_reset_predictor_after_opposite_step( ...
        state, state_index, z, edges(edge_index).measurement, current_time, cfg);
    if predictor_reset
        c_value = 0;
        alarm_entered = false;
        alarm_cleared = true;
    else
        [state, alarm_entered, alarm_cleared] = local_update_alarm(state, state_index, c_value, z, cfg);
    end

    freeze_predictor = state.alarm_active(state_index) || ...
        c_value >= cfg.ekf_cusum_predictor_freeze_threshold;
    if ~freeze_predictor && abs(z) <= cfg.ekf_cusum_innovation_gate
        state = local_adapt_baseline(state, state_index, innovation, cfg);
    end
    weight = local_alarm_weight(c_value, z, state.alarm_active(state_index), cfg);
    candidate_weights(edge_index) = weight;
    detail = local_write_detail(detail, edge_index, state, state_index, innovation, ...
        baseline_variance, z, c_value, true, weight);
    detail.alarm_entered(edge_index) = alarm_entered;
    detail.alarm_cleared(edge_index) = alarm_cleared;
    if ~predictor_reset
        state = local_update_range_predictor(state, state_index, predicted_range, ...
            innovation, edges(edge_index).measurement, current_time, cfg, freeze_predictor);
    end
end

weights = local_select_weights(edges, candidate_weights, detail, cfg);
detail.detector_weight_before_ekf_adaptation = candidate_weights;
detail.final_weight = weights;
end

function state = local_empty_state()
state.keys = zeros(0, 2);
state.baseline_count = zeros(0, 1);
state.baseline_mean = zeros(0, 1);
state.baseline_m2 = zeros(0, 1);
state.baseline_variance = zeros(0, 1);
state.baseline_valid = false(0, 1);
state.cusum_positive = zeros(0, 1);
state.cusum_negative = zeros(0, 1);
state.alarm_active = false(0, 1);
state.alarm_direction = zeros(0, 1);
state.confirm_count = zeros(0, 1);
state.release_count = zeros(0, 1);
state.alarm_count = zeros(0, 1);
state.baseline_update_count = zeros(0, 1);
state.last_seen_time = nan(0, 1);
state.range_initialized = false(0, 1);
state.range_value = zeros(0, 1);
state.range_rate = zeros(0, 1);
state.range_time = nan(0, 1);
state.range_last_measurement = nan(0, 1);
state.opposite_innovation_count = zeros(0, 1);
state.predictor_reset_count = zeros(0, 1);
end

function state_index = local_find_or_add_state(state, pair)
state_index = find(state.keys(:, 1) == pair(1) & state.keys(:, 2) == pair(2), 1, 'first');
end

function state = local_add_state(state, pair)
state.keys(end + 1, :) = pair;
fields = fieldnames(state);
for field_index = 1:numel(fields)
    name = fields{field_index};
    if strcmp(name, 'keys')
        continue;
    end
    if islogical(state.(name))
        state.(name)(end + 1, 1) = false;
    else
        state.(name)(end + 1, 1) = 0;
    end
end
state.last_seen_time(end, 1) = NaN;
state.range_last_measurement(end, 1) = NaN;
end

function [state, accepted] = local_calibrate(state, state_index, innovation, variance, cfg)
accepted = abs(innovation) <= cfg.ekf_cusum_innovation_gate * sqrt(variance);
if ~accepted
    return;
end
count = state.baseline_count(state_index) + 1;
delta = innovation - state.baseline_mean(state_index);
mean_value = state.baseline_mean(state_index) + delta / count;
state.baseline_m2(state_index) = state.baseline_m2(state_index) + ...
    delta * (innovation - mean_value);
state.baseline_count(state_index) = count;
state.baseline_mean(state_index) = mean_value;
if count >= 2
    state.baseline_variance(state_index) = max(cfg.ekf_cusum_baseline_variance_floor, ...
        state.baseline_m2(state_index) / (count - 1));
end
if count >= cfg.ekf_cusum_calibration_samples
    state.baseline_valid(state_index) = true;
end
end

function state = local_adapt_baseline(state, state_index, innovation, cfg)
gain = cfg.ekf_cusum_baseline_adapt_gain;
old_mean = state.baseline_mean(state_index);
centered = innovation - old_mean;
state.baseline_mean(state_index) = old_mean + gain * centered;
state.baseline_variance(state_index) = max(cfg.ekf_cusum_baseline_variance_floor, ...
    (1 - gain) * state.baseline_variance(state_index) + gain * centered^2);
state.baseline_count(state_index) = state.baseline_count(state_index) + 1;
state.baseline_update_count(state_index) = state.baseline_update_count(state_index) + 1;
end

function [state, predicted_range, innovation, initialized] = local_predict_range( ...
        state, state_index, measurement, current_time)
initialized = state.range_initialized(state_index);
if ~initialized
    state.range_initialized(state_index) = true;
    state.range_value(state_index) = measurement;
    state.range_rate(state_index) = 0;
    state.range_time(state_index) = current_time;
    state.range_last_measurement(state_index) = measurement;
    state.last_seen_time(state_index) = current_time;
    predicted_range = measurement;
    innovation = 0;
    return;
end
dt = max(current_time - state.range_time(state_index), eps);
predicted_range = state.range_value(state_index) + state.range_rate(state_index) * dt;
innovation = measurement - predicted_range;
state.last_seen_time(state_index) = current_time;
end

function state = local_update_range_predictor(state, state_index, predicted_range, ...
        innovation, measurement, current_time, cfg, freeze_predictor)
if nargin < 8
    freeze_predictor = false;
end
dt = max(current_time - state.range_time(state_index), eps);
if freeze_predictor
    % Keep the trusted range level independent of the anomalous measurement.
    % Smooth consecutive range differences still carry useful motion
    % information because a persistent additive bias cancels in a difference.
    old_rate = state.range_rate(state_index);
    last_measurement = state.range_last_measurement(state_index);
    if isfinite(last_measurement)
        observed_rate = (measurement - last_measurement) / dt;
        rate_error = observed_rate - old_rate;
        if abs(rate_error) <= cfg.ekf_cusum_frozen_rate_gate
            state.range_rate(state_index) = old_rate + ...
                cfg.ekf_cusum_frozen_rate_adapt_gain * rate_error;
        end
    end
    state.range_value(state_index) = predicted_range + ...
        (state.range_rate(state_index) - old_rate) * dt;
else
    state.range_value(state_index) = predicted_range + ...
        cfg.ekf_cusum_range_predictor_alpha * innovation;
    state.range_rate(state_index) = state.range_rate(state_index) + ...
        cfg.ekf_cusum_range_predictor_beta * innovation / dt;
end
state.range_time(state_index) = current_time;
state.range_last_measurement(state_index) = measurement;
end

function [state, entered, cleared] = local_update_alarm(state, state_index, c_value, z, cfg)
entered = false;
cleared = false;
if ~state.alarm_active(state_index)
    if c_value >= cfg.ekf_cusum_alarm_on_threshold
        state.confirm_count(state_index) = state.confirm_count(state_index) + 1;
    else
        state.confirm_count(state_index) = 0;
    end
    if state.confirm_count(state_index) >= cfg.ekf_cusum_alarm_confirm_epochs
        state.alarm_active(state_index) = true;
        state.alarm_direction(state_index) = sign( ...
            state.cusum_positive(state_index) - state.cusum_negative(state_index));
        if state.alarm_direction(state_index) == 0
            state.alarm_direction(state_index) = sign(z);
        end
        state.alarm_count(state_index) = state.alarm_count(state_index) + 1;
        state.release_count(state_index) = 0;
        entered = true;
    end
    return;
end
cusum_recovered = c_value <= cfg.ekf_cusum_alarm_off_threshold && ...
    abs(z) <= cfg.ekf_cusum_innovation_gate;
nominal_innovation_run = abs(z) <= cfg.ekf_cusum_release_innovation_gate;
if cusum_recovered || nominal_innovation_run
    state.release_count(state_index) = state.release_count(state_index) + 1;
else
    state.release_count(state_index) = 0;
end
if state.release_count(state_index) >= cfg.ekf_cusum_alarm_release_epochs
    state.alarm_active(state_index) = false;
    state.confirm_count(state_index) = 0;
    state.release_count(state_index) = 0;
    state.cusum_positive(state_index) = 0;
    state.cusum_negative(state_index) = 0;
    state.alarm_direction(state_index) = 0;
    cleared = true;
end
end

function [state, reset_triggered] = local_reset_predictor_after_opposite_step( ...
        state, state_index, z, measurement, current_time, cfg)
% A strong, persistent step opposite to an active CUSUM indicates that the
% prior step fault may have cleared or changed sign.  Reinitialize only after
% online confirmation; a newly persistent fault will immediately start a new
% CUSUM accumulation.  This decision never reads truth or fault metadata.
reset_triggered = false;
if ~cfg.range_predictor_opposite_step_reset_enable || ...
        ~state.alarm_active(state_index)
    state.opposite_innovation_count(state_index) = 0;
    return;
end
alarm_direction = state.alarm_direction(state_index);
strong_opposite = alarm_direction ~= 0 && ...
    alarm_direction * z <= -cfg.range_predictor_opposite_step_reset_threshold;
if ~strong_opposite
    state.opposite_innovation_count(state_index) = 0;
    return;
end
state.opposite_innovation_count(state_index) = ...
    state.opposite_innovation_count(state_index) + 1;
if state.opposite_innovation_count(state_index) < ...
        cfg.range_predictor_opposite_step_reset_epochs
    return;
end

state.cusum_positive(state_index) = 0;
state.cusum_negative(state_index) = 0;
state.alarm_active(state_index) = false;
state.alarm_direction(state_index) = 0;
state.confirm_count(state_index) = 0;
state.release_count(state_index) = 0;
state.opposite_innovation_count(state_index) = 0;
state.range_initialized(state_index) = true;
state.range_value(state_index) = measurement;
state.range_rate(state_index) = 0;
state.range_time(state_index) = current_time;
state.range_last_measurement(state_index) = measurement;
state.predictor_reset_count(state_index) = ...
    state.predictor_reset_count(state_index) + 1;
reset_triggered = true;
end

function weight = local_alarm_weight(c_value, z, ~, cfg)
% Preserve nominal ranges exactly inside the warning deadzone, then soften
% continuously as persistent evidence accumulates.  This makes the soft
% stage observable before a confirmed alarm triggers selective isolation.
cusum_excess = max(0, c_value - cfg.ekf_cusum_weight_deadzone);
innovation_excess = max(0, abs(z) - cfg.ekf_cusum_innovation_gate);
weight = 1 / (1 + cfg.ekf_cusum_weight_gain * ...
    (cusum_excess + innovation_excess^2));
weight = max(cfg.ekf_cusum_weight_min, min(1, weight));
end

function weights = local_select_weights(edges, candidate_weights, detail, cfg)
weights = ones(size(candidate_weights));
mode = char(cfg.ekf_cusum_soft_weight_mode);
if strcmpi(mode, 'all')
    weights = candidate_weights;
    return;
end

low_num = cfg.uav_num - cfg.high_num;
for follower = 1:low_num
    candidates = zeros(0, 1);
    for edge_index = 1:numel(edges)
        pair = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
        if pair(1) == follower && pair(2) > low_num && ...
                candidate_weights(edge_index) < 1 - 10 * eps
            candidates(end + 1, 1) = edge_index; %#ok<AGROW>
        end
    end
    if isempty(candidates)
        continue;
    end
    [~, local_index] = max(detail.cusum_value(candidates));
    selected = candidates(local_index);
    weights(selected) = candidate_weights(selected);
end
end

function detail = local_empty_detail(edges)
edge_count = numel(edges);
detail.global_pairs = zeros(edge_count, 2);
detail.measurement = nan(edge_count, 1);
detail.predicted_range = nan(edge_count, 1);
detail.innovation = nan(edge_count, 1);
detail.innovation_variance = nan(edge_count, 1);
detail.centered_innovation = nan(edge_count, 1);
detail.effective_innovation_variance = nan(edge_count, 1);
detail.baseline_normalized_innovation = nan(edge_count, 1);
detail.signed_normalized_innovation = nan(edge_count, 1);
detail.innovation_change = nan(edge_count, 1);
detail.cusum_positive = zeros(edge_count, 1);
detail.cusum_negative = zeros(edge_count, 1);
detail.cusum_value = zeros(edge_count, 1);
detail.cusum_excess = zeros(edge_count, 1);
detail.effective_weight_scale = ones(edge_count, 1);
detail.final_weight = ones(edge_count, 1);
detail.baseline_count = zeros(edge_count, 1);
detail.baseline_mean = nan(edge_count, 1);
detail.baseline_variance = nan(edge_count, 1);
detail.baseline_ready = false(edge_count, 1);
detail.baseline_unverified = true(edge_count, 1);
detail.calibrating = true(edge_count, 1);
detail.calibration_accepted = false(edge_count, 1);
detail.calibration_rejected_count = zeros(edge_count, 1);
detail.baseline_frozen = false(edge_count, 1);
detail.baseline_update_count = zeros(edge_count, 1);
detail.baseline_freeze_count = zeros(edge_count, 1);
detail.baseline_unfreeze_count = zeros(edge_count, 1);
detail.baseline_freeze_event = false(edge_count, 1);
detail.baseline_unfreeze_event = false(edge_count, 1);
detail.alarm_active = false(edge_count, 1);
detail.confirm_count = zeros(edge_count, 1);
detail.release_count = zeros(edge_count, 1);
detail.alarm_count = zeros(edge_count, 1);
detail.alarm_entered = false(edge_count, 1);
detail.alarm_cleared = false(edge_count, 1);
detail.consensus_available = false(edge_count, 1);
detail.consensus_neighbor_count = zeros(edge_count, 1);
detail.consensus_reference = nan(edge_count, 1);
detail.zero_range_count = 0;
detail.nonfinite_fallback_count = 0;
detail.missing_decay_count = 0;
detail.reset_count = 0;
end

function detail = local_write_detail(detail, edge_index, state, state_index, ...
        innovation, variance, z, c_value, calibration_accepted, weight)
detail.centered_innovation(edge_index) = innovation - state.baseline_mean(state_index);
detail.effective_innovation_variance(edge_index) = variance;
detail.baseline_normalized_innovation(edge_index) = z;
detail.signed_normalized_innovation(edge_index) = z;
detail.cusum_positive(edge_index) = state.cusum_positive(state_index);
detail.cusum_negative(edge_index) = state.cusum_negative(state_index);
detail.cusum_value(edge_index) = c_value;
detail.cusum_excess(edge_index) = max(0, c_value);
detail.effective_weight_scale(edge_index) = weight;
detail.baseline_count(edge_index) = state.baseline_count(state_index);
detail.baseline_mean(edge_index) = state.baseline_mean(state_index);
detail.baseline_variance(edge_index) = state.baseline_variance(state_index);
detail.baseline_ready(edge_index) = state.baseline_valid(state_index);
detail.baseline_unverified(edge_index) = ~state.baseline_valid(state_index);
detail.calibrating(edge_index) = ~state.baseline_valid(state_index);
detail.calibration_accepted(edge_index) = calibration_accepted;
detail.baseline_frozen(edge_index) = state.alarm_active(state_index);
detail.baseline_update_count(edge_index) = state.baseline_update_count(state_index);
detail.alarm_active(edge_index) = state.alarm_active(state_index);
detail.confirm_count(edge_index) = state.confirm_count(state_index);
detail.release_count(edge_index) = state.release_count(state_index);
detail.alarm_count(edge_index) = state.alarm_count(state_index);
detail.reset_count(edge_index) = state.predictor_reset_count(state_index);
end

function local_validate_inputs(edges, cfg)
required = {'sigma_dis', 'cusum_eps', 'ekf_cusum_calibration_samples', ...
    'ekf_cusum_baseline_adapt_gain', 'ekf_cusum_baseline_variance_floor', ...
    'ekf_cusum_lambda', 'ekf_cusum_kappa', 'ekf_cusum_alarm_on_threshold', ...
    'ekf_cusum_alarm_off_threshold', 'ekf_cusum_alarm_confirm_epochs', ...
    'ekf_cusum_alarm_release_epochs', 'ekf_cusum_innovation_gate', ...
    'ekf_cusum_release_innovation_gate', ...
    'ekf_cusum_weight_deadzone', 'ekf_cusum_weight_gain', ...
    'ekf_cusum_weight_min', 'ekf_cusum_soft_weight_mode', ...
    'ekf_cusum_range_predictor_alpha', 'ekf_cusum_range_predictor_beta', ...
    'ekf_cusum_range_predictor_std', 'ekf_cusum_predictor_freeze_threshold', ...
    'ekf_cusum_frozen_rate_adapt_gain', 'ekf_cusum_frozen_rate_gate', ...
    'range_predictor_opposite_step_reset_enable', ...
    'range_predictor_opposite_step_reset_threshold', ...
    'range_predictor_opposite_step_reset_epochs', ...
    'uav_num', 'high_num'};
for field_index = 1:numel(required)
    if ~isfield(cfg, required{field_index})
        error('compute_ekf_cusum_range_weights:MissingConfig', 'Missing cfg.%s.', required{field_index});
    end
end
if ~isstruct(edges) || ~all(isfield(edges, {'global_i', 'global_j', 'measurement'}))
    error('compute_ekf_cusum_range_weights:InvalidEdges', 'Each edge needs global_i, global_j, and measurement.');
end
for edge_index = 1:numel(edges)
    edge = edges(edge_index);
    if edge.global_i < 1 || edge.global_j < 1 || edge.global_i > cfg.uav_num || ...
            edge.global_j > cfg.uav_num || edge.global_i == edge.global_j || ...
            edge.measurement <= 0 || any(~isfinite([edge.global_i, edge.global_j, edge.measurement]))
        error('compute_ekf_cusum_range_weights:InvalidEdges', 'An EKF-CUSUM edge is invalid.');
    end
end
if cfg.sigma_dis <= 0 || cfg.cusum_eps <= 0 || cfg.ekf_cusum_calibration_samples < 2 || ...
        cfg.ekf_cusum_baseline_adapt_gain < 0 || cfg.ekf_cusum_baseline_adapt_gain > 1 || ...
        cfg.ekf_cusum_baseline_variance_floor <= 0 || cfg.ekf_cusum_lambda < 0 || ...
        cfg.ekf_cusum_lambda > 1 || cfg.ekf_cusum_kappa < 0 || ...
        cfg.ekf_cusum_alarm_on_threshold <= cfg.ekf_cusum_alarm_off_threshold || ...
        cfg.ekf_cusum_alarm_confirm_epochs < 1 || cfg.ekf_cusum_alarm_release_epochs < 1 || ...
        cfg.ekf_cusum_innovation_gate <= 0 || ...
        cfg.ekf_cusum_release_innovation_gate <= 0 || ...
        cfg.ekf_cusum_release_innovation_gate > cfg.ekf_cusum_innovation_gate || ...
        cfg.ekf_cusum_weight_deadzone < 0 || ...
        cfg.ekf_cusum_weight_gain < 0 || ...
        cfg.ekf_cusum_weight_min <= 0 || cfg.ekf_cusum_weight_min > 1 || ...
        cfg.ekf_cusum_range_predictor_alpha <= 0 || cfg.ekf_cusum_range_predictor_alpha > 1 || ...
        cfg.ekf_cusum_range_predictor_beta < 0 || cfg.ekf_cusum_range_predictor_std <= 0 || ...
        cfg.ekf_cusum_predictor_freeze_threshold < 0 || ...
        cfg.ekf_cusum_frozen_rate_adapt_gain < 0 || ...
        cfg.ekf_cusum_frozen_rate_adapt_gain > 1 || cfg.ekf_cusum_frozen_rate_gate <= 0 || ...
        ~isscalar(cfg.range_predictor_opposite_step_reset_enable) || ...
        (~islogical(cfg.range_predictor_opposite_step_reset_enable) && ...
        ~ismember(cfg.range_predictor_opposite_step_reset_enable, [0, 1])) || ...
        ~isscalar(cfg.range_predictor_opposite_step_reset_threshold) || ...
        ~isfinite(cfg.range_predictor_opposite_step_reset_threshold) || ...
        cfg.range_predictor_opposite_step_reset_threshold <= 0 || ...
        ~isscalar(cfg.range_predictor_opposite_step_reset_epochs) || ...
        cfg.range_predictor_opposite_step_reset_epochs < 1 || ...
        cfg.range_predictor_opposite_step_reset_epochs ~= ...
        floor(cfg.range_predictor_opposite_step_reset_epochs)
    error('compute_ekf_cusum_range_weights:InvalidConfig', 'EKF-CUSUM configuration is outside its valid range.');
end
end
