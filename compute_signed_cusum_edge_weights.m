function [weights, detail, state] = compute_signed_cusum_edge_weights( ...
    edges, node_positions_prior, node_covariances_prior, cfg, state, current_time)
%COMPUTE_SIGNED_CUSUM_EDGE_WEIGHTS Online signed-innovation CUSUM weighting.
%   This function uses only current range measurements and pre-range priors.
%   It deliberately accepts no truth, fault label, fault time, or fault size.

if nargin < 6
    error('compute_signed_cusum_edge_weights:NotEnoughInputs', ...
        'edges, priors, cfg, state, and current_time are required.');
end
if isempty(state)
    state = local_empty_state();
else
    state = local_normalize_state(state);
end

edge_count = numel(edges);
weights = ones(edge_count, 1);
detail = local_empty_detail(edge_count);
if ~isfield(cfg, 'cusum_apply') || ~cfg.cusum_apply
    return;
end

local_validate_cfg(cfg);
local_validate_prior_inputs(node_positions_prior, node_covariances_prior, edges);
active_keys = zeros(edge_count, 2);
for edge_index = 1:edge_count
    [global_i, global_j, measurement] = local_read_edge(edges(edge_index));
    active_keys(edge_index, :) = sort([global_i, global_j]);
    detail.global_pairs(edge_index, :) = active_keys(edge_index, :);
    detail.measurement(edge_index) = measurement;
end
if edge_count > 1 && size(unique(active_keys, 'rows'), 1) ~= edge_count
    error('compute_signed_cusum_edge_weights:DuplicateEdge', ...
        'Each undirected edge must appear at most once per graph epoch.');
end

[state, missing_decay_count, reset_count] = local_decay_missing_edges( ...
    state, active_keys, cfg, current_time);
detail.missing_decay_count = missing_decay_count;
detail.reset_count = reset_count;

for edge_index = 1:edge_count
    global_i = active_keys(edge_index, 1);
    global_j = active_keys(edge_index, 2);
    state_index = local_find_state(state, global_i, global_j);
    if isempty(state_index)
        state = local_add_state(state, global_i, global_j, current_time);
        state_index = size(state.keys, 1);
    end

    measurement = detail.measurement(edge_index);
    position_i = node_positions_prior(:, global_i);
    position_j = node_positions_prior(:, global_j);
    covariance_i = node_covariances_prior(:, :, global_i);
    covariance_j = node_covariances_prior(:, :, global_j);
    displacement = position_i - position_j;
    predicted_range = norm(displacement);
    if ~isfinite(measurement) || ~isreal(measurement) || ...
            ~isfinite(predicted_range) || predicted_range <= cfg.cusum_eps || ...
            any(~isfinite(position_i)) || any(~isfinite(position_j)) || ...
            any(~isfinite(covariance_i(:))) || any(~isfinite(covariance_j(:)))
        detail.zero_range_count = detail.zero_range_count + ...
            double(isfinite(predicted_range) && predicted_range <= cfg.cusum_eps);
        detail.nonfinite_fallback_count = detail.nonfinite_fallback_count + 1;
        state = local_mark_seen(state, state_index, current_time);
        continue;
    end

    unit_direction = displacement / predicted_range;
    innovation_variance = unit_direction' * covariance_i * unit_direction + ...
        unit_direction' * covariance_j * unit_direction + cfg.sigma_dis^2;
    if ~isfinite(innovation_variance) || ~isreal(innovation_variance) || ...
            innovation_variance <= cfg.cusum_eps
        detail.nonfinite_fallback_count = detail.nonfinite_fallback_count + 1;
        state = local_mark_seen(state, state_index, current_time);
        continue;
    end

    innovation = measurement - predicted_range;
    detail.innovation(edge_index) = innovation;
    detail.innovation_variance(edge_index) = innovation_variance;

    % Calibration is per edge, not tied to global simulation time.  A new
    % edge has no trustworthy nominal residual yet, so it keeps full weight
    % and is explicitly marked unverified until enough plausible samples are
    % gathered.  Without a historical baseline, an edge born already faulty
    % is fundamentally not identifiable from this edge alone.
    if state.calibrating(state_index)
        [state, calibration_accepted] = local_update_edge_calibration( ...
            state, state_index, innovation, innovation_variance, cfg);
        state.cusum_positive(state_index) = 0;
        state.cusum_negative(state_index) = 0;
        state.alarm_active(state_index) = false;
        state.confirm_count(state_index) = 0;
        state.release_count(state_index) = 0;
        weights(edge_index) = 1;
        detail.calibration_accepted(edge_index) = calibration_accepted;
        detail = local_write_detail_state(detail, edge_index, state, state_index, ...
            innovation_variance, cfg);
        state = local_mark_seen(state, state_index, current_time);
        continue;
    end

    [baseline_mean, effective_variance, baseline_ready] = ...
        local_baseline_reference(state, state_index, innovation_variance, cfg);
    centered_innovation = innovation - baseline_mean;
    baseline_normalized_innovation = centered_innovation / sqrt(effective_variance);
    [signed_normalized_innovation, consensus_available, consensus_neighbor_count, ...
        consensus_reference] = local_consensus_normalized_innovation( ...
        edges, edge_index, node_positions_prior, node_covariances_prior, ...
        state, baseline_normalized_innovation, cfg);
    innovation_change = local_innovation_change(state, state_index, innovation, effective_variance);
    [state, c_plus, c_minus, c_value, alarm_entered, alarm_cleared] = ...
        local_update_detector(state, state_index, signed_normalized_innovation, ...
        innovation_change, cfg);

    [state, may_adapt_baseline, freeze_event, unfreeze_event] = ...
        local_update_baseline_state(state, state_index, baseline_normalized_innovation, ...
        innovation_change, c_value, cfg);
    if baseline_ready && may_adapt_baseline
        state = local_adapt_baseline(state, state_index, innovation, cfg);
    end

    c_excess = max(0, c_value - cfg.cusum_weight_deadzone);
    % The scale contracts continuously as evidence approaches the alarm
    % threshold.  It preserves exact unit weight in the deadzone while
    % improving weak persistent-fault suppression without a label-driven rule.
    scale_contraction = 1 + cfg.cusum_alarm_weight_gain * ...
        min(1, c_value / cfg.cusum_alarm_on_threshold);
    effective_weight_scale = cfg.cusum_scale_h / scale_contraction;
    weight = 1 / (1 + (c_excess / effective_weight_scale)^2);
    weight = min(cfg.cusum_weight_max, max(cfg.cusum_weight_min, weight));
    weights(edge_index) = weight;
    state = local_mark_seen(state, state_index, current_time);

    detail.centered_innovation(edge_index) = centered_innovation;
    detail.effective_innovation_variance(edge_index) = effective_variance;
    detail.baseline_normalized_innovation(edge_index) = baseline_normalized_innovation;
    detail.signed_normalized_innovation(edge_index) = signed_normalized_innovation;
    detail.consensus_available(edge_index) = consensus_available;
    detail.consensus_neighbor_count(edge_index) = consensus_neighbor_count;
    detail.consensus_reference(edge_index) = consensus_reference;
    detail.innovation_change(edge_index) = innovation_change;
    detail.cusum_positive(edge_index) = c_plus;
    detail.cusum_negative(edge_index) = c_minus;
    detail.cusum_value(edge_index) = c_value;
    detail.cusum_excess(edge_index) = c_excess;
    detail.effective_weight_scale(edge_index) = effective_weight_scale;
    detail.final_weight(edge_index) = weight;
    detail.alarm_entered(edge_index) = alarm_entered;
    detail.alarm_cleared(edge_index) = alarm_cleared;
    detail.baseline_freeze_event(edge_index) = freeze_event;
    detail.baseline_unfreeze_event(edge_index) = unfreeze_event;
    detail = local_write_detail_state(detail, edge_index, state, state_index, ...
        innovation_variance, cfg);
end
end

function state = local_empty_state()
state.keys = zeros(0, 2);
state.cusum_positive = zeros(0, 1);
state.cusum_negative = zeros(0, 1);
state.last_seen_time = zeros(0, 1);
state.prev_active = false(0, 1);
state.was_reset = false(0, 1);
state.baseline_count = zeros(0, 1);
state.baseline_mean = zeros(0, 1);
state.baseline_m2 = zeros(0, 1);
state.baseline_variance = nan(0, 1);
state.baseline_last_innovation = nan(0, 1);
state.baseline_has_last_innovation = false(0, 1);
state.baseline_valid = false(0, 1);
state.calibrating = true(0, 1);
state.calibration_rejected_count = zeros(0, 1);
state.baseline_unverified = true(0, 1);
state.baseline_frozen = false(0, 1);
state.baseline_release_count = zeros(0, 1);
state.baseline_update_count = zeros(0, 1);
state.baseline_freeze_count = zeros(0, 1);
state.baseline_unfreeze_count = zeros(0, 1);
state.alarm_active = false(0, 1);
state.confirm_count = zeros(0, 1);
state.release_count = zeros(0, 1);
state.alarm_count = zeros(0, 1);
end

function state = local_normalize_state(state)
template = local_empty_state();
state_count = size(state.keys, 1);
fields = fieldnames(template);
for index = 1:numel(fields)
    field_name = fields{index};
    if ~isfield(state, field_name) || ...
            (~strcmp(field_name, 'keys') && numel(state.(field_name)) ~= state_count)
        state.(field_name) = local_state_default(field_name, state_count);
    end
end
end

function value = local_state_default(field_name, state_count)
switch field_name
    case 'keys'
        value = zeros(state_count, 2);
    case {'prev_active', 'was_reset', 'baseline_has_last_innovation', ...
            'baseline_valid', 'calibrating', 'baseline_unverified', ...
            'baseline_frozen', 'alarm_active'}
        value = false(state_count, 1);
    case {'baseline_variance', 'baseline_last_innovation'}
        value = nan(state_count, 1);
    otherwise
        value = zeros(state_count, 1);
end
end

function detail = local_empty_detail(edge_count)
detail.global_pairs = zeros(edge_count, 2);
detail.measurement = nan(edge_count, 1);
detail.innovation = nan(edge_count, 1);
detail.innovation_variance = nan(edge_count, 1);
detail.centered_innovation = nan(edge_count, 1);
detail.effective_innovation_variance = nan(edge_count, 1);
detail.baseline_normalized_innovation = nan(edge_count, 1);
detail.signed_normalized_innovation = nan(edge_count, 1);
detail.consensus_available = false(edge_count, 1);
detail.consensus_neighbor_count = zeros(edge_count, 1);
detail.consensus_reference = nan(edge_count, 1);
detail.innovation_change = nan(edge_count, 1);
detail.cusum_positive = zeros(edge_count, 1);
detail.cusum_negative = zeros(edge_count, 1);
detail.cusum_value = zeros(edge_count, 1);
detail.cusum_excess = zeros(edge_count, 1);
detail.effective_weight_scale = nan(edge_count, 1);
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
detail.zero_range_count = 0;
detail.nonfinite_fallback_count = 0;
detail.missing_decay_count = 0;
detail.reset_count = 0;
end

function local_validate_cfg(cfg)
required = {'cusum_lambda', 'cusum_kappa', 'cusum_scale_h', ...
    'cusum_weight_deadzone', 'cusum_alarm_weight_gain', 'cusum_weight_min', 'cusum_weight_max', ...
    'cusum_missing_decay', 'cusum_reset_after', 'cusum_eps', 'sigma_dis', ...
    'cusum_alarm_on_threshold', 'cusum_alarm_off_threshold', ...
    'cusum_alarm_confirm_epochs', 'cusum_alarm_release_epochs', ...
    'cusum_release_lambda', 'cusum_release_innovation_gate', ...
    'cusum_consensus_enable', 'cusum_consensus_min_neighbors', 'cusum_consensus_leader_only', ...
    'cusum_baseline_enable', 'cusum_edge_calibration_samples', ...
    'cusum_calibration_innovation_gate', 'cusum_baseline_adapt_gain', ...
    'cusum_baseline_innovation_gate', 'cusum_baseline_cusum_gate', ...
    'cusum_baseline_change_gate', 'cusum_baseline_release_cusum_gate', ...
    'cusum_baseline_release_innovation_gate', 'cusum_baseline_release_epochs', ...
    'cusum_baseline_variance_floor'};
for index = 1:numel(required)
    if ~isfield(cfg, required{index})
        error('compute_signed_cusum_edge_weights:MissingConfig', ...
            'Missing cfg.%s.', required{index});
    end
end
integer_fields = {'cusum_alarm_confirm_epochs', 'cusum_alarm_release_epochs', ...
    'cusum_edge_calibration_samples', 'cusum_baseline_release_epochs', ...
    'cusum_consensus_min_neighbors'};
for index = 1:numel(integer_fields)
    value = cfg.(integer_fields{index});
    if ~isscalar(value) || value < 1 || value ~= floor(value)
        error('compute_signed_cusum_edge_weights:InvalidConfig', ...
            'cfg.%s must be a positive integer.', integer_fields{index});
    end
end
if cfg.cusum_lambda < 0 || cfg.cusum_lambda > 1 || ...
        cfg.cusum_release_lambda < 0 || cfg.cusum_release_lambda >= 1 || ...
        cfg.cusum_missing_decay < 0 || cfg.cusum_missing_decay > 1 || ...
        cfg.cusum_kappa < 0 || cfg.cusum_scale_h <= 0 || ...
        cfg.cusum_weight_deadzone < 0 || cfg.cusum_alarm_weight_gain < 0 || cfg.sigma_dis <= 0 || ...
        cfg.cusum_weight_min <= 0 || cfg.cusum_weight_min > cfg.cusum_weight_max || ...
        cfg.cusum_weight_max > 1 || cfg.cusum_eps <= 0 || ...
        cfg.cusum_alarm_on_threshold <= cfg.cusum_alarm_off_threshold || ...
        cfg.cusum_release_innovation_gate <= 0 || ...
        ~isscalar(cfg.cusum_consensus_enable) || ...
        ~isscalar(cfg.cusum_consensus_leader_only) || ...
        cfg.cusum_calibration_innovation_gate <= 0 || ...
        cfg.cusum_baseline_adapt_gain < 0 || cfg.cusum_baseline_adapt_gain > 1 || ...
        cfg.cusum_baseline_innovation_gate <= 0 || cfg.cusum_baseline_cusum_gate < 0 || ...
        cfg.cusum_baseline_change_gate <= 0 || ...
        cfg.cusum_baseline_release_cusum_gate < 0 || ...
        cfg.cusum_baseline_release_innovation_gate <= 0 || ...
        cfg.cusum_baseline_variance_floor <= 0
    error('compute_signed_cusum_edge_weights:InvalidConfig', ...
        'CUSUM configuration is outside its valid range.');
end
end

function local_validate_prior_inputs(node_positions_prior, node_covariances_prior, edges)
if size(node_positions_prior, 1) ~= 3 || ~isreal(node_positions_prior) || ...
        any(~isfinite(node_positions_prior(:)))
    error('compute_signed_cusum_edge_weights:InvalidPriorPositions', ...
        'Prior node positions must be finite, real 3-D coordinates in one metric frame.');
end
node_count = size(node_positions_prior, 2);
if ~isequal(size(node_covariances_prior), [3, 3, node_count]) || ...
        ~isreal(node_covariances_prior) || any(~isfinite(node_covariances_prior(:)))
    error('compute_signed_cusum_edge_weights:InvalidPriorCovariances', ...
        'Prior covariances must be finite, real 3-by-3 matrices for every node.');
end
for node_index = 1:node_count
    covariance = node_covariances_prior(:, :, node_index);
    symmetry_tolerance = 1e-10 * max(1, norm(covariance, inf));
    if norm(covariance - covariance', inf) > symmetry_tolerance
        error('compute_signed_cusum_edge_weights:NonSymmetricPriorCovariance', ...
            'Prior covariance for node %d is not symmetric.', node_index);
    end
    eigenvalues = eig((covariance + covariance') / 2);
    if min(eigenvalues) < -symmetry_tolerance
        error('compute_signed_cusum_edge_weights:NonPsdPriorCovariance', ...
            'Prior covariance for node %d is not positive semidefinite.', node_index);
    end
end
for edge_index = 1:numel(edges)
    if edges(edge_index).global_i > node_count || edges(edge_index).global_j > node_count
        error('compute_signed_cusum_edge_weights:InvalidEdgeNode', ...
            'An edge refers to a node outside the supplied prior arrays.');
    end
end
end

function [global_i, global_j, measurement] = local_read_edge(edge)
required = {'global_i', 'global_j', 'measurement'};
for index = 1:numel(required)
    if ~isfield(edge, required{index})
        error('compute_signed_cusum_edge_weights:InvalidEdge', ...
            'Each edge must contain %s.', required{index});
    end
end
global_i = edge.global_i;
global_j = edge.global_j;
measurement = edge.measurement;
if ~isscalar(global_i) || ~isscalar(global_j) || global_i ~= floor(global_i) || ...
        global_j ~= floor(global_j) || global_i < 1 || global_j < 1 || global_i == global_j
    error('compute_signed_cusum_edge_weights:InvalidEdge', ...
        'Global edge identifiers must be distinct positive integers.');
end
end

function state_index = local_find_state(state, global_i, global_j)
state_index = find(state.keys(:, 1) == global_i & ...
    state.keys(:, 2) == global_j, 1, 'first');
end

function state = local_add_state(state, global_i, global_j, current_time)
state.keys(end + 1, :) = [global_i, global_j];
fields = fieldnames(state);
for index = 1:numel(fields)
    field_name = fields{index};
    if strcmp(field_name, 'keys')
        continue;
    end
    value = local_state_default(field_name, 1);
    state.(field_name)(end + 1, 1) = value;
end
state.last_seen_time(end, 1) = current_time;
state.calibrating(end, 1) = true;
state.baseline_unverified(end, 1) = true;
end

function [state, decay_count, reset_count] = local_decay_missing_edges( ...
        state, active_keys, cfg, current_time)
decay_count = 0;
reset_count = 0;
for state_index = 1:size(state.keys, 1)
    is_active = any(active_keys(:, 1) == state.keys(state_index, 1) & ...
        active_keys(:, 2) == state.keys(state_index, 2));
    if is_active
        continue;
    end
    missing_duration = current_time - state.last_seen_time(state_index);
    if missing_duration > cfg.cusum_reset_after
        if ~state.was_reset(state_index)
            state = local_reset_detector(state, state_index);
            % A previously valid baseline remains trusted across a link gap.
            % A never-calibrated edge remains in protected calibration.
            state.calibrating(state_index) = ~state.baseline_valid(state_index);
            state.was_reset(state_index) = true;
            reset_count = reset_count + 1;
        end
    else
        state.cusum_positive(state_index) = cfg.cusum_missing_decay * ...
            state.cusum_positive(state_index);
        state.cusum_negative(state_index) = cfg.cusum_missing_decay * ...
            state.cusum_negative(state_index);
        state.was_reset(state_index) = false;
        decay_count = decay_count + 1;
    end
    state.prev_active(state_index) = false;
end
end

function state = local_reset_detector(state, state_index)
state.cusum_positive(state_index) = 0;
state.cusum_negative(state_index) = 0;
state.alarm_active(state_index) = false;
state.confirm_count(state_index) = 0;
state.release_count(state_index) = 0;
state.baseline_frozen(state_index) = false;
state.baseline_release_count(state_index) = 0;
end

function state = local_mark_seen(state, state_index, current_time)
state.last_seen_time(state_index) = current_time;
state.prev_active(state_index) = true;
state.was_reset(state_index) = false;
end

function [state, accepted] = local_update_edge_calibration( ...
        state, state_index, innovation, innovation_variance, cfg)
raw_normalized_innovation = innovation / sqrt(innovation_variance);
innovation_change = local_innovation_change(state, state_index, innovation, innovation_variance);
accepted = abs(raw_normalized_innovation) <= cfg.cusum_calibration_innovation_gate && ...
    innovation_change <= cfg.cusum_baseline_change_gate;
if ~accepted
    state.calibration_rejected_count(state_index) = ...
        state.calibration_rejected_count(state_index) + 1;
    return;
end
count = state.baseline_count(state_index) + 1;
delta = innovation - state.baseline_mean(state_index);
mean_value = state.baseline_mean(state_index) + delta / count;
state.baseline_m2(state_index) = state.baseline_m2(state_index) + ...
    delta * (innovation - mean_value);
state.baseline_count(state_index) = count;
state.baseline_mean(state_index) = mean_value;
state.baseline_last_innovation(state_index) = innovation;
state.baseline_has_last_innovation(state_index) = true;
if count >= 2
    state.baseline_variance(state_index) = max(cfg.cusum_baseline_variance_floor, ...
        state.baseline_m2(state_index) / (count - 1));
end
if count >= cfg.cusum_edge_calibration_samples
    state.baseline_valid(state_index) = true;
    state.calibrating(state_index) = false;
    state.baseline_unverified(state_index) = false;
end
end

function [mean_value, variance_value, is_ready] = local_baseline_reference( ...
        state, state_index, innovation_variance, cfg)
mean_value = 0;
variance_value = innovation_variance;
is_ready = false;
if ~cfg.cusum_baseline_enable || ~state.baseline_valid(state_index)
    return;
end
mean_value = state.baseline_mean(state_index);
baseline_variance = state.baseline_variance(state_index);
if ~isfinite(baseline_variance)
    baseline_variance = cfg.cusum_baseline_variance_floor;
end
variance_value = max([innovation_variance, baseline_variance, ...
    cfg.cusum_baseline_variance_floor]);
is_ready = true;
end

function innovation_change = local_innovation_change(state, state_index, innovation, variance)
if ~state.baseline_has_last_innovation(state_index)
    innovation_change = 0;
else
    innovation_change = abs(innovation - state.baseline_last_innovation(state_index)) / sqrt(variance);
end
end

function [z_detector, available, neighbor_count, reference_value] = ...
        local_consensus_normalized_innovation(edges, edge_index, positions, covariances, ...
        state, z_local, cfg)
% Use only simultaneous, independently calibrated edges incident to the same
% low-precision node.  It is an online consistency statistic, not an oracle.
z_detector = z_local;
available = false;
neighbor_count = 0;
reference_value = NaN;
if ~cfg.cusum_consensus_enable
    return;
end
current_pair = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
low_num = cfg.uav_num - cfg.high_num;
shared_low_nodes = current_pair(current_pair <= low_num);
if cfg.cusum_consensus_leader_only && numel(shared_low_nodes) ~= 1
    % A follower-to-follower range is not an independent leader reference.
    return;
end
neighbor_values = zeros(0, 1);
for shared_node = shared_low_nodes
    for candidate_index = 1:numel(edges)
        if candidate_index == edge_index
            continue;
        end
        candidate_pair = sort([edges(candidate_index).global_i, edges(candidate_index).global_j]);
        if ~any(candidate_pair == shared_node)
            continue;
        end
        if cfg.cusum_consensus_leader_only
            other_node = candidate_pair(candidate_pair ~= shared_node);
            if numel(other_node) ~= 1 || other_node <= low_num
                continue;
            end
        end
        candidate_state_index = local_find_state(state, candidate_pair(1), candidate_pair(2));
        if isempty(candidate_state_index) || ~state.baseline_valid(candidate_state_index) || ...
                state.calibrating(candidate_state_index)
            continue;
        end
        measurement = edges(candidate_index).measurement;
        position_i = positions(:, candidate_pair(1));
        position_j = positions(:, candidate_pair(2));
        displacement = position_i - position_j;
        predicted_range = norm(displacement);
        if ~isfinite(measurement) || ~isfinite(predicted_range) || ...
                predicted_range <= cfg.cusum_eps
            continue;
        end
        unit_direction = displacement / predicted_range;
        covariance_i = covariances(:, :, candidate_pair(1));
        covariance_j = covariances(:, :, candidate_pair(2));
        innovation_variance = unit_direction' * covariance_i * unit_direction + ...
            unit_direction' * covariance_j * unit_direction + cfg.sigma_dis^2;
        if ~isfinite(innovation_variance) || innovation_variance <= cfg.cusum_eps
            continue;
        end
        [baseline_mean, effective_variance, baseline_ready] = ...
            local_baseline_reference(state, candidate_state_index, innovation_variance, cfg);
        if ~baseline_ready
            continue;
        end
        neighbor_values(end + 1, 1) = (measurement - predicted_range - baseline_mean) / ...
            sqrt(effective_variance); %#ok<AGROW>
    end
end
neighbor_count = numel(neighbor_values);
if neighbor_count < cfg.cusum_consensus_min_neighbors
    return;
end
reference_value = median(neighbor_values);
% The sqrt term keeps the differenced statistic on an approximately unit
% scale when each normalized innovation has comparable healthy variance.
z_detector = (z_local - reference_value) / sqrt(1 + 1 / neighbor_count);
available = true;
end

function [state, c_plus, c_minus, c_value, alarm_entered, alarm_cleared] = ...
        local_update_detector(state, state_index, z, innovation_change, cfg)
old_plus = state.cusum_positive(state_index);
old_minus = state.cusum_negative(state_index);
release_candidate = state.alarm_active(state_index) && ...
    abs(z) <= cfg.cusum_release_innovation_gate && ...
    innovation_change <= cfg.cusum_baseline_change_gate;
if release_candidate
    lambda_value = cfg.cusum_release_lambda;
else
    lambda_value = cfg.cusum_lambda;
end
c_plus = max(0, lambda_value * old_plus + z - cfg.cusum_kappa);
c_minus = max(0, lambda_value * old_minus - z - cfg.cusum_kappa);
c_value = max(c_plus, c_minus);
state.cusum_positive(state_index) = c_plus;
state.cusum_negative(state_index) = c_minus;
alarm_entered = false;
alarm_cleared = false;
if ~state.alarm_active(state_index)
    if c_value >= cfg.cusum_alarm_on_threshold
        state.confirm_count(state_index) = state.confirm_count(state_index) + 1;
    else
        state.confirm_count(state_index) = 0;
    end
    if state.confirm_count(state_index) >= cfg.cusum_alarm_confirm_epochs
        state.alarm_active(state_index) = true;
        state.alarm_count(state_index) = state.alarm_count(state_index) + 1;
        state.release_count(state_index) = 0;
        alarm_entered = true;
    end
else
    if release_candidate && c_value <= cfg.cusum_alarm_off_threshold
        state.release_count(state_index) = state.release_count(state_index) + 1;
    else
        state.release_count(state_index) = 0;
    end
    if state.release_count(state_index) >= cfg.cusum_alarm_release_epochs
        state.alarm_active(state_index) = false;
        state.confirm_count(state_index) = 0;
        state.release_count(state_index) = 0;
        alarm_cleared = true;
    end
end
end

function [state, may_adapt, freeze_event, unfreeze_event] = ...
        local_update_baseline_state(state, state_index, z, innovation_change, c_value, cfg)
may_adapt = false;
freeze_event = false;
unfreeze_event = false;
if ~cfg.cusum_baseline_enable || ~state.baseline_valid(state_index)
    return;
end
freeze_condition = c_value > cfg.cusum_baseline_cusum_gate || ...
    abs(z) > cfg.cusum_baseline_innovation_gate || ...
    innovation_change > cfg.cusum_baseline_change_gate || ...
    state.alarm_active(state_index);
if freeze_condition
    if ~state.baseline_frozen(state_index)
        state.baseline_frozen(state_index) = true;
        state.baseline_freeze_count(state_index) = state.baseline_freeze_count(state_index) + 1;
        freeze_event = true;
    end
    state.baseline_release_count(state_index) = 0;
    return;
end
if state.baseline_frozen(state_index)
    healthy_release = ~state.alarm_active(state_index) && ...
        c_value <= cfg.cusum_baseline_release_cusum_gate && ...
        abs(z) <= cfg.cusum_baseline_release_innovation_gate && ...
        innovation_change <= cfg.cusum_baseline_change_gate;
    if healthy_release
        state.baseline_release_count(state_index) = state.baseline_release_count(state_index) + 1;
    else
        state.baseline_release_count(state_index) = 0;
    end
    if state.baseline_release_count(state_index) < cfg.cusum_baseline_release_epochs
        return;
    end
    state.baseline_frozen(state_index) = false;
    state.baseline_release_count(state_index) = 0;
    state.baseline_unfreeze_count(state_index) = state.baseline_unfreeze_count(state_index) + 1;
    unfreeze_event = true;
end
may_adapt = true;
end

function state = local_adapt_baseline(state, state_index, innovation, cfg)
gain = cfg.cusum_baseline_adapt_gain;
if gain <= 0
    return;
end
old_mean = state.baseline_mean(state_index);
centered_innovation = innovation - old_mean;
state.baseline_mean(state_index) = old_mean + gain * centered_innovation;
old_variance = state.baseline_variance(state_index);
if ~isfinite(old_variance)
    old_variance = cfg.cusum_baseline_variance_floor;
end
state.baseline_variance(state_index) = max(cfg.cusum_baseline_variance_floor, ...
    (1 - gain) * old_variance + gain * centered_innovation^2);
state.baseline_count(state_index) = state.baseline_count(state_index) + 1;
state.baseline_last_innovation(state_index) = innovation;
state.baseline_has_last_innovation(state_index) = true;
state.baseline_update_count(state_index) = state.baseline_update_count(state_index) + 1;
end

function detail = local_write_detail_state(detail, edge_index, state, state_index, ...
        innovation_variance, cfg)
[baseline_mean, effective_variance, baseline_ready] = ...
    local_baseline_reference(state, state_index, innovation_variance, cfg);
detail.baseline_count(edge_index) = state.baseline_count(state_index);
detail.baseline_mean(edge_index) = baseline_mean;
detail.baseline_variance(edge_index) = effective_variance;
detail.baseline_ready(edge_index) = baseline_ready;
detail.baseline_unverified(edge_index) = state.baseline_unverified(state_index);
detail.calibrating(edge_index) = state.calibrating(state_index);
detail.calibration_rejected_count(edge_index) = state.calibration_rejected_count(state_index);
detail.baseline_frozen(edge_index) = state.baseline_frozen(state_index);
detail.baseline_update_count(edge_index) = state.baseline_update_count(state_index);
detail.baseline_freeze_count(edge_index) = state.baseline_freeze_count(state_index);
detail.baseline_unfreeze_count(edge_index) = state.baseline_unfreeze_count(state_index);
detail.alarm_active(edge_index) = state.alarm_active(state_index);
detail.confirm_count(edge_index) = state.confirm_count(state_index);
detail.release_count(edge_index) = state.release_count(state_index);
detail.alarm_count(edge_index) = state.alarm_count(state_index);
end
