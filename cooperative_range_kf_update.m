function [Xc, PK, Xerr, diagnostics] = cooperative_range_kf_update( ...
        posiN, Xc, PK, Xerr, follower_global_index, edges, ...
        frozen_node_positions_enu, frozen_node_covariances_enu, cfg, weights)
%COOPERATIVE_RANGE_KF_UPDATE Backward-compatible implementation of the EKF update.
%   The caller supplies one frozen, current-epoch network snapshot.  All
%   usable ranges incident to FOLLOWER_GLOBAL_INDEX are processed together,
%   so the result is independent of the input edge traversal order.  Range
%   weights only alter the effective range variance R_base / weight.

local_validate_inputs(posiN, Xc, PK, follower_global_index, edges, ...
    frozen_node_positions_enu, frozen_node_covariances_enu, cfg, weights);

state_to_enu = local_state_to_enu(posiN);
edge_count = numel(edges);
keys = zeros(edge_count, 2);
measurements = zeros(edge_count, 1);
edge_weights = zeros(edge_count, 1);
neighbors = zeros(edge_count, 1);
directions = zeros(3, edge_count);
residuals = zeros(edge_count, 1);
valid = false(edge_count, 1);

for edge_index = 1:edge_count
    global_i = edges(edge_index).global_i;
    global_j = edges(edge_index).global_j;
    if global_i ~= follower_global_index && global_j ~= follower_global_index
        continue;
    end
    if global_i == follower_global_index
        neighbor = global_j;
        sign_for_follower = 1;
    else
        neighbor = global_i;
        sign_for_follower = -1;
    end
    displacement = frozen_node_positions_enu(:, global_i) - ...
        frozen_node_positions_enu(:, global_j);
    predicted_range = norm(displacement);
    if predicted_range <= eps
        error('cooperative_range_kf_update:DegenerateRange', ...
            'A cooperative range edge has coincident frozen node positions.');
    end
    valid(edge_index) = true;
    keys(edge_index, :) = sort([global_i, global_j]);
    measurements(edge_index) = edges(edge_index).measurement;
    edge_weights(edge_index) = weights(edge_index);
    neighbors(edge_index) = neighbor;
    directions(:, edge_index) = sign_for_follower * displacement / predicted_range;
    residuals(edge_index) = predicted_range - measurements(edge_index);
end

keys = keys(valid, :);
measurements = measurements(valid);
edge_weights = edge_weights(valid);
neighbors = neighbors(valid);
directions = directions(:, valid);
residuals = residuals(valid);
if isempty(keys)
    diagnostics = local_empty_diagnostics();
    Xerr = local_kf_error_std(PK, posiN, Xerr);
    return;
end

[sorted_keys, order] = sortrows(keys, [1, 2]);
if size(unique(sorted_keys, 'rows'), 1) ~= size(sorted_keys, 1)
    error('cooperative_range_kf_update:DuplicateEdge', ...
        'Each undirected range edge may appear at most once per update.');
end
edge_weights = edge_weights(order);
neighbors = neighbors(order);
directions = directions(:, order);
residuals = residuals(order);

measurement_count = numel(residuals);
H = zeros(measurement_count, 18);
R = zeros(measurement_count, measurement_count);
for row = 1:measurement_count
    direction = directions(:, row);
    H(row, 7:9) = direction' * state_to_enu;
    neighbor_covariance = frozen_node_covariances_enu(:, :, neighbors(row));
    R(row, row) = cfg.sigma_dis^2 / edge_weights(row) + ...
        direction' * neighbor_covariance * direction;
end

innovation_covariance = H * PK * H' + R;
innovation_covariance = (innovation_covariance + innovation_covariance') / 2;
if any(~isfinite(innovation_covariance(:))) || rcond(innovation_covariance) <= eps
    error('cooperative_range_kf_update:InvalidInnovationCovariance', ...
        'The batch range innovation covariance is singular or non-finite.');
end

gain = (PK * H') / innovation_covariance;
innovation = residuals - H * Xc;
Xc = Xc + gain * innovation;
identity = eye(18);
PK = (identity - gain * H) * PK * (identity - gain * H)' + gain * R * gain';
PK = (PK + PK') / 2;
PK = local_apply_position_covariance_floor(PK, posiN, cfg);
if any(~isfinite(Xc)) || any(~isfinite(PK(:)))
    error('cooperative_range_kf_update:NonFiniteUpdate', ...
        'The cooperative range update produced a non-finite state or covariance.');
end

Xerr = local_kf_error_std(PK, posiN, Xerr);

diagnostics = local_empty_diagnostics();
diagnostics.global_pairs = sorted_keys;
diagnostics.measurement = measurements(order);
diagnostics.residual = residuals;
diagnostics.effective_variance = diag(R);
diagnostics.weights = edge_weights;
diagnostics.innovation_covariance = innovation_covariance;
end

function PK = local_apply_position_covariance_floor(PK, posiN, cfg)
% Keep the independent local EKF conservative after a cooperative batch.
% The follower-to-follower cross-covariances required by a fully joint EKF
% are not represented in this 18-state filter, so an ENU uncertainty floor
% avoids a spurious covariance collapse that would suppress later GPS fixes.
if ~isfield(cfg, 'ekf_range_position_std_floor') || isempty(cfg.ekf_range_position_std_floor)
    return;
end
floor_enu = reshape(cfg.ekf_range_position_std_floor, 3, 1);
if ~isreal(floor_enu) || any(~isfinite(floor_enu)) || any(floor_enu <= 0)
    error('cooperative_range_kf_update:InvalidPositionCovarianceFloor', ...
        'cfg.ekf_range_position_std_floor must be a positive finite [E;N;U] vector.');
end
earth_radius = 6378137.0;
flattening = 1 / 298.257;
latitude = posiN(2) * pi / 180;
height = posiN(3);
meridian_radius = earth_radius * (1 - 2 * flattening + 3 * flattening * sin(latitude)^2);
transverse_radius = earth_radius * (1 + flattening * sin(latitude)^2);
north_scale = meridian_radius + height;
east_scale = (transverse_radius + height) * cos(latitude);
state_floor = [(floor_enu(2) / north_scale)^2; ...
    (floor_enu(1) / east_scale)^2; floor_enu(3)^2];
for state_index = 7:9
    PK(state_index, state_index) = max(PK(state_index, state_index), ...
        state_floor(state_index - 6));
end
PK = (PK + PK') / 2;
end

function local_validate_inputs(posiN, Xc, PK, follower_global_index, edges, positions, covariances, cfg, weights)
if ~isnumeric(posiN) || ~isreal(posiN) || numel(posiN) ~= 3 || any(~isfinite(posiN(:)))
    error('cooperative_range_kf_update:InvalidPosition', 'posiN must be a finite three-element navigation position.');
end
if ~isnumeric(Xc) || ~isreal(Xc) || ~isequal(size(Xc), [18, 1]) || any(~isfinite(Xc))
    error('cooperative_range_kf_update:InvalidState', 'Xc must be a finite real 18-by-1 vector.');
end
if ~isnumeric(PK) || ~isreal(PK) || ~isequal(size(PK), [18, 18]) || any(~isfinite(PK(:))) || ...
        norm(PK - PK', 'fro') > 1e-8 * max(1, norm(PK, 'fro'))
    error('cooperative_range_kf_update:InvalidCovariance', 'PK must be a finite symmetric 18-by-18 covariance matrix.');
end
if ~isscalar(follower_global_index) || follower_global_index < 1 || follower_global_index ~= floor(follower_global_index)
    error('cooperative_range_kf_update:InvalidFollowerIndex', 'follower_global_index must be a positive integer.');
end
node_count = size(positions, 2);
if ~isnumeric(positions) || ~isreal(positions) || size(positions, 1) ~= 3 || ...
        any(~isfinite(positions(:))) || follower_global_index > node_count
    error('cooperative_range_kf_update:InvalidFrozenPositions', 'Frozen node positions must be finite 3-by-node_count ENU coordinates.');
end
if ~isnumeric(covariances) || ~isreal(covariances) || ...
        ~isequal(size(covariances), [3, 3, node_count]) || any(~isfinite(covariances(:)))
    error('cooperative_range_kf_update:InvalidFrozenCovariances', ...
        'Frozen node covariances must be finite 3-by-3-by-node_count matrices.');
end
for node = 1:node_count
    covariance = covariances(:, :, node);
    if norm(covariance - covariance', 'fro') > 1e-8 * max(1, norm(covariance, 'fro')) || ...
            min(eig((covariance + covariance') / 2)) < -1e-10
        error('cooperative_range_kf_update:InvalidFrozenCovariances', ...
            'Each frozen node covariance must be symmetric positive semidefinite.');
    end
end
if ~isstruct(edges) || ~all(isfield(edges, {'global_i', 'global_j', 'measurement'}))
    error('cooperative_range_kf_update:InvalidEdges', 'edges must contain global_i, global_j, and measurement fields.');
end
if ~isnumeric(weights) || ~isreal(weights) || numel(weights) ~= numel(edges) || ...
        any(~isfinite(weights(:))) || any(weights(:) <= 0)
    error('cooperative_range_kf_update:InvalidWeights', 'weights must be positive finite values aligned with edges.');
end
for index = 1:numel(edges)
    values = [edges(index).global_i, edges(index).global_j, edges(index).measurement];
    if any(~isfinite(values)) || edges(index).global_i ~= floor(edges(index).global_i) || ...
            edges(index).global_j ~= floor(edges(index).global_j) || ...
            edges(index).global_i < 1 || edges(index).global_j < 1 || ...
            edges(index).global_i > node_count || edges(index).global_j > node_count || ...
            edges(index).global_i == edges(index).global_j || edges(index).measurement <= 0
        error('cooperative_range_kf_update:InvalidEdges', 'Each edge must connect two valid nodes with a positive finite measurement.');
    end
end
if ~isstruct(cfg) || ~isfield(cfg, 'sigma_dis') || ~isscalar(cfg.sigma_dis) || ...
        ~isreal(cfg.sigma_dis) || ~isfinite(cfg.sigma_dis) || cfg.sigma_dis <= 0
    error('cooperative_range_kf_update:InvalidRangeStd', 'cfg.sigma_dis must be a positive finite scalar.');
end
end

function state_to_enu = local_state_to_enu(posiN)
earth_radius = 6378137.0;
flattening = 1 / 298.257;
latitude = posiN(2) * pi / 180;
height = posiN(3);
meridian_radius = earth_radius * (1 - 2 * flattening + 3 * flattening * sin(latitude)^2);
transverse_radius = earth_radius * (1 + flattening * sin(latitude)^2);
state_to_enu = [0, (transverse_radius + height) * cos(latitude), 0; ...
    (meridian_radius + height), 0, 0; 0, 0, 1];
end

function Xerr = local_kf_error_std(PK, posiN, Xerr)
if ~isnumeric(Xerr) || ~isreal(Xerr) || numel(Xerr) ~= 18
    Xerr = zeros(1, 18);
else
    Xerr = reshape(Xerr, 1, 18);
end
earth_radius = 6378137.0;
flattening = 1 / 298.257;
gravity = 9.7803698;
latitude = posiN(2) * pi / 180;
height = posiN(3);
meridian_radius = earth_radius * (1 - 2 * flattening + 3 * flattening * sin(latitude)^2);
transverse_radius = earth_radius * (1 + flattening * sin(latitude)^2);
standard_deviation = sqrt(max(real(diag(PK)), 0));
Xerr(1:3) = standard_deviation(1:3)' * 180 * 3600 / pi;
Xerr(4:6) = standard_deviation(4:6)';
Xerr(7) = standard_deviation(7) * (meridian_radius + height);
Xerr(8) = standard_deviation(8) * (transverse_radius + height) * cos(latitude);
Xerr(9) = standard_deviation(9);
Xerr(10:15) = standard_deviation(10:15)' * 180 * 3600 / pi;
Xerr(16:18) = standard_deviation(16:18)' / gravity;
end

function diagnostics = local_empty_diagnostics()
diagnostics = struct('global_pairs', zeros(0, 2), 'measurement', zeros(0, 1), ...
    'residual', zeros(0, 1), 'effective_variance', zeros(0, 1), ...
    'weights', zeros(0, 1), 'innovation_covariance', zeros(0, 0));
end
