function report = run_stage1_cusum_comparison(cfg)
%RUN_STAGE1_CUSUM_COMPARISON Fair Equal-FGO versus CUSUM-FGO comparison.
%   The two methods consume the same cached IMU, GPS, and range-noise inputs.
%   Their only algorithmic difference is the range-factor weight.

if nargin < 1 || isempty(cfg)
    cfg = stage1_cusum_default_config('full');
else
    cfg = local_merge_config(stage1_cusum_default_config('full'), cfg);
end
local_validate_experiment_config(cfg);

scenario_names = local_scenario_names(cfg);
scenario_count = numel(scenario_names);
seed_count = numel(cfg.seeds);
seed_results = repmat(struct('seed', [], 'equal', [], 'cusum', [], 'original_equal', []), seed_count, 1);
scenario_results = struct();
for scenario_index = 1:scenario_count
    scenario_results.(scenario_names{scenario_index}) = ...
        repmat(struct('seed', [], 'equal', [], 'cusum', [], 'original_equal', []), seed_count, 1);
end
total_runs = 2 * scenario_count * seed_count;
run_index = 0;
for seed_index = 1:seed_count
    seed = cfg.seeds(seed_index);
    cache = local_make_input_cache(cfg, seed);
    for scenario_index = 1:scenario_count
        scenario_name = scenario_names{scenario_index};
        scenario_cfg = cfg;
        scenario_cfg.fault_enable = strcmp(scenario_name, 'fault');

        equal_cfg = scenario_cfg;
        equal_cfg.cusum_apply = false;
        start_time = tic;
        equal_result = local_run_cached_simulation(cache, equal_cfg);
        run_index = run_index + 1;
        if cfg.verbose
            local_print_progress(run_index, total_runs, scenario_name, 'equal', seed, toc(start_time));
        end

        cusum_cfg = scenario_cfg;
        cusum_cfg.cusum_apply = true;
        start_time = tic;
        cusum_result = local_run_cached_simulation(cache, cusum_cfg);
        run_index = run_index + 1;
        if cfg.verbose
            local_print_progress(run_index, total_runs, scenario_name, 'cusum', seed, toc(start_time));
        end

        original_equal_result = [];
        if cfg.enable_original_baseline_regression
            original_equal_result = local_run_original_equal_cached(cache, scenario_cfg);
        end

        scenario_results.(scenario_name)(seed_index).seed = seed;
        scenario_results.(scenario_name)(seed_index).equal = equal_result;
        scenario_results.(scenario_name)(seed_index).cusum = cusum_result;
        scenario_results.(scenario_name)(seed_index).original_equal = original_equal_result;
    end

    % Preserve the original public result layout: with a configured fault,
    % seed_results is the fault scenario; otherwise it is the healthy one.
    primary_scenario = scenario_names{end};
    seed_results(seed_index) = scenario_results.(primary_scenario)(seed_index);
end

report.cfg = cfg;
report.seed_results = seed_results;
report.aggregate = local_aggregate(seed_results);
report.scenario_results = scenario_results;
report.scenario_aggregates = struct();
for scenario_index = 1:scenario_count
    scenario_name = scenario_names{scenario_index};
    report.scenario_aggregates.(scenario_name) = local_aggregate(scenario_results.(scenario_name));
end
if cfg.verbose
    local_print_comparison_tables(report, scenario_names);
end
end

function cache = local_make_input_cache(cfg, seed)
% Generate the exact random quantities once, in the original call order.
old_rng = rng;
cleanup = onCleanup(@() rng(old_rng)); %#ok<NASGU>
rng(seed, 'twister');

low_num = cfg.uav_num - cfg.high_num;
step_count = round(cfg.t_stop / cfg.dt);
graph_stride = round(cfg.graph_interval / cfg.dt);
graph_count = floor(step_count / graph_stride);
cache.seed = seed;
cache.step_count = step_count;
cache.graph_count = graph_count;
cache.graph_stride = graph_stride;
cache.imu_gyro_b = zeros(3, low_num, step_count + 1);
cache.imu_gyro_r = zeros(3, low_num, step_count + 1);
cache.imu_gyro_wg = zeros(3, low_num, step_count + 1);
cache.imu_acc_r = zeros(3, low_num, step_count + 1);
cache.gps_low_standard = zeros(3, low_num, graph_count);
cache.gps_high_standard = zeros(3, cfg.high_num, graph_count);
cache.range_standard = zeros(low_num, cfg.uav_num, graph_count);

gyro_b = zeros(3, low_num);
gyro_r = zeros(3, low_num);
gyro_wg = zeros(3, low_num);
acc_r = zeros(3, low_num);
for vehicle = 1:low_num
    [gyro_b(:, vehicle), gyro_r(:, vehicle), gyro_wg(:, vehicle), acc_r(:, vehicle)] = ...
        imu_err_random(0, cfg.dt, gyro_b(:, vehicle), gyro_r(:, vehicle), ...
        gyro_wg(:, vehicle), acc_r(:, vehicle));
end
cache.imu_gyro_b(:, :, 1) = gyro_b;
cache.imu_gyro_r(:, :, 1) = gyro_r;
cache.imu_gyro_wg(:, :, 1) = gyro_wg;
cache.imu_acc_r(:, :, 1) = acc_r;

graph_index = 0;
for step = 1:step_count
    current_time = step * cfg.dt;
    for vehicle = 1:low_num
        [gyro_b(:, vehicle), gyro_r(:, vehicle), gyro_wg(:, vehicle), acc_r(:, vehicle)] = ...
            imu_err_random(current_time, cfg.dt, gyro_b(:, vehicle), gyro_r(:, vehicle), ...
            gyro_wg(:, vehicle), acc_r(:, vehicle));
    end
    cache.imu_gyro_b(:, :, step + 1) = gyro_b;
    cache.imu_gyro_r(:, :, step + 1) = gyro_r;
    cache.imu_gyro_wg(:, :, step + 1) = gyro_wg;
    cache.imu_acc_r(:, :, step + 1) = acc_r;
    if mod(step, graph_stride) == 0
        graph_index = graph_index + 1;
        cache.gps_low_standard(:, :, graph_index) = randn(3, low_num);
        cache.gps_high_standard(:, :, graph_index) = randn(3, cfg.high_num);
        cache.range_standard(:, :, graph_index) = randn(low_num, cfg.uav_num);
    end
end
end

function result = local_run_cached_simulation(cache, cfg, is_original_equal_baseline)
if nargin < 3
    is_original_equal_baseline = false;
end
low_num = cfg.uav_num - cfg.high_num;
if cfg.assert_node_mapping
    assert(local_global_to_internal(3, cfg) == 1 && ...
        local_global_to_internal(4, cfg) == 2 && ...
        local_global_to_internal(5, cfg) == 3 && ...
        local_global_to_internal(1, cfg) == 4 && ...
        local_global_to_internal(2, cfg) == 5, ...
        'Stage-1 global-to-internal node mapping changed unexpectedly.');
end

posi_e_all = load('posi_e_all.dat');
posi_n_all = load('posi_n_all.dat');
posi_u_all = load('posi_u_all.dat');
posi_ini = [118; 32; 200.0];
posi_w_all = zeros(3, low_num);
posi_w_enu_all = zeros(3, low_num);
posi_L_all = zeros(3, cfg.high_num);
posi_L_enu_all = zeros(3, cfg.high_num);
for vehicle = 1:low_num
    posi_w_all(:, vehicle) = [posi_e_all(1, vehicle); posi_n_all(1, vehicle); posi_u_all(1, vehicle)];
    posi_w_enu_all(:, vehicle) = posical_enu(posi_w_all(:, vehicle), posi_ini);
end
for vehicle = 1:cfg.high_num
    source_index = low_num + vehicle;
    posi_L_all(:, vehicle) = [posi_e_all(1, source_index); posi_n_all(1, source_index); posi_u_all(1, source_index)];
    posi_L_enu_all(:, vehicle) = posical_enu(posi_L_all(:, vehicle), posi_ini);
end

veloB_all = zeros(3, cfg.uav_num);
velo_all = zeros(3, cfg.uav_num);
atti_all = zeros(3, cfg.uav_num);
atti_rate_all = zeros(3, cfg.uav_num);
acceB_all = zeros(3, cfg.uav_num);
for vehicle = 1:cfg.uav_num
    veloB_all(2, vehicle) = 5;
    atti_all(:, vehicle) = [0; 0; 90];
    velo_all(:, vehicle) = veloN0(atti_all(:, vehicle), veloB_all(:, vehicle));
end
attiN_all = atti_all(:, 1:low_num);
veloN_all = zeros(3, low_num);
for vehicle = 1:low_num
    veloN_all(:, vehicle) = veloN0(attiN_all(:, vehicle), veloB_all(:, vehicle));
end
posiN_w_all = posi_w_enu_all;
WnbbA_old = zeros(3, low_num);
gyro_modi_all = zeros(3, low_num);
acc_modi_all = zeros(3, low_num);
gyro_b = cache.imu_gyro_b(:, :, 1);
gyro_r = cache.imu_gyro_r(:, :, 1);
gyro_wg = cache.imu_gyro_wg(:, :, 1);
acc_r = cache.imu_acc_r(:, :, 1);

Xc_all = cell(1, low_num);
PK_all = cell(1, low_num);
Xerr_all = cell(1, low_num);
for vehicle = 1:low_num
    [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = ...
        kalm_factor_init(posi_w_enu_all(:, vehicle), zeros(18, 1), zeros(18, 18), zeros(1, 18));
end

sample_count = cache.step_count + 1;
truth_xyz = zeros(sample_count, 3, low_num);
navigation_xyz = zeros(sample_count, 3, low_num);
for vehicle = 1:low_num
    truth_xyz(1, :, vehicle) = posi_w_all(:, vehicle)';
    navigation_xyz(1, :, vehicle) = posi_w_all(:, vehicle)';
end
history = repmat(local_empty_history(), cache.graph_count, 1);
cusum_state = [];
graph_index = 0;
current_time = 0;

for step = 1:cache.step_count
    current_time = step * cfg.dt;
    old_veloB_all = veloB_all;
    old_atti_all = atti_all;
    [~, atti_all(:, 1), atti_rate_all(:, 1), veloB_all(:, 1), acceB_all(:, 1)] = ...
        trace(current_time - cfg.dt, cfg.dt, atti_all(:, 1), atti_rate_all(:, 1), veloB_all(:, 1), acceB_all(:, 1));
    velo_all(:, 1) = veloN0(atti_all(:, 1), veloB_all(:, 1));
    [Wibb, Fb] = IMUout(cfg.dt, posi_w_enu_all(:, 1), atti_all(:, 1), atti_rate_all(:, 1), ...
        veloB_all(:, 1), acceB_all(:, 1), old_veloB_all(:, 1), old_atti_all(:, 1));
    for vehicle = 1:(cfg.uav_num - 1)
        destination = vehicle + 1;
        atti_all(:, destination) = atti_all(:, 1);
        atti_rate_all(:, destination) = atti_rate_all(:, 1);
        veloB_all(:, destination) = veloB_all(:, 1);
        acceB_all(:, destination) = acceB_all(:, 1);
        velo_all(:, destination) = velo_all(:, 1);
    end
    for vehicle = 1:cfg.uav_num
        if vehicle <= low_num
            posi_w_enu_all(:, vehicle) = posi_out(cfg.dt, posi_w_enu_all(:, vehicle), veloB_all(:, vehicle), atti_all(:, vehicle));
        else
            high_index = vehicle - low_num;
            posi_L_enu_all(:, high_index) = posi_out(cfg.dt, posi_L_enu_all(:, high_index), veloB_all(:, vehicle), atti_all(:, vehicle));
        end
    end

    gyro_b = cache.imu_gyro_b(:, :, step + 1);
    gyro_r = cache.imu_gyro_r(:, :, step + 1);
    gyro_wg = cache.imu_gyro_wg(:, :, step + 1);
    acc_r = cache.imu_acc_r(:, :, step + 1);
    Fb_noise_all = zeros(3, low_num);
    for vehicle = 1:low_num
        Wibb_noise = Wibb + gyro_b(:, vehicle) / 0.01745329252 + ...
            gyro_r(:, vehicle) / 0.01745329252 + gyro_wg(:, vehicle) / 0.01745329252;
        Fb_noise = Fb + acc_r(:, vehicle);
        Fb_noise_all(:, vehicle) = Fb_noise;
        [attiN_all(:, vehicle), WnbbA_old(:, vehicle)] = atti_cal_cq_modi(cfg.dt, ...
            Wibb_noise - gyro_modi_all(:, vehicle) / 0.01745329252, attiN_all(:, vehicle), ...
            veloN_all(:, vehicle), posiN_w_all(:, vehicle), WnbbA_old(:, vehicle));
        veloN_all(:, vehicle) = velo_cal(cfg.dt, Fb_noise - acc_modi_all(:, vehicle), ...
            attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle));
        posiN_w_all(:, vehicle) = posi_cal(cfg.dt, veloN_all(:, vehicle), posiN_w_all(:, vehicle));
    end

    if mod(step, cache.graph_stride) == 0
        graph_index = graph_index + 1;
        [posiG_w_all, posiG_L] = local_cached_gps_measurements( ...
            posi_w_enu_all, posi_L_enu_all, cache, graph_index);
        for vehicle = 1:low_num
            [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = kalm_factor_time_update( ...
                current_time, cfg.graph_interval, Fb_noise_all(:, vehicle), attiN_all(:, vehicle), ...
                veloN_all(:, vehicle), posiN_w_all(:, vehicle), zeros(18, 1), ...
                PK_all{vehicle}, Xerr_all{vehicle});
        end

        [posi_w_all, posi_L_all, dis_true] = distance_cal( ...
            posi_w_enu_all, posi_L_enu_all, posi_ini, cfg.uav_num, cfg.high_num);
        [dis_measure, uav_link_num] = local_cached_pseudorange( ...
            dis_true, cache.range_standard(:, :, graph_index), cfg);
        dis_measure = local_inject_fault(dis_measure, current_time, cfg);

        posi_w_graph = zeros(3, low_num);
        posi_L_graph = zeros(3, cfg.high_num);
        for vehicle = 1:low_num
            posi_w_graph(:, vehicle) = posical_xyz(posiG_w_all(:, vehicle), posi_ini);
        end
        for vehicle = 1:cfg.high_num
            posi_L_graph(:, vehicle) = posical_xyz(posiG_L(:, vehicle), posi_ini);
        end
        % CUSUM uses the online KF/SINS prediction after its time update and
        % before this epoch's ranges are inserted into the factor graph.
        % Range factors themselves retain the original GPS graph priors.
        graph_positions_prior = [posi_w_graph, posi_L_graph];
        node_positions_prior = zeros(3, cfg.uav_num);
        node_covariances_prior = zeros(3, 3, cfg.uav_num);
        for vehicle = 1:low_num
            node_positions_prior(:, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini);
            node_covariances_prior(:, :, vehicle) = local_kf_position_covariance_enu( ...
                PK_all{vehicle}, posiN_w_all(:, vehicle));
        end
        for vehicle = (low_num + 1):cfg.uav_num
            node_positions_prior(:, vehicle) = posi_L_graph(:, vehicle - low_num);
            node_covariances_prior(:, :, vehicle) = diag([0.2, 0.2, 0.5].^2);
        end

        edges = local_build_active_edges(dis_measure, uav_link_num, cfg);
        if is_original_equal_baseline
            weights = ones(numel(edges), 1);
            detail = local_equal_weight_detail(edges);
        else
            [weights, detail, cusum_state] = compute_signed_cusum_edge_weights( ...
                edges, node_positions_prior, node_covariances_prior, cfg, cusum_state, current_time);
        end

        graph_1 = factor_graph_centralization(posi_L_graph, [0.2; 0.2; 0.5]);
        for vehicle = 1:low_num
            graph_1.para_add(posi_w_graph(:, vehicle), [10; 10; 20], cfg.high_num + vehicle);
        end
        for edge_index = 1:numel(edges)
            global_i = edges(edge_index).global_i;
            global_j = edges(edge_index).global_j;
            position_i = graph_positions_prior(:, global_i);
            position_j = graph_positions_prior(:, global_j);
            [residual, jaco] = residual_cal(position_i, position_j, edges(edge_index).measurement);
            scale = sqrt(weights(edge_index));
            graph_1.factor_add(scale * jaco, scale * residual, ...
                local_global_to_internal(global_i, cfg), local_global_to_internal(global_j, cfg), ...
                edges(edge_index).measurement, weights(edge_index));
        end
        graph_1.Gauss_Newton(posi_L_graph, posi_w_graph, [0.2; 0.2; 0.5], [10; 10; 20], cfg.high_num);
        graph_1.covariance();
        covariance = diag(graph_1.P_all);
        for vehicle = 1:low_num
            internal_index = local_global_to_internal(vehicle, cfg);
            posi_w_graph(:, vehicle) = graph_1.parameters((internal_index - 1) * 3 + 1:internal_index * 3);
            posiN_w_graph = posical_enu(posi_w_graph(:, vehicle), posi_ini);
            cov_graph = sqrt(covariance((internal_index - 1) * 3 + 1:internal_index * 3));
            [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = kalm_factor_measure_update( ...
                current_time, posiN_w_all(:, vehicle), posiN_w_graph, cov_graph, ...
                Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}, 1);
            [attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle)] = ...
                kalm_modi(attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle), Xc_all{vehicle});
            gyro_modi_all(:, vehicle) = Xc_all{vehicle}(10:12) + Xc_all{vehicle}(13:15);
            acc_modi_all(:, vehicle) = Xc_all{vehicle}(16:18);
        end
        history(graph_index).time = current_time;
        history(graph_index).pairs = detail.global_pairs;
        history(graph_index).detail = detail;
        history(graph_index).edge_count = numel(edges);
        history(graph_index).cusum_prior_positions = node_positions_prior;
        history(graph_index).cusum_prior_covariances = node_covariances_prior;
        history(graph_index).graph_prior_positions = graph_positions_prior;
        history(graph_index).graph_follower_positions = posi_w_graph;
        history(graph_index).graph_follower_covariances = local_extract_follower_covariances( ...
            graph_1.P_all, low_num, cfg);
        history(graph_index).gn_iteration_count = graph_1.iteration_count;
        history(graph_index).gn_final_step_norm = graph_1.final_step_norm;
        history(graph_index).gn_converged = graph_1.converged;
    end

    for vehicle = 1:low_num
        posi_w_all(:, vehicle) = posical_xyz(posi_w_enu_all(:, vehicle), posi_ini);
        truth_xyz(step + 1, :, vehicle) = posi_w_all(:, vehicle)';
        navigation_xyz(step + 1, :, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini)';
    end
end

result.seed = cache.seed;
result.time = (0:cache.step_count)' * cfg.dt;
result.truth_xyz = truth_xyz;
result.navigation_xyz = navigation_xyz;
result.error_xyz = navigation_xyz - truth_xyz;
result.history = history;
result.final_cusum_state = cusum_state;
result.metrics = local_metrics(result, cfg);
result.diagnostics = local_diagnostics(history, cfg);
result.gn_diagnostics = local_gn_diagnostics(history);
end

function [gps_low, gps_high] = local_cached_gps_measurements(posi_low, posi_high, cache, graph_index)
low_num = size(posi_low, 2);
high_num = size(posi_high, 2);
gps_low = zeros(size(posi_low));
gps_high = zeros(size(posi_high));
for vehicle = 1:low_num
    gps_low(:, vehicle) = local_gps_with_standard_noise(posi_low(:, vehicle), 10, 10, 20, ...
        cache.gps_low_standard(:, vehicle, graph_index));
end
for vehicle = 1:high_num
    gps_high(:, vehicle) = local_gps_with_standard_noise(posi_high(:, vehicle), 0.2, 0.2, 0.5, ...
        cache.gps_high_standard(:, vehicle, graph_index));
end
end

function gps = local_gps_with_standard_noise(posi, east_sigma, north_sigma, up_sigma, standard_noise)
Re = 6378137.0;
f = 1 / 298.257;
longitude = posi(1) * pi / 180;
latitude = posi(2) * pi / 180;
height = posi(3);
Rm = Re * (1 - 2 * f + 3 * f * sin(latitude)^2);
Rn = Re * (1 + f * sin(latitude)^2);
error = [east_sigma * standard_noise(1) / (Rn + height) / cos(latitude) / pi * 180; ...
    north_sigma * standard_noise(2) / (Rm + height) / pi * 180; ...
    up_sigma * standard_noise(3)];
gps = posi + error;
end

function [dis_measure, uav_link_num] = local_cached_pseudorange(dis_true, standard_noise, cfg)
low_num = cfg.uav_num - cfg.high_num;
dis_measure = dis_true + cfg.sigma_dis * standard_noise;
for source = 1:low_num
    dis_measure(source:low_num, source) = 0;
end
dis_measure(1:low_num, 1:low_num) = dis_measure(1:low_num, 1:low_num) + ...
    dis_measure(1:low_num, 1:low_num)';
uav_link_num = cell(1, low_num);
for source = 1:low_num
    for target = 1:cfg.uav_num
        if dis_true(source, target) <= cfg.communication_range && source ~= target
            uav_link_num{source} = [uav_link_num{source}, target]; %#ok<AGROW>
        end
    end
end
end

function dis_measure = local_inject_fault(dis_measure, current_time, cfg)
if ~cfg.fault_enable || current_time < cfg.fault_start || current_time > cfg.fault_end
    return;
end
fault_key = sort(cfg.fault_edge(:)');
low_num = cfg.uav_num - cfg.high_num;
if fault_key(1) <= low_num
    source = fault_key(1);
    target = fault_key(2);
    dis_measure(source, target) = dis_measure(source, target) + cfg.fault_bias;
elseif fault_key(2) <= low_num
    source = fault_key(2);
    target = fault_key(1);
    dis_measure(source, target) = dis_measure(source, target) + cfg.fault_bias;
end
end

function edges = local_build_active_edges(dis_measure, uav_link_num, cfg)
low_num = cfg.uav_num - cfg.high_num;
edges = repmat(struct('global_i', 0, 'global_j', 0, 'measurement', 0), 0, 1);
for source = 1:low_num
    for link_index = 1:numel(uav_link_num{source})
        target = uav_link_num{source}(link_index);
        if target > source
            edges(end + 1, 1).global_i = source; %#ok<AGROW>
            edges(end).global_j = target;
            edges(end).measurement = dis_measure(source, target);
        end
    end
end
end

function internal = local_global_to_internal(global_index, cfg)
low_num = cfg.uav_num - cfg.high_num;
if global_index <= low_num
    internal = cfg.high_num + global_index;
else
    internal = global_index - low_num;
end
end

function result = local_run_original_equal_cached(cache, cfg)
% Minimal original Equal-FGO path: it bypasses CUSUM entirely while retaining
% the same cached inputs and the corrected common navigation/GN mechanics.
cfg.cusum_apply = false;
result = local_run_cached_simulation(cache, cfg, true);
end

function covariance_enu = local_kf_position_covariance_enu(PK, posi)
% KF position states are [latitude(rad), longitude(rad), height(m)] at 7:9.
% posical_xyz uses [east, north, up], so longitude and latitude are reordered.
if ~isequal(size(PK), [18, 18]) || ~isreal(PK) || any(~isfinite(PK(:)))
    error('run_stage1_cusum_comparison:InvalidKfCovariance', ...
        'PK_all must be a finite real 18-by-18 KF covariance matrix.');
end
Re = 6378137.0;
f = 1 / 298.257;
latitude = posi(2) * pi / 180;
height = posi(3);
Rm = Re * (1 - 2 * f + 3 * f * sin(latitude)^2);
Rn = Re * (1 + f * sin(latitude)^2);
state_to_enu = [0, (Rn + height) * cos(latitude), 0; ...
    (Rm + height), 0, 0; ...
    0, 0, 1];
covariance_enu = state_to_enu * PK(7:9, 7:9) * state_to_enu';
covariance_enu = (covariance_enu + covariance_enu') / 2;
end

function detail = local_equal_weight_detail(edges)
edge_count = numel(edges);
detail.global_pairs = zeros(edge_count, 2);
detail.measurement = nan(edge_count, 1);
for edge_index = 1:edge_count
    detail.global_pairs(edge_index, :) = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
    detail.measurement(edge_index) = edges(edge_index).measurement;
end
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
detail.effective_weight_scale = nan(edge_count, 1);
detail.final_weight = ones(edge_count, 1);
detail.baseline_count = zeros(edge_count, 1);
detail.baseline_mean = nan(edge_count, 1);
detail.baseline_variance = nan(edge_count, 1);
detail.baseline_ready = false(edge_count, 1);
detail.baseline_unverified = false(edge_count, 1);
detail.calibrating = false(edge_count, 1);
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

function covariances = local_extract_follower_covariances(graph_covariance, low_num, cfg)
covariances = zeros(3, 3, low_num);
for vehicle = 1:low_num
    internal_index = local_global_to_internal(vehicle, cfg);
    rows = (internal_index - 1) * 3 + 1:internal_index * 3;
    covariances(:, :, vehicle) = graph_covariance(rows, rows);
end
end

function diagnostics = local_gn_diagnostics(history)
iterations = [history.gn_iteration_count]';
step_norms = [history.gn_final_step_norm]';
converged = [history.gn_converged]';
valid = isfinite(step_norms);
diagnostics.mean_iterations = mean(iterations(valid));
diagnostics.max_iterations = max(iterations(valid));
diagnostics.nonconverged_count = sum(~converged(valid));
diagnostics.final_step_norm = max(step_norms(valid));
end

function history = local_empty_history()
history.time = NaN;
history.pairs = zeros(0, 2);
history.detail = struct();
history.edge_count = 0;
history.cusum_prior_positions = zeros(3, 0);
history.cusum_prior_covariances = zeros(3, 3, 0);
history.graph_prior_positions = zeros(3, 0);
history.graph_follower_positions = zeros(3, 0);
history.graph_follower_covariances = zeros(3, 3, 0);
history.gn_iteration_count = NaN;
history.gn_final_step_norm = NaN;
history.gn_converged = false;
end

function metrics = local_metrics(result, cfg)
low_num = cfg.uav_num - cfg.high_num;
window_mask = result.time >= cfg.fault_start & result.time <= cfg.fault_end;
if ~any(window_mask)
    window_mask = false(size(result.time));
end
metrics.full_rmse_3d = zeros(low_num, 1);
metrics.window_rmse_3d = nan(low_num, 1);
for vehicle = 1:low_num
    squared_error = sum(result.error_xyz(:, :, vehicle).^2, 2);
    metrics.full_rmse_3d(vehicle) = sqrt(mean(squared_error));
    if any(window_mask)
        metrics.window_rmse_3d(vehicle) = sqrt(mean(squared_error(window_mask)));
    end
end
end

function diagnostics = local_diagnostics(history, cfg)
diagnostics = struct('healthy_signed_innovation_mean', NaN, ...
    'healthy_signed_innovation_median', NaN, 'healthy_weight_median', NaN, ...
    'healthy_weight_below_095_fraction', NaN, 'healthy_weight_below_08_fraction', NaN, ...
    'max_cusum', 0, 'nonfinite_fallback_count', 0, 'missing_decay_count', 0, ...
    'reset_count', 0, 'target_cplus_median', NaN, 'target_cminus_median', NaN, ...
    'target_cusum_median', NaN, 'target_weight_median', NaN, ...
    'other_weight_median', NaN, 'target_to_other_weight_ratio', NaN, ...
    'target_weight_median_fault_window', NaN, ...
    'other_weight_median_fault_window', NaN, ...
    'target_to_other_weight_ratio_fault_window', NaN, ...
    'other_weight_median_full', NaN, ...
    'target_signed_innovation_mean_fault_window', NaN, ...
    'target_signed_innovation_median_fault_window', NaN, ...
    'other_signed_innovation_mean_fault_window', NaN, ...
    'other_signed_innovation_median_fault_window', NaN, ...
    'detect_095_delay', NaN, 'detect_08_delay', NaN, 'recovery_095_delay', NaN, ...
    'alarm_delay', NaN, 'weight_drop_095_delay', NaN, 'weight_drop_08_delay', NaN, ...
    'alarm_recovery_delay', NaN, 'health_alarm_count', 0, ...
    'baseline_freeze_event_count', 0, 'baseline_unfreeze_event_count', 0, ...
    'baseline_update_event_count', 0, 'unverified_edge_count', 0, 'edge', struct([]));
pairs = zeros(0, 2); times = zeros(0, 1); innovations = zeros(0, 1);
weights = zeros(0, 1); cplus = zeros(0, 1); cminus = zeros(0, 1); cvalues = zeros(0, 1);
alarm_active = false(0, 1); alarm_entered = false(0, 1); alarm_cleared = false(0, 1);
freeze_events = false(0, 1); unfreeze_events = false(0, 1);
baseline_updates = zeros(0, 1); unverified = false(0, 1); calibrating = false(0, 1);
for index = 1:numel(history)
    if isempty(history(index).pairs)
        continue;
    end
    count = size(history(index).pairs, 1);
    pairs = [pairs; history(index).pairs]; %#ok<AGROW>
    times = [times; repmat(history(index).time, count, 1)]; %#ok<AGROW>
    innovations = [innovations; history(index).detail.signed_normalized_innovation]; %#ok<AGROW>
    weights = [weights; history(index).detail.final_weight]; %#ok<AGROW>
    cplus = [cplus; history(index).detail.cusum_positive]; %#ok<AGROW>
    cminus = [cminus; history(index).detail.cusum_negative]; %#ok<AGROW>
    cvalues = [cvalues; history(index).detail.cusum_value]; %#ok<AGROW>
    alarm_active = [alarm_active; history(index).detail.alarm_active]; %#ok<AGROW>
    alarm_entered = [alarm_entered; history(index).detail.alarm_entered]; %#ok<AGROW>
    alarm_cleared = [alarm_cleared; history(index).detail.alarm_cleared]; %#ok<AGROW>
    freeze_events = [freeze_events; history(index).detail.baseline_freeze_event]; %#ok<AGROW>
    unfreeze_events = [unfreeze_events; history(index).detail.baseline_unfreeze_event]; %#ok<AGROW>
    baseline_updates = [baseline_updates; history(index).detail.baseline_update_count]; %#ok<AGROW>
    unverified = [unverified; history(index).detail.baseline_unverified]; %#ok<AGROW>
    calibrating = [calibrating; history(index).detail.calibrating]; %#ok<AGROW>
    diagnostics.nonfinite_fallback_count = diagnostics.nonfinite_fallback_count + history(index).detail.nonfinite_fallback_count;
    diagnostics.missing_decay_count = diagnostics.missing_decay_count + history(index).detail.missing_decay_count;
    diagnostics.reset_count = diagnostics.reset_count + history(index).detail.reset_count;
end
finite_innovation = isfinite(innovations);
if any(finite_innovation)
    diagnostics.healthy_signed_innovation_mean = mean(innovations(finite_innovation));
    diagnostics.healthy_signed_innovation_median = median(innovations(finite_innovation));
end
if ~isempty(weights)
    diagnostics.healthy_weight_median = median(weights);
    diagnostics.healthy_weight_below_095_fraction = mean(weights < 0.95);
    diagnostics.healthy_weight_below_08_fraction = mean(weights < 0.8);
    diagnostics.max_cusum = max(cvalues);
end
diagnostics.health_alarm_count = sum(alarm_entered);
diagnostics.baseline_freeze_event_count = sum(freeze_events);
diagnostics.baseline_unfreeze_event_count = sum(unfreeze_events);
diagnostics.baseline_update_event_count = sum(diff([0; baseline_updates]) > 0);
diagnostics.unverified_edge_count = size(unique(pairs(unverified | calibrating, :), 'rows'), 1);
diagnostics.edge = local_per_edge_diagnostics(pairs, innovations, weights, cvalues, ...
    alarm_active, alarm_entered, freeze_events, unfreeze_events, baseline_updates, unverified);
if ~cfg.fault_enable || isempty(pairs)
    return;
end
    target = sort(cfg.fault_edge(:)');
    target_mask = pairs(:, 1) == target(1) & pairs(:, 2) == target(2);
    fault_window = times >= cfg.fault_start & times <= cfg.fault_end;
    target_window = target_mask & fault_window;
    other_window = ~target_mask & fault_window;
    other_full = ~target_mask;
    if any(target_window)
        diagnostics.target_cplus_median = median(cplus(target_window));
        diagnostics.target_cminus_median = median(cminus(target_window));
        diagnostics.target_cusum_median = median(cvalues(target_window));
        diagnostics.target_weight_median_fault_window = median(weights(target_window));
        diagnostics.target_weight_median = diagnostics.target_weight_median_fault_window;
        target_innovation = innovations(target_window & isfinite(innovations));
        if ~isempty(target_innovation)
            diagnostics.target_signed_innovation_mean_fault_window = mean(target_innovation);
            diagnostics.target_signed_innovation_median_fault_window = median(target_innovation);
        end
    end
    if any(other_window)
        diagnostics.other_weight_median_fault_window = median(weights(other_window));
        diagnostics.other_weight_median = diagnostics.other_weight_median_fault_window;
        other_innovation = innovations(other_window & isfinite(innovations));
        if ~isempty(other_innovation)
            diagnostics.other_signed_innovation_mean_fault_window = mean(other_innovation);
            diagnostics.other_signed_innovation_median_fault_window = median(other_innovation);
        end
    end
    if any(other_full)
        diagnostics.other_weight_median_full = median(weights(other_full));
    end
    if isfinite(diagnostics.target_weight_median_fault_window) && ...
            isfinite(diagnostics.other_weight_median_fault_window) && ...
            diagnostics.other_weight_median_fault_window > 0
        diagnostics.target_to_other_weight_ratio_fault_window = ...
            diagnostics.target_weight_median_fault_window / diagnostics.other_weight_median_fault_window;
        diagnostics.target_to_other_weight_ratio = diagnostics.target_to_other_weight_ratio_fault_window;
    end
diagnostics.detect_095_delay = local_first_threshold_crossing( ...
    times, target_mask, weights, 0.95, cfg.fault_start, cfg.fault_end, false);
diagnostics.detect_08_delay = local_first_threshold_crossing( ...
    times, target_mask, weights, 0.8, cfg.fault_start, cfg.fault_end, false);
diagnostics.recovery_095_delay = local_first_threshold_crossing( ...
    times, target_mask, weights, 0.95, cfg.fault_end, Inf, true);
diagnostics.weight_drop_095_delay = diagnostics.detect_095_delay;
diagnostics.weight_drop_08_delay = diagnostics.detect_08_delay;
diagnostics.alarm_delay = local_first_event_delay(times, target_mask, alarm_entered, ...
    cfg.fault_start, cfg.fault_end);
diagnostics.alarm_recovery_delay = local_first_event_delay(times, target_mask, alarm_cleared, ...
    cfg.fault_end, Inf);
end

function edge_diagnostics = local_per_edge_diagnostics(pairs, innovations, weights, cvalues, ...
        alarm_active, alarm_entered, freeze_events, unfreeze_events, baseline_updates, unverified)
template = struct('pair', [NaN, NaN], 'innovation_mean', NaN, ...
    'innovation_median', NaN, 'innovation_std', NaN, 'max_positive_run', 0, ...
    'max_negative_run', 0, 'weight_median', NaN, 'weight_below_095_fraction', NaN, ...
    'weight_below_08_fraction', NaN, 'max_cusum', NaN, 'alarm_count', 0, ...
    'alarm_active_fraction', NaN, 'freeze_count', 0, 'unfreeze_count', 0, ...
    'baseline_update_count', 0, 'baseline_unverified', false);
if isempty(pairs)
    edge_diagnostics = repmat(template, 0, 1);
    return;
end
unique_pairs = unique(pairs, 'rows');
edge_diagnostics = repmat(template, size(unique_pairs, 1), 1);
for edge_index = 1:size(unique_pairs, 1)
    mask = pairs(:, 1) == unique_pairs(edge_index, 1) & ...
        pairs(:, 2) == unique_pairs(edge_index, 2);
    values = innovations(mask & isfinite(innovations));
    edge_diagnostics(edge_index).pair = unique_pairs(edge_index, :);
    if ~isempty(values)
        edge_diagnostics(edge_index).innovation_mean = mean(values);
        edge_diagnostics(edge_index).innovation_median = median(values);
        edge_diagnostics(edge_index).innovation_std = std(values);
        [edge_diagnostics(edge_index).max_positive_run, ...
            edge_diagnostics(edge_index).max_negative_run] = local_max_sign_runs(values);
    end
    edge_weights = weights(mask);
    edge_diagnostics(edge_index).weight_median = median(edge_weights);
    edge_diagnostics(edge_index).weight_below_095_fraction = mean(edge_weights < 0.95);
    edge_diagnostics(edge_index).weight_below_08_fraction = mean(edge_weights < 0.8);
    edge_diagnostics(edge_index).max_cusum = max(cvalues(mask));
    edge_diagnostics(edge_index).alarm_count = sum(alarm_entered(mask));
    edge_diagnostics(edge_index).alarm_active_fraction = mean(alarm_active(mask));
    edge_diagnostics(edge_index).freeze_count = sum(freeze_events(mask));
    edge_diagnostics(edge_index).unfreeze_count = sum(unfreeze_events(mask));
    edge_diagnostics(edge_index).baseline_update_count = max(baseline_updates(mask));
    edge_diagnostics(edge_index).baseline_unverified = any(unverified(mask));
end
end

function [positive_run, negative_run] = local_max_sign_runs(values)
positive_run = 0;
negative_run = 0;
current_positive = 0;
current_negative = 0;
for index = 1:numel(values)
    if values(index) > 0
        current_positive = current_positive + 1;
        current_negative = 0;
    elseif values(index) < 0
        current_negative = current_negative + 1;
        current_positive = 0;
    else
        current_positive = 0;
        current_negative = 0;
    end
    positive_run = max(positive_run, current_positive);
    negative_run = max(negative_run, current_negative);
end
end

function delay = local_first_event_delay(times, target_mask, event_mask, start_time, end_time)
indices = find(target_mask & event_mask & times >= start_time & times <= end_time);
if isempty(indices)
    delay = NaN;
else
    delay = times(indices(1)) - start_time;
end
end

function delay = local_first_threshold_crossing(times, target_mask, weights, threshold, start_time, end_time, upward)
% Report an actual post-start threshold crossing, not a pre-existing alarm.
candidate_indices = find(target_mask & times >= start_time & times <= end_time);
delay = NaN;
for candidate = candidate_indices'
    previous_indices = find(target_mask & times < times(candidate), 1, 'last');
    if isempty(previous_indices)
        previous_weight = 1;
    else
        previous_weight = weights(previous_indices);
    end
    if (~upward && previous_weight >= threshold && weights(candidate) < threshold) || ...
            (upward && previous_weight <= threshold && weights(candidate) > threshold)
        delay = times(candidate) - start_time;
        return;
    end
end
end

function names = local_scenario_names(cfg)
if cfg.fault_enable
    names = {'healthy', 'fault'};
else
    names = {'healthy'};
end
end

function local_print_progress(run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds)
fprintf('[%2d/%d ] scenario=%-7s method=%-5s seed=%-6g ... done (%.2f s)\n', ...
    run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds);
end

function local_print_comparison_tables(report, scenario_names)
cfg = report.cfg;
fprintf('\nStage 1 CUSUM comparison\n');
fprintf('  Mode                : %s\n', local_mode_label(cfg));
fprintf('  Seeds               : %s\n', local_seed_text(cfg.seeds));
if cfg.fault_enable
    fault_key = sort(cfg.fault_edge(:)');
    fprintf('  Fault configuration : edge (%d,%d), %.3f-%.3f s, bias %+.3f m\n', ...
        fault_key(1), fault_key(2), cfg.fault_start, cfg.fault_end, cfg.fault_bias);
else
    fprintf('  Fault configuration : disabled\n');
end
fprintf('  CUSUM weights       : applied\n');
if cfg.cusum_baseline_enable
    fprintf('  Nominal calibration : enabled (warm-up + abrupt-change lock)\n');
else
    fprintf('  Nominal calibration : disabled\n');
end
fprintf('  Output              : disabled (in-memory results only)\n');

fprintf('\nPer-follower positioning summary (averaged across seeds only)\n');
fprintf('  %-8s %-7s %-10s %14s %22s %14s %16s\n', ...
    'Scenario', 'Method', 'Follower', 'Full RMSE(m)', 'Fault-window RMSE(m)', ...
    'Full vs Equal', 'Window vs Equal');
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    aggregate = report.scenario_aggregates.(scenario_name);
    local_print_summary_rows(scenario_name, 'equal', aggregate.equal_full_mean, ...
        aggregate.equal_window_mean, aggregate.equal_full_mean, aggregate.equal_window_mean);
    local_print_summary_rows(scenario_name, 'cusum', aggregate.cusum_full_mean, ...
        aggregate.cusum_window_mean, aggregate.equal_full_mean, aggregate.equal_window_mean);
end

if numel(cfg.seeds) > 1
    fprintf('\nPer-follower RMSE standard deviation across seeds\n');
    fprintf('  %-8s %-7s %-10s %20s %28s\n', ...
        'Scenario', 'Method', 'Follower', 'Full RMSE Std(m)', 'Fault-window RMSE Std(m)');
    for scenario_index = 1:numel(scenario_names)
        scenario_name = scenario_names{scenario_index};
        aggregate = report.scenario_aggregates.(scenario_name);
        local_print_std_rows(scenario_name, 'equal', aggregate.equal_full_std, aggregate.equal_window_std);
        local_print_std_rows(scenario_name, 'cusum', aggregate.cusum_full_std, aggregate.cusum_window_std);
    end
    fprintf('\nWorst-seed CUSUM-FGO full RMSE\n');
    fprintf('  %-8s %-10s %18s %10s\n', 'Scenario', 'Follower', 'Worst RMSE(m)', 'Seed');
    for scenario_index = 1:numel(scenario_names)
        scenario_name = scenario_names{scenario_index};
        aggregate = report.scenario_aggregates.(scenario_name);
        for vehicle = 1:numel(aggregate.cusum_full_worst)
            fprintf('  %-8s %-10s %18.6f %10g\n', scenario_name, ...
                sprintf('Follower%d', vehicle), aggregate.cusum_full_worst(vehicle), ...
                aggregate.cusum_full_worst_seed(vehicle));
        end
    end
end

local_print_weight_diagnostics(report, scenario_names);
local_print_gn_diagnostics(report, scenario_names);
end

function local_print_summary_rows(scenario_name, method_name, full_values, window_values, equal_full, equal_window)
for vehicle = 1:numel(full_values)
    if strcmp(method_name, 'equal')
        full_change = 0;
        window_change = 0;
    else
        full_change = local_percent_change(full_values(vehicle), equal_full(vehicle));
        window_change = local_percent_change(window_values(vehicle), equal_window(vehicle));
    end
    fprintf('  %-8s %-7s %-10s %14.6f %22.6f %14s %16s\n', ...
        scenario_name, method_name, sprintf('Follower%d', vehicle), ...
        full_values(vehicle), window_values(vehicle), ...
        local_percent_text(full_change), local_percent_text(window_change));
end
end

function local_print_std_rows(scenario_name, method_name, full_std, window_std)
for vehicle = 1:numel(full_std)
    fprintf('  %-8s %-7s %-10s %20.6f %28.6f\n', ...
        scenario_name, method_name, sprintf('Follower%d', vehicle), ...
        full_std(vehicle), window_std(vehicle));
end
end

function local_print_weight_diagnostics(report, scenario_names)
fprintf('\nCUSUM weight diagnostics\n');
fprintf('  %-8s %-6s %8s %10s %14s %8s\n', ...
    'Scenario', 'Seed', 'Max C', 'Fallback', 'Missing decay', 'Reset');
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    results = report.scenario_results.(scenario_name);
    for seed_index = 1:numel(results)
        diagnostics = results(seed_index).cusum.diagnostics;
        fprintf('  %-8s %-6g %8.3f %10d %14d %8d\n', ...
            scenario_name, results(seed_index).seed, diagnostics.max_cusum, ...
            diagnostics.nonfinite_fallback_count, diagnostics.missing_decay_count, diagnostics.reset_count);
    end
end

if isfield(report.scenario_results, 'fault')
    results = report.scenario_results.fault;
    fprintf('\nCUSUM fault-edge diagnostics\n');
    fprintf('  %-6s %12s %14s %13s %12s %13s %12s %15s %15s\n', ...
        'Seed', 'Fault weight', 'Healthy wt(window)', 'Fault/Healthy', ...
        'Alarm(s)', 'Drop .95(s)', 'Drop .8(s)', 'Alarm clear(s)', 'Weight rec(s)');
    for seed_index = 1:numel(results)
        diagnostics = results(seed_index).cusum.diagnostics;
        fprintf('  %-6g %12.6f %14.6f %13.6f %12s %13s %12s %15s %15s\n', ...
            results(seed_index).seed, diagnostics.target_weight_median_fault_window, ...
            diagnostics.other_weight_median_fault_window, ...
            diagnostics.target_to_other_weight_ratio_fault_window, ...
            local_number_or_label(diagnostics.alarm_delay, 'not alarmed'), ...
            local_number_or_label(diagnostics.weight_drop_095_delay, 'not dropped'), ...
            local_number_or_label(diagnostics.weight_drop_08_delay, 'not dropped'), ...
            local_number_or_label(diagnostics.alarm_recovery_delay, 'not cleared'), ...
            local_number_or_label(diagnostics.recovery_095_delay, 'not recovered'));
    end
end

if isfield(report.scenario_results, 'healthy')
    results = report.scenario_results.healthy;
    fprintf('\nCUSUM healthy-edge diagnostics\n');
    fprintf('  %-6s %14s %15s %14s %8s %9s %9s\n', ...
        'Seed', 'Median weight', 'Weight<0.95', 'Weight<0.8', 'Max C', 'Alarms', 'Freezes');
    for seed_index = 1:numel(results)
        diagnostics = results(seed_index).cusum.diagnostics;
        fprintf('  %-6g %14.6f %14.2f%% %13.2f%% %8.3f %9d %9d\n', ...
            results(seed_index).seed, diagnostics.healthy_weight_median, ...
            100 * diagnostics.healthy_weight_below_095_fraction, ...
            100 * diagnostics.healthy_weight_below_08_fraction, diagnostics.max_cusum, ...
            diagnostics.health_alarm_count, diagnostics.baseline_freeze_event_count);
    end
end
local_print_per_edge_diagnostics(report, scenario_names);
fprintf('\n');
end

function local_print_per_edge_diagnostics(report, scenario_names)
fprintf('\nCUSUM per-edge diagnostics\n');
fprintf('  %-8s %-6s %-6s %9s %9s %6s %6s %8s %10s %10s %10s %9s %7s %7s %8s\n', ...
    'Scenario', 'Seed', 'Edge', 'Innov mean', 'Innov std', 'Run+', 'Run-', 'Max C', ...
    'Median wt', 'Wt<.95', 'Wt<.8', 'Alarms', 'Freeze', 'Unfreeze', 'Updates');
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    results = report.scenario_results.(scenario_name);
    for seed_index = 1:numel(results)
        edge_diagnostics = results(seed_index).cusum.diagnostics.edge;
        for edge_index = 1:numel(edge_diagnostics)
            edge = edge_diagnostics(edge_index);
            fprintf('  %-8s %-6g %d-%-4d %9.3f %9.3f %6d %6d %8.3f %10.6f %9.2f%% %9.2f%% %9d %7d %7d %8d\n', ...
                scenario_name, results(seed_index).seed, edge.pair(1), edge.pair(2), ...
                edge.innovation_mean, edge.innovation_std, edge.max_positive_run, ...
                edge.max_negative_run, edge.max_cusum, ...
                edge.weight_median, 100 * edge.weight_below_095_fraction, ...
                100 * edge.weight_below_08_fraction, edge.alarm_count, ...
                edge.freeze_count, edge.unfreeze_count, edge.baseline_update_count);
        end
    end
end
end

function local_print_gn_diagnostics(report, scenario_names)
fprintf('\nGN convergence diagnostics\n');
fprintf('  %-8s %-7s %-6s %15s %14s %14s %17s\n', ...
    'Scenario', 'Method', 'Seed', 'Mean iterations', 'Max iterations', ...
    'Nonconverged', 'Final step norm');
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    results = report.scenario_results.(scenario_name);
    for seed_index = 1:numel(results)
        equal_gn = results(seed_index).equal.gn_diagnostics;
        cusum_gn = results(seed_index).cusum.gn_diagnostics;
        local_print_gn_row(scenario_name, 'equal', results(seed_index).seed, equal_gn);
        local_print_gn_row(scenario_name, 'cusum', results(seed_index).seed, cusum_gn);
    end
end
fprintf('\n');
end

function local_print_gn_row(scenario_name, method_name, seed, diagnostics)
fprintf('  %-8s %-7s %-6g %15.3f %14d %14d %17.3e\n', ...
    scenario_name, method_name, seed, diagnostics.mean_iterations, ...
    diagnostics.max_iterations, diagnostics.nonconverged_count, diagnostics.final_step_norm);
end

function value = local_percent_change(value, reference)
if ~isfinite(value) || ~isfinite(reference) || reference == 0
    value = NaN;
else
    value = 100 * (value - reference) / reference;
end
end

function text_value = local_percent_text(value)
if isfinite(value)
    text_value = sprintf('%+.2f%%', value);
else
    text_value = 'NaN';
end
end

function text_value = local_number_or_label(value, label)
if isfinite(value)
    text_value = sprintf('%.2f', value);
else
    text_value = label;
end
end

function label = local_mode_label(cfg)
if isfield(cfg, 'mode')
    label = cfg.mode;
elseif cfg.t_stop <= 20
    label = 'quick';
else
    label = 'full';
end
end

function text_value = local_seed_text(seeds)
text_value = strtrim(sprintf('%g ', seeds));
end

function aggregate = local_aggregate(seed_results)
seed_count = numel(seed_results);
low_num = numel(seed_results(1).equal.metrics.full_rmse_3d);
equal_full = zeros(seed_count, low_num);
cusum_full = zeros(seed_count, low_num);
equal_window = nan(seed_count, low_num);
cusum_window = nan(seed_count, low_num);
for index = 1:seed_count
    equal_full(index, :) = seed_results(index).equal.metrics.full_rmse_3d';
    cusum_full(index, :) = seed_results(index).cusum.metrics.full_rmse_3d';
    equal_window(index, :) = seed_results(index).equal.metrics.window_rmse_3d';
    cusum_window(index, :) = seed_results(index).cusum.metrics.window_rmse_3d';
end
aggregate.equal_full_mean = mean(equal_full, 1)';
aggregate.equal_full_std = std(equal_full, 0, 1)';
aggregate.cusum_full_mean = mean(cusum_full, 1)';
aggregate.cusum_full_std = std(cusum_full, 0, 1)';
aggregate.equal_window_mean = mean(equal_window, 1, 'omitnan')';
aggregate.equal_window_std = std(equal_window, 0, 1, 'omitnan')';
aggregate.cusum_window_mean = mean(cusum_window, 1, 'omitnan')';
aggregate.cusum_window_std = std(cusum_window, 0, 1, 'omitnan')';
[aggregate.cusum_full_worst, worst_index] = max(cusum_full, [], 1);
aggregate.cusum_full_worst = aggregate.cusum_full_worst';
aggregate.cusum_full_worst_seed = [seed_results(worst_index).seed]';
end

function merged = local_merge_config(defaults, supplied)
merged = defaults;
names = fieldnames(supplied);
for index = 1:numel(names)
    merged.(names{index}) = supplied.(names{index});
end
end

function local_validate_experiment_config(cfg)
if cfg.uav_num ~= 5 || cfg.high_num ~= 3
    error('run_stage1_cusum_comparison:OriginalScenarioOnly', ...
        'Stage 1 preserves the original five-UAV, three-high-precision-UAV scenario.');
end
if cfg.dt <= 0 || cfg.graph_interval <= 0 || abs(cfg.graph_interval / cfg.dt - round(cfg.graph_interval / cfg.dt)) > eps
    error('run_stage1_cusum_comparison:InvalidTiming', ...
        'graph_interval must be an integer multiple of dt.');
end
if abs(cfg.sigma_dis - 0.2) > eps
    error('run_stage1_cusum_comparison:OriginalNoiseModelOnly', ...
        'Stage 1 preserves the original range standard deviation sigma_dis = 0.2 m.');
end
if numel(cfg.fault_edge) ~= 2 || any(cfg.fault_edge < 1) || any(cfg.fault_edge > cfg.uav_num) || cfg.fault_edge(1) == cfg.fault_edge(2)
    error('run_stage1_cusum_comparison:InvalidFaultEdge', 'fault_edge must identify two different global UAVs.');
end
if cfg.fault_enable && (cfg.fault_start < 0 || cfg.fault_end <= cfg.fault_start || cfg.fault_end > cfg.t_stop)
    error('run_stage1_cusum_comparison:InvalidFaultWindow', ...
        'For an enabled fault, require 0 <= fault_start < fault_end <= t_stop.');
end
end
