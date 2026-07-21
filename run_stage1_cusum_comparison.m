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

comparison_mode = local_comparison_mode(cfg);
method_labels = local_method_labels(comparison_mode);

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
        equal_cfg.graph_mode = 'single_epoch';
        start_time = tic;
        equal_result = local_run_cached_simulation(cache, equal_cfg);
        run_index = run_index + 1;
        if cfg.verbose
            local_print_progress(run_index, total_runs, scenario_name, method_labels{1}, seed, toc(start_time));
        end

        cusum_cfg = scenario_cfg;
        switch comparison_mode
            case 'original_vs_sliding_cusum'
                cusum_cfg.cusum_apply = true;
                cusum_cfg.graph_mode = 'sliding_window';
                cusum_cfg.cusum_consensus_enable = cfg.sliding_window_cusum_consensus_enable;
            case 'original_vs_sliding_equal'
                % Regression/ablation path: same sliding-window graph but
                % strictly equal range weights and no alarm-edge exclusion.
                cusum_cfg.cusum_apply = false;
                cusum_cfg.graph_mode = 'sliding_window';
                cusum_cfg.sliding_window_exclude_alarmed_edges = false;
                cusum_cfg.cusum_consensus_enable = false;
            otherwise
                cusum_cfg.cusum_apply = true;
                cusum_cfg.graph_mode = 'single_epoch';
        end
        start_time = tic;
        cusum_result = local_run_cached_simulation(cache, cusum_cfg);
        run_index = run_index + 1;
        if cfg.verbose
            local_print_progress(run_index, total_runs, scenario_name, method_labels{2}, seed, toc(start_time));
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
report.comparison_mode = comparison_mode;
report.method_labels = method_labels;
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
if isfield(cfg, 'plot_component_comparison') && cfg.plot_component_comparison
    local_plot_component_comparison(report, scenario_names);
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
    for leader = 1:cfg.high_num
        assert(local_global_to_internal(low_num + leader, cfg) == leader, ...
            'Leader global-to-internal node mapping changed unexpectedly.');
    end
    for follower = 1:low_num
        assert(local_global_to_internal(follower, cfg) == cfg.high_num + follower, ...
            'Follower global-to-internal node mapping changed unexpectedly.');
    end
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
base_leader_count = 3;
for vehicle = 1:min(cfg.high_num, base_leader_count)
    source_index = low_num + vehicle;
    posi_L_all(:, vehicle) = [posi_e_all(1, source_index); posi_n_all(1, source_index); posi_u_all(1, source_index)];
    posi_L_enu_all(:, vehicle) = posical_enu(posi_L_all(:, vehicle), posi_ini);
end
for vehicle = (base_leader_count + 1):cfg.high_num
    extra_index = vehicle - base_leader_count;
    posi_L_all(:, vehicle) = cfg.additional_leader_positions_xyz(:, extra_index);
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
window_history = repmat(local_empty_window_entry(low_num, cfg.uav_num), 0, 1);
last_sins_after_graph = zeros(3, low_num);
has_last_sins_after_graph = false;
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

        admitted_edge_mask = true(numel(edges), 1);
        graph_solve_timer = tic;
        if strcmp(local_graph_mode(cfg), 'sliding_window')
            % The CUSUM decision is made once when the range arrives.  Its
            % resulting weight travels with that range factor while the key
            % frame remains inside the window; it is never recomputed from
            % hindsight data.
            current_entry = local_make_window_entry(graph_positions_prior, edges, weights, detail, ...
                posiN_w_all, posi_ini, last_sins_after_graph, has_last_sins_after_graph, cfg);
            admitted_edge_mask = current_entry.admitted_edge_mask;
            window_history(end + 1, 1) = current_entry;
            if numel(window_history) > cfg.sliding_window_length
                window_history(1) = [];
            end
            [posi_w_graph, cov_graph, graph_solver] = local_solve_sliding_window( ...
                window_history, cfg, low_num);
        else
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
            cov_graph = zeros(3, low_num);
            for vehicle = 1:low_num
                internal_index = local_global_to_internal(vehicle, cfg);
                posi_w_graph(:, vehicle) = graph_1.parameters((internal_index - 1) * 3 + 1:internal_index * 3);
                cov_graph(:, vehicle) = sqrt(covariance((internal_index - 1) * 3 + 1:internal_index * 3));
            end
            graph_solver = graph_1;
        end
        graph_solve_seconds = toc(graph_solve_timer);
        graph_linear_diagnostics = local_linear_system_diagnostics( ...
            graph_solver.A, cfg.graph_condition_diagnostics);
        for vehicle = 1:low_num
            posiN_w_graph = posical_enu(posi_w_graph(:, vehicle), posi_ini);
            [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = kalm_factor_measure_update( ...
                current_time, posiN_w_all(:, vehicle), posiN_w_graph, cov_graph(:, vehicle), ...
                Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}, 1);
            [attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle)] = ...
                kalm_modi(attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle), Xc_all{vehicle});
            gyro_modi_all(:, vehicle) = Xc_all{vehicle}(10:12) + Xc_all{vehicle}(13:15);
            acc_modi_all(:, vehicle) = Xc_all{vehicle}(16:18);
        end
        for vehicle = 1:low_num
            last_sins_after_graph(:, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini);
        end
        has_last_sins_after_graph = true;
        history(graph_index).time = current_time;
        history(graph_index).pairs = detail.global_pairs;
        history(graph_index).detail = detail;
        history(graph_index).edge_count = numel(edges);
        history(graph_index).admitted_range_pairs = detail.global_pairs(admitted_edge_mask, :);
        history(graph_index).excluded_range_pairs = detail.global_pairs(~admitted_edge_mask, :);
        history(graph_index).graph_solve_seconds = graph_solve_seconds;
        history(graph_index).graph_linear_diagnostics = graph_linear_diagnostics;
        history(graph_index).cusum_prior_positions = node_positions_prior;
        history(graph_index).cusum_prior_covariances = node_covariances_prior;
        history(graph_index).graph_prior_positions = graph_positions_prior;
        history(graph_index).graph_follower_positions = posi_w_graph;
        history(graph_index).graph_follower_covariances = local_extract_current_follower_covariances( ...
            graph_solver, low_num, cfg);
        history(graph_index).gn_iteration_count = graph_solver.iteration_count;
        history(graph_index).gn_final_step_norm = graph_solver.final_step_norm;
        history(graph_index).gn_converged = graph_solver.converged;
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

function entry = local_empty_window_entry(low_num, node_count)
entry.node_priors = zeros(3, node_count);
entry.range_nodes = zeros(2, 0);
entry.range_measurements = zeros(0, 1);
entry.range_weights = zeros(0, 1);
entry.admitted_edge_mask = false(0, 1);
entry.motion_delta = zeros(3, low_num);
entry.has_motion = false;
end

function entry = local_make_window_entry(graph_positions_prior, edges, weights, detail, posiN_w_all, ...
        posi_ini, last_sins_after_graph, has_last_sins_after_graph, cfg)
low_num = cfg.uav_num - cfg.high_num;
entry = local_empty_window_entry(low_num, cfg.uav_num);
% Sliding-window nodes are ordered [leaders, followers], matching the
% original centralized factor graph.  CUSUM edge IDs remain in their
% established global order [followers, leaders] and are converted below.
entry.node_priors = [graph_positions_prior(:, low_num + 1:end), ...
    graph_positions_prior(:, 1:low_num)];
edge_count = numel(edges);
include_edge = local_window_admission_mask(edges, detail, cfg);
entry.admitted_edge_mask = include_edge;
included_count = sum(include_edge);
entry.range_nodes = zeros(2, included_count);
entry.range_measurements = zeros(included_count, 1);
entry.range_weights = zeros(included_count, 1);
entry_index = 0;
for admission_edge_index = 1:edge_count
    if ~include_edge(admission_edge_index)
        continue;
    end
    entry_index = entry_index + 1;
    entry.range_nodes(:, entry_index) = [ ...
        local_global_to_internal(edges(admission_edge_index).global_i, cfg); ...
        local_global_to_internal(edges(admission_edge_index).global_j, cfg)];
    entry.range_measurements(entry_index) = edges(admission_edge_index).measurement;
    entry.range_weights(entry_index) = weights(admission_edge_index);
end

entry.has_motion = has_last_sins_after_graph && cfg.sliding_window_motion_enable;
if has_last_sins_after_graph
    current_sins_position = zeros(3, low_num);
    for vehicle = 1:low_num
        current_sins_position(:, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini);
    end
    entry.motion_delta = current_sins_position - last_sins_after_graph;
end

function include_edge = local_window_admission_mask(edges, detail, cfg)
edge_count = numel(edges);
include_edge = true(edge_count, 1);
if ~cfg.sliding_window_exclude_alarmed_edges || ~cfg.cusum_apply || ...
        ~isfield(detail, 'alarm_active') || numel(detail.alarm_active) ~= edge_count
    return;
end
alarm_active = detail.alarm_active(:);
switch cfg.sliding_window_alarm_exclusion_mode
    case 'all'
        include_edge = ~alarm_active;
    case 'per_follower_max'
        low_num = cfg.uav_num - cfg.high_num;
        if ~isfield(detail, 'cusum_value') || numel(detail.cusum_value) ~= edge_count
            error('run_stage1_cusum_comparison:MissingCusumDetail', ...
                'CUSUM values are required for per-follower alarm admission.');
        end
        scores = detail.cusum_value(:);
        for follower = 1:low_num
            candidates = zeros(0, 1);
            for candidate_edge_index = 1:edge_count
                pair = [edges(candidate_edge_index).global_i, edges(candidate_edge_index).global_j];
                if ~alarm_active(candidate_edge_index) || ~any(pair == follower)
                    continue;
                end
                other_node = pair(pair ~= follower);
                % Retain the inter-follower range.  The selective rule is for
                % competing leader ranges that determine this follower's
                % absolute positioning geometry.
                if numel(other_node) == 1 && other_node > low_num
                    candidates(end + 1, 1) = candidate_edge_index; %#ok<AGROW>
                end
            end
            if isempty(candidates)
                continue;
            end
            candidate_scores = scores(candidates);
            candidate_scores(~isfinite(candidate_scores)) = -Inf;
            [~, local_index] = max(candidate_scores);
            include_edge(candidates(local_index)) = false;
        end
    otherwise
        error('run_stage1_cusum_comparison:InvalidSlidingWindowAlarmMode', ...
            'Unknown sliding_window_alarm_exclusion_mode.');
end
end
end

function [follower_positions, follower_std, graph] = local_solve_sliding_window(window_history, cfg, low_num)
frame_count = numel(window_history);
graph = factor_graph_sliding_window(frame_count, cfg.uav_num);
for frame_index = 1:frame_count
    entry = window_history(frame_index);
    graph.set_frame_initial(frame_index, entry.node_priors);
    for leader_index = 1:cfg.high_num
        graph.add_prior(frame_index, leader_index, entry.node_priors(:, leader_index), [0.2; 0.2; 0.5]);
    end
    for follower_index = 1:low_num
        graph.add_prior(frame_index, cfg.high_num + follower_index, ...
            entry.node_priors(:, cfg.high_num + follower_index), [10; 10; 20]);
    end
    for range_index = 1:numel(entry.range_measurements)
        graph.add_range(frame_index, entry.range_nodes(1, range_index), ...
            entry.range_nodes(2, range_index), entry.range_measurements(range_index), ...
            cfg.sigma_dis, entry.range_weights(range_index));
    end
    if frame_index > 1 && entry.has_motion
        for follower_index = 1:low_num
            graph.add_motion(frame_index - 1, frame_index, cfg.high_num + follower_index, ...
                entry.motion_delta(:, follower_index), cfg.sliding_window_motion_std);
        end
    end
end
graph.Gauss_Newton();
graph.covariance();
follower_positions = zeros(3, low_num);
follower_std = zeros(3, low_num);
for vehicle = 1:low_num
    node_index = cfg.high_num + vehicle;
    follower_positions(:, vehicle) = graph.get_position(frame_count, node_index);
    position_covariance = graph.get_position_covariance(frame_count, node_index);
    follower_std(:, vehicle) = sqrt(max(0, diag(position_covariance)));
end
end

function covariances = local_extract_current_follower_covariances(graph_solver, low_num, cfg)
if isa(graph_solver, 'factor_graph_sliding_window')
    covariances = zeros(3, 3, low_num);
    for vehicle = 1:low_num
        covariances(:, :, vehicle) = graph_solver.get_position_covariance( ...
            graph_solver.frame_count, cfg.high_num + vehicle);
    end
else
    covariances = local_extract_follower_covariances(graph_solver.P_all, low_num, cfg);
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

function diagnostics = local_empty_linear_system_diagnostics()
diagnostics.enabled = false;
diagnostics.row_count = NaN;
diagnostics.state_count = NaN;
diagnostics.rank = NaN;
diagnostics.minimum_singular_value = NaN;
diagnostics.condition_number = NaN;
diagnostics.information_rcond = NaN;
end

function diagnostics = local_linear_system_diagnostics(A, enabled)
diagnostics = local_empty_linear_system_diagnostics();
diagnostics.enabled = enabled;
if ~enabled
    return;
end
diagnostics.row_count = size(A, 1);
diagnostics.state_count = size(A, 2);
singular_values = svd(A, 'econ');
if isempty(singular_values)
    return;
end
tolerance = max(size(A)) * eps(max(singular_values));
diagnostics.rank = sum(singular_values > tolerance);
diagnostics.minimum_singular_value = singular_values(end);
if singular_values(end) > tolerance
    diagnostics.condition_number = singular_values(1) / singular_values(end);
else
    diagnostics.condition_number = Inf;
end
information = A' * A;
diagnostics.information_rcond = rcond((information + information') / 2);
end

function history = local_empty_history()
history.time = NaN;
history.pairs = zeros(0, 2);
history.detail = struct();
history.edge_count = 0;
history.admitted_range_pairs = zeros(0, 2);
history.excluded_range_pairs = zeros(0, 2);
history.graph_solve_seconds = NaN;
history.graph_linear_diagnostics = local_empty_linear_system_diagnostics();
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

function mode = local_comparison_mode(cfg)
mode = 'equal_vs_cusum';
if isfield(cfg, 'comparison_mode') && ~isempty(cfg.comparison_mode)
    mode = lower(char(cfg.comparison_mode));
end
valid_modes = {'equal_vs_cusum', 'original_vs_sliding_cusum', 'original_vs_sliding_equal'};
if ~any(strcmp(mode, valid_modes))
    error('run_stage1_cusum_comparison:InvalidComparisonMode', ...
        ['comparison_mode must be ''equal_vs_cusum'', ''original_vs_sliding_cusum'', ' ...
        'or ''original_vs_sliding_equal''.']);
end
end

function mode = local_graph_mode(cfg)
mode = 'single_epoch';
if isfield(cfg, 'graph_mode') && ~isempty(cfg.graph_mode)
    mode = lower(char(cfg.graph_mode));
end
if ~any(strcmp(mode, {'single_epoch', 'sliding_window'}))
    error('run_stage1_cusum_comparison:InvalidGraphMode', ...
        'graph_mode must be ''single_epoch'' or ''sliding_window''.');
end
end

function labels = local_method_labels(comparison_mode)
switch comparison_mode
    case 'original_vs_sliding_cusum'
        labels = {'original', 'sw-cusum'};
    case 'original_vs_sliding_equal'
        labels = {'original', 'sw-equal'};
    otherwise
        labels = {'equal', 'cusum'};
end
end

function local_print_progress(run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds)
fprintf('[%2d/%d ] scenario=%-7s method=%-9s seed=%-6g ... done (%.2f s)\n', ...
    run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds);
end

function local_plot_component_comparison(report, scenario_names)
cfg = report.cfg;
scenario_name = scenario_names{end};
results = report.scenario_results.(scenario_name);
seed_count = numel(results);
time = results(1).equal.time;
low_num = size(results(1).equal.error_xyz, 3);
baseline_error = zeros(size(results(1).equal.error_xyz));
enhanced_error = zeros(size(results(1).cusum.error_xyz));
for seed_index = 1:seed_count
    baseline_error = baseline_error + results(seed_index).equal.error_xyz;
    enhanced_error = enhanced_error + results(seed_index).cusum.error_xyz;
end
baseline_error = baseline_error / seed_count;
enhanced_error = enhanced_error / seed_count;

followers = 1:low_num;
if isfield(cfg, 'plot_follower_indices') && ~isempty(cfg.plot_follower_indices)
    followers = cfg.plot_follower_indices(:)';
end
component_names = {'East error (m)', 'North error (m)', 'Up error (m)'};
switch report.comparison_mode
    case 'original_vs_sliding_equal'
        baseline_label = 'Original FGO';
        enhanced_label = 'Sliding-window Equal FGO';
        figure_label = 'original vs sliding-window Equal FGO';
    case 'original_vs_sliding_cusum'
        baseline_label = 'Original FGO';
        enhanced_label = 'Sliding-window + CUSUM FGO';
        figure_label = 'original vs sliding-window CUSUM';
    otherwise
        baseline_label = 'Equal FGO';
        enhanced_label = 'CUSUM FGO';
        figure_label = 'single-epoch Equal FGO vs CUSUM FGO';
end
for follower = followers
    figure('Name', sprintf('Follower%d: %s', follower, figure_label), ...
        'NumberTitle', 'off');
    for component = 1:3
        subplot(3, 1, component);
        plot(time, baseline_error(:, component, follower), 'b-', 'LineWidth', 1.0); hold on;
        plot(time, enhanced_error(:, component, follower), 'r-', 'LineWidth', 1.0);
        if cfg.fault_enable
            xline(cfg.fault_start, 'k--', 'HandleVisibility', 'off');
            xline(cfg.fault_end, 'k--', 'HandleVisibility', 'off');
        end
        grid on;
        ylabel(component_names{component});
        if component == 1
            legend(baseline_label, enhanced_label, 'Location', 'best');
            if seed_count == 1
                title(sprintf('Follower%d position-error comparison (%s)', follower, scenario_name));
            else
                title(sprintf('Follower%d mean position-error comparison across %d seeds (%s)', ...
                    follower, seed_count, scenario_name));
            end
        end
        if component == 3
            xlabel('Time (s)');
        end
    end
end
end

function local_print_comparison_tables(report, scenario_names)
cfg = report.cfg;
if strcmp(report.comparison_mode, 'original_vs_sliding_cusum')
    fprintf('\nStage 1 original FGO vs sliding-window CUSUM-FGO\n');
    reference_label = 'Original';
elseif strcmp(report.comparison_mode, 'original_vs_sliding_equal')
    fprintf('\nStage 1 original FGO vs sliding-window Equal-FGO\n');
    reference_label = 'Original';
else
    fprintf('\nStage 1 CUSUM comparison\n');
    reference_label = 'Equal';
end
fprintf('  Mode                : %s\n', local_mode_label(cfg));
fprintf('  Seeds               : %s\n', local_seed_text(cfg.seeds));
if cfg.fault_enable
    fault_key = sort(cfg.fault_edge(:)');
    fprintf('  Fault configuration : edge (%d,%d), %.3f-%.3f s, bias %+.3f m\n', ...
        fault_key(1), fault_key(2), cfg.fault_start, cfg.fault_end, cfg.fault_bias);
else
    fprintf('  Fault configuration : disabled\n');
end
if strcmp(report.comparison_mode, 'original_vs_sliding_equal')
    fprintf('  CUSUM weights       : disabled (equal range weights)\n');
else
    fprintf('  CUSUM weights       : applied\n');
end
if strcmp(report.comparison_mode, 'original_vs_sliding_cusum')
    fprintf('  SW CUSUM consensus  : %s\n', ...
        local_enabled_label(cfg.sliding_window_cusum_consensus_enable));
    fprintf('  Alarm-edge admission: %s\n', local_alarm_admission_label(cfg));
end
if cfg.cusum_baseline_enable
    fprintf('  Nominal calibration : enabled (warm-up + abrupt-change lock)\n');
else
    fprintf('  Nominal calibration : disabled\n');
end
if cfg.plot_component_comparison
    fprintf('  Output              : command window + figures (no files)\n');
else
    fprintf('  Output              : command window only (no files)\n');
end

fprintf('\nPer-follower positioning summary (averaged across seeds only)\n');
fprintf('  %-8s %-10s %-10s %14s %22s %18s %20s\n', ...
    'Scenario', 'Method', 'Follower', 'Full RMSE(m)', 'Fault-window RMSE(m)', ...
    ['Full vs ' reference_label], ['Window vs ' reference_label]);
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    aggregate = report.scenario_aggregates.(scenario_name);
    local_print_summary_rows(scenario_name, report.method_labels{1}, aggregate.equal_full_mean, ...
        aggregate.equal_window_mean, aggregate.equal_full_mean, aggregate.equal_window_mean);
    local_print_summary_rows(scenario_name, report.method_labels{2}, aggregate.cusum_full_mean, ...
        aggregate.cusum_window_mean, aggregate.equal_full_mean, aggregate.equal_window_mean);
end

if numel(cfg.seeds) > 1
    fprintf('\nPer-follower RMSE standard deviation across seeds\n');
    fprintf('  %-8s %-10s %-10s %20s %28s\n', ...
        'Scenario', 'Method', 'Follower', 'Full RMSE Std(m)', 'Fault-window RMSE Std(m)');
    for scenario_index = 1:numel(scenario_names)
        scenario_name = scenario_names{scenario_index};
        aggregate = report.scenario_aggregates.(scenario_name);
        local_print_std_rows(scenario_name, report.method_labels{1}, aggregate.equal_full_std, aggregate.equal_window_std);
        local_print_std_rows(scenario_name, report.method_labels{2}, aggregate.cusum_full_std, aggregate.cusum_window_std);
    end
    fprintf('\nWorst-seed %s full RMSE\n', report.method_labels{2});
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

if ~strcmp(report.comparison_mode, 'original_vs_sliding_equal')
    local_print_weight_diagnostics(report, scenario_names);
end
local_print_gn_diagnostics(report, scenario_names);
end

function local_print_summary_rows(scenario_name, method_name, full_values, window_values, equal_full, equal_window)
for vehicle = 1:numel(full_values)
    if all(full_values(vehicle) == equal_full(vehicle)) && ...
            ((isnan(window_values(vehicle)) && isnan(equal_window(vehicle))) || ...
            window_values(vehicle) == equal_window(vehicle))
        full_change = 0;
        window_change = 0;
    else
        full_change = local_percent_change(full_values(vehicle), equal_full(vehicle));
        window_change = local_percent_change(window_values(vehicle), equal_window(vehicle));
    end
    fprintf('  %-8s %-10s %-10s %14.6f %22.6f %18s %20s\n', ...
        scenario_name, method_name, sprintf('Follower%d', vehicle), ...
        full_values(vehicle), window_values(vehicle), ...
        local_percent_text(full_change), local_percent_text(window_change));
end
end

function local_print_std_rows(scenario_name, method_name, full_std, window_std)
for vehicle = 1:numel(full_std)
    fprintf('  %-8s %-10s %-10s %20.6f %28.6f\n', ...
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
    fprintf('  %-8s %-10s %-6s %15s %14s %14s %17s\n', ...
    'Scenario', 'Method', 'Seed', 'Mean iterations', 'Max iterations', ...
    'Nonconverged', 'Final step norm');
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    results = report.scenario_results.(scenario_name);
    for seed_index = 1:numel(results)
        equal_gn = results(seed_index).equal.gn_diagnostics;
        cusum_gn = results(seed_index).cusum.gn_diagnostics;
        local_print_gn_row(scenario_name, report.method_labels{1}, results(seed_index).seed, equal_gn);
        local_print_gn_row(scenario_name, report.method_labels{2}, results(seed_index).seed, cusum_gn);
    end
end
fprintf('\n');
end

function local_print_gn_row(scenario_name, method_name, seed, diagnostics)
 fprintf('  %-8s %-10s %-6g %15.3f %14d %14d %17.3e\n', ...
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

function label = local_enabled_label(value)
if value
    label = 'enabled';
else
    label = 'disabled';
end
end

function label = local_alarm_admission_label(cfg)
if ~cfg.sliding_window_exclude_alarmed_edges
    label = 'disabled';
else
    label = cfg.sliding_window_alarm_exclusion_mode;
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
low_num = cfg.uav_num - cfg.high_num;
if ~((cfg.uav_num == 5 && cfg.high_num == 3) || (cfg.uav_num == 6 && cfg.high_num == 4)) || low_num ~= 2
    error('run_stage1_cusum_comparison:UnsupportedScenario', ...
        'Use the original 5-UAV/3-leader scenario or the redundant 6-UAV/4-leader scenario.');
end
extra_leader_count = cfg.high_num - 3;
if ~isnumeric(cfg.additional_leader_positions_xyz) || ...
        ~isequal(size(cfg.additional_leader_positions_xyz), [3, extra_leader_count]) || ...
        any(~isfinite(cfg.additional_leader_positions_xyz(:)))
    error('run_stage1_cusum_comparison:InvalidAdditionalLeaders', ...
        'additional_leader_positions_xyz must be a finite 3-by-(high_num-3) matrix.');
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
if ~isscalar(cfg.sliding_window_length) || cfg.sliding_window_length < 1 || ...
        cfg.sliding_window_length ~= floor(cfg.sliding_window_length)
    error('run_stage1_cusum_comparison:InvalidSlidingWindowLength', ...
        'sliding_window_length must be a positive integer.');
end
if ~isscalar(cfg.sliding_window_exclude_alarmed_edges) || ...
        (~islogical(cfg.sliding_window_exclude_alarmed_edges) && ...
        ~ismember(cfg.sliding_window_exclude_alarmed_edges, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidSlidingWindowAlarmExclusion', ...
        'sliding_window_exclude_alarmed_edges must be logical or 0/1.');
end
if ~ischar(cfg.sliding_window_alarm_exclusion_mode) && ~isstring(cfg.sliding_window_alarm_exclusion_mode)
    error('run_stage1_cusum_comparison:InvalidSlidingWindowAlarmMode', ...
        'sliding_window_alarm_exclusion_mode must be a character vector or string.');
end
alarm_mode = lower(char(cfg.sliding_window_alarm_exclusion_mode));
if ~any(strcmp(alarm_mode, {'all', 'per_follower_max'}))
    error('run_stage1_cusum_comparison:InvalidSlidingWindowAlarmMode', ...
        'sliding_window_alarm_exclusion_mode must be ''all'' or ''per_follower_max''.');
end
if ~isscalar(cfg.sliding_window_cusum_consensus_enable) || ...
        (~islogical(cfg.sliding_window_cusum_consensus_enable) && ...
        ~ismember(cfg.sliding_window_cusum_consensus_enable, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidSlidingWindowConsensus', ...
        'sliding_window_cusum_consensus_enable must be logical or 0/1.');
end
if ~isnumeric(cfg.sliding_window_motion_std) || ~isreal(cfg.sliding_window_motion_std) || ...
        numel(cfg.sliding_window_motion_std) ~= 3 || ...
        any(~isfinite(cfg.sliding_window_motion_std(:))) || any(cfg.sliding_window_motion_std(:) <= 0)
    error('run_stage1_cusum_comparison:InvalidSlidingWindowMotionStd', ...
        'sliding_window_motion_std must be a positive finite three-element vector.');
end
if ~isscalar(cfg.sliding_window_motion_enable) || ...
        (~islogical(cfg.sliding_window_motion_enable) && ...
        ~ismember(cfg.sliding_window_motion_enable, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidSlidingWindowMotionEnable', ...
        'sliding_window_motion_enable must be logical or 0/1.');
end
if ~isscalar(cfg.graph_condition_diagnostics) || ...
        (~islogical(cfg.graph_condition_diagnostics) && ...
        ~ismember(cfg.graph_condition_diagnostics, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidGraphConditionDiagnostics', ...
        'graph_condition_diagnostics must be logical or 0/1.');
end
if isfield(cfg, 'plot_follower_indices') && ~isempty(cfg.plot_follower_indices) && ...
        (any(cfg.plot_follower_indices ~= floor(cfg.plot_follower_indices)) || ...
        any(cfg.plot_follower_indices < 1) || any(cfg.plot_follower_indices > cfg.uav_num - cfg.high_num))
    error('run_stage1_cusum_comparison:InvalidPlotFollowers', ...
        'plot_follower_indices must contain valid follower indices.');
end
local_comparison_mode(cfg);
local_graph_mode(cfg);
end
