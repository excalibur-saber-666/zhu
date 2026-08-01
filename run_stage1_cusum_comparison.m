function report = run_stage1_cusum_comparison(cfg)
%RUN_STAGE1_CUSUM_COMPARISON Run the selected Stage-1 comparison methods.
%   CFG.METHODS is any non-empty subset of:
%   EKF, FGO, CUSUM_EKF, and CUSUM_FGO.  Every selected method consumes the
%   same cached IMU, GPS, and range-noise inputs for a fair comparison.

if nargin < 1 || isempty(cfg)
    cfg = stage1_cusum_3f3l_config('full');
else
    cfg = local_merge_config(stage1_cusum_3f3l_config('full'), cfg);
end
local_validate_experiment_config(cfg);
report = local_run_four_method_comparison(cfg);
end

function report = local_run_four_method_comparison(cfg)
% Run all four methods or a requested non-empty subset with shared inputs.
method_names = cfg.methods;
method_labels = local_selected_method_labels(method_names);
scenario_names = local_scenario_names(cfg);
seed_count = numel(cfg.seeds);
template = struct('seed', [], 'ekf', [], 'fgo', [], 'cusum_ekf', [], 'cusum_fgo', []);
seed_results = repmat(template, seed_count, 1);
scenario_results = struct();
for scenario_index = 1:numel(scenario_names)
    scenario_results.(scenario_names{scenario_index}) = repmat(template, seed_count, 1);
end
fault_schedules = cell(seed_count, 1);

total_runs = numel(method_names) * numel(scenario_names) * seed_count;
run_index = 0;
for seed_index = 1:seed_count
    seed = cfg.seeds(seed_index);
    cache = local_make_input_cache(cfg, seed);
    fault_schedules{seed_index} = cache.fault_segments;
    for scenario_index = 1:numel(scenario_names)
        scenario_name = scenario_names{scenario_index};
        scenario_cfg = cfg;
        scenario_cfg.fault_enable = strcmp(scenario_name, 'fault');
        scenario_cfg.resolved_fault_segments = cache.fault_segments;
        current = template;
        current.seed = seed;
        for method_index = 1:numel(method_names)
            method_name = method_names{method_index};
            method_cfg = local_four_method_config(scenario_cfg, method_name);
            start_time = tic;
            is_equal_weight = ~method_cfg.cusum_apply;
            current.(method_name) = local_run_cached_simulation(cache, method_cfg, ...
                is_equal_weight, method_cfg.estimator_mode);
            run_index = run_index + 1;
            if cfg.verbose
                local_print_progress(run_index, total_runs, scenario_name, ...
                    method_labels{method_index}, seed, toc(start_time));
            end
        end
        scenario_results.(scenario_name)(seed_index) = current;
    end
    seed_results(seed_index) = scenario_results.(scenario_names{end})(seed_index);
end

report.cfg = cfg;
report.comparison_mode = 'four_method';
report.method_names = method_names;
report.method_labels = method_labels;
report.seed_results = seed_results;
report.aggregate = local_aggregate_four_methods(seed_results, method_names);
report.scenario_results = scenario_results;
report.fault_schedules = fault_schedules;
report.scenario_aggregates = struct();
for scenario_index = 1:numel(scenario_names)
    name = scenario_names{scenario_index};
    report.scenario_aggregates.(name) = local_aggregate_four_methods( ...
        scenario_results.(name), method_names);
end
report.paired_comparisons = local_four_method_paired_comparisons( ...
    report.scenario_aggregates, scenario_names);
report.position_metrics = compute_stage1_position_metrics(report);
report.fault_segment_metrics = local_fault_segment_position_metrics( ...
    scenario_results, method_names);
if cfg.verbose
    local_print_four_method_rmse(report, scenario_names);
end
end

function method_cfg = local_four_method_config(cfg, method_name)
method_cfg = cfg;
switch method_name
    case 'ekf'
        method_cfg.estimator_mode = 'ekf';
        method_cfg.cusum_apply = false;
        method_cfg.graph_mode = 'single_epoch';
        method_cfg.sliding_window_exclude_alarmed_edges = false;
    case 'fgo'
        method_cfg.estimator_mode = 'fgo';
        method_cfg.cusum_apply = false;
        method_cfg.graph_mode = 'single_epoch';
        method_cfg.sliding_window_exclude_alarmed_edges = false;
    case 'cusum_ekf'
        method_cfg.estimator_mode = 'ekf';
        method_cfg.cusum_apply = true;
        method_cfg.graph_mode = 'single_epoch';
        method_cfg.sliding_window_exclude_alarmed_edges = true;
        method_cfg.sliding_window_alarm_exclusion_mode = 'per_follower_max';
    case 'cusum_fgo'
        method_cfg.estimator_mode = 'fgo';
        method_cfg.cusum_apply = true;
        method_cfg.graph_mode = 'sliding_window';
        method_cfg.cusum_consensus_enable = cfg.sliding_window_cusum_consensus_enable;
        method_cfg.sliding_window_exclude_alarmed_edges = true;
    otherwise
        error('run_stage1_cusum_comparison:InvalidFourMethodName', ...
            'Unknown four-method comparison member "%s".', method_name);
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
cache.fault_segments = local_resolve_fault_segments(cfg, seed);
end

function result = local_run_cached_simulation(cache, cfg, use_equal_weights, estimator_mode)
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

data_directory = stage1_project_root();
posi_e_all = load(fullfile(data_directory, 'posi_e_all.dat'));
posi_n_all = load(fullfile(data_directory, 'posi_n_all.dat'));
posi_u_all = load(fullfile(data_directory, 'posi_u_all.dat'));
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
    source_index = cfg.base_leader_source_indices(vehicle);
    if source_index > size(posi_e_all, 2) || source_index > size(posi_n_all, 2) || ...
            source_index > size(posi_u_all, 2)
        error('run_stage1_cusum_comparison:LeaderSourceOutOfRange', ...
            'base_leader_source_indices contains a column unavailable in the position data.');
    end
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
leader_truth_xyz = zeros(sample_count, 3, cfg.high_num);
for vehicle = 1:low_num
    truth_xyz(1, :, vehicle) = posi_w_all(:, vehicle)';
    navigation_xyz(1, :, vehicle) = posi_w_all(:, vehicle)';
end
for leader = 1:cfg.high_num
    leader_truth_xyz(1, :, leader) = posi_L_all(:, leader)';
end
history = repmat(local_empty_history(), cache.graph_count, 1);
range_error_history = nan(cache.graph_count, low_num, cfg.uav_num);
cusum_state = [];
window_history = repmat(local_empty_window_entry(low_num, cfg.uav_num), 0, 1);
imu_interval_buffer = local_empty_imu_interval_buffer(low_num);
imu_preint_active = strcmp(estimator_mode, 'fgo') && ...
    strcmp(local_graph_mode(cfg), 'sliding_window') && ...
    cfg.imu_preintegration_enable;
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
        gyro_for_sins_deg_s = Wibb_noise - gyro_modi_all(:, vehicle) / 0.01745329252;
        acc_for_sins_mps2 = Fb_noise - acc_modi_all(:, vehicle);
        if imu_preint_active
            imu_interval_buffer{vehicle}.gyro_rad_s(:, end + 1) = ...
                gyro_for_sins_deg_s * pi / 180;
            imu_interval_buffer{vehicle}.specific_force_mps2(:, end + 1) = ...
                acc_for_sins_mps2;
            imu_interval_buffer{vehicle}.dt(end + 1) = cfg.dt;
            imu_interval_buffer{vehicle}.time(end + 1) = current_time;
        end
        [attiN_all(:, vehicle), WnbbA_old(:, vehicle)] = atti_cal_cq_modi(cfg.dt, ...
            gyro_for_sins_deg_s, attiN_all(:, vehicle), ...
            veloN_all(:, vehicle), posiN_w_all(:, vehicle), WnbbA_old(:, vehicle));
        veloN_all(:, vehicle) = velo_cal(cfg.dt, acc_for_sins_mps2, ...
            attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle));
        posiN_w_all(:, vehicle) = posi_cal(cfg.dt, veloN_all(:, vehicle), posiN_w_all(:, vehicle));
    end

    if mod(step, cache.graph_stride) == 0
        graph_index = graph_index + 1;
        [posiG_w_all, posiG_L] = local_cached_gps_measurements( ...
            posi_w_enu_all, posi_L_enu_all, cache, graph_index);
        % The preceding key frame has already injected its estimated error
        % into the SINS nominal state.  Start this error-state propagation
        % from zero so that position, velocity and attitude corrections are
        % not applied a second time at the next graph epoch.  The covariance
        % and the IMU bias corrections kept by the SINS loop are retained.
        for vehicle = 1:low_num
            Xc_all{vehicle} = zeros(18, 1);
        end
        for vehicle = 1:low_num
            [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = kalm_factor_time_update( ...
                current_time, cfg.graph_interval, Fb_noise_all(:, vehicle), attiN_all(:, vehicle), ...
                veloN_all(:, vehicle), posiN_w_all(:, vehicle), zeros(18, 1), ...
                PK_all{vehicle}, Xerr_all{vehicle});
        end

        % Form the time-update snapshot.  FGO retains this prediction for
        % its online range detector; EKF replaces it below with the
        % GPS-corrected nominal state before assimilating ranges.
        node_positions_prior = zeros(3, cfg.uav_num);
        node_covariances_prior = zeros(3, 3, cfg.uav_num);
        for vehicle = 1:low_num
            node_positions_prior(:, vehicle) = posical_xyz( ...
                posiN_w_all(:, vehicle), posi_ini);
            node_covariances_prior(:, :, vehicle) = local_kf_position_covariance_enu( ...
                PK_all{vehicle}, posiN_w_all(:, vehicle));
        end
        for vehicle = (low_num + 1):cfg.uav_num
            node_positions_prior(:, vehicle) = posical_xyz( ...
                posiG_L(:, vehicle - low_num), posi_ini);
            node_covariances_prior(:, :, vehicle) = diag([0.2, 0.2, 0.5].^2);
        end

        if strcmp(estimator_mode, 'ekf')
            % Inject the GPS update before processing cooperative ranges.
            % The range Jacobian is therefore linearized around the same
            % corrected nominal state for every follower, which avoids
            % mixing an uncorrected self state with corrected neighbours.
            for vehicle = 1:low_num
                [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = ...
                    kalm_factor_measure_update(current_time, posiN_w_all(:, vehicle), ...
                    posiG_w_all(:, vehicle), [10; 10; 20], Xc_all{vehicle}, ...
                    PK_all{vehicle}, Xerr_all{vehicle}, 1);
                [attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle)] = ...
                    kalm_modi(attiN_all(:, vehicle), veloN_all(:, vehicle), ...
                    posiN_w_all(:, vehicle), Xc_all{vehicle});
                gyro_modi_all(:, vehicle) = Xc_all{vehicle}(10:12) + ...
                    Xc_all{vehicle}(13:15);
                acc_modi_all(:, vehicle) = Xc_all{vehicle}(16:18);
                Xc_all{vehicle} = zeros(18, 1);
                node_positions_prior(:, vehicle) = posical_xyz( ...
                    posiN_w_all(:, vehicle), posi_ini);
                node_covariances_prior(:, :, vehicle) = ...
                    local_kf_position_covariance_enu(PK_all{vehicle}, ...
                    posiN_w_all(:, vehicle));
            end
        end

        [posi_w_all, posi_L_all, dis_true] = distance_cal( ...
            posi_w_enu_all, posi_L_enu_all, posi_ini, cfg.uav_num, cfg.high_num);
        [dis_measure, uav_link_num] = local_cached_pseudorange( ...
            dis_true, cache.range_standard(:, :, graph_index), cfg);
        dis_measure = local_inject_fault(dis_measure, current_time, cfg);
        range_error_history(graph_index, :, :) = reshape( ...
            dis_measure - dis_true, [1, low_num, cfg.uav_num]);

        posi_w_graph = zeros(3, low_num);
        posi_L_graph = zeros(3, cfg.high_num);
        for vehicle = 1:low_num
            posi_w_graph(:, vehicle) = posical_xyz(posiG_w_all(:, vehicle), posi_ini);
        end
        for vehicle = 1:cfg.high_num
            posi_L_graph(:, vehicle) = posical_xyz(posiG_L(:, vehicle), posi_ini);
        end
        % FGO uses GPS position priors in its graph.  EKF uses the
        % GPS-corrected SINS snapshot above as its range-update nominal state.
        graph_positions_prior = [posi_w_graph, posi_L_graph];
        range_update_positions_prior = node_positions_prior;
        range_update_covariances_prior = node_covariances_prior;

        edges = local_build_active_edges(dis_measure, uav_link_num, cfg);
        use_range_predictor_cusum = strcmp(estimator_mode, 'ekf') || ...
            (strcmp(estimator_mode, 'fgo') && ...
            strcmpi(cfg.fgo_cusum_detector_mode, 'range_predictor'));
        cusum_detector_cfg = local_cusum_detector_config(cfg, estimator_mode, ...
            use_range_predictor_cusum);
        if cfg.cusum_apply && use_range_predictor_cusum
            [weights, detail, cusum_state] = compute_ekf_cusum_range_weights( ...
                edges, cusum_detector_cfg, cusum_state, current_time);
        elseif use_equal_weights
            weights = ones(numel(edges), 1);
            detail = local_equal_weight_detail(edges);
        else
            [weights, detail, cusum_state] = compute_signed_cusum_edge_weights( ...
                edges, node_positions_prior, node_covariances_prior, cfg, cusum_state, current_time);
        end

        admitted_edge_mask = local_window_admission_mask(edges, detail, cfg);
        current_entry = local_empty_window_entry(low_num, cfg.uav_num);
        imu_state_diagnostics = local_empty_imu_state_diagnostics(low_num);
        graph_solve_timer = tic;
        graph_solver = [];
        graph_linear_diagnostics = local_empty_linear_system_diagnostics();
        if strcmp(estimator_mode, 'ekf')
            % Freeze every node before any follower update.  This makes the
            % batch EKF update independent of follower and edge traversal
            % order while carrying neighbour uncertainty into R_eff.
            admitted_edges = edges(admitted_edge_mask);
            admitted_weights = weights(admitted_edge_mask);
            [admitted_edges, admitted_weights] = ...
                local_ekf_alarm_surrogate_edges(admitted_edges, ...
                admitted_weights, edges, detail, admitted_edge_mask, cfg);
            [admitted_edges, admitted_weights] = local_ekf_range_edges( ...
                admitted_edges, admitted_weights, cfg);
            for vehicle = 1:low_num
                % All node means and covariances are frozen after the GPS
                % update.  This makes the follower updates order-independent
                % and keeps the range residual consistent with its Jacobian.
                follower_update_positions = range_update_positions_prior;
                [Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}] = ...
                    cooperative_range_ekf_update(posiN_w_all(:, vehicle), ...
                    Xc_all{vehicle}, PK_all{vehicle}, Xerr_all{vehicle}, vehicle, ...
                    admitted_edges, follower_update_positions, ...
                    range_update_covariances_prior, cfg, admitted_weights);
                [attiN_all(:, vehicle), veloN_all(:, vehicle), posiN_w_all(:, vehicle)] = ...
                    kalm_modi(attiN_all(:, vehicle), veloN_all(:, vehicle), ...
                    posiN_w_all(:, vehicle), Xc_all{vehicle});
                gyro_modi_all(:, vehicle) = Xc_all{vehicle}(10:12) + Xc_all{vehicle}(13:15);
                acc_modi_all(:, vehicle) = Xc_all{vehicle}(16:18);
                Xc_all{vehicle} = zeros(18, 1);
            end
        elseif strcmp(local_graph_mode(cfg), 'sliding_window')
            % The CUSUM decision is made once when the range arrives.  Its
            % resulting weight travels with that range factor while the key
            % frame remains inside the window; it is never recomputed from
            % hindsight data.
            current_entry = local_make_window_entry(current_time, graph_positions_prior, edges, weights, detail, ...
                posiN_w_all, veloN_all, attiN_all, imu_interval_buffer, posi_ini, ...
                last_sins_after_graph, has_last_sins_after_graph, cfg);
            admitted_edge_mask = current_entry.admitted_edge_mask;
            window_history(end + 1, 1) = current_entry;
            if imu_preint_active
                window_limit = cfg.imu_preint_window_length;
            else
                window_limit = cfg.sliding_window_length;
            end
            if numel(window_history) > window_limit
                window_history(1) = [];
            end
            if imu_preint_active
                [posi_w_graph, cov_graph, graph_solver, imu_state_diagnostics] = ...
                    local_solve_sliding_window_imu_preint(window_history, cfg, low_num);
                window_history = local_store_imu_window_states( ...
                    window_history, imu_state_diagnostics);
                imu_interval_buffer = local_empty_imu_interval_buffer(low_num);
            else
                [posi_w_graph, cov_graph, graph_solver] = ...
                    local_solve_sliding_window(window_history, cfg, low_num);
            end
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
        if strcmp(estimator_mode, 'fgo')
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
        if strcmp(estimator_mode, 'fgo')
            history(graph_index).graph_follower_positions = posi_w_graph;
            history(graph_index).graph_follower_covariances = local_extract_current_follower_covariances( ...
                graph_solver, low_num, cfg);
            history(graph_index).gn_iteration_count = graph_solver.iteration_count;
            history(graph_index).gn_final_step_norm = graph_solver.final_step_norm;
            history(graph_index).gn_converged = graph_solver.converged;
        else
            history(graph_index).graph_follower_positions = local_navigation_positions_enu( ...
                posiN_w_all, posi_ini);
            history(graph_index).graph_follower_covariances = local_kf_follower_covariances( ...
                PK_all, posiN_w_all);
        end
        history(graph_index).imu_preintegration_seconds = current_entry.preintegration_seconds;
        history(graph_index).imu_sample_counts = current_entry.imu_sample_counts;
        history(graph_index).imu_delta_t = current_entry.imu_delta_t;
        history(graph_index).imu_state_diagnostics = imu_state_diagnostics;
        history(graph_index).imu_full_state_prior_count = ...
            imu_state_diagnostics.full_state_prior_count;
        history(graph_index).imu_position_prior_count = ...
            imu_state_diagnostics.position_prior_count;
        history(graph_index).imu_repropagation_count = ...
            imu_state_diagnostics.repropagation_count;
        history(graph_index).imu_repropagation_seconds = ...
            imu_state_diagnostics.repropagation_seconds;
    end

    for vehicle = 1:low_num
        posi_w_all(:, vehicle) = posical_xyz(posi_w_enu_all(:, vehicle), posi_ini);
        truth_xyz(step + 1, :, vehicle) = posi_w_all(:, vehicle)';
        navigation_xyz(step + 1, :, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini)';
    end
    for leader = 1:cfg.high_num
        leader_truth_xyz(step + 1, :, leader) = ...
            posical_xyz(posi_L_enu_all(:, leader), posi_ini)';
    end
end

function [edges, weights] = local_ekf_range_edges(edges, weights, cfg)
% A local 18-state EKF has no cross-covariance between different followers.
% Therefore it only assimilates ranges to independent high-precision leaders;
% follower-to-follower ranges remain available to FGO and to online logging.
if ~cfg.ekf_range_leader_only || isempty(edges)
    return;
end

low_num = cfg.uav_num - cfg.high_num;
keep = false(numel(edges), 1);
for edge_index = 1:numel(edges)
    pair = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
    keep(edge_index) = pair(1) <= low_num && pair(2) > low_num;
end
edges = edges(keep);
weights = weights(keep);
end

function [admitted_edges, admitted_weights] = local_ekf_alarm_surrogate_edges( ...
        admitted_edges, admitted_weights, edges, detail, admitted_edge_mask, cfg)
% Keep a confirmed raw leader range isolated.  If enabled, replace it with a
% low-information surrogate generated by the same online range predictor
% that feeds CUSUM.  This preserves weak geometry without reusing the raw
% faulty observation or any offline fault information.
if ~cfg.cusum_apply || ~cfg.ekf_cusum_alarm_surrogate_enable || ...
        ~isfield(detail, 'alarm_active') || ...
        ~isfield(detail, 'predicted_range') || ...
        numel(detail.alarm_active) ~= numel(edges) || ...
        numel(detail.predicted_range) ~= numel(edges)
    return;
end
low_num = cfg.uav_num - cfg.high_num;
for edge_index = 1:numel(edges)
    pair = sort([edges(edge_index).global_i, edges(edge_index).global_j]);
    if admitted_edge_mask(edge_index) || pair(1) > low_num || ...
            pair(2) <= low_num || ~detail.alarm_active(edge_index)
        continue;
    end
    predicted_range = detail.predicted_range(edge_index);
    if ~isfinite(predicted_range) || predicted_range <= 0
        continue;
    end
    surrogate = edges(edge_index);
    surrogate.measurement = predicted_range;
    admitted_edges(end + 1, 1) = surrogate; %#ok<AGROW>
    admitted_weights(end + 1, 1) = ...
        cfg.ekf_cusum_alarm_surrogate_weight; %#ok<AGROW>
end
end

result.seed = cache.seed;
result.time = (0:cache.step_count)' * cfg.dt;
result.truth_xyz = truth_xyz;
result.leader_truth_xyz = leader_truth_xyz;
result.navigation_xyz = navigation_xyz;
result.error_xyz = navigation_xyz - truth_xyz;
result.range_time = (1:cache.graph_count)' * cfg.graph_interval;
result.range_error = range_error_history;
result.fault_segments = local_active_fault_segments(cfg);
result.history = history;
result.final_cusum_state = cusum_state;
result.estimator_mode = estimator_mode;
result.graph_mode = local_graph_mode(cfg);
result.gps_update_applied = strcmp(estimator_mode, 'ekf');
result.cusum_soft_weighting_applied = cfg.cusum_apply;
result.imu_preintegration_applied = imu_preint_active;
result.alarm_edge_exclusion_applied = cfg.cusum_apply && ...
    cfg.sliding_window_exclude_alarmed_edges;
result.complete_cusum_applied = result.cusum_soft_weighting_applied && ...
    result.alarm_edge_exclusion_applied;
if strcmp(estimator_mode, 'ekf')
    result.cusum_detector_mode = 'range_predictor';
elseif cfg.cusum_apply
    result.cusum_detector_mode = lower(char(cfg.fgo_cusum_detector_mode));
else
    result.cusum_detector_mode = 'disabled';
end
result.metrics = local_metrics(result, cfg);
result.diagnostics = local_diagnostics(history, cfg);
result.gn_diagnostics = local_gn_diagnostics(history);
result.imu_preintegration_diagnostics = local_imu_preintegration_diagnostics(history);
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
segments = local_active_fault_segments(cfg);
if isempty(segments)
    return;
end
low_num = cfg.uav_num - cfg.high_num;
for segment_index = 1:numel(segments)
    segment = segments(segment_index);
    if current_time < segment.start || current_time > segment.end
        continue;
    end
    fault_key = sort(segment.edge(:)');
    if fault_key(1) <= low_num
        source = fault_key(1);
        target = fault_key(2);
        dis_measure(source, target) = dis_measure(source, target) + segment.bias;
    elseif fault_key(2) <= low_num
        source = fault_key(2);
        target = fault_key(1);
        dis_measure(source, target) = dis_measure(source, target) + segment.bias;
    end
end
end

function segments = local_resolve_fault_segments(cfg, seed)
segments = repmat(struct('start', NaN, 'end', NaN, 'bias', NaN, ...
    'edge', [NaN, NaN]), 0, 1);
if ~cfg.fault_enable
    return;
end

mode = lower(char(cfg.fault_mode));
if strcmp(mode, 'single')
    segments(1, 1) = struct('start', cfg.fault_start, 'end', cfg.fault_end, ...
        'bias', cfg.fault_bias, 'edge', sort(cfg.fault_edge(:)'));
    return;
end

specification = cfg.fault_segments;
low_num = cfg.uav_num - cfg.high_num;
if strcmp(mode, 'segmented_explicit_edges')
    segments = repmat(struct('start', NaN, 'end', NaN, 'bias', NaN, ...
        'edge', [NaN, NaN]), size(specification, 1), 1);
    for segment_index = 1:size(specification, 1)
        segments(segment_index) = struct( ...
            'start', specification(segment_index, 1), ...
            'end', specification(segment_index, 2), ...
            'bias', specification(segment_index, 3), ...
            'edge', sort(specification(segment_index, 4:5)));
    end
    return;
end

candidates = zeros(low_num * cfg.high_num, 2);
candidate_index = 0;
for follower = 1:low_num
    for leader = 1:cfg.high_num
        candidate_index = candidate_index + 1;
        candidates(candidate_index, :) = [follower, low_num + leader];
    end
end

% Select after the navigation input cache has been generated and use a
% separate deterministic stream.  Thus edge selection is reproducible but
% does not alter the IMU/GPS/range noise shared by the four methods.
old_rng = rng;
cleanup = onCleanup(@() rng(old_rng)); %#ok<NASGU>
fault_seed = mod(round(double(seed)) + 104729, 2^32);
rng(fault_seed, 'twister');
if size(specification, 1) <= size(candidates, 1)
    selected = randperm(size(candidates, 1), size(specification, 1));
else
    selected = randi(size(candidates, 1), size(specification, 1), 1);
end

segments = repmat(struct('start', NaN, 'end', NaN, 'bias', NaN, ...
    'edge', [NaN, NaN]), size(specification, 1), 1);
for segment_index = 1:size(specification, 1)
    segments(segment_index) = struct( ...
        'start', specification(segment_index, 1), ...
        'end', specification(segment_index, 2), ...
        'bias', specification(segment_index, 3), ...
        'edge', candidates(selected(segment_index), :));
end
end

function segments = local_active_fault_segments(cfg)
segments = repmat(struct('start', NaN, 'end', NaN, 'bias', NaN, ...
    'edge', [NaN, NaN]), 0, 1);
if ~cfg.fault_enable
    return;
end
if isfield(cfg, 'resolved_fault_segments')
    segments = cfg.resolved_fault_segments;
elseif strcmpi(cfg.fault_mode, 'single')
    segments(1, 1) = struct('start', cfg.fault_start, 'end', cfg.fault_end, ...
        'bias', cfg.fault_bias, 'edge', sort(cfg.fault_edge(:)'));
else
    error('run_stage1_cusum_comparison:UnresolvedFaultSegments', ...
        'Segmented random fault edges must be resolved once from the experiment seed.');
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

function positions_enu = local_navigation_positions_enu(posiN_w_all, posi_ini)
low_num = size(posiN_w_all, 2);
positions_enu = zeros(3, low_num);
for vehicle = 1:low_num
    positions_enu(:, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini);
end
end

function covariances_enu = local_kf_follower_covariances(PK_all, posiN_w_all)
low_num = numel(PK_all);
covariances_enu = zeros(3, 3, low_num);
for vehicle = 1:low_num
    covariances_enu(:, :, vehicle) = local_kf_position_covariance_enu( ...
        PK_all{vehicle}, posiN_w_all(:, vehicle));
end
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
entry.time = NaN;
entry.node_priors = zeros(3, node_count);
entry.follower_velocity_prior = zeros(3, low_num);
entry.follower_rotation_prior = repmat(eye(3), 1, 1, low_num);
entry.follower_gyro_bias_prior = zeros(3, low_num);
entry.follower_acc_bias_prior = zeros(3, low_num);
entry.preintegrations = cell(1, low_num);
entry.preintegration_seconds = 0;
entry.imu_sample_counts = zeros(low_num, 1);
entry.imu_delta_t = zeros(low_num, 1);
entry.range_nodes = zeros(2, 0);
entry.range_measurements = zeros(0, 1);
entry.range_weights = zeros(0, 1);
entry.admitted_edge_mask = false(0, 1);
entry.motion_delta = zeros(3, low_num);
entry.has_motion = false;
entry.has_optimized_state = false;
entry.optimized_follower_states = cell(1, low_num);
end

function entry = local_make_window_entry(current_time, graph_positions_prior, edges, weights, detail, posiN_w_all, ...
        veloN_all, attiN_all, imu_interval_buffer, posi_ini, ...
        last_sins_after_graph, has_last_sins_after_graph, cfg)
low_num = cfg.uav_num - cfg.high_num;
entry = local_empty_window_entry(low_num, cfg.uav_num);
entry.time = current_time;
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

for vehicle = 1:low_num
    entry.follower_velocity_prior(:, vehicle) = veloN_all(:, vehicle);
    entry.follower_rotation_prior(:, :, vehicle) = ...
        local_enu_rotation_from_attitude(attiN_all(:, vehicle));
end
entry.has_motion = has_last_sins_after_graph && ...
    cfg.sliding_window_motion_enable && ~cfg.imu_preintegration_enable;
if has_last_sins_after_graph
    current_sins_position = zeros(3, low_num);
    for vehicle = 1:low_num
        current_sins_position(:, vehicle) = posical_xyz(posiN_w_all(:, vehicle), posi_ini);
    end
    entry.motion_delta = current_sins_position - last_sins_after_graph;
end
if cfg.imu_preintegration_enable
    noise = imu_preintegration_default_noise(cfg);
    preintegration_timer = tic;
    for vehicle = 1:low_num
        samples = imu_interval_buffer{vehicle};
        if isempty(samples.dt)
            error('run_stage1_cusum_comparison:MissingImuSamples', ...
                'IMU preintegration requires samples for every graph interval.');
        end
        entry.preintegrations{vehicle} = imu_preintegrate_interval( ...
            samples.gyro_rad_s, samples.specific_force_mps2, samples.dt, ...
            zeros(3, 1), zeros(3, 1), noise);
        entry.imu_sample_counts(vehicle) = ...
            entry.preintegrations{vehicle}.sample_count;
        entry.imu_delta_t(vehicle) = entry.preintegrations{vehicle}.delta_t;
    end
    entry.preintegration_seconds = toc(preintegration_timer);
end
end

function buffers = local_empty_imu_interval_buffer(low_num)
template = struct('gyro_rad_s', zeros(3, 0), ...
    'specific_force_mps2', zeros(3, 0), 'dt', zeros(1, 0), ...
    'time', zeros(1, 0));
buffers = repmat({template}, 1, low_num);
end

function rotation = local_enu_rotation_from_attitude(attitude_deg)
roll = attitude_deg(1) * pi / 180;
pitch = attitude_deg(2) * pi / 180;
heading = attitude_deg(3) * pi / 180;
cbn = [cos(roll)*cos(heading)+sin(roll)*sin(pitch)*sin(heading), ...
    -cos(roll)*sin(heading)+sin(roll)*sin(pitch)*cos(heading), ...
    -sin(roll)*cos(pitch); ...
    cos(pitch)*sin(heading), cos(pitch)*cos(heading), sin(pitch); ...
    sin(roll)*cos(heading)-cos(roll)*sin(pitch)*sin(heading), ...
    -sin(roll)*sin(heading)-cos(roll)*sin(pitch)*cos(heading), ...
    cos(roll)*cos(pitch)];
rotation = cbn';
end

function detector_cfg = local_cusum_detector_config(cfg, estimator_mode, ...
        use_range_predictor_cusum)
% Keep the EKF and FGO range-predictor soft-weight policies independent.
% Both paths share the same online statistic and confirmed-alarm admission
% logic; the FGO-only option is needed because a graph can retain several
% current range factors for one follower in its sliding window.
detector_cfg = cfg;
if strcmp(estimator_mode, 'fgo') && use_range_predictor_cusum
    detector_cfg.ekf_cusum_soft_weight_mode = cfg.fgo_cusum_soft_weight_mode;
    detector_cfg.range_predictor_opposite_step_reset_enable = ...
        cfg.fgo_cusum_opposite_step_reset_enable;
    detector_cfg.range_predictor_opposite_step_reset_threshold = ...
        cfg.fgo_cusum_opposite_step_reset_threshold;
    detector_cfg.range_predictor_opposite_step_reset_epochs = ...
        cfg.fgo_cusum_opposite_step_reset_epochs;
end
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

function [follower_positions, follower_std, graph, state_diagnostics] = local_solve_sliding_window_imu_preint(window_history, cfg, low_num)
frame_count = numel(window_history);
graph = factor_graph_sliding_window_imu_preint(frame_count, cfg.uav_num, cfg.high_num, cfg);
prior_mode = local_imu_preint_prior_mode(cfg);
feedback_mode = local_imu_preint_feedback_mode(cfg);
prior_std = struct('position', cfg.imu_preint_position_prior_std, ...
    'velocity', cfg.imu_preint_velocity_prior_std, ...
    'rotation', cfg.imu_preint_rotation_prior_std, ...
    'gyro_bias', cfg.imu_preint_gyro_bias_prior_std, ...
    'acc_bias', cfg.imu_preint_acc_bias_prior_std);
state_diagnostics = local_empty_imu_state_diagnostics(low_num);
state_diagnostics.frame_times = reshape([window_history.time], [], 1);
state_diagnostics.initial_states = cell(frame_count, low_num);
state_diagnostics.optimized_states = cell(frame_count, low_num);
state_diagnostics.initial_from_carryover = false(frame_count, low_num);
for frame_index = 1:frame_count
    entry = window_history(frame_index);
    initial_velocity = zeros(3, low_num);
    initial_rotation = repmat(eye(3), 1, 1, low_num);
    initial_bg = zeros(3, low_num);
    initial_ba = zeros(3, low_num);
    initial_node_positions = entry.node_priors;
    for follower_index = 1:low_num
        node_index = cfg.high_num + follower_index;
        [initial_state, from_carryover] = local_initial_imu_follower_state(entry, node_index, ...
            follower_index, feedback_mode);
        initial_node_positions(:, node_index) = initial_state.p;
        initial_velocity(:, follower_index) = initial_state.v;
        initial_rotation(:, :, follower_index) = initial_state.R;
        initial_bg(:, follower_index) = initial_state.bg;
        initial_ba(:, follower_index) = initial_state.ba;
        state_diagnostics.initial_states{frame_index, follower_index} = initial_state;
        state_diagnostics.initial_from_carryover(frame_index, follower_index) = from_carryover;
    end
    graph.set_frame_initial(frame_index, initial_node_positions, initial_velocity, initial_rotation, initial_bg, initial_ba);
    for leader_index = 1:cfg.high_num
        graph.add_leader_prior(frame_index, leader_index, entry.node_priors(:, leader_index), [0.2; 0.2; 0.5]);
    end
    for follower_index = 1:low_num
        node_index = cfg.high_num + follower_index;
        prior_state = state_diagnostics.initial_states{frame_index, follower_index};
        % GPS remains a direct position observation at every frame.  In the
        % fixed mode, only the first active frame receives the SINS-derived
        % velocity, attitude and residual-bias prior; later frames are linked
        % by the IMU factor instead of receiving the same IMU information twice.
        prior_state.p = entry.node_priors(:, node_index);
        if strcmp(prior_mode, 'all_frames_full_legacy') || frame_index == 1
            graph.add_follower_prior(frame_index, follower_index, prior_state, prior_std);
            state_diagnostics.full_state_prior_count = state_diagnostics.full_state_prior_count + 1;
        else
            graph.add_follower_position_prior(frame_index, follower_index, prior_state.p, prior_std.position);
            state_diagnostics.position_prior_count = state_diagnostics.position_prior_count + 1;
        end
    end
    for range_index = 1:numel(entry.range_measurements)
        graph.add_range(frame_index, entry.range_nodes(1, range_index), ...
            entry.range_nodes(2, range_index), entry.range_measurements(range_index), ...
            cfg.sigma_dis, entry.range_weights(range_index));
    end
    if frame_index > 1
        for follower_index = 1:low_num
            graph.add_imu_factor(frame_index - 1, frame_index, follower_index, ...
                entry.preintegrations{follower_index});
        end
    end
end
graph.Gauss_Newton();
graph.covariance();
state_diagnostics.repropagation_count = graph.repropagation_count;
state_diagnostics.repropagation_seconds = graph.repropagation_seconds;
state_diagnostics.repropagation_per_factor = graph.repropagation_per_factor;
follower_positions = zeros(3, low_num);
follower_std = zeros(3, low_num);
for vehicle = 1:low_num
    node_index = cfg.high_num + vehicle;
    follower_positions(:, vehicle) = graph.get_position(frame_count, node_index);
    position_covariance = graph.get_position_covariance(frame_count, node_index);
    follower_std(:, vehicle) = sqrt(max(0, diag(position_covariance)));
end
for frame_index = 1:frame_count
    for follower_index = 1:low_num
        state_diagnostics.optimized_states{frame_index, follower_index} = ...
            graph.get_follower_state(frame_index, follower_index);
    end
end
end

function [state, from_carryover] = local_initial_imu_follower_state(entry, node_index, follower_index, feedback_mode)
state = struct('p', entry.node_priors(:, node_index), ...
    'v', entry.follower_velocity_prior(:, follower_index), ...
    'R', entry.follower_rotation_prior(:, :, follower_index), ...
    'bg', entry.follower_gyro_bias_prior(:, follower_index), ...
    'ba', entry.follower_acc_bias_prior(:, follower_index));
from_carryover = false;
if strcmp(feedback_mode, 'state_carryover') && entry.has_optimized_state && ...
        numel(entry.optimized_follower_states) >= follower_index && ...
        ~isempty(entry.optimized_follower_states{follower_index})
    state = entry.optimized_follower_states{follower_index};
    from_carryover = true;
end
end

function window_history = local_store_imu_window_states(window_history, state_diagnostics)
for frame_index = 1:numel(window_history)
    window_history(frame_index).optimized_follower_states = state_diagnostics.optimized_states(frame_index, :);
    window_history(frame_index).has_optimized_state = true;
end
end

function diagnostics = local_empty_imu_state_diagnostics(low_num)
diagnostics.frame_times = zeros(0, 1);
diagnostics.initial_states = cell(0, low_num);
diagnostics.optimized_states = cell(0, low_num);
diagnostics.initial_from_carryover = false(0, low_num);
diagnostics.full_state_prior_count = 0;
diagnostics.position_prior_count = 0;
diagnostics.repropagation_count = 0;
diagnostics.repropagation_seconds = 0;
diagnostics.repropagation_per_factor = zeros(0, 1);
end
function covariances = local_extract_current_follower_covariances(graph_solver, low_num, cfg)
if isa(graph_solver, 'factor_graph_sliding_window') || ...
        isa(graph_solver, 'factor_graph_sliding_window_imu_preint')
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
if ~any(valid)
    diagnostics.mean_iterations = NaN;
    diagnostics.max_iterations = NaN;
    diagnostics.nonconverged_count = 0;
    diagnostics.final_step_norm = NaN;
    return;
end
diagnostics.mean_iterations = mean(iterations(valid));
diagnostics.max_iterations = max(iterations(valid));
diagnostics.nonconverged_count = sum(~converged(valid));
diagnostics.final_step_norm = max(step_norms(valid));
end

function diagnostics = local_imu_preintegration_diagnostics(history)
seconds = [history.imu_preintegration_seconds]';
sample_counts = vertcat(history.imu_sample_counts);
delta_t = vertcat(history.imu_delta_t);
repropagation_counts = [history.imu_repropagation_count]';
repropagation_seconds = [history.imu_repropagation_seconds]';
diagnostics.total_seconds = sum(seconds(isfinite(seconds)));
diagnostics.mean_seconds_per_graph = mean(seconds(isfinite(seconds)));
diagnostics.mean_sample_count = mean(sample_counts(sample_counts > 0));
diagnostics.mean_delta_t = mean(delta_t(delta_t > 0));
diagnostics.max_sample_count = max([0; sample_counts(:)]);
diagnostics.repropagation_count = sum(repropagation_counts(isfinite(repropagation_counts)));
diagnostics.repropagation_seconds = sum(repropagation_seconds(isfinite(repropagation_seconds)));
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
history.imu_preintegration_seconds = 0;
history.imu_sample_counts = zeros(0, 1);
history.imu_delta_t = zeros(0, 1);
history.imu_state_diagnostics = local_empty_imu_state_diagnostics(0);
history.imu_full_state_prior_count = 0;
history.imu_position_prior_count = 0;
history.imu_repropagation_count = 0;
history.imu_repropagation_seconds = 0;
end

function metrics = local_metrics(result, cfg)
low_num = cfg.uav_num - cfg.high_num;
window_mask = local_fault_window_mask(result.time, result.fault_segments);
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

function mask = local_fault_window_mask(times, segments)
mask = false(size(times));
for segment_index = 1:numel(segments)
    mask = mask | (times >= segments(segment_index).start & ...
        times <= segments(segment_index).end);
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
segments = local_active_fault_segments(cfg);
target_window = false(size(times));
fault_window = false(size(times));
for segment_index = 1:numel(segments)
    target = sort(segments(segment_index).edge(:)');
    segment_window = times >= segments(segment_index).start & ...
        times <= segments(segment_index).end;
    target_window = target_window | (segment_window & ...
        pairs(:, 1) == target(1) & pairs(:, 2) == target(2));
    fault_window = fault_window | segment_window;
end
other_window = fault_window & ~target_window;
other_full = ~target_window;
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
diagnostics.fault_segment = local_fault_segment_diagnostics(segments, ...
    pairs, times, weights, alarm_entered, alarm_cleared);
diagnostics.detect_095_delay = local_finite_median( ...
    [diagnostics.fault_segment.detect_095_delay]);
diagnostics.detect_08_delay = local_finite_median( ...
    [diagnostics.fault_segment.detect_08_delay]);
diagnostics.recovery_095_delay = local_finite_median( ...
    [diagnostics.fault_segment.recovery_095_delay]);
diagnostics.weight_drop_095_delay = diagnostics.detect_095_delay;
diagnostics.weight_drop_08_delay = diagnostics.detect_08_delay;
diagnostics.alarm_delay = local_finite_median( ...
    [diagnostics.fault_segment.alarm_delay]);
diagnostics.alarm_recovery_delay = local_finite_median( ...
    [diagnostics.fault_segment.alarm_recovery_delay]);
end

function segment_diagnostics = local_fault_segment_diagnostics(segments, ...
        pairs, times, weights, alarm_entered, alarm_cleared)
template = struct('start', NaN, 'end', NaN, 'bias', NaN, ...
    'edge', [NaN, NaN], 'target_weight_median', NaN, ...
    'detect_095_delay', NaN, 'detect_08_delay', NaN, ...
    'recovery_095_delay', NaN, 'alarm_delay', NaN, ...
    'alarm_recovery_delay', NaN);
segment_diagnostics = repmat(template, numel(segments), 1);
for segment_index = 1:numel(segments)
    segment = segments(segment_index);
    target = sort(segment.edge(:)');
    target_mask = pairs(:, 1) == target(1) & pairs(:, 2) == target(2);
    target_window = target_mask & times >= segment.start & times <= segment.end;
    entry = template;
    entry.start = segment.start;
    entry.end = segment.end;
    entry.bias = segment.bias;
    entry.edge = target;
    if any(target_window)
        entry.target_weight_median = median(weights(target_window));
    end
    entry.detect_095_delay = local_first_threshold_crossing( ...
        times, target_mask, weights, 0.95, segment.start, segment.end, false);
    entry.detect_08_delay = local_first_threshold_crossing( ...
        times, target_mask, weights, 0.8, segment.start, segment.end, false);
    entry.recovery_095_delay = local_first_threshold_crossing( ...
        times, target_mask, weights, 0.95, segment.end, Inf, true);
    entry.alarm_delay = local_first_event_delay(times, target_mask, ...
        alarm_entered, segment.start, segment.end);
    entry.alarm_recovery_delay = local_first_event_delay(times, target_mask, ...
        alarm_cleared, segment.end, Inf);
    segment_diagnostics(segment_index) = entry;
end
end

function value = local_finite_median(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = median(values);
end
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
    if cfg.include_healthy_scenario
        names = {'healthy', 'fault'};
    else
        names = {'fault'};
    end
else
    names = {'healthy'};
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

function mode = local_estimator_mode(cfg)
mode = 'fgo';
if isfield(cfg, 'estimator_mode') && ~isempty(cfg.estimator_mode)
    mode = lower(char(cfg.estimator_mode));
end
if ~any(strcmp(mode, {'ekf', 'fgo'}))
    error('run_stage1_cusum_comparison:InvalidEstimatorMode', ...
        'estimator_mode must be ''ekf'' or ''fgo''.');
end
end

function mode = local_imu_preint_prior_mode(cfg)
mode = lower(char(cfg.imu_preint_prior_mode));
if ~any(strcmp(mode, {'first_frame_full', 'all_frames_full_legacy'}))
    error('run_stage1_cusum_comparison:InvalidImuPreintPriorMode', ...
        ['imu_preint_prior_mode must be ''first_frame_full'' or ' ...
        '''all_frames_full_legacy''.']);
end
end

function mode = local_imu_preint_feedback_mode(cfg)
mode = lower(char(cfg.imu_preint_feedback_mode));
if ~any(strcmp(mode, {'diagnostic', 'state_carryover'}))
    error('run_stage1_cusum_comparison:InvalidImuPreintFeedbackMode', ...
        ['imu_preint_feedback_mode must be ''diagnostic'' or ' ...
        '''state_carryover''.']);
end
end

function labels = local_selected_method_labels(method_names)
labels = cell(size(method_names));
for index = 1:numel(method_names)
    switch method_names{index}
        case 'ekf'
            labels{index} = 'EKF';
        case 'fgo'
            labels{index} = 'FGO';
        case 'cusum_ekf'
            labels{index} = 'CUSUM-EKF';
        case 'cusum_fgo'
            labels{index} = 'CUSUM-FGO';
    end
end
end

function local_print_progress(run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds)
fprintf('[%2d/%d ] scenario=%-7s method=%-9s seed=%-6g ... done (%.2f s)\n', ...
    run_index, total_runs, scenario_name, method_name, seed, elapsed_seconds);
end

function local_print_four_method_rmse(report, scenario_names)
fprintf('\nStage 1 four-method position RMSE (pooled across configured seeds)\n');
fprintf('  %-8s %-10s %-10s %12s %12s %12s %12s\n', ...
    'Scenario', 'Method', 'Follower', 'E RMSE(m)', 'N RMSE(m)', 'U RMSE(m)', '3D RMSE(m)');
for scenario_index = 1:numel(scenario_names)
    name = scenario_names{scenario_index};
    scenario_metrics = report.position_metrics.scenarios.(name);
    for method_index = 1:numel(report.method_names)
        method_name = report.method_names{method_index};
        method_metrics = scenario_metrics.(method_name);
        for follower = 1:numel(method_metrics.RMSE_E)
            fprintf('  %-8s %-10s Follower%-2d %12.6f %12.6f %12.6f %12.6f\n', ...
                name, report.method_labels{method_index}, follower, ...
                method_metrics.RMSE_E(follower), method_metrics.RMSE_N(follower), ...
                method_metrics.RMSE_U(follower), method_metrics.RMSE_3D(follower));
        end
    end
end
fprintf('\n');

if isfield(report.paired_comparisons.scenarios, 'fault') && ...
        ~isempty(report.paired_comparisons.pair_names)
    comparisons = report.paired_comparisons.scenarios.fault;
    pair_names = report.paired_comparisons.pair_names;
    if strcmpi(report.cfg.fault_mode, 'single')
        fprintf(['Fault-scenario paired 3D RMSE (mean of per-seed RMSE; ' ...
            'fault window %.3f--%.3f s)\n'], ...
            report.cfg.fault_start, report.cfg.fault_end);
    else
        fprintf(['Fault-scenario paired 3D RMSE (mean of per-seed RMSE; ' ...
            'combined configured fault segments)\n']);
    end
    fprintf('  %-21s %-10s %11s %11s %10s %11s %11s %10s\n', ...
        'Comparison', 'Follower', 'Base full', 'CUSUM full', 'Change(%)', ...
        'Base window', 'CUSUM window', 'Change(%)');
    for pair_index = 1:numel(pair_names)
        pair = comparisons.(pair_names{pair_index});
        for follower = 1:numel(pair.full.baseline_rmse_3d)
            fprintf('  %-21s Follower%-2d %11.6f %11.6f %10.2f %11.6f %11.6f %10.2f\n', ...
                pair.label, follower, pair.full.baseline_rmse_3d(follower), ...
                pair.full.cusum_rmse_3d(follower), pair.full.percent_change(follower), ...
                pair.fault_window.baseline_rmse_3d(follower), ...
                pair.fault_window.cusum_rmse_3d(follower), ...
                pair.fault_window.percent_change(follower));
        end
    end
    fprintf('\n');
end
end


function aggregate = local_aggregate_four_methods(seed_results, method_names)
aggregate = struct();
for method_index = 1:numel(method_names)
    method_name = method_names{method_index};
    seed_count = numel(seed_results);
    low_num = numel(seed_results(1).(method_name).metrics.full_rmse_3d);
    full_values = zeros(seed_count, low_num);
    window_values = nan(seed_count, low_num);
    for seed_index = 1:seed_count
        full_values(seed_index, :) = seed_results(seed_index).(method_name).metrics.full_rmse_3d';
        window_values(seed_index, :) = seed_results(seed_index).(method_name).metrics.window_rmse_3d';
    end
    aggregate.(method_name).full_rmse_mean = mean(full_values, 1)';
    aggregate.(method_name).full_rmse_std = std(full_values, 0, 1)';
    aggregate.(method_name).window_rmse_mean = mean(window_values, 1, 'omitnan')';
    aggregate.(method_name).window_rmse_std = std(window_values, 0, 1, 'omitnan')';
end
end

function segment_metrics = local_fault_segment_position_metrics( ...
        scenario_results, method_names)
segment_metrics = struct([]);
if ~isfield(scenario_results, 'fault')
    return;
end
results = scenario_results.fault;
template = struct('seed', NaN, 'segments', struct([]), 'methods', struct());
segment_metrics = repmat(template, numel(results), 1);
for seed_index = 1:numel(results)
    reference = results(seed_index).(method_names{1});
    segments = reference.fault_segments;
    entry = template;
    entry.seed = results(seed_index).seed;
    entry.segments = segments;
    for method_index = 1:numel(method_names)
        method_name = method_names{method_index};
        result = results(seed_index).(method_name);
        low_num = size(result.error_xyz, 3);
        rmse_3d = nan(numel(segments), low_num);
        for segment_index = 1:numel(segments)
            mask = result.time >= segments(segment_index).start & ...
                result.time <= segments(segment_index).end;
            for follower = 1:low_num
                squared_error = sum(result.error_xyz(mask, :, follower).^2, 2);
                rmse_3d(segment_index, follower) = sqrt(mean(squared_error));
            end
        end
        entry.methods.(method_name).rmse_3d = rmse_3d;
    end
    segment_metrics(seed_index) = entry;
end
end

function paired = local_four_method_paired_comparisons(scenario_aggregates, scenario_names)
paired.pair_names = cell(0, 1);
pair_definitions = {
    'cusum_ekf_vs_ekf', 'EKF -> CUSUM-EKF', 'ekf', 'cusum_ekf';
    'cusum_fgo_vs_fgo', 'FGO -> CUSUM-FGO', 'fgo', 'cusum_fgo'
};
available_methods = fieldnames(scenario_aggregates.(scenario_names{1}));
for pair_index = 1:size(pair_definitions, 1)
    if ismember(pair_definitions{pair_index, 3}, available_methods) && ...
            ismember(pair_definitions{pair_index, 4}, available_methods)
        paired.pair_names{end + 1, 1} = pair_definitions{pair_index, 1}; %#ok<AGROW>
    end
end
paired.scenarios = struct();
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    aggregate = scenario_aggregates.(scenario_name);
    scenario_comparisons = struct();
    for pair_index = 1:size(pair_definitions, 1)
        pair_name = pair_definitions{pair_index, 1};
        if ~ismember(pair_name, paired.pair_names)
            continue;
        end
        baseline_name = pair_definitions{pair_index, 3};
        cusum_name = pair_definitions{pair_index, 4};
        baseline = aggregate.(baseline_name);
        cusum = aggregate.(cusum_name);

        comparison.label = pair_definitions{pair_index, 2};
        comparison.baseline_method = baseline_name;
        comparison.cusum_method = cusum_name;
        comparison.full.baseline_rmse_3d = baseline.full_rmse_mean;
        comparison.full.cusum_rmse_3d = cusum.full_rmse_mean;
        comparison.full.percent_change = local_percent_change_vector( ...
            cusum.full_rmse_mean, baseline.full_rmse_mean);
        comparison.fault_window.baseline_rmse_3d = baseline.window_rmse_mean;
        comparison.fault_window.cusum_rmse_3d = cusum.window_rmse_mean;
        comparison.fault_window.percent_change = local_percent_change_vector( ...
            cusum.window_rmse_mean, baseline.window_rmse_mean);
        scenario_comparisons.(pair_name) = comparison;
    end
    paired.scenarios.(scenario_name) = scenario_comparisons;
end
end

function percent = local_percent_change_vector(value, reference)
percent = nan(size(value));
valid = isfinite(value) & isfinite(reference) & abs(reference) > eps;
percent(valid) = 100 * (value(valid) - reference(valid)) ./ reference(valid);
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
if cfg.uav_num ~= 6 || cfg.high_num ~= 3 || low_num ~= 3
    error('run_stage1_cusum_comparison:UnsupportedScenario', ...
        'The streamlined runner supports only the 3-follower/3-leader scenario.');
end
if ~isscalar(cfg.include_healthy_scenario) || ...
        (~islogical(cfg.include_healthy_scenario) && ...
        ~ismember(cfg.include_healthy_scenario, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidHealthyScenarioSwitch', ...
        'include_healthy_scenario must be logical or 0/1.');
end
valid_methods = {'ekf', 'fgo', 'cusum_ekf', 'cusum_fgo'};
if ~iscell(cfg.methods) || isempty(cfg.methods) || ...
        ~all(cellfun(@(name) ischar(name) || isstring(name), cfg.methods)) || ...
        any(~ismember(cellfun(@char, cfg.methods, 'UniformOutput', false), valid_methods)) || ...
        numel(unique(cellfun(@char, cfg.methods, 'UniformOutput', false))) ~= numel(cfg.methods)
    error('run_stage1_cusum_comparison:InvalidMethods', ...
        'methods must be a non-empty unique subset of EKF/FGO/CUSUM-EKF/CUSUM-FGO.');
end
if ~isnumeric(cfg.base_leader_source_indices) || ...
        ~isequal(size(cfg.base_leader_source_indices), [1, 3]) || ...
        any(~isfinite(cfg.base_leader_source_indices)) || ...
        any(cfg.base_leader_source_indices < 1) || ...
        any(cfg.base_leader_source_indices ~= floor(cfg.base_leader_source_indices))
    error('run_stage1_cusum_comparison:InvalidLeaderSources', ...
        'base_leader_source_indices must be a row vector containing three positive integers.');
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
if ~ischar(cfg.fault_mode) && ~isstring(cfg.fault_mode)
    error('run_stage1_cusum_comparison:InvalidFaultMode', ...
        ['fault_mode must be ''single'', ''segmented_random_edge'', or ', ...
        '''segmented_explicit_edges''.']);
end
fault_mode = lower(char(cfg.fault_mode));
if ~any(strcmp(fault_mode, {'single', 'segmented_random_edge', ...
        'segmented_explicit_edges'}))
    error('run_stage1_cusum_comparison:InvalidFaultMode', ...
        ['fault_mode must be ''single'', ''segmented_random_edge'', or ', ...
        '''segmented_explicit_edges''.']);
end
if strcmp(fault_mode, 'single')
    if numel(cfg.fault_edge) ~= 2 || any(cfg.fault_edge < 1) || ...
            any(cfg.fault_edge > cfg.uav_num) || cfg.fault_edge(1) == cfg.fault_edge(2)
        error('run_stage1_cusum_comparison:InvalidFaultEdge', ...
            'fault_edge must identify two different global UAVs.');
    end
    if cfg.fault_enable && (cfg.fault_start < 0 || ...
            cfg.fault_end <= cfg.fault_start || cfg.fault_end > cfg.t_stop)
        error('run_stage1_cusum_comparison:InvalidFaultWindow', ...
            'For an enabled fault, require 0 <= fault_start < fault_end <= t_stop.');
    end
else
    explicit_edges = strcmp(fault_mode, 'segmented_explicit_edges');
    expected_columns = 3 + 2 * explicit_edges;
    if ~isnumeric(cfg.fault_segments) || ...
            size(cfg.fault_segments, 2) ~= expected_columns || ...
            isempty(cfg.fault_segments) || any(~isfinite(cfg.fault_segments(:))) || ...
            any(cfg.fault_segments(:, 1) < 0) || ...
            any(cfg.fault_segments(:, 2) <= cfg.fault_segments(:, 1)) || ...
            any(cfg.fault_segments(:, 2) > cfg.t_stop) || ...
            any(cfg.fault_segments(:, 3) == 0)
        if explicit_edges
            description = ['fault_segments must be rows [start, end, bias, ', ...
                'follower, leader_global] inside the simulation interval.'];
        else
            description = ['fault_segments must be rows [start, end, bias] ', ...
                'inside the simulation interval.'];
        end
        error('run_stage1_cusum_comparison:InvalidFaultSegments', description);
    end
    if explicit_edges
        followers = cfg.fault_segments(:, 4);
        leaders = cfg.fault_segments(:, 5);
        if any(followers ~= floor(followers)) || ...
                any(leaders ~= floor(leaders)) || any(followers < 1) || ...
                any(followers > low_num) || any(leaders <= low_num) || ...
                any(leaders > cfg.uav_num)
            error('run_stage1_cusum_comparison:InvalidExplicitFaultEdge', ...
                ['Explicit fault rows must use a valid follower index and a ', ...
                'valid global leader index.']);
        end
    elseif any(cfg.fault_segments(2:end, 1) <= cfg.fault_segments(1:end-1, 2))
        error('run_stage1_cusum_comparison:InvalidFaultSegments', ...
            ['Segmented random fault rows must be ordered and non-overlapping ', ...
            'with a healthy gap between rows.']);
    end
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
if (~ischar(cfg.ekf_cusum_soft_weight_mode) && ...
        ~isstring(cfg.ekf_cusum_soft_weight_mode)) || ...
        ~any(strcmpi(char(cfg.ekf_cusum_soft_weight_mode), ...
        {'all', 'per_follower_max'}))
    error('run_stage1_cusum_comparison:InvalidEkfCusumSoftWeightMode', ...
        'ekf_cusum_soft_weight_mode must be ''all'' or ''per_follower_max''.');
end
if (~ischar(cfg.fgo_cusum_soft_weight_mode) && ...
        ~isstring(cfg.fgo_cusum_soft_weight_mode)) || ...
        ~any(strcmpi(char(cfg.fgo_cusum_soft_weight_mode), ...
        {'all', 'per_follower_max'}))
    error('run_stage1_cusum_comparison:InvalidFgoCusumSoftWeightMode', ...
        'fgo_cusum_soft_weight_mode must be ''all'' or ''per_follower_max''.');
end
if ~isscalar(cfg.fgo_cusum_opposite_step_reset_enable) || ...
        (~islogical(cfg.fgo_cusum_opposite_step_reset_enable) && ...
        ~ismember(cfg.fgo_cusum_opposite_step_reset_enable, [0, 1])) || ...
        ~isscalar(cfg.fgo_cusum_opposite_step_reset_threshold) || ...
        ~isfinite(cfg.fgo_cusum_opposite_step_reset_threshold) || ...
        cfg.fgo_cusum_opposite_step_reset_threshold <= 0 || ...
        ~isscalar(cfg.fgo_cusum_opposite_step_reset_epochs) || ...
        cfg.fgo_cusum_opposite_step_reset_epochs < 1 || ...
        cfg.fgo_cusum_opposite_step_reset_epochs ~= ...
        floor(cfg.fgo_cusum_opposite_step_reset_epochs)
    error('run_stage1_cusum_comparison:InvalidFgoCusumRecovery', ...
        'FGO predictor recovery settings must be finite positive scalars.');
end
if ~isscalar(cfg.ekf_cusum_alarm_surrogate_enable) || ...
        (~islogical(cfg.ekf_cusum_alarm_surrogate_enable) && ...
        ~ismember(cfg.ekf_cusum_alarm_surrogate_enable, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidEkfCusumSurrogateEnable', ...
        'ekf_cusum_alarm_surrogate_enable must be logical or 0/1.');
end
if ~isscalar(cfg.ekf_cusum_alarm_surrogate_weight) || ...
        ~isreal(cfg.ekf_cusum_alarm_surrogate_weight) || ...
        ~isfinite(cfg.ekf_cusum_alarm_surrogate_weight) || ...
        cfg.ekf_cusum_alarm_surrogate_weight <= 0 || ...
        cfg.ekf_cusum_alarm_surrogate_weight > 1
    error('run_stage1_cusum_comparison:InvalidEkfCusumSurrogateWeight', ...
        'ekf_cusum_alarm_surrogate_weight must be a finite scalar in (0, 1].');
end
if (~ischar(cfg.fgo_cusum_detector_mode) && ...
        ~isstring(cfg.fgo_cusum_detector_mode)) || ...
        ~any(strcmpi(char(cfg.fgo_cusum_detector_mode), ...
        {'prior_innovation', 'range_predictor'}))
    error('run_stage1_cusum_comparison:InvalidFgoCusumDetectorMode', ...
        ['fgo_cusum_detector_mode must be ''prior_innovation'' or ', ...
        '''range_predictor''.']);
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
if ~isscalar(cfg.imu_preintegration_enable) || ...
        (~islogical(cfg.imu_preintegration_enable) && ...
        ~ismember(cfg.imu_preintegration_enable, [0, 1]))
    error('run_stage1_cusum_comparison:InvalidImuPreintegrationEnable', ...
        'imu_preintegration_enable must be logical or 0/1.');
end
if ~isscalar(cfg.imu_preint_window_length) || ...
        cfg.imu_preint_window_length < 1 || ...
        cfg.imu_preint_window_length ~= floor(cfg.imu_preint_window_length)
    error('run_stage1_cusum_comparison:InvalidImuPreintWindowLength', ...
        'imu_preint_window_length must be a positive integer.');
end
positive_scalar_fields = {'imu_preint_gyro_bias_repropagate_threshold', ...
    'imu_preint_acc_bias_repropagate_threshold', ...
    'imu_preint_gyro_noise_std', 'imu_preint_gyro_bias_rw_std', ...
    'imu_preint_acc_bias_rw_std', 'imu_preint_gn_step_tolerance', ...
    'imu_preint_covariance_regularization', 'imu_preint_position_eps', ...
    'imu_preint_velocity_eps', 'imu_preint_rotation_eps', ...
    'imu_preint_gyro_bias_eps', 'imu_preint_acc_bias_eps'};
for field_index = 1:numel(positive_scalar_fields)
    value = cfg.(positive_scalar_fields{field_index});
    if ~isscalar(value) || ~isreal(value) || ~isfinite(value) || value <= 0
        error('run_stage1_cusum_comparison:InvalidImuPreintParameter', ...
            '%s must be a positive finite scalar.', ...
            positive_scalar_fields{field_index});
    end
end
if ~isscalar(cfg.imu_preint_acc_noise_std) || ...
        ~isreal(cfg.imu_preint_acc_noise_std) || ...
        ~isfinite(cfg.imu_preint_acc_noise_std) || ...
        cfg.imu_preint_acc_noise_std < 0
    error('run_stage1_cusum_comparison:InvalidImuPreintParameter', ...
        'imu_preint_acc_noise_std must be a non-negative finite scalar.');
end
if ~isscalar(cfg.imu_preint_gn_max_iterations) || ...
        cfg.imu_preint_gn_max_iterations < 1 || ...
        cfg.imu_preint_gn_max_iterations ~= ...
        floor(cfg.imu_preint_gn_max_iterations)
    error('run_stage1_cusum_comparison:InvalidImuPreintIterations', ...
        'imu_preint_gn_max_iterations must be a positive integer.');
end
positive_vector_fields = {'imu_preint_position_prior_std', ...
    'imu_preint_velocity_prior_std', 'imu_preint_rotation_prior_std', ...
    'imu_preint_gyro_bias_prior_std', 'imu_preint_acc_bias_prior_std'};
for field_index = 1:numel(positive_vector_fields)
    value = cfg.(positive_vector_fields{field_index});
    if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 || ...
            any(~isfinite(value(:))) || any(value(:) <= 0)
        error('run_stage1_cusum_comparison:InvalidImuPreintPriorStd', ...
            '%s must be a positive finite three-element vector.', ...
            positive_vector_fields{field_index});
    end
end
if ~any(strcmpi(char(cfg.imu_preint_covariance_mode), ...
        {'full_pinv_legacy', 'current_frame_only'}))
    error('run_stage1_cusum_comparison:InvalidImuPreintCovarianceMode', ...
        ['imu_preint_covariance_mode must be ''full_pinv_legacy'' or ' ...
        '''current_frame_only''.']);
end
local_imu_preint_prior_mode(cfg);
local_imu_preint_feedback_mode(cfg);
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
local_graph_mode(cfg);
local_estimator_mode(cfg);
end
