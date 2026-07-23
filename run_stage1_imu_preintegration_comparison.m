function report = run_stage1_imu_preintegration_comparison(cfg)
%RUN_STAGE1_IMU_PREINTEGRATION_COMPARISON Compare original, SINS, and IMU-preint graphs.
%   All methods reuse the same deterministic seed, trajectory, IMU, GPS,
%   range-noise and fault-injection generation in run_stage1_cusum_comparison.
%   The original branch is checked bitwise across the two calls.

if nargin < 1 || isempty(cfg)
    cfg = stage1_cusum_3f3l_config('full');
end
plot_requested = isfield(cfg, 'plot_component_comparison') && cfg.plot_component_comparison;
cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = false;

sins_cfg = cfg;
sins_cfg.sliding_window_motion_model = 'sins_delta';
start_time = tic;
sins_report = run_stage1_cusum_comparison(sins_cfg);
sins_elapsed_seconds = toc(start_time);

preint_cfg = cfg;
preint_cfg.sliding_window_motion_model = 'imu_preint';
start_time = tic;
preint_report = run_stage1_cusum_comparison(preint_cfg);
preint_elapsed_seconds = toc(start_time);

local_assert_shared_baseline(sins_report, preint_report);
report.cfg = cfg;
report.original = sins_report;
report.sw_cusum_sins = sins_report;
report.sw_cusum_preint = preint_report;
report.total_runtime_seconds = struct('sw_cusum_sins', sins_elapsed_seconds, ...
    'sw_cusum_preint', preint_elapsed_seconds);
if cfg.verbose
    local_print_tables(report);
end
if plot_requested
    local_plot_three_method_errors(report);
end
end

function local_assert_shared_baseline(first, second)
scenario_names = fieldnames(first.scenario_results);
for scenario_index = 1:numel(scenario_names)
    name = scenario_names{scenario_index};
    for seed_index = 1:numel(first.scenario_results.(name))
        left = first.scenario_results.(name)(seed_index).equal.navigation_xyz;
        right = second.scenario_results.(name)(seed_index).equal.navigation_xyz;
        if ~isequal(left, right)
            error('run_stage1_imu_preintegration_comparison:BaselineMismatch', ...
                'Original Equal-FGO did not receive identical cached inputs across methods.');
        end
    end
end
end

function local_print_tables(report)
scenario_names = fieldnames(report.original.scenario_aggregates);
fprintf('\nIMU preintegration comparison (same seeds and cached-input sequence)\n');
fprintf('  Motion models        : original, SINS position delta, IMU preintegration\n');
fprintf('  IMU-preint window    : %d key frames\n', report.cfg.imu_preint_window_length);
fprintf('  %-8s %-18s %-10s %14s %22s\n', 'Scenario', 'Method', 'Follower', 'Full RMSE(m)', 'Fault-window RMSE(m)');
for scenario_index = 1:numel(scenario_names)
    name = scenario_names{scenario_index};
    original = report.original.scenario_aggregates.(name);
    sins = report.sw_cusum_sins.scenario_aggregates.(name);
    preint = report.sw_cusum_preint.scenario_aggregates.(name);
    for follower = 1:numel(original.equal_full_mean)
        fprintf('  %-8s %-18s Follower%-2d %14.6f %22.6f\n', name, 'original', follower, ...
            original.equal_full_mean(follower), original.equal_window_mean(follower));
        fprintf('  %-8s %-18s Follower%-2d %14.6f %22.6f\n', name, 'sw_cusum_sins', follower, ...
            sins.cusum_full_mean(follower), sins.cusum_window_mean(follower));
        fprintf('  %-8s %-18s Follower%-2d %14.6f %22.6f\n', name, 'sw_cusum_preint', follower, ...
            preint.cusum_full_mean(follower), preint.cusum_window_mean(follower));
    end
end
preint_result = local_primary_results(report.sw_cusum_preint);
fprintf('\nRuntime diagnostics\n');
fprintf('  SINS comparison runtime (s) : %.3f\n', report.total_runtime_seconds.sw_cusum_sins);
fprintf('  Preint comparison runtime(s) : %.3f\n', report.total_runtime_seconds.sw_cusum_preint);
fprintf('  Preintegration total (s)     : %.3f\n', preint_result.imu_preintegration_diagnostics.total_seconds);
fprintf('  Mean IMU samples / interval  : %.2f\n', preint_result.imu_preintegration_diagnostics.mean_sample_count);
fprintf('  Mean preintegrated dt (s)    : %.6f\n', preint_result.imu_preintegration_diagnostics.mean_delta_t);
fprintf('  Mean GN iterations            : %.3f\n', preint_result.gn_diagnostics.mean_iterations);
fprintf('  Non-converged graph epochs    : %d\n\n', preint_result.gn_diagnostics.nonconverged_count);
end

function result = local_primary_results(comparison)
names = fieldnames(comparison.scenario_results);
if isfield(comparison.scenario_results, 'fault')
    name = 'fault';
else
    name = names{1};
end
result = comparison.scenario_results.(name)(1).cusum;
end

function local_plot_three_method_errors(report)
scenario_names = fieldnames(report.original.scenario_results);
for scenario_index = 1:numel(scenario_names)
    name = scenario_names{scenario_index};
    original = report.original.scenario_results.(name);
    sins = report.sw_cusum_sins.scenario_results.(name);
    preint = report.sw_cusum_preint.scenario_results.(name);
    seed_count = numel(original); low_num = size(original(1).equal.error_xyz, 3);
    for follower = 1:low_num
        errors = zeros(size(original(1).equal.error_xyz, 1), 3, 3);
        for seed_index = 1:seed_count
            errors(:, :, 1) = errors(:, :, 1) + original(seed_index).equal.error_xyz(:, :, follower);
            errors(:, :, 2) = errors(:, :, 2) + sins(seed_index).cusum.error_xyz(:, :, follower);
            errors(:, :, 3) = errors(:, :, 3) + preint(seed_index).cusum.error_xyz(:, :, follower);
        end
        errors = errors / seed_count; time = original(1).equal.time;
        figure('Name', sprintf('Follower%d: original vs SINS vs IMU preint (%s)', follower, name), 'NumberTitle', 'off');
        labels = {'East error (m)', 'North error (m)', 'Up error (m)'};
        for axis_index = 1:3
            subplot(3, 1, axis_index);
            plot(time, errors(:, axis_index, 1), 'b-', time, errors(:, axis_index, 2), 'r-', ...
                time, errors(:, axis_index, 3), 'k-', 'LineWidth', 1.0); grid on;
            if report.cfg.fault_enable, xline(report.cfg.fault_start, 'k--', 'HandleVisibility', 'off'); xline(report.cfg.fault_end, 'k--', 'HandleVisibility', 'off'); end
            ylabel(labels{axis_index});
            if axis_index == 1, legend('Original FGO', 'SW + CUSUM + SINS', 'SW + CUSUM + IMU preint', 'Location', 'best'); end
            if axis_index == 3, xlabel('Time (s)'); end
        end
    end
end
end
