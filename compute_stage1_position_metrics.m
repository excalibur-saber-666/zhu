function metrics = compute_stage1_position_metrics(report)
%COMPUTE_STAGE1_POSITION_METRICS E/N/U/3D RMSE for a four-method report.
%   Errors use the single convention estimate - truth.  For multiple seeds,
%   each RMSE is pooled over all samples and all configured seeds.

if ~isstruct(report) || ~isfield(report, 'comparison_mode') || ...
        ~strcmp(report.comparison_mode, 'four_method') || ...
        ~isfield(report, 'scenario_results') || ~isfield(report, 'method_names')
    error('compute_stage1_position_metrics:InvalidReport', ...
        'report must be produced by run_stage1_kf_fgo_cusum_comparison.');
end

metrics.method_names = report.method_names;
metrics.method_labels = report.method_labels;
scenario_names = fieldnames(report.scenario_results);
metrics.scenario_names = scenario_names;
metrics.primary_scenario = scenario_names{end};
metrics.scenarios = struct();
for scenario_index = 1:numel(scenario_names)
    scenario_name = scenario_names{scenario_index};
    results = report.scenario_results.(scenario_name);
    scenario_metrics = struct();
    for method_index = 1:numel(report.method_names)
        method_name = report.method_names{method_index};
        reference_error = results(1).(method_name).error_xyz;
        low_num = size(reference_error, 3);
        rmse_e = zeros(low_num, 1);
        rmse_n = zeros(low_num, 1);
        rmse_u = zeros(low_num, 1);
        rmse_3d = zeros(low_num, 1);
        for follower = 1:low_num
            errors = zeros(0, 3);
            for seed_index = 1:numel(results)
                error_xyz = results(seed_index).(method_name).error_xyz(:, :, follower);
                if ~isequal(size(error_xyz, 2), 3) || any(~isfinite(error_xyz(:)))
                    error('compute_stage1_position_metrics:InvalidResult', ...
                        'Every four-method position error must be finite N-by-3 data.');
                end
                errors = [errors; error_xyz]; %#ok<AGROW>
            end
            rmse_e(follower) = sqrt(mean(errors(:, 1).^2));
            rmse_n(follower) = sqrt(mean(errors(:, 2).^2));
            rmse_u(follower) = sqrt(mean(errors(:, 3).^2));
            rmse_3d(follower) = sqrt(mean(sum(errors.^2, 2)));
        end
        scenario_metrics.(method_name) = struct('RMSE_E', rmse_e, ...
            'RMSE_N', rmse_n, 'RMSE_U', rmse_u, 'RMSE_3D', rmse_3d);
    end
    metrics.scenarios.(scenario_name) = scenario_metrics;
end
metrics.primary = metrics.scenarios.(metrics.primary_scenario);
end
