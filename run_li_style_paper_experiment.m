function summary = run_li_style_paper_experiment(seed_list, representative_seed, output_directory, profile)
%RUN_LI_STYLE_PAPER_EXPERIMENT Run the Li-style Monte Carlo experiment.
%   SUMMARY = RUN_LI_STYLE_PAPER_EXPERIMENT(SEED_LIST,
%   REPRESENTATIVE_SEED, OUTPUT_DIRECTORY, PROFILE) runs the four-method comparison,
%   keeps compact source data, writes CSV/XLSX statistics, and generates
%   MATLAB figures analogous to Li's simulation trajectory, pseudorange
%   error, positioning-error, and positioning-error CDF figures.  PROFILE
%   defaults to 'li_style_dense'; 'li_style_dense_tuned' is an opt-in
%   CUSUM-FGO soft-weighting variant, while
%   'li_style_dense_fgo_recovered' additionally enables online predictor
%   recovery for a cleared step fault.

if nargin < 1 || isempty(seed_list)
    seed_list = 1:50;
end
if nargin < 2 || isempty(representative_seed)
    representative_seed = 23;
end
if nargin < 4 || isempty(profile)
    profile = 'li_style_dense';
end
if nargin < 3 || isempty(output_directory)
    if strcmpi(char(profile), 'li_style_dense_tuned')
        output_directory = fullfile(stage1_project_root(), 'li_style_mc50_tuned_results');
    elseif strcmpi(char(profile), 'li_style_dense_fgo_recovered')
        output_directory = fullfile(stage1_project_root(), ...
            'li_style_mc50_recovered_results');
    else
        output_directory = fullfile(stage1_project_root(), 'li_style_mc50_results');
    end
end

setup_project();
seed_list = seed_list(:)';
if ~isnumeric(seed_list) || isempty(seed_list) || any(~isfinite(seed_list)) || ...
        any(seed_list < 0) || any(seed_list ~= floor(seed_list)) || ...
        numel(unique(seed_list)) ~= numel(seed_list)
    error('run_li_style_paper_experiment:InvalidSeeds', ...
        'seed_list must contain unique nonnegative integer seeds.');
end
if ~isscalar(representative_seed) || ~isfinite(representative_seed) || ...
        representative_seed < 0 || representative_seed ~= floor(representative_seed)
    error('run_li_style_paper_experiment:InvalidRepresentativeSeed', ...
        'representative_seed must be a nonnegative integer scalar.');
end
if ~isfolder(output_directory)
    mkdir(output_directory);
end

cfg = stage1_cusum_3f3l_config(profile);
cfg.plot_position_results = false;
cfg.verbose = false;
method_names = {'ekf', 'fgo', 'cusum_ekf', 'cusum_fgo'};
method_labels = {'EKF', 'FGO', 'CUSUM-EKF', 'CUSUM-FGO'};
low_num = cfg.uav_num - cfg.high_num;
method_count = numel(method_names);
seed_count = numel(seed_list);
sample_count = round(cfg.t_stop / cfg.dt) + 1;

pooled_error_3d = zeros(seed_count * sample_count, method_count, low_num, 'single');
per_seed_rmse_3d = nan(seed_count, method_count, low_num);
per_seed_fault_rmse_3d = nan(seed_count, method_count, low_num);
per_seed_mean_error_3d = nan(seed_count, method_count, low_num);
per_seed_max_error_3d = nan(seed_count, method_count, low_num);
per_seed_cdf95_3d = nan(seed_count, method_count, low_num);
schedule_seed = zeros(seed_count * size(cfg.fault_segments, 1), 1);
schedule_segment = zeros(size(schedule_seed));
schedule_start_s = zeros(size(schedule_seed));
schedule_end_s = zeros(size(schedule_seed));
schedule_bias_m = zeros(size(schedule_seed));
schedule_follower = zeros(size(schedule_seed));
schedule_leader = zeros(size(schedule_seed));
representative = struct([]);

experiment_timer = tic;
for seed_index = 1:seed_count
    seed = seed_list(seed_index);
    seed_cfg = cfg;
    seed_cfg.seeds = seed;
    seed_report = main_stage1('four_method', seed_cfg);
    fault_result = seed_report.scenario_results.fault(1);
    segments = seed_report.fault_schedules{1};
    time = fault_result.(method_names{1}).time;
    fault_mask = local_fault_mask(time, segments);
    pooled_rows = (seed_index - 1) * sample_count + (1:sample_count);

    for method_index = 1:method_count
        result = fault_result.(method_names{method_index});
        error_3d = squeeze(sqrt(sum(result.error_xyz.^2, 2)));
        pooled_error_3d(pooled_rows, method_index, :) = ...
            reshape(single(error_3d), [sample_count, 1, low_num]);
        per_seed_rmse_3d(seed_index, method_index, :) = ...
            sqrt(mean(error_3d.^2, 1));
        per_seed_fault_rmse_3d(seed_index, method_index, :) = ...
            sqrt(mean(error_3d(fault_mask, :).^2, 1));
        per_seed_mean_error_3d(seed_index, method_index, :) = ...
            mean(error_3d, 1);
        per_seed_max_error_3d(seed_index, method_index, :) = ...
            max(error_3d, [], 1);
        per_seed_cdf95_3d(seed_index, method_index, :) = ...
            prctile(error_3d, 95, 1);
    end

    schedule_rows = (seed_index - 1) * numel(segments) + (1:numel(segments));
    for segment_index = 1:numel(segments)
        segment = segments(segment_index);
        schedule_seed(schedule_rows(segment_index)) = seed;
        schedule_segment(schedule_rows(segment_index)) = segment_index;
        schedule_start_s(schedule_rows(segment_index)) = segment.start;
        schedule_end_s(schedule_rows(segment_index)) = segment.end;
        schedule_bias_m(schedule_rows(segment_index)) = segment.bias;
        schedule_follower(schedule_rows(segment_index)) = segment.edge(1);
        schedule_leader(schedule_rows(segment_index)) = segment.edge(2) - low_num;
    end

    if seed == representative_seed
        representative = local_compact_representative(seed_report, method_names, method_labels);
    end
    fprintf('Monte Carlo seed %d/%d (seed=%d) completed, elapsed %.1f s\n', ...
        seed_index, seed_count, seed, toc(experiment_timer));
    clear seed_report fault_result;
end

if isempty(representative)
    representative_cfg = cfg;
    representative_cfg.seeds = representative_seed;
    representative_report = main_stage1('four_method', representative_cfg);
    representative = local_compact_representative( ...
        representative_report, method_names, method_labels);
    clear representative_report;
end

fault_schedule_table = table(schedule_seed, schedule_segment, ...
    schedule_start_s, schedule_end_s, schedule_bias_m, ...
    schedule_follower, schedule_leader, ...
    'VariableNames', {'Seed', 'Segment', 'Start_s', 'End_s', ...
    'Bias_m', 'Follower', 'Leader'});
performance_table = local_performance_table(pooled_error_3d, ...
    per_seed_rmse_3d, per_seed_fault_rmse_3d, method_labels);
improvement_table = local_improvement_table(per_seed_rmse_3d, ...
    per_seed_fault_rmse_3d, method_labels);
per_seed_table = local_per_seed_table(seed_list, per_seed_rmse_3d, ...
    per_seed_fault_rmse_3d, per_seed_mean_error_3d, ...
    per_seed_max_error_3d, per_seed_cdf95_3d, method_labels);
parameter_table = local_parameter_table(cfg, seed_list);
[cdf_probability, cdf_error_m] = local_cdf_curves( ...
    pooled_error_3d, method_count, low_num);
[mc_timewise_mean_error_3d, mc_timewise_std_error_3d, ...
    mc_error_statistics_table] = local_mc_figure_statistics( ...
    pooled_error_3d, sample_count, seed_count, method_labels);
[per_seed_segment_target_rmse_3d, segment_performance_table, ...
    segment_improvement_table] = compute_li_style_segment_performance( ...
    pooled_error_3d, fault_schedule_table, cfg, seed_list, method_labels);

summary = struct();
summary.cfg = cfg;
summary.seed_list = seed_list;
summary.representative_seed = representative_seed;
summary.method_names = method_names;
summary.method_labels = method_labels;
summary.representative = representative;
summary.pooled_error_3d = pooled_error_3d;
summary.per_seed_rmse_3d = per_seed_rmse_3d;
summary.per_seed_fault_rmse_3d = per_seed_fault_rmse_3d;
summary.per_seed_mean_error_3d = per_seed_mean_error_3d;
summary.per_seed_max_error_3d = per_seed_max_error_3d;
summary.per_seed_cdf95_3d = per_seed_cdf95_3d;
summary.per_seed_segment_target_rmse_3d = ...
    per_seed_segment_target_rmse_3d;
summary.cdf_probability = cdf_probability;
summary.cdf_error_m = cdf_error_m;
summary.mc_timewise_mean_error_3d = mc_timewise_mean_error_3d;
summary.mc_timewise_std_error_3d = mc_timewise_std_error_3d;
summary.mc_error_statistics_table = mc_error_statistics_table;
summary.parameter_table = parameter_table;
summary.performance_table = performance_table;
summary.improvement_table = improvement_table;
summary.per_seed_table = per_seed_table;
summary.fault_schedule_table = fault_schedule_table;
summary.segment_performance_table = segment_performance_table;
summary.segment_improvement_table = segment_improvement_table;
summary.elapsed_seconds = toc(experiment_timer);
summary.output_directory = output_directory;

local_write_tables(summary, output_directory);
summary.figure_files = plot_li_style_matlab_figures(summary, output_directory);
save(fullfile(output_directory, 'li_style_source_data.mat'), ...
    'summary', '-v7.3');
local_print_summary(summary);
end

function mask = local_fault_mask(time, segments)
mask = false(size(time));
for segment_index = 1:numel(segments)
    mask = mask | (time >= segments(segment_index).start & ...
        time <= segments(segment_index).end);
end
end

function representative = local_compact_representative( ...
        report, method_names, method_labels)
fault_result = report.scenario_results.fault(1);
reference = fault_result.(method_names{1});
sample_count = numel(reference.time);
low_num = size(reference.error_xyz, 3);
method_count = numel(method_names);
error_3d = zeros(sample_count, method_count, low_num, 'single');
for method_index = 1:method_count
    method_result = fault_result.(method_names{method_index});
    values = squeeze(sqrt(sum(method_result.error_xyz.^2, 2)));
    error_3d(:, method_index, :) = ...
        reshape(single(values), [sample_count, 1, low_num]);
end
representative.seed = reference.seed;
representative.time = reference.time;
representative.truth_xyz = reference.truth_xyz;
representative.leader_truth_xyz = reference.leader_truth_xyz;
representative.range_time = reference.range_time;
representative.range_error = reference.range_error;
representative.fault_segments = reference.fault_segments;
representative.error_3d = error_3d;
representative.method_names = method_names;
representative.method_labels = method_labels;
end

function performance_table = local_performance_table(pooled_error_3d, ...
        per_seed_rmse_3d, per_seed_fault_rmse_3d, method_labels)
[~, method_count, low_num] = size(pooled_error_3d);
row_count = method_count * (low_num + 1);
Method = strings(row_count, 1);
Follower = strings(row_count, 1);
RMSE_m = nan(row_count, 1);
FaultRMSE_m = nan(row_count, 1);
MeanError_m = nan(row_count, 1);
MaximumError_m = nan(row_count, 1);
CDF95_m = nan(row_count, 1);
SeedRMSEMean_m = nan(row_count, 1);
SeedRMSEStd_m = nan(row_count, 1);
row = 0;
for method_index = 1:method_count
    for follower = 1:(low_num + 1)
        row = row + 1;
        Method(row) = method_labels{method_index};
        if follower <= low_num
            Follower(row) = "Follower" + follower;
            samples = double(pooled_error_3d(:, method_index, follower));
            fault_rmse = squeeze(per_seed_fault_rmse_3d(:, method_index, follower));
            seed_rmse = squeeze(per_seed_rmse_3d(:, method_index, follower));
        else
            Follower(row) = "All";
            samples = reshape(double(pooled_error_3d(:, method_index, :)), [], 1);
            fault_rmse = sqrt(mean( ...
                squeeze(per_seed_fault_rmse_3d(:, method_index, :)).^2, 2));
            seed_rmse = sqrt(mean( ...
                squeeze(per_seed_rmse_3d(:, method_index, :)).^2, 2));
        end
        RMSE_m(row) = sqrt(mean(samples.^2));
        FaultRMSE_m(row) = sqrt(mean(fault_rmse.^2));
        MeanError_m(row) = mean(samples);
        MaximumError_m(row) = max(samples);
        CDF95_m(row) = prctile(samples, 95);
        SeedRMSEMean_m(row) = mean(seed_rmse);
        SeedRMSEStd_m(row) = std(seed_rmse);
    end
end
performance_table = table(Method, Follower, RMSE_m, FaultRMSE_m, ...
    MeanError_m, MaximumError_m, CDF95_m, SeedRMSEMean_m, ...
    SeedRMSEStd_m);
end

function improvement_table = local_improvement_table(per_seed_rmse_3d, ...
        per_seed_fault_rmse_3d, method_labels)
pairs = [1, 3; 2, 4];
low_num = size(per_seed_rmse_3d, 3);
row_count = size(pairs, 1) * (low_num + 1);
Comparison = strings(row_count, 1);
Follower = strings(row_count, 1);
BaselineRMSE_m = nan(row_count, 1);
CUSUMRMSE_m = nan(row_count, 1);
Improvement_pct = nan(row_count, 1);
WinRate_pct = nan(row_count, 1);
FaultBaselineRMSE_m = nan(row_count, 1);
FaultCUSUMRMSE_m = nan(row_count, 1);
FaultImprovement_pct = nan(row_count, 1);
FaultWinRate_pct = nan(row_count, 1);
row = 0;
for pair_index = 1:size(pairs, 1)
    baseline_index = pairs(pair_index, 1);
    cusum_index = pairs(pair_index, 2);
    for follower = 1:(low_num + 1)
        row = row + 1;
        Comparison(row) = string(method_labels{baseline_index}) + " -> " + ...
            string(method_labels{cusum_index});
        if follower <= low_num
            Follower(row) = "Follower" + follower;
            baseline = squeeze(per_seed_rmse_3d(:, baseline_index, follower));
            cusum = squeeze(per_seed_rmse_3d(:, cusum_index, follower));
            fault_baseline = squeeze( ...
                per_seed_fault_rmse_3d(:, baseline_index, follower));
            fault_cusum = squeeze( ...
                per_seed_fault_rmse_3d(:, cusum_index, follower));
        else
            Follower(row) = "All";
            baseline = sqrt(mean( ...
                squeeze(per_seed_rmse_3d(:, baseline_index, :)).^2, 2));
            cusum = sqrt(mean( ...
                squeeze(per_seed_rmse_3d(:, cusum_index, :)).^2, 2));
            fault_baseline = sqrt(mean( ...
                squeeze(per_seed_fault_rmse_3d(:, baseline_index, :)).^2, 2));
            fault_cusum = sqrt(mean( ...
                squeeze(per_seed_fault_rmse_3d(:, cusum_index, :)).^2, 2));
        end
        BaselineRMSE_m(row) = mean(baseline);
        CUSUMRMSE_m(row) = mean(cusum);
        Improvement_pct(row) = 100 * ...
            (BaselineRMSE_m(row) - CUSUMRMSE_m(row)) / BaselineRMSE_m(row);
        WinRate_pct(row) = 100 * mean(cusum < baseline);
        FaultBaselineRMSE_m(row) = mean(fault_baseline);
        FaultCUSUMRMSE_m(row) = mean(fault_cusum);
        FaultImprovement_pct(row) = 100 * ...
            (FaultBaselineRMSE_m(row) - FaultCUSUMRMSE_m(row)) / ...
            FaultBaselineRMSE_m(row);
        FaultWinRate_pct(row) = 100 * mean(fault_cusum < fault_baseline);
    end
end
improvement_table = table(Comparison, Follower, BaselineRMSE_m, ...
    CUSUMRMSE_m, Improvement_pct, WinRate_pct, FaultBaselineRMSE_m, ...
    FaultCUSUMRMSE_m, FaultImprovement_pct, FaultWinRate_pct);
end

function per_seed_table = local_per_seed_table(seed_list, per_seed_rmse_3d, ...
        per_seed_fault_rmse_3d, per_seed_mean_error_3d, ...
        per_seed_max_error_3d, per_seed_cdf95_3d, method_labels)
seed_count = numel(seed_list);
method_count = numel(method_labels);
low_num = size(per_seed_rmse_3d, 3);
row_count = seed_count * method_count * low_num;
Seed = zeros(row_count, 1);
Method = strings(row_count, 1);
Follower = zeros(row_count, 1);
RMSE_m = nan(row_count, 1);
FaultRMSE_m = nan(row_count, 1);
MeanError_m = nan(row_count, 1);
MaximumError_m = nan(row_count, 1);
CDF95_m = nan(row_count, 1);
row = 0;
for seed_index = 1:seed_count
    for method_index = 1:method_count
        for follower = 1:low_num
            row = row + 1;
            Seed(row) = seed_list(seed_index);
            Method(row) = method_labels{method_index};
            Follower(row) = follower;
            RMSE_m(row) = per_seed_rmse_3d(seed_index, method_index, follower);
            FaultRMSE_m(row) = ...
                per_seed_fault_rmse_3d(seed_index, method_index, follower);
            MeanError_m(row) = ...
                per_seed_mean_error_3d(seed_index, method_index, follower);
            MaximumError_m(row) = ...
                per_seed_max_error_3d(seed_index, method_index, follower);
            CDF95_m(row) = ...
                per_seed_cdf95_3d(seed_index, method_index, follower);
        end
    end
end
per_seed_table = table(Seed, Method, Follower, RMSE_m, FaultRMSE_m, ...
    MeanError_m, MaximumError_m, CDF95_m);
end

function parameter_table = local_parameter_table(cfg, seed_list)
fault_segments = cfg.fault_segments;
fault_magnitudes = unique(fault_segments(:, 3))';
fault_durations = fault_segments(:, 2) - fault_segments(:, 1);
fault_magnitude_text = strjoin(arrayfun(@(value) sprintf('%g', value), ...
    fault_magnitudes, 'UniformOutput', false), ' / ');
fault_duration_text = sprintf('%g--%g', min(fault_durations), max(fault_durations));
fault_time_text = sprintf('%g--%g', min(fault_segments(:, 1)), ...
    max(fault_segments(:, 2)));
Parameter = [ ...
    "Simulation duration"; "SINS integration interval"; ...
    "Graph/range update interval"; "Follower count"; "Leader count"; ...
    "Communication range"; "Range noise standard deviation"; ...
    "Follower position prior standard deviation (E/N/U)"; ...
    "Leader position prior standard deviation (E/N/U)"; ...
    "CUSUM-FGO sliding-window length"; "IMU preintegration"; ...
    "Monte Carlo trial count"; "Range-fault magnitudes"; ...
    "Range-fault duration"; "Range-fault time span"; ...
    "Faulty range-edge arrangement"];
Value = [ ...
    string(cfg.t_stop); string(cfg.dt); string(cfg.graph_interval); ...
    string(cfg.uav_num - cfg.high_num); string(cfg.high_num); ...
    string(cfg.communication_range); string(cfg.sigma_dis); ...
    "10 / 10 / 20"; "0.2 / 0.2 / 0.5"; ...
    string(cfg.sliding_window_length); ...
    string(cfg.imu_preintegration_enable); string(numel(seed_list)); ...
    string(fault_magnitude_text); string(fault_duration_text); ...
    string(fault_time_text); ...
    sprintf('%d events on follower-leader edges; overlapping events permitted', ...
    size(fault_segments, 1))];
Unit = [ ...
    "s"; "s"; "s"; "-"; "-"; "m"; "m"; "m"; "m"; ...
    "keyframes"; "-"; "trials"; "m"; "s"; "s"; "-"];
parameter_table = table(Parameter, Value, Unit);
end

function [probability, cdf_error_m] = local_cdf_curves( ...
        pooled_error_3d, method_count, low_num)
probability = linspace(0, 1, 1001)';
cdf_error_m = nan(numel(probability), method_count, low_num);
for method_index = 1:method_count
    for follower = 1:low_num
        samples = double(pooled_error_3d(:, method_index, follower));
        cdf_error_m(:, method_index, follower) = ...
            prctile(samples, 100 * probability);
    end
end
end

function [timewise_mean_error_3d, timewise_std_error_3d, ...
        error_statistics_table] = local_mc_figure_statistics( ...
        pooled_error_3d, sample_count, seed_count, method_labels)
% Compute the two Monte Carlo statistics used in the manuscript figures.
% The timewise quantities retain the follower dimension.  The scalar
% quantities pool all 50 trials, all simulation instants, and all followers
% for each method; this population is reported explicitly in the source data.
[~, method_count, low_num] = size(pooled_error_3d);
error_mc = reshape(pooled_error_3d, ...
    [sample_count, seed_count, method_count, low_num]);
timewise_mean_error_3d = reshape(mean(error_mc, 2), ...
    [sample_count, method_count, low_num]);
timewise_std_error_3d = reshape(std(error_mc, 0, 2), ...
    [sample_count, method_count, low_num]);

Method = string(method_labels(:));
Mean3DError_m = nan(method_count, 1);
Variance3DError_m2 = nan(method_count, 1);
Std3DError_m = nan(method_count, 1);
SampleCount = zeros(method_count, 1);
TrialCount = repmat(seed_count, method_count, 1);
FollowerCount = repmat(low_num, method_count, 1);
for method_index = 1:method_count
    samples = double(reshape( ...
        pooled_error_3d(:, method_index, :), [], 1));
    Mean3DError_m(method_index) = mean(samples);
    Variance3DError_m2(method_index) = var(samples, 0);
    Std3DError_m(method_index) = std(samples, 0);
    SampleCount(method_index) = numel(samples);
end
error_statistics_table = table(Method, Mean3DError_m, ...
    Variance3DError_m2, Std3DError_m, SampleCount, TrialCount, ...
    FollowerCount);
end

function local_write_tables(summary, output_directory)
writetable(summary.parameter_table, ...
    fullfile(output_directory, 'Table1_simulation_parameters.csv'));
writetable(summary.performance_table, ...
    fullfile(output_directory, 'Table2_positioning_performance.csv'));
writetable(summary.improvement_table, ...
    fullfile(output_directory, 'Table3_method_improvement.csv'));
writetable(summary.per_seed_table, ...
    fullfile(output_directory, 'SourceData_per_seed_metrics.csv'));
writetable(summary.fault_schedule_table, ...
    fullfile(output_directory, 'SourceData_fault_schedule.csv'));
writetable(summary.segment_performance_table, ...
    fullfile(output_directory, 'Table4_fault_segment_performance.csv'));
writetable(summary.segment_improvement_table, ...
    fullfile(output_directory, 'Table5_fault_segment_improvement.csv'));
writetable(summary.mc_error_statistics_table, ...
    fullfile(output_directory, 'SourceData_Fig8_MC50_error_statistics.csv'));
timewise_mean_table = local_timewise_mean_table(summary);
writetable(timewise_mean_table, ...
    fullfile(output_directory, 'SourceData_Fig6_MC50_timewise_mean.csv'));
figure_data_manifest = local_figure_data_manifest();
writetable(figure_data_manifest, ...
    fullfile(output_directory, 'MC50_figure_data_manifest.csv'));

workbook = fullfile(output_directory, 'Li_style_experiment_tables.xlsx');
writetable(summary.parameter_table, workbook, 'Sheet', 'Parameters', ...
    'WriteMode', 'overwritesheet');
writetable(summary.performance_table, workbook, 'Sheet', 'Performance', ...
    'WriteMode', 'overwritesheet');
writetable(summary.improvement_table, workbook, 'Sheet', 'Improvement', ...
    'WriteMode', 'overwritesheet');
writetable(summary.per_seed_table, workbook, 'Sheet', 'Per-seed metrics', ...
    'WriteMode', 'overwritesheet');
writetable(summary.fault_schedule_table, workbook, 'Sheet', 'Fault schedule', ...
    'WriteMode', 'overwritesheet');
writetable(summary.segment_performance_table, workbook, ...
    'Sheet', 'Segment performance', 'WriteMode', 'overwritesheet');
writetable(summary.segment_improvement_table, workbook, ...
    'Sheet', 'Segment improvement', 'WriteMode', 'overwritesheet');
writetable(summary.mc_error_statistics_table, workbook, ...
    'Sheet', 'MC figure statistics', 'WriteMode', 'overwritesheet');
end

function timewise_mean_table = local_timewise_mean_table(summary)
time = summary.representative.time(:);
sample_count = numel(time);
method_count = numel(summary.method_labels);
low_num = size(summary.mc_timewise_mean_error_3d, 3);
row_count = sample_count * method_count * low_num;
Time_s = zeros(row_count, 1);
Method = strings(row_count, 1);
Follower = zeros(row_count, 1);
Mean3DError_m = nan(row_count, 1);
Std3DError_m = nan(row_count, 1);
row = 0;
for follower = 1:low_num
    for method_index = 1:method_count
        rows = row + (1:sample_count);
        Time_s(rows) = time;
        Method(rows) = string(summary.method_labels{method_index});
        Follower(rows) = follower;
        Mean3DError_m(rows) = squeeze(double( ...
            summary.mc_timewise_mean_error_3d(:, method_index, follower)));
        Std3DError_m(rows) = squeeze(double( ...
            summary.mc_timewise_std_error_3d(:, method_index, follower)));
        row = row + sample_count;
    end
end
timewise_mean_table = table(Time_s, Method, Follower, Mean3DError_m, ...
    Std3DError_m);
end

function figure_data_manifest = local_figure_data_manifest()
Figure = ["Fig6_MC50_mean_positioning_error"; ...
          "Fig6_MC50_mean_positioning_error"; ...
          "Fig8_MC50_mean_variance_comparison"; ...
          "Fig8_MC50_mean_variance_comparison"];
Statistic = ["Timewise mean 3D positioning error"; ...
             "Timewise standard deviation across trials"; ...
             "Pooled mean 3D positioning error"; ...
             "Pooled variance of 3D positioning error"];
Unit = ["m"; "m"; "m"; "m^2"];
Population = ["50 trials at each time and follower"; ...
              "50 trials at each time and follower"; ...
              "50 trials, all time instants, three followers"; ...
              "50 trials, all time instants, three followers"];
SourceDataFile = ["SourceData_Fig6_MC50_timewise_mean.csv"; ...
                  "SourceData_Fig6_MC50_timewise_mean.csv"; ...
                  "SourceData_Fig8_MC50_error_statistics.csv"; ...
                  "SourceData_Fig8_MC50_error_statistics.csv"];
figure_data_manifest = table(Figure, Statistic, Unit, Population, ...
    SourceDataFile);
end

function local_print_summary(summary)
fprintf('\nLi-style Monte Carlo experiment completed in %.1f s\n', ...
    summary.elapsed_seconds);
disp(summary.improvement_table);
fprintf('Figures and tables: %s\n', summary.output_directory);
end
