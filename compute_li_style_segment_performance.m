function [per_seed_target_rmse, performance_table, improvement_table] = ...
        compute_li_style_segment_performance(pooled_error_3d, ...
        fault_schedule_table, cfg, seed_list, method_labels)
%COMPUTE_LI_STYLE_SEGMENT_PERFORMANCE Statistics for scheduled fault segments.
%   Each segment is evaluated on the follower directly connected to its
%   scheduled faulty follower-leader range edge.

seed_list = seed_list(:)';
seed_count = numel(seed_list);
method_count = numel(method_labels);
segment_count = size(cfg.fault_segments, 1);
sample_count = round(cfg.t_stop / cfg.dt) + 1;
time = (0:(sample_count - 1))' * cfg.dt;
per_seed_target_rmse = nan(seed_count, method_count, segment_count);

for seed_index = 1:seed_count
    pooled_rows = (seed_index - 1) * sample_count + (1:sample_count);
    for segment_index = 1:segment_count
        schedule_row = fault_schedule_table.Seed == seed_list(seed_index) & ...
            fault_schedule_table.Segment == segment_index;
        if sum(schedule_row) ~= 1
            error('compute_li_style_segment_performance:InvalidSchedule', ...
                'Each seed and segment must have exactly one resolved fault edge.');
        end
        start_time = fault_schedule_table.Start_s(schedule_row);
        end_time = fault_schedule_table.End_s(schedule_row);
        follower = fault_schedule_table.Follower(schedule_row);
        segment_mask = time >= start_time & time <= end_time;
        for method_index = 1:method_count
            samples = double(pooled_error_3d( ...
                pooled_rows, method_index, follower));
            per_seed_target_rmse(seed_index, method_index, segment_index) = ...
                sqrt(mean(samples(segment_mask).^2));
        end
    end
end

row_count = method_count * segment_count;
Segment = zeros(row_count, 1);
Bias_m = zeros(row_count, 1);
Duration_s = zeros(row_count, 1);
Method = strings(row_count, 1);
TargetRMSEMean_m = nan(row_count, 1);
TargetRMSEStd_m = nan(row_count, 1);
TargetRMSEMedian_m = nan(row_count, 1);
TargetRMSEMaximum_m = nan(row_count, 1);
row = 0;
for segment_index = 1:segment_count
    for method_index = 1:method_count
        row = row + 1;
        values = per_seed_target_rmse(:, method_index, segment_index);
        Segment(row) = segment_index;
        Bias_m(row) = cfg.fault_segments(segment_index, 3);
        Duration_s(row) = cfg.fault_segments(segment_index, 2) - ...
            cfg.fault_segments(segment_index, 1);
        Method(row) = method_labels{method_index};
        TargetRMSEMean_m(row) = mean(values);
        TargetRMSEStd_m(row) = std(values);
        TargetRMSEMedian_m(row) = median(values);
        TargetRMSEMaximum_m(row) = max(values);
    end
end
performance_table = table(Segment, Bias_m, Duration_s, Method, ...
    TargetRMSEMean_m, TargetRMSEStd_m, TargetRMSEMedian_m, ...
    TargetRMSEMaximum_m);

pairs = [1, 3; 2, 4];
row_count = size(pairs, 1) * segment_count;
Segment = zeros(row_count, 1);
Bias_m = zeros(row_count, 1);
Duration_s = zeros(row_count, 1);
Comparison = strings(row_count, 1);
BaselineRMSE_m = nan(row_count, 1);
CUSUMRMSE_m = nan(row_count, 1);
Improvement_pct = nan(row_count, 1);
WinRate_pct = nan(row_count, 1);
row = 0;
for segment_index = 1:segment_count
    for pair_index = 1:size(pairs, 1)
        row = row + 1;
        baseline_index = pairs(pair_index, 1);
        cusum_index = pairs(pair_index, 2);
        baseline = per_seed_target_rmse(:, baseline_index, segment_index);
        cusum = per_seed_target_rmse(:, cusum_index, segment_index);
        Segment(row) = segment_index;
        Bias_m(row) = cfg.fault_segments(segment_index, 3);
        Duration_s(row) = cfg.fault_segments(segment_index, 2) - ...
            cfg.fault_segments(segment_index, 1);
        Comparison(row) = string(method_labels{baseline_index}) + ...
            " -> " + string(method_labels{cusum_index});
        BaselineRMSE_m(row) = mean(baseline);
        CUSUMRMSE_m(row) = mean(cusum);
        Improvement_pct(row) = 100 * ...
            (BaselineRMSE_m(row) - CUSUMRMSE_m(row)) / ...
            BaselineRMSE_m(row);
        WinRate_pct(row) = 100 * mean(cusum < baseline);
    end
end
improvement_table = table(Segment, Bias_m, Duration_s, Comparison, ...
    BaselineRMSE_m, CUSUMRMSE_m, Improvement_pct, WinRate_pct);
end
