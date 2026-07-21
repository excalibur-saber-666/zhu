function test_stage1_redundant_sliding_window_cusum_smoke()
%TEST_STAGE1_REDUNDANT_SLIDING_WINDOW_CUSUM_SMOKE Six-UAV admission test.
%   Confirms that the independent four-leader experiment runs and that, at
%   each key frame, no follower loses more than one alarmed leader range.

cfg = stage1_cusum_redundant_config('quick');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_bias = 5;
cfg.verbose = false;
cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = false;
report = run_stage1_cusum_comparison(cfg);

fault_result = report.scenario_results.fault(1).cusum;
assert(all(isfinite(fault_result.metrics.full_rmse_3d)), ...
    'The redundant sliding-window result contains a non-finite full RMSE.');
assert(all(isfinite(fault_result.metrics.window_rmse_3d)), ...
    'The redundant sliding-window result contains a non-finite fault-window RMSE.');
assert(any(arrayfun(@(entry) ~isempty(entry.excluded_range_pairs), fault_result.history)), ...
    'The quick fault did not activate selective alarm-edge admission.');

low_num = cfg.uav_num - cfg.high_num;
for epoch_index = 1:numel(fault_result.history)
    entry = fault_result.history(epoch_index);
    pairs = entry.pairs;
    excluded_pairs = entry.excluded_range_pairs;
    for follower = 1:low_num
        leader_mask = (pairs(:, 1) == follower | pairs(:, 2) == follower) & ...
            max(pairs, [], 2) > low_num;
        excluded_leader_mask = (excluded_pairs(:, 1) == follower | ...
            excluded_pairs(:, 2) == follower) & max(excluded_pairs, [], 2) > low_num;
        assert(sum(excluded_leader_mask) <= 1, ...
            'More than one leader edge was excluded for one follower at one key frame.');
        if ~any(excluded_leader_mask)
            continue;
        end

        candidate_indices = find(leader_mask & entry.detail.alarm_active);
        candidate_scores = entry.detail.cusum_value(candidate_indices);
        [~, strongest_local_index] = max(candidate_scores);
        strongest_pair = pairs(candidate_indices(strongest_local_index), :);
        excluded_pair = excluded_pairs(find(excluded_leader_mask, 1, 'first'), :);
        assert(isequal(strongest_pair, excluded_pair), ...
            'The excluded leader edge is not the maximum-CUSUM alarm for its follower.');
    end
end
fprintf('test_stage1_redundant_sliding_window_cusum_smoke: PASS\n');
end
