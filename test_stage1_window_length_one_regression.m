function test_stage1_window_length_one_regression()
%TEST_STAGE1_WINDOW_LENGTH_ONE_REGRESSION Sliding equal graph must reduce
% to the single-epoch equal graph when it has exactly one frame.

cfg = stage1_cusum_redundant_config('quick');
cfg.seeds = 23;
cfg.fault_enable = false;
cfg.verbose = false;
cfg.plot_component_comparison = false;
cfg.comparison_mode = 'original_vs_sliding_equal';
cfg.sliding_window_length = 1;
cfg.sliding_window_exclude_alarmed_edges = false;
cfg.sliding_window_cusum_consensus_enable = false;
report = run_stage1_cusum_comparison(cfg);

original = report.scenario_results.healthy(1).equal;
sliding_equal = report.scenario_results.healthy(1).cusum;
position_difference = max(abs(original.navigation_xyz(:) - sliding_equal.navigation_xyz(:)));
error_difference = max(abs(original.error_xyz(:) - sliding_equal.error_xyz(:)));
rmse_difference = max(abs(original.metrics.full_rmse_3d - ...
    sliding_equal.metrics.full_rmse_3d));
graph_position_difference = local_history_difference( ...
    original.history, sliding_equal.history, 'graph_follower_positions');
graph_covariance_difference = local_history_difference( ...
    original.history, sliding_equal.history, 'graph_follower_covariances');

tolerance = 1e-8;
assert(position_difference <= tolerance && error_difference <= tolerance && ...
    rmse_difference <= tolerance && graph_position_difference <= tolerance && ...
    graph_covariance_difference <= tolerance, ...
    'A one-frame sliding equal graph does not reproduce single-epoch Equal-FGO.');
fprintf('test_stage1_window_length_one_regression: PASS\n');
end

function difference = local_history_difference(history_a, history_b, field_name)
difference = 0;
for index = 1:numel(history_a)
    difference = max(difference, max(abs( ...
        history_a(index).(field_name)(:) - history_b(index).(field_name)(:))));
end
end
