function test_stage1_sliding_without_motion_regression()
%TEST_STAGE1_SLIDING_WITHOUT_MOTION_REGRESSION Without inter-frame factors,
% a multi-frame graph is block diagonal and must equal single-epoch FGO.

cfg = stage1_cusum_redundant_config('quick');
cfg.seeds = 23;
cfg.fault_enable = false;
cfg.verbose = false;
cfg.plot_component_comparison = false;
cfg.comparison_mode = 'original_vs_sliding_equal';
cfg.sliding_window_length = 10;
cfg.sliding_window_motion_enable = false;
cfg.sliding_window_exclude_alarmed_edges = false;
cfg.sliding_window_cusum_consensus_enable = false;
report = run_stage1_cusum_comparison(cfg);

original = report.scenario_results.healthy(1).equal;
sliding_equal = report.scenario_results.healthy(1).cusum;
% The two QR paths can stop one Gauss-Newton iteration apart at the shared
% 1e-5 step threshold, leaving only round-off-scale position differences.
tolerance = 1e-7;
assert(max(abs(original.navigation_xyz(:) - sliding_equal.navigation_xyz(:))) <= tolerance && ...
    max(abs(original.metrics.full_rmse_3d - sliding_equal.metrics.full_rmse_3d)) <= tolerance, ...
    'A sliding graph without motion factors does not reduce to independent single epochs.');
fprintf('test_stage1_sliding_without_motion_regression: PASS\n');
end
