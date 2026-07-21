function test_stage1_sliding_window_cusum_smoke()
%TEST_STAGE1_SLIDING_WINDOW_CUSUM_SMOKE Short no-plot regression check.

cfg = stage1_cusum_default_config('quick');
cfg.seeds = 23;
cfg.fault_enable = false;
cfg.verbose = false;
original_visibility = get(0, 'DefaultFigureVisible');
cleanup = onCleanup(@() set(0, 'DefaultFigureVisible', original_visibility)); %#ok<NASGU>
set(0, 'DefaultFigureVisible', 'off');
report = run_stage1_sliding_window_cusum_comparison(cfg);

assert(strcmp(report.comparison_mode, 'original_vs_sliding_cusum'));
assert(strcmp(report.method_labels{1}, 'original'));
assert(strcmp(report.method_labels{2}, 'sw-cusum'));
assert(numel(report.seed_results) == 1);
assert(all(isfinite(report.seed_results(1).equal.metrics.full_rmse_3d)));
assert(all(isfinite(report.seed_results(1).cusum.metrics.full_rmse_3d)));
assert(numel(findall(0, 'Type', 'figure')) == 2, ...
    'The sliding-window CUSUM entry did not create one error figure per follower.');
close all;
fprintf('test_stage1_sliding_window_cusum_smoke: PASS\n');
end
