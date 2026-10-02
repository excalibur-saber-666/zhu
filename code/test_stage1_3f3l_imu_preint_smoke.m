function test_stage1_3f3l_imu_preint_smoke()
%TEST_STAGE1_3F3L_IMU_PREINT_SMOKE Quick end-to-end IMU-preintegration check.
cfg = stage1_cusum_3f3l_config('quick');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_edge = [2, 4];
cfg.fault_bias = 5;
cfg.imu_preintegration_enable = true;
cfg.imu_preint_window_length = 3;
cfg.verbose = false;
cfg.plot_position_results = false;
report = main_stage1('cusum_fgo', cfg);
result = report.scenario_results.fault(1).cusum_fgo;
assert(result.imu_preintegration_applied, ...
    'The IMU-preintegration switch did not activate CUSUM-FGO.');
assert(all(isfinite(result.metrics.full_rmse_3d)) && all(isfinite(result.metrics.window_rmse_3d)));
assert(result.diagnostics.target_weight_median_fault_window < 1, ...
    'CUSUM target-edge weighting is not active in IMU preintegration mode.');
sample_counts = vertcat(result.history.imu_sample_counts);
delta_t = vertcat(result.history.imu_delta_t);
assert(all(sample_counts(:) == round(cfg.graph_interval / cfg.dt)), ...
    'Every key-frame interval must contain 50 IMU samples in quick mode.');
assert(all(abs(delta_t(:) - cfg.graph_interval) < 1e-12), ...
    'IMU preintegration duration does not match graph_interval.');
assert(result.imu_preintegration_diagnostics.total_seconds > 0, ...
    'Preintegration time was not recorded.');
assert(all(isfinite(result.navigation_xyz(:))));
fprintf('test_stage1_3f3l_imu_preint_smoke: PASS\n');
end
