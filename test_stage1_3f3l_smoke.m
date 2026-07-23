function test_stage1_3f3l_smoke()
%TEST_STAGE1_3F3L_SMOKE Smoke test for the three-follower/three-leader case.

cfg = stage1_cusum_3f3l_config('quick');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_bias = 5;
cfg.verbose = false;
cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = false;

local_assert_initial_geometry(cfg);
report = run_stage1_cusum_comparison(cfg);
fault_result = report.scenario_results.fault(1).cusum;

assert(numel(fault_result.metrics.full_rmse_3d) == 3 && ...
    all(isfinite(fault_result.metrics.full_rmse_3d)), ...
    'The 3-follower/3-leader run did not return finite RMSEs for all followers.');
assert(all(isfinite(fault_result.metrics.window_rmse_3d)), ...
    'The 3-follower/3-leader fault-window RMSE contains a non-finite value.');
assert(fault_result.diagnostics.target_weight_median_fault_window < 1, ...
    'The configured Follower2--Leader1 fault did not reduce its CUSUM weight.');
assert(any(arrayfun(@(entry) any(entry.pairs(:) == 3), fault_result.history)), ...
    'Follower3 has no active range edge in the simulated graph.');
fprintf('test_stage1_3f3l_smoke: PASS\n');
end

function local_assert_initial_geometry(cfg)
position_east = load('posi_e_all.dat');
position_north = load('posi_n_all.dat');
position_up = load('posi_u_all.dat');
positions = [position_east; position_north; position_up];
followers = positions(:, 1:3);
leaders = positions(:, cfg.base_leader_source_indices);
for follower = 1:3
    line_of_sight = leaders - followers(:, follower);
    ranges = vecnorm(line_of_sight, 2, 1);
    assert(all(ranges <= cfg.communication_range), ...
        'A selected leader is outside the communication range of a follower.');
    assert(rank(line_of_sight ./ ranges) == 3, ...
        'The selected three-leader geometry is not full rank for a follower.');
end
end
