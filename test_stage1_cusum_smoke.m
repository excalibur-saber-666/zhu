function test_stage1_cusum_smoke()
%TEST_STAGE1_CUSUM_SMOKE Short cached-input integration test.

cfg = stage1_cusum_default_config('quick');
cfg.seeds = 17;
cfg.verbose = false;
cfg.fault_enable = false;
healthy = run_stage1_cusum_comparison(cfg);
assert(local_result_is_finite(healthy.seed_results(1).equal));
assert(local_result_is_finite(healthy.seed_results(1).cusum));
assert(all(healthy.seed_results(1).equal.history(1).edge_count > 0));
assert(cfg.cusum_warmup_time < cfg.fault_start && cfg.fault_start < cfg.fault_end && cfg.fault_end < cfg.t_stop, ...
    'Quick configuration does not leave a post-warm-up fault interval.');
healthy_history = healthy.seed_results(1).cusum.history(1);
assert(norm(healthy_history.cusum_prior_positions(:, 1) - healthy_history.graph_prior_positions(:, 1)) > 1e-9, ...
    'CUSUM follower predictor unexpectedly equals the GPS graph prior.');
cusum_covariance = healthy_history.cusum_prior_covariances(:, :, 1);
assert(norm(cusum_covariance - diag([10, 10, 20].^2), 'fro') > 1e-6 && ...
    norm(cusum_covariance - cusum_covariance', 'fro') < 1e-10 && ...
    min(eig(cusum_covariance)) >= -1e-10, ...
    'CUSUM follower covariance is not the converted KF prediction covariance.');

cfg.fault_enable = true;
fault = run_stage1_cusum_comparison(cfg);
assert(local_result_is_finite(fault.seed_results(1).cusum));
assert(all(arrayfun(@(entry) entry.edge_count, fault.seed_results(1).cusum.history) > 0), ...
    'A valid range edge was deleted during the CUSUM run.');
fault_diagnostics = fault.seed_results(1).cusum.diagnostics;
assert(fault_diagnostics.target_cusum_median > 0, 'CUSUM did not accumulate during the quick fault interval.');
assert(fault_diagnostics.target_weight_median_fault_window < 1, 'Fault-edge CUSUM weight did not decrease.');
assert(isfinite(fault_diagnostics.detect_095_delay), 'Quick fault was not detected after warm-up.');
window_statistics = local_fault_window_statistics(fault.seed_results(1).cusum.history, cfg);
assert(abs(fault_diagnostics.target_weight_median_fault_window - window_statistics.target_weight_median) < 1e-12 && ...
    abs(fault_diagnostics.other_weight_median_fault_window - window_statistics.other_weight_median) < 1e-12 && ...
    abs(fault_diagnostics.target_to_other_weight_ratio_fault_window - window_statistics.weight_ratio) < 1e-12, ...
    'Fault-edge and healthy-edge diagnostics do not use the same fault window.');
assert(isfinite(fault.seed_results(1).cusum.gn_diagnostics.final_step_norm) && ...
    fault.seed_results(1).cusum.gn_diagnostics.mean_iterations >= 1, ...
    'GN diagnostics are not valid.');

% Deterministic repeatability test: a second cached Equal run is identical.
repeat = run_stage1_cusum_comparison(cfg);
delta = max(abs(fault.seed_results(1).equal.navigation_xyz(:) - repeat.seed_results(1).equal.navigation_xyz(:)));
assert(delta == 0, 'Cached Equal-FGO baseline is not repeatable.');
fprintf('deterministic repeatability test: PASS\n');

% Original baseline regression test: direct original-Equal branch versus the
% new framework Equal branch, under the exact same cached input.
cfg.enable_original_baseline_regression = true;
regression = run_stage1_cusum_comparison(cfg);
new_equal = regression.scenario_results.fault(1).equal;
original_equal = regression.scenario_results.fault(1).original_equal;
navigation_delta = max(abs(new_equal.navigation_xyz(:) - original_equal.navigation_xyz(:)));
rmse_delta = max(abs([new_equal.metrics.full_rmse_3d; new_equal.metrics.window_rmse_3d] - ...
    [original_equal.metrics.full_rmse_3d; original_equal.metrics.window_rmse_3d]));
graph_position_delta = local_history_max_difference(new_equal.history, original_equal.history, 'graph_follower_positions');
graph_covariance_delta = local_history_max_difference(new_equal.history, original_equal.history, 'graph_follower_covariances');
assert(navigation_delta <= 1e-10 && rmse_delta <= 1e-10 && ...
    graph_position_delta <= 1e-10 && graph_covariance_delta <= 1e-10, ...
    'New Equal-FGO does not regress to the original Equal baseline.');
fprintf('original baseline regression test: PASS\n');
fprintf('test_stage1_cusum_smoke: PASS\n');
end

function is_finite = local_result_is_finite(result)
is_finite = all(isfinite(result.navigation_xyz(:))) && all(isfinite(result.error_xyz(:)));
for index = 1:numel(result.history)
    detail = result.history(index).detail;
    if isempty(fieldnames(detail))
        continue;
    end
    is_finite = is_finite && all(isfinite(detail.final_weight)) && isreal(detail.final_weight) && ...
        all(detail.final_weight > 0) && all(detail.final_weight <= 1);
end
end

function delta = local_history_max_difference(history_a, history_b, field_name)
delta = 0;
for index = 1:numel(history_a)
    delta = max(delta, max(abs(history_a(index).(field_name)(:) - history_b(index).(field_name)(:))));
end
end

function statistics = local_fault_window_statistics(history, cfg)
target = sort(cfg.fault_edge(:)');
target_weights = zeros(0, 1);
other_weights = zeros(0, 1);
for index = 1:numel(history)
    if history(index).time < cfg.fault_start || history(index).time > cfg.fault_end
        continue;
    end
    pairs = history(index).pairs;
    weights = history(index).detail.final_weight;
    target_mask = pairs(:, 1) == target(1) & pairs(:, 2) == target(2);
    target_weights = [target_weights; weights(target_mask)]; %#ok<AGROW>
    other_weights = [other_weights; weights(~target_mask)]; %#ok<AGROW>
end
statistics.target_weight_median = median(target_weights);
statistics.other_weight_median = median(other_weights);
statistics.weight_ratio = statistics.target_weight_median / statistics.other_weight_median;
end
