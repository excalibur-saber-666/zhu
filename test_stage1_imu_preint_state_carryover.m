function test_stage1_imu_preint_state_carryover()
%TEST_STAGE1_IMU_PREINT_STATE_CARRYOVER Verify overlap initialization and priors.
cfg = stage1_cusum_3f3l_config('quick');
cfg.seeds = 23;
cfg.fault_enable = true;
cfg.fault_edge = [2, 4];
cfg.fault_bias = 5;
cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.sliding_window_motion_model = 'imu_preint';
cfg.imu_preint_window_length = 3;
cfg.imu_preint_prior_mode = 'first_frame_full';
cfg.imu_preint_feedback_mode = 'state_carryover';
cfg.verbose = false;
report = run_stage1_cusum_comparison(cfg);
history = report.scenario_results.fault(1).cusum.history;
low_num = cfg.uav_num - cfg.high_num;
carryover_observed = false;
for index = 2:numel(history)
    current = history(index).imu_state_diagnostics;
    previous = history(index - 1).imu_state_diagnostics;
    if isempty(current.frame_times) || isempty(previous.frame_times)
        continue;
    end
    for current_frame = 1:numel(current.frame_times)
        previous_frame = find(abs(previous.frame_times - current.frame_times(current_frame)) < 1e-12, 1);
        if isempty(previous_frame)
            continue;
        end
        for follower = 1:low_num
            assert(current.initial_from_carryover(current_frame, follower), ...
                'Overlapping frame was not initialized from the prior optimized state.');
            assert(local_state_distance(current.initial_states{current_frame, follower}, ...
                previous.optimized_states{previous_frame, follower}) < 1e-10, ...
                'Carried IMU state differs from the prior window optimum.');
        end
        carryover_observed = true;
    end
    assert(history(index).imu_full_state_prior_count == low_num, ...
        'Fixed prior mode must add exactly one complete follower prior per window.');
    expected_position_priors = low_num * (numel(current.frame_times) - 1);
    assert(history(index).imu_position_prior_count == expected_position_priors, ...
        'Every non-anchor frame must receive only its position prior.');
end
assert(carryover_observed, 'No overlapping window state was available to test carryover.');
fprintf('test_stage1_imu_preint_state_carryover: PASS\n');
end

function distance = local_state_distance(left, right)
distance = norm([left.p - right.p; left.v - right.v; so3_log(left.R' * right.R); ...
    left.bg - right.bg; left.ba - right.ba], inf);
end
