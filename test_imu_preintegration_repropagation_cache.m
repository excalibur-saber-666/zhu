function test_imu_preintegration_repropagation_cache()
%TEST_IMU_PREINTEGRATION_REPROPAGATION_CACHE Verify cached bias repropagation.
cfg = stage1_cusum_default_config('quick');
cfg.imu_preint_covariance_mode = 'current_frame_only';
graph = factor_graph_sliding_window_imu_preint(2, 2, 1, cfg);
gravity = [0; 0; -9.7803698];
gyro_bias = [1e-3; -5e-4; 2e-4];
leader_position = [10; 0; 0];
state_1 = local_state([0; 0; 0], [0; 0; 0], eye(3), gyro_bias, zeros(3, 1));
state_2 = local_state(0.5 * gravity, gravity, eye(3), gyro_bias, zeros(3, 1));
graph.set_frame_initial(1, [leader_position, state_1.p], state_1.v, state_1.R, state_1.bg, state_1.ba);
graph.set_frame_initial(2, [leader_position, state_2.p], state_2.v, state_2.R, state_2.bg, state_2.ba);
prior_std = struct('position', 0.1 * ones(3, 1), 'velocity', 0.1 * ones(3, 1), ...
    'rotation', 0.01 * ones(3, 1), 'gyro_bias', 1e-4 * ones(3, 1), ...
    'acc_bias', 1e-3 * ones(3, 1));
for frame = 1:2
    graph.add_leader_prior(frame, 1, leader_position, 0.1 * ones(3, 1));
end
graph.add_follower_prior(1, 1, state_1, prior_std);
graph.add_follower_prior(2, 1, state_2, prior_std);
noise = imu_preintegration_default_noise(cfg);
preint = imu_preintegrate_interval(repmat(gyro_bias, 1, 50), zeros(3, 50), cfg.dt, ...
    zeros(3, 1), zeros(3, 1), noise);
graph.add_imu_factor(1, 2, 1, preint);
graph.Gauss_Newton();
assert(graph.repropagation_count >= 1, 'A changed reference bias must trigger repropagation.');
assert(norm(graph.imu_factors(1).preint.bias_gyro_ref - graph.get_follower_state(1, 1).bg) <= ...
    cfg.imu_preint_gyro_bias_repropagate_threshold, 'Repropagated result was not written back to the factor cache.');
graph.Gauss_Newton();
assert(graph.repropagation_count == 0, 'An unchanged reference bias must not repropagate the same cached factor again.');
fprintf('test_imu_preintegration_repropagation_cache: PASS\n');
end

function state = local_state(p, v, R, bg, ba)
state = struct('p', p, 'v', v, 'R', R, 'bg', bg, 'ba', ba);
end
