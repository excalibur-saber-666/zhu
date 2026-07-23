function test_imu_preintegration_factor_zero_residual()
%TEST_IMU_PREINTEGRATION_FACTOR_ZERO_RESIDUAL Construct exactly consistent states.
cfg = stage1_cusum_default_config('quick');
noise = imu_preintegration_default_noise(cfg);
sample_count = round(cfg.graph_interval / cfg.dt);
gyro = repmat([0.03; -0.02; 0.04], 1, sample_count);
specific_force = repmat([0.6; -0.3; 0.2], 1, sample_count);
preint = imu_preintegrate_interval(gyro, specific_force, cfg.dt, zeros(3, 1), zeros(3, 1), noise);
gravity = [0; 0; -9.7803698];
previous = struct('p', [4; -3; 2], 'v', [0.2; -0.1; 0.05], ...
    'R', so3_exp([0.2; -0.1; 0.15]), 'bg', zeros(3, 1), 'ba', zeros(3, 1));
current = previous;
current.p = previous.p + previous.v * preint.delta_t + 0.5 * gravity * preint.delta_t^2 + previous.R * preint.delta_p;
current.v = previous.v + gravity * preint.delta_t + previous.R * preint.delta_v;
current.R = previous.R * preint.delta_R;
residual = imu_preintegration_residual(previous, current, preint, gravity);
assert(norm(residual, inf) < 1e-10, 'Consistent states must yield a near-zero IMU factor residual.');
fprintf('test_imu_preintegration_factor_zero_residual: PASS\n');
end
