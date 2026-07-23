function test_imu_preintegration_bias_recovery()
%TEST_IMU_PREINTEGRATION_BIAS_RECOVERY Verify residual-bias correction direction.
cfg = stage1_cusum_default_config('quick');
noise = imu_preintegration_default_noise(cfg);
sample_count = round(cfg.graph_interval / cfg.dt);
gravity = [0; 0; -9.7803698];
gyro_bias = [8e-4; -5e-4; 3e-4];
acc_bias = [0.04; -0.03; 0.02];
gyro = repmat(gyro_bias, 1, sample_count);
specific_force = repmat(-gravity + acc_bias, 1, sample_count);
preint = imu_preintegrate_interval(gyro, specific_force, cfg.dt, zeros(3, 1), zeros(3, 1), noise);
previous = struct('p', zeros(3, 1), 'v', zeros(3, 1), 'R', eye(3), ...
    'bg', zeros(3, 1), 'ba', zeros(3, 1));
current = previous;
uncorrected = imu_preintegration_residual(previous, current, preint, gravity);
previous.bg = gyro_bias;
previous.ba = acc_bias;
current.bg = gyro_bias;
current.ba = acc_bias;
corrected = imu_preintegration_residual(previous, current, preint, gravity);
assert(norm(corrected(1:9)) < 0.05 * norm(uncorrected(1:9)), ...
    'Using the correct residual bias must substantially reduce the motion residual.');
assert(norm(corrected(10:15), inf) < 1e-12);
fprintf('test_imu_preintegration_bias_recovery: PASS\n');
end
