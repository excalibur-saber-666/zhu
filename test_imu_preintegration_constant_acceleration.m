function test_imu_preintegration_constant_acceleration()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
acceleration = [1.2; -0.4; 0.8]; sample_count = 50; duration = sample_count * cfg.dt;
preint = imu_preintegrate_interval(zeros(3, sample_count), repmat(acceleration, 1, sample_count), ...
    cfg.dt, zeros(3, 1), zeros(3, 1), noise);
assert(norm(preint.delta_v - acceleration * duration) < 1e-10);
assert(norm(preint.delta_p - 0.5 * acceleration * duration^2) < 2e-2);
fprintf('test_imu_preintegration_constant_acceleration: PASS\n');
end
