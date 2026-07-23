function test_imu_preintegration_constant_rotation()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
angular_rate = [0.03; -0.02; 0.1]; sample_count = 50; duration = sample_count * cfg.dt;
preint = imu_preintegrate_interval(repmat(angular_rate, 1, sample_count), zeros(3, sample_count), ...
    cfg.dt, zeros(3, 1), zeros(3, 1), noise);
assert(norm(so3_log(preint.delta_R) - angular_rate * duration) < 1e-10);
fprintf('test_imu_preintegration_constant_rotation: PASS\n');
end
