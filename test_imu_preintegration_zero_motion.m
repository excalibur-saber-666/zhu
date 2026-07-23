function test_imu_preintegration_zero_motion()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
preint = imu_preintegrate_interval(zeros(3, 50), zeros(3, 50), cfg.dt, zeros(3, 1), zeros(3, 1), noise);
assert(norm(preint.delta_R - eye(3), 'fro') < 1e-12);
assert(norm(preint.delta_v) < 1e-12 && norm(preint.delta_p) < 1e-12);
assert(abs(preint.delta_t - 1) < 1e-12 && preint.sample_count == 50);
assert(all(isfinite(preint.covariance(:))) && min(eig(preint.covariance)) > 0);
fprintf('test_imu_preintegration_zero_motion: PASS\n');
end
