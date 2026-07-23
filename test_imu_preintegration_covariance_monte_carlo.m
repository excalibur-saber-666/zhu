function test_imu_preintegration_covariance_monte_carlo()
%TEST_IMU_PREINTEGRATION_COVARIANCE_MONTE_CARLO Check covariance scale on a short interval.
rng(71, 'twister');
dt = 0.02;
sample_count = 50;
noise = struct('gyro_noise_std', 2e-3, 'acc_noise_std', 4e-2, ...
    'gyro_bias_rw_std', 0, 'acc_bias_rw_std', 0, 'covariance_regularization', 1e-14);
predicted = imu_preintegrate_interval(zeros(3, sample_count), zeros(3, sample_count), ...
    dt, zeros(3, 1), zeros(3, 1), noise);
trial_count = 160;
errors = zeros(trial_count, 9);
for trial = 1:trial_count
    gyro = noise.gyro_noise_std / sqrt(dt) * randn(3, sample_count);
    specific_force = noise.acc_noise_std / sqrt(dt) * randn(3, sample_count);
    sample = imu_preintegrate_interval(gyro, specific_force, dt, zeros(3, 1), zeros(3, 1), noise);
    errors(trial, :) = [sample.delta_p; sample.delta_v; so3_log(sample.delta_R)]';
end
sample_covariance = cov(errors, 1);
predicted_covariance = predicted.covariance(1:9, 1:9);
ratios = diag(sample_covariance) ./ diag(predicted_covariance);
assert(all(isfinite(ratios)) && all(ratios > 0.35) && all(ratios < 2.8), ...
    'Monte-Carlo and propagated position/velocity/attitude variances disagree by more than expected sampling error.');
fprintf('test_imu_preintegration_covariance_monte_carlo: PASS (variance ratio %.2f to %.2f)\n', ...
    min(ratios), max(ratios));
end
