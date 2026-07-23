function test_imu_preintegration_partition_consistency()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
sample_count = 50; gyro = repmat([0.02; -0.01; 0.04], 1, sample_count);
acc = repmat([0.5; 0.2; -0.1], 1, sample_count);
full = imu_preintegrate_interval(gyro, acc, cfg.dt, zeros(3, 1), zeros(3, 1), noise);
first = imu_preintegrate_interval(gyro(:, 1:25), acc(:, 1:25), cfg.dt, zeros(3, 1), zeros(3, 1), noise);
second = imu_preintegrate_interval(gyro(:, 26:50), acc(:, 26:50), cfg.dt, zeros(3, 1), zeros(3, 1), noise);
composed_rotation = first.delta_R * second.delta_R;
composed_velocity = first.delta_v + first.delta_R * second.delta_v;
composed_position = first.delta_p + first.delta_v * second.delta_t + first.delta_R * second.delta_p;
assert(norm(so3_log(full.delta_R' * composed_rotation)) < 1e-10);
assert(norm(full.delta_v - composed_velocity) < 1e-10);
assert(norm(full.delta_p - composed_position) < 1e-10);
fprintf('test_imu_preintegration_partition_consistency: PASS\n');
end
