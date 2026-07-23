function test_imu_preintegration_jacobian()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
preint = imu_preintegrate_interval(repmat([0.02; 0.01; -0.03], 1, 50), ...
    repmat([0.4; -0.2; 0.1], 1, 50), cfg.dt, zeros(3, 1), zeros(3, 1), noise);
previous = struct('p', [1; 2; 3], 'v', [0.1; -0.2; 0.3], 'R', so3_exp([0.1; -0.05; 0.02]), ...
    'bg', [1e-5; -2e-5; 1e-5], 'ba', [1e-3; 0; -1e-3]);
current = struct('p', [1.3; 1.7; 3.1], 'v', [0.4; -0.1; 0.2], 'R', so3_exp([0.12; -0.04; -0.01]), ...
    'bg', [2e-5; -1e-5; 0], 'ba', [0; 1e-3; 0]);
epsilons = struct('position', 1e-5, 'velocity', 1e-5, 'rotation', 1e-7, 'gyro_bias', 1e-8, 'acc_bias', 1e-6);
[~, analytic_numeric, ~] = imu_preintegration_numeric_jacobian(previous, current, preint, [0; 0; -9.7803698], epsilons);
step = 1e-5; plus = previous; minus = previous; plus.p(1) = plus.p(1) + step; minus.p(1) = minus.p(1) - step;
reference = (imu_preintegration_residual(plus, current, preint, [0; 0; -9.7803698]) - ...
    imu_preintegration_residual(minus, current, preint, [0; 0; -9.7803698])) / (2 * step);
assert(norm(reference - analytic_numeric(:, 1)) < 1e-7);
fprintf('test_imu_preintegration_jacobian: PASS\n');
end
