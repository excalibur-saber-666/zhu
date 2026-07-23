function test_imu_preintegration_jacobian()
cfg = stage1_cusum_default_config('quick'); noise = imu_preintegration_default_noise(cfg);
preint = imu_preintegrate_interval(repmat([0.02; 0.01; -0.03], 1, 50), ...
    repmat([0.4; -0.2; 0.1], 1, 50), cfg.dt, zeros(3, 1), zeros(3, 1), noise);
previous = struct('p', [1; 2; 3], 'v', [0.1; -0.2; 0.3], 'R', so3_exp([0.1; -0.05; 0.02]), ...
    'bg', [1e-5; -2e-5; 1e-5], 'ba', [1e-3; 0; -1e-3]);
current = struct('p', [1.3; 1.7; 3.1], 'v', [0.4; -0.1; 0.2], 'R', so3_exp([0.12; -0.04; -0.01]), ...
    'bg', [2e-5; -1e-5; 0], 'ba', [0; 1e-3; 0]);
epsilons = struct('position', 1e-5, 'velocity', 1e-5, 'rotation', 1e-7, 'gyro_bias', 1e-8, 'acc_bias', 1e-6);
[~, jacobian_previous, jacobian_current] = imu_preintegration_numeric_jacobian( ...
    previous, current, preint, [0; 0; -9.7803698], epsilons);
steps = [repmat(4e-5, 6, 1); repmat(4e-6, 3, 1); repmat(4e-8, 3, 1); repmat(4e-5, 3, 1)];
for column = 1:15
    reference = local_five_point(previous, current, preint, [0; 0; -9.7803698], column, steps(column), true);
    assert(norm(reference - jacobian_previous(:, column), inf) < 2e-5, ...
        'Previous-state Jacobian column %d disagrees with the independent stencil.', column);
    reference = local_five_point(previous, current, preint, [0; 0; -9.7803698], column, steps(column), false);
    assert(norm(reference - jacobian_current(:, column), inf) < 2e-5, ...
        'Current-state Jacobian column %d disagrees with the independent stencil.', column);
end
fprintf('test_imu_preintegration_jacobian: PASS\n');
end

function derivative = local_five_point(previous, current, preint, gravity, column, step_size, perturb_previous)
samples = [-2, -1, 1, 2];
residuals = zeros(15, numel(samples));
for index = 1:numel(samples)
    increment = zeros(15, 1);
    increment(column) = samples(index) * step_size;
    if perturb_previous
        residuals(:, index) = imu_preintegration_residual(local_apply_increment(previous, increment), current, preint, gravity);
    else
        residuals(:, index) = imu_preintegration_residual(previous, local_apply_increment(current, increment), preint, gravity);
    end
end
derivative = (residuals(:, 1) - 8 * residuals(:, 2) + 8 * residuals(:, 3) - residuals(:, 4)) / (12 * step_size);
end

function state = local_apply_increment(state, increment)
state.p = state.p + increment(1:3);
state.v = state.v + increment(4:6);
state.R = state.R * so3_exp(increment(7:9));
state.bg = state.bg + increment(10:12);
state.ba = state.ba + increment(13:15);
end
