function [residual, jacobian_previous, jacobian_current] = imu_preintegration_numeric_jacobian(previous_state, current_state, preint, gravity_enu, epsilons)
%IMU_PREINTEGRATION_NUMERIC_JACOBIAN Central-difference Jacobians on SO(3).
if nargin < 5 || isempty(epsilons)
    epsilons = struct('position', 1e-5, 'velocity', 1e-5, 'rotation', 1e-7, ...
        'gyro_bias', 1e-8, 'acc_bias', 1e-6);
end
residual = imu_preintegration_residual(previous_state, current_state, preint, gravity_enu);
steps = [repmat(epsilons.position, 3, 1); repmat(epsilons.velocity, 3, 1); ...
    repmat(epsilons.rotation, 3, 1); repmat(epsilons.gyro_bias, 3, 1); ...
    repmat(epsilons.acc_bias, 3, 1)];
if any(~isfinite(steps)) || any(steps <= 0)
    error('imu_preintegration_numeric_jacobian:InvalidEpsilons', 'All numeric-Jacobian steps must be positive.');
end
jacobian_previous = zeros(15, 15);
jacobian_current = zeros(15, 15);
for column = 1:15
    step = zeros(15, 1); step(column) = steps(column);
    plus = local_apply_increment(previous_state, step);
    minus = local_apply_increment(previous_state, -step);
    jacobian_previous(:, column) = (imu_preintegration_residual(plus, current_state, preint, gravity_enu) - ...
        imu_preintegration_residual(minus, current_state, preint, gravity_enu)) / (2 * steps(column));
    plus = local_apply_increment(current_state, step);
    minus = local_apply_increment(current_state, -step);
    jacobian_current(:, column) = (imu_preintegration_residual(previous_state, plus, preint, gravity_enu) - ...
        imu_preintegration_residual(previous_state, minus, preint, gravity_enu)) / (2 * steps(column));
end
end

function state = local_apply_increment(state, increment)
state.p = state.p + increment(1:3);
state.v = state.v + increment(4:6);
state.R = state.R * so3_exp(increment(7:9));
state.bg = state.bg + increment(10:12);
state.ba = state.ba + increment(13:15);
end
