function residual = imu_preintegration_residual(previous_state, current_state, preint, gravity_enu)
%IMU_PREINTEGRATION_RESIDUAL Fifteen-dimensional ENU/body IMU factor residual.
%   Residual order is [position; velocity; rotation; gyro bias; accel bias].
gravity_enu = gravity_enu(:);
if numel(gravity_enu) ~= 3 || any(~isfinite(gravity_enu))
    error('imu_preintegration_residual:InvalidGravity', 'gravity_enu must be a finite 3-vector.');
end
local_validate_state(previous_state, 'previous_state');
local_validate_state(current_state, 'current_state');
required = {'delta_R', 'delta_v', 'delta_p', 'delta_t', 'bias_gyro_ref', 'bias_acc_ref', ...
    'J_R_bg', 'J_v_bg', 'J_v_ba', 'J_p_bg', 'J_p_ba'};
for index = 1:numel(required)
    if ~isfield(preint, required{index})
        error('imu_preintegration_residual:InvalidPreintegration', 'Missing preint.%s.', required{index});
    end
end
delta_bg = previous_state.bg - preint.bias_gyro_ref;
delta_ba = previous_state.ba - preint.bias_acc_ref;
corrected_rotation = preint.delta_R * so3_exp(preint.J_R_bg * delta_bg);
corrected_velocity = preint.delta_v + preint.J_v_bg * delta_bg + preint.J_v_ba * delta_ba;
corrected_position = preint.delta_p + preint.J_p_bg * delta_bg + preint.J_p_ba * delta_ba;
duration = preint.delta_t;
relative_rotation = previous_state.R' * current_state.R;
residual_position = previous_state.R' * (current_state.p - previous_state.p - ...
    previous_state.v * duration - 0.5 * gravity_enu * duration^2) - corrected_position;
residual_velocity = previous_state.R' * (current_state.v - previous_state.v - gravity_enu * duration) - corrected_velocity;
residual_rotation = so3_log(corrected_rotation' * relative_rotation);
residual = [residual_position; residual_velocity; residual_rotation; ...
    current_state.bg - previous_state.bg; current_state.ba - previous_state.ba];
end

function local_validate_state(state, name)
required = {'p', 'v', 'R', 'bg', 'ba'};
for index = 1:numel(required)
    field = required{index};
    if ~isfield(state, field)
        error('imu_preintegration_residual:InvalidState', '%s.%s is required.', name, field);
    end
end
vectors = {'p', 'v', 'bg', 'ba'};
for index = 1:numel(vectors)
    value = state.(vectors{index});
    if numel(value) ~= 3 || any(~isfinite(value(:)))
        error('imu_preintegration_residual:InvalidState', '%s.%s must be finite 3-vector.', name, vectors{index});
    end
end
if ~isequal(size(state.R), [3, 3]) || any(~isfinite(state.R(:)))
    error('imu_preintegration_residual:InvalidState', '%s.R must be finite 3-by-3.', name);
end
end
