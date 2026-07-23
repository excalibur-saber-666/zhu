classdef factor_graph_sliding_window_imu_preint < handle
%FACTOR_GRAPH_SLIDING_WINDOW_IMU_PREINT Sliding-window graph with IMU preintegration.
%   Leaders keep 3-D ENU position states.  Each follower uses the local
%   15-state tangent block [p_ENU; v_ENU; theta_right; b_g; b_a].  R maps
%   body vectors to ENU and is updated only by R <- R * Exp(delta_theta).

    properties
        frame_count
        node_count
        leader_count
        follower_count
        parameters = []
        follower_rotations = []
        leader_priors = struct('frame', {}, 'leader', {}, 'measurement', {}, 'std', {})
        follower_position_priors = struct('frame', {}, 'follower', {}, 'measurement', {}, 'std', {})
        follower_priors = struct('frame', {}, 'follower', {}, 'state', {}, 'std', {})
        range_factors = struct('frame', {}, 'first_node', {}, 'second_node', {}, ...
            'measurement', {}, 'std', {}, 'weight', {})
        imu_factors = struct('previous_frame', {}, 'current_frame', {}, 'follower', {}, 'preint', {})
        gravity_enu = [0; 0; -9.7803698]
        numeric_jacobian_epsilons
        repropagate_enable = true
        gyro_bias_repropagate_threshold = 5e-5
        acc_bias_repropagate_threshold = 5e-3
        gn_max_iterations = 30
        gn_step_tolerance = 1e-5
        A = []
        d = []
        P_all = []
        covariance_mode = 'current_frame_only'
        current_follower_position_covariances = []
        iteration_count = 0
        final_step_norm = NaN
        converged = false
        repropagation_count = 0
        repropagation_seconds = 0
        repropagation_per_factor = []
    end

    methods
        function obj = factor_graph_sliding_window_imu_preint(frame_count, node_count, leader_count, cfg)
            if ~isscalar(frame_count) || frame_count < 1 || frame_count ~= floor(frame_count) || ...
                    ~isscalar(node_count) || node_count < 2 || node_count ~= floor(node_count) || ...
                    ~isscalar(leader_count) || leader_count < 1 || leader_count >= node_count || ...
                    leader_count ~= floor(leader_count)
                error('factor_graph_sliding_window_imu_preint:InvalidSize', ...
                    'frame_count, node_count, and leader_count are inconsistent.');
            end
            obj.frame_count = frame_count;
            obj.node_count = node_count;
            obj.leader_count = leader_count;
            obj.follower_count = node_count - leader_count;
            obj.parameters = zeros(frame_count * (3 * leader_count + 15 * obj.follower_count), 1);
            obj.follower_rotations = repmat(eye(3), 1, 1, frame_count, obj.follower_count);
            obj.numeric_jacobian_epsilons = struct('position', cfg.imu_preint_position_eps, ...
                'velocity', cfg.imu_preint_velocity_eps, 'rotation', cfg.imu_preint_rotation_eps, ...
                'gyro_bias', cfg.imu_preint_gyro_bias_eps, 'acc_bias', cfg.imu_preint_acc_bias_eps);
            obj.repropagate_enable = cfg.imu_preint_repropagate_enable;
            obj.gyro_bias_repropagate_threshold = cfg.imu_preint_gyro_bias_repropagate_threshold;
            obj.acc_bias_repropagate_threshold = cfg.imu_preint_acc_bias_repropagate_threshold;
            obj.gn_max_iterations = cfg.imu_preint_gn_max_iterations;
            obj.gn_step_tolerance = cfg.imu_preint_gn_step_tolerance;
            obj.covariance_mode = lower(char(cfg.imu_preint_covariance_mode));
            if ~any(strcmp(obj.covariance_mode, {'full_pinv_legacy', 'current_frame_only'}))
                error('factor_graph_sliding_window_imu_preint:InvalidCovarianceMode', ...
                    'imu_preint_covariance_mode must be ''full_pinv_legacy'' or ''current_frame_only''.');
            end
            obj.repropagation_per_factor = zeros(0, 1);
        end

        function set_frame_initial(obj, frame, node_positions, follower_velocity, follower_rotation, follower_bg, follower_ba)
            obj.validate_frame(frame);
            if ~isequal(size(node_positions), [3, obj.node_count]) || ...
                    ~isequal(size(follower_velocity), [3, obj.follower_count]) || ...
                    size(follower_rotation, 1) ~= 3 || size(follower_rotation, 2) ~= 3 || ...
                    size(follower_rotation, 3) ~= obj.follower_count || ...
                    ~isequal(size(follower_bg), [3, obj.follower_count]) || ...
                    ~isequal(size(follower_ba), [3, obj.follower_count]) || ...
                    any(~isfinite(node_positions(:))) || any(~isfinite(follower_velocity(:))) || ...
                    any(~isfinite(follower_rotation(:))) || any(~isfinite(follower_bg(:))) || any(~isfinite(follower_ba(:)))
                error('factor_graph_sliding_window_imu_preint:InvalidInitialState', ...
                    'Initial state arrays have invalid sizes or non-finite entries.');
            end
            for leader = 1:obj.leader_count
                obj.parameters(obj.leader_columns(frame, leader)) = node_positions(:, leader);
            end
            for follower = 1:obj.follower_count
                columns = obj.follower_columns(frame, follower);
                obj.parameters(columns(1:3)) = node_positions(:, obj.leader_count + follower);
                obj.parameters(columns(4:6)) = follower_velocity(:, follower);
                obj.parameters(columns(7:9)) = zeros(3, 1);
                obj.parameters(columns(10:12)) = follower_bg(:, follower);
                obj.parameters(columns(13:15)) = follower_ba(:, follower);
                obj.follower_rotations(:, :, frame, follower) = local_project_rotation(follower_rotation(:, :, follower));
            end
        end

        function add_leader_prior(obj, frame, leader, measurement, std_value)
            obj.validate_frame(frame); obj.validate_leader(leader);
            obj.validate_vector(measurement, 'measurement'); obj.validate_positive_vector(std_value, 'std_value');
            obj.leader_priors(end + 1) = struct('frame', frame, 'leader', leader, ...
                'measurement', measurement(:), 'std', std_value(:));
        end

        function add_follower_prior(obj, frame, follower, state, std_value)
            obj.validate_frame(frame); obj.validate_follower(follower);
            local_validate_follower_state(state); local_validate_prior_std(std_value);
            state.R = local_project_rotation(state.R);
            obj.follower_priors(end + 1) = struct('frame', frame, 'follower', follower, ...
                'state', state, 'std', std_value);
        end

        function add_follower_position_prior(obj, frame, follower, measurement, std_value)
            obj.validate_frame(frame); obj.validate_follower(follower);
            obj.validate_vector(measurement, 'measurement'); obj.validate_positive_vector(std_value, 'std_value');
            obj.follower_position_priors(end + 1) = struct('frame', frame, 'follower', follower, ...
                'measurement', measurement(:), 'std', std_value(:));
        end

        function add_range(obj, frame, first_node, second_node, measurement, std_value, weight)
            obj.validate_frame(frame); obj.validate_node(first_node); obj.validate_node(second_node);
            if first_node == second_node || ~isscalar(measurement) || ~isfinite(measurement) || measurement <= 0 || ...
                    ~isscalar(std_value) || ~isfinite(std_value) || std_value <= 0 || ...
                    ~isscalar(weight) || ~isfinite(weight) || weight <= 0
                error('factor_graph_sliding_window_imu_preint:InvalidRange', 'Range factor inputs are invalid.');
            end
            obj.range_factors(end + 1) = struct('frame', frame, 'first_node', first_node, ...
                'second_node', second_node, 'measurement', measurement, 'std', std_value, 'weight', weight);
        end

        function add_imu_factor(obj, previous_frame, current_frame, follower, preint)
            obj.validate_frame(previous_frame); obj.validate_frame(current_frame); obj.validate_follower(follower);
            if current_frame ~= previous_frame + 1 || ~isfield(preint, 'sqrt_info') || ...
                    ~isequal(size(preint.sqrt_info), [15, 15]) || any(~isfinite(preint.sqrt_info(:)))
                error('factor_graph_sliding_window_imu_preint:InvalidImuFactor', ...
                    'IMU factors require consecutive frames and a valid 15-by-15 sqrt_info.');
            end
            obj.imu_factors(end + 1) = struct('previous_frame', previous_frame, ...
                'current_frame', current_frame, 'follower', follower, 'preint', preint);
            obj.repropagation_per_factor(end + 1, 1) = 0;
        end

        function Gauss_Newton(obj)
            obj.iteration_count = 0; obj.final_step_norm = NaN; obj.converged = false;
            obj.repropagation_count = 0; obj.repropagation_seconds = 0;
            obj.repropagation_per_factor(:) = 0;
            for iteration = 1:obj.gn_max_iterations
                [obj.A, obj.d] = obj.linearized_system();
                [Q, R] = qr(obj.A, 0);
                if any(abs(diag(R)) < 1e-12)
                    error('factor_graph_sliding_window_imu_preint:RankDeficientGraph', ...
                        'The preintegration graph is rank deficient; inspect priors and range geometry.');
                end
                delta = -R \ (Q' * obj.d);
                if any(~isfinite(delta))
                    error('factor_graph_sliding_window_imu_preint:NonFiniteStep', 'Gauss-Newton produced a non-finite step.');
                end
                obj.apply_increment(delta);
                obj.iteration_count = iteration;
                obj.final_step_norm = norm(delta, inf);
                if obj.final_step_norm < obj.gn_step_tolerance
                    obj.converged = true;
                    break;
                end
            end
            [obj.A, obj.d] = obj.linearized_system();
        end

        function covariance(obj)
            if isempty(obj.A)
                [obj.A, obj.d] = obj.linearized_system();
            end
            information = obj.A' * obj.A;
            information = (information + information') / 2;
            obj.P_all = [];
            obj.current_follower_position_covariances = [];
            switch obj.covariance_mode
                case 'full_pinv_legacy'
                    obj.P_all = pinv(information);
                    obj.P_all = (obj.P_all + obj.P_all') / 2;
                case 'current_frame_only'
                    obj.current_follower_position_covariances = zeros(3, 3, obj.follower_count);
                    for follower = 1:obj.follower_count
                        columns = obj.follower_columns(obj.frame_count, follower);
                        position_columns = columns(1:3);
                        selector = zeros(size(information, 1), 3);
                        selector(position_columns, :) = eye(3);
                        covariance_columns = information \ selector;
                        block = covariance_columns(position_columns, :);
                        obj.current_follower_position_covariances(:, :, follower) = (block + block') / 2;
                    end
            end
        end

        function position = get_position(obj, frame, node)
            obj.validate_frame(frame); obj.validate_node(node);
            [position, ~] = obj.node_position_and_columns(frame, node);
        end

        function covariance = get_position_covariance(obj, frame, node)
            if ~isempty(obj.P_all)
                [~, columns] = obj.node_position_and_columns(frame, node);
                covariance = obj.P_all(columns, columns);
                return;
            end
            if isempty(obj.current_follower_position_covariances) || frame ~= obj.frame_count || node <= obj.leader_count
                error('factor_graph_sliding_window_imu_preint:CovarianceUnavailable', ...
                    'Current-frame follower covariance is unavailable; use full_pinv_legacy for other blocks.');
            end
            covariance = obj.current_follower_position_covariances(:, :, node - obj.leader_count);
        end

        function state = get_follower_state(obj, frame, follower)
            obj.validate_frame(frame); obj.validate_follower(follower);
            state = obj.follower_state(frame, follower);
        end
    end

    methods (Access = private)
        function [A, d] = linearized_system(obj)
            row_count = 3 * numel(obj.leader_priors) + 3 * numel(obj.follower_position_priors) + ...
                15 * numel(obj.follower_priors) + ...
                numel(obj.range_factors) + 15 * numel(obj.imu_factors);
            state_count = numel(obj.parameters);
            if row_count < state_count
                error('factor_graph_sliding_window_imu_preint:UnderdeterminedGraph', ...
                    'Residual rows are fewer than graph state variables.');
            end
            A = zeros(row_count, state_count); d = zeros(row_count, 1); row = 0;
            for index = 1:numel(obj.leader_priors)
                factor = obj.leader_priors(index); columns = obj.leader_columns(factor.frame, factor.leader);
                rows = row + (1:3); sqrt_info = diag(1 ./ factor.std);
                A(rows, columns) = sqrt_info; d(rows) = sqrt_info * (obj.parameters(columns) - factor.measurement); row = row + 3;
            end
            for index = 1:numel(obj.follower_position_priors)
                factor = obj.follower_position_priors(index);
                columns = obj.follower_columns(factor.frame, factor.follower); columns = columns(1:3);
                rows = row + (1:3); sqrt_info = diag(1 ./ factor.std);
                A(rows, columns) = sqrt_info;
                d(rows) = sqrt_info * (obj.parameters(columns) - factor.measurement);
                row = row + 3;
            end
            for index = 1:numel(obj.follower_priors)
                factor = obj.follower_priors(index); columns = obj.follower_columns(factor.frame, factor.follower);
                state = obj.follower_state(factor.frame, factor.follower); residual = local_follower_prior_residual(state, factor.state);
                sqrt_info = diag(1 ./ local_prior_std_vector(factor.std)); rows = row + (1:15);
                A(rows, columns) = sqrt_info; d(rows) = sqrt_info * residual; row = row + 15;
            end
            for index = 1:numel(obj.range_factors)
                factor = obj.range_factors(index); [first_position, first_columns] = obj.node_position_and_columns(factor.frame, factor.first_node);
                [second_position, second_columns] = obj.node_position_and_columns(factor.frame, factor.second_node);
                [residual, jacobian] = range_residual_cal(first_position, second_position, factor.measurement);
                scale = sqrt(factor.weight) / factor.std; row = row + 1;
                A(row, first_columns) = scale * jacobian(1, 1:3); A(row, second_columns) = scale * jacobian(1, 4:6); d(row) = scale * residual;
            end
            for index = 1:numel(obj.imu_factors)
                factor = obj.imu_factors(index); preint = obj.maybe_repropagate(index);
                previous = obj.follower_state(factor.previous_frame, factor.follower);
                current = obj.follower_state(factor.current_frame, factor.follower);
                [residual, jacobian_previous, jacobian_current] = imu_preintegration_numeric_jacobian( ...
                    previous, current, preint, obj.gravity_enu, obj.numeric_jacobian_epsilons);
                rows = row + (1:15); sqrt_info = preint.sqrt_info;
                A(rows, obj.follower_columns(factor.previous_frame, factor.follower)) = sqrt_info * jacobian_previous;
                A(rows, obj.follower_columns(factor.current_frame, factor.follower)) = sqrt_info * jacobian_current;
                d(rows) = sqrt_info * residual; row = row + 15;
            end
        end

        function preint = maybe_repropagate(obj, index)
            factor = obj.imu_factors(index);
            preint = factor.preint;
            if ~obj.repropagate_enable
                return;
            end
            previous = obj.follower_state(factor.previous_frame, factor.follower);
            if norm(previous.bg - preint.bias_gyro_ref) <= obj.gyro_bias_repropagate_threshold && ...
                    norm(previous.ba - preint.bias_acc_ref) <= obj.acc_bias_repropagate_threshold
                return;
            end
            repropagation_timer = tic;
            preint = imu_preintegrate_interval(preint.gyro_rad_s, preint.specific_force_mps2, ...
                preint.dt_samples, previous.bg, previous.ba, preint.noise);
            obj.imu_factors(index).preint = preint;
            obj.repropagation_count = obj.repropagation_count + 1;
            obj.repropagation_per_factor(index) = obj.repropagation_per_factor(index) + 1;
            obj.repropagation_seconds = obj.repropagation_seconds + toc(repropagation_timer);
        end

        function apply_increment(obj, increment)
            for frame = 1:obj.frame_count
                for leader = 1:obj.leader_count
                    columns = obj.leader_columns(frame, leader); obj.parameters(columns) = obj.parameters(columns) + increment(columns);
                end
                for follower = 1:obj.follower_count
                    columns = obj.follower_columns(frame, follower);
                    obj.parameters(columns([1:6, 10:15])) = obj.parameters(columns([1:6, 10:15])) + increment(columns([1:6, 10:15]));
                    obj.follower_rotations(:, :, frame, follower) = obj.follower_rotations(:, :, frame, follower) * so3_exp(increment(columns(7:9)));
                    obj.parameters(columns(7:9)) = zeros(3, 1);
                end
            end
        end

        function state = follower_state(obj, frame, follower)
            columns = obj.follower_columns(frame, follower);
            state.p = obj.parameters(columns(1:3)); state.v = obj.parameters(columns(4:6));
            state.R = obj.follower_rotations(:, :, frame, follower) * so3_exp(obj.parameters(columns(7:9)));
            state.bg = obj.parameters(columns(10:12)); state.ba = obj.parameters(columns(13:15));
        end

        function [position, columns] = node_position_and_columns(obj, frame, node)
            if node <= obj.leader_count
                columns = obj.leader_columns(frame, node); position = obj.parameters(columns);
            else
                columns = obj.follower_columns(frame, node - obj.leader_count); columns = columns(1:3); position = obj.parameters(columns);
            end
        end

        function columns = leader_columns(obj, frame, leader)
            start_index = (frame - 1) * (3 * obj.leader_count + 15 * obj.follower_count) + (leader - 1) * 3 + 1;
            columns = start_index:(start_index + 2);
        end

        function columns = follower_columns(obj, frame, follower)
            start_index = (frame - 1) * (3 * obj.leader_count + 15 * obj.follower_count) + ...
                3 * obj.leader_count + (follower - 1) * 15 + 1;
            columns = start_index:(start_index + 14);
        end

        function validate_frame(obj, frame)
            if ~isscalar(frame) || frame ~= floor(frame) || frame < 1 || frame > obj.frame_count
                error('factor_graph_sliding_window_imu_preint:InvalidFrame', 'Invalid frame index.');
            end
        end
        function validate_node(obj, node)
            if ~isscalar(node) || node ~= floor(node) || node < 1 || node > obj.node_count
                error('factor_graph_sliding_window_imu_preint:InvalidNode', 'Invalid node index.');
            end
        end
        function validate_leader(obj, leader)
            if ~isscalar(leader) || leader ~= floor(leader) || leader < 1 || leader > obj.leader_count
                error('factor_graph_sliding_window_imu_preint:InvalidLeader', 'Invalid leader index.');
            end
        end
        function validate_follower(obj, follower)
            if ~isscalar(follower) || follower ~= floor(follower) || follower < 1 || follower > obj.follower_count
                error('factor_graph_sliding_window_imu_preint:InvalidFollower', 'Invalid follower index.');
            end
        end
        function validate_vector(~, value, name)
            if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 || any(~isfinite(value(:)))
                error('factor_graph_sliding_window_imu_preint:InvalidVector', '%s must be a finite 3-vector.', name);
            end
        end
        function validate_positive_vector(obj, value, name)
            obj.validate_vector(value, name); if any(value(:) <= 0), error('factor_graph_sliding_window_imu_preint:InvalidStd', '%s must be positive.', name); end
        end
    end
end

function local_validate_follower_state(state)
required = {'p', 'v', 'R', 'bg', 'ba'};
for index = 1:numel(required)
    if ~isfield(state, required{index}), error('factor_graph_sliding_window_imu_preint:InvalidState', 'Missing state.%s.', required{index}); end
end
if any(~isfinite([state.p(:); state.v(:); state.bg(:); state.ba(:)])) || ...
        numel(state.p) ~= 3 || numel(state.v) ~= 3 || numel(state.bg) ~= 3 || numel(state.ba) ~= 3 || ...
        ~isequal(size(state.R), [3, 3]) || any(~isfinite(state.R(:)))
    error('factor_graph_sliding_window_imu_preint:InvalidState', 'Follower prior state is invalid.');
end
end

function local_validate_prior_std(std_value)
fields = {'position', 'velocity', 'rotation', 'gyro_bias', 'acc_bias'};
for index = 1:numel(fields)
    value = std_value.(fields{index});
    if numel(value) ~= 3 || any(~isfinite(value(:))) || any(value(:) <= 0)
        error('factor_graph_sliding_window_imu_preint:InvalidStd', 'Follower prior std.%s must be positive 3-vector.', fields{index});
    end
end
end

function vector = local_prior_std_vector(std_value)
vector = [std_value.position(:); std_value.velocity(:); std_value.rotation(:); ...
    std_value.gyro_bias(:); std_value.acc_bias(:)];
end

function residual = local_follower_prior_residual(state, prior)
residual = [state.p - prior.p; state.v - prior.v; so3_log(prior.R' * state.R); ...
    state.bg - prior.bg; state.ba - prior.ba];
end

function rotation = local_project_rotation(rotation)
[left, ~, right] = svd(rotation); rotation = left * diag([1, 1, det(left * right')]) * right';
end
