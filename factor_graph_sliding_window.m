classdef factor_graph_sliding_window < handle
%FACTOR_GRAPH_SLIDING_WINDOW Position-only sliding-window factor graph.
%   Each key frame contains one 3-D position node per UAV.  The graph uses
%   per-frame position priors and range factors, plus SINS relative-position
%   factors between consecutive follower states.  It intentionally leaves
%   factor_graph_centralization unchanged for the Stage-1 CUSUM comparison.

    properties
        frame_count
        node_count
        parameters = []
        prior_frames = zeros(0, 1)
        prior_nodes = zeros(0, 1)
        prior_measurements = zeros(3, 0)
        prior_std = zeros(3, 0)
        range_frames = zeros(0, 1)
        range_nodes = zeros(2, 0)
        range_measurements = zeros(0, 1)
        range_std = zeros(0, 1)
        range_weights = zeros(0, 1)
        motion_previous_frames = zeros(0, 1)
        motion_current_frames = zeros(0, 1)
        motion_nodes = zeros(0, 1)
        motion_displacements = zeros(3, 0)
        motion_std = zeros(3, 0)
        A = []
        d = []
        P_all = []
        iteration_count = 0
        final_step_norm = NaN
        converged = false
    end

    methods
        function obj = factor_graph_sliding_window(frame_count, node_count)
            if ~isscalar(frame_count) || frame_count < 1 || frame_count ~= floor(frame_count) || ...
                    ~isscalar(node_count) || node_count < 1 || node_count ~= floor(node_count)
                error('factor_graph_sliding_window:InvalidSize', ...
                    'frame_count and node_count must be positive integers.');
            end
            obj.frame_count = frame_count;
            obj.node_count = node_count;
            obj.parameters = zeros(3 * frame_count * node_count, 1);
        end

        function set_frame_initial(obj, frame_index, positions)
            obj.validate_frame(frame_index);
            if ~isequal(size(positions), [3, obj.node_count]) || ...
                    ~isreal(positions) || any(~isfinite(positions(:)))
                error('factor_graph_sliding_window:InvalidInitialState', ...
                    'positions must be a finite real 3-by-node_count matrix.');
            end
            for node_index = 1:obj.node_count
                obj.parameters(obj.state_columns(frame_index, node_index)) = positions(:, node_index);
            end
        end

        function add_prior(obj, frame_index, node_index, measurement, std_value)
            obj.validate_frame(frame_index);
            obj.validate_node(node_index);
            obj.validate_vector(measurement, 'measurement');
            obj.validate_positive_vector(std_value, 'std_value');
            obj.prior_frames(end + 1, 1) = frame_index;
            obj.prior_nodes(end + 1, 1) = node_index;
            obj.prior_measurements(:, end + 1) = measurement(:);
            obj.prior_std(:, end + 1) = std_value(:);
        end

        function add_range(obj, frame_index, first_node, second_node, measurement, std_value, weight)
            % nargin includes obj: with all six public arguments supplied it
            % is 7.  The previous < 8 check silently replaced every supplied
            % CUSUM weight with one, making the sliding-window comparison an
            % equal-weight graph.
            if nargin < 7
                weight = 1;
            end
            obj.validate_frame(frame_index);
            obj.validate_node(first_node);
            obj.validate_node(second_node);
            if first_node == second_node || ~isscalar(measurement) || ~isreal(measurement) || ...
                    ~isfinite(measurement) || measurement <= 0 || ~isscalar(std_value) || ...
                    ~isreal(std_value) || ~isfinite(std_value) || std_value <= 0 || ...
                    ~isscalar(weight) || ~isreal(weight) || ~isfinite(weight) || weight <= 0
                error('factor_graph_sliding_window:InvalidRangeFactor', ...
                    'Range nodes must differ and measurement, std_value, and weight must be positive finite scalars.');
            end
            obj.range_frames(end + 1, 1) = frame_index;
            obj.range_nodes(:, end + 1) = [first_node; second_node];
            obj.range_measurements(end + 1, 1) = measurement;
            obj.range_std(end + 1, 1) = std_value;
            obj.range_weights(end + 1, 1) = weight;
        end

        function add_motion(obj, previous_frame, current_frame, node_index, displacement, std_value)
            obj.validate_frame(previous_frame);
            obj.validate_frame(current_frame);
            obj.validate_node(node_index);
            obj.validate_vector(displacement, 'displacement');
            obj.validate_positive_vector(std_value, 'std_value');
            if current_frame ~= previous_frame + 1
                error('factor_graph_sliding_window:InvalidMotionFactor', ...
                    'A motion factor must connect consecutive frames.');
            end
            obj.motion_previous_frames(end + 1, 1) = previous_frame;
            obj.motion_current_frames(end + 1, 1) = current_frame;
            obj.motion_nodes(end + 1, 1) = node_index;
            obj.motion_displacements(:, end + 1) = displacement(:);
            obj.motion_std(:, end + 1) = std_value(:);
        end

        function Gauss_Newton(obj)
            max_iterations = 30;
            threshold = 1e-5;
            obj.iteration_count = 0;
            obj.final_step_norm = NaN;
            obj.converged = false;
            for iteration = 1:max_iterations
                [obj.A, obj.d] = obj.linearized_system();
                [Q, R] = qr(obj.A, 0);
                delta = -R \ (Q' * obj.d);
                if any(~isfinite(delta))
                    error('factor_graph_sliding_window:NonFiniteStep', ...
                        'The Gauss-Newton step is non-finite.');
                end
                obj.parameters = obj.parameters + delta;
                obj.iteration_count = iteration;
                obj.final_step_norm = norm(delta, inf);
                if obj.final_step_norm < threshold
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
            obj.P_all = pinv((information + information') / 2);
            obj.P_all = (obj.P_all + obj.P_all') / 2;
        end

        function position = get_position(obj, frame_index, node_index)
            obj.validate_frame(frame_index);
            obj.validate_node(node_index);
            position = obj.parameters(obj.state_columns(frame_index, node_index));
        end

        function covariance = get_position_covariance(obj, frame_index, node_index)
            if isempty(obj.P_all)
                error('factor_graph_sliding_window:CovarianceUnavailable', ...
                    'Call covariance before requesting a position covariance.');
            end
            columns = obj.state_columns(frame_index, node_index);
            covariance = obj.P_all(columns, columns);
        end
    end

    methods (Access = private)
        function [A, d] = linearized_system(obj)
            prior_count = numel(obj.prior_frames);
            range_count = numel(obj.range_frames);
            motion_count = numel(obj.motion_nodes);
            row_count = 3 * prior_count + range_count + 3 * motion_count;
            state_count = numel(obj.parameters);
            if row_count < state_count
                error('factor_graph_sliding_window:UnderdeterminedGraph', ...
                    'The graph has fewer residual rows than state variables.');
            end
            A = zeros(row_count, state_count);
            d = zeros(row_count, 1);
            row = 0;

            for index = 1:prior_count
                columns = obj.state_columns(obj.prior_frames(index), obj.prior_nodes(index));
                sqrt_information = diag(1 ./ obj.prior_std(:, index));
                rows = row + (1:3);
                A(rows, columns) = sqrt_information;
                d(rows) = sqrt_information * ...
                    (obj.parameters(columns) - obj.prior_measurements(:, index));
                row = row + 3;
            end

            for index = 1:range_count
                frame_index = obj.range_frames(index);
                first_columns = obj.state_columns(frame_index, obj.range_nodes(1, index));
                second_columns = obj.state_columns(frame_index, obj.range_nodes(2, index));
                first_position = obj.parameters(first_columns);
                second_position = obj.parameters(second_columns);
                [residual, jacobian] = range_residual_cal(first_position, second_position, ...
                    obj.range_measurements(index));
                scale = sqrt(obj.range_weights(index)) / obj.range_std(index);
                row = row + 1;
                A(row, first_columns) = scale * jacobian(1, 1:3);
                A(row, second_columns) = scale * jacobian(1, 4:6);
                d(row) = scale * residual;
            end

            for index = 1:motion_count
                previous_columns = obj.state_columns(obj.motion_previous_frames(index), ...
                    obj.motion_nodes(index));
                current_columns = obj.state_columns(obj.motion_current_frames(index), ...
                    obj.motion_nodes(index));
                sqrt_information = diag(1 ./ obj.motion_std(:, index));
                rows = row + (1:3);
                A(rows, previous_columns) = -sqrt_information;
                A(rows, current_columns) = sqrt_information;
                d(rows) = sqrt_information * ...
                    (obj.parameters(current_columns) - obj.parameters(previous_columns) - ...
                    obj.motion_displacements(:, index));
                row = row + 3;
            end
        end

        function columns = state_columns(obj, frame_index, node_index)
            start_index = ((frame_index - 1) * obj.node_count + node_index - 1) * 3 + 1;
            columns = start_index:(start_index + 2);
        end

        function validate_frame(obj, frame_index)
            if ~isscalar(frame_index) || frame_index ~= floor(frame_index) || ...
                    frame_index < 1 || frame_index > obj.frame_count
                error('factor_graph_sliding_window:InvalidFrame', ...
                    'frame_index must identify a frame in the graph.');
            end
        end

        function validate_node(obj, node_index)
            if ~isscalar(node_index) || node_index ~= floor(node_index) || ...
                    node_index < 1 || node_index > obj.node_count
                error('factor_graph_sliding_window:InvalidNode', ...
                    'node_index must identify a node in the graph.');
            end
        end

        function validate_vector(~, value, name)
            if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 || any(~isfinite(value(:)))
                error('factor_graph_sliding_window:InvalidVector', ...
                    '%s must be a finite real three-element vector.', name);
            end
        end

        function validate_positive_vector(obj, value, name)
            obj.validate_vector(value, name);
            if any(value(:) <= 0)
                error('factor_graph_sliding_window:InvalidStd', ...
                    '%s must contain positive values.', name);
            end
        end
    end
end
