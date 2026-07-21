function diagnostics = test_stage1_redundant_geometry()
%TEST_STAGE1_REDUNDANT_GEOMETRY Verify Leader4 coordinates and observability.
%   This mirrors the truth-trajectory propagation in the Stage-1 runner,
%   without constructing a graph or using any random measurements.

cfg = stage1_cusum_redundant_config('full');
low_num = cfg.uav_num - cfg.high_num;
position_origin = [118; 32; 200];
position_east = load('posi_e_all.dat');
position_north = load('posi_n_all.dat');
position_up = load('posi_u_all.dat');

follower_geodetic = zeros(3, low_num);
leader_geodetic = zeros(3, cfg.high_num);
for vehicle = 1:low_num
    follower_geodetic(:, vehicle) = posical_enu([ ...
        position_east(1, vehicle); position_north(1, vehicle); position_up(1, vehicle)], ...
        position_origin);
end
for leader = 1:3
    source_index = low_num + leader;
    leader_geodetic(:, leader) = posical_enu([ ...
        position_east(1, source_index); position_north(1, source_index); position_up(1, source_index)], ...
        position_origin);
end
leader_geodetic(:, 4) = posical_enu(cfg.additional_leader_positions_xyz(:, 1), position_origin);
leader4_initial_xyz = posical_xyz(leader_geodetic(:, 4), position_origin);
assert(max(abs(leader4_initial_xyz - cfg.additional_leader_positions_xyz(:, 1))) < 1e-3, ...
    'Leader4 local ENU coordinates were not converted to geodetic coordinates correctly.');

step_count = round(cfg.t_stop / cfg.dt);
distance_min = inf(low_num, 1);
distance_max = -inf(low_num, 1);
link_count = zeros(low_num, 1);
minimum_singular_value = inf(low_num, 1);
maximum_condition_number = zeros(low_num, 1);
minimum_rank = inf(low_num, 1);
leave_one_out_minimum_singular_value = inf(low_num, cfg.high_num);
leave_one_out_maximum_condition_number = zeros(low_num, cfg.high_num);
leave_one_out_minimum_rank = inf(low_num, cfg.high_num);

velocity_body = zeros(3, cfg.uav_num);
velocity_body(2, :) = 5;
attitude = repmat([0; 0; 90], 1, cfg.uav_num);
attitude_rate = zeros(3, cfg.uav_num);
acceleration_body = zeros(3, cfg.uav_num);
for sample = 1:(step_count + 1)
    follower_xyz = zeros(3, low_num);
    leader_xyz = zeros(3, cfg.high_num);
    for vehicle = 1:low_num
        follower_xyz(:, vehicle) = posical_xyz(follower_geodetic(:, vehicle), position_origin);
    end
    for leader = 1:cfg.high_num
        leader_xyz(:, leader) = posical_xyz(leader_geodetic(:, leader), position_origin);
    end
    for follower = 1:low_num
        leader4_distance = norm(follower_xyz(:, follower) - leader_xyz(:, 4));
        distance_min(follower) = min(distance_min(follower), leader4_distance);
        distance_max(follower) = max(distance_max(follower), leader4_distance);
        link_count(follower) = link_count(follower) + (leader4_distance <= cfg.communication_range);

        line_of_sight = leader_xyz - follower_xyz(:, follower);
        line_of_sight = line_of_sight ./ vecnorm(line_of_sight, 2, 1);
        singular_values = svd(line_of_sight');
        minimum_singular_value(follower) = min(minimum_singular_value(follower), singular_values(end));
        maximum_condition_number(follower) = max(maximum_condition_number(follower), ...
            singular_values(1) / singular_values(end));
        minimum_rank(follower) = min(minimum_rank(follower), rank(line_of_sight'));
        for omitted_leader = 1:cfg.high_num
            retained_leaders = setdiff(1:cfg.high_num, omitted_leader);
            retained_singular_values = svd(line_of_sight(:, retained_leaders)');
            leave_one_out_minimum_singular_value(follower, omitted_leader) = ...
                min(leave_one_out_minimum_singular_value(follower, omitted_leader), ...
                retained_singular_values(end));
            leave_one_out_maximum_condition_number(follower, omitted_leader) = ...
                max(leave_one_out_maximum_condition_number(follower, omitted_leader), ...
                retained_singular_values(1) / retained_singular_values(end));
            leave_one_out_minimum_rank(follower, omitted_leader) = ...
                min(leave_one_out_minimum_rank(follower, omitted_leader), ...
                rank(line_of_sight(:, retained_leaders)'));
        end
    end

    if sample > step_count
        break;
    end
    current_time = sample * cfg.dt;
    [~, attitude(:, 1), attitude_rate(:, 1), velocity_body(:, 1), acceleration_body(:, 1)] = ...
        trace(current_time - cfg.dt, cfg.dt, attitude(:, 1), attitude_rate(:, 1), ...
        velocity_body(:, 1), acceleration_body(:, 1));
    for vehicle = 2:cfg.uav_num
        attitude(:, vehicle) = attitude(:, 1);
        attitude_rate(:, vehicle) = attitude_rate(:, 1);
        velocity_body(:, vehicle) = velocity_body(:, 1);
        acceleration_body(:, vehicle) = acceleration_body(:, 1);
    end
    for vehicle = 1:low_num
        follower_geodetic(:, vehicle) = posi_out(cfg.dt, follower_geodetic(:, vehicle), ...
            velocity_body(:, vehicle), attitude(:, vehicle));
    end
    for leader = 1:cfg.high_num
        leader_geodetic(:, leader) = posi_out(cfg.dt, leader_geodetic(:, leader), ...
            velocity_body(:, low_num + leader), attitude(:, low_num + leader));
    end
end

diagnostics.leader4_initial_xyz = leader4_initial_xyz;
diagnostics.leader4_distance_min = distance_min;
diagnostics.leader4_distance_max = distance_max;
diagnostics.leader4_link_uptime = link_count / (step_count + 1);
diagnostics.minimum_los_rank = minimum_rank;
diagnostics.minimum_los_singular_value = minimum_singular_value;
diagnostics.maximum_los_condition_number = maximum_condition_number;
diagnostics.leave_one_out_minimum_los_rank = leave_one_out_minimum_rank;
diagnostics.leave_one_out_minimum_los_singular_value = leave_one_out_minimum_singular_value;
diagnostics.leave_one_out_maximum_los_condition_number = leave_one_out_maximum_condition_number;
for follower = 1:low_num
    fprintf(['Follower%d--Leader4: range %.3f-%.3f m, uptime %.2f%%, ' ...
        'min sv %.6f, max cond %.3f\n'], follower, distance_min(follower), ...
        distance_max(follower), 100 * diagnostics.leader4_link_uptime(follower), ...
        minimum_singular_value(follower), maximum_condition_number(follower));
    fprintf('Follower%d leave-one-out: min sv [%s], max cond [%s]\n', follower, ...
        num2str(leave_one_out_minimum_singular_value(follower, :), ' %.6f'), ...
        num2str(leave_one_out_maximum_condition_number(follower, :), ' %.3f'));
end

assert(all(diagnostics.leader4_link_uptime == 1), ...
    'Leader4 is not within the configured communication range for the full run.');
assert(all(diagnostics.minimum_los_rank == 3) && ...
    all(diagnostics.minimum_los_singular_value > 1e-6), ...
    'The four leader lines of sight do not provide full 3-D geometric rank.');
assert(all(diagnostics.leave_one_out_minimum_los_rank(:) == 3) && ...
    all(diagnostics.leave_one_out_minimum_los_singular_value(:) > 1e-6), ...
    'Removing one leader leaves an insufficiently ranked three-leader geometry.');
fprintf('test_stage1_redundant_geometry: PASS\n');
end
