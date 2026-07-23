function test_imu_preintegration_static_with_gravity()
%TEST_IMU_PREINTEGRATION_STATIC_WITH_GRAVITY Verify stationary ENU dynamics.
cfg = stage1_cusum_default_config('quick');
noise = imu_preintegration_default_noise(cfg);
sample_count = round(cfg.graph_interval / cfg.dt);
gravity = [0; 0; -9.7803698];
specific_force = repmat(-gravity, 1, sample_count);
preint = imu_preintegrate_interval(zeros(3, sample_count), specific_force, cfg.dt, ...
    zeros(3, 1), zeros(3, 1), noise);
previous = local_state([3; -2; 7], [0; 0; 0], eye(3), zeros(3, 1), zeros(3, 1));
current = previous;
residual = imu_preintegration_residual(previous, current, preint, gravity);
assert(norm(residual(1:9), inf) < 1e-10, ...
    'A stationary IMU with gravity-compensating specific force must have zero motion residual.');
assert(norm(residual(10:15), inf) < 1e-12);
fprintf('test_imu_preintegration_static_with_gravity: PASS\n');
end

function state = local_state(p, v, R, bg, ba)
state = struct('p', p, 'v', v, 'R', R, 'bg', bg, 'ba', ba);
end
