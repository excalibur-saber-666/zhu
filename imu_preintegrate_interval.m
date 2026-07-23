function preint = imu_preintegrate_interval(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref, bias_acc_ref, noise)
%IMU_PREINTEGRATE_INTERVAL Preintegrate body IMU samples between graph frames.
%   State convention: R maps body vectors to local ENU; gyro is rad/s and
%   accelerometer input is body-frame specific force (m/s^2).  Gravity is
%   deliberately excluded here and is applied only in the IMU factor.
gyro_rad_s = local_validate_samples(gyro_rad_s, 'gyro_rad_s');
specific_force_mps2 = local_validate_samples(specific_force_mps2, 'specific_force_mps2');
if size(gyro_rad_s, 2) ~= size(specific_force_mps2, 2)
    error('imu_preintegrate_interval:SampleCountMismatch', 'gyro and accelerometer sample counts must match.');
end
sample_count = size(gyro_rad_s, 2);
dt = local_validate_dt(dt, sample_count);
bias_gyro_ref = local_validate_vector(bias_gyro_ref, 'bias_gyro_ref');
bias_acc_ref = local_validate_vector(bias_acc_ref, 'bias_acc_ref');
local_validate_noise(noise);

core = local_integrate(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref, bias_acc_ref, noise, true);
preint = core;
preint.bias_gyro_ref = bias_gyro_ref;
preint.bias_acc_ref = bias_acc_ref;
preint.sample_count = sample_count;
preint.gyro_rad_s = gyro_rad_s;
preint.specific_force_mps2 = specific_force_mps2;
preint.dt_samples = dt;
preint.noise = noise;

gyro_eps = 1e-8;
acc_eps = 1e-6;
preint.J_R_bg = zeros(3, 3);
preint.J_v_bg = zeros(3, 3);
preint.J_v_ba = zeros(3, 3);
preint.J_p_bg = zeros(3, 3);
preint.J_p_ba = zeros(3, 3);
for axis = 1:3
    perturbation = zeros(3, 1); perturbation(axis) = gyro_eps;
    plus = local_integrate(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref + perturbation, bias_acc_ref, noise, false);
    minus = local_integrate(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref - perturbation, bias_acc_ref, noise, false);
    preint.J_R_bg(:, axis) = so3_log(minus.delta_R' * plus.delta_R) / (2 * gyro_eps);
    preint.J_v_bg(:, axis) = (plus.delta_v - minus.delta_v) / (2 * gyro_eps);
    preint.J_p_bg(:, axis) = (plus.delta_p - minus.delta_p) / (2 * gyro_eps);

    perturbation = zeros(3, 1); perturbation(axis) = acc_eps;
    plus = local_integrate(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref, bias_acc_ref + perturbation, noise, false);
    minus = local_integrate(gyro_rad_s, specific_force_mps2, dt, bias_gyro_ref, bias_acc_ref - perturbation, noise, false);
    preint.J_v_ba(:, axis) = (plus.delta_v - minus.delta_v) / (2 * acc_eps);
    preint.J_p_ba(:, axis) = (plus.delta_p - minus.delta_p) / (2 * acc_eps);
end
end

function result = local_integrate(gyro, acc, dt, bias_g, bias_a, noise, propagate_covariance)
result.delta_R = eye(3);
result.delta_v = zeros(3, 1);
result.delta_p = zeros(3, 1);
result.delta_t = sum(dt);
result.covariance = zeros(15, 15);
for index = 1:numel(dt)
    h = dt(index);
    omega = gyro(:, index) - bias_g;
    force = acc(:, index) - bias_a;
    rotation = result.delta_R;
    result.delta_p = result.delta_p + result.delta_v * h + 0.5 * rotation * force * h^2;
    result.delta_v = result.delta_v + rotation * force * h;
    result.delta_R = result.delta_R * so3_exp(omega * h);
    if propagate_covariance
        phi = eye(15);
        phi(1:3, 4:6) = eye(3) * h;
        phi(1:3, 7:9) = -0.5 * rotation * so3_hat(force) * h^2;
        phi(1:3, 13:15) = -0.5 * rotation * h^2;
        phi(4:6, 7:9) = -rotation * so3_hat(force) * h;
        phi(4:6, 13:15) = -rotation * h;
        phi(7:9, 10:12) = -eye(3) * h;
        q = diag([repmat(noise.acc_noise_std^2 * h, 1, 3), ...
                  repmat(noise.gyro_noise_std^2 * h, 1, 3), ...
                  repmat(noise.gyro_bias_rw_std^2 * h, 1, 3), ...
                  repmat(noise.acc_bias_rw_std^2 * h, 1, 3)]);
        gain = zeros(15, 12);
        gain(1:3, 1:3) = 0.5 * rotation * h^2;
        gain(4:6, 1:3) = rotation * h;
        gain(7:9, 4:6) = eye(3) * h;
        gain(10:12, 7:9) = eye(3);
        gain(13:15, 10:12) = eye(3);
        result.covariance = phi * result.covariance * phi' + gain * q * gain';
    end
end
result.covariance = (result.covariance + result.covariance') / 2 + noise.covariance_regularization * eye(15);
[chol_factor, flag] = chol(result.covariance, 'lower');
if flag ~= 0
    error('imu_preintegrate_interval:InvalidCovariance', 'Preintegration covariance is not positive definite.');
end
result.sqrt_info = chol_factor \ eye(15);
end

function samples = local_validate_samples(samples, name)
if ~isnumeric(samples) || ~isreal(samples) || size(samples, 1) ~= 3 || size(samples, 2) < 1 || any(~isfinite(samples(:)))
    error('imu_preintegrate_interval:InvalidSamples', '%s must be finite 3-by-N data with N >= 1.', name);
end
end

function dt = local_validate_dt(dt, count)
if isscalar(dt)
    dt = repmat(dt, 1, count);
end
dt = dt(:)';
if numel(dt) ~= count || any(~isfinite(dt)) || any(dt <= 0)
    error('imu_preintegrate_interval:InvalidDt', 'dt must be positive and match the IMU sample count.');
end
end

function value = local_validate_vector(value, name)
value = value(:);
if ~isnumeric(value) || ~isreal(value) || numel(value) ~= 3 || any(~isfinite(value))
    error('imu_preintegrate_interval:InvalidBias', '%s must be a finite 3-vector.', name);
end
end

function local_validate_noise(noise)
required = {'gyro_noise_std', 'acc_noise_std', 'gyro_bias_rw_std', 'acc_bias_rw_std', 'covariance_regularization'};
for index = 1:numel(required)
    name = required{index};
    if ~isfield(noise, name) || ~isscalar(noise.(name)) || ~isfinite(noise.(name)) || noise.(name) <= 0
        error('imu_preintegrate_interval:InvalidNoise', 'noise.%s must be positive and finite.', name);
    end
end
end
