function noise = imu_preintegration_default_noise(cfg)
%IMU_PREINTEGRATION_DEFAULT_NOISE Map imu_err_random.m to SI noise densities.
%   Gyro_wg is independent discrete white noise with a 10 deg/h sample
%   standard deviation, hence its continuous density is sigma/sqrt(dt).
%   Gyro_r and Acc_r are first-order Markov errors.  Their corresponding
%   random-walk approximations use sqrt(2/T)*stationary_sigma as driving
%   densities.  The simulator has no independent accelerometer-white-noise
%   term, so acc_noise_std is deliberately zero rather than double-counting
%   Acc_r as both measurement noise and a bias process.
if ~isfield(cfg, 'dt') || ~isscalar(cfg.dt) || cfg.dt <= 0
    error('imu_preintegration_default_noise:InvalidConfig', 'cfg.dt must be positive.');
end
g = 9.7803698;
gyro_sample_std = 10 * pi / (3600 * 180);
acc_stationary_std = 1e-3 * g;
gyro_markov_std = 10 * pi / (3600 * 180);
noise.gyro_noise_std = gyro_sample_std / sqrt(cfg.dt);
noise.acc_noise_std = 0;
noise.gyro_bias_rw_std = sqrt(2 / 3600) * gyro_markov_std;
noise.acc_bias_rw_std = sqrt(2 / 1800) * acc_stationary_std;
noise.covariance_regularization = 1e-12;
if isfield(cfg, 'imu_preint_gyro_noise_std')
    noise.gyro_noise_std = cfg.imu_preint_gyro_noise_std;
    noise.acc_noise_std = cfg.imu_preint_acc_noise_std;
    noise.gyro_bias_rw_std = cfg.imu_preint_gyro_bias_rw_std;
    noise.acc_bias_rw_std = cfg.imu_preint_acc_bias_rw_std;
end
if isfield(cfg, 'imu_preint_covariance_regularization')
    noise.covariance_regularization = cfg.imu_preint_covariance_regularization;
end
end
