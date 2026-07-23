function noise = imu_preintegration_default_noise(cfg)
%IMU_PREINTEGRATION_DEFAULT_NOISE Map the existing IMU simulator to SI noise densities.
%   Gyro white noise in imu_err_random.m is independently sampled every dt
%   with std 10 deg/h.  Its equivalent continuous density is sigma/sqrt(dt).
%   Accelerometer Markov error has stationary std 0.001 g and time constant
%   1800 s; its driving-noise density is sqrt(2/T)*sigma.
if ~isfield(cfg, 'dt') || ~isscalar(cfg.dt) || cfg.dt <= 0
    error('imu_preintegration_default_noise:InvalidConfig', 'cfg.dt must be positive.');
end
g = 9.7803698;
gyro_sample_std = 10 * pi / (3600 * 180);
acc_stationary_std = 1e-3 * g;
gyro_markov_std = 10 * pi / (3600 * 180);
noise.gyro_noise_std = gyro_sample_std / sqrt(cfg.dt);
noise.acc_noise_std = acc_stationary_std / sqrt(cfg.dt);
noise.gyro_bias_rw_std = sqrt(2 / 3600) * gyro_markov_std;
noise.acc_bias_rw_std = sqrt(2 / 1800) * acc_stationary_std;
noise.covariance_regularization = 1e-12;
if isfield(cfg, 'imu_preint_gyro_noise_std')
    noise.gyro_noise_std = cfg.imu_preint_gyro_noise_std;
    noise.acc_noise_std = cfg.imu_preint_acc_noise_std;
    noise.gyro_bias_rw_std = cfg.imu_preint_gyro_bias_rw_std;
    noise.acc_bias_rw_std = cfg.imu_preint_acc_bias_rw_std;
end
end
