function cfg = revision_experiment_config(scenario, seed)
%REVISION_EXPERIMENT_CONFIG Frozen E3/E2 scenarios; no estimator retuning.
if nargin < 2, seed = 1; end
cfg = stage1_cusum_3f3l_config('li_style_dense_random_nlos');
cfg.revision_experiment_enable = true;
cfg.revision_record_inputs = true;
cfg.revision_scenario = char(scenario);
cfg.seeds = seed;
cfg.verbose = false;
cfg.plot_position_results = false;
cfg.include_healthy_scenario = false;
cfg.revision_follower_source_indices = [1 2 3];
cfg.revision_nlos_draw_indices = 1:size(cfg.fault_segments,1);
% Changes in global node IDs are explicit; leader identity remains L1-L3.
base = cfg.fault_segments;
switch char(scenario)
    case 'baseline_3f'
    case 'healthy_maneuver'
        cfg.fault_enable = false;
        cfg.revision_relative_maneuver_enable = true;
        % E3 concerns the two detector paths, not a new four-method study.
        cfg.methods = {'cusum_ekf','cusum_fgo'};
    case 'size_2f'
        cfg.uav_num = 5;
        cfg.revision_follower_source_indices = [1 2];
        keep = base(:,3)<=2;
        cfg.fault_segments = base(keep,:);
        cfg.fault_segments(:,4) = cfg.fault_segments(:,4)-1;
        cfg.revision_nlos_draw_indices = find(keep)';
    case 'size_5f'
        cfg.uav_num = 8;
        % Columns 4 and 21 meet 500 m range and full local leader geometry.
        cfg.revision_follower_source_indices = [1 2 3 4 21];
        cfg.fault_segments(:,4) = cfg.fault_segments(:,4)+2;
        extra = base(ismember(base(:,3),[1 2]),:);
        extra(:,3) = extra(:,3)+3;
        extra(:,4) = extra(:,4)+2;
        cfg.fault_segments = [cfg.fault_segments; extra];
        cfg.revision_nlos_draw_indices = 1:size(cfg.fault_segments,1);
    case 'nlos_weak'
        cfg.revision_nlos_amplitude_scale = 0.5;
    case 'nlos_strong'
        cfg.revision_nlos_amplitude_scale = 1.5;
    otherwise
        error('revision:UnknownScenario','Unknown revision scenario: %s',char(scenario));
end
assert(cfg.ekf_cusum_alarm_on_threshold==5 && cfg.ekf_cusum_kappa==0.5 && ...
    ~cfg.imu_preintegration_enable && ~cfg.fgo_cusum_opposite_step_reset_enable && ...
    ~cfg.ekf_cusum_alarm_surrogate_enable, 'revision:BaselineChanged', ...
    'The revision must retain the manuscript detector and estimator definitions.');
revision_validate_config(cfg);
end
