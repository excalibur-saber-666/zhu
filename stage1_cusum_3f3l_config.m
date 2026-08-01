function cfg = stage1_cusum_3f3l_config(profile)
%STAGE1_CUSUM_3F3L_CONFIG Six-UAV experiment with three followers and leaders.
%   UAVs 1--3 are followers.  UAVs 4--6 are leaders.  The source columns
%   below were selected from the supplied initial-position data so every
%   follower-leader range is within the existing 500 m communication limit
%   and the three leader line-of-sight vectors have rank three.  All vehicles
%   retain the existing prescribed trajectory model.

if nargin < 1 || isempty(profile)
    profile = 'full';
end
requested_profile = lower(char(profile));
base_profile = requested_profile;
if any(strcmp(requested_profile, {'li_style', 'li_style_dense', ...
        'li_style_dense_tuned', 'li_style_dense_fgo_recovered'}))
    base_profile = 'full';
end
cfg = stage1_cusum_default_config(base_profile);
cfg.uav_num = 6;
cfg.high_num = 3;
cfg.additional_leader_positions_xyz = zeros(3, 0);
cfg.base_leader_source_indices = [11, 14, 15];

% Global node numbering is [Follower1, Follower2, Follower3, ...
% Leader1, Leader2, Leader3].  This keeps the former type of test fault on
% Follower2--Leader1; [2, 3] would instead be a follower-to-follower edge.
cfg.fault_edge = [2, 4];

% With only three leader edges per follower, leader-only median consensus is
% not robust to one faulty leader edge.  Use the calibrated per-edge CUSUM
% statistic, while retaining the online rule that withholds at most the most
% suspicious leader edge for each follower.
cfg.cusum_consensus_enable = false;
cfg.cusum_consensus_leader_only = false;
cfg.sliding_window_cusum_consensus_enable = false;
cfg.sliding_window_alarm_exclusion_mode = 'per_follower_max';

% In the 3F3L comparison, graph-prior innovations can absorb a persistent
% range bias through estimator feedback and can accumulate on the wrong
% leader edge for some seeds.  Use the independent online range predictor
% for detection while retaining the complete CUSUM-FGO soft-weighting,
% sliding-window optimization, and confirmed-alarm selective isolation.
cfg.fgo_cusum_detector_mode = 'range_predictor';

% Li-style simulation variant: three separated fault periods have increasing
% amplitudes, while one distinct follower-leader edge is selected
% reproducibly for each period from cfg.seeds.  This profile is opt-in so
% the established single-fault baseline remains unchanged.
if strcmp(requested_profile, 'li_style')
    cfg.mode = 'li_style';
    cfg.fault_enable = true;
    cfg.include_healthy_scenario = false;
    cfg.fault_mode = 'segmented_random_edge';
    cfg.fault_segments = [100, 115, 3; ...
                          280, 300, 4; ...
                          470, 480, 6];
end

% Dense Li-style simulation variant: the fixed schedule contains both
% isolated and simultaneous follower-leader range faults.  It is separate
% from li_style so the previous three-segment random-edge baseline remains
% reproducible.  At a shared epoch, each follower has at most one corrupted
% leader range; this preserves the intended selective-isolation geometry.
if any(strcmp(requested_profile, {'li_style_dense', 'li_style_dense_tuned', ...
        'li_style_dense_fgo_recovered'}))
    cfg.mode = 'li_style_dense';
    cfg.fault_enable = true;
    cfg.include_healthy_scenario = false;
    cfg.fault_mode = 'segmented_explicit_edges';
    % Rows: [start_s, end_s, additive_bias_m, follower, leader_global].
    cfg.fault_segments = [ ...
         80,  92, 3, 1, 4; ...
        120, 135, 4, 2, 5; ...
        170, 185, 3, 3, 6; ...
        245, 262, 4, 1, 5; ...
        245, 262, 6, 2, 4; ...
        285, 302, 6, 3, 4; ...
        310, 326, 4, 2, 6; ...
        365, 380, 3, 1, 6; ...
        365, 380, 4, 3, 5; ...
        430, 447, 6, 2, 5; ...
        485, 500, 3, 1, 4; ...
        525, 545, 4, 3, 6; ...
        535, 552, 6, 2, 4];
end

% Tuned dense profile: retain the independent range-predictor CUSUM and
% per-follower confirmed-alarm isolation, but soften every leader edge that
% is currently suspicious for the same follower.  This prevents a second
% large-CUSUM leader range from entering a window at unit weight while the
% largest one is selectively isolated.  It is opt-in, leaves li_style_dense
% unchanged, and uses no truth, fault-time, or fault-amplitude information.
if any(strcmp(requested_profile, {'li_style_dense_tuned', ...
        'li_style_dense_fgo_recovered'}))
    cfg.mode = requested_profile;
    cfg.fgo_cusum_soft_weight_mode = 'all';
    % CUSUM-EKF confirms a large step at its first observed epoch, isolates
    % the raw range, and adds only a low-information online-predictor
    % surrogate while the alarm is active.  This prevents abrupt loss of
    % range geometry without using fault metadata or retaining the raw edge.
    cfg.ekf_cusum_alarm_on_threshold = 4.0;
    cfg.ekf_cusum_alarm_confirm_epochs = 1;
    cfg.ekf_cusum_weight_deadzone = 0;
    cfg.ekf_cusum_weight_gain = 100;
    cfg.ekf_cusum_weight_min = 1e-4;
    cfg.ekf_cusum_alarm_release_epochs = 1;
    cfg.ekf_cusum_alarm_surrogate_enable = true;
    cfg.ekf_cusum_alarm_surrogate_weight = 0.20;
end

% Recovery variant: retain li_style_dense_tuned exactly, and opt in to the
% FGO-only predictor recovery separately.  This keeps prior 50-trial results
% reproducible while addressing a stale-alarm geometry loss seen in seed 40.
if strcmp(requested_profile, 'li_style_dense_fgo_recovered')
    cfg.fgo_cusum_opposite_step_reset_enable = true;
    cfg.fgo_cusum_opposite_step_reset_threshold = 2.5;
    cfg.fgo_cusum_opposite_step_reset_epochs = 2;
end

end
