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
cfg = stage1_cusum_default_config(profile);
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
end
