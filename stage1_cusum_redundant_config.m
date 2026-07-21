function cfg = stage1_cusum_redundant_config(profile)
%STAGE1_CUSUM_REDUNDANT_CONFIG Independent 2-follower, 4-leader experiment.
%   The original five-UAV Stage-1 setup is left unchanged.  This companion
%   configuration adds one high-precision leader at a fixed local position;
%   it shares the original prescribed flight manoeuvre, so the existing five
%   trajectories are never edited.

if nargin < 1 || isempty(profile)
    profile = 'full';
end
cfg = stage1_cusum_default_config(profile);
cfg.uav_num = 6;
cfg.high_num = 4;

% Leader4 is placed east/south-east of the existing formation.  It remains
% within the 500 m communication radius of both followers and adds a
% materially different horizontal line of sight.
cfg.additional_leader_positions_xyz = [600; 250; 300];

% With four leaders, a leader edge has three alternate leader edges.  The
% median is then robust to one bad range, unlike the original three-leader
% geometry.  The inter-follower range is excluded from this consensus.
cfg.cusum_consensus_enable = true;
cfg.cusum_consensus_min_neighbors = 3;
cfg.cusum_consensus_leader_only = true;
cfg.sliding_window_cusum_consensus_enable = true;

% At most one leader range for a follower is withheld at a key frame.  This
% prevents a contaminated follower state from removing all of its healthy
% geometric constraints after a single-range fault.
cfg.sliding_window_alarm_exclusion_mode = 'per_follower_max';
end
