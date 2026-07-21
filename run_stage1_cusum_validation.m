function validation = run_stage1_cusum_validation(seeds)
%RUN_STAGE1_CUSUM_VALIDATION Fixed-parameter Stage-1 fault-profile validation.
%   Uses only the offline fault configuration in the experiment runner.  The
%   online CUSUM function receives no profile, fault edge, timing, or truth.
%
%   validation = run_stage1_cusum_validation(6:20);

if nargin < 1 || isempty(seeds)
    seeds = 6:20;
end
profiles = struct( ...
    'name', {'plus3_50s', 'plus5_50s', 'plus8_10s', 'minus3_50s', 'minus5_50s', 'other_edge_plus5'}, ...
    'edge', {[2, 3], [2, 3], [2, 3], [2, 3], [2, 3], [1, 3]}, ...
    'start', {100, 100, 100, 100, 100, 100}, ...
    'stop', {150, 150, 110, 150, 150, 150}, ...
    'bias', {3, 5, 8, -3, -5, 5});
validation.seeds = seeds;
validation.profiles = repmat(struct('name', '', 'cfg', struct(), 'report', struct()), ...
    numel(profiles), 1);

fprintf('\nStage 1 fixed-parameter validation\n');
fprintf('  Seeds : %s\n', strtrim(sprintf('%g ', seeds)));
fprintf('  Output: in-memory results only\n\n');
fprintf('  %-17s %13s %13s %13s %13s\n', ...
    'Profile', 'F1 window', 'F2 window', 'F1 full', 'F2 full');
for profile_index = 1:numel(profiles)
    profile = profiles(profile_index);
    cfg = stage1_cusum_default_config('full');
    cfg.seeds = seeds;
    cfg.fault_enable = true;
    cfg.fault_edge = profile.edge;
    cfg.fault_start = profile.start;
    cfg.fault_end = profile.stop;
    cfg.fault_bias = profile.bias;
    cfg.verbose = false;
    report = run_stage1_cusum_comparison(cfg);
    aggregate = report.scenario_aggregates.fault;
    window_change = 100 * (aggregate.cusum_window_mean ./ aggregate.equal_window_mean - 1);
    full_change = 100 * (aggregate.cusum_full_mean ./ aggregate.equal_full_mean - 1);
    fprintf('  %-17s %+12.2f%% %+12.2f%% %+12.2f%% %+12.2f%%\n', ...
        profile.name, window_change(1), window_change(2), full_change(1), full_change(2));
    validation.profiles(profile_index).name = profile.name;
    validation.profiles(profile_index).cfg = cfg;
    validation.profiles(profile_index).report = report;
end
fprintf('\n');
end
