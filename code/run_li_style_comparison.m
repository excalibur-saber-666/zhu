function report = run_li_style_comparison(seed, plot_results, profile)
%RUN_LI_STYLE_COMPARISON Run one reproducible Li-style four-method trial.
%   REPORT = RUN_LI_STYLE_COMPARISON(SEED, PLOT_RESULTS) runs the dense
%   PROFILE selects the Li-style fault schedule.  The current paper profile
%   'li_style_dense_random_nlos' draws one stochastic positive NLOS bias per
%   active segment.  All four methods reuse the same IMU, GPS, range noise,
%   and complete resolved fault realization.  PROFILE defaults to the preserved
%   fixed-bias baseline 'li_style_dense'.  Use
%   'li_style_dense_random_nlos' for the current paper's stochastic NLOS
%   experiment, 'li_style_dense_tuned' to enable the opt-in
%   CUSUM-FGO multi-edge soft-weight policy, or
%   'li_style_dense_fgo_recovered' to additionally enable online predictor
%   recovery after a confirmed opposite innovation step.

if nargin < 1 || isempty(seed)
    seed = 23;
end
if nargin < 2 || isempty(plot_results)
    plot_results = true;
end
if nargin < 3 || isempty(profile)
    profile = 'li_style_dense';
end

cfg = stage1_cusum_3f3l_config(profile);
cfg.seeds = seed;
cfg.plot_position_results = logical(plot_results);
report = main_stage1('four_method', cfg);

segments = report.fault_schedules{1};
fprintf('\nDense Li-style NLOS schedule\n');
for segment_index = 1:numel(segments)
    segment = segments(segment_index);
    if isempty(segment.bias_samples)
        fprintf('  Segment%d: %.0f--%.0f s, Follower%d--Leader%d, bias %+g m\n', ...
            segment_index, segment.start, segment.end, segment.edge(1), ...
            segment.edge(2) - cfg.high_num, segment.bias);
    else
        fprintf(['  Segment%d: %.0f--%.0f s, Follower%d--Leader%d, ', ...
            'random persistent bias %.3f m\n'], ...
            segment_index, segment.start, segment.end, segment.edge(1), ...
            segment.edge(2) - cfg.high_num, segment.bias_mean);
    end
end

fprintf('\nPer-segment 3D position RMSE (m)\n');
metrics = report.fault_segment_metrics(1);
for segment_index = 1:numel(segments)
    fprintf('  Segment%d (mean %+g m, Follower%d--Leader%d)\n', ...
        segment_index, segments(segment_index).bias_mean, ...
        segments(segment_index).edge(1), ...
        segments(segment_index).edge(2) - cfg.high_num);
    for method_index = 1:numel(report.method_names)
        method_name = report.method_names{method_index};
        values = metrics.methods.(method_name).rmse_3d(segment_index, :);
        fprintf('    %-10s F1 %.6f  F2 %.6f  F3 %.6f\n', ...
            report.method_labels{method_index}, values(1), values(2), values(3));
    end
end
end
