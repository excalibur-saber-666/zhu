function report = run_stage1_sliding_window_cusum_comparison(cfg)
%RUN_STAGE1_SLIDING_WINDOW_CUSUM_COMPARISON Original FGO versus SW+CUSUM.
%   This entry compares the original single-epoch, equal-weight factor graph
%   against a sliding-window factor graph whose range factors use the online
%   CUSUM weights.  Both methods consume the same cached IMU, GPS, and range
%   noise for every seed.  The output is a three-component error figure for
%   each follower; no data files are written.

if nargin < 1 || isempty(cfg)
    cfg = stage1_cusum_default_config('full');
else
    defaults = stage1_cusum_default_config('full');
    supplied_names = fieldnames(cfg);
    for index = 1:numel(supplied_names)
        defaults.(supplied_names{index}) = cfg.(supplied_names{index});
    end
    cfg = defaults;
end

cfg.comparison_mode = 'original_vs_sliding_cusum';
cfg.plot_component_comparison = true;
% This convenience entry always selects the comparison above.  To switch
% between all supported modes, set cfg.comparison_mode explicitly and call
% run_stage1_cusum_comparison(cfg) instead; see stage1_cusum_default_config.
% The default configuration has cfg.verbose = true, so RMSE, standard
% deviation, CUSUM-weight, and GN tables accompany the figures.  Preserve an
% explicit cfg.verbose = false for automated no-output runs.

report = run_stage1_cusum_comparison(cfg);
end
