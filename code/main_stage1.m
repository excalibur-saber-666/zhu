function report = main_stage1(mode, cfg)
%MAIN_STAGE1 Unified Stage-1 experiment entry point.
%   REPORT = MAIN_STAGE1(MODE, CFG) runs all four methods or one ablation:
%     four_method    EKF/FGO/CUSUM-EKF/CUSUM-FGO comparison
%     ekf            ordinary EKF only
%     fgo            ordinary single-epoch FGO only
%     cusum_ekf      adapted CUSUM-EKF only
%     cusum_fgo      established sliding-window CUSUM-FGO only
%
%   CFG may be a complete configuration from STAGE1_CUSUM_3F3L_CONFIG or a
%   partial override. Selected methods share one cached random input sequence.
%   Set CFG.IMU_PREINTEGRATION_ENABLE = true to replace only CUSUM-FGO's
%   SINS-delta motion factor with the optional IMU-preintegration factor.

if nargin < 1 || isempty(mode)
    mode = 'four_method';
elseif isstruct(mode)
    if nargin >= 2 && ~isempty(cfg)
        error('main_stage1:AmbiguousInputs', ...
            'Pass a mode and configuration, or pass one configuration structure.');
    end
    cfg = mode;
    mode = 'four_method';
end
setup_project();
if nargin < 2 || isempty(cfg)
    cfg = stage1_cusum_3f3l_config('full');
else
    cfg = local_merge_config(stage1_cusum_3f3l_config('full'), cfg);
end

mode = lower(char(mode));
switch mode
    case 'four_method'
        cfg.methods = {'ekf', 'fgo', 'cusum_ekf', 'cusum_fgo'};
    case {'ekf', 'fgo', 'cusum_ekf', 'cusum_fgo'}
        cfg.methods = {mode};
    otherwise
        error('main_stage1:UnknownMode', ...
            ['Unknown mode "%s". Use "four_method", "ekf", "fgo", ' ...
            '"cusum_ekf", or "cusum_fgo".'], mode);
end

if ~isfield(cfg, 'plot_position_results')
    cfg.plot_position_results = true;
end
report = run_stage1_cusum_comparison(cfg);
if cfg.plot_position_results
    plot_stage1_four_method_position_results(report);
end

report.selected_method = mode;
if strcmp(mode, 'four_method')
    report.selected_result_field = 'scenario_results';
else
    report.selected_result_field = mode;
end
report.selected_mode = mode;
end

function merged = local_merge_config(defaults, supplied)
merged = defaults;
fields = fieldnames(supplied);
for index = 1:numel(fields)
    merged.(fields{index}) = supplied.(fields{index});
end
end
