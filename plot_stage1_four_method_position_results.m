function plot_stage1_four_method_position_results(report, scenario_name)
%PLOT_STAGE1_FOUR_METHOD_POSITION_RESULTS Plot stored four-method positions.
%   This plotting helper only reads REPORT; it never reruns a simulation or
%   writes image/result files.

if nargin < 2 || isempty(scenario_name)
    scenario_name = report.position_metrics.primary_scenario;
end
if ~isfield(report, 'scenario_results') || ~isfield(report.scenario_results, scenario_name) || ...
        ~isfield(report, 'method_names') || isempty(report.method_names) || ...
        numel(report.method_names) > 4
    error('plot_stage1_four_method_position_results:InvalidReport', ...
        'report must contain one to four selected Stage-1 methods to plot.');
end
results = report.scenario_results.(scenario_name);
method_names = report.method_names;
method_labels = report.method_labels;
time = results(1).(method_names{1}).time;
low_num = size(results(1).(method_names{1}).truth_xyz, 3);
component_labels = {'East position error / m', 'North position error / m', 'Up position error / m'};
position_labels = {'East position / m', 'North position / m', 'Up position / m'};
line_styles = {'b-', 'r-', 'm-', 'g-'};

for follower = 1:low_num
    errors = zeros(numel(time), 3, numel(method_names));
    estimates = zeros(numel(time), 3, numel(method_names));
    truth = zeros(numel(time), 3);
    for seed_index = 1:numel(results)
        truth = truth + results(seed_index).(method_names{1}).truth_xyz(:, :, follower);
        for method_index = 1:numel(method_names)
            result = results(seed_index).(method_names{method_index});
            errors(:, :, method_index) = errors(:, :, method_index) + result.error_xyz(:, :, follower);
            estimates(:, :, method_index) = estimates(:, :, method_index) + result.navigation_xyz(:, :, follower);
        end
    end
    truth = truth / numel(results);
    errors = errors / numel(results);
    estimates = estimates / numel(results);

    figure('Name', sprintf('Follower%d selected-method position errors (%s)', follower, scenario_name), ...
        'NumberTitle', 'off');
    for component = 1:3
        subplot(3, 1, component); hold on;
        for method_index = 1:numel(method_names)
            plot(time, errors(:, component, method_index), line_styles{method_index}, 'LineWidth', 1.0);
        end
        if strcmp(scenario_name, 'fault') && report.cfg.fault_enable
            local_plot_fault_boundaries(report);
        end
        grid on; ylabel(component_labels{component});
        if component == 1, legend(method_labels, 'Location', 'best'); end
        if component == 3, xlabel('Time / s'); end
    end

    figure('Name', sprintf('Follower%d truth and selected-method positions (%s)', follower, scenario_name), ...
        'NumberTitle', 'off');
    for component = 1:3
        subplot(3, 1, component); hold on;
        plot(time, truth(:, component), 'k-', 'LineWidth', 1.2);
        for method_index = 1:numel(method_names)
            plot(time, estimates(:, component, method_index), line_styles{method_index}, 'LineWidth', 1.0);
        end
        grid on; ylabel(position_labels{component});
        if component == 1, legend([{'Truth'}, method_labels], 'Location', 'best'); end
        if component == 3, xlabel('Time / s'); end
    end
end
end

function local_plot_fault_boundaries(report)
if isfield(report, 'fault_schedules') && ~isempty(report.fault_schedules) && ...
        ~isempty(report.fault_schedules{1})
    segments = report.fault_schedules{1};
    for segment_index = 1:numel(segments)
        xline(segments(segment_index).start, 'k--', 'HandleVisibility', 'off');
        xline(segments(segment_index).end, 'k--', 'HandleVisibility', 'off');
    end
else
    xline(report.cfg.fault_start, 'k--', 'HandleVisibility', 'off');
    xline(report.cfg.fault_end, 'k--', 'HandleVisibility', 'off');
end
end
