function figure_files = plot_li_style_matlab_figures(summary, output_directory)
%PLOT_LI_STYLE_MATLAB_FIGURES Generate MATLAB figures analogous to Li.
%   Outputs editable FIG files and PNG previews.  The figures are intended
%   for experiment review; publication styling can be performed separately.

if nargin < 2 || isempty(output_directory)
    output_directory = fullfile(stage1_project_root(), 'li_style_mc30_results');
end
if ~isfolder(output_directory)
    mkdir(output_directory);
end

colors = [0.20, 0.20, 0.20; ...
          0.20, 0.45, 0.75; ...
          0.85, 0.45, 0.12; ...
          0.15, 0.60, 0.35];
line_styles = {'--', '--', '-', '-'};
representative = summary.representative;

figure_files = strings(7, 2);
figure_files(1, :) = local_plot_trajectory(representative, output_directory);
figure_files(2, :) = local_plot_range_errors(representative, output_directory);
figure_files(3, :) = local_plot_position_errors( ...
    representative, colors, line_styles, output_directory);
figure_files(4, :) = local_plot_mc_mean_position_errors( ...
    summary, colors, line_styles, output_directory);
figure_files(5, :) = local_plot_cdf( ...
    summary, colors, line_styles, output_directory);
figure_files(6, :) = local_plot_mc_mean_variance_comparison( ...
    summary, colors, output_directory);
figure_files(7, :) = local_plot_rmse_boxchart(summary, colors, output_directory);
end

function files = local_plot_trajectory(representative, output_directory)
figure_handle = figure('Color', 'w', 'Name', 'Li-style simulated trajectory', ...
    'NumberTitle', 'off', 'Position', [100, 100, 900, 650]);
axes_handle = axes(figure_handle);
local_style_axes(axes_handle);
hold(axes_handle, 'on');
grid(axes_handle, 'on');
axis(axes_handle, 'equal');
follower_colors = [0.00, 0.45, 0.74; 0.30, 0.75, 0.93; 0.10, 0.25, 0.55];
leader_colors = [0.85, 0.33, 0.10; 0.93, 0.69, 0.13; 0.49, 0.18, 0.56];
plot_stride = max(1, round(1 / (representative.time(2) - representative.time(1))));
indices = 1:plot_stride:numel(representative.time);

for follower = 1:size(representative.truth_xyz, 3)
    trajectory = representative.truth_xyz(indices, :, follower);
    plot(axes_handle, trajectory(:, 1), trajectory(:, 2), '-', ...
        'Color', follower_colors(follower, :), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Follower%d', follower));
    plot(axes_handle, trajectory(1, 1), trajectory(1, 2), 'o', ...
        'Color', follower_colors(follower, :), ...
        'MarkerFaceColor', follower_colors(follower, :), ...
        'HandleVisibility', 'off');
end
for leader = 1:size(representative.leader_truth_xyz, 3)
    trajectory = representative.leader_truth_xyz(indices, :, leader);
    plot(axes_handle, trajectory(:, 1), trajectory(:, 2), '--', ...
        'Color', leader_colors(leader, :), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Leader%d', leader));
    plot(axes_handle, trajectory(1, 1), trajectory(1, 2), 's', ...
        'Color', leader_colors(leader, :), ...
        'MarkerFaceColor', leader_colors(leader, :), ...
        'HandleVisibility', 'off');
end
xlabel(axes_handle, 'East position / m');
ylabel(axes_handle, 'North position / m');
title_handle = title(axes_handle, 'Simulated cooperative trajectory');
title_handle.Color = [0.1, 0.1, 0.1];
legend_handle = legend(axes_handle, 'Location', 'bestoutside', 'NumColumns', 2);
local_style_legend(legend_handle);
files = local_export_figure(figure_handle, output_directory, ...
    'Fig4_simulated_trajectory');
end

function files = local_plot_range_errors(representative, output_directory)
figure_handle = figure('Color', 'w', 'Name', 'Li-style range errors', ...
    'NumberTitle', 'off', 'Position', [100, 100, 1000, 560]);
axes_handle = axes(figure_handle);
local_style_axes(axes_handle);
hold(axes_handle, 'on');
grid(axes_handle, 'on');
low_num = size(representative.range_error, 2);
uav_num = size(representative.range_error, 3);
leader_edge_count = low_num * (uav_num - low_num);
edge_colors = lines(leader_edge_count);
range_handles = gobjects(leader_edge_count, 1);
edge_index = 0;
for follower = 1:low_num
    for leader_global = (low_num + 1):uav_num
        edge_index = edge_index + 1;
        values = squeeze(representative.range_error(:, follower, leader_global));
        range_handles(edge_index) = plot(axes_handle, representative.range_time, ...
            values, 'Color', edge_colors(edge_index, :), 'LineWidth', 0.8, ...
            'DisplayName', sprintf('F%d-L%d', follower, leader_global - low_num));
    end
end
xlabel(axes_handle, 'Time / s');
ylabel(axes_handle, 'Relative range observation error / m');
title_handle = title(axes_handle, 'Range observation errors');
title_handle.Color = [0.1, 0.1, 0.1];
legend_handle = legend(axes_handle, range_handles, ...
    'Location', 'northoutside', ...
    'NumColumns', 4);
local_style_legend(legend_handle);
files = local_export_figure(figure_handle, output_directory, ...
    'Fig5_range_observation_errors');
end

function files = local_plot_position_errors( ...
        representative, colors, line_styles, output_directory)
figure_handle = figure('Color', 'w', 'Name', 'Li-style position errors', ...
    'NumberTitle', 'off', 'Position', [100, 50, 1050, 820]);
layout = tiledlayout(figure_handle, 3, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
plot_stride = max(1, round(0.2 / ...
    (representative.time(2) - representative.time(1))));
indices = 1:plot_stride:numel(representative.time);
for follower = 1:size(representative.error_3d, 3)
    axes_handle = nexttile(layout);
    local_style_axes(axes_handle);
    hold(axes_handle, 'on');
    grid(axes_handle, 'on');
    for method_index = 1:numel(representative.method_labels)
        values = squeeze(double( ...
            representative.error_3d(indices, method_index, follower)));
        plot(axes_handle, representative.time(indices), values, ...
            'Color', colors(method_index, :), ...
            'LineStyle', line_styles{method_index}, 'LineWidth', 1.1, ...
            'DisplayName', representative.method_labels{method_index});
    end
    ylabel(axes_handle, sprintf('Follower%d error / m', follower));
    if follower == 1
        legend_handle = legend(axes_handle, 'Location', 'northoutside', ...
            'NumColumns', 4);
        local_style_legend(legend_handle);
    end
    if follower == size(representative.error_3d, 3)
        xlabel(axes_handle, 'Time / s');
    end
end
title_handle = title(layout, ...
    'Three-dimensional positioning error (representative trial)');
title_handle.Color = [0.1, 0.1, 0.1];
files = local_export_figure(figure_handle, output_directory, ...
    'Supplementary_representative_positioning_error_time_series');
end

function files = local_plot_mc_mean_position_errors( ...
        summary, colors, line_styles, output_directory)
figure_handle = figure('Color', 'w', ...
    'Name', 'Monte Carlo mean positioning errors', ...
    'NumberTitle', 'off', 'Position', [100, 50, 1050, 820]);
layout = tiledlayout(figure_handle, 3, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
time = summary.representative.time(:);
plot_stride = max(1, round(0.2 / (time(2) - time(1))));
indices = 1:plot_stride:numel(time);
mean_error_3d = summary.mc_timewise_mean_error_3d;
global_max = max(double(mean_error_3d(:)));
y_upper = max(1, 1.10 * global_max);
for follower = 1:size(mean_error_3d, 3)
    axes_handle = nexttile(layout);
    local_style_axes(axes_handle);
    hold(axes_handle, 'on');
    grid(axes_handle, 'on');
    local_shade_fault_intervals(axes_handle, summary.cfg.fault_segments);
    for method_index = 1:numel(summary.method_labels)
        values = squeeze(double( ...
            mean_error_3d(indices, method_index, follower)));
        plot(axes_handle, time(indices), values, ...
            'Color', colors(method_index, :), ...
            'LineStyle', line_styles{method_index}, 'LineWidth', 1.2, ...
            'DisplayName', summary.method_labels{method_index});
    end
    ylim(axes_handle, [0, y_upper]);
    ylabel(axes_handle, sprintf('Follower%d mean error / m', follower));
    if follower == 1
        legend_handle = legend(axes_handle, 'Location', 'northoutside', ...
            'NumColumns', 4);
        local_style_legend(legend_handle);
    end
    if follower == size(mean_error_3d, 3)
        xlabel(axes_handle, 'Time / s');
    end
end
title_handle = title(layout, sprintf( ...
    'Mean three-dimensional positioning error (%d Monte Carlo trials)', ...
    numel(summary.seed_list)));
title_handle.Color = [0.1, 0.1, 0.1];
files = local_export_figure(figure_handle, output_directory, ...
    'Fig6_MC50_mean_positioning_error');
end

function files = local_plot_cdf(summary, colors, line_styles, output_directory)
figure_handle = figure('Color', 'w', 'Name', 'Li-style positioning error CDF', ...
    'NumberTitle', 'off', 'Position', [80, 120, 1250, 420]);
layout = tiledlayout(figure_handle, 1, 3, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
low_num = size(summary.cdf_error_m, 3);
for follower = 1:low_num
    axes_handle = nexttile(layout);
    local_style_axes(axes_handle);
    hold(axes_handle, 'on');
    grid(axes_handle, 'on');
    for method_index = 1:numel(summary.method_labels)
        plot(axes_handle, squeeze( ...
            summary.cdf_error_m(:, method_index, follower)), ...
            summary.cdf_probability, ...
            'Color', colors(method_index, :), ...
            'LineStyle', line_styles{method_index}, 'LineWidth', 1.4, ...
            'DisplayName', summary.method_labels{method_index});
    end
    xlabel(axes_handle, '3D positioning error / m');
    ylabel(axes_handle, 'CDF');
    follower_title = title(axes_handle, sprintf('Follower%d', follower));
    follower_title.Color = [0.1, 0.1, 0.1];
    ylim(axes_handle, [0, 1]);
    if follower == 1
        legend_handle = legend(axes_handle, 'Location', 'southeast');
        local_style_legend(legend_handle);
    end
end
title_handle = title(layout, sprintf( ...
    'Positioning-error CDF (%d Monte Carlo trials)', ...
    numel(summary.seed_list)));
title_handle.Color = [0.1, 0.1, 0.1];
files = local_export_figure(figure_handle, output_directory, ...
    'Fig7_positioning_error_CDF');
end

function files = local_plot_mc_mean_variance_comparison( ...
        summary, colors, output_directory)
statistics = summary.mc_error_statistics_table;
method_count = height(statistics);
figure_handle = figure('Color', 'w', ...
    'Name', 'Monte Carlo mean and variance comparison', ...
    'NumberTitle', 'off', 'Position', [100, 100, 1050, 460]);
layout = tiledlayout(figure_handle, 1, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

axes_handle = nexttile(layout);
local_style_axes(axes_handle);
hold(axes_handle, 'on');
grid(axes_handle, 'on');
mean_bar = bar(axes_handle, 1:method_count, statistics.Mean3DError_m, ...
    0.66, 'FaceColor', 'flat');
mean_bar.CData = colors;
local_label_bar_values(axes_handle, statistics.Mean3DError_m);
xticks(axes_handle, 1:method_count);
xticklabels(axes_handle, statistics.Method);
xtickangle(axes_handle, 18);
ylabel(axes_handle, 'Mean 3D positioning error / m');
title_handle = title(axes_handle, '(a) Mean error');
title_handle.Color = [0.1, 0.1, 0.1];

axes_handle = nexttile(layout);
local_style_axes(axes_handle);
hold(axes_handle, 'on');
grid(axes_handle, 'on');
variance_bar = bar(axes_handle, 1:method_count, ...
    statistics.Variance3DError_m2, 0.66, 'FaceColor', 'flat');
variance_bar.CData = colors;
local_label_bar_values(axes_handle, statistics.Variance3DError_m2);
xticks(axes_handle, 1:method_count);
xticklabels(axes_handle, statistics.Method);
xtickangle(axes_handle, 18);
ylabel(axes_handle, '3D positioning-error variance / m^2');
title_handle = title(axes_handle, '(b) Error variance');
title_handle.Color = [0.1, 0.1, 0.1];

title_handle = title(layout, sprintf( ...
    'Overall error statistics (%d Monte Carlo trials, three followers)', ...
    numel(summary.seed_list)));
title_handle.Color = [0.1, 0.1, 0.1];
files = local_export_figure(figure_handle, output_directory, ...
    'Fig8_MC50_mean_variance_comparison');
end

function files = local_plot_rmse_boxchart(summary, colors, output_directory)
figure_handle = figure('Color', 'w', 'Name', 'Monte Carlo RMSE boxchart', ...
    'NumberTitle', 'off', 'Position', [80, 120, 1250, 420]);
layout = tiledlayout(figure_handle, 1, 3, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
low_num = size(summary.per_seed_rmse_3d, 3);
method_count = numel(summary.method_labels);
for follower = 1:low_num
    axes_handle = nexttile(layout);
    local_style_axes(axes_handle);
    hold(axes_handle, 'on');
    for method_index = 1:method_count
        values = squeeze(summary.per_seed_rmse_3d(:, method_index, follower));
        boxchart(axes_handle, repmat(method_index, size(values)), values, ...
            'BoxFaceColor', colors(method_index, :), ...
            'MarkerColor', colors(method_index, :));
    end
    grid(axes_handle, 'on');
    xlim(axes_handle, [0.5, method_count + 0.5]);
    xticks(axes_handle, 1:method_count);
    xticklabels(axes_handle, summary.method_labels);
    xtickangle(axes_handle, 25);
    ylabel(axes_handle, '3D RMSE / m');
    follower_title = title(axes_handle, sprintf('Follower%d', follower));
    follower_title.Color = [0.1, 0.1, 0.1];
end
title_handle = title(layout, sprintf( ...
    'Per-seed RMSE distribution (%d trials)', numel(summary.seed_list)));
title_handle.Color = [0.1, 0.1, 0.1];
files = local_export_figure(figure_handle, output_directory, ...
    'Supplementary_MC_RMSE_boxchart');
end

function files = local_export_figure(figure_handle, output_directory, base_name)
png_path = fullfile(output_directory, base_name + ".png");
fig_path = fullfile(output_directory, base_name + ".fig");
exportgraphics(figure_handle, png_path, 'Resolution', 300, ...
    'BackgroundColor', 'white');
savefig(figure_handle, fig_path);
files = [string(png_path), string(fig_path)];
close(figure_handle);
end

function local_style_axes(axes_handle)
set(axes_handle, 'Color', 'w', ...
    'XColor', [0.1, 0.1, 0.1], ...
    'YColor', [0.1, 0.1, 0.1], ...
    'GridColor', [0.75, 0.75, 0.75], ...
    'GridAlpha', 0.45, ...
    'MinorGridColor', [0.85, 0.85, 0.85], ...
    'FontName', 'Arial', ...
    'FontSize', 10, ...
    'LineWidth', 0.8, ...
    'Box', 'on', ...
    'Layer', 'top');
axes_handle.Toolbar.Visible = 'off';
end

function local_style_legend(legend_handle)
set(legend_handle, 'Color', 'w', ...
    'TextColor', [0.1, 0.1, 0.1], ...
    'EdgeColor', [0.7, 0.7, 0.7], ...
    'FontName', 'Arial', ...
    'FontSize', 9);
end

function local_shade_fault_intervals(axes_handle, fault_segments)
for segment_index = 1:size(fault_segments, 1)
    region_handle = xregion(axes_handle, fault_segments(segment_index, 1), ...
        fault_segments(segment_index, 2), ...
        'FaceColor', [0.85, 0.25, 0.25], 'FaceAlpha', 0.055, ...
        'HandleVisibility', 'off');
    region_handle.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
end

function local_label_bar_values(axes_handle, values)
y_upper = max(values);
if y_upper <= 0
    y_upper = 1;
end
ylim(axes_handle, [0, 1.16 * y_upper]);
for index = 1:numel(values)
    text(axes_handle, index, values(index) + 0.035 * y_upper, ...
        sprintf('%.2f', values(index)), 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', 'FontName', 'Arial', ...
        'FontSize', 9, 'Color', [0.1, 0.1, 0.1]);
end
end
