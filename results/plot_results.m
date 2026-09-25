function figures = plot_results(outputDirectory, series)
%PLOT_RESULTS Plot the saved Seattle flight and controller study.
%   addpath("results"); plot_results
%   plot_results(outputDirectory) selects the PNG and PDF destination.
%   plot_results(outputDirectory, series) also plots flight traces.
arguments
    outputDirectory (1,1) string = ""
    series (1,:) struct = struct([])
end
here = string(fileparts(mfilename("fullpath")));
if strlength(outputDirectory) == 0
    outputDirectory = fullfile(fileparts(here), "outputs", "study_figures");
end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
study = readtable(fullfile(here, "case_metrics.csv"), "TextType", "string");
figures = struct;
blue = [0.00 0.38 0.62];
orange = [0.80 0.35 0.08];
grey = [0.35 0.35 0.35];

% Three-axis reference tracking and signed Cartesian errors.
if ~isempty(series)
figures.tracking = newFigure([1280 850]);
layout = tiledlayout(figures.tracking, 3, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Seattle delivery task", "FontSize", 18, "FontWeight", "normal");
subtitle(layout, "Energy aware plan | GP/eNMPC", "FontSize", 12);
s = series(1);
assert(string(s.method) == "GP/eNMPC");
axisNames = ["x", "y", "z"];
for dim = 1:3
    ax = nexttile(layout); style(ax);
    segmentedLine(ax, s.time_s, s.reference_position_m(:,dim), s.leg_index, grey, "--", "Reference", 1.6);
    segmentedLine(ax, s.time_s, s.position_m(:,dim), s.leg_index, blue, "-", "Aircraft", 1.1);
    title(ax, axisNames(dim) + " position", "FontWeight", "normal");
    ylabel(ax, axisNames(dim) + " (m)");
    xlim(ax, [0 s.time_s(end)]);
    if dim == 1, legend(ax, "Location", "best", "Box", "off"); end
    if dim == 3, xlabel(ax, "Mission time (s)"); else, ax.XTickLabel = []; end
    ax = nexttile(layout); style(ax);
    error = s.position_m(:,dim) - s.reference_position_m(:,dim);
    yline(ax, 0, ":", "Color", [0.65 0.65 0.65], "HandleVisibility", "off");
    segmentedLine(ax, s.time_s, error, s.leg_index, blue, "-", "Tracking error", 1.1);
    title(ax, axisNames(dim) + " tracking error", "FontWeight", "normal");
    ylabel(ax, "Error (m)");
    xlim(ax, [0 s.time_s(end)]);
    if dim == 3, xlabel(ax, "Mission time (s)"); else, ax.XTickLabel = []; end
end
saveFigure(figures.tracking, outputDirectory, "seattle-tracking");

% Integrate each recorded flight leg with the trapezoidal rule.
figures.energy = newFigure([1280 580]);
layout = tiledlayout(figures.energy, 1, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Power and flight energy", "FontSize", 18, "FontWeight", "normal");
subtitle(layout, "Seattle 01 | Energy aware plan", "FontSize", 12);
powerAxes = nexttile(layout); style(powerAxes);
energyAxes = nexttile(layout); style(energyAxes);
colors = [blue; orange];
styles = ["-", "--"];
lastTime = 0;
for k = 1:numel(series)
    s = series(k);
    energy = zeros(size(s.time_s));
    offset = 0;
    for leg = unique(s.leg_index, "stable").'
        rows = find(s.leg_index == leg);
        energy(rows) = offset + cumtrapz(s.time_s(rows), s.power_w(rows));
        offset = energy(rows(end));
    end
    assert(abs(energy(end) - s.flight_energy_j) < 1e-5, ...
        "plot_results:Energy", "Integrated flight energy differs from the saved metric.");
    segmentedLine(powerAxes, s.time_s, s.power_w / 1000, s.leg_index, colors(k,:), styles(k), s.method, 1.3);
    segmentedLine(energyAxes, s.time_s, energy / 1000, s.leg_index, colors(k,:), styles(k), s.method, 1.6);
    lastTime = max(lastTime, s.time_s(end));
end
title(powerAxes, "Modeled power", "FontWeight", "normal");
xlabel(powerAxes, "Mission time (s)"); ylabel(powerAxes, "Power (kW)");
title(energyAxes, "Cumulative flight energy", "FontWeight", "normal");
xlabel(energyAxes, "Mission time (s)"); ylabel(energyAxes, "Energy (kJ)");
for ax = [powerAxes, energyAxes]
    xlim(ax, [0 lastTime]);
    legend(ax, "Location", "northwest", "Box", "off");
end
saveFigure(figures.energy, outputDirectory, "seattle-energy");
end

% Each point is one task/planner pair; diamonds show the arithmetic mean.
methods = ["N0_NOMINAL_SE3", "C0_CASCADED_PID", "R0_ROBUST_SE3", ...
    "GR0_GP_ROBUST_SE3", "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP", "B2_ENMPC_GP_MEAN_TOTAL_TUBE"];
labels = ["Nominal SE(3)", "PID", "Robust SE(3)", "GP + robust SE(3)", "eNMPC", "GP/eNMPC"];
colors = [0.58 0.58 0.58; 0.43 0.43 0.43; 0.27 0.52 0.48; ...
    0.44 0.40 0.63; orange; blue];
figures.comparison = newFigure([1380 630]);
layout = tiledlayout(figures.comparison, 1, 2, "TileSpacing", "compact", "Padding", "compact");
title(layout, "Energy and tracking across the controller study", "FontSize", 18, "FontWeight", "normal");
subtitle(layout, "6 tasks x 2 planners | 12 cases per controller | Points: cases; diamonds: mean", "FontSize", 12);
energyAxes = nexttile(layout); style(energyAxes);
errorAxes = nexttile(layout); style(errorAxes);
baseline = study(study.stratum == "controller_comparison" & study.runtime_method_id == "C0_CASCADED_PID", :);
baseline = sortrows(baseline, ["mission_id", "planner_id"]);
xline(energyAxes, 0, ":", "Color", [0.65 0.65 0.65]);
for k = 1:numel(methods)
    rows = study(study.stratum == "controller_comparison" & study.runtime_method_id == methods(k), :);
    rows = sortrows(rows, ["mission_id", "planner_id"]);
    assert(height(rows) == 12 && height(unique(rows(:,["mission_id", "planner_id"]))) == 12);
    assert(isequal(rows(:,["mission_id", "planner_id"]), baseline(:,["mission_id", "planner_id"])));
    energy = 100 * (rows.service_corrected_modeled_energy_j ./ baseline.service_corrected_modeled_energy_j - 1);
    error = rows.position_rms_m;
    y = k + linspace(-0.17, 0.17, height(rows)).';
    for pair = 1:2
        if pair == 1, ax = energyAxes; values = energy; else, ax = errorAxes; values = error; end
        scatter(ax, values, y, 32, colors(k,:), "filled", "MarkerFaceAlpha", 0.55, ...
            "MarkerEdgeColor", "none");
        scatter(ax, mean(values), k, 74, colors(k,:), "d", "filled", ...
            "MarkerEdgeColor", [0.12 0.12 0.12], "LineWidth", 0.8);
    end
end
for ax = [energyAxes, errorAxes]
    ax.YTick = 1:numel(methods); ax.YTickLabel = labels; ax.YDir = "reverse";
    ylim(ax, [0.5 numel(methods)+0.5]); ax.YGrid = "off";
end
title(energyAxes, "Modeled mission energy relative to PID", "FontWeight", "normal"); xlabel(energyAxes, "Energy change (%)");
title(errorAxes, "Position tracking", "FontWeight", "normal"); xlabel(errorAxes, "Position RMSE (m)");
xlim(errorAxes, [0 max(0.8, max(study.position_rms_m(study.stratum == "controller_comparison"))*1.05)]);
saveFigure(figures.comparison, outputDirectory, "controller-comparison");
end

function fig = newFigure(sizePx)
fig = figure("Color", "white", "Units", "pixels", "Position", [40 40 sizePx], ...
    "Visible", "off", "NumberTitle", "off");
end

function style(ax)
hold(ax, "on");
set(ax, "FontName", "Arial", "FontSize", 11, "LineWidth", 0.8, ...
    "Box", "off", "TickDir", "out", "GridAlpha", 0.12);
grid(ax, "on");
end

function segmentedLine(ax, time, value, legs, color, lineStyle, label, width)
first = true;
for leg = unique(legs, "stable").'
    rows = legs == leg;
    handle = plot(ax, time(rows), value(rows), "Color", color, "LineStyle", lineStyle, ...
        "LineWidth", width, "DisplayName", label);
    if ~first, handle.HandleVisibility = "off"; end
    first = false;
end
end

function saveFigure(fig, folder, name)
drawnow;
exportgraphics(fig, fullfile(folder, name + ".png"), "Resolution", 160, "BackgroundColor", "white");
exportgraphics(fig, fullfile(folder, name + ".pdf"), "ContentType", "vector", "BackgroundColor", "white");
end
