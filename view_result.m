function fig = view_result(source, options)
%VIEW_RESULT Show the trajectory and control response from one simulation.
%   view_result("outputs/.../simulation.mat") opens a six-panel figure.
%   view_result(trace, Result=result) also accepts a trace struct directly.
%   Set OutputDirectory to save a figure; ExportFormats can include
%   "fig", "png" and "pdf". Curves use the recorded samples directly.
%
% Each panel describes one recorded run. Curves retain every source sample,
% with line breaks between separate flight legs.

arguments
    source
    options.Result (1,1) struct = struct
    options.Title (1,1) string = ""
    options.Visible (1,1) string {mustBeMember(options.Visible,["on","off"])} = "on"
    options.OutputDirectory (1,1) string = ""
    options.ExportFormats (1,:) string {mustBeMember(options.ExportFormats, ...
        ["fig","png","pdf"])} = "fig"
end

[trace, result, configuration, sourceFile] = readSource(source);
if ~isempty(fieldnames(options.Result))
    result = options.Result;
end
assert(isstruct(trace) && isscalar(trace), "view_result:Trace", ...
    "Expected a scalar trace struct.");
assert(isfield(trace,"global_time_s") && isnumeric(trace.global_time_s), ...
    "view_result:Time", "The trace must contain global_time_s.");
time = double(trace.global_time_s(:));
count = numel(time);

% Assemble the six-panel response figure from the recorded trace.
fig = figure("Name","Flight response","NumberTitle","off", ...
    "Color","white","Units","pixels","Position",[70 70 1400 820], ...
    "Visible",options.Visible);
layout = tiledlayout(fig,2,3,"TileSpacing","compact","Padding","compact");
heading = options.Title;
if strlength(heading) == 0
    heading = figureTitle(result);
end
title(layout,heading,"FontName","Arial","FontSize",16, ...
    "FontWeight","normal","Interpreter","none");
colors = [0.00 0.45 0.70; 0.90 0.62 0.00; 0.00 0.62 0.45; ...
    0.80 0.40 0.00; 0.80 0.47 0.65; 0.34 0.34 0.34];
styles = ["-","--",":","-.","-","--"];
axesHandles = gobjects(1,6);
for index = 1:6
    ax = nexttile(layout,index);
    axesHandles(index) = ax;
    hold(ax,"on");
    set(ax,"FontName","Arial","FontSize",11,"LineWidth",0.8, ...
        "Box","off","XColor",[0.20 0.20 0.20], ...
        "YColor",[0.20 0.20 0.20],"GridAlpha",0.12, ...
        "ColorOrder",colors,"TickDir","out");
    grid(ax,"on");
end

% a: Reference and simulated path in the same Cartesian coordinates.
ax = axesHandles(1);
panelTitle(ax,"a","Horizontal trajectory");
reference = fieldSeries(trace,"reference_position_m",count,3);
position = fieldSeries(trace,"position_m",count,3);
if ~isempty(reference)
    drawSeries(ax,reference(:,1),reference(:,2),trace, ...
        [0.40 0.40 0.40],"--","Reference");
end
if ~isempty(position)
    drawSeries(ax,position(:,1),position(:,2),trace,colors(1,:),"-","Aircraft");
end
xlabel(ax,"x (m)"); ylabel(ax,"y (m)"); axis(ax,"equal");
finishPanel(ax,"Trajectory not available");

% b: Use the logged norm, or calculate the same Euclidean position error.
ax = axesHandles(2);
panelTitle(ax,"b","Position tracking error");
positionError = fieldSeries(trace,"position_error_norm_m",count,1);
errorSource = "position_error_norm_m";
if isempty(positionError) && ~isempty(reference) && ~isempty(position)
    positionError = vecnorm(reference-position,2,2);
    errorSource = "norm(reference_position_m-position_m,2)";
end
if ~isempty(positionError)
    drawSeries(ax,time,positionError,trace,colors(1,:),"-","");
end
xlabel(ax,"Time (s)"); ylabel(ax,"Error norm (m)");
finishPanel(ax,"Position error not available");

% c: Actual stored six-rotor force commands, in newtons.
ax = axesHandles(3);
panelTitle(ax,"c","Rotor commands");
rotors = fieldSeries(trace,"rotor_command_n",count,6);
if ~isempty(rotors)
    for rotor = 1:6
        drawSeries(ax,time,rotors(:,rotor),trace,colors(rotor,:), ...
            styles(rotor),string(rotor));
    end
end
xlabel(ax,"Time (s)"); ylabel(ax,"Command (N)");
finishPanel(ax,"Rotor commands not available",3);

% d: Applied wind realization recorded by the simulator.
ax = axesHandles(4);
panelTitle(ax,"d","Wind disturbance");
wind = fieldSeries(trace,"actual_wind_xy_mps",count,2);
if ~isempty(wind)
    drawSeries(ax,time,wind(:,1),trace,colors(1,:),"-","x");
    drawSeries(ax,time,wind(:,2),trace,colors(2,:),"--","y");
    finiteWind = wind(isfinite(wind));
    if ~isempty(finiteWind)
        limits = [min(finiteWind),max(finiteWind)];
        padding = max(0.08*diff(limits),0.05);
        ylim(ax,limits+[-padding,padding]);
    end
end
xlabel(ax,"Time (s)"); ylabel(ax,"Wind velocity (m/s)");
finishPanel(ax,"Wind data not available");

% e: Applied total and pre-filter component targets.
ax = axesHandles(5);
ax.Tag = "control_compensation";
panelTitle(ax,"e","Control compensation");
compensationSources = strings(0,1);
[total,totalSource] = firstSeries(trace,["total_compensation_i_mps2", ...
    "augmentation_acceleration_i_mps2","robust_compensation_i_mps2"],count,3);
if ~isempty(total)
    drawSeries(ax,time,vecnorm(total,2,2),trace,colors(1,:),"-", ...
        "Total applied");
    compensationSources(end+1) = "Total applied: " + totalSource;
end
gp = fieldSeries(trace,"gp_execution_feedforward_i_mps2",count,3);
gpSource = "gp_execution_feedforward_i_mps2";
if isempty(gp)
    gpMean = fieldSeries(trace,"gp_mean_f_mps2",count,3);
    trust = fieldSeries(trace,"trust_weight",count,1);
    if ~isempty(gpMean) && ~isempty(trust)
        gp = -trust .* gpMean;
        gpSource = "-trust_weight .* gp_mean_f_mps2";
    end
end
methodName = methodIdentity(result);
knownMethod = strlength(methodName) > 0;
knownNoGp = any(startsWith(methodName,["B1_","R0_","N0_","C0_"])) || ...
    contains(lower(methodName),"no_gp");
showGp = ~knownMethod || (~knownNoGp && contains(lower(methodName),"gp")) || ...
    startsWith(methodName,"B2_");
if ~isempty(gp) && showGp
    drawSeries(ax,time,vecnorm(gp,2,2),trace,colors(2,:),"--","GP target");
    compensationSources(end+1) = "GP target: " + gpSource;
end
[robust,robustSource] = firstSeries(trace, ...
    ["residual_robust_compensation_i_mps2","robust_acceleration_f_mps2"],count,3);
if ~isempty(robust)
    drawSeries(ax,time,vecnorm(robust,2,2),trace,colors(3,:),":","Robust target");
    compensationSources(end+1) = "Robust target: " + robustSource;
end
xlabel(ax,"Time (s)"); ylabel(ax,"Acceleration norm (m/s^{2})");
finishPanel(ax,"Compensation data not available");

% f: eNMPC elapsed time, or geometric attitude error for fixed-reference runs.
ax = axesHandles(6);
solveTime = fieldSeries(trace,"solver_last_elapsed_s",count,1);
if ~isempty(solveTime)
    panelTitle(ax,"f","eNMPC solve time");
    drawSeries(ax,time,solveTime,trace,colors(1,:),"-","Latest solve");
    deadline = fieldSeries(trace,"solver_deadline_s",count,1);
    if isempty(deadline)
        deadlineScalar = scalarField(result,"solver_deadline_s");
        if isempty(deadlineScalar) && isfield(configuration,"enmpc")
            deadlineScalar = scalarField(configuration.enmpc,"solver_deadline_s");
        end
        if ~isempty(deadlineScalar) && deadlineScalar > 0
            yline(ax,deadlineScalar,"--","Color",[0.4 0.4 0.4], ...
                "LineWidth",1.1,"DisplayName","Deadline");
        end
    else
        drawSeries(ax,time,deadline,trace,[0.4 0.4 0.4],"--","Deadline");
    end
    ylabel(ax,"Elapsed time (s)");
    timingSource = "solver_last_elapsed_s; supplied solver_deadline_s";
else
    panelTitle(ax,"f","Geometric attitude error");
    attitudeError = fieldSeries(trace,"attitude_error_rad",count,3);
    if ~isempty(attitudeError)
        % The stored field contains the geometric e_R attitude-error vector.
        drawSeries(ax,time,vecnorm(attitudeError,2,2),trace, ...
            colors(1,:),"-","");
    end
    ylabel(ax,"Geometric error norm");
    timingSource = "norm(attitude_error_rad,2), dimensionless geometric e_R";
end
xlabel(ax,"Time (s)");
finishPanel(ax,"Response data not available");

for index = 2:6
    setTimeLimits(axesHandles(index),time);
end
fig.UserData = struct("source_file",sourceFile,"source_samples",count, ...
    "panel_sources",struct( ...
        "a","reference_position_m and position_m, columns 1:2", ...
        "b",errorSource,"c","rotor_command_n, columns 1:6", ...
        "d","actual_wind_xy_mps, columns 1:2", ...
        "e",strjoin(compensationSources,"; "), ...
        "f",timingSource), ...
    "processing","All samples retained; Euclidean norms; breaks at leg changes", ...
    "screen_size_pixels",[1400 820], ...
    "exported_files",strings(0,1));
drawnow;

if strlength(options.OutputDirectory) > 0
    folder = options.OutputDirectory;
    if ~isfolder(folder)
        [created,message] = mkdir(folder);
        assert(created,"view_result:OutputFolder","%s",message);
    end
    stem = "response_" + string(datetime("now","Format","yyyyMMdd_HHmmss_SSS"));
    base = fullfile(folder,stem);
    suffix = 1;
    while any(isfile(base + [".fig",".png",".pdf"]))
        base = fullfile(folder,stem + "_" + suffix);
        suffix = suffix + 1;
    end
    files = base + "." + unique(options.ExportFormats,"stable");
    data = fig.UserData;
    data.exported_files = files(:);
    fig.UserData = data;
    for format = unique(options.ExportFormats,"stable")
        outputFile = base + "." + format;
        switch format
            case "fig"
                savefig(fig,outputFile);
            case "png"
                exportgraphics(fig,outputFile,"Resolution",180,"BackgroundColor","white");
            case "pdf"
                exportgraphics(fig,outputFile,"ContentType","vector", ...
                    "BackgroundColor","white");
        end
    end
end
end

function [trace,result,configuration,sourceFile] = readSource(source)
result = struct;
configuration = struct;
sourceFile = "";
if isstruct(source)
    stored = source;
else
    assert((ischar(source) || isstring(source)) && isscalar(string(source)), ...
        "view_result:Source","Provide a MAT file or a trace struct.");
    sourceFile = string(source);
    if startsWith(sourceFile,"file:","IgnoreCase",true)
        sourceFile = string(char(java.io.File(java.net.URI(char(sourceFile))).getPath()));
    end
    assert(isfile(sourceFile),"view_result:Source","File not found: %s",sourceFile);
    stored = load(sourceFile);
end
if isfield(stored,"trace")
    trace = stored.trace;
    if isfield(stored,"result"), result = stored.result; end
    if isfield(stored,"configuration"), configuration = stored.configuration; end
else
    trace = stored;
end
for field = ["mission_id","method_id","planner_id"]
    if ~isfield(result,field) && isfield(trace,field)
        result.(field) = trace.(field);
    end
end
end

function value = fieldSeries(trace,name,count,columns)
value = [];
if ~isfield(trace,name), return; end
candidate = trace.(name);
if ~isnumeric(candidate) && ~islogical(candidate), return; end
if size(candidate,1) ~= count || size(candidate,2) < columns, return; end
value = double(candidate(:,1:columns));
end

function [value,source] = firstSeries(trace,names,count,columns)
value = [];
source = "";
for name = names
    value = fieldSeries(trace,name,count,columns);
    if ~isempty(value)
        source = name;
        return;
    end
end
end

function value = scalarField(data,name)
value = [];
if isstruct(data) && isfield(data,name)
    candidate = data.(name);
    if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
        value = double(candidate);
    end
end
end

function drawSeries(ax,x,y,trace,color,style,name)
if isempty(x) || isempty(y), return; end
x = double(x(:)); y = double(y(:));
if numel(x) ~= numel(y), return; end
[displayX,displayY] = breakAtLegChanges(x,y,trace);
handle = plot(ax,displayX,displayY,"Color",color,"LineStyle",style, ...
    "LineWidth",1.2,"DisplayName",name);
if numel(x) == 1
    set(handle,"Marker",".","MarkerSize",14);
end
end

function [x,y] = breakAtLegChanges(x,y,trace)
if ~isfield(trace,"leg_index") || numel(trace.leg_index) ~= numel(x)
    return;
end
leg = trace.leg_index(:);
boundaries = find(leg(2:end) ~= leg(1:end-1)) + 1;
if isempty(boundaries), return; end
offset = zeros(numel(x),1);
offset(boundaries) = 1;
indices = (1:numel(x)).' + cumsum(offset);
expandedX = nan(numel(x)+numel(boundaries),1);
expandedY = expandedX;
expandedX(indices) = x; expandedY(indices) = y;
x = expandedX; y = expandedY;
end

function panelTitle(ax,letter,name)
title(ax,letter + "   " + name,"FontSize",12,"FontWeight","normal", ...
    "HorizontalAlignment","left","Interpreter","none");
end

function finishPanel(ax,emptyMessage,columns)
if nargin < 3, columns = 2; end
lines = findobj(ax,"Type","line");
if isempty(lines)
    text(ax,0.5,0.5,emptyMessage,"Units","normalized", ...
        "HorizontalAlignment","center","FontName","Arial", ...
        "FontSize",11,"Color",[0.45 0.45 0.45],"Interpreter","none");
else
    named = arrayfun(@(h) strlength(string(h.DisplayName)) > 0,lines);
    if any(named)
        legend(ax,"show","Location","best","Box","off", ...
            "FontSize",9,"NumColumns",min(columns,sum(named)), ...
            "Interpreter","none");
    end
end
end

function setTimeLimits(ax,time)
finiteTime = time(isfinite(time));
if isempty(finiteTime), return; end
limits = [min(finiteTime),max(finiteTime)];
if limits(1) == limits(2)
    limits = limits + [-0.5 0.5];
end
xlim(ax,limits);
end

function name = methodIdentity(result)
name = "";
for field = ["runtime_method_id","method_id","method","public_method_name"]
    if isfield(result,field) && strlength(string(result.(field))) > 0
        name = string(result.(field));
        return;
    end
end
end

function heading = figureTitle(result)
identity = methodIdentity(result);
switch identity
    case "B2_ENMPC_GP_MEAN_TOTAL_TUBE"
        method = "Gaussian process enhanced eNMPC with robust tracking";
    case "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP"
        method = "eNMPC with robust tracking";
    case "R0_ROBUST_SE3"
        method = "Robust SE(3) tracking";
    case "N0_NOMINAL_SE3"
        method = "Geometric SE(3) tracking";
    case "GR0_GP_ROBUST_SE3"
        method = "Gaussian process enhanced robust SE(3) tracking";
    case "C0_CASCADED_PID"
        method = "Cascaded PID control";
    otherwise
        method = "Flight response";
end
city = "";
if isfield(result,"mission_id")
    mission = upper(string(result.mission_id));
    if contains(mission,"CAMBRIDGE"), city = "Cambridge";
    elseif contains(mission,"SEATTLE"), city = "Seattle";
    elseif contains(mission,"MANHATTAN"), city = "Manhattan";
    end
end
if strlength(city) > 0
    heading = city + " | " + method;
else
    heading = method;
end
if isfield(result,"run_mode") && string(result.run_mode) == "sample_limited"
    heading = heading + " | Short run";
end
end
