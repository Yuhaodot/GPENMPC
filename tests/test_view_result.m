function report = test_view_result(options)
%TEST_VIEW_RESULT Check compensation fields, processing stages and missing data.
arguments
    options.OutputRoot (1,1) string = ""
    options.RunShortSimulation (1,1) logical = false
    options.ExistingSimulation (1,1) string = ""
end
root = string(fileparts(fileparts(mfilename("fullpath"))));
originalPath = path;
restorePath = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(root,"-begin");
trace = struct("global_time_s",(0:3).' .* 0.01);
total = [1 2 2; 0 3 4; 2 0 0; 1 0 0];
gp = [0.1 0 0; 0 0.2 0; 0 0 0.3; 0.4 0 0];
robust = [0 1 0; 0 0 2; 3 0 0; 0 4 0];
native = trace;
native.total_compensation_i_mps2 = total;
native.robust_compensation_i_mps2 = total .* 10;
native.augmentation_acceleration_i_mps2 = total .* 20;
native.gp_execution_feedforward_i_mps2 = gp;
native.gp_mean_f_mps2 = total .* 100;
native.trust_weight = ones(4,1);
native.residual_robust_compensation_i_mps2 = robust;
native.robust_acceleration_f_mps2 = robust .* 10;
checkFigure(native,"B2_ENMPC_GP_MEAN_TOTAL_TUBE", ...
    ["Total applied","GP target","Robust target"], ...
    {vecnorm(total,2,2),vecnorm(gp,2,2),vecnorm(robust,2,2)});

fixed = trace;
fixed.augmentation_acceleration_i_mps2 = total;
fixed.gp_mean_f_mps2 = repmat([3 4 0],4,1);
fixed.trust_weight = [0;0.25;0.5;1];
fixed.robust_acceleration_f_mps2 = robust;
checkFigure(fixed,"GR0_GP_ROBUST_SE3", ...
    ["Total applied","GP target","Robust target"], ...
    {vecnorm(total,2,2),[0;1.25;2.5;5],vecnorm(robust,2,2)});
checkFigure(fixed,"R0_ROBUST_SE3",["Total applied","Robust target"], ...
    {vecnorm(total,2,2),vecnorm(robust,2,2)});
checkFigure(native,"B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP", ...
    ["Total applied","Robust target"],{vecnorm(total,2,2),vecnorm(robust,2,2)});
checkFigure(fixed,"N0_NOMINAL_SE3",["Total applied","Robust target"], ...
    {vecnorm(total,2,2),vecnorm(robust,2,2)});

missingTrust = rmfield(fixed,"trust_weight");
checkFigure(missingTrust,"GR0_GP_ROBUST_SE3", ...
    ["Total applied","Robust target"],{vecnorm(total,2,2),vecnorm(robust,2,2)});
missingMean = rmfield(fixed,"gp_mean_f_mps2");
checkFigure(missingMean,"GR0_GP_ROBUST_SE3", ...
    ["Total applied","Robust target"],{vecnorm(total,2,2),vecnorm(robust,2,2)});
robustOnly = trace;
robustOnly.robust_acceleration_f_mps2 = robust;
checkFigure(robustOnly,"GR0_GP_ROBUST_SE3","Robust target",{vecnorm(robust,2,2)});
checkFigure(trace,"GR0_GP_ROBUST_SE3",strings(0,1),{});

legacy = trace;
legacy.robust_compensation_i_mps2 = total;
checkFigure(legacy,"B2_ENMPC_GP_MEAN_TOTAL_TUBE","Total applied",{vecnorm(total,2,2)});
invalid = fixed;
invalid.gp_mean_f_mps2 = ones(3,3);
invalid.robust_acceleration_f_mps2 = "unavailable";
checkFigure(invalid,"GR0_GP_ROBUST_SE3","Total applied",{vecnorm(total,2,2)});
separateLegs = fixed;
separateLegs.leg_index = [1;1;2;2];
separateLegs.augmentation_acceleration_i_mps2(4,:) = NaN;
checkFigure(separateLegs,"GR0_GP_ROBUST_SE3", ...
    ["Total applied","GP target","Robust target"], ...
    {[3;5;NaN;2;NaN],[0;1.25;NaN;2.5;5],[1;2;NaN;3;4]});

report = struct("passed",true,"fixture_cases",12,"runs",struct([]));
if options.RunShortSimulation
    assert(strlength(options.OutputRoot)>0,"test_view_result:Output", ...
        "Set OutputRoot for the short simulation and exports.");
    [~,~,folder] = run_case("cambridge_01","energy",Method="gp_robust_se3", ...
        MaximumInnerSamples=80,OutputRoot=fullfile(options.OutputRoot,"short_run"));
    report.runs = renderSimulation(fullfile(folder,"simulation.mat"), ...
        fullfile(options.OutputRoot,"fixed_reference"));
end
if strlength(options.ExistingSimulation)>0
    entry = renderSimulation(options.ExistingSimulation, ...
        fullfile(options.OutputRoot,"existing_run"));
    report.runs = [report.runs;entry];
end
disp(report);
end

function checkFigure(trace,method,names,expected)
fig = view_result(trace,Result=struct("runtime_method_id",method),Visible="off");
closeFigure = onCleanup(@() close(fig)); %#ok<NASGU>
ax = findobj(fig,"Type","axes","Tag","control_compensation");
lines = findobj(ax,"Type","line");
assert(numel(lines)==numel(names),"test_view_result:CurveCount", ...
    "Unexpected compensation curve count for %s.",method);
for index = 1:numel(names)
    line = findobj(ax,"Type","line","DisplayName",names(index));
    assert(isscalar(line),"test_view_result:Label","Missing curve: %s",names(index));
    actual = line.YData(:);
    expectedValues = expected{index}(:);
    assert(isequal(isnan(actual),isnan(expectedValues)));
    finite = isfinite(expectedValues);
    assert(all(abs(actual(finite)-expectedValues(finite))<1e-12), ...
        "test_view_result:Values","Incorrect values for %s.",names(index));
end
if isempty(names)
    messages = findobj(ax,"Type","text","String","Compensation data not available");
    assert(~isempty(messages));
end
end

function entry = renderSimulation(source,folder)
stored = load(source,"trace","result");
trace = stored.trace;
fig = view_result(source,Visible="off",OutputDirectory=folder, ...
    ExportFormats=["fig","png","pdf"]);
closeFigure = onCleanup(@() close(fig)); %#ok<NASGU>
ax = findobj(fig,"Type","axes","Tag","control_compensation");
totalLine = findobj(ax,"Type","line","DisplayName","Total applied");
if isfield(trace,"total_compensation_i_mps2")
    total = trace.total_compensation_i_mps2;
else
    total = trace.augmentation_acceleration_i_mps2;
end
assert(isscalar(totalLine));
totalValues = totalLine.YData(isfinite(totalLine.XData));
assert(max(abs(totalValues(:)-vecnorm(total,2,2)))<1e-12);
gpLine = findobj(ax,"Type","line","DisplayName","GP target");
if isfield(trace,"gp_execution_feedforward_i_mps2")
    gp = trace.gp_execution_feedforward_i_mps2;
else
    gp = -trace.trust_weight .* trace.gp_mean_f_mps2;
end
assert(isscalar(gpLine));
gpValues = gpLine.YData(isfinite(gpLine.XData));
assert(max(abs(gpValues(:)-vecnorm(gp,2,2)))<1e-12);
assert(all(isfile(fig.UserData.exported_files)));
entry = struct("source",string(source),"samples",numel(trace.global_time_s), ...
    "method",string(stored.result.runtime_method_id), ...
    "gp_target_nonzero_samples",sum(vecnorm(gp,2,2)>0), ...
    "panel_source",fig.UserData.panel_sources.e, ...
    "exports",fig.UserData.exported_files);
native = entry.exports(endsWith(entry.exports,".fig"));
reopened = openfig(native,"invisible");
assert(isequal(reopened.UserData.panel_sources,fig.UserData.panel_sources));
close(reopened);
end
