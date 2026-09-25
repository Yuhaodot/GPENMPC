function [trace, result, folder, fig] = main(taskId, planner, options)
%MAIN Run a delivery task and display its trajectory and control responses.
%   main runs Cambridge 01 with energy aware planning and a Gaussian process
%   (GP) enhanced eNMPC controller.
%   main(MaximumInnerSamples=300) runs a short initial segment.
%   main("seattle_01", "energy", Method="enmpc") selects another case.

arguments
    taskId (1,1) string = "cambridge_01"
    planner (1,1) string = "energy"
    options.Method (1,1) string = "gp_enmpc"
    options.MaximumInnerSamples (1,1) double = Inf
    options.OutputRoot (1,1) string = ""
    options.Visible (1,1) string {mustBeMember(options.Visible,["on","off"])} = "on"
    options.SaveFigure (1,1) logical = true
end

root = string(fileparts(mfilename("fullpath")));
originalPath = path;
restorePath = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(root, "-begin");
[trace, result, folder] = run_case(taskId, planner, ...
    Method=options.Method, MaximumInnerSamples=options.MaximumInnerSamples, ...
    OutputRoot=options.OutputRoot);

figureFolder = "";
if options.SaveFigure
    figureFolder = fullfile(folder, "figures");
end
fig = view_result(trace, Result=result, Visible=options.Visible, ...
    OutputDirectory=figureFolder, ExportFormats="png");
end
