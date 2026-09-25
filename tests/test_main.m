function report = test_main(options)
%TEST_MAIN Exercise the numerical entry point and saved response figure.

arguments
    options.OutputRoot (1,1) string = ""
end

root = string(fileparts(fileparts(mfilename("fullpath"))));
originalPath = path;
restorePath = onCleanup(@() path(originalPath)); %#ok<NASGU>
addpath(root, "-begin");
[trace, result, folder, fig] = main(MaximumInnerSamples=40, ...
    OutputRoot=options.OutputRoot, Visible="off");
closeFigure = onCleanup(@() close(fig)); %#ok<NASGU>
assert(numel(trace.global_time_s) == 40, "test_main:Samples", ...
    "Expected 40 simulation samples.");
assert(~result.task_complete, "test_main:Segment", ...
    "The selected segment ends before task completion.");
assert(all(isfinite(trace.position_m), "all"), "test_main:Position", ...
    "Expected finite position samples.");
assert(isfile(fullfile(folder, "simulation.mat")), "test_main:SavedRun", ...
    "Expected simulation.mat in the output folder.");
figures = dir(fullfile(folder, "figures", "*.png"));
assert(~isempty(figures) && all([figures.bytes] > 0), ...
    "test_main:Figure", "Expected a saved PNG response figure.");
report = struct("passed", true, "samples", numel(trace.global_time_s), ...
    "output_folder", folder, "figure_count", numel(figures));
disp(report);
end
