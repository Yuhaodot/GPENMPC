function result = runNativeDataParity(projectRoot)
%RUNNATIVEDATAPARITY End-to-end MATLAB-only raw-log reconstruction test.

arguments
    projectRoot (1,1) string
end
dataSource = gpenmpcExternalPath("training_data");
reference = gpenmpcExternalPath("training_data_reference");
outputFolder = fullfile(projectRoot, "matlab", "data", "native_training_data");
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end
outputPath = fullfile(outputFolder, "MATLAB_NATIVE_CAUSAL_F17_DATA_PROVISIONAL.mat");
result = buildGPENMPCNativeTrainingData(dataSource, outputPath, reference);
assert(result.parity.pass);
assert(result.training_rows == 4680);
assert(result.calibration_rows == 2340);
fprintf("MATLAB_NATIVE_DATA_PARITY_PASS Xtrain=%.3g Ytrain=%.3g Xcal=%.3g Ycal=%.3g\n", ...
    result.parity.max_abs_x_train, result.parity.max_abs_y_train, ...
    result.parity.max_abs_x_calibration, result.parity.max_abs_y_calibration);
end
