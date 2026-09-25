function result = runPythonMatlabInferenceParity(projectRoot)
%RUNPYTHONMATLABINFERENCEPARITY Compare MATLAB inference with frozen Python values.

arguments
    projectRoot (1,1) string
end

inputPath = fullfile(projectRoot, "matlab", "inference", "GPENMPC_SPARSE_GP_INFERENCE_PARITY_INPUT.mat");
loaded = load(inputPath);
prediction = gpenmpcSparseGpPredict(loaded, loaded.parity_inputs);

errors = struct;
errors.mean_absolute_mps2 = max(abs(prediction.mean_mps2 - loaded.python_mean_mps2), [], "all");
errors.raw_std_absolute_mps2 = max(abs(prediction.raw_std_mps2 - loaded.python_raw_std_mps2), [], "all");
errors.half_width_absolute_mps2 = max(abs(prediction.calibrated_half_width_mps2 - loaded.python_calibrated_half_width_mps2), [], "all");
errors.variance_absolute_mps4 = max(abs(prediction.latent_variance_standardized - loaded.python_latent_variance_standardized), [], "all");
errors.support_distance_absolute = max(abs(prediction.support_distance - loaded.python_support_distance(:)), [], "all");
errors.trust_absolute = max(abs(prediction.trust - loaded.python_trust(:)), [], "all");
errors.control_contribution_absolute_mps2 = max(abs(prediction.applied_mean_f_mps2 - loaded.python_applied_mean_f_mps2), [], "all");
errors.hard_invalid_mismatch = nnz(prediction.hard_invalid ~= logical(loaded.python_hard_invalid(:)));

tolerances = struct;
tolerances.mean_absolute_mps2 = 1.0e-9;
tolerances.variance_absolute_mps4 = 1.0e-8;
tolerances.raw_std_absolute_mps2 = 1.0e-9;
tolerances.half_width_absolute_mps2 = 1.0e-9;
tolerances.support_distance_absolute = 1.0e-9;
tolerances.trust_absolute = 1.0e-9;
tolerances.control_contribution_absolute_mps2 = 1.0e-9;

checks = struct;
checks.mean = errors.mean_absolute_mps2 <= tolerances.mean_absolute_mps2;
checks.variance = errors.variance_absolute_mps4 <= tolerances.variance_absolute_mps4;
checks.raw_std = errors.raw_std_absolute_mps2 <= tolerances.raw_std_absolute_mps2;
checks.half_width = errors.half_width_absolute_mps2 <= tolerances.half_width_absolute_mps2;
checks.support_distance = errors.support_distance_absolute <= tolerances.support_distance_absolute;
checks.trust = errors.trust_absolute <= tolerances.trust_absolute;
checks.control_contribution = errors.control_contribution_absolute_mps2 <= tolerances.control_contribution_absolute_mps2;
checks.hard_invalid = errors.hard_invalid_mismatch == 0;

result = struct;
result.schema = "GPENMPC_PYTHON_MATLAB_GP_INFERENCE_PARITY_V1";
checkValues = structfun(@(value) logical(value), checks);
allChecks = all(checkValues);
if allChecks
    result.status = "PASS_NUMERIC_EQUIVALENCE";
else
    result.status = "FAIL_NUMERIC_EQUIVALENCE";
end
result.sample_count = size(loaded.parity_inputs, 1);
result.feature_count = size(loaded.parity_inputs, 2);
result.output_count = 3;
result.errors = errors;
result.tolerances = tolerances;
result.checks = checks;

outputPath = fullfile(projectRoot, "matlab", "inference", "MATLAB_GP_INFERENCE_PARITY_RESULT.json");
fileId = fopen(outputPath, "w", "n", "UTF-8");
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s\n", jsonencode(result, PrettyPrint=true));
clear cleanup

if ~allChecks
    error("runPythonMatlabInferenceParity:Mismatch", "Python and MATLAB GP inference differ beyond the frozen tolerances.");
end
end
