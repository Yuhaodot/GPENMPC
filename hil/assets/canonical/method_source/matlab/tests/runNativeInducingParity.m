function result = runNativeInducingParity(nativeDataPath, referenceMatPath, outputPath)
%RUNNATIVEINDUCINGPARITY Verify MATLAB-native inducing-point selection.

arguments
    nativeDataPath (1,1) string
    referenceMatPath (1,1) string
    outputPath (1,1) string
end
native = load(nativeDataPath);
reference = load(referenceMatPath, "inducing_standardized_python", "training_seed_python");
standardized = (native.X_train - native.input_mean_matlab) ./ native.input_scale_matlab;
[inducing_standardized_matlab, inducing_audit] = gpenmpcSelectInducingPoints(standardized, 256);
maximumAbsoluteDifference = max(abs(inducing_standardized_matlab ...
    - reference.inducing_standardized_python), [], "all");
result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_INDUCING_SELECTION_PARITY_RESULT_V1";
result.status = "PASS";
result.seed = uint32(1323034700);
result.reference_seed = reference.training_seed_python;
result.maximum_absolute_difference = maximumAbsoluteDifference;
result.maximum_allowed_absolute_difference = 1.0e-10;
result.exact_within_tolerance = maximumAbsoluteDifference <= 1.0e-10;
result.inducing_count = 256;
result.input_dimension = 17;
result.python_runtime_used = false;
result.audit = inducing_audit;
if ~result.exact_within_tolerance
    result.status = "FAIL_PARITY";
    error("runNativeInducingParity:Parity", ...
        "Native inducing points differ from reference by %.17g", maximumAbsoluteDifference);
end
if isfile(outputPath)
    error("runNativeInducingParity:AppendOnly", "Refusing to overwrite %s", outputPath);
end
save(outputPath, "inducing_standardized_matlab", "inducing_audit", "result", "-v7.3");
fprintf("MATLAB_NATIVE_INDUCING_PARITY_PASS max_abs=%.3g\n", maximumAbsoluteDifference);
end
