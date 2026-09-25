function result = buildGPENMPCNativeInducingData(dataPath, outputPath, referenceMatPath)
%BUILDGPENMPCNATIVEINDUCINGDATA Select and validate 256 standardized F17 centres.
% An optional reference MAT supplies inducing_standardized_matlab or
% inducing_standardized_python for a separate numerical comparison.

arguments
    dataPath (1,1) string
    outputPath (1,1) string
    referenceMatPath (1,1) string = ""
end
if isfile(outputPath) || isfolder(outputPath)
    error("buildGPENMPCNativeInducingData:AppendOnly", "Output already exists: %s", outputPath);
end
data = load(dataPath, "X_train", "input_mean_matlab", "input_scale_matlab");
required = ["X_train", "input_mean_matlab", "input_scale_matlab"];
if ~all(isfield(data, required)) || ~isequal(size(data.X_train), [4680, 17]) ...
        || ~isequal(size(data.input_mean_matlab), [1, 17]) ...
        || ~isequal(size(data.input_scale_matlab), [1, 17]) ...
        || any(~isfinite(data.X_train), "all") ...
        || any(~isfinite(data.input_mean_matlab)) ...
        || any(~isfinite(data.input_scale_matlab) | data.input_scale_matlab <= 0)
    error("buildGPENMPCNativeInducingData:Input", "Finite 4680-by-17 training inputs and row normalization are required.");
end
computedScale = std(data.X_train, 0, 1);
computedScale(computedScale <= 1.0e-10) = 1.0;
if max(abs(mean(data.X_train, 1) - data.input_mean_matlab)) > 1.0e-12 ...
        || max(abs(computedScale - data.input_scale_matlab)) > 1.0e-12
    error("buildGPENMPCNativeInducingData:Normalization", "Normalization does not match the training inputs.");
end
standardized = (data.X_train - data.input_mean_matlab) ./ data.input_scale_matlab;
[inducing_standardized_matlab, inducing_audit] = gpenmpcSelectInducingPoints(standardized, 256);
[repeated, repeatedAudit] = gpenmpcSelectInducingPoints(standardized, 256);
maximumDifference = max(abs(inducing_standardized_matlab - repeated), [], "all");
result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_INDUCING_DATA_V1";
result.status = "PASS";
result.data_sha256 = gpenmpcSha256File(dataPath);
result.seed = inducing_audit.seed;
result.inducing_count = 256;
result.input_dimension = 17;
result.repeated_selection_max_absolute_difference = maximumDifference;
result.repeated_initial_indices_equal = isequal( ...
    inducing_audit.initial_selected_rows_one_based, repeatedAudit.initial_selected_rows_one_based);
result.validation_pass = isequal(size(inducing_standardized_matlab), [256, 17]) ...
    && all(isfinite(inducing_standardized_matlab), "all") ...
    && maximumDifference == 0 && result.repeated_initial_indices_equal;
result.parity = struct("reference_used", false, "status", "NOT_REQUESTED");
if ~result.validation_pass
    error("buildGPENMPCNativeInducingData:Validation", "Inducing selection is nonfinite or not repeatable.");
end
if strlength(referenceMatPath) > 0
    reference = load(referenceMatPath);
    if isfield(reference, "inducing_standardized_matlab")
        expected = reference.inducing_standardized_matlab;
    elseif isfield(reference, "inducing_standardized_python")
        expected = reference.inducing_standardized_python;
    else
        error("buildGPENMPCNativeInducingData:Reference", "Reference MAT has no standardized inducing matrix.");
    end
    if ~isequal(size(expected), [256, 17]) || any(~isfinite(expected), "all")
        error("buildGPENMPCNativeInducingData:Reference", "Reference inducing matrix must be finite and 256 by 17.");
    end
    difference = max(abs(inducing_standardized_matlab - expected), [], "all");
    result.parity = struct("reference_used", true, "status", "PASS", ...
        "reference_path", referenceMatPath, "maximum_absolute_difference", difference, ...
        "maximum_allowed_absolute_difference", 1.0e-10, "pass", difference <= 1.0e-10);
    if ~result.parity.pass
        error("buildGPENMPCNativeInducingData:Parity", ...
            "Inducing points differ from reference by %.17g.", difference);
    end
end
outputFolder = fileparts(outputPath);
if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
    mkdir(outputFolder);
end
save(outputPath, "inducing_standardized_matlab", "inducing_audit", "result", "-v7.3");
end
