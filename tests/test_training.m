function report = test_training(dataPath, outputRoot, referenceInducingPath)
%TEST_TRAINING Check F17 inducing selection and two-step sparse-GP training.
arguments
    dataPath (1,1) string
    outputRoot (1,1) string
    referenceInducingPath (1,1) string = ""
end
assert(~isfolder(outputRoot) && ~isfile(outputRoot), ...
    "test_training:OutputExists", "Use a new output directory.");
mkdir(outputRoot);
data = load(dataPath);
assert(~data.native_data_audit.parity.reference_used, ...
    "test_training:NoReferenceCase", "Provide data built without an optional reference.");
assert(~isfield(data.native_data_audit.parity, "pass"));
x = (data.X_train - data.input_mean_matlab) ./ data.input_scale_matlab;
reject(@() gpenmpcSelectInducingPoints(x(1:4648, :)), "gpenmpcSelectInducingPoints:Domain");
reject(@() gpenmpcSelectInducingPoints([x; x(1,:)]), "gpenmpcSelectInducingPoints:Domain");
reject(@() gpenmpcSelectInducingPoints(x(:,1:16)), "gpenmpcSelectInducingPoints:Domain");
reject(@() gpenmpcSelectInducingPoints(x, 4681), "gpenmpcSelectInducingPoints:Count");
[first, uniforms] = gpenmpcPcg64AeroKmeansStream(4);
assert(first == 4648 && numel(uniforms) == 4);

inducingPath = fullfile(outputRoot, "inducing.mat");
inducingResult = buildGPENMPCNativeInducingData(dataPath, inducingPath);
assert(inducingResult.validation_pass && ~inducingResult.parity.reference_used);
assert(~isfield(inducingResult.parity, "pass"));
inducingData = load(inducingPath);
assert(inducingData.inducing_audit.initial_selected_rows_one_based(1) == 4649);
reject(@() buildGPENMPCNativeInducingData(dataPath, inducingPath), ...
    "buildGPENMPCNativeInducingData:AppendOnly");
referenceResult = struct("reference_used", false);
if strlength(referenceInducingPath) > 0
    referenceResult = buildGPENMPCNativeInducingData(dataPath, ...
        fullfile(outputRoot, "inducing_reference.mat"), referenceInducingPath);
    assert(referenceResult.parity.pass);
end
inducing_standardized_matlab = inducingData.inducing_standardized_matlab;
inducing_standardized_matlab(1,1) = inducing_standardized_matlab(1,1) + 0.01;
badReference = fullfile(outputRoot, "bad_reference.mat");
save(badReference, "inducing_standardized_matlab");
reject(@() buildGPENMPCNativeInducingData(dataPath, ...
    fullfile(outputRoot, "rejected_inducing.mat"), badReference), ...
    "buildGPENMPCNativeInducingData:Parity");

invalid = data;
invalid.native_data_audit.parity = struct("reference_used", true);
invalidPath = fullfile(outputRoot, "missing_parity.mat");
save(invalidPath, "-struct", "invalid", "-v7.3");
reject(@() trainGPENMPCSparseGpNative(invalidPath, inducingPath, ...
    fullfile(outputRoot, "rejected_missing_parity"), OptimizationSteps=1), ...
    "trainGPENMPCSparseGpNative:DataAudit");
invalid.native_data_audit.parity.pass = false;
failedPath = fullfile(outputRoot, "failed_parity.mat");
save(failedPath, "-struct", "invalid", "-v7.3");
reject(@() trainGPENMPCSparseGpNative(failedPath, inducingPath, ...
    fullfile(outputRoot, "rejected_failed_parity"), OptimizationSteps=1), ...
    "trainGPENMPCSparseGpNative:DataAudit");
invalid = data;
invalid.native_data_audit.source_root = "DIFFERENT_DATA_FILE";
mismatchPath = fullfile(outputRoot, "different_data.mat");
save(mismatchPath, "-struct", "invalid", "-v7.3");
reject(@() trainGPENMPCSparseGpNative(mismatchPath, inducingPath, ...
    fullfile(outputRoot, "rejected_inducing_binding"), OptimizationSteps=1), ...
    "trainGPENMPCSparseGpNative:InducingAudit");

modelRoot = fullfile(outputRoot, "two_step_model");
result = trainGPENMPCSparseGpNative(dataPath, inducingPath, modelRoot, ...
    OptimizationSteps=2, Restarts=2);
saved = load(fullfile(modelRoot, "MATLAB_NATIVE_SPARSE_GP_MODEL.mat"));
model = saved.nativeModel;
y = (data.Y_train - data.output_mean_matlab) ./ data.output_scale_matlab;
recomputed = vfe(x, y, model.inducing_standardized, ...
    model.lengthscale, model.signal_std, model.noise_std_standardized);
tolerance = 1.0e-10 * max(1, abs(recomputed));
assert(abs(recomputed - result.selected_loss_per_sample_axis) <= tolerance);
assert(isequaln(saved.result, result));
restartLoss = zeros(result.restarts, 1);
for k = 1:result.restarts
    record = result.restart_records(k);
    restartLoss(k) = vfe(x, y, model.inducing_standardized, ...
        record.final_lengthscale, record.final_signal_std, record.final_noise_std_standardized);
    assert(abs(restartLoss(k) - record.final_loss_per_sample_axis) <= tolerance);
    assert(record.final_parameter_update_count == 2 ...
        && isequal(record.loss_trace_parameter_update_counts, [0;1]));
end
[minimum, selected] = min(restartLoss);
assert(result.selected_restart == selected - 1 && abs(minimum - recomputed) <= tolerance);
preUpdate = result.restart_records(selected).loss_trace(end);
assert(abs(preUpdate - recomputed) > 1.0e-8, ...
    "test_training:LossSensitivity", "The regression must distinguish pre-update and saved losses.");
reject(@() trainGPENMPCSparseGpNative(dataPath, inducingPath, modelRoot, OptimizationSteps=1), ...
    "trainGPENMPCSparseGpNative:AppendOnly");
gradientCheck = testGPENMPCSparseVfeAutodiff(fullfile(outputRoot, "gradient_check.json"));
report = struct("passed", true, "training_rows", size(x,1), ...
    "inducing_count", size(model.inducing_standardized,1), ...
    "optimization_steps", result.optimization_steps, "restarts", result.restarts, ...
    "pcg64_first_row_one_based", first + 1, ...
    "inducing_validation", inducingResult, "reference_inducing_validation", referenceResult, ...
    "recomputed_saved_loss", recomputed, "reported_saved_loss", result.selected_loss_per_sample_axis, ...
    "absolute_loss_difference", abs(recomputed - result.selected_loss_per_sample_axis), ...
    "last_pre_update_loss", preUpdate, "restart_recomputed_losses", restartLoss, ...
    "gradient_check", gradientCheck);
save(fullfile(outputRoot, "training_test.mat"), "report");
fid = fopen(fullfile(outputRoot, "training_test.json"), "w", "n", "UTF-8");
assert(fid >= 0); cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(report, PrettyPrint=true), "char");
end

function reject(call, identifier)
try
    call();
catch exception
    assert(string(exception.identifier) == identifier, ...
        "test_training:WrongError", "Expected %s; received %s.", identifier, exception.identifier);
    return
end
error("test_training:ExpectedError", "Expected %s.", identifier);
end

function loss = vfe(x, y, inducing, lengthscale, signalStd, noiseStd)
% Recompute the scalar collapsed objective without training gradients.
m = size(inducing,1); n = size(x,1);
kmm = gpenmpcArdRbfKernel(inducing, inducing, lengthscale, signalStd) + 1.0e-6 * eye(m);
kmn = gpenmpcArdRbfKernel(inducing, x, lengthscale, signalStd);
phi = chol(0.5 * (kmm+kmm.'), "lower") \ kmn;
gap = max(n * signalStd^2 - sum(phi.^2, "all"), 0);
loss = 0;
for axis = 1:3
    scaled = phi ./ noiseStd(axis);
    system = eye(m) + scaled * scaled.';
    factor = chol(0.5 * (system+system.') + 1.0e-9 * eye(m), "lower");
    projected = factor \ (scaled * y(:,axis));
    variance = noiseStd(axis)^2;
    loss = loss + 0.5 * (n * log(2*pi) + n * log(variance) ...
        + 2 * sum(log(diag(factor))) ...
        + (sum(y(:,axis).^2) - sum(projected.^2) + gap) / variance);
end
loss = loss / (3*n);
end
