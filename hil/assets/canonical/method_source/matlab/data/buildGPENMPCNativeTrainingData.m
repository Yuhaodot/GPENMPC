function result = buildGPENMPCNativeTrainingData(sourceRoot, outputPath, referenceMatPath)
%BUILDGPENMPCNATIVETRAININGDATA Build F17 train/cal matrices from raw NPZ logs.
%
% Training and calibration partitions use causal features. Row selection
% uses features and validity masks; labels are copied afterward. The
% optional reference MAT checks reconstructed arrays and selection indices.

arguments
    sourceRoot (1,1) string
    outputPath (1,1) string
    referenceMatPath (1,1) string = ""
end
if isfile(outputPath)
    error("buildGPENMPCNativeTrainingData:AppendOnly", "Refusing to overwrite %s", outputPath);
end
outputFolder = fileparts(outputPath);
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end

trainIds = compose("SLRGP_TR_%03d", 1:12);
calibrationIds = compose("SLRGP_CA_%03d", 1:6);
featureNames = {};
[X_train, Y_train, train_mission_index, train_selected_trace_index_one_based, ...
    train_audits] = derivePartition(sourceRoot, "training", trainIds);
[X_calibration, Y_calibration, calibration_mission_index, ...
    calibration_selected_trace_index_one_based, calibration_audits] = ...
    derivePartition(sourceRoot, "calibration", calibrationIds);
featureNames = train_audits(1).feature_names;

if ~isequal(size(X_train), [4680, 17]) || ~isequal(size(Y_train), [4680, 3])
    error("buildGPENMPCNativeTrainingData:Denominator", "Training denominator drift.");
end
if ~isequal(size(X_calibration), [2340, 17]) ...
        || ~isequal(size(Y_calibration), [2340, 3])
    error("buildGPENMPCNativeTrainingData:Denominator", "Calibration denominator drift.");
end

train_mission_ids = cellstr(trainIds);
calibration_mission_ids = cellstr(calibrationIds);
input_mean_matlab = mean(X_train, 1);
input_scale_matlab = std(X_train, 0, 1);
input_scale_matlab(input_scale_matlab <= 1.0e-10) = 1.0;
output_mean_matlab = mean(Y_train, 1);
output_scale_matlab = std(Y_train, 0, 1);
output_scale_matlab(output_scale_matlab <= 1.0e-12) = 1.0;

parity = struct("reference_used", false);
if strlength(referenceMatPath) > 0
    if ~isfile(referenceMatPath)
        error("buildGPENMPCNativeTrainingData:Reference", "Reference MAT missing: %s", referenceMatPath);
    end
    reference = load(referenceMatPath);
    parity.reference_used = true;
    parity.reference_path = referenceMatPath;
    parity.max_abs_x_train = max(abs(X_train - reference.X_train), [], "all");
    parity.max_abs_y_train = max(abs(Y_train - reference.Y_train), [], "all");
    parity.max_abs_x_calibration = max(abs(X_calibration - reference.X_calibration), [], "all");
    parity.max_abs_y_calibration = max(abs(Y_calibration - reference.Y_calibration), [], "all");
    parity.train_selected_index_equal = isequal(int64(train_selected_trace_index_one_based(:).'), ...
        int64(reference.train_selected_trace_index_one_based(:).'));
    parity.calibration_selected_index_equal = isequal( ...
        int64(calibration_selected_trace_index_one_based(:).'), ...
        int64(reference.calibration_selected_trace_index_one_based(:).'));
    parity.maximum_allowed_absolute_difference = 1.0e-12;
    parity.pass = parity.max_abs_x_train <= 1.0e-12 ...
        && parity.max_abs_y_train <= 1.0e-12 ...
        && parity.max_abs_x_calibration <= 1.0e-12 ...
        && parity.max_abs_y_calibration <= 1.0e-12 ...
        && parity.train_selected_index_equal ...
        && parity.calibration_selected_index_equal;
    if ~parity.pass
        error("buildGPENMPCNativeTrainingData:Parity", ...
            "MATLAB-native raw-log reconstruction does not match the read-only reference.");
    end
end

native_data_audit = struct;
native_data_audit.schema = "GPENMPC_MATLAB_NATIVE_RAW_LOG_TO_F17_DATA_AUDIT_V1";
native_data_audit.status = "PASS_MATLAB_NATIVE_CAUSAL_DATA_RECONSTRUCTION";
native_data_audit.source_root = sourceRoot;
native_data_audit.training_missions = 12;
native_data_audit.calibration_missions = 6;
native_data_audit.training_rows = size(X_train, 1);
native_data_audit.calibration_rows = size(X_calibration, 1);
native_data_audit.input_dimension = 17;
native_data_audit.output_dimension = 3;
native_data_audit.label_formula = "(v_hat[k+1]-v_hat[k])/dt[k]-a_nominal_known[k]";
native_data_audit.task_level_split = true;
native_data_audit.plant_private_truth_used = false;
native_data_audit.future_window_used = false;
native_data_audit.python_or_pytorch_runtime_used = false;
native_data_audit.per_mission_selection_is_label_blind = true;
native_data_audit.parity = parity;
native_data_audit.train_audits = train_audits;
native_data_audit.calibration_audits = calibration_audits;
native_data_audit.development_confirmation_formal_opened = false;

save(outputPath, "X_train", "Y_train", "X_calibration", "Y_calibration", ...
    "train_mission_ids", "calibration_mission_ids", "train_mission_index", ...
    "calibration_mission_index", "train_selected_trace_index_one_based", ...
    "calibration_selected_trace_index_one_based", "featureNames", ...
    "input_mean_matlab", "input_scale_matlab", "output_mean_matlab", ...
    "output_scale_matlab", "native_data_audit", "-v7.3");
result = native_data_audit;
result.output_path = outputPath;
end


function [features, labels, missionIndex, selectedIndices, audits] = ...
        derivePartition(sourceRoot, partition, missionIds)
features = zeros(390 * numel(missionIds), 17);
labels = zeros(390 * numel(missionIds), 3);
missionIndex = zeros(1, 390 * numel(missionIds), "int32");
selectedIndices = zeros(1, 390 * numel(missionIds), "int64");
audits = repmat(struct, numel(missionIds), 1);
cursor = 1;
for missionNumber = 1:numel(missionIds)
    missionId = missionIds(missionNumber);
    missionRoot = fullfile(sourceRoot, partition, missionId);
    tracePath = fullfile(missionRoot, "raw_trace.npz");
    summaryFiles = dir(fullfile(missionRoot, "MISSION_DATA_SUMMARY_*.json"));
    if ~isfile(tracePath) || numel(summaryFiles) ~= 1
        error("buildGPENMPCNativeTrainingData:Source", ...
            "Incomplete source identity for %s/%s", partition, missionId);
    end
    summary = jsondecode(fileread(fullfile(summaryFiles(1).folder, summaryFiles(1).name)));
    if string(summary.mission_id) ~= missionId || string(summary.partition) ~= partition
        error("buildGPENMPCNativeTrainingData:Identity", ...
            "Mission summary identity drift for %s", missionId);
    end
    [trace, npzAudit] = gpenmpcReadNpz(tracePath);
    extracted = gpenmpcExtractCausalAeroF17(trace);
    [selected, selectionAudit] = gpenmpcSelectMissionCoverageRows( ...
        extracted.features, extracted.valid_mask, 390);
    if numel(selected) ~= 390
        error("buildGPENMPCNativeTrainingData:Rows", ...
            "Mission %s selected %d rather than 390 rows", missionId, numel(selected));
    end
    destination = cursor:(cursor + 389);
    features(destination, :) = extracted.features(selected, :);
    labels(destination, :) = extracted.labels_f(selected, :);
    missionIndex(destination) = int32(missionNumber);
    selectedIndices(destination) = int64(selected);
    audits(missionNumber).mission_id = missionId;
    audits(missionNumber).partition = partition;
    audits(missionNumber).family = string(summary.family);
    audits(missionNumber).source_trace = string(tracePath);
    audits(missionNumber).source_summary = string(fullfile(summaryFiles(1).folder, summaryFiles(1).name));
    audits(missionNumber).raw_row_count = extracted.audit.row_count;
    audits(missionNumber).causal_valid_count = extracted.audit.causal_valid_count;
    audits(missionNumber).fit_valid_count = extracted.audit.fit_valid_count;
    audits(missionNumber).selected_count = numel(selected);
    audits(missionNumber).feature_names = extracted.feature_names;
    audits(missionNumber).npz_member_count = npzAudit.member_count;
    audits(missionNumber).causal_audit = extracted.audit;
    audits(missionNumber).selection_audit = selectionAudit;
    cursor = cursor + 390;
end
end
