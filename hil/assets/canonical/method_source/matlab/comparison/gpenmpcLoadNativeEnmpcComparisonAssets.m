function assets = gpenmpcLoadNativeEnmpcComparisonAssets(projectRoot, outerPeriodS)
%GPENMPCLOADNATIVEENMPCCOMPARISONASSETS Load self-contained native assets.

arguments
    projectRoot (1,1) string
    outerPeriodS (1,1) double
end
assetRoot = fullfile(projectRoot, "matlab", "simulink", "assets");
runtimePath = fullfile(assetRoot, "enmpc_runtime_configuration.json");
aeroPath = fullfile(assetRoot, "aero_gp_runtime_configuration.json");
calibrationPath = fullfile(assetRoot, "M600_DYN_CALIBRATION.json");
profilePath = fullfile(assetRoot, "M600_PLATFORM_PROFILE.json");
modelPath = fullfile(projectRoot, "matlab", "native_training", ...
    "raw", "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");
required = [runtimePath, aeroPath, calibrationPath, profilePath, modelPath];
for path = required
    if ~isfile(path)
        error("gpenmpcLoadNativeEnmpcComparisonAssets:Missing", ...
            "Required native asset is missing: %s", path);
    end
end
assets = struct;
assets.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_COMPARISON_ASSETS_V1";
assets.project_root = projectRoot;
assets.outer_period_s = outerPeriodS;
assets.enmpc = gpenmpcLoadOrdinaryB2Config(runtimePath, outerPeriodS);
assets.aero = jsondecode(fileread(aeroPath));
assets.robust = assets.aero.robust_control;
assets.calibration = jsondecode(fileread(calibrationPath));
assets.profile = jsondecode(fileread(profilePath));
stored = load(modelPath, "nativeModel");
assets.gp_model = gpenmpcPrepareSparseGpModel(stored.nativeModel);
assets.runtime_configuration_path = string(runtimePath);
assets.aero_configuration_path = string(aeroPath);
assets.calibration_path = string(calibrationPath);
assets.profile_path = string(profilePath);
assets.gp_model_path = string(modelPath);
assets.runtime_configuration_sha256 = gpenmpcSha256File(runtimePath);
assets.aero_configuration_sha256 = gpenmpcSha256File(aeroPath);
assets.calibration_sha256 = gpenmpcSha256File(calibrationPath);
assets.profile_sha256 = gpenmpcSha256File(profilePath);
assets.gp_model_sha256 = gpenmpcSha256File(modelPath);
end
