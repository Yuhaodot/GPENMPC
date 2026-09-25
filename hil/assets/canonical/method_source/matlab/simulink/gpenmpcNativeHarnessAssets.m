function assets = gpenmpcNativeHarnessAssets()
%GPENMPCNATIVEHARNESSASSETS Load the self-contained native Simulink assets.

persistent cachedAssets
if ~isempty(cachedAssets)
    assets = cachedAssets;
    return
end

simulinkRoot = fileparts(mfilename("fullpath"));
projectRoot = fileparts(fileparts(simulinkRoot));
assetRoot = fullfile(simulinkRoot, "assets");

assets = struct;
assets.project_root = string(projectRoot);
assets.sample_period_s = 0.01;
assets.outer_period_s = 0.30;
assets.runtime_configuration_path = fullfile(assetRoot, ...
    "enmpc_runtime_configuration.json");
assets.aero_configuration_path = fullfile(assetRoot, ...
    "aero_gp_runtime_configuration.json");
assets.calibration_path = fullfile(assetRoot, "M600_DYN_CALIBRATION.json");
assets.profile_path = fullfile(assetRoot, "M600_PLATFORM_PROFILE.json");
assets.gp_model_path = fullfile(projectRoot, "matlab", ...
    "native_training", "raw", "MATLAB_NATIVE_SPARSE_GP_MODEL.mat");

required = [assets.runtime_configuration_path, assets.aero_configuration_path, ...
    assets.calibration_path, assets.profile_path, assets.gp_model_path];
for path = required
    if ~isfile(path)
        error("gpenmpcNativeHarnessAssets:Missing", ...
            "Required native harness asset is missing: %s", path);
    end
end

assets.runtime = jsondecode(fileread(assets.runtime_configuration_path));
assets.aero = jsondecode(fileread(assets.aero_configuration_path));
assets.robust = assets.aero.robust_control;
assets.calibration = jsondecode(fileread(assets.calibration_path));
assets.profile = jsondecode(fileread(assets.profile_path));
gp = load(assets.gp_model_path, "nativeModel");
assets.gp_model = gp.nativeModel;
assets.mission = gpenmpcNativeHarnessMission();
assets.enmpc = gpenmpcLoadOrdinaryB2Config( ...
    assets.runtime_configuration_path, assets.outer_period_s);
cachedAssets = assets;
end
