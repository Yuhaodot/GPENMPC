function result = runPreparedPowerTableEquivalence(projectRoot, outputPath)
%RUNPREPAREDPOWERTABLEEQUIVALENCE Verify identical power interpolation.

arguments
    projectRoot (1,1) string
    outputPath (1,1) string = ""
end
addpath(genpath(fullfile(projectRoot, "matlab")));
profilePath = fullfile(projectRoot, "matlab", "simulink", "assets", ...
    "M600_PLATFORM_PROFILE.json");
profile = jsondecode(fileread(profilePath));
prepared = gpenmpcPrepareM600PhasePowerTable(profile);
payloads = [-0.2, 0.0, 1.1, 2.7, 4.54, 6.0];
airspeeds = [-0.1, 0.0, 0.19, 0.2, 0.7, 2.0, 6.5, 12.0, 18.0];
verticalSpeeds = [-1.0, -0.1, 0.0, 0.1, 1.0];
maximumDifference = 0.0;
statusMismatchCount = 0;
rowCount = 0;
for payload = payloads
    for airspeed = airspeeds
        for verticalSpeed = verticalSpeeds
            [legacyPower, legacyStatus] = gpenmpcM600PhasePower( ...
                profile, payload, airspeed, verticalSpeed);
            [preparedPower, preparedStatus] = gpenmpcM600PreparedPhasePower( ...
                prepared, payload, airspeed, verticalSpeed);
            maximumDifference = max(maximumDifference, ...
                abs(legacyPower - preparedPower));
            statusMismatchCount = statusMismatchCount ...
                + double(legacyStatus ~= preparedStatus);
            rowCount = rowCount + 1;
        end
    end
end
rng(20260902, "twister");
randomRowCount = 1000;
for index = 1:randomRowCount
    payload = -0.5 + 7.0 .* rand;
    airspeed = -0.5 + 20.0 .* rand;
    verticalSpeed = -1.5 + 3.0 .* rand;
    [legacyPower, legacyStatus] = gpenmpcM600PhasePower( ...
        profile, payload, airspeed, verticalSpeed);
    [preparedPower, preparedStatus] = gpenmpcM600PreparedPhasePower( ...
        prepared, payload, airspeed, verticalSpeed);
    maximumDifference = max(maximumDifference, ...
        abs(legacyPower - preparedPower));
    statusMismatchCount = statusMismatchCount ...
        + double(legacyStatus ~= preparedStatus);
    rowCount = rowCount + 1;
end
assert(maximumDifference <= 1.0e-12);
assert(statusMismatchCount == 0);

repeatCount = 400;
legacyStarted = tic;
for index = 1:repeatCount
    gpenmpcM600PhasePower(profile, 2.7, 6.5, 0.0);
end
legacyElapsedS = toc(legacyStarted);
preparedStarted = tic;
for index = 1:repeatCount
    gpenmpcM600PreparedPhasePower(prepared, 2.7, 6.5, 0.0);
end
preparedElapsedS = toc(preparedStarted);

result = struct;
result.schema = "GPENMPC_PREPARED_POWER_TABLE_EQUIVALENCE_V1";
result.status = "PASS";
result.test_rows = rowCount;
result.random_rows = randomRowCount;
result.maximum_power_difference_w = maximumDifference;
result.status_mismatch_count = statusMismatchCount;
result.timing_repeats = repeatCount;
result.legacy_elapsed_s = legacyElapsedS;
result.prepared_elapsed_s = preparedElapsedS;
result.elapsed_ratio_prepared_over_legacy = preparedElapsedS ./ legacyElapsedS;
if strlength(outputPath) > 0
    parent = fileparts(outputPath);
    if ~isfolder(parent)
        mkdir(parent);
    end
    writelines(jsonencode(result, PrettyPrint=true), outputPath, ...
        Encoding="UTF-8");
end
fprintf("%s\n", jsonencode(result));
end
