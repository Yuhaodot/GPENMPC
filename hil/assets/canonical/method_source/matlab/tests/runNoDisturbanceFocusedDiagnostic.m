function receipt = runNoDisturbanceFocusedDiagnostic( ...
        projectRoot, outputRoot, maximumInnerSamples)
%RUNNODISTURBANCEFOCUSEDDIAGNOSTIC Compare B1/B2 responses on a zero-disturbance task prefix.

arguments
    projectRoot (1,1) string
    outputRoot (1,1) string
    maximumInnerSamples (1,1) double {mustBeInteger,mustBePositive} = 2000
end

addpath(genpath(fullfile(projectRoot, "matlab")));
multiCityRoot = ...
    gpenmpcExternalPath("comparison_fixed_reference_cases");
ordinaryB2Root = ...
    gpenmpcExternalPath("comparison_enmpc_cases");
units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root);
task = gpenmpcLoadNativeComparisonUnit(units(1), ...
    fullfile(projectRoot, "matlab", "data"));
task = gpenmpcMakeNoDisturbanceDiagnosticTask(task);
assets = gpenmpcLoadNativeEnmpcComparisonAssets(projectRoot, 0.30);
methods = [assets.enmpc.b1_fallback_method, assets.enmpc.method];

rows = repmat(rowTemplate(), 2, 1);
traces = cell(2, 1);
results = cell(2, 1);
for index = 1:2
    [trace, result] = gpenmpcRunNativeEnmpcWholeTask( ...
        task, methods(index), assets, ...
        MaximumInnerSamples=maximumInnerSamples, FocusedSmoke=true, ...
        AttitudeContinuityEnabled=true, ...
        AttitudeContinuityTimeConstantS=0.08, ...
        AttitudeContinuityMaximumAngularVelocityRadS=1.5, ...
        AttitudeContinuityMaximumAngularAccelerationRadS2=8.0, ...
        AttitudeContinuityMaximumContinuousDtS=0.05);
    assert(result.focused_smoke && ~result.full_task_mode);
    assert(result.sample_count == maximumInnerSamples);
    traces{index} = trace;
    results{index} = result;
    rows(index) = summarize(methods(index), trace, result, ...
        assets.enmpc.reference_transition_jerk_limit_mps3);
end

if ~isfolder(outputRoot)
    mkdir(outputRoot);
end
tracePath = fullfile(outputRoot, "NO_DISTURBANCE_FOCUSED_TRACES.mat");
save(tracePath, "traces", "results", "-v7");

receipt = struct;
receipt.schema = "GPENMPC_NO_DISTURBANCE_FOCUSED_DIAGNOSTIC_V1";
receipt.status = "COMPLETE_SAMPLE_LIMITED_DIAGNOSTIC";
receipt.mission_id = task.mission_id;
receipt.maximum_inner_samples = maximumInnerSamples;
receipt.rows = rows;
receipt.b2_minus_b1 = numericDifference(rows(2), rows(1));
receipt.trace_mat_path = string(tracePath);
receipt.trace_mat_sha256 = gpenmpcSha256File(tracePath);
written = gpenmpcWriteContentAddressedJson(outputRoot, ...
    "NO_DISTURBANCE_FOCUSED_DIAGNOSTIC", receipt);
receipt.receipt_path = written.path;
receipt.receipt_sha256 = written.sha256;
fprintf("NO_DISTURBANCE_DIAGNOSTIC_COMPLETE samples=%d B1z=%.6g B2z=%.6g\n", ...
    maximumInnerSamples, rows(1).vertical_abs_p95_m, rows(2).vertical_abs_p95_m);
end


function row = summarize(method, trace, result, referenceJerkLimit)
vertical = abs(trace.position_m(:,3) - trace.reference_position_m(:,3));
referenceJerkNorm = vecnorm(trace.reference_jerk_mps3, 2, 2);
dt = median(diff(trace.global_time_s));
row = rowTemplate();
row.method_id = method;
row.sample_count = result.sample_count;
row.vertical_abs_mean_m = mean(vertical);
row.vertical_abs_p95_m = empiricalQuantile(vertical, 0.95);
row.position_p95_m = result.position_p95_m;
row.energy_j = result.mission_controller_sensitive_energy_j;
row.mission_time_s = result.mission_time_s;
row.robust_compensation_p95_mps2 = result.robust_compensation_p95_mps2;
row.rotor_command_total_variation_n = result.rotor_command_total_variation_n;
row.jerk_max_mps3 = result.jerk_max_mps3;
row.reference_jerk_saturation_time_s = dt .* nnz( ...
    referenceJerkNorm >= referenceJerkLimit - 1.0e-10);
row.outer_correction_z_abs_mean_mps2 = ...
    mean(abs(trace.outer_correction_i_mps2(:,3)));
row.outer_correction_z_abs_p95_mps2 = ...
    empiricalQuantile(abs(trace.outer_correction_i_mps2(:,3)), 0.95);
row.vertical_observer_abs_mean_mps2 = ...
    mean(abs(trace.vertical_observer_compensation_i_mps2(:,3)));
row.gp_agreement_weight_mean_f = mean(trace.gp_agreement_weight_f, 1);
row.gp_agreement_weight_p95_f = prctile(trace.gp_agreement_weight_f, 95, 1);
row.solver_updates = result.solver.update_count;
row.gp_adopted_count = result.solver.gp_adopted_count;
row.gp_downweighted_count = result.solver.gp_downweighted_count;
row.gp_hard_invalid_count = result.solver.gp_hard_invalid_count;
row.gp_b1_fallback_count = result.solver.gp_b1_fallback_count;
row.supervisor_b1_direct_request_count = optionalNumeric( ...
    result.solver, "supervisor_b1_direct_request_count");
row.supervisor_transition_count = optionalNumeric( ...
    result.solver, "supervisor_transition_count");
row.supervisor_reentry_count = optionalNumeric( ...
    result.solver, "supervisor_reentry_count");
end


function row = rowTemplate()
row = struct( ...
    "method_id", "", "sample_count", 0, ...
    "vertical_abs_mean_m", 0.0, "vertical_abs_p95_m", 0.0, ...
    "position_p95_m", 0.0, "energy_j", 0.0, "mission_time_s", 0.0, ...
    "robust_compensation_p95_mps2", 0.0, ...
    "rotor_command_total_variation_n", 0.0, "jerk_max_mps3", 0.0, ...
    "reference_jerk_saturation_time_s", 0.0, ...
    "outer_correction_z_abs_mean_mps2", 0.0, ...
    "outer_correction_z_abs_p95_mps2", 0.0, ...
    "vertical_observer_abs_mean_mps2", 0.0, ...
    "gp_agreement_weight_mean_f", zeros(1,3), ...
    "gp_agreement_weight_p95_f", zeros(1,3), ...
    "solver_updates", 0, "gp_adopted_count", 0, ...
    "gp_downweighted_count", 0, "gp_hard_invalid_count", 0, ...
    "gp_b1_fallback_count", 0, ...
    "supervisor_b1_direct_request_count", 0, ...
    "supervisor_transition_count", 0, "supervisor_reentry_count", 0);
end


function difference = numericDifference(left, right)
difference = struct;
names = fieldnames(left);
for index = 1:numel(names)
    name = names{index};
    if isnumeric(left.(name)) && isscalar(left.(name))
        difference.(name) = double(left.(name)) - double(right.(name));
    end
end
end


function value = empiricalQuantile(input, probability)
input = sort(double(input(:)));
index = max(1, min(numel(input), ceil(probability .* numel(input))));
value = input(index);
end


function value = optionalNumeric(source, name)
if isfield(source, name)
    value = double(source.(name));
else
    value = 0.0;
end
end
