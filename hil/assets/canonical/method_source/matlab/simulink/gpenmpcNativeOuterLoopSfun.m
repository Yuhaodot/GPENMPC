function gpenmpcNativeOuterLoopSfun(block)
%GPENMPCNATIVEOUTERLOOPSFUN Native sparse-GP ordinary-B2 outer loop.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 4;
block.NumOutputPorts = 4;
inputWidths = [19, 12, 5, 3];
outputWidths = [12, 10, 7, 9];
for index = 1:4
    block.InputPort(index).Dimensions = inputWidths(index);
    block.InputPort(index).DatatypeID = 0;
    block.InputPort(index).Complexity = "Real";
    block.InputPort(index).DirectFeedthrough = true;
    block.InputPort(index).SamplingMode = "Sample";
    block.OutputPort(index).Dimensions = outputWidths(index);
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = "Real";
    block.OutputPort(index).SamplingMode = "Sample";
end
block.SampleTimes = [0.01, 0];
block.SimStateCompliance = "DefaultSimState";
block.RegBlockMethod("PostPropagationSetup", @postPropagationSetup);
block.RegBlockMethod("InitializeConditions", @initialize);
block.RegBlockMethod("Outputs", @outputs);
block.RegBlockMethod("Update", @update);
end

function postPropagationSetup(block)
block.NumDworks = 14;
names = ["warm", "selected", "gp", "decision", "solver", "next_update", ...
    "phase_rate", "tangent", "history", "previous_velocity", ...
    "previous_nominal_acceleration", "initialized", "update_count", ...
    "previous_payload"];
widths = [7, 12, 10, 7, 9, 1, 1, 2, 3, 3, 3, 1, 1, 1];
for index = 1:numel(names)
    block.Dwork(index).Name = names(index);
    block.Dwork(index).Dimensions = widths(index);
    block.Dwork(index).DatatypeID = 0;
    block.Dwork(index).Complexity = "Real";
    block.Dwork(index).UsedAsDiscState = true;
end
end

function initialize(block)
for index = 1:block.NumDworks
    block.Dwork(index).Data = zeros(block.Dwork(index).Dimensions, 1);
end
block.Dwork(2).Data(3) = 1.5;
block.Dwork(6).Data = 0.0;
block.Dwork(7).Data = 1.0;
block.Dwork(8).Data = [1.0; 0.0];
end

function outputs(block)
block.OutputPort(1).Data = block.Dwork(2).Data;
block.OutputPort(2).Data = block.Dwork(3).Data;
block.OutputPort(3).Data = block.Dwork(4).Data;
block.OutputPort(4).Data = block.Dwork(5).Data;
end

function update(block)
assets = gpenmpcNativeHarnessAssets();
state = double(block.InputPort(1).Data(:));
fixedReference = double(block.InputPort(2).Data(:));
environment = double(block.InputPort(3).Data(:));
previousForce = double(block.InputPort(4).Data(:));

velocity = state(4:6);
payloadChanged = block.Dwork(12).Data > 0.5 ...
    && abs(environment(5) - block.Dwork(14).Data) > 1.0e-12;
if payloadChanged
    block.Dwork(9).Data = zeros(3,1);
elseif block.Dwork(12).Data > 0.5
    previousTangent = block.Dwork(8).Data;
    previousFrame = [previousTangent(1), -previousTangent(2), 0; ...
        previousTangent(2), previousTangent(1), 0; 0, 0, 1];
    [block.Dwork(9).Data, ~, ~] = gpenmpcAdvanceCausalResidualHistory( ...
        block.Dwork(9).Data, block.Dwork(10).Data, velocity, ...
        block.Dwork(11).Data, previousFrame, assets.sample_period_s, true, 0.50);
end
horizontal = fixedReference(4:5);
tangent = block.Dwork(8).Data;
if norm(horizontal) >= 0.75
    tangent = horizontal ./ norm(horizontal);
end
block.Dwork(8).Data = tangent;
block.Dwork(10).Data = velocity;
block.Dwork(11).Data = gpenmpcKnownNominalAccelerationFromObservation( ...
    state, environment(1:2), environment(5), assets.calibration, assets.profile);
block.Dwork(12).Data = 1.0;
block.Dwork(14).Data = environment(5);

if block.CurrentTime + 1.0e-12 < block.Dwork(6).Data
    return
end
[selected, prediction, decision, warm, audit] = gpenmpcNativeHarnessOuterStep( ...
    state, fixedReference, environment, previousForce, block.Dwork(1).Data.', ...
    block.Dwork(9).Data, block.Dwork(8).Data, block.Dwork(7).Data, assets);
block.Dwork(1).Data = warm(:);
block.Dwork(2).Data = selected(:);
block.Dwork(3).Data = [prediction.mean_mps2(:); ...
    prediction.calibrated_half_width_mps2(:); double(prediction.trust); ...
    double(prediction.hard_invalid); double(prediction.support_distance); ...
    norm(prediction.applied_mean_f_mps2)];
block.Dwork(4).Data = [double(decision.decision_blocks_s_inv(:)); ...
    double(decision.outer_acceleration_correction_f_mps2(:))];
reason = fallbackCode(string(decision.fallback_reason));
block.Dwork(5).Data = [double(decision.success); double(decision.fallback_active); ...
    reason; double(decision.elapsed_seconds); double(decision.solver_iterations); ...
    double(decision.minimum_constraint_margin); double(decision.predicted_energy_j); ...
    double(decision.mean_trust); double(audit.b1_fallback_active)];
if decision.success && ~decision.fallback_active
    block.Dwork(7).Data = min(max(block.Dwork(7).Data ...
        + assets.outer_period_s .* decision.phase_acceleration_s_inv, ...
        assets.enmpc.phase_rate_min), assets.enmpc.phase_rate_max);
end
block.Dwork(6).Data = block.Dwork(6).Data + assets.outer_period_s;
block.Dwork(13).Data = block.Dwork(13).Data + 1.0;
end

function code = fallbackCode(reason)
switch reason
    case ""
        code = 0.0;
    case "GP_HARD_INVALID"
        code = 1.0;
    case "GP_NO_FEASIBLE_PROFILE"
        code = 2.0;
    case "SOLVER_DEADLINE_MISSED"
        code = 3.0;
    otherwise
        code = 9.0;
end
end
