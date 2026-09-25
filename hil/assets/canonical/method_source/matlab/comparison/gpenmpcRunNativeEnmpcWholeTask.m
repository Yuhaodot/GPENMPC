function [trace, result] = gpenmpcRunNativeEnmpcWholeTask( ...
        task, methodId, assets, options)
%GPENMPCRUNNATIVEENMPCWHOLETASK Execute one native B1 or ordinary-B2 mission.
%
% The fixed task supplies only C3 reference and exogenous environment data.
% State, control, GP inference, eNMPC decisions, six-rotor allocation, plant
% propagation and energy accounting are all generated in this MATLAB run.

arguments
    task (1,1) struct
    methodId (1,1) string
    assets (1,1) struct
    options.MaximumInnerSamples (1,1) double {mustBeInteger,mustBePositive} = intmax("int32")
    options.FocusedSmoke (1,1) logical = false
    options.AttitudeContinuityEnabled (1,1) logical = false
    options.AttitudeContinuityTimeConstantS (1,1) double {mustBeNonnegative} = 0.08
    options.AttitudeContinuityMaximumAngularVelocityRadS (1,1) double {mustBePositive} = 1.5
    options.AttitudeContinuityMaximumAngularAccelerationRadS2 (1,1) double {mustBePositive} = 8.0
    options.AttitudeContinuityMaximumAngularJerkRadS3 (1,1) double {mustBePositive} = 50.0
    options.AttitudeContinuityMaximumContinuousDtS (1,1) double {mustBePositive} = 0.05
end
allowed = [assets.enmpc.b1_fallback_method, assets.enmpc.method];
if ~any(methodId == allowed)
    error("gpenmpcRunNativeEnmpcWholeTask:Method", ...
        "Method must be native B1 or ordinary B2.");
end
if ~logical(task.parent_control_injection_prohibited)
    error("gpenmpcRunNativeEnmpcWholeTask:ParentControl", ...
        "The comparison task must prohibit parent control injection.");
end

dt = double(task.mission_config.simulation.sample_period_s);
if abs(dt - 0.01) > 1.0e-12
    error("gpenmpcRunNativeEnmpcWholeTask:SamplePeriod", ...
        "The shared native plant must retain the 100 Hz inner-loop period.");
end
config = assets.enmpc;
attitudeContinuityConfig = struct( ...
    "enabled", options.AttitudeContinuityEnabled, ...
    "time_constant_s", options.AttitudeContinuityTimeConstantS, ...
    "maximum_angular_velocity_rad_s", ...
        options.AttitudeContinuityMaximumAngularVelocityRadS, ...
    "maximum_angular_acceleration_rad_s2", ...
        options.AttitudeContinuityMaximumAngularAccelerationRadS2, ...
    "maximum_angular_jerk_rad_s3", ...
        options.AttitudeContinuityMaximumAngularJerkRadS3, ...
    "maximum_continuous_dt_s", ...
        options.AttitudeContinuityMaximumContinuousDtS);
context = gpenmpcNativeEnmpcPredictionContext(assets);
referenceInput = task.reference;
legIds = unique(referenceInput.leg_index(:), "stable");
expectedLegs = numel(task.plans(1).legs);
if numel(legIds) ~= expectedLegs
    error("gpenmpcRunNativeEnmpcWholeTask:LegCount", ...
        "Reference and frozen plan leg counts differ.");
end

capacity = min(double(options.MaximumInnerSamples), ...
    max(128, ceil(numel(referenceInput.global_time_s) .* 1.60)));
log = initializeLog(capacity);
cursor = 0;
globalTime = 0.0;
serviceTime = 0.0;
serviceEnergy = 0.0;
solver = initializeSolverCounters(methodId);
outerUpdateEvents = struct([]);
allLegsComplete = true;
stopRequested = false;
firstLegIndex = find(referenceInput.leg_index == legIds(1));
firstTrajectory = gpenmpcSampledC3Trajectory( ...
    referenceInput.local_time_s(firstLegIndex), ...
    referenceInput.position_m(firstLegIndex,:), ...
    referenceInput.velocity_mps(firstLegIndex,:), ...
    referenceInput.acceleration_mps2(firstLegIndex,:), ...
    referenceInput.jerk_mps3(firstLegIndex,:));
firstPayloadKg = double(task.plans(1).legs(1).payload_departure_kg);
firstReference = gpenmpcPhaseReference(firstTrajectory, 0.0, 1.0, 0.0, 0.0);
nominalMass = double(assets.profile.mass_properties.base_mass_kg) + firstPayloadKg;
hover = nominalMass .* 9.80665 ./ 6.0;
state = [firstReference.position_m; zeros(3,1); 1; zeros(6,1); ...
    repmat(hover, 6, 1)];
robustState = gpenmpcInitializeRobustSe3State();

for legCursor = 1:numel(legIds)
    legId = legIds(legCursor);
    sourceIndex = find(referenceInput.leg_index == legId);
    localTime = referenceInput.local_time_s(sourceIndex);
    trajectory = gpenmpcSampledC3Trajectory(localTime, ...
        referenceInput.position_m(sourceIndex,:), ...
        referenceInput.velocity_mps(sourceIndex,:), ...
        referenceInput.acceleration_mps2(sourceIndex,:), ...
        referenceInput.jerk_mps3(sourceIndex,:));
    planLeg = task.plans(1).legs(legCursor);
    payloadKg = double(planLeg.payload_departure_kg);
    nominalMass = double(assets.profile.mass_properties.base_mass_kg) + payloadKg;
    warmStart = zeros(1, config.decision_dimension);
    phase = 0.0;
    rate = 1.0;
    previousPhaseAcceleration = 0.0;
    targetPhaseAcceleration = 0.0;
    rawTargetPhaseAcceleration = 0.0;
    previousOuterCorrectionI = zeros(3,1);
    targetOuterCorrectionF = zeros(3,1);
    rawTargetOuterCorrectionF = zeros(3,1);
    outerCommandSourceCode = 0;
    previousDesiredForce = nominalMass .* [0;0;9.80665];
    nextOuterUpdate = 0.0;
    legWallTime = 0.0;
    causalValid = false;
    latestGpEvidence = struct;
    lastSupervisorRequestReason = "NOT_REQUESTED";
    gpAgreementWeightF = zeros(3, 1);
    previousRotorCommandN = state(14:19);
    outerSupervisor = gpenmpcInitializeOuterSupervisor(config);
    if commandContinuityEnabled(config)
        commandContinuity = gpenmpcInitializeCommandContinuity(config);
        outerCommandSourceCode = double( ...
            config.command_source_codebook.SAFE_NEUTRAL_COMMAND);
    else
        commandContinuity = struct;
    end
    residualHistoryF = zeros(3,1);
    previousAcceleration = zeros(3,1);
    havePreviousAcceleration = false;
    attitudeContinuityState = gpenmpcInitializeDesiredAttitudeContinuityState();
    robustState = gpenmpcResetCausalVerticalDisturbanceObserver( ...
        robustState, state(4:6), globalTime, legId, payloadKg);
    maximumLegSamples = ceil(trajectory.total_duration_s ...
        ./ (config.phase_rate_min .* dt)) + 4;

    for legSample = 1:maximumLegSamples
        if cursor >= options.MaximumInnerSamples
            stopRequested = true;
            allLegsComplete = false;
            break
        end
        windEstimate = exogenousAt(referenceInput, "wind_estimate_xy_mps", globalTime);
        actualWind = exogenousAt(referenceInput, "actual_wind_xy_mps", globalTime);

        if legWallTime + 1.0e-12 >= nextOuterUpdate
            observation = struct( ...
                "position_m", state(1:3), ...
                "velocity_mps", state(4:6), ...
                "quaternion_wxyz", state(7:10), ...
                "body_rate_rad_s", state(11:13), ...
                "previous_desired_force_n", previousDesiredForce, ...
                "previous_rotor_command_n", previousRotorCommandN, ...
                "previous_outer_acceleration_correction_i_mps2", previousOuterCorrectionI, ...
                "runtime_vertical_observer_shadow_i_mps2", [0; 0; ...
                    -numericFieldOr(robustState, ...
                    "vertical_disturbance_ewma_mps2", 0.0)], ...
                "wind_estimate_xy_mps", windEstimate, ...
                "payload_kg", payloadKg, ...
                "corridor_half_width_m", config.corridor_half_width_m);
            phaseState = struct( ...
                "progress_s", phase, ...
                "progress_rate", rate, ...
                "previous_phase_acceleration_s_inv", previousPhaseAcceleration);
            causal = struct("valid", causalValid, ...
                "values", [gpenmpcNormalizedTotalRotorCommand( ...
                    previousRotorCommandN, payloadKg); residualHistoryF], ...
                "runtime_gp_axis_weight_f", gpAgreementWeightF, ...
                "runtime_gp_responsibility_blend", ...
                    numericFieldOr(robustState, ...
                    "gp_responsibility_blend", 0.0), ...
                "runtime_gp_filtered_mean_f_mps2", ...
                    vectorFieldOr(robustState, ...
                    "responsibility_filtered_gp_mean_f_mps2", 0.0).');
            if methodId == config.method
                activeShadow = latestGpEvidence;
                coordinatedSingleSupervisor = isfield(config, ...
                    "coordinated_single_supervisor_enabled") ...
                    && logical(config.coordinated_single_supervisor_enabled);
                directPredictionCommit = isfield(config, ...
                    "prediction_effective_direct_b2_enabled") ...
                    && logical(config.prediction_effective_direct_b2_enabled);
                if coordinatedSingleSupervisor
                    [decision, warmStart, audit] = gpenmpcCoordinatedOuterStep( ...
                        warmStart, trajectory, phaseState, observation, ...
                        context, assets.gp_model, causal, activeShadow, config);
                    lastSupervisorRequestReason = ...
                        "AXISWISE_CAUSAL_RESPONSIBILITY";
                elseif directPredictionCommit
                    responsibilityGateEnabled = isfield(config, ...
                        "gp_responsibility_gate_enabled") ...
                        && logical(config.gp_responsibility_gate_enabled);
                    responsibilityBlend = numericFieldOr(robustState, ...
                        "gp_responsibility_blend", 0.0);
                    if responsibilityGateEnabled ...
                            && responsibilityBlend <= 1.0e-12
                        % The responsibility gate owns the complete learned
                        % execution layer.  When it is closed, use the exact
                        % B1 outer solve as well as the exact B1 inner
                        % augmentation.  Merely multiplying the GP mean by
                        % zero is insufficient because method-specific trust
                        % and selector bookkeeping can otherwise alter the
                        % committed eNMPC decision.
                        lastSupervisorRequestReason = ...
                            "RESPONSIBILITY_GATE_EXACT_B1";
                        [decision, warmStart, audit] = gpenmpcNativeB1OuterStep( ...
                            warmStart, trajectory, phaseState, observation, ...
                            context, config);
                        audit.shadow_gp = activeShadow;
                        audit.gp_responsibility_exact_b1 = true;
                    else
                        lastSupervisorRequestReason = ...
                            "PREDICTION_EFFECTIVE_DIRECT_B2";
                        [decision, warmStart, audit] = gpenmpcOrdinaryB2OuterStep( ...
                            warmStart, trajectory, phaseState, observation, ...
                            context, assets.gp_model, causal, config);
                        audit.shadow_gp = activeShadow;
                        audit.gp_responsibility_exact_b1 = false;
                    end
                else
                    supervisorRequest = gpenmpcOuterSupervisorRequest( ...
                        outerSupervisor, causal.valid, config, activeShadow);
                    lastSupervisorRequestReason = ...
                        string(supervisorRequest.request_reason);
                    if supervisorRequest.requested_method == config.method
                        [decision, warmStart, audit] = gpenmpcOrdinaryB2OuterStep( ...
                            warmStart, trajectory, phaseState, observation, context, ...
                            assets.gp_model, causal, config);
                        audit.shadow_gp = activeShadow;
                    else
                        [decision, warmStart, audit] = gpenmpcNativeB1OuterStep( ...
                            warmStart, trajectory, phaseState, observation, context, config);
                        audit.shadow_gp = activeShadow;
                    end
                end
            else
                [decision, warmStart, audit] = gpenmpcNativeB1OuterStep( ...
                    warmStart, trajectory, phaseState, observation, context, config);
            end
            [solver, rawTargetPhaseAcceleration, rawTargetOuterCorrectionF] = ...
                observeDecision(solver, decision, audit, methodId, config);
            targetPhaseAcceleration = rawTargetPhaseAcceleration;
            targetOuterCorrectionF = rawTargetOuterCorrectionF;
            if methodId == config.method
                coordinatedSingleSupervisor = isfield(config, ...
                    "coordinated_single_supervisor_enabled") ...
                    && logical(config.coordinated_single_supervisor_enabled);
                directPredictionCommit = isfield(config, ...
                    "prediction_effective_direct_b2_enabled") ...
                    && logical(config.prediction_effective_direct_b2_enabled);
                if ~coordinatedSingleSupervisor && ~directPredictionCommit
                    [outerSupervisor, appliedTarget, supervisorEvent] = ...
                        gpenmpcOuterSupervisorStep(outerSupervisor, ...
                            supervisorRequest, decision, audit, config, rate);
                    targetPhaseAcceleration = ...
                        appliedTarget.phase_acceleration_s_inv;
                    targetOuterCorrectionF = ...
                        appliedTarget.outer_acceleration_correction_f_mps2;
                    solver = gpenmpcObserveSupervisorEvent( ...
                        solver, supervisorEvent);
                end
            end
            if commandContinuityEnabled(config)
                [commandContinuity, appliedTarget, commandEvent] = ...
                    gpenmpcApplyCommandContinuity( ...
                        commandContinuity, decision, audit, methodId, config);
                targetPhaseAcceleration = ...
                    appliedTarget.phase_acceleration_s_inv;
                targetOuterCorrectionF = ...
                    appliedTarget.outer_acceleration_correction_f_mps2;
                outerCommandSourceCode = appliedTarget.source_code;
                commandEvent.global_time_s = globalTime;
                commandEvent.leg_index = double(legId);
                commandEvent.leg_wall_time_s = legWallTime;
                if isempty(outerUpdateEvents)
                    outerUpdateEvents = commandEvent;
                else
                    outerUpdateEvents(end).next_update_recovery = ...
                        appliedTarget.source;
                    outerUpdateEvents(end).next_update_raw_success = ...
                        logical(decision.success);
                    commandEvent = orderfields(commandEvent, outerUpdateEvents);
                    outerUpdateEvents(end+1) = commandEvent; %#ok<AGROW>
                end
                solver = observeCommandEvent(solver, commandEvent);
            end
            nextOuterUpdate = nextOuterUpdate + config.outer_period_s;
        end

        transition = gpenmpcJerkBoundedReferenceTransition(trajectory, phase, rate, ...
            previousPhaseAcceleration, targetPhaseAcceleration, ...
            previousOuterCorrectionI, targetOuterCorrectionF, dt, ...
            config.reference_transition_jerk_limit_mps3);
        reference = transition.reference;
        [robustState, ~] = gpenmpcObserveCausalVerticalDisturbance( ...
            robustState, state(4:6), globalTime, legId, payloadKg, assets.robust);
        coordinatedGpExecution = methodId == config.method ...
            && isfield(config, "coordinated_gp_execution_feedforward_enabled") ...
            && logical(config.coordinated_gp_execution_feedforward_enabled);
        if coordinatedGpExecution
            [augmentation, robustState, robustDiagnostic] = ...
                gpenmpcCoordinatedGpRobustSe3Augmentation( ...
                state, reference, assets.robust, robustState, dt, ...
                latestGpEvidence, gpAgreementWeightF, config);
        else
            [augmentation, robustState, robustDiagnostic] = ...
                gpenmpcRobustSe3Augmentation( ...
                state, reference, assets.robust, robustState, dt);
            robustDiagnostic = attachInheritedCompensationBreakdown( ...
                robustDiagnostic, augmentation);
        end
        attitudeCommand = [];
        if attitudeContinuityConfig.enabled
            rawSe3Command = gpenmpcDesiredSe3Command(state, reference, ...
                payloadKg, windEstimate, augmentation, assets.calibration, ...
                assets.profile);
            [attitudeCommand, attitudeContinuityState] = ...
                gpenmpcUpdateDesiredAttitudeContinuity( ...
                    attitudeContinuityState, rawSe3Command.desired_rotation, ...
                    dt, legSample == 1, attitudeContinuityConfig);
        end
        [derivative, diagnostic] = gpenmpcM600ClosedLoopDerivative( ...
            state, reference, payloadKg, actualWind, windEstimate, globalTime, ...
            augmentation, task.mission_config, assets.calibration, assets.profile, ...
            attitudeCommand);
        [powerW, powerDiagnostic] = gpenmpcM600ControllerSensitivePower( ...
            assets.profile, assets.calibration, payloadKg, state, reference, ...
            actualWind, windEstimate);

        cursor = cursor + 1;
        if cursor > capacity
            capacity = min(double(options.MaximumInnerSamples), max(cursor, capacity * 2));
            log = growLog(log, capacity);
        end
        acceleration = diagnostic.actual_acceleration_mps2;
        if havePreviousAcceleration
            jerk = (acceleration - previousAcceleration) ./ dt;
        else
            jerk = zeros(3,1);
        end
        log = storeSample(log, cursor, globalTime, legId, phase, rate, ...
            payloadKg, reference, state, diagnostic, augmentation, acceleration, ...
            jerk, powerW, powerDiagnostic, solver, transition, robustDiagnostic, ...
            gpAgreementWeightF, actualWind, windEstimate, latestGpEvidence, ...
            outerSupervisor, lastSupervisorRequestReason, config, ...
            rawTargetPhaseAcceleration, rawTargetOuterCorrectionF, ...
            targetPhaseAcceleration, targetOuterCorrectionF, ...
            outerCommandSourceCode);

        pendingGpEvidence = gpenmpcPredictCurrentGpEvidence( ...
            state(4:6), reference, windEstimate, payloadKg, ...
            diagnostic.desired_force_projected_n, previousRotorCommandN, ...
            residualHistoryF, context, assets.gp_model, causalValid, config);

        robustState = gpenmpcRobustSe3ObserveControl(robustState, diagnostic, dt);
        robustState = gpenmpcCommitCausalVerticalDisturbanceObserver( ...
            robustState, state(4:6), diagnostic.nominal_acceleration_mps2, ...
            globalTime, legId, payloadKg);
        nextState = integrateRk4(state, derivative, reference, payloadKg, ...
            actualWind, windEstimate, globalTime, augmentation, task.mission_config, ...
            assets.calibration, assets.profile, dt, attitudeCommand);
        frame = pendingGpEvidence.gp_frame_i_from_f;
        [residualHistoryF, residualF, labelValid] = ...
            gpenmpcAdvanceCausalResidualHistory(residualHistoryF, ...
            state(4:6), nextState(4:6), diagnostic.nominal_acceleration_mps2, ...
            frame, dt, true, 0.50);
        latestObservedInnovationI = frame * residualF;
        observedInnovationAvailable = labelValid ...
            && all(isfinite(latestObservedInnovationI));
        latestGpEvidence = gpenmpcCloseGpInnovationEvidence( ...
            pendingGpEvidence, latestObservedInnovationI, ...
            observedInnovationAvailable, config);
        [gpAgreementWeightF, consistencyDiagnostic] = ...
            gpenmpcUpdateGpAgreementWeight( ...
            gpAgreementWeightF, latestGpEvidence, dt, config);
        latestGpEvidence = attachConsistencyDiagnostic( ...
            latestGpEvidence, consistencyDiagnostic, config);
        causalValid = labelValid && all(isfinite(residualHistoryF));

        state = nextState;
        previousDesiredForce = diagnostic.desired_force_projected_n;
        previousRotorCommandN = diagnostic.rotor_command_n;
        previousAcceleration = acceleration;
        havePreviousAcceleration = true;
        previousPhaseAcceleration = transition.phase_acceleration_s_inv;
        previousOuterCorrectionI = transition.outer_correction_i_mps2;
        nextRate = min(max(rate + dt .* previousPhaseAcceleration, ...
            config.phase_rate_min), config.phase_rate_max);
        nextPhase = min(trajectory.total_duration_s, ...
            phase + dt .* rate + 0.5 .* dt.^2 .* previousPhaseAcceleration);
        rate = nextRate;
        phase = nextPhase;
        legWallTime = legWallTime + dt;
        globalTime = globalTime + dt;
        if phase >= trajectory.total_duration_s - 1.0e-12
            break
        end
    end
    if phase < trajectory.total_duration_s - 1.0e-9
        allLegsComplete = false;
    end
    if stopRequested
        break
    end
    [duration, payloadAfter] = serviceAccounting(planLeg, payloadKg);
    energy = 0.0;
    if duration > 0.0
        serviceNode = gpenmpcEvaluateTrajectoryDerivative( ...
            trajectory, trajectory.total_duration_s, 0);
        [state, service] = gpenmpcApplyFixedC3ServiceTransition( ...
            state, payloadKg, payloadAfter, duration, serviceNode, assets.profile);
        energy = double(service.controller_sensitive_energy_j);
    end
    serviceTime = serviceTime + duration;
    serviceEnergy = serviceEnergy + energy;
    globalTime = globalTime + duration;
end

trace = trimLog(log, cursor);
trace.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_WHOLE_TASK_TRACE_V1";
trace.method_id = methodId;
trace.mission_id = task.mission_id;
trace.planner_id = task.planner_id;
trace.outer_period_s = config.outer_period_s;
trace.prediction_step_s = config.prediction_step_s;
trace.prediction_horizon_s = config.prediction_horizon_s;
trace.attitude_continuity_enabled = attitudeContinuityConfig.enabled;
trace.attitude_continuity_time_constant_s = attitudeContinuityConfig.time_constant_s;
trace.attitude_continuity_maximum_angular_velocity_rad_s = ...
    attitudeContinuityConfig.maximum_angular_velocity_rad_s;
trace.attitude_continuity_maximum_angular_acceleration_rad_s2 = ...
    attitudeContinuityConfig.maximum_angular_acceleration_rad_s2;
trace.attitude_continuity_maximum_angular_jerk_rad_s3 = ...
    attitudeContinuityConfig.maximum_angular_jerk_rad_s3;
trace.supervisor_mode_codebook = struct( ...
    "B2_ACTIVE", 1, "B1_ONLY", 2, "B2_REENTRY_PROBE", 3, ...
    "FIXED_REFERENCE_HOLD", 4);
trace.supervisor_transition_reason_codebook = struct( ...
    "NO_MODE_CHANGE", 0, ...
    "B1_SHADOW_INNOVATION_INCONSISTENT_RESET", 1, ...
    "B1_DWELL_COMPLETE", 2, "B1_DWELL_CONTINUES", 3, ...
    "B2_PROBE_SUCCESS_PERSISTENCE_PENDING", 4, ...
    "B2_REENTRY_PERSISTENCE_SATISFIED", 5, ...
    "GP_HARD_INVALID_TO_B1", 6, "OTHER", 99);
trace.supervisor_request_reason_codebook = struct( ...
    "B2_ACTIVE", 0, "B1_RECOVERY_DWELL", 1, ...
    "B2_REENTRY_PROBE_WITH_CLOSED_GP_EVIDENCE", 2, ...
    "B2_REENTRY_GP_INNOVATION_INCONSISTENT_DIRECT_B1", 3, ...
    "PREDICTION_EFFECTIVE_DIRECT_B2", 4, ...
    "RESPONSIBILITY_GATE_EXACT_B1", 5, ...
    "OTHER", 99);
trace.command_continuity_enabled = ...
    commandContinuityEnabled(config);
trace.outer_update_events = outerUpdateEvents;
if trace.command_continuity_enabled
    trace.outer_command_source_codebook = ...
        config.command_source_codebook;
else
    trace.outer_command_source_codebook = struct;
end

allocation = gpenmpcM600Allocation(assets.calibration);
result = computeResult(trace, task, methodId, config, solver, ...
    allLegsComplete, stopRequested, serviceTime, serviceEnergy, ...
    allocation.per_rotor_upper_n, allocation.maximum_tilt_rad, options.FocusedSmoke);
end

function log = initializeLog(count)
log = struct;
one = [count, 1]; three = [count, 3]; four = [count, 4]; six = [count, 6];
log.global_time_s = zeros(one); log.leg_index = zeros(one);
log.progress_s = zeros(one); log.progress_rate = zeros(one);
log.payload_kg = zeros(one); log.power_w = zeros(one);
log.public_power_w = zeros(one); log.position_error_norm_m = zeros(one);
log.attitude_error_norm_rad = zeros(one); log.acceleration_norm_mps2 = zeros(one);
log.jerk_norm_mps3 = zeros(one); log.robust_compensation_norm_mps2 = zeros(one);
log.tilt_rad = zeros(one);
log.attitude_continuity_enabled = false(one);
log.attitude_continuity_reset_active = false(one);
log.solver_update_count_cumulative = zeros(one);
log.reference_position_m = zeros(three); log.reference_velocity_mps = zeros(three);
log.reference_acceleration_mps2 = zeros(three); log.reference_jerk_mps3 = zeros(three);
log.position_m = zeros(three); log.velocity_mps = zeros(three);
log.body_rate_rad_s = zeros(three); log.actual_acceleration_mps2 = zeros(three);
log.actual_jerk_mps3 = zeros(three); log.robust_compensation_i_mps2 = zeros(three);
log.outer_correction_i_mps2 = zeros(three);
log.outer_correction_jerk_i_mps3 = zeros(three);
log.outer_raw_target_phase_acceleration_s_inv = zeros(one);
log.outer_applied_target_phase_acceleration_s_inv = zeros(one);
log.outer_raw_target_correction_f_mps2 = zeros(three);
log.outer_applied_target_correction_f_mps2 = zeros(three);
log.outer_command_source_code = zeros(one);
log.robust_target_i_mps2 = zeros(three);
log.vertical_observer_compensation_i_mps2 = zeros(three);
log.gp_execution_feedforward_i_mps2 = zeros(three);
log.gp_raw_control_target_i_mps2 = zeros(three);
log.gp_filtered_control_target_i_mps2 = zeros(three);
log.residual_robust_compensation_i_mps2 = zeros(three);
log.total_compensation_i_mps2 = zeros(three);
log.gp_execution_authority = zeros(one);
log.gp_prediction_axis_authority_f = zeros(three);
log.gp_physical_axis_authority_f = zeros(three);
log.gp_responsibility_blend = zeros(one);
log.gp_responsibility_mode_active = false(one);
log.gp_innovation_ratio_ewma_f = zeros(three);
log.gp_innovation_ratio_instant_f = zeros(three);
log.gp_residual_floor_f_mps2 = zeros(three);
log.gp_mode_transition_count = zeros(one);
log.vertical_disturbance_estimate_mps2 = zeros(one);
log.vertical_observer_shadow_i_mps2 = zeros(three);
log.gp_agreement_weight_f = zeros(three);
log.actual_wind_xy_mps = zeros(count, 2);
log.wind_estimate_xy_mps = zeros(count, 2);
log.wind_estimation_error_xy_mps = zeros(count, 2);
log.residual_acceleration_mps2 = zeros(three);
log.structured_acceleration_mps2 = zeros(three);
log.fast_acceleration_mps2 = zeros(three);
log.desired_force_raw_n = zeros(three);
log.desired_force_projected_n = zeros(three);
log.desired_moment_nm = zeros(three);
log.desired_rotation_matrix_i_from_b = zeros(count, 9);
log.gp_predicted_mean_f_mps2 = zeros(three);
log.gp_calibrated_half_width_f_mps2 = zeros(three);
log.gp_observed_innovation_f_mps2 = zeros(three);
log.gp_innovation_error_f_mps2 = zeros(three);
log.gp_trust = zeros(one);
log.gp_support_distance = zeros(one);
log.gp_latent_variance_max = zeros(one);
log.gp_evidence_available = false(one);
log.gp_observed_innovation_available = false(one);
log.gp_hard_invalid = false(one);
log.gp_observed_innovation_consistent = false(one);
log.gp_eligible_for_b1_dwell = false(one);
log.gp_normalized_innovation_error_f = zeros(three);
log.gp_consistency_score_instant_f = zeros(three);
log.gp_consistency_evidence_active = false(one);
log.gp_consistency_ewma_aggregate = zeros(one);
log.gp_consistency_enter_eligible = false(one);
log.gp_consistency_exit_low = false(one);
log.supervisor_mode_code = zeros(one);
log.supervisor_transition_reason_code = zeros(one);
log.supervisor_request_reason_code = zeros(one);
log.supervisor_transition_count_cumulative = zeros(one);
log.supervisor_b1_success_dwell_ticks = zeros(one);
log.supervisor_b2_probe_success_ticks = zeros(one);
log.solver_last_elapsed_s = zeros(one);
log.solver_elapsed_available = false(one);
log.solver_deadline_s = zeros(one);
log.solver_last_deadline_miss = false(one);
log.attitude_error_rad = zeros(three); log.rotor_command_n = zeros(six);
log.desired_angular_velocity_body_rad_s = zeros(three);
log.desired_angular_acceleration_body_rad_s2 = zeros(three);
log.geometric_feedforward_moment_nm = zeros(three);
log.per_rotor_thrust_n = zeros(six); log.quaternion_wxyz = zeros(four);
end

function log = growLog(log, count)
names = fieldnames(log);
for index = 1:numel(names)
    value = log.(names{index});
    value(count, size(value,2)) = 0.0;
    log.(names{index}) = value;
end
end

function log = storeSample(log, k, time, leg, phase, rate, payload, ...
        reference, state, diagnostic, augmentation, acceleration, jerk, ...
        powerW, powerDiagnostic, solver, transition, robustDiagnostic, ...
        gpAgreementWeightF, actualWind, windEstimate, gpEvidence, ...
        supervisor, supervisorRequestReason, config, ...
        rawTargetPhaseAcceleration, rawTargetOuterCorrectionF, ...
        appliedTargetPhaseAcceleration, appliedTargetOuterCorrectionF, ...
        outerCommandSourceCode)
log.global_time_s(k) = time; log.leg_index(k) = leg;
log.progress_s(k) = phase; log.progress_rate(k) = rate;
log.payload_kg(k) = payload; log.power_w(k) = powerW;
log.public_power_w(k) = powerDiagnostic.public_power_w;
log.reference_position_m(k,:) = reference.position_m.';
log.reference_velocity_mps(k,:) = reference.velocity_mps.';
log.reference_acceleration_mps2(k,:) = reference.acceleration_mps2.';
log.reference_jerk_mps3(k,:) = reference.jerk_mps3.';
log.position_m(k,:) = state(1:3).'; log.velocity_mps(k,:) = state(4:6).';
log.quaternion_wxyz(k,:) = (state(7:10) ./ max(norm(state(7:10)),1e-15)).';
log.body_rate_rad_s(k,:) = state(11:13).';
log.per_rotor_thrust_n(k,:) = state(14:19).';
log.rotor_command_n(k,:) = diagnostic.rotor_command_n.';
log.desired_angular_velocity_body_rad_s(k,:) = ...
    diagnostic.desired_angular_velocity_body_rad_s.';
log.desired_angular_acceleration_body_rad_s2(k,:) = ...
    diagnostic.desired_angular_acceleration_body_rad_s2.';
log.geometric_feedforward_moment_nm(k,:) = ...
    diagnostic.geometric_feedforward_moment_nm.';
log.attitude_continuity_enabled(k) = diagnostic.attitude_continuity_enabled;
log.attitude_continuity_reset_active(k) = ...
    diagnostic.attitude_continuity_reset_active;
log.actual_acceleration_mps2(k,:) = acceleration.';
log.actual_jerk_mps3(k,:) = jerk.';
log.robust_compensation_i_mps2(k,:) = augmentation.';
log.outer_correction_i_mps2(k,:) = transition.outer_correction_i_mps2.';
log.outer_correction_jerk_i_mps3(k,:) = ...
    transition.outer_correction_jerk_i_mps3.';
log.outer_raw_target_phase_acceleration_s_inv(k) = ...
    rawTargetPhaseAcceleration;
log.outer_applied_target_phase_acceleration_s_inv(k) = ...
    appliedTargetPhaseAcceleration;
log.outer_raw_target_correction_f_mps2(k,:) = ...
    rawTargetOuterCorrectionF(:).';
log.outer_applied_target_correction_f_mps2(k,:) = ...
    appliedTargetOuterCorrectionF(:).';
log.outer_command_source_code(k) = outerCommandSourceCode;
log.robust_target_i_mps2(k,:) = robustDiagnostic.target_i_mps2.';
log.vertical_observer_compensation_i_mps2(k,:) = ...
    robustDiagnostic.vertical_observer_compensation_i_mps2.';
log.gp_execution_feedforward_i_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_execution_feedforward_i_mps2", 0.0);
log.gp_raw_control_target_i_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_raw_control_target_i_mps2", 0.0);
log.gp_filtered_control_target_i_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_filtered_control_target_i_mps2", 0.0);
log.residual_robust_compensation_i_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "residual_robust_compensation_i_mps2", 0.0);
log.total_compensation_i_mps2(k,:) = augmentation.';
log.gp_execution_authority(k) = numericFieldOr( ...
    robustDiagnostic, "gp_authority", 0.0);
log.gp_prediction_axis_authority_f(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_prediction_axis_authority_f", 0.0);
log.gp_physical_axis_authority_f(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_physical_axis_authority_f", 0.0);
log.gp_responsibility_blend(k) = numericFieldOr( ...
    robustDiagnostic, "gp_responsibility_blend", 0.0);
log.gp_responsibility_mode_active(k) = logicalFieldOr( ...
    robustDiagnostic, "gp_responsibility_mode_active", false);
log.gp_innovation_ratio_ewma_f(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_innovation_ratio_ewma_f", 0.0);
log.gp_innovation_ratio_instant_f(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_innovation_ratio_instant_f", 0.0);
log.gp_residual_floor_f_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "gp_residual_floor_f_mps2", 0.0);
log.gp_mode_transition_count(k) = numericFieldOr( ...
    robustDiagnostic, "gp_mode_transition_count", 0.0);
log.vertical_disturbance_estimate_mps2(k) = ...
    robustDiagnostic.vertical_disturbance_estimate_mps2;
log.vertical_observer_shadow_i_mps2(k,:) = vectorFieldOr( ...
    robustDiagnostic, "vertical_observer_shadow_i_mps2", 0.0);
log.gp_agreement_weight_f(k,:) = gpAgreementWeightF.';
log.actual_wind_xy_mps(k,:) = actualWind(:).';
log.wind_estimate_xy_mps(k,:) = windEstimate(:).';
log.wind_estimation_error_xy_mps(k,:) = ...
    windEstimate(:).' - actualWind(:).';
log.residual_acceleration_mps2(k,:) = ...
    diagnostic.residual_acceleration_mps2(:).';
log.structured_acceleration_mps2(k,:) = ...
    diagnostic.structured_acceleration_mps2(:).';
log.fast_acceleration_mps2(k,:) = ...
    diagnostic.fast_acceleration_mps2(:).';
log.desired_force_raw_n(k,:) = diagnostic.desired_force_raw_n(:).';
log.desired_force_projected_n(k,:) = ...
    diagnostic.desired_force_projected_n(:).';
log.desired_moment_nm(k,:) = diagnostic.desired_moment_nm(:).';
log.desired_rotation_matrix_i_from_b(k,:) = ...
    reshape(diagnostic.desired_rotation, 1, 9);
[log.gp_predicted_mean_f_mps2(k,:), ...
    log.gp_calibrated_half_width_f_mps2(k,:), ...
    log.gp_observed_innovation_f_mps2(k,:), ...
    log.gp_innovation_error_f_mps2(k,:)] = evidenceVectors(gpEvidence);
log.gp_trust(k) = numericFieldOr(gpEvidence, "trust", 0.0);
log.gp_support_distance(k) = ...
    numericFieldOr(gpEvidence, "support_distance", 0.0);
log.gp_latent_variance_max(k) = ...
    numericFieldOr(gpEvidence, "latent_variance_max", 0.0);
log.gp_evidence_available(k) = logicalFieldOr(gpEvidence, "available", false);
log.gp_observed_innovation_available(k) = logicalFieldOr( ...
    gpEvidence, "observed_innovation_available", false);
log.gp_hard_invalid(k) = logicalFieldOr(gpEvidence, "hard_invalid", false);
log.gp_observed_innovation_consistent(k) = logicalFieldOr( ...
    gpEvidence, "observed_innovation_consistent", false);
log.gp_eligible_for_b1_dwell(k) = logicalFieldOr( ...
    gpEvidence, "eligible_for_b1_dwell", false);
log.gp_normalized_innovation_error_f(k,:) = vectorFieldOr( ...
    gpEvidence, "normalized_innovation_error_f", 0.0);
log.gp_consistency_score_instant_f(k,:) = vectorFieldOr( ...
    gpEvidence, "innovation_consistency_score_f", 0.0);
log.gp_consistency_evidence_active(k) = logicalFieldOr( ...
    gpEvidence, "consistency_evidence_active", false);
log.gp_consistency_ewma_aggregate(k) = numericFieldOr( ...
    gpEvidence, "consistency_ewma_aggregate", 0.0);
log.gp_consistency_enter_eligible(k) = logicalFieldOr( ...
    gpEvidence, "consistency_enter_eligible", false);
log.gp_consistency_exit_low(k) = logicalFieldOr( ...
    gpEvidence, "consistency_exit_low", false);
log.supervisor_mode_code(k) = supervisorModeCode(supervisor);
log.supervisor_transition_reason_code(k) = ...
    supervisorTransitionReasonCode(supervisor);
log.supervisor_request_reason_code(k) = ...
    supervisorRequestReasonCode(supervisorRequestReason);
log.supervisor_transition_count_cumulative(k) = ...
    numericFieldOr(supervisor, "transition_count", 0.0);
log.supervisor_b1_success_dwell_ticks(k) = ...
    numericFieldOr(supervisor, "b1_success_dwell_ticks", 0.0);
log.supervisor_b2_probe_success_ticks(k) = ...
    numericFieldOr(supervisor, "b2_probe_success_ticks", 0.0);
log.solver_elapsed_available(k) = ~isempty(solver.elapsed_seconds);
log.solver_last_elapsed_s(k) = lastOrZero(solver.elapsed_seconds);
log.solver_deadline_s(k) = double(config.solver_deadline_s);
log.solver_last_deadline_miss(k) = ...
    log.solver_last_elapsed_s(k) > log.solver_deadline_s(k);
positionError = norm(state(1:3) - reference.position_m);
log.position_error_norm_m(k) = positionError;
log.attitude_error_rad(k,:) = diagnostic.attitude_error.';
log.attitude_error_norm_rad(k) = norm(diagnostic.attitude_error);
force = diagnostic.desired_force_projected_n;
log.tilt_rad(k) = atan2(norm(force(1:2)), max(force(3), 1.0e-12));
log.acceleration_norm_mps2(k) = norm(acceleration);
log.jerk_norm_mps3(k) = norm(jerk);
log.robust_compensation_norm_mps2(k) = norm(augmentation);
log.solver_update_count_cumulative(k) = solver.update_count;
end


function [meanF, halfWidthF, observedF, errorF] = evidenceVectors(evidence)
meanF = vectorFieldOr(evidence, "predicted_mean_f_mps2", 0.0);
halfWidthF = vectorFieldOr(evidence, "calibrated_half_width_f_mps2", 0.0);
observedF = vectorFieldOr(evidence, "observed_innovation_f_mps2", 0.0);
errorF = vectorFieldOr(evidence, "innovation_error_f_mps2", 0.0);
end


function value = vectorFieldOr(source, name, fallback)
value = fallback .* ones(1, 3);
if isstruct(source) && isfield(source, name)
    candidate = double(source.(name));
    if numel(candidate) == 3
        value = reshape(candidate, 1, 3);
    end
end
end


function evidence = attachConsistencyDiagnostic(evidence, diagnostic, config)
evidence.consistency_ewma_f = ...
    double(diagnostic.next_weight_f(:).');
evidence.consistency_evidence_active = ...
    logical(diagnostic.valid_closed_evidence);
evidence.consistency_ewma_aggregate = ...
    double(diagnostic.aggregate_consistency_score);
evidence.normalized_innovation_error_f = ...
    double(diagnostic.normalized_innovation_error_f(:).');
evidence.innovation_consistency_score_f = ...
    double(diagnostic.instantaneous_consistency_score_f(:).');
evidence.consistency_enter_eligible = logical(diagnostic.enter_eligible);
evidence.consistency_exit_low = logical(diagnostic.exit_low);
evidence.eligible_for_b1_dwell = logical(diagnostic.enter_eligible) ...
    && logicalFieldOr(evidence, "available", false) ...
    && logicalFieldOr(evidence, "observed_innovation_available", false) ...
    && ~logicalFieldOr(evidence, "hard_invalid", true) ...
    && numericFieldOr(evidence, "trust", 0.0) ...
        >= double(config.minimum_soft_trust);
end


function value = numericFieldOr(source, name, fallback)
value = double(fallback);
if isstruct(source) && isfield(source, name)
    candidate = double(source.(name));
    if isscalar(candidate)
        value = candidate;
    end
end
end


function value = logicalFieldOr(source, name, fallback)
value = logical(fallback);
if isstruct(source) && isfield(source, name)
    candidate = logical(source.(name));
    if isscalar(candidate)
        value = candidate;
    end
end
end


function value = lastOrZero(input)
if isempty(input)
    value = 0.0;
else
    value = double(input(end));
end
end


function code = supervisorModeCode(supervisor)
mode = string(fieldOr(supervisor, "mode", ""));
if mode == "B2_ACTIVE"
    code = 1;
elseif mode == "B1_ONLY"
    code = 2;
elseif mode == "B2_REENTRY_PROBE"
    code = 3;
elseif mode == "FIXED_REFERENCE_HOLD"
    code = 4;
else
    code = 0;
end
end


function code = supervisorTransitionReasonCode(supervisor)
reason = string(fieldOr(supervisor, "last_transition_reason", ""));
if reason == "NO_MODE_CHANGE"
    code = 0;
elseif reason == "B1_SHADOW_INNOVATION_INCONSISTENT_RESET"
    code = 1;
elseif reason == "B1_DWELL_COMPLETE"
    code = 2;
elseif reason == "B1_DWELL_CONTINUES"
    code = 3;
elseif reason == "B2_PROBE_SUCCESS_PERSISTENCE_PENDING"
    code = 4;
elseif reason == "B2_REENTRY_PERSISTENCE_SATISFIED"
    code = 5;
elseif reason == "GP_HARD_INVALID_TO_B1"
    code = 6;
else
    code = 99;
end
end


function code = supervisorRequestReasonCode(reason)
reason = string(reason);
if reason == "B2_ACTIVE"
    code = 0;
elseif reason == "B1_RECOVERY_DWELL"
    code = 1;
elseif reason == "B2_REENTRY_PROBE_WITH_CLOSED_GP_EVIDENCE"
    code = 2;
elseif reason == "B2_REENTRY_GP_INNOVATION_INCONSISTENT_DIRECT_B1"
    code = 3;
elseif reason == "PREDICTION_EFFECTIVE_DIRECT_B2"
    code = 4;
elseif reason == "RESPONSIBILITY_GATE_EXACT_B1"
    code = 5;
else
    code = 99;
end
end


function value = fieldOr(source, name, fallback)
value = fallback;
if isstruct(source) && isfield(source, name)
    value = source.(name);
end
end

function output = trimLog(log, count)
output = struct;
names = fieldnames(log);
for index = 1:numel(names)
    output.(names{index}) = log.(names{index})(1:count,:);
end
end

function state = integrateRk4(state, k1, reference, payload, actualWind, ...
        windEstimate, globalTime, augmentation, mission, calibration, profile, ...
        dt, attitudeCommand)
midReference = gpenmpcReferenceExtrapolation(reference, 0.5 .* dt);
nextReference = gpenmpcReferenceExtrapolation(reference, dt);
k2 = gpenmpcM600ClosedLoopDerivative(state + 0.5 .* dt .* k1, ...
    midReference, payload, actualWind, windEstimate, globalTime + 0.5 .* dt, ...
    augmentation, mission, calibration, profile, attitudeCommand);
k3 = gpenmpcM600ClosedLoopDerivative(state + 0.5 .* dt .* k2, ...
    midReference, payload, actualWind, windEstimate, globalTime + 0.5 .* dt, ...
    augmentation, mission, calibration, profile, attitudeCommand);
k4 = gpenmpcM600ClosedLoopDerivative(state + dt .* k3, nextReference, ...
    payload, actualWind, windEstimate, globalTime + dt, augmentation, ...
    mission, calibration, profile, attitudeCommand);
state = state + dt .* (k1 + 2 .* k2 + 2 .* k3 + k4) ./ 6.0;
state(7:10) = state(7:10) ./ max(norm(state(7:10)), 1.0e-15);
end

function value = exogenousAt(reference, name, globalTime)
time = double(reference.global_time_s(:));
query = min(max(globalTime, time(1)), time(end));
value = interp1(time, double(reference.(name)), query, "linear").';
end

function [duration, payloadAfter] = serviceAccounting(planLeg, payloadBefore)
duration = 0.0; payloadAfter = payloadBefore;
if ~isfield(planLeg, "service") || isfield(planLeg.service, "status")
    return
end
service = planLeg.service;
fields = ["ascent_duration_s", "descent_duration_s", "ground_service_duration_s"];
for field = fields
    if isfield(service, field)
        duration = duration + double(service.(field));
    end
end
if isfield(service, "payload_after_kg")
    payloadAfter = double(service.payload_after_kg);
end
end

function counters = initializeSolverCounters(methodId)
counters = struct("method_id", methodId, "update_count", 0, ...
    "success_count", 0, "failure_count", 0, "deadline_miss_count", 0, ...
    "no_feasible_count", 0, "fallback_count", 0, "gp_adopted_count", 0, ...
    "gp_downweighted_count", 0, "gp_hard_invalid_count", 0, ...
    "gp_b1_fallback_count", 0, "elapsed_seconds", zeros(0,1), ...
    "command_new_feasible_count", 0, ...
    "command_one_period_hold_count", 0, ...
    "command_exact_b1_fallback_count", 0, ...
    "command_safe_neutral_count", 0, ...
    "command_hard_invalid_termination_count", 0, ...
    "candidate_switch_suppressed_count", 0, ...
    "prediction_effective_guard_count", 0, ...
    "prediction_effective_execution_rejection_count", 0, ...
    "prediction_effective_required_improvement_sum", 0.0, ...
    "prediction_effective_uncertainty_penalty_sum", 0.0, ...
    "prediction_effective_switch_penalty_sum", 0.0, ...
    "objective_term_order", ["ENERGY", "POSITION", "VELOCITY", ...
        "ROTOR_VARIATION", "JERK", "PHASE_ACCELERATION", ...
        "PHASE_ACCELERATION_VARIATION", "PROGRESS", ...
        "TERMINAL_ENERGY_TO_GO"], ...
    "selected_candidate_index", zeros(0,1), ...
    "selected_candidate_index_without_energy", zeros(0,1), ...
    "selected_objective_vector", zeros(0,9), ...
    "selected_predicted_energy_j", zeros(0,1), ...
    "candidate_predicted_energy_spread_j", zeros(0,1), ...
    "energy_term_changed_selected_candidate_count", 0, ...
    "trust", zeros(0,1));
end

function counters = observeCommandEvent(counters, event)
source = string(event.applied_command_source);
if source == "NEW_FEASIBLE_CANDIDATE"
    counters.command_new_feasible_count = ...
        counters.command_new_feasible_count + 1;
elseif source == "ONE_PERIOD_LAST_FEASIBLE_HOLD"
    counters.command_one_period_hold_count = ...
        counters.command_one_period_hold_count + 1;
elseif source == "EXACT_B1_FALLBACK"
    counters.command_exact_b1_fallback_count = ...
        counters.command_exact_b1_fallback_count + 1;
elseif source == "SAFE_NEUTRAL_COMMAND"
    counters.command_safe_neutral_count = ...
        counters.command_safe_neutral_count + 1;
elseif source == "HARD_INVALID_TERMINATION"
    counters.command_hard_invalid_termination_count = ...
        counters.command_hard_invalid_termination_count + 1;
else
    error("gpenmpcRunNativeEnmpcWholeTask:CommandSource", ...
        "Unknown command source %s.", source);
end
end

function enabled = commandContinuityEnabled(config)
enabled = isfield(config, "command_continuity_enabled") ...
    && logical(config.command_continuity_enabled);
end

function [counters, phaseAcceleration, correctionF] = observeDecision( ...
        counters, decision, audit, methodId, config)
counters.update_count = counters.update_count + 1;
counters.elapsed_seconds(end+1,1) = double(decision.elapsed_seconds);
counters.trust(end+1,1) = double(decision.mean_trust);
selectedIndex = 0;
if isfield(audit, "selected_index")
    selectedIndex = double(audit.selected_index);
end
counters.selected_candidate_index(end+1,1) = selectedIndex;
withoutEnergyIndex = 0;
if isfield(audit, "selected_index_without_energy_term")
    withoutEnergyIndex = double(audit.selected_index_without_energy_term);
end
counters.selected_candidate_index_without_energy(end+1,1) = ...
    withoutEnergyIndex;
objectiveVector = nan(1,9);
selectedPredictedEnergy = NaN;
energySpread = NaN;
if selectedIndex > 0 && isfield(audit, "candidate_objective_vector") ...
        && size(audit.candidate_objective_vector, 1) >= selectedIndex
    objectiveVector = double(audit.candidate_objective_vector(selectedIndex,:));
end
if selectedIndex > 0 && isfield(audit, "candidate_predicted_energy_j") ...
        && numel(audit.candidate_predicted_energy_j) >= selectedIndex
    selectedPredictedEnergy = ...
        double(audit.candidate_predicted_energy_j(selectedIndex));
end
if isfield(audit, "candidate_predicted_energy_j") ...
        && ~isempty(audit.candidate_predicted_energy_j)
    energyValues = double(audit.candidate_predicted_energy_j(:));
    energyValues = energyValues(isfinite(energyValues));
    if ~isempty(energyValues)
        energySpread = max(energyValues) - min(energyValues);
    end
end
counters.selected_objective_vector(end+1,:) = objectiveVector;
counters.selected_predicted_energy_j(end+1,1) = selectedPredictedEnergy;
counters.candidate_predicted_energy_spread_j(end+1,1) = energySpread;
if isfield(audit, "energy_term_changed_selected_candidate") ...
        && logical(audit.energy_term_changed_selected_candidate)
    counters.energy_term_changed_selected_candidate_count = ...
        counters.energy_term_changed_selected_candidate_count + 1;
end
if logical(decision.success)
    counters.success_count = counters.success_count + 1;
    phaseAcceleration = double(decision.phase_acceleration_s_inv);
    correctionF = double(decision.outer_acceleration_correction_f_mps2(:));
else
    counters.failure_count = counters.failure_count + 1;
    phaseAcceleration = 0.0;
    correctionF = zeros(3,1);
end
reason = string(decision.fallback_reason);
if contains(reason, "DEADLINE")
    counters.deadline_miss_count = counters.deadline_miss_count + 1;
end
if contains(reason, "NO_FEASIBLE")
    counters.no_feasible_count = counters.no_feasible_count + 1;
end
if logical(decision.fallback_active) || logical(decision.gp_b1_fallback_active)
    counters.fallback_count = counters.fallback_count + 1;
end
if isfield(decision, "profile_switch_suppressed") ...
        && logical(decision.profile_switch_suppressed)
    counters.candidate_switch_suppressed_count = ...
        counters.candidate_switch_suppressed_count + 1;
end
if isfield(audit, "prediction_effective_commit_guard_active") ...
        && logical(audit.prediction_effective_commit_guard_active)
    counters.prediction_effective_guard_count = ...
        counters.prediction_effective_guard_count + 1;
    if isfield(audit, "prediction_effective_execution_nonworse") ...
            && ~logical(audit.prediction_effective_execution_nonworse)
        counters.prediction_effective_execution_rejection_count = ...
            counters.prediction_effective_execution_rejection_count + 1;
    end
    counters.prediction_effective_required_improvement_sum = ...
        counters.prediction_effective_required_improvement_sum + ...
        numericFieldOr(audit, ...
            "prediction_effective_required_relative_improvement", 0.0);
    counters.prediction_effective_uncertainty_penalty_sum = ...
        counters.prediction_effective_uncertainty_penalty_sum + ...
        numericFieldOr(audit, ...
            "prediction_effective_uncertainty_penalty_fraction", 0.0);
    counters.prediction_effective_switch_penalty_sum = ...
        counters.prediction_effective_switch_penalty_sum + ...
        numericFieldOr(audit, ...
            "prediction_effective_switch_penalty_fraction", 0.0);
end
if methodId == config.method
    hard = logical(decision.hard_invalid);
    trust = double(decision.mean_trust);
    if hard
        counters.gp_hard_invalid_count = counters.gp_hard_invalid_count + 1;
    elseif trust > 0.0 && logical(decision.success)
        counters.gp_adopted_count = counters.gp_adopted_count + 1;
        if trust < 1.0 - 1.0e-12
            counters.gp_downweighted_count = counters.gp_downweighted_count + 1;
        end
    end
    if isfield(audit, "b1_fallback_active") && logical(audit.b1_fallback_active)
        counters.gp_b1_fallback_count = counters.gp_b1_fallback_count + 1;
    end
end
end

function result = computeResult(trace, task, methodId, config, solver, ...
        taskComplete, stopped, serviceTime, serviceEnergy, rotorUpper, ...
        tiltLimit, focused)
flightEnergy = segmentedEnergy(trace.power_w, trace.global_time_s, trace.leg_index);
position = trace.position_error_norm_m;
acceleration = trace.acceleration_norm_mps2;
jerk = trace.jerk_norm_mps3;
attitude = trace.attitude_error_norm_rad;
robust = trace.robust_compensation_norm_mps2;
residualRobust = vecnorm(trace.residual_robust_compensation_i_mps2, 2, 2);
gpExecutionFeedforward = vecnorm(trace.gp_execution_feedforward_i_mps2, 2, 2);
gpRawControlTarget = vecnorm(trace.gp_raw_control_target_i_mps2, 2, 2);
gpFilteredControlTarget = vecnorm(trace.gp_filtered_control_target_i_mps2, 2, 2);
observerCompensation = vecnorm(trace.vertical_observer_compensation_i_mps2, 2, 2);
observerShadow = vecnorm(trace.vertical_observer_shadow_i_mps2, 2, 2);
rotorTv = segmentedTotalVariation(trace.rotor_command_n, trace.leg_index);
minimumMargin = min(rotorUpper - trace.rotor_command_n, [], "all");
minimumClearance = min(arrayfun(@(leg) ...
    double(leg.minimum_exact_buffered_clearance_m), task.plans(1).legs));
conservativeClearance = minimumClearance - max(position);
result = struct;
result.schema = "GPENMPC_MATLAB_NATIVE_ENMPC_WHOLE_TASK_RESULT_V1";
result.status = ternary(taskComplete, "VALID_METHOD_CASE", "INCOMPLETE_METHOD_CASE");
result.focused_smoke = focused;
result.full_task_mode = ~focused;
result.mission_id = task.mission_id;
result.planner_id = task.planner_id;
result.method_id = methodId;
result.outer_period_s = config.outer_period_s;
result.solver_deadline_s = config.solver_deadline_s;
result.prediction_step_s = config.prediction_step_s;
result.horizon_steps = config.horizon_steps;
result.prediction_horizon_s = config.prediction_horizon_s;
result.task_complete = taskComplete;
result.stopped_by_maximum_inner_samples = stopped;
result.sample_count = numel(trace.global_time_s);
result.flight_time_s = sumSegmentDurations(trace.global_time_s, trace.leg_index);
result.service_time_s = serviceTime;
result.mission_time_s = result.flight_time_s + serviceTime;
result.flight_controller_sensitive_energy_j = flightEnergy;
result.service_energy_j = serviceEnergy;
result.mission_controller_sensitive_energy_j = flightEnergy + serviceEnergy;
result.position_rms_m = sqrt(mean(position.^2));
result.position_p95_m = empiricalQuantile(position, 0.95);
result.position_max_m = max(position);
result.attitude_p95_rad = empiricalQuantile(attitude, 0.95);
result.tilt_max_rad = max(trace.tilt_rad);
result.acceleration_p95_mps2 = empiricalQuantile(acceleration, 0.95);
result.acceleration_max_mps2 = max(acceleration);
result.jerk_p95_mps3 = empiricalQuantile(jerk, 0.95);
result.jerk_max_mps3 = max(jerk);
result.robust_compensation_p95_mps2 = empiricalQuantile(robust, 0.95);
result.residual_robust_compensation_p95_mps2 = ...
    empiricalQuantile(residualRobust, 0.95);
result.gp_execution_feedforward_p95_mps2 = ...
    empiricalQuantile(gpExecutionFeedforward, 0.95);
result.gp_raw_control_target_p95_mps2 = ...
    empiricalQuantile(gpRawControlTarget, 0.95);
result.gp_filtered_control_target_p95_mps2 = ...
    empiricalQuantile(gpFilteredControlTarget, 0.95);
result.vertical_observer_compensation_p95_mps2 = ...
    empiricalQuantile(observerCompensation, 0.95);
result.vertical_observer_shadow_p95_mps2 = ...
    empiricalQuantile(observerShadow, 0.95);
result.total_compensation_p95_mps2 = ...
    empiricalQuantile(vecnorm(trace.total_compensation_i_mps2, 2, 2), 0.95);
result.gp_execution_authority_mean = mean(trace.gp_execution_authority);
result.gp_execution_active_fraction = mean(trace.gp_execution_authority > 1.0e-12);
result.gp_prediction_axis_authority_mean_f = ...
    mean(trace.gp_prediction_axis_authority_f, 1);
result.gp_physical_axis_authority_mean_f = ...
    mean(trace.gp_physical_axis_authority_f, 1);
result.gp_responsibility_blend_mean = mean(trace.gp_responsibility_blend);
result.gp_responsibility_mode_active_fraction = ...
    mean(trace.gp_responsibility_mode_active);
result.gp_responsibility_mode_transition_count = ...
    max(trace.gp_mode_transition_count);
result.rotor_command_total_variation_n = rotorTv;
result.minimum_per_rotor_margin_n = minimumMargin;
result.minimum_inherited_c3_clearance_m = minimumClearance;
result.minimum_conservative_vehicle_clearance_lower_bound_m = conservativeClearance;
deadlineMissMask = solver.elapsed_seconds > config.solver_deadline_s;
solver.deadline_miss_count = nnz(deadlineMissMask);
result.solver = summarizeSolver(solver, config);
result.solver_elapsed_seconds = solver.elapsed_seconds;
result.solver_gp_trust = solver.trust;
result.solver_deadline_miss_indices = find(deadlineMissMask).';
result.screens = struct( ...
    "task_complete", taskComplete, ...
    "finite", gpenmpcAllFiniteActiveTrace(trace), ...
    "economic_reference_complete", taskComplete, ...
    "position_max_le_1p2", result.position_max_m <= 1.2 + 1.0e-12, ...
    "acceleration_max_le_2p2", result.acceleration_max_mps2 <= 2.2 + 1.0e-12, ...
    "jerk_max_le_4p0", result.jerk_max_mps3 <= 4.0 + 1.0e-12, ...
    "tilt_within_common_limit", result.tilt_max_rad <= tiltLimit + 1.0e-12, ...
    "continuous_clearance_lower_bound_positive", conservativeClearance > 0.0, ...
    "rotor_margin_nonnegative", result.minimum_per_rotor_margin_n >= -1.0e-12, ...
    "solver_deadline_miss_zero", solver.deadline_miss_count == 0);
result.all_threshold_checks_pass = all(struct2array(result.screens));
end


function diagnostic = attachInheritedCompensationBreakdown(diagnostic, augmentation)
diagnostic.gp_execution_feedforward_i_mps2 = zeros(3,1);
diagnostic.gp_raw_control_target_i_mps2 = zeros(3,1);
diagnostic.gp_filtered_control_target_i_mps2 = zeros(3,1);
diagnostic.residual_robust_compensation_i_mps2 = ...
    diagnostic.raw_target_i_mps2 ...
    - diagnostic.vertical_observer_compensation_i_mps2;
diagnostic.vertical_observer_shadow_i_mps2 = ...
    diagnostic.vertical_observer_compensation_i_mps2;
diagnostic.total_applied_compensation_i_mps2 = double(augmentation(:));
diagnostic.gp_authority = 0.0;
diagnostic.gp_responsibility_blend = 0.0;
diagnostic.gp_responsibility_mode_active = false;
diagnostic.gp_innovation_ratio_ewma_f = zeros(3,1);
diagnostic.gp_innovation_ratio_instant_f = zeros(3,1);
diagnostic.gp_residual_floor_f_mps2 = zeros(3,1);
diagnostic.gp_mode_transition_count = 0;
diagnostic.exact_b1_fallback = true;
end

function summary = summarizeSolver(solver, config)
summary = rmfield(solver, ["elapsed_seconds", "trust"]);
summary.deadline_s = config.solver_deadline_s;
summary.elapsed_max_s = maximumOrZero(solver.elapsed_seconds);
summary.elapsed_p95_s = empiricalQuantile(solver.elapsed_seconds, 0.95);
summary.gp_trust_mean = meanOrZero(solver.trust);
summary.gp_trust_positive_fraction = meanOrZero(solver.trust > 0.0);
end

function energy = segmentedEnergy(power, time, segment)
[energy, ~] = gpenmpcIntegrateControllerSensitiveEnergy(power, time, segment);
end

function duration = sumSegmentDurations(time, segment)
duration = 0.0;
for identity = unique(segment(:)).'
    index = segment == identity;
    duration = duration + max(time(index)) - min(time(index));
end
end

function total = segmentedTotalVariation(signal, segment)
total = 0.0;
for identity = unique(segment(:)).'
    value = signal(segment == identity, :);
    total = total + sum(abs(diff(value, 1, 1)), "all");
end
end

function value = empiricalQuantile(input, probability)
input = sort(double(input(:)));
if isempty(input)
    value = 0.0;
    return
end
index = max(1, min(numel(input), ceil(probability .* numel(input))));
value = input(index);
end

function value = maximumOrZero(input)
if isempty(input), value = 0.0; else, value = max(input); end
end

function value = meanOrZero(input)
if isempty(input), value = 0.0; else, value = mean(input); end
end

function pass = allFiniteTrace(trace)
pass = true;
names = fieldnames(trace);
predictionDiagnostics = [ ...
    "gp_predicted_mean_f_mps2", ...
    "gp_calibrated_half_width_f_mps2", ...
    "gp_support_distance", ...
    "gp_latent_variance_max"];
closedDiagnostics = [ ...
    "gp_observed_innovation_f_mps2", ...
    "gp_innovation_error_f_mps2", ...
    "gp_normalized_innovation_error_f", ...
    "gp_consistency_score_instant_f"];
for index = 1:numel(names)
    name = string(names{index});
    value = trace.(names{index});
    if any(name == predictionDiagnostics)
        value = value(logical(trace.gp_evidence_available), :);
    elseif any(name == closedDiagnostics)
        value = value(logical(trace.gp_observed_innovation_available), :);
    end
    if isnumeric(value) && any(~isfinite(double(value)), "all")
        pass = false;
        return
    end
end
end

function value = ternary(condition, yes, no)
if condition, value = yes; else, value = no; end
end
