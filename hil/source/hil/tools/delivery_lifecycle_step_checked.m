function [state,command,event,audit]=delivery_lifecycle_step_checked(state,obs,timeS,config)
% Wrap the Cambridge lifecycle with the 30 s FINAL_NATIVE_LAND bound for
% an armed vehicle on dual ground, and an independent 8 s grounded-service minimum.
arguments
    state (1,1) struct
    obs (1,1) struct
    timeS (1,1) double {mustBeFinite,mustBeNonnegative}
    config (1,1) struct
end
verifyParent();
prior=string(state.name);
audit=struct('parent_sha256', ...
    "4C3C6AB152C75584277E03AC866B57D6FD050668178C8ADB19661DD075F19CA8", ...
    'precondition_override',false,'reason',"",'reference_mutated',false, ...
    'clock_basis',"CALLER_MONOTONE_EXECUTION_TIME_SECONDS", ...
    'no_physical_or_network_action',true);
if prior=="FINAL_NATIVE_LAND"
    assert(isfinite(state.state_enter_time_s) && timeS>=state.state_enter_time_s, ...
        'gpenmpcTask:LifecycleTime','Invalid final landing execution time.');
    assert(config.native_land_confirmation_timeout_s==30.0, ...
        'gpenmpcTask:LifecycleContract','Canonical final landing bound must remain 30 s.');
    if timeS-state.state_enter_time_s>=config.native_land_confirmation_timeout_s
        reject("FINAL_NATIVE_LAND_CONFIRMATION_TIMEOUT");
        return
    end
end
if prior=="SERVICE_GROUNDED_DISARMED"
    assert(config.service_dwell_s==8.0, ...
        'gpenmpcTask:LifecycleContract','Canonical Cambridge ground dwell must remain 8 s.');
    assert(isfinite(state.service_start_s) && timeS>=state.service_start_s, ...
        'gpenmpcTask:LifecycleTime','Invalid grounded service execution time.');
    if isfield(obs,'service_phase') && string(obs.service_phase)=="SERVICE_ASCENT" && ...
            timeS-state.service_start_s<config.service_dwell_s-1e-9
        reject("SERVICE_ASCENT_BEFORE_EIGHT_SECOND_GROUND_DWELL");
        return
    end
end
[state,command,event]=gpenmpcHil.deliveryLifecycleStep(state,obs,timeS,config);

    function reject(reason)
        state.name="FAIL_CLOSED";
        state.state_enter_time_s=timeS;
        state.failure_code=reason;
        state.task_complete=false;
        command=struct('request_arm',false,'request_land',false, ...
            'request_disarm',false,'request_offboard',false, ...
            'allow_payload_update',false,'hold_reference',true, ...
            'advance_reference',false,'align_ground_dwell_start',false);
        event="FAIL_CLOSED";
        audit.precondition_override=true;
        audit.reason=reason;
    end
end

function verifyParent()
persistent checked
expected=string(gpenmpc_external_path('delivery_lifecycle_source'));
assert(strcmp(which('gpenmpcHil.deliveryLifecycleStep'),expected), ...
    'gpenmpcTask:LifecycleShadow','Unexpected parent lifecycle on MATLAB path.');
if isempty(checked)
    assert(gpenmpcHil.sha256File(expected)== ...
        "4C3C6AB152C75584277E03AC866B57D6FD050668178C8ADB19661DD075F19CA8", ...
        'gpenmpcTask:LifecycleSource','Parent lifecycle source identity changed.');
    checked=true;
end
end
