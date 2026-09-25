function [response,stopRequested]=liveHilPlantHostService( ...
        action,payload,workRoot)
% Dispatch one MATLAB M600 plant through loopback.
arguments
    action (1,1) string
    payload (1,1) struct = struct()
    workRoot (1,1) string = ""
end
persistent service initialized
if isempty(initialized),initialized=false;end
stopRequested=false;
try
    switch upper(action)
        case "RESET"
            clear service
            initialized=false;
            response=baseResponse("RESET");
        case "INITIALIZE"
            assert(~initialized&&strlength(workRoot)>0&&isfolder(workRoot), ...
                'gpenmpcNative:LivePlantInitializeState', ...
                'Live plant must initialize exactly once in a valid root.');
            requireFields(payload,{'schema','task_path','expected'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_INITIALIZE_REQUEST_V1", ...
                'gpenmpcNative:LivePlantInitializeSchema','Bad schema.');
            service=gpenmpcNative.LiveHilPlantService( ...
                workRoot,string(payload.task_path),payload.expected);
            initialized=true;
            response=baseResponse("INITIALIZED");
            response.schema="GPENMPC_LIVE_PLANT_INITIALIZE_RESPONSE_V1";
            response.service=service.status();
        case "PREPARE"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','px4_sample'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_PREPARE_REQUEST_V1", ...
                'gpenmpcNative:LivePlantPrepareSchema','Bad schema.');
            response=service.prepare(double(payload.now_ns),payload.px4_sample);
        case "ADMIT"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','px4_sample'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_ADMIT_REQUEST_V1", ...
                'gpenmpcNative:LivePlantAdmitSchema','Bad schema.');
            response=service.admit(double(payload.now_ns),payload.px4_sample);
        case "BEGIN"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','px4_sample'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_BEGIN_REQUEST_V1", ...
                'gpenmpcNative:LivePlantBeginSchema','Bad schema.');
            response=service.beginSample(double(payload.now_ns),payload.px4_sample);
        case "COMMIT_ADVANCE_AND_BEGIN"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','px4_sample', ...
                'module_guard','actuator_output'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_COMMIT_ADVANCE_REQUEST_V1", ...
                'gpenmpcNative:LivePlantCommitSchema','Bad schema.');
            response=service.commitAdvanceAndBegin(double(payload.now_ns), ...
                payload.px4_sample,payload.module_guard,payload.actuator_output);
        case "LAND_STEP"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','actuator_output','board_armed'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_LAND_STEP_REQUEST_V1", ...
                'gpenmpcNative:LivePlantLandStepSchema','Bad schema.');
            response=service.landingStep(double(payload.now_ns), ...
                payload.actuator_output,logical(payload.board_armed));
        case "GROUND_STEP"
            requireInitialized(initialized);
            requireFields(payload,{'schema','dt_s','board_armed'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_GROUND_STEP_REQUEST_V1", ...
                'gpenmpcNative:LivePlantGroundStepSchema','Bad schema.');
            response=service.groundStep(double(payload.dt_s), ...
                logical(payload.board_armed));
        case "RESUME"
            requireInitialized(initialized);
            requireFields(payload,{'schema','now_ns','px4_sample'});
            assert(string(payload.schema)== ...
                "GPENMPC_LIVE_PLANT_RESUME_REQUEST_V1", ...
                'gpenmpcNative:LivePlantResumeSchema','Bad schema.');
            response=service.resumeAdmission(double(payload.now_ns), ...
                payload.px4_sample);
        case "STATUS"
            requireInitialized(initialized);
            response=service.status();
        case "STOP"
            if initialized
                finalStatus=service.status();
            else
                finalStatus=struct('initialized',false);
            end
            clear service
            initialized=false;
            response=baseResponse("STOPPED");
            response.final_status=finalStatus;
            stopRequested=true;
        otherwise
            error('gpenmpcNative:LivePlantAction','Unknown action %s.',action);
    end
catch exception
    response=baseResponse("FAIL_CLOSED");
    response.failure_code=string(exception.identifier)+":"+string(exception.message);
    response.fail_closed=true;
end
end

function requireInitialized(initialized)
assert(initialized,'gpenmpcNative:LivePlantNotInitialized', ...
    'Initialize the live plant before this action.');
end

function requireFields(value,names)
assert(isstruct(value)&&isscalar(value)&&all(isfield(value,names)), ...
    'gpenmpcNative:LivePlantRequestFields','Missing request field.');
end

function response=baseResponse(status)
response=struct('status',string(status),'fail_closed',false, ...
    'hardware_actions',0,'com_open',0,'board_access',0, ...
    'parameter_writes',0,'mapping_writes',0,'arm_requests',0, ...
    'mode_requests',0,'task_rows',0,'physical_output_commands',0);
end
