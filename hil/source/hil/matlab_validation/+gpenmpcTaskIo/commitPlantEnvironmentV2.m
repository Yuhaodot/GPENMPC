function [s,ack]=commitPlantEnvironmentV2(previous,core,p)
%#codegen
% Pure commit AFTER the actual same core step. No plant or network action.
% core fields are sourced from actual step/diagnostic/y.mass_kg, not a host
% assertion that a datagram was sent. applied_input_generation tags exactly
% the snapshot used for this step. No ACK if reset, rejected step, wrong mass,
% late receipt or time reversal. Original applied environment stays intact.
s=previous;
gpenmpcTaskIo.initPlantEnvironmentV2(p,previous.initialized_io_time_s);
applied=false;
if ~s.task_env_failed
    if ~s.pending_valid
        % Duplicate commit is idempotent: report held ACK, never apply again.
    else
        good=islogical(core.step_accepted)&&isscalar(core.step_accepted)&&core.step_accepted&& ...
            islogical(core.model_failed)&&isscalar(core.model_failed)&&~core.model_failed&& ...
            islogical(core.reset_applied)&&isscalar(core.reset_applied)&&~core.reset_applied&& ...
            islogical(core.ground_confirmed)&&isscalar(core.ground_confirmed)&& ...
            finiteScalar(core.io_time_s)&&core.io_time_s>=s.pending_io_time_s&& ...
            core.io_time_s-s.pending_io_time_s<=p.max_commit_delay_s&& ...
            finiteScalar(core.core_time_s)&&core.core_time_s>s.last_commit_core_time_s&& ...
            finiteScalar(core.applied_input_generation)&&core.applied_input_generation==s.pending_frame(2)&& ...
            p.expected_session_token==s.expected_session_token&&finiteScalar(core.total_mass_kg);
        % Match derivativeSoftware/copterSimIoCore left-to-right operation
        % order, including nonzero mismatch bias; real32/64 bits can differ
        % under reassociation even though a numerical tolerance would pass.
        expectedMass=p.base_mass_kg+s.pending_frame(5)+p.mass_bias_kg;
        good=good&&abs(core.total_mass_kg-expectedMass)<=p.mass_tolerance_kg;
        if good&&s.pending_unload
            good=core.ground_confirmed&&s.board_frame(25)==15&&s.board_frame(26)==1&& ...
                core.io_time_s-s.board_frame(24)<=p.max_board_age_s&& ...
                s.board_frame(24)-core.io_time_s<=p.max_future_skew_s&& ...
                s.ground_disarmed_since_s>=0&&core.io_time_s-s.ground_disarmed_since_s>=8.0;
        end
        if good
            f=s.pending_frame;
            s.environment.reference_jet_ned=f(9:20);s.environment.payload_kg=f(5);s.environment.wind_xy_mps=f(6:7);
            s.applied_frame=f;s.applied_frame_generation=f(2);s.applied_payload_generation=f(21);
            s.applied_source_io_time_s=f(3);s.applied_task_time_s=f(4);s.applied_total_mass_kg=core.total_mass_kg;
            s.last_commit_core_time_s=core.core_time_s;s.last_commit_io_time_s=core.io_time_s;
            s.has_applied_frame=true;s.applied_count=s.applied_count+uint32(1);
            if s.pending_unload,s.unload_count=s.unload_count+uint32(1);s.ground_disarmed_since_s=-1;end
            s.pending_valid=false;applied=true;
        else
            s.task_env_failed=true;s.failure_code=uint8(15);s.pending_valid=false;s.ground_disarmed_since_s=-1;
        end
    end
end
% ACK state is retained for repeated observation, but new_credit only once.
% Output layout attachment is deliberately NOT implemented in this helper.
ack=struct('new_credit',applied,'valid',s.has_applied_frame&&~s.task_env_failed, ...
    'session_token',s.expected_session_token,'applied_frame_generation',s.applied_frame_generation, ...
    'applied_payload_generation',s.applied_payload_generation,'actual_payload_kg',s.environment.payload_kg, ...
    'actual_total_mass_kg',s.applied_total_mass_kg,'applied_core_time_s',s.last_commit_core_time_s, ...
    'task_env_failed',s.task_env_failed,'failure_code',s.failure_code,'unload_count',s.unload_count, ...
    'continue_same_plant',true,'plant_step_performed',false,'plant_reset_requested',false);
end
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
