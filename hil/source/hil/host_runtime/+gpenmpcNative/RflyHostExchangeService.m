classdef RflyHostExchangeService < handle
    % Exchange bytes through the bound coordinator and worker.
    % The caller supplies verified registration and echo associations.
    properties (SetAccess=private)
        Failed=false
        Failure=""
        Closed=false
        LastSourceProfile=[] % Observation diagnostics.
    end
    properties (Access=private)
        Coordinator
        Service
        Context
        Assets
        Expected
        RegisteredAssociation
        Serializer
        Dialect
        Samples={}
        PendingOuter=[]
        HeldOuterSource=[]
        PendingCommand=[]
        LastActiveSource=[]
        ReferenceGeneration=uint64(0)
        LastHostNs=uint64(0)
    end
    methods
        function obj=RflyHostExchangeService(workRoot,expected,trajectory,registeredAssociation,serializer,dialect,sourceCapacity)
            obj.verifyAssociation(expected,registeredAssociation);
            assert(string(expected.full_inner_evidence_scope)=="BOARD_COMMIT_RFC1" ...
                &&isequal(expected.async_outer_required,true));
            obj.Expected=expected;obj.RegisteredAssociation=registeredAssociation;
            obj.LastHostNs=registeredAssociation.original_host_receive_ns;
            obj.Serializer=serializer;obj.Dialect=dialect;
            obj.Assets=gpenmpcNative.loadCanonicalAssets(string(expected.canonical_package_root));
            c=expected.rfly_board_commit;
            assert(c.reference_max_age_us<=idivide(intmax('uint64'),uint64(1000)) ...
                &&c.outer_max_age_us<=idivide(intmax('uint64'),uint64(1000)));
            obj.Context=gpenmpcNative.RflyHostContextBinding(expected,sourceCapacity, ...
                c.reference_max_age_us*uint64(1000),c.outer_max_age_us*uint64(1000));
            obj.Coordinator=gpenmpcNative.CanonicalOuterCoordinator(workRoot,expected,trajectory);
            obj.Service=obj.Coordinator.controlService(); % Reuse the service handle.
        end
        function r=prepareSnapshot(obj,message,originalRxNs,nowNs,rotor,wind,origin)
            try
                obj.event(nowNs);obj.origin(origin);
                assert(originalRxNs<=nowNs);s=obj.record(message,originalRxNs);
                r=obj.Coordinator.prepare(nowNs,s,rotor,wind);
                r.exchange_scope='HOST_PREPARATION_WITH_CALLER_REGISTERED_ASSOCIATION';
                r.publication_authority=false;
            catch ex,obj.fail(ex);end
        end
        function r=source(obj,message,originalRxNs,nowNs,rotor,wind,origin)
            timing=struct('scope','HOST_EXECUTION_TIMING', ...
                'original_now_input_ns',nowNs,'original_source_receive_ns',originalRxNs, ...
                'actual_rotor_input',rotor,'actual_wind_input',wind,'actual_origin_input',origin, ...
                'actual_source_bytes',message,'clock_error','');
            timing=sourceProfileStamp(timing,'entry_ns');
            try
                obj.event(nowNs);obj.origin(origin);assert(originalRxNs<=nowNs);
                assert(isempty(obj.PendingCommand),'gpenmpcNative:ExchangePending','Prior inner output is not committed.');
                s=obj.record(message,originalRxNs);
                timing=sourceProfileStamp(timing,'after_record_ns');
                scheduled=obj.Coordinator.flightSample(nowNs,s,rotor,wind);
                obj.LastActiveSource=s;
                timing=sourceProfileStamp(timing,'after_scheduler_ns');
                obj.outerResult(scheduled.poll,nowNs);
                if isfield(scheduled.submit,'status')&&string(scheduled.submit.status)=="SUBMITTED"
                    assert(isempty(obj.PendingOuter));
                    obj.Context.submitted(uint64(scheduled.submit.generation),s.source_ticket,nowNs);
                    obj.PendingOuter=s;
                end
                r=struct('schema','RFLY_HOST_EXCHANGE_SOURCE_V1','status','WAITING_FOR_COMMITTED_OUTER', ...
                    'scheduler',scheduled,'packets',{{}},'publication_authority',false,'hardware_actions',0);
                if isempty(obj.HeldOuterSource)
                    obj.retireUnheld();timing=sourceProfileStamp(timing,'return_ns');
                    obj.LastSourceProfile=timing;r.host_source_profile=timing;return
                end
                % Use the private source's measured cadence.
                dt=double(s.actual_source_delta_us)*1e-6;
                assert(dt>0&&dt<=.0100001,'gpenmpcNative:ExchangeSourceCadence', ...
                    'A measured prior private source interval is required.');
                physical=struct('schema','GPENMPC_PHYSICAL_CAUSAL_RUNTIME_INPUT_V1', ...
                    'task_identity_sha256',obj.Expected.task_identity_sha256,'plant_truth_used',false, ...
                    'virtual_actuator',rotor,'wind',wind,'task',struct('source','FROZEN_GPENMPC_TASK_STATE', ...
                    'payload_kg',obj.Service.status().payload_kg,'leg_index',obj.Service.status().leg_index));
                timing.actual_physical_input=physical;timing.actual_source_sample=s;
                timing=sourceProfileStamp(timing,'before_physical_prepare_ns');
                prepared=obj.Service.stepPhysicalReference(uint64(s.position_generation),nowNs,s,physical,dt);
                timing=sourceProfileStamp(timing,'after_physical_prepare_ns');
                if isfield(prepared,'host_physical_profile'),timing.physical_function_stages=prepared.host_physical_profile;end
                assert(prepared.accepted,'gpenmpcNative:ExchangePrepare','Physical command not prepared.');
                command=prepared.full_inner_command;ref=prepared.base_reference.reference_ned;
                ref11=[ref.position_ned_m(:);ref.velocity_ned_mps(:);ref.acceleration_ned_mps2(:);ref.yaw_rad;ref.yawspeed_rad_s];
                obj.ReferenceGeneration=obj.ReferenceGeneration+uint64(1);
                [context,binding]=obj.Context.reference(obj.ReferenceGeneration,s.source_ticket,ref11,nowNs);
                timing=sourceProfileStamp(timing,'after_context_reference_ns');
                binding.command_generation=command.generation; % actual source-tagged command, NOT reference ordinal
                [contextPackets,contextReceipt,contextBytes]=gpenmpcNative.RflyContextEncoder(context,obj.Serializer,obj.Dialect);
                timing=sourceProfileStamp(timing,'after_context_encode_ns');
                [commandPackets,commandReceipt,commandBytes]=gpenmpcNative.RflySlimCommandEncoder( ...
                    command,obj.Assets,binding,obj.Serializer,obj.Dialect);
                timing=sourceProfileStamp(timing,'after_slim_encode_ns');
                association=struct('context',context,'binding',binding,'outer_source_sample',obj.HeldOuterSource);
                obj.PendingCommand=struct('command',command,'source',s,'association',association,'submitted',false);
                r.status='PREPARED_CONTEXT_AND_COMMAND_BYTES_NO_SEND';r.prepared=prepared;
                r.packets=[contextPackets;commandPackets];r.context_bytes=contextBytes;r.command_bytes=commandBytes;
                r.context_receipt=contextReceipt;r.command_receipt=commandReceipt;r.context=context;r.binding=binding;
                % Export the host expiry so every send can check source age before transmission.
                sourceAgeNs=uint64(obj.Expected.maximum_runtime_age_ns);
                assert(s.source_host_receive_ns<=intmax('uint64')-sourceAgeNs, ...
                    'gpenmpcNative:ExchangeSourceExpiryOverflow','Original source expiry overflow.');
                r.send_validity=struct('source_host_receive_ns',s.source_host_receive_ns, ...
                    'maximum_runtime_age_ns',sourceAgeNs,'source_valid_until_ns',s.source_host_receive_ns+sourceAgeNs);
                r.original_source=s;r.registered_association_supplied=true;
                r.live_registration_proven=false;r.transport_source_authenticated=false;
                obj.retireUnheld();
                timing=sourceProfileStamp(timing,'return_ns');obj.LastSourceProfile=timing;r.host_source_profile=timing;
            catch ex
                timing=sourceProfileStamp(timing,'failure_observed_ns');obj.LastSourceProfile=timing;obj.fail(ex);
            end
        end
        function r=beginPrepare(obj,message,originalRxNs,nowNs,rotor,wind,origin)
            try
                obj.event(nowNs);obj.origin(origin);assert(originalRxNs<=nowNs);
                s=obj.record(message,originalRxNs);
                r=obj.Coordinator.beginPrepare(nowNs,s,rotor,wind);
                r.original_source=s;r.packets={};r.publication_authority=false;
            catch ex,obj.fail(ex);end
        end
        function r=pollPreparation(obj,nowNs)
            try
                obj.event(nowNs);r=obj.Coordinator.pollPreparation(nowNs);
                r.packets={};r.publication_authority=false;obj.retireUnheld();
            catch ex,obj.fail(ex);end
        end
        function r=observeDisarmedSnapshot(obj,message,originalRxNs,nowNs,origin,boardArmed)
            try
                obj.event(nowNs);obj.origin(origin);assert(originalRxNs<=nowNs);
                assert(isempty(obj.PendingCommand),'gpenmpcNative:ExchangePending','Pending command cannot be drained away.');
                s=obj.record(message,originalRxNs);
                r=obj.Coordinator.observeDisarmedSample(nowNs,s,boardArmed);
                r.original_source=s;r.packets={};r.publication_authority=false;
                r.scientific_sample_consumed=false;r.operational_observation_only=true;
                % Retain returned evidence and monotonic replay watermarks on retirement.
                obj.retireUnheld();
            catch ex,obj.fail(ex);end
        end
        function r=disarmedStartupSnapshot(obj,message,originalRxNs,nowNs,rotor,wind,origin,boardArmed)
            % Use the NEXT real private observation after prepare, while the
            % board remains in its disarmed observation-only branch. The
            % same worker/result/ticket path prepares only the phase-0 outer;
            % no inner command or flight/observer step is created here.
            try
                obj.event(nowNs);obj.origin(origin);assert(originalRxNs<=nowNs);
                assert(isempty(obj.PendingCommand)&&isempty(obj.PendingOuter) ...
                    &&isempty(obj.HeldOuterSource),'gpenmpcNative:ExchangeStartupState', ...
                    'Startup cannot replace an existing pending or held outer.');
                s=obj.record(message,originalRxNs);
                r=obj.Coordinator.disarmedStartupSample(nowNs,s,rotor,wind,boardArmed);
                obj.Context.submitted(uint64(r.submit.generation),s.source_ticket,nowNs);
                obj.PendingOuter=s;
                r.status='DISARMED_STARTUP_OUTER_SUBMITTED_NO_INNER_COMMAND';
                r.original_source=s;r.packets={};r.publication_authority=false;
                r.live_registration_proven=false;r.transport_source_authenticated=false;
                obj.retireUnheld();
            catch ex,obj.fail(ex);end
        end
        function r=poll(obj,nowNs)
            try
                obj.event(nowNs);r=obj.Coordinator.poll(nowNs);obj.outerResult(r.poll,nowNs);
                r.packages_created=0;r.publication_authority=false;obj.retireUnheld();
            catch ex,obj.fail(ex);end
        end
        function r=submitted(obj,commandGeneration,originalSubmitNs)
            try
                obj.event(originalSubmitNs);
                assert(~isempty(obj.PendingCommand)&&~obj.PendingCommand.submitted ...
                    &&isequal(commandGeneration,obj.PendingCommand.command.generation), ...
                    'gpenmpcNative:ExchangeSubmission','Exact unsent pending command required.');
                r=obj.Service.recordPhysicalBoardCommandSubmission(commandGeneration, ...
                    obj.PendingCommand.association,originalSubmitNs);
                obj.PendingCommand.submitted=true;
            catch ex,obj.fail(ex);end
        end
        function r=feedback(obj,message,originalRxNs,nowNs,origin)
            try
                obj.event(nowNs);obj.origin(origin);assert(originalRxNs<=nowNs);
                assert(~isempty(obj.PendingCommand)&&obj.PendingCommand.submitted, ...
                    'gpenmpcNative:ExchangeFeedback','No original submitted inner command.');
                envelope=struct('schema','GPENMPC_RFLY_BOARD_COMMIT_INGRESS_V1','message',message, ...
                    'original_host_receive_ns',originalRxNs,'origin',origin);
                r=obj.Service.commitPhysicalControl(obj.PendingCommand.command.generation,nowNs,envelope);
                assert(r.accepted,'gpenmpcNative:ExchangeCommit','Actual feedback rejected by causal runtime.');
                obj.PendingCommand=[];obj.retireUnheld();
            catch ex,obj.fail(ex);end
        end
        function r=suspendForNativeLand(obj,nowNs)
            % Stop reference and outer work at the last source after resolving pending input.
            % The IO owner handles board stop, LAND, ground confirmation and release.
            try
                obj.event(nowNs);
                assert(isempty(obj.PendingCommand)&&~isempty(obj.LastActiveSource), ...
                    'gpenmpcNative:ExchangeLandPending', ...
                    'Normal LAND suspension requires the last original inner exchange to be closed.');
                before=obj.Coordinator.status();
                assert(string(before.state)=="FLIGHT",'gpenmpcNative:ExchangeLandState', ...
                    'Normal LAND suspension is once only from the current flight.');
                r=obj.Coordinator.enterNativeLand(nowNs,obj.LastActiveSource);
                after=obj.Coordinator.status();
                if isfield(before.source_clock,'last_sample_timestamp_ns')
                    sameSource=isequal(after.source_clock.last_sample_timestamp_ns, ...
                        before.source_clock.last_sample_timestamp_ns);
                else
                    sameSource=after.source_clock.last_time_s==before.source_clock.last_time_s;
                end
                assert(after.service.phase_s==before.service.phase_s ...
                    &&sameSource, ...
                    'gpenmpcNative:ExchangeLandClock','LAND cannot add an invented flight sample or task progress.');
                r.original_last_source=obj.LastActiveSource;
                r.packets={};r.packages_created=0;r.publication_authority=false;
                r.board_stop_sent=false;r.land_command_sent=false;r.board_disarmed_proven=false;
                r.plant_cache_zero_proven=false;r.session_release_proven=false;
                r.ground_observation_owner='EXISTING_SAME_IO_GENERAL_TELEMETRY_AND_MODEL_CACHE__NOT_RECREATED_RSP1';
                r.next_flight_requires_new_registered_exchange=true;
            catch ex,obj.fail(ex);end
        end
        function r=status(obj)
            r=struct('schema','RFLY_SINGLE_OWNER_HOST_EXCHANGE_STATUS_V1','failed',obj.Failed, ...
                'failure',obj.Failure,'closed',obj.Closed,'coordinator',obj.Coordinator.status(), ...
                'context',obj.Context.status(),'pending_command',~isempty(obj.PendingCommand), ...
                'startup',obj.Coordinator.status().startup, ...
                'registered_association',obj.RegisteredAssociation,'reference_generation',obj.ReferenceGeneration, ...
                'owned_coordinators',1,'owned_solvers',1,'owned_io_endpoints',0,'hardware_actions',0, ...
                'publication_authority',false,'live_registration_proven',false,'transport_source_authenticated',false);
        end
        function close(obj)
            if obj.Closed,return;end
            obj.Closed=true;obj.PendingCommand=[];
            if ~isempty(obj.Coordinator),obj.Coordinator.close();end
            % Local close is NOT a board revoke, native LAND, plant stop or ACK.
        end
        function delete(obj),obj.close();end
    end
    methods (Access=private)
        function s=record(obj,message,rx)
            s=obj.Context.recordSnapshot(message,rx,obj.LastHostNs);obj.Samples{end+1}=s;
        end
        function outerResult(obj,response,now)
            if ~isfield(response,'status')||~any(string(response.status)== ...
                    ["FRESH_RESULT_COMMITTED","DEADLINE_FALLBACK_COMMITTED"]),return;end
            assert(~isempty(obj.PendingOuter),'gpenmpcNative:ExchangeOuter','Unassociated outer result.');
            obj.Context.committed(response,now);obj.HeldOuterSource=obj.PendingOuter;obj.PendingOuter=[];
        end
        function retireUnheld(obj)
            keep=true(1,numel(obj.Samples));
            for k=1:numel(obj.Samples)
                if ~isempty(obj.PendingCommand)&&isequal(obj.Samples{k}.source_ticket,obj.PendingCommand.source.source_ticket),continue;end
                keep(k)=~obj.Context.retireSource(obj.Samples{k}.source_ticket);
            end
            obj.Samples=obj.Samples(keep);
        end
        function origin(obj,o)
            c=obj.Expected.rfly_board_commit;
            assert(isequal(o.link_lifecycle_generation,c.link_lifecycle_generation) ...
                &&strcmpi(o.execution_session_sha256,c.execution_session_sha256) ...
                &&isequal(o.source_system,uint8(obj.Expected.system_id)) ...
                &&isequal(o.source_component,uint8(obj.Expected.component_id)), ...
                'gpenmpcNative:ExchangeOrigin','Original registered route tuple mismatch.');
        end
        function event(obj,t)
            assert(~obj.Failed&&~obj.Closed,'gpenmpcNative:ExchangeClosed','Closed or failed exchange.');
            assert(isa(t,'uint64')&&isscalar(t)&&t>0&&t>=obj.LastHostNs, ...
                'gpenmpcNative:ExchangeClock','Original monotonic HOST uint64 event required.');
            obj.LastHostNs=t;
        end
        function fail(obj,ex)
            if ~obj.Failed,obj.Failed=true;obj.Failure="EXCHANGE__"+string(ex.identifier);end
            obj.close();rethrow(ex)
        end
    end
    methods (Static,Access=private)
        function verifyAssociation(expected,a)
            % Use the integration owner's verified registration receipt.
            assert(isstruct(a)&&isscalar(a)&&string(a.schema)=="GPENMPC_RFLY_REGISTERED_SESSION_ASSOCIATION_V1" ...
                &&string(a.registration_result)=="Registered"&&string(a.echo_confirmation_result)=="Confirmed", ...
                'gpenmpcNative:ExchangeRegistration','Previously registered/confirmed association required.');
            e=a.echo;c=expected.rfly_board_commit;
            assert(string(e.identity_semantics)=="BoardRegisteredExecutionSessionGenerationV1" ...
                &&isequal(e.host_challenge,a.original_host_challenge) ...
                &&isa(e.host_challenge,'uint64')&&numel(e.host_challenge)==2&&any(e.host_challenge) ...
                &&string(e.uid)==string(expected.uid)&&isequal(e.system,uint8(expected.system_id)) ...
                &&isequal(e.component,uint8(expected.component_id)) ...
                &&isequal(e.process_session_generation,uint64(expected.boot_generation)) ...
                &&isequal(e.board_registration_hrt_us,c.board_registration_hrt_us) ...
                &&isequal(e.link_lifecycle_generation,c.link_lifecycle_generation) ...
                &&strcmpi(e.configuration_payload_sha256,expected.configuration_payload_sha256) ...
                &&isa(a.original_host_receive_ns,'uint64')&&a.original_host_receive_ns>0, ...
                'gpenmpcNative:ExchangeRegistration','Echo does not bind original challenge/identity/session/link/config.');
            for value={e.uid,e.process_session_generation,e.board_registration_hrt_us,e.link_lifecycle_generation}
                assert(isa(value{1},'uint64')&&isscalar(value{1})&&value{1}>0);
            end
            bytes=[uint8('RSE1').';uint8(1);be(e.host_challenge);be(e.uid);e.system;e.component; ...
                be(e.board_registration_hrt_us);be(e.process_session_generation);be(e.link_lifecycle_generation); ...
                uint8(sscanf(char(e.configuration_payload_sha256),'%2x'))];
            md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes,'int8'));
            sha=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
            assert(strcmpi(sha,c.execution_session_sha256)&&strcmpi(sha,a.execution_session_sha256), ...
                'gpenmpcNative:ExchangeSessionDigest','Exact existing C++ RSE1 digest required.');
            function b=be(v)
                [~,~,endian]=computer;if endian=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);
            end
        end
    end
end
function profile=sourceProfileStamp(profile,name)
% Diagnostic clock failures are retained, not promoted into a new control gate.
try,profile.(name)=gpenmpcNative.rflyOriginalHostMonotonicNs();
catch problem,profile.(name)=uint64(0);profile.clock_error=problem.identifier;end
end
