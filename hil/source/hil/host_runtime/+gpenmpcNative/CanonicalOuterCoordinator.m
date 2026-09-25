classdef CanonicalOuterCoordinator < handle
    % Coordinate observed inputs, one outer service/worker and lifecycle.
    % The caller supplies validated observations and timestamps.
    properties (SetAccess=private)
        Failed=false
        FailureCode=""
        Initialized=false
        State="UNPREPARED"
        InputEvents=uint64(0)
        SubmitAttempts=uint64(0)
        ActualSolverSubmissions=uint64(0)
        RejectedDispatches=uint64(0)
        PollCount=uint64(0)
        LastHostNs=uint64(0)
    end
    properties (Access=private)
        Service
        Clock=[]
        Expected
        LastSourceTimestampNs=uint64(0)
        Startup=[]
        ClockLegOffset=0
    end
    methods
        function obj=CanonicalOuterCoordinator(workRoot,expected,trajectory)
            assert(isfield(expected,'async_outer_required') ...
                &&isequal(expected.async_outer_required,true), ...
                'gpenmpcNative:CoordinatorAsyncRequired', ...
                'The real-time coordinator requires the single-flight async worker.');
            obj.Expected=expected;
            obj.Service=gpenmpcNative.BoardOuterService(workRoot,expected,trajectory);
            % Map a later global mission leg to local clock leg 1 without changing task identity.
            s=obj.Service.status();obj.ClockLegOffset=double(s.leg_index)-1;
        end

        function r=prepare(obj,nowNs,sample,rotor,wind)
            assert(~obj.Initialized&&~obj.Failed&&obj.State=="UNPREPARED", ...
                'gpenmpcNative:CoordinatorPrepareState','Preparation is once only.');
            obj.checkHostTime(nowNs);
            % Pure identity/clock checks precede solver preparation. A bad
            % wrapper input must not leave service Prepared but owner unset.
            e=obj.clockEvent(sample,false,true);
            [next,c]=obj.clockStep([],e);
            assert(c.accepted,'gpenmpcNative:CoordinatorClockInit','%s',c.reason);
            try
                runtime=obj.observedRuntime(nowNs,sample,rotor,wind);
                r=obj.Service.prepare(nowNs,sample,runtime);
                if string(r.status)~="PASS_HOST_PREPARATION"
                    obj.fail("PREPARATION__"+string(r.reason));
                end
            catch ex
                obj.recordException(ex);rethrow(ex)
            end
            obj.Clock=next;obj.Initialized=true;obj.State="PREPARED_PAUSED";
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r.clock=c;r.final_hil_admission=false;r.hardware_actions=0;
        end

        function r=flightSample(obj,nowNs,sample,rotor,wind)
            % Call once per actual canonical source sample; POLL below is
            % independent and must not advance phase or the source clock.
            obj.requireReady();obj.checkHostTime(nowNs);
            assert(any(obj.State==["PREPARED_PAUSED","FLIGHT"]), ...
                'gpenmpcNative:CoordinatorFlightLifecycle', ...
                'LAND/ground cannot re-enter flight without an explicit next leg.');
            poll=obj.pollWorker(nowNs);
            if obj.State=="PREPARED_PAUSED"&&~isempty(obj.Startup)
                ready=obj.startupReceipt();
                assert(ready.ready&&sample.original_sample_hrt_us>obj.Startup.source_board_us ...
                    &&sample.original_sample_hrt_us<=obj.Startup.expiry_board_us, ...
                    'gpenmpcNative:CoordinatorStartupNotFresh', ...
                    'First flight requires a committed startup outer and a NEW source within its original HOST/BOARD deadlines.');
            end
            [next,c]=obj.clockStep(obj.Clock, ...
                obj.clockEvent(sample,true,false));
            if ~c.accepted,obj.fail("CLOCK__"+string(c.reason));end
            % This is source-observation/due-slot accounting, NOT a control
            % commit. A subsequent runtime rejection retains the observed
            % slot and latches Failed; it is counted as rejected, never
            % hidden by rolling the clock back or retried as another slot.
            obj.Clock=next;obj.State="FLIGHT";obj.InputEvents=obj.InputEvents+uint64(1);
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            submit=struct('status','NOT_DUE','hardware_actions',0);
            if c.dispatch_solver&&~isempty(obj.Startup)&&~obj.Startup.slot_consumed
                % Consume the phase-zero slot before the first inner step.
                % The clock schedules the next 0.30 s update.
                assert(c.dispatch_slot_index==0&&obj.Startup.committed);
                obj.Startup.slot_consumed=true;
                submit=struct('status','STARTUP_OUTER_ALREADY_COMMITTED', ...
                    'generation',obj.Startup.generation,'hardware_actions',0);
            elseif c.dispatch_solver
                obj.SubmitAttempts=obj.SubmitAttempts+uint64(1);
                before=obj.Service.status();
                try
                    runtime=obj.observedRuntime(nowNs,sample,rotor,wind);
                    g=uint64(before.last_generation)+uint64(1);
                    submit=obj.Service.submitUpdate(g,nowNs,sample,runtime);
                    after=obj.Service.status();
                    obj.ActualSolverSubmissions=uint64(after.async_submitted);
                    if after.async_submitted==before.async_submitted
                        obj.RejectedDispatches=obj.RejectedDispatches+uint64(1);
                    end
                catch ex
                    after=obj.Service.status();
                    obj.ActualSolverSubmissions=uint64(after.async_submitted);
                    if after.async_submitted==before.async_submitted
                        obj.RejectedDispatches=obj.RejectedDispatches+uint64(1);
                    end
                    obj.recordException(ex);rethrow(ex)
                end
            end
            obj.checkService();
            r=obj.receipt('FLIGHT_SAMPLE');r.clock=c;r.poll=poll;r.submit=submit;
        end

        function r=beginPrepare(obj,nowNs,sample,rotor,wind)
            assert(~obj.Initialized&&~obj.Failed&&obj.State=="UNPREPARED", ...
                'gpenmpcNative:CoordinatorPrepareState','Preparation is once only.');
            obj.checkHostTime(nowNs);
            [next,c]=obj.clockStep([],obj.clockEvent(sample,false,true));
            assert(c.accepted,'gpenmpcNative:CoordinatorClockInit','%s',c.reason);
            try
                runtime=obj.observedRuntime(nowNs,sample,rotor,wind);
                r=obj.Service.beginPreparation(nowNs,sample,runtime);
                if string(r.status)~="PREPARATION_SUBMITTED"
                    obj.fail("PREPARATION__"+string(r.reason));
                end
            catch ex,obj.recordException(ex);rethrow(ex);end
            obj.Clock=next;obj.State="PREPARING";
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r.clock=c;r.final_hil_admission=false;r.hardware_actions=0;
        end

        function r=pollPreparation(obj,nowNs)
            assert(~obj.Failed&&obj.State=="PREPARING", ...
                'gpenmpcNative:CoordinatorPrepareState','No outstanding async preparation.');
            obj.checkHostTime(nowNs);before=obj.Clock;
            try
                r=obj.Service.pollPreparation(nowNs);
                if string(r.status)=="FAIL_CLOSED",obj.fail(string(r.reason));end
                if string(r.status)=="PASS_HOST_PREPARATION"
                    obj.Initialized=true;obj.State="PREPARED_PAUSED";
                end
            catch ex,obj.recordException(ex);rethrow(ex);end
            r.source_clock_unchanged=isequaln(before,obj.Clock);
            r.final_hil_admission=false;r.hardware_actions=0;
        end

        function r=observeDisarmedSample(obj,nowNs,sample,boardArmed)
            % Drain observations while the worker prepares generation 0.
            assert(~obj.Failed&&any(obj.State==["PREPARING","PREPARED_PAUSED"]) ...
                &&isequal(boardArmed,false),'gpenmpcNative:CoordinatorDisarmedObservation', ...
                'Only an observed-disarmed preparation/startup interval can be drained.');
            obj.checkHostTime(nowNs);
            [next,c]=obj.clockStep(obj.Clock,obj.clockEvent(sample,false,false));
            if ~c.accepted,obj.fail("DISARMED_CLOCK__"+string(c.reason));end
            assert(~c.dispatch_solver&&c.leg_flight_time_s==0);
            obj.Clock=next;obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r=obj.receipt('DISARMED_OBSERVATION_NO_SOLVE');r.clock=c;
            r.physical_samples_begun=0;r.physical_commits=0;r.packages_created=0;
        end

        function r=disarmedStartupSample(obj,nowNs,sample,rotor,wind,boardArmed)
            % Solve once at phase zero per leg after preparation warmup.
            % Require the caller's disarmed observation.
            obj.requireReady();obj.checkHostTime(nowNs);
            assert(obj.State=="PREPARED_PAUSED"&&isempty(obj.Startup) ...
                &&isequal(boardArmed,false), ...
                'gpenmpcNative:CoordinatorStartupState', ...
                'Startup requires observed disarm, a paused leg and no previous startup submission.');
            assert(sample.sample_timestamp_ns>obj.LastSourceTimestampNs, ...
                'gpenmpcNative:CoordinatorStartupSource','Preparation source cannot be reused.');
            maxAge=obj.Expected.rfly_board_commit.outer_max_age_us;
            assert(isa(maxAge,'uint64')&&isscalar(maxAge)&&maxAge>0 ...
                &&maxAge<=idivide(intmax('uint64'),uint64(1000)) ...
                &&isa(sample.source_host_receive_ns,'uint64') ...
                &&sample.source_host_receive_ns<=nowNs ...
                &&sample.source_host_receive_ns<=intmax('uint64')-maxAge*uint64(1000) ...
                &&isa(sample.original_sample_hrt_us,'uint64') ...
                &&sample.original_sample_hrt_us<=intmax('uint64')-maxAge, ...
                'gpenmpcNative:CoordinatorStartupOriginalTime','Exact original startup receipts and bounded expiry required.');
            [next,c]=obj.clockStep(obj.Clock,obj.clockEvent(sample,false,false));
            if ~c.accepted,obj.fail("STARTUP_CLOCK__"+string(c.reason));end
            assert(~c.dispatch_solver&&c.leg_flight_time_s==0);
            before=obj.Service.status();obj.SubmitAttempts=obj.SubmitAttempts+uint64(1);
            try
                runtime=obj.observedRuntime(nowNs,sample,rotor,wind);
                g=uint64(before.last_generation)+uint64(1);
                submit=obj.Service.submitUpdate(g,nowNs,sample,runtime);
                after=obj.Service.status();obj.ActualSolverSubmissions=uint64(after.async_submitted);
                assert(string(submit.status)=="SUBMITTED", ...
                    'gpenmpcNative:CoordinatorStartupSubmit','Actual async startup submission required.');
            catch ex
                after=obj.Service.status();obj.ActualSolverSubmissions=uint64(after.async_submitted);
                if after.async_submitted==before.async_submitted
                    obj.RejectedDispatches=obj.RejectedDispatches+uint64(1);
                end
                obj.recordException(ex);rethrow(ex)
            end
            obj.Startup=struct('generation',g,'source_host_ns',sample.source_host_receive_ns, ...
                'source_board_us',sample.original_sample_hrt_us, ...
                'expiry_host_ns',sample.source_host_receive_ns+maxAge*uint64(1000), ...
                'expiry_board_us',sample.original_sample_hrt_us+maxAge, ...
                'committed',false,'slot_consumed',false);
            obj.Clock=next;obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            obj.checkService();r=obj.receipt('DISARMED_STARTUP_SUBMITTED');r.clock=c;r.submit=submit;
            r.physical_samples_begun=0;r.physical_commits=0;r.packages_created=0;
        end

        function r=poll(obj,nowNs)
            obj.requireReady();obj.checkHostTime(nowNs);
            before=obj.Clock;
            response=obj.pollWorker(nowNs);obj.checkService();
            r=obj.receipt('SOLVER_POLL');r.poll=response;
            r.source_clock_unchanged=isequaln(before,obj.Clock);
        end

        function r=enterNativeLand(obj,nowNs,sample)
            obj.requireReady();obj.checkHostTime(nowNs);
            assert(obj.State=="FLIGHT",'gpenmpcNative:CoordinatorLandState', ...
                'Native LAND starts only after the active flight segment.');
            % Validate the clock before either owner changes state. The
            % service refuses to abandon an uncommitted physical sample.
            [next,c]=obj.clockStep(obj.Clock, ...
                obj.clockEvent(sample,false,false));
            if ~c.accepted,obj.fail("LAND_CLOCK__"+string(c.reason));end
            suspension=obj.Service.suspendOuterForLifecycle('NATIVE_LAND',nowNs);
            obj.Clock=next;obj.State="NATIVE_LAND";
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r=obj.receipt('OUTER_SUSPENDED_FOR_NATIVE_LAND');
            r.clock=c;r.suspension=suspension;r.land_command_sent=false;
        end

        function r=groundSample(obj,nowNs,sample,groundConfirmed,boardArmed)
            obj.requireReady();obj.checkHostTime(nowNs);
            assert(any(obj.State==["NATIVE_LAND","GROUND"]), ...
                'gpenmpcNative:CoordinatorGroundState','No airborne ground transition.');
            assert(isequal(groundConfirmed,true)&&isequal(boardArmed,false), ...
                'gpenmpcNative:CoordinatorGroundAuthority', ...
                'Ground service requires separately verified ground and disarm.');
            poll=obj.pollWorker(nowNs);
            [next,c]=obj.clockStep(obj.Clock, ...
                obj.clockEvent(sample,false,false));
            if ~c.accepted,obj.fail("GROUND_CLOCK__"+string(c.reason));end
            obj.Clock=next;obj.State="GROUND";obj.checkService();
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r=obj.receipt('GROUND_FLIGHT_CLOCK_PAUSED');r.clock=c;r.poll=poll;
            r.payload_write_sent=false;
        end

        function r=beginNextLeg(obj,nowNs,sample,trajectory,payloadKg, ...
                groundConfirmed,boardArmed)
            obj.requireReady();obj.checkHostTime(nowNs);
            assert(obj.State=="GROUND",'gpenmpcNative:CoordinatorNextLegState', ...
                'The next leg requires completed ground service.');
            s=obj.Service.status();e=obj.clockEvent(sample,false,true);
            e.leg_id=s.leg_index+1-obj.ClockLegOffset;
            [next,c]=obj.clockStep(obj.Clock,e);
            if ~c.accepted,obj.fail("NEXT_LEG_CLOCK__"+string(c.reason));end
            leg=obj.Service.beginLeg(s.leg_index+1,trajectory,payloadKg, ...
                groundConfirmed,boardArmed,nowNs,sample);
            obj.Clock=next;obj.State="PREPARED_PAUSED";
            obj.Startup=[];
            obj.LastSourceTimestampNs=uint64(sample.sample_timestamp_ns);
            r=obj.receipt('NEXT_LEG_REQUIRES_FRESH_SOLVE');r.clock=c;r.leg=leg;
        end

        function r=status(obj)
            r=obj.receipt('STATUS');
        end

        function s=controlService(obj)
            % Return the existing service handle for control begin/commit and causal updates.
            s=obj.Service;
        end

        function close(obj)
            if ~isempty(obj.Service),obj.Service.close();end
        end
        function delete(obj),obj.close();end
    end
    methods (Access=private)
        function runtime=observedRuntime(obj,nowNs,sample,rotor,wind)
            assert(isstruct(sample)&&isfield(sample,'position_generation'), ...
                'gpenmpcNative:CoordinatorSample','Missing observed EKF sample.');
            runtime=obj.Service.observedOuterRuntime( ...
                sample.position_generation,nowNs,rotor,wind);
        end
        function e=clockEvent(obj,sample,active,reset)
            assert(isstruct(sample)&&isscalar(sample)&&all(isfield(sample, ...
                {'sample_timestamp_ns','boot_generation','uid'})) ...
                &&isnumeric(sample.sample_timestamp_ns)&&isscalar(sample.sample_timestamp_ns) ...
                &&isfinite(sample.sample_timestamp_ns)&&sample.sample_timestamp_ns>=1 ...
                &&sample.sample_timestamp_ns==fix(sample.sample_timestamp_ns) ...
                &&isequal(sample.boot_generation,obj.Expected.boot_generation) ...
                &&string(sample.uid)==string(obj.Expected.uid), ...
                'gpenmpcNative:CoordinatorSourceClock','Source time or boot/UID mismatch.');
            s=obj.Service.status();
            e=struct('time_s',double(sample.sample_timestamp_ns)*1e-9, ...
                'leg_id',s.leg_index-obj.ClockLegOffset,'flight_active',logical(active), ...
                'reset_leg',logical(reset),'solver_in_flight',s.async_in_flight, ...
                'phase_rate',s.phase_rate);
            if obj.measuredSourceClock()
                assert(isa(sample.sample_timestamp_ns,'uint64') ...
                    &&isfield(sample,'actual_source_delta_us') ...
                    &&isa(sample.actual_source_delta_us,'uint64') ...
                    &&isscalar(sample.actual_source_delta_us), ...
                    'gpenmpcNative:CoordinatorMeasuredSource', ...
                    'Private-source scheduling requires the original uint64 sample and measured delta.');
                assert(isfield(sample,'source') ...
                    &&string(sample.source)=="PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1" ...
                    &&all(isfield(sample,{'source_export_bytes','source_host_receive_ns', ...
                        'original_sample_hrt_us'})), ...
                    'gpenmpcNative:CoordinatorMeasuredSource', ...
                    'Private scheduling cannot substitute another stream or a caller-rewritten clock.');
                original=gpenmpcNative.RflySnapshotSample(sample.source_export_bytes, ...
                    sample.source_host_receive_ns,obj.Expected);
                clockFields={'sample_timestamp_ns','actual_source_delta_us', ...
                    'original_sample_hrt_us','uid','boot_generation'};
                for k=1:numel(clockFields)
                    assert(isequaln(sample.(clockFields{k}),original.(clockFields{k})), ...
                        'gpenmpcNative:CoordinatorMeasuredSource', ...
                        'Source clock must match the original checksum-verified RSP bytes.');
                end
                e=rmfield(e,'time_s');
                e.sample_timestamp_ns=sample.sample_timestamp_ns;
                e.actual_source_delta_us=sample.actual_source_delta_us;
            end
        end
        function yes=measuredSourceClock(obj)
            yes=isfield(obj.Expected,'full_inner_evidence_scope') ...
                &&string(obj.Expected.full_inner_evidence_scope)=="BOARD_COMMIT_RFC1";
        end
        function [next,c]=clockStep(obj,state,event)
            % Use measured RSP intervals rather than an assumed 10 ms grid.
            if obj.measuredSourceClock()
                [next,c]=gpenmpcNative.canonicalMeasuredSourceOuterClock(state,event);
            else
                [next,c]=gpenmpcNative.canonicalOuterClock(state,event);
            end
        end
        function r=pollWorker(obj,nowNs)
            obj.PollCount=obj.PollCount+uint64(1);r=obj.Service.pollUpdate(nowNs);
            if ~isempty(obj.Startup)&&isfield(r,'generation') ...
                    &&uint64(r.generation)==obj.Startup.generation ...
                    &&string(r.status)=="FRESH_RESULT_COMMITTED"
                s=obj.Service.status();obj.Startup.committed=s.fresh_outer_for_current_leg;
            end
        end
        function r=startupReceipt(obj)
            r=struct('requested',~isempty(obj.Startup),'committed',false,'ready',false, ...
                'original_source_host_receive_ns',uint64(0),'original_source_board_us',uint64(0), ...
                'original_outer_expiry_host_ns',uint64(0),'original_outer_expiry_board_us',uint64(0), ...
                'generation',uint64(0),'first_flight_slot_consumed',false, ...
                'readiness_observed_host_ns',obj.LastHostNs,'arm_admission',false,'HOST_HRT_mapping',false);
            if isempty(obj.Startup),return;end
            a=obj.Startup;r.committed=a.committed;r.generation=a.generation;
            r.original_source_host_receive_ns=a.source_host_ns;r.original_source_board_us=a.source_board_us;
            r.original_outer_expiry_host_ns=a.expiry_host_ns;r.original_outer_expiry_board_us=a.expiry_board_us;
            r.first_flight_slot_consumed=a.slot_consumed;
            r.ready=a.committed&&~obj.Failed&&obj.State=="PREPARED_PAUSED" ...
                &&obj.LastHostNs<=a.expiry_host_ns;
        end
        function requireReady(obj)
            assert(obj.Initialized&&~obj.Failed, ...
                'gpenmpcNative:CoordinatorNotReady','Coordinator is unprepared or failed.');
        end
        function checkHostTime(obj,nowNs)
            assert(isnumeric(nowNs)&&isscalar(nowNs)&&isfinite(nowNs) ...
                &&nowNs>=double(obj.LastHostNs)&&nowNs==fix(nowNs), ...
                'gpenmpcNative:CoordinatorHostTime','Host monotonic time reversed or invalid.');
            obj.LastHostNs=uint64(nowNs);
        end
        function checkService(obj)
            s=obj.Service.status();if s.failed,obj.fail("SERVICE__"+string(s.failure_code));end
        end
        function fail(obj,reason)
            obj.Failed=true;obj.FailureCode=reason;
            error('gpenmpcNative:CoordinatorFailClosed','%s',reason);
        end
        function recordException(obj,ex)
            if ~obj.Failed
                obj.Failed=true;obj.FailureCode="IMPLEMENTATION__"+string(ex.identifier);
            end
        end
        function r=receipt(obj,kind)
            s=obj.Service.status();
            r=struct('schema','CANONICAL_SINGLE_OWNER_OUTER_COORDINATOR_V1', ...
                'event',string(kind),'state',obj.State,'failed',obj.Failed, ...
                'failure_code',obj.FailureCode,'initialized',obj.Initialized, ...
                'input_events',double(obj.InputEvents),'poll_count',double(obj.PollCount), ...
                'submit_attempts',double(obj.SubmitAttempts), ...
                'actual_solver_submissions',double(obj.ActualSolverSubmissions), ...
                'rejected_dispatches',double(obj.RejectedDispatches), ...
                'source_clock',obj.Clock,'service',s, ...
                'source_clock_is_observation_accounting_not_control_commit',true, ...
                'clock_leg_semantics','SESSION_LOCAL_SEQUENTIAL_INDEX', ...
                'clock_global_task_leg_offset',obj.ClockLegOffset,'global_task_leg_index',s.leg_index, ...
                'startup',obj.startupReceipt(), ...
                'owned_outer_services',1,'owned_plants',0,'owned_io_endpoints',0, ...
                'final_hil_admission',false,'logical_arm_authority',false, ...
                'control_publication_authority',false,'hardware_actions',0);
        end
    end
end
