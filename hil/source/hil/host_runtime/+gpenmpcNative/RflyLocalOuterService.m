classdef RflyLocalOuterService < handle
    % Own asynchronous eNMPC solving, continuity and measured-source scheduling.
    properties (SetAccess=private)
        Failed=false
        Closed=false
        Failure=""
        CallsSubmitted=uint64(0)
        ResultsCompleted=uint64(0)
        FreshResults=uint64(0)
        DeadlineDiscards=uint64(0)
        LastCommand=[]
        LastEvent=struct()
        Prepared=false
        PreparationCalls=uint64(0)
        PreparationReceipt=struct()
        Preconstructed=false
        SessionBound=false
        SessionBindings=uint64(0)
        SubmissionSuspended=false
        WorkerConstruction=struct()
        WorkerCloseReceipt=struct()
    end
    properties (Access=private)
        Assets
        Registered
        Trajectory
        Fixed
        Worker
        Adapter
        Warm
        Clock=[]
        Pending=[]
        LastNs=uint64(0)
        MaximumObservationAgeNs
        FlightObservationAgeNs
        CommandLifetimeNs
        BootstrapSubmitted=false
        BootstrapSlotConsumed=false
        Preparation=[]
        LatestObservation=[]
    end
    methods
        function obj=RflyLocalOuterService(assets,registered,trajectory,observationAgeNs,commandLifetimeNs,construction)
            if nargin<6,construction="BOUND";end
            assert(isscalar(string(construction))&&any(string(construction)==["BOUND","PREPARE_UNBOUND"]), ...
                'gpenmpcNative:LocalOuterConstruction','Explicit bound or unbound construction is required.');
            obj.Preconstructed=string(construction)=="PREPARE_UNBOUND";
            if obj.Preconstructed
                assert(isempty(registered),'gpenmpcNative:LocalOuterUnbound','Preconstruction cannot contain registration authority.');
            else
                validateRegistration(registered);obj.SessionBound=true;obj.SessionBindings=uint64(1);
            end
            assert(isa(observationAgeNs,'uint64')&&isscalar(observationAgeNs)&&observationAgeNs>0 ...
                &&isa(commandLifetimeNs,'uint64')&&isscalar(commandLifetimeNs) ...
                &&commandLifetimeNs>=uint64(280000000)&&commandLifetimeNs<=uint64(700000000));
            assert(assets.binding.verified_source_entries==160 ...
                &&strcmpi(assets.binding.effective_configuration_payload_sha256, ...
                    'A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA'));
            obj.Assets=assets;obj.Registered=registered;obj.Trajectory=trajectory;
            obj.MaximumObservationAgeNs=observationAgeNs;obj.CommandLifetimeNs=commandLifetimeNs;
            % Reserve the .280 s solve deadline within the outer command lifetime.
            % Cap matched-observation age at .320 s even when transport lifetime is .700 s.
            % Transport and result latency can still expire the command.
            solverBudgetNs=uint64(round(assets.enmpc.solver_deadline_s*1e9));
            assert(commandLifetimeNs>=solverBudgetNs,'gpenmpcNative:LocalOuterLatencyBudget');
            obj.FlightObservationAgeNs=max(observationAgeNs,min(uint64(320000000),commandLifetimeNs-solverBudgetNs));
            obj.Adapter=gpenmpcNative.initializeBoardReferenceAdapter(assets.enmpc,trajectory,string(assets.enmpc.method));
            obj.Warm=zeros(1,assets.enmpc.decision_dimension);
            fixed=outerFixed(assets,trajectory);constructionTimer=tic;
            obj.Fixed=parallel.pool.Constant(fixed);
            obj.Worker=gpenmpcNative.OuterMailboxWorker(obj.Fixed,assets.enmpc.solver_deadline_s,double(obj.FlightObservationAgeNs)*1e-9);
            obj.WorkerConstruction=struct('elapsed_s',toc(constructionTimer), ...
                'scope','HOST_CONSTRUCTION_TIMING','worker_constructions',1, ...
                'worker_source',which('gpenmpcNative.OuterMailboxWorker'), ...
                'worker_loop_source',which('gpenmpcNative.outerMailboxLoop'), ...
                'source_root',fixed.source_root,'source_binding_sha256',fixed.source_binding_sha256, ...
                'configuration_payload_sha256',fixed.configuration_payload_sha256, ...
                'gp_model_sha256',fixed.gp_model_sha256,'solver_deadline_s',assets.enmpc.solver_deadline_s, ...
                'observation_admission_ns',obj.FlightObservationAgeNs, ...
                'source_to_result_max_ns',obj.FlightObservationAgeNs+solverBudgetNs, ...
                'solve_submissions',obj.Worker.Submitted,'session_bound_at_construction',obj.SessionBound);
        end
        function bindPreparedSession(obj,assets,registered,trajectory,observationAgeNs,commandLifetimeNs)
            % Attach the registered session to the existing worker.
            assert(obj.Preconstructed&&~obj.SessionBound&&~obj.Closed&&~obj.Failed&&~obj.SubmissionSuspended ...
                &&obj.SessionBindings==0&&obj.Worker.Submitted==0&&~obj.Worker.isBusy(), ...
                'gpenmpcNative:LocalOuterPreparedOwner','Only the unused, live preconstructed owner can bind once.');
            assert(isequaln(outerFixed(assets,trajectory),obj.Fixed.Value)&&isequaln(trajectory,obj.Trajectory) ...
                &&isequal(observationAgeNs,obj.MaximumObservationAgeNs)&&isequal(commandLifetimeNs,obj.CommandLifetimeNs), ...
                'gpenmpcNative:LocalOuterPreparedBinding','Original fixed source/configuration, trajectory and deadlines must match exactly.');
            validateRegistration(registered);
            obj.Registered=registered;obj.SessionBound=true;obj.SessionBindings=uint64(1);
        end
        function r=beginPreparation(obj,source,inputs,nowNs)
            % Warm generation zero on the persistent worker without installing its result.
            obj.checkTime(nowNs);
            assert(~obj.BootstrapSubmitted&&~obj.Prepared&&obj.PreparationCalls==0 ...
                &&isempty(obj.Preparation)&&isempty(obj.Pending)&&~obj.Worker.isBusy(), ...
                'gpenmpcNative:LocalOuterPreparation');
            [snapshot,binding]=gpenmpcNative.makeLocalCommittedOuterSnapshot(obj.Assets,obj.Trajectory,obj.Warm, ...
                source,[],inputs,obj.Registered,nowNs,obj.MaximumObservationAgeNs,true);
            assert(obj.Worker.submit(obj.compact(snapshot),0,double(nowNs)), ...
                'gpenmpcNative:LocalOuterPreparationSubmit');
            obj.PreparationCalls=obj.PreparationCalls+uint64(1);
            obj.Preparation=struct('submitted_ns',nowNs,'snapshot_ns',uint64(snapshot.snapshot_timestamp_ns), ...
                'binding',binding,'warm_start',obj.Warm,'adapter',obj.Adapter);
            r=struct('status','PREPARATION_SUBMITTED','generation',0,'command_committed',false);
            obj.PreparationReceipt=r;
        end
        function r=pollPreparation(obj,nowNs)
            obj.checkTime(nowNs);
            assert(~isempty(obj.Preparation)&&~obj.Prepared&&~obj.BootstrapSubmitted, ...
                'gpenmpcNative:LocalOuterPreparation');
            [fresh,result,reason]=obj.Worker.poll(double(nowNs),0);
            wallS=double(nowNs-obj.Preparation.submitted_ns)*1e-9;
            r=struct('status','PREPARATION_PENDING','worker_status',reason,'wall_s',wallS, ...
                'generation',0,'command_committed',false,'solver_deadline_s',obj.Assets.enmpc.solver_deadline_s);
            if wallS>=60||strcmp(reason,'WORKER_ERROR_NO_COMMAND')
                obj.Failed=true;obj.Failure="LOCAL_OUTER_PREPARATION__"+string(reason);
                if wallS>=60,obj.Failure="PREPARATION_COLD_LIMIT_EXPIRED";end
                r.status='FAIL_CLOSED';obj.PreparationReceipt=r;return
            end
            late=strcmp(reason,'LATE_RESULT_DISCARDED');
            if ~fresh&&~late
                assert(any(strcmp(reason,{'PENDING','EXPIRED_STILL_BUSY_NO_REQUEUE'})), ...
                    'gpenmpcNative:LocalOuterPreparationDisposition');
                obj.PreparationReceipt=r;return
            end
            if late,result=obj.Worker.LastCompletedDiagnostic;end
            assert(result.used_current_supervisor&&result.dynamic_snapshot_only&&result.immutable_worker_constant ...
                &&uint64(result.snapshot_timestamp_ns)==obj.Preparation.snapshot_ns ...
                &&strcmpi(result.configuration_payload_sha256,obj.Assets.binding.effective_configuration_payload_sha256), ...
                'gpenmpcNative:LocalOuterPreparationBinding');
            % Warm command-continuity conversion on a temporary preparation-result copy.
            gpenmpcNative.commitBoardReferenceDecision(obj.Adapter,result.decision,result.audit,uint64(1),obj.Assets.enmpc);
            assert(isequaln(obj.Preparation.warm_start,obj.Warm)&&isequaln(obj.Preparation.adapter,obj.Adapter) ...
                &&isempty(obj.Clock)&&isempty(obj.LastCommand)&&obj.CallsSubmitted==0, ...
                'gpenmpcNative:LocalOuterPreparationMutation');
            r.status='PASS_HOST_PREPARATION';r.expired_diagnostic_only=late;r.numerical_state_unchanged=true;
            r.decision=result.decision;r.audit=result.audit;obj.Prepared=true;obj.Preparation=[];
            obj.PreparationReceipt=r;
        end
        function r=bootstrap(obj,source,inputs,nowNs,activeSchedule)
            % Allow omitted RLC only for explicit new-leg initialization.
            if nargin<5,activeSchedule=false;end
            assert(islogical(activeSchedule)&&isscalar(activeSchedule));
            obj.checkTime(nowNs);
            assert(~obj.BootstrapSubmitted&&isempty(obj.Clock)&&isempty(obj.Pending)&&isempty(obj.Preparation), ...
                'gpenmpcNative:LocalOuterBootstrap','Bootstrap cannot reset a running leg.');
            s=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
            event=obj.clockEvent(s,false,true,false,1);
            [next,r]=gpenmpcNative.canonicalMeasuredSourceOuterClock([],event);
            assert(r.accepted,'gpenmpcNative:LocalOuterClock','%s',r.reason);
            if activeSchedule
                % The actually armed runtime dispatches its original solver
                % here. Its next .30-s slot starts at THIS source, not at a
                % later exported control record. No synthetic RLC/state is
                % used: subsequent solves still require actual committed data.
                event=obj.clockEvent(s,true,false,false,1);
                [next,start]=gpenmpcNative.canonicalMeasuredSourceOuterClock(next,event);
                assert(start.accepted&&start.dispatch_solver&&start.dispatch_slot_index==0, ...
                    'gpenmpcNative:LocalOuterBootstrapClock');
                obj.BootstrapSlotConsumed=true;
                r.active_schedule=start;
            end
            obj.Clock=next;obj.BootstrapSubmitted=true;
            obj.submit(source,[],inputs,nowNs,true);r.outer_submitted=true;
        end
        function r=sample(obj,source,committed,inputs,flightActive,nowNs,deferClock)
            if nargin<7,deferClock=false;end
            assert(islogical(deferClock)&&isscalar(deferClock));
            obj.checkTime(nowNs);
            % Poll the worker before fast receive; avoid consuming a newly completed
            % result again during post-input history processing.
            if ~deferClock,obj.poll(nowNs);end
            assert(~obj.Failed,'gpenmpcNative:LocalOuterClosed','Hard-invalid result cannot continue source processing.');
            assert(obj.BootstrapSubmitted&&~isempty(committed)&&islogical(flightActive)&&isscalar(flightActive), ...
                'gpenmpcNative:LocalOuterLifecycle','Flight samples require actual committed local state.');
            assert(nowNs>=source.original_host_receive_ns&&nowNs>=committed.original_host_receive_ns, ...
                'gpenmpcNative:LocalOuterSourceAge','Original observations cannot be in the future.');
            if nowNs-source.original_host_receive_ns>obj.FlightObservationAgeNs ...
                    ||nowNs-committed.original_host_receive_ns>obj.FlightObservationAgeNs
                % History may finish arriving after its numerical lifetime.
                % Preserve the actual commit in MethodService, but do not
                % advance the outer clock, dispatch a solve or renew a
                % command using it. The board's existing expiry still acts.
                r=struct('status','HISTORICAL_COMMIT_NOT_USED_BY_OUTER', ...
                    'outer_submitted',false,'freshness_renewed',false, ...
                    'source_age_ns',nowNs-source.original_host_receive_ns, ...
                    'commit_receive_age_ns',nowNs-committed.original_host_receive_ns);
                obj.LastEvent=r;return
            end
            % BEGIN_OUTER_RETAINED_SOURCE
            % Reuse the IO owner's validated decode without replacing the fast-source cache.
            if isfield(source,'decoded')&&~isempty(source.decoded)
                s=source.decoded;
                assert(isequal(s.original_bytes,source.message(:)) ...
                    &&isequal(s.original_host_receive_ns,source.original_host_receive_ns), ...
                    'gpenmpcNative:LocalOuterRetainedSource','Retained source lost its original bytes/time.');
            else
                s=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
            end
            % END_OUTER_RETAINED_SOURCE
            % BEGIN_OUTER_RETAINED_COMMIT
            if isfield(committed,'decoded')&&~isempty(committed.decoded)
                c=committed.decoded;
                assert(isequal(c.original_bytes,committed.message(:)) ...
                    &&isequal(c.original_host_receive_ns,committed.original_host_receive_ns), ...
                    'gpenmpcNative:LocalOuterRetainedCommit','Retained commit lost its original bytes/time.');
            else
                c=gpenmpcNative.RflyLocalCommittedDecoder(committed.message,committed.original_host_receive_ns);
            end
            % END_OUTER_RETAINED_COMMIT
            % Check each observation's source and installed-state association.
            % Build the solver input only when the .30 s clock dispatches.
            assert(isequal(s.identity,obj.Registered.identity)&&isequal(c.identity,s.identity) ...
                &&c.source_generation==s.source_generation&&c.source_timestamp_ns==s.original_sample_us*uint64(1000) ...
                &&c.leg_index==obj.Registered.leg_index&&c.numeric_installed==1 ...
                &&strcmpi(upper(reshape(dec2hex(c.configuration_sha256,2).',1,[])),obj.Assets.configurationBinding.effective_configuration_payload_sha256) ...
                &&strcmpi(upper(reshape(dec2hex(c.reference_asset_sha256,2).',1,[])),obj.Registered.reference_asset_sha256) ...
                &&c.installed_phase2(1)>=0&&c.installed_phase2(1)<=obj.Trajectory.total_duration_s ...
                &&c.installed_phase2(2)>=obj.Assets.enmpc.phase_rate_min ...
                &&c.installed_phase2(2)<=obj.Assets.enmpc.phase_rate_max, ...
                'gpenmpcNative:LocalOuterCommitBinding','Original matched installed source/phase required even between solver slots.');
            if deferClock
                % Schedule from fresh RLS independently of the retained RLC observation and its timestamp.
                if ~isempty(obj.LatestObservation)
                    assert(s.original_sample_us>obj.LatestObservation.sample_us, ...
                        'gpenmpcNative:LocalOuterObservationOrder');
                end
                obj.LatestObservation=struct('source',source,'committed',committed,'inputs',inputs, ...
                    'sample_us',s.original_sample_us,'generation',s.source_generation,'phase_rate',c.installed_phase2(2));
                r=struct('status','MATCHED_OBSERVATION_RETAINED','outer_submitted',false, ...
                    'source_generation',s.source_generation,'freshness_renewed',false);
                obj.LastEvent=r;return
            end
            r=obj.advanceClock(s,flightActive,c.installed_phase2(2),source,committed,inputs,nowNs);
        end
        function r=serviceSourceClock(obj,source,flightActive,nowNs)
            obj.checkTime(nowNs);
            % Use the worker state from the pre-receive poll.
            assert(obj.BootstrapSubmitted&&islogical(flightActive)&&isscalar(flightActive));
            % BEGIN_OUTER_RETAINED_SOURCE
            % Reuse the IO owner's validated decode without replacing the fast-source cache.
            if isfield(source,'decoded')&&~isempty(source.decoded)
                s=source.decoded;
                assert(isequal(s.original_bytes,source.message(:)) ...
                    &&isequal(s.original_host_receive_ns,source.original_host_receive_ns), ...
                    'gpenmpcNative:LocalOuterRetainedSource','Retained source lost its original bytes/time.');
            else
                s=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
            end
            % END_OUTER_RETAINED_SOURCE
            assert(isequal(s.identity,obj.Registered.identity)&&nowNs>=source.original_host_receive_ns, ...
                'gpenmpcNative:LocalOuterClockSource','Actual same-session, nonfuture RLS required.');
            r=struct('status','NO_VALID_MATCHED_OBSERVATION','outer_submitted',false);
            if nowNs-source.original_host_receive_ns>obj.MaximumObservationAgeNs||isempty(obj.LatestObservation)
                obj.LastEvent=r;return
            end
            p=obj.LatestObservation;
            assert(p.sample_us<=s.original_sample_us&&nowNs>=p.source.original_host_receive_ns ...
                &&nowNs>=p.committed.original_host_receive_ns,'gpenmpcNative:LocalOuterClockOrder');
            % Enforce .32 s observation admission and .28 s solve/return bounds separately;
            % the mailbox also checks their combined source-to-result age.
            sourceBudget=obj.FlightObservationAgeNs;
            if nowNs-p.source.original_host_receive_ns>sourceBudget ...
                    ||nowNs-p.committed.original_host_receive_ns>obj.FlightObservationAgeNs
                obj.LastEvent=r;return
            end
            r=obj.advanceClock(s,flightActive,p.phase_rate,p.source,p.committed,p.inputs,nowNs);
            r.schedule_source_generation=s.source_generation;
            r.numerical_source_generation=p.generation;
            r.numerical_source_age_ns=nowNs-p.source.original_host_receive_ns;
            r.freshness_renewed=false;obj.LastEvent=r;
        end
        function r=poll(obj,nowNs)
            obj.checkTime(nowNs);r=struct('status','IDLE','new_command',false);
            if isempty(obj.Pending),return;end
            [fresh,result,reason]=obj.Worker.poll(double(nowNs),double(obj.Pending.generation));
            r.status=reason;
            completed=~obj.Worker.isBusy();
            if completed&&any(strcmp(reason,{'FRESH_RESULT','LATE_RESULT_DISCARDED','STALE_GENERATION_DISCARDED'}))
                raw=obj.Worker.LastCompletedDiagnostic;
                if isfield(raw,'used_current_supervisor')&&raw.used_current_supervisor
                    obj.ResultsCompleted=obj.ResultsCompleted+uint64(1);
                end
                if isfield(raw,'source_binding')
                    binding=raw.source_binding;
                    assert(binding.source_generation==obj.Pending.source_generation ...
                        &&binding.original_host_source_receive_ns==obj.Pending.original_host_source_receive_ns ...
                        &&binding.source_timestamp_ns==obj.Pending.original_sample_us*uint64(1000), ...
                        'gpenmpcNative:LocalOuterResult','Worker observation must retain its exact original source.');
                    obj.Pending.binding=binding;
                end
            end
            if ~obj.Pending.accounted
                if fresh
                    assert(result.used_current_supervisor&&result.dynamic_snapshot_only&&result.immutable_worker_constant ...
                        &&uint64(result.snapshot_timestamp_ns)==obj.Pending.original_host_source_receive_ns ...
                        &&strcmpi(result.configuration_payload_sha256,obj.Assets.binding.effective_configuration_payload_sha256), ...
                        'gpenmpcNative:LocalOuterResult','Original single worker result binding changed.');
                    obj.Warm=result.warm_start;obj.FreshResults=obj.FreshResults+uint64(1);
                    r=obj.commit(result.decision,result.audit,nowNs,reason);obj.Pending.accounted=true;
                elseif any(strcmp(reason,{'EXPIRED_STILL_BUSY_NO_REQUEUE','LATE_RESULT_DISCARDED','WORKER_ERROR_NO_COMMAND','STALE_GENERATION_DISCARDED'}))
                    hard=~any(strcmp(reason,{'EXPIRED_STILL_BUSY_NO_REQUEUE','LATE_RESULT_DISCARDED'}));
                    decision=struct('method',string(obj.Assets.enmpc.method),'success',false,'hard_invalid',hard, ...
                        'phase_acceleration_s_inv',0,'outer_acceleration_correction_f_mps2',zeros(3,1), ...
                        'fallback_reason',string(reason),'elapsed_seconds',double(nowNs-obj.Pending.submitted_ns)*1e-9, ...
                        'gp_b1_fallback_active',false);
                    audit=struct('current_hard_invalid',hard,'coordinated_selected_method',string(obj.Assets.enmpc.method));
                    r=obj.commit(decision,audit,nowNs,reason);obj.Pending.accounted=true;
                    if ~hard,obj.DeadlineDiscards=obj.DeadlineDiscards+uint64(1);end
                end
            end
            if completed,obj.Pending=[];end
            obj.LastEvent=r;
        end
        function r=status(obj)
            r=struct('failed',obj.Failed,'failure',obj.Failure,'closed',obj.Closed,'clock',obj.Clock, ...
                'outer_submissions',obj.CallsSubmitted,'completed_original_solver_results',obj.ResultsCompleted, ...
                'fresh_results',obj.FreshResults,'deadline_discards',obj.DeadlineDiscards, ...
                'last_command',obj.LastCommand,'last_event',obj.LastEvent,'host_inner_steps',0,'plant_steps',0, ...
                'outer_owner_count',1,'hardware_actions',0);
            r.prepared=obj.Prepared;r.preparation_calls=obj.PreparationCalls;r.preparation=obj.PreparationReceipt;
            r.bootstrap_observation_age_ns=obj.MaximumObservationAgeNs;
            r.flight_observation_age_ns=obj.FlightObservationAgeNs;
            r.preconstructed=obj.Preconstructed;r.session_bound=obj.SessionBound;r.session_bindings=obj.SessionBindings;
            r.submission_suspended=obj.SubmissionSuspended;
            r.shutdown_join_deferred=obj.Preconstructed&&obj.SubmissionSuspended&&~obj.Closed;
            r.worker_construction=obj.WorkerConstruction;r.worker_close=obj.WorkerCloseReceipt;
            if ~isempty(obj.Worker)
                r.worker=struct('in_flight',obj.Worker.isBusy(),'maximum_in_flight',obj.Worker.MaximumInFlight, ...
                    'errors',obj.Worker.WorkerErrors,'last_error',obj.Worker.LastErrorDiagnostic, ...
                    'submitted',obj.Worker.Submitted);
            end
        end
        function suspendForNativeLand(obj)
            % Borrowed owner only: stop all future HOST effects without
            % waiting for the original background worker. Preserve its raw
            % pending/result state; the outer caller owns the later join.
            assert(obj.Preconstructed,'gpenmpcNative:LocalOuterSuspension','Only the explicitly borrowed preconstructed owner defers its join.');
            obj.SubmissionSuspended=true;
        end
        function close(obj)
            if obj.Closed,return;end
            obj.Closed=true;
            if ~isempty(obj.Worker)
                obj.Worker.close();
                obj.WorkerCloseReceipt=struct('closed',true,'clean_stop',obj.Worker.StopWasClean, ...
                    'submitted',obj.Worker.Submitted,'errors',obj.Worker.WorkerErrors);
            end
            if ~isempty(obj.Fixed),delete(obj.Fixed);end
            obj.Worker=[];obj.Fixed=[];
        end
        function delete(obj),obj.close();end
    end
    methods (Access=private)
        function r=advanceClock(obj,s,flightActive,rate,source,committed,inputs,nowNs)
            event=obj.clockEvent(s,flightActive,false,obj.Worker.isBusy(),rate);
            if flightActive&&~obj.BootstrapSlotConsumed
                assert(~isempty(obj.LastCommand),'gpenmpcNative:LocalOuterBootstrapSlot');
                event.solver_in_flight=false;
            end
            [next,r]=gpenmpcNative.canonicalMeasuredSourceOuterClock(obj.Clock,event);
            assert(r.accepted,'gpenmpcNative:LocalOuterClock','%s',r.reason);obj.Clock=next;
            if r.dispatch_solver
                if ~obj.BootstrapSlotConsumed
                    assert(r.dispatch_slot_index==0&&~isempty(obj.LastCommand), ...
                        'gpenmpcNative:LocalOuterBootstrapSlot');
                    obj.BootstrapSlotConsumed=true;r.outer_submitted=false;r.bootstrap_slot_consumed=true;
                else
                    obj.submit(source,committed,inputs,nowNs,false);r.outer_submitted=true;
                end
            else,r.outer_submitted=false;end
            obj.LastEvent=r;
        end
        function checkTime(obj,t)
            assert(~obj.Closed&&~obj.Failed,'gpenmpcNative:LocalOuterClosed','No restart after close/fault.');
            assert(~obj.SubmissionSuspended,'gpenmpcNative:LocalOuterSuspended','Native safety has stopped all method processing and submissions.');
            assert(obj.SessionBound,'gpenmpcNative:LocalOuterUnbound','No solve or source processing before actual session binding.');
            assert(isa(t,'uint64')&&isscalar(t)&&t>0&&t>=obj.LastNs&&t<=uint64(flintmax), ...
                'gpenmpcNative:LocalOuterTime','Original monotonic HOST uint64 ns required without double precision loss.');
            obj.LastNs=t;
        end
        function e=clockEvent(~,s,active,reset,busy,rate)
            % Each explicitly registered board leg has one local clock; no
            % hidden cross-leg reset or HOST integration of reference phase.
            % sample_delta_us is since the board's last installed control,
            % while successive original_sample_us values drive this clock.
            % An intervening unexecuted RLS does not move the control anchor.
            e=struct('sample_timestamp_ns',s.original_sample_us*uint64(1000), ...
                'actual_source_delta_us',s.sample_delta_us,'leg_id',1,'flight_active',active, ...
                'reset_leg',reset,'solver_in_flight',busy,'phase_rate',rate, ...
                'board_local_source_bound_us',uint64(50000));
            % Use the board-local 50000 us source bound and retain measured intervals.
        end
        function submit(obj,source,committed,inputs,t,bootstrap)
            assert(isempty(obj.Pending)&&~obj.Worker.isBusy(),'gpenmpcNative:LocalOuterSingleFlight');
            age=obj.MaximumObservationAgeNs;
            if ~bootstrap,age=obj.FlightObservationAgeNs;end
            workerOriginal=~bootstrap&&isfield(source,'decoded')&&~isempty(source.decoded) ...
                &&isfield(committed,'decoded')&&~isempty(committed.decoded) ...
                &&isstruct(inputs)&&isfield(inputs,'held_input_decoded') ...
                &&isfield(inputs.original_input_source,'decoded')&&~isempty(inputs.original_input_source.decoded);
            if ~workerOriginal
                [snapshot,binding]=gpenmpcNative.makeLocalCommittedOuterSnapshot(obj.Assets,obj.Trajectory,obj.Warm, ...
                    source,committed,inputs,obj.Registered,t,age,true);
                sampleUs=idivide(binding.source_timestamp_ns,uint64(1000));sourceGeneration=binding.source_generation;
            else
                % Run the numerical adapter on the solver worker using immutable source data.
                if isfield(source,'decoded')&&~isempty(source.decoded)
                    s=source.decoded;
                else
                    s=gpenmpcNative.RflyLocalSnapshotDecoder(source.message,source.original_host_receive_ns);
                end
                assert(isequal(s.original_bytes,source.message(:)) ...
                    &&isequal(s.original_host_receive_ns,source.original_host_receive_ns) ...
                    &&t>=source.original_host_receive_ns&&t-source.original_host_receive_ns<=age, ...
                    'gpenmpcNative:LocalOuterRetainedSource','Original matched observation required.');
                snapshot=struct('snapshot_timestamp_ns',double(source.original_host_receive_ns), ...
                    'configuration_payload_sha256',obj.Assets.configurationBinding.effective_configuration_payload_sha256, ...
                    'warm_start',obj.Warm,'original_observation',struct('source',source,'committed',committed, ...
                    'inputs',inputs,'registered',obj.Registered,'dispatch_ns',t,'maximum_age_ns',age));
                sourceGeneration=s.source_generation;sampleUs=s.original_sample_us;
                binding=[]; % Validation is pending worker completion.
            end
            generation=obj.Adapter.last_committed_generation+uint64(1);
            assert(obj.Worker.submit(obj.compact(snapshot),double(generation),double(t)),'gpenmpcNative:LocalOuterSubmit');
            obj.CallsSubmitted=obj.CallsSubmitted+uint64(1);
            obj.Pending=struct('generation',generation,'source_generation',sourceGeneration, ...
                'original_sample_us',sampleUs, ...
                'original_host_source_receive_ns',source.original_host_receive_ns,'submitted_ns',t, ...
                'accounted',false,'bootstrap',bootstrap,'binding',binding);
        end
        function s=compact(obj,snapshot)
            % Keep fixed trajectory and configuration in the worker Constant.
            % Send only measured state, phase, warm-start and evidence per solve.
            s=snapshot;s.work_root='LOCAL_FULL_INNER_COMMON_IO';
            s.source_manifest_sha256=string(obj.Assets.binding.source_manifest_sha256);
            s.source_binding_sha256=string(obj.Assets.binding.passport_sha256);
            s.gp_model_sha256=string(obj.Assets.binding.gp_model_sha256);
        end
        function r=commit(obj,decision,audit,t,reason)
            [obj.Adapter,event]=gpenmpcNative.commitBoardReferenceDecision(obj.Adapter,decision,audit,obj.Pending.generation,obj.Assets.enmpc);
            p=obj.Pending;expiry=p.original_host_source_receive_ns+obj.CommandLifetimeNs;
            allowed=obj.Adapter.publication_allowed&&t<=expiry;
            if obj.Adapter.hard_invalid_latched,obj.Failed=true;obj.Failure=string(reason);obj.LastCommand=[];end
            if allowed
                obj.LastCommand=struct('generation',p.generation,'source_generation',p.source_generation, ...
                    'original_sample_us',p.original_sample_us,'original_host_source_receive_ns',p.original_host_source_receive_ns, ...
                    'original_creation_ns',t,'original_expiry_ns',expiry, ...
                    'target4',[obj.Adapter.target_phase_acceleration_s_inv;obj.Adapter.target_outer_correction_f_mps2]);
            end
            r=struct('status',reason,'new_command',allowed,'generation',p.generation,'decision',decision,'audit',audit, ...
                'continuity_event',event,'source_binding',p.binding,'original_creation_ns',t,'original_expiry_ns',expiry, ...
                'hard_invalid',obj.Adapter.hard_invalid_latched,'host_reference_phase_steps',0,'host_inner_steps',0, ...
                'hardware_actions',0);
        end
    end
end
function validateRegistration(registered)
assert(isstruct(registered)&&isscalar(registered)&&all(isfield(registered,{'local_full_inner','leg_index'})) ...
    &&isequal(registered.local_full_inner,true)&&registered.leg_index>=1&&registered.leg_index<=5, ...
    'gpenmpcNative:LocalOuterRegistration','Actual registered local leg is required.');
end
function fixed=outerFixed(assets,trajectory)
fixed=struct('source_root',assets.sourceRoot,'config',assets.enmpc,'context',assets.prediction_context,'profile',assets.profile, ...
    'gp_model',assets.gp_model,'trajectory',trajectory,'source_manifest_sha256',string(assets.binding.source_manifest_sha256), ...
    'source_binding_sha256',string(assets.binding.passport_sha256), ...
    'configuration_payload_sha256',string(assets.binding.effective_configuration_payload_sha256), ...
    'gp_model_sha256',string(assets.binding.gp_model_sha256));
end
