classdef RflyLocalMethodService < handle
    % Service outer control, GP inference and manual references over a bound IO session.
    % The caller owns heartbeat, environment updates and landing recovery.
    properties (SetAccess=private)
        Failed=false
        Failure=''
        Closed=false
        Phase
        LastEvent=struct()
        OperatorFinishRequested=false
    end
    properties (Access=private)
        Io
        Outer
        Gp
        Getter
        Environment
        Registered
        Task
        Leg
        ReferenceBinding
        Cfg
        Mode='PREPARE'
        PreparationStarted=false
        BootstrapStarted=false
        Source=[]
        ArmPrepared=[]
        Pending={}
        LastWindow=[]
        WindowTx=[]
        WindowGeneration=uint64(0)
        LastCommittedWindow=uint64(0)
        LastNow=uint64(0)
        Counts=struct('snapshots_taken',0,'inputs_sent',0,'commits',0,'gp_replies',0, ...
            'disarmed_sources_retired',0,'missing_binding_polls',0,'expired_disarmed_sources',0,'reference_windows_sent',0)
        Busy=false
        StreamLifecycle=[]
        StreamWitnessed=false(1,3)
        InputCodecBackend='MATLAB_ORIGINAL'
        InputCodecBinding=[]
        InputCodecAdapter=[]
        InputCodecMex=[]
        InputCodecCalls=uint64(0)
        ComponentInitialization=false
        NominalCommand=[]
        LastCommitSource=uint64(0)
        PendingGp=[]
    end
    methods
        function obj=RflyLocalMethodService(assets,bundle,trajectory,referenceBinding,registered,association,io,getter,environment,task,cfg)
            assert(isa(getter,'gpenmpcNative.RflyLocalOriginalGetterBuffer') ...
                &&isa(environment,'gpenmpcNative.RflyLocalEnvironmentLedger'));
            assert(registered.local_full_inner&&all(isfield(cfg,{'source_max_age_ns','command_lifetime_ns','pending_capacity'})) ...
                &&isa(cfg.source_max_age_ns,'uint64')&&cfg.source_max_age_ns>0 ...
                &&cfg.pending_capacity==fix(cfg.pending_capacity)&&cfg.pending_capacity>=4&&cfg.pending_capacity<=64);
            state=io.pollCanonical();raw=io.evidence();e=raw.canonical_exchange_expected;
            assert(state.bound&&state.board_local_full_inner&&gpenmpcNative.sameSoleMavlinkOwner(state)&&state.additional_connections==0 ...
                &&isempty(state.failure)&&isequal(e.uid,registered.identity.uid) ...
                &&isequal(e.session_generation,registered.identity.boot_generation) ...
                &&e.source_system==registered.identity.system&&e.source_component==registered.identity.component ...
                &&strcmpi(e.execution_session_sha256,registered.execution_session_sha256) ...
                &&strcmpi(e.task_sha256,registered.task_sha256)&&e.leg_index==registered.leg_index ...
                &&strcmpi(e.configuration_sha256,registered.configuration_sha256), ...
                'gpenmpcNative:LocalMethodOwner','Already-bound same IO/session required; no substitute transport.');
            % Require the registration receipt retained by the bound IO owner.
            assert(isfield(raw,'canonical_exchange_association')&&~isempty(raw.canonical_exchange_association) ...
                &&isequaln(raw.canonical_exchange_association,association), ...
                'gpenmpcNative:LocalMethodRegistration','Same-IO verified registration receipts required.');
            % Explicit, immutable construction-time opt-in only. Reject an
            % invalid binding before constructing either numerical worker.
            % The adapter preserves the original private-source/getter/ENV
            % checks; the existing IO still owns callback/deadline admission.
            if isfield(cfg,'input_codec_backend')
                if isfield(cfg,'prepared_input_codec_backend')
                    [obj.InputCodecMex,obj.InputCodecAdapter,obj.InputCodecBinding]= ...
                        reusePreparedInputCodecBackend(cfg.prepared_input_codec_backend,cfg.input_codec_backend);
                else
                    [obj.InputCodecMex,obj.InputCodecAdapter,obj.InputCodecBinding]=checkedInputCodecBackend(cfg.input_codec_backend);
                end
                obj.InputCodecBackend='CANONICAL_LOCAL_TASK_WIRE_MEX';
            end
            obj.Io=io;obj.Registered=registered;obj.Getter=getter;obj.Environment=environment;obj.Task=task;obj.Cfg=cfg;
            obj.ComponentInitialization=isfield(cfg,'component_initialization')&&isequal(cfg.component_initialization,true);
            if isfield(cfg,'stream_lifecycle')
                assert(isa(cfg.stream_lifecycle,'gpenmpcNative.RflyLocalStreamLifecycle'), ...
                    'gpenmpcNative:LocalMethodStreamLifecycle','Explicit original stream lifecycle handle required.');
                streamState=cfg.stream_lifecycle.status();
                assert(streamState.enable_send_returned&&~streamState.failed&&streamState.disable_attempts==0, ...
                    'gpenmpcNative:LocalMethodStreamNotEnabled','Caller must explicitly enable the bound stream before constructing this opt-in service.');
                obj.StreamLifecycle=cfg.stream_lifecycle;
                % The caller owns stream enable/disable operations.
            end
            % Discard expired preparation observations without renewing source timestamps.
            obj.Cfg.prearm_observation_max_age_ns=min(cfg.source_max_age_ns,uint64(round(assets.enmpc.solver_deadline_s*1e9)));
            obj.Leg=bundle.legs{double(registered.leg_index)};obj.ReferenceBinding=referenceBinding;
            obj.Phase=gpenmpcNative.RflyLocalPhaseView(bundle,trajectory,registered,association);
            g=struct('uid',e.uid,'boot_generation',e.session_generation,'board_system',e.source_system, ...
                'board_component',e.source_component,'host_system',e.target_system,'host_component',e.target_component, ...
                'link_lifecycle_generation',e.link_lifecycle_generation,'confirmed_host_rx_ns',e.confirmed_host_rx_ns, ...
                'execution_session_sha256',e.execution_session_sha256);
            % Resolve and validate the selected GP binary at construction.
            gpPrepared=[];
            if isfield(cfg,'gp_backend'),g.gp_backend=cfg.gp_backend;end
            if isfield(cfg,'prepared_gp_backend'),gpPrepared=cfg.prepared_gp_backend;end
            obj.Gp=gpenmpcNative.RflyLocalGpService(assets,g,gpPrepared);
            rcReceiveOnly=obj.ComponentInitialization&&isfield(cfg,'operator_reference');
            % Match the RC environment service's explicit manual profile.
            % Autonomous sessions always keep the saved-task wind contract.
            obj.Task.manual_disturbance=rcReceiveOnly;
            if obj.Gp.ReceiveInlineEnabled&&(~obj.ComponentInitialization||rcReceiveOnly)
                % Share the native receiver; manual mode disables GP evaluation.
                obj.Io.attachCanonicalInlineGp(obj.Gp,true,obj.Cfg.source_max_age_ns,rcReceiveOnly);
            end
            if isfield(cfg,'prepared_outer')
                assert(isa(cfg.prepared_outer,'gpenmpcNative.RflyLocalOuterService')&&isscalar(cfg.prepared_outer), ...
                    'gpenmpcNative:LocalMethodPreparedOuter','Explicit original preconstructed outer owner required.');
                cfg.prepared_outer.bindPreparedSession(assets,registered,trajectory,cfg.source_max_age_ns,cfg.command_lifetime_ns);
                obj.Outer=cfg.prepared_outer;
            else
                obj.Outer=gpenmpcNative.RflyLocalOuterService(assets,registered,trajectory,cfg.source_max_age_ns,cfg.command_lifetime_ns);
            end
        end
        function event=poll(obj,getterBatch,mode,serviceIo,observingDisarmed,serviceHeartbeat)
            if nargin<4,serviceIo=@() [];end
            if nargin<5,observingDisarmed=false;end
            if nargin<6,serviceHeartbeat=@() [];end
            try
                assert(~obj.Closed&&~obj.Failed&&~obj.Busy,'gpenmpcNative:LocalMethodClosed');
                obj.Busy=true;unlock=onCleanup(@()obj.unlock()); %#ok<NASGU>
                % Resolve the ARM transition after receiving ACKs and snapshots.
                modeReader=[];
                if isa(mode,'function_handle'),modeReader=mode;mode=obj.Mode;end
                mode=char(mode);now=obj.now();event=struct('mode',mode,'now_ns',now,'status','IDLE','source',[], ...
                    'input_binding',[],'source_retired_without_send',false,'outer',[],'gp',{{}},'committed',{{}},'window',[], ...
                    'work_timing_ns',struct('observe_outer',uint64(0),'outer_result',uint64(0),'receive',uint64(0),'gp',uint64(0),'committed',uint64(0), ...
                    'source_bind',uint64(0),'input_encode',uint64(0),'input_send',uint64(0),'window',uint64(0)));
                % Consume the caller batch before cooperative service can
                % ingest a newer one; original sequence order is retained.
                getterState=obj.Getter.status();
                if ~getterState.source_matching_started,obj.Getter.observeOnly(getterBatch);
                else,obj.Getter.ingest(getterBatch);end
                runtimeStateOnly=isfield(obj.Cfg,'runtime_state_only')&&obj.Cfg.runtime_state_only;
                env=obj.Io.takeCanonicalEnvironmentRecords(~runtimeStateOnly);
                for k=1:numel(env)
                    if strcmp(env{k}.kind,'ENV_TX'),obj.Environment.sent(env{k});
                    elseif strcmp(env{k}.kind,'DIAGNOSTIC_RX'),obj.Environment.received(env{k});
                    else,error('gpenmpcNative:LocalMethodEnvironment','Unknown original environment event');end
                    if mod(k,8)==0
                        if runtimeStateOnly,serviceHeartbeat();else,serviceIo();end
                    end
                end
                % Complete worker processing before fast-state receive, then bind and send input
                % before retained RLC work. Service model, environment and getter beforehand.
                if runtimeStateOnly
                    outerStart=obj.now();
                    if obj.PreparationStarted&&~obj.Outer.Prepared
                        event.outer=obj.Outer.pollPreparation(obj.now());assert(~obj.Outer.Failed,'gpenmpcNative:LocalMethodOuter');
                    elseif obj.BootstrapStarted
                        event.outer=obj.Outer.poll(obj.now());assert(~obj.Outer.Failed,'gpenmpcNative:LocalMethodOuter');
                    end
                    event.work_timing_ns.outer_result=obj.now()-outerStart;
                    % Reuse the model/environment/getter snapshot; refresh only if source binding requires it.
                end
                if ~runtimeStateOnly,serviceIo();end
                % Complete queued GP work before receiving the next fast-state source.
                % BEGIN_ASYNC_GP_RECEIVE_SERVICE
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'FLIGHT')&&obj.Gp.AsyncEnabled
                    mark=obj.now();answer=obj.Gp.poll();
                    if ~isempty(answer)
                        q=obj.PendingGp;
                        assert(~isempty(q)&&isequal(answer.request.original_bytes,q.message), ...
                            'gpenmpcNative:LocalGpBackendReply','Finished reply must match the original pending request.');
                        sent=obj.Io.sendCanonicalLocalGp(answer.reply_bytes,q);
                        obj.Counts.gp_replies=obj.Counts.gp_replies+double(sent.messages_send_returned==3);
                        event.gp{end+1}=struct('request',q,'computed',answer,'send',sent);
                        obj.PendingGp=[];
                    elseif ~obj.Gp.isPending()
                        q=obj.Io.takeCanonical('gp_request',false);
                        if ~isempty(q)
                            witness=obj.witnessStream(q,1);
                            if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                            obj.Gp.begin(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                            obj.PendingGp=q;
                        end
                    end
                    event.work_timing_ns.gp=event.work_timing_ns.gp+obj.now()-mark;
                    serviceHeartbeat();
                end
                % END_ASYNC_GP_RECEIVE_SERVICE
                receiveStart=obj.now();
                link=obj.Io.pollCanonical(runtimeStateOnly,serviceHeartbeat);
                assert(isempty(link.failure),'gpenmpcNative:LocalMethodTransport','%s',link.failure);
                gpQueued=any(string(link.channels)=="gp_request" & link.completed_queue_counts>0);
                if runtimeStateOnly&&((link.snapshot_receive_pending&&(~gpQueued||obj.Gp.ReceiveInlineEnabled) ...
                        &&~any(string(link.channels)=="snapshot" & link.completed_queue_counts>0)) ...
                        ||(gpQueued&&~obj.Gp.AsyncEnabled&&~obj.Gp.ReceiveInlineEnabled ...
                        &&~any(string(link.channels)=="snapshot" & link.completed_queue_counts>0)))
                    % Continue the receive cursor once within its bounded work slice before binding the newest completed state.
                    serviceHeartbeat();
                    link=obj.Io.pollCanonical(true,serviceHeartbeat);
                    assert(isempty(link.failure),'gpenmpcNative:LocalMethodTransport','%s',link.failure);
                    gpQueued=any(string(link.channels)=="gp_request" & link.completed_queue_counts>0);
                end
                event.work_timing_ns.receive=obj.now()-receiveStart;
                if ~isempty(modeReader),mode=char(modeReader());end
                % Use the armed bootstrap result and fresh state from this receive batch.
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'BOOTSTRAP') ...
                        &&~observingDisarmed&&obj.BootstrapStarted&&~isempty(obj.Outer.LastCommand) ...
                        &&obj.now()<obj.Outer.LastCommand.original_expiry_ns
                    mode='FLIGHT';
                end
                assert(ismember(mode,{'PREPARE','BOOTSTRAP','ARM_WAIT','FLIGHT','COMPONENT'}));
                if strcmp(mode,'COMPONENT'),assert(obj.ComponentInitialization);end
                if strcmp(obj.Mode,'FLIGHT'),assert(strcmp(mode,'FLIGHT'),'gpenmpcNative:LocalMethodNoImplicitReset');end
                if strcmp(obj.Mode,'BOOTSTRAP'),assert(~strcmp(mode,'PREPARE'),'gpenmpcNative:LocalMethodNoImplicitReset');end
                if strcmp(obj.Mode,'ARM_WAIT'),assert(ismember(mode,{'ARM_WAIT','FLIGHT'}),'gpenmpcNative:LocalMethodNoImplicitReset');end
                obj.Mode=mode;event.mode=mode;
                % Dispatch new GP work without waiting for or serializing its result.
                % BEGIN_ASYNC_GP_NEW_RX_DISPATCH
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'FLIGHT') ...
                        &&obj.Gp.AsyncEnabled&&gpQueued&&~obj.Gp.isPending()
                    mark=obj.now();q=obj.Io.takeCanonical('gp_request',false);
                    if ~isempty(q)
                        witness=obj.witnessStream(q,1);
                        if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                        obj.Gp.begin(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                        obj.PendingGp=q;
                    end
                    event.work_timing_ns.gp=event.work_timing_ns.gp+obj.now()-mark;
                end
                % END_ASYNC_GP_NEW_RX_DISPATCH
                % ARM_WAIT may send a current original slow input while the ACK is pending.
                % Start source matching before the board module; pre-matching fixtures remain observation-only.
                if isempty(obj.Source)&&~runtimeStateOnly
                    obj.Source=obj.Io.takeCanonical('snapshot',false);
                    if ~isempty(obj.Source)
                        witness=obj.witnessStream(obj.Source,2);
                        if ~isempty(witness),event.stream_startup_witnesses{2}=witness;end
                        obj.Counts.snapshots_taken=obj.Counts.snapshots_taken+1;
                    end
                end
                if ~runtimeStateOnly,serviceIo();end
                if ~runtimeStateOnly&&obj.PreparationStarted&&~obj.Outer.Prepared
                    event.outer=obj.Outer.pollPreparation(obj.now());assert(~obj.Outer.Failed,'gpenmpcNative:LocalMethodOuter');
                elseif ~runtimeStateOnly&&obj.BootstrapStarted
                    event.outer=obj.Outer.poll(obj.now());assert(~obj.Outer.Failed,'gpenmpcNative:LocalMethodOuter');
                end
                mark=obj.now();event.work_timing_ns.observe_outer=mark-event.now_ns;
                % Service due heartbeats between work sections.
                serviceHeartbeat();
                if ~runtimeStateOnly
                % GP requests and committed observations originate exclusively
                % from this existing callback owner. Bounded dequeue work.
                for k=1:4
                    q=obj.Io.takeCanonical('gp_request',false);if isempty(q),break;end
                    witness=obj.witnessStream(q,1);
                    if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                    assert(strcmp(mode,'FLIGHT'),'gpenmpcNative:LocalMethodUnexpectedControl');
                    if obj.Gp.ReceiveInlineEnabled
                        assert(isfield(q,'inline_gp')&&~isempty(q.inline_gp),'gpenmpcNative:LocalGpReceiveMissing', ...
                            'Inline GP computation is required for this request.');
                        answer=q.inline_gp.computed;
                    else
                        answer=obj.Gp.process(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                    end
                    sent=obj.Io.sendCanonicalLocalGp(answer.reply_bytes,q);obj.Counts.gp_replies=obj.Counts.gp_replies+1;
                    event.gp{end+1}=struct('request',q,'computed',answer,'send',sent);
                    serviceIo();
                end
                next=obj.now();event.work_timing_ns.gp=next-mark;mark=next;
                end
                mark=obj.now();
                if runtimeStateOnly&&~obj.ComponentInitialization&&ismember(mode,{'ARM_WAIT','FLIGHT'})
                    % Retain the actual fast snapshots in the existing bounded
                    % pending list. An RLC may use a newer uORB state than the
                    % still-valid slow input; neither source is retimestamped.
                    if ~isempty(obj.Source)
                        snew=obj.Source.decoded;
                        if ~any(cellfun(@(p)p.generation==snew.source_generation,obj.Pending))
                            if numel(obj.Pending)==obj.Cfg.pending_capacity,obj.Pending(1)=[];end
                            obj.Pending{end+1}=struct('generation',snew.source_generation,'sample_us',snew.original_sample_us, ...
                                'source',obj.Source,'inputs',[],'send',[]);
                        end
                    end
                    state=obj.Io.pollCanonical(false);
                    si=find(string(state.channels)=="snapshot");
                    for queued=1:state.completed_queue_counts(si)
                        source=obj.Io.takeCanonical('snapshot',false);
                        if ~isempty(obj.Source)
                            old=obj.Source.decoded;
                            obj.Getter.retireResolvedSource(old.source_generation,old);
                        end
                        % Retain source snapshots for RLC matching; only the newest drives input transmission.
                        snew=source.decoded;
                        obj.Source=source;obj.ArmPrepared=[];obj.Counts.snapshots_taken=obj.Counts.snapshots_taken+1;
                        keep=cellfun(@(p)snew.original_sample_us<=p.sample_us||snew.original_sample_us-p.sample_us<=uint64(400000),obj.Pending);
                        obj.Pending=obj.Pending(keep);
                        ix=find(cellfun(@(p)p.generation==snew.source_generation,obj.Pending));
                        if isempty(ix)
                            if numel(obj.Pending)==obj.Cfg.pending_capacity,obj.Pending(1)=[];end
                            obj.Pending{end+1}=struct('generation',snew.source_generation,'sample_us',snew.original_sample_us, ...
                                'source',source,'inputs',[],'send',[]);
                        end
                    end
                end
                if ~runtimeStateOnly
                for k=1:4
                    if obj.ComponentInitialization,break;end
                    c=obj.Io.takeCanonical('committed_state',false);if isempty(c),break;end
                    witness=obj.witnessStream(c,3);
                    if ~isempty(witness),event.stream_startup_witnesses{3}=witness;end
                    assert(ismember(mode,{'FLIGHT','COMPONENT'}),'gpenmpcNative:LocalMethodUnexpectedControl');
                    dc=gpenmpcNative.RflyLocalCommittedDecoder(c.message,c.original_host_receive_ns);
                    index=find(cellfun(@(x)x.generation==dc.source_generation,obj.Pending));
                    assert(numel(index)==1,'gpenmpcNative:LocalMethodUnmatchedCommit');
                    p=obj.Pending{index};obj.Phase.ingest(c);
                    outerInputs=p.inputs;
                    outer=obj.Outer.sample(p.source,c,outerInputs,true,obj.now());
                    assert(dc.window_generation<=obj.WindowGeneration&&dc.window_generation>=obj.LastCommittedWindow, ...
                        'gpenmpcNative:LocalMethodWindowReceipt');obj.LastCommittedWindow=dc.window_generation;
                    obj.Pending(index)=[];
                    obj.Counts.commits=obj.Counts.commits+1;
                    event.committed{end+1}=struct('raw',c,'original_input_source',p,'outer_observation_inputs',outerInputs,'outer_event',outer);
                    serviceIo();
                end
                next=obj.now();event.work_timing_ns.committed=next-mark;mark=next;
                else
                    next=obj.now();mark=next;
                end
                % Dequeue completed snapshots before waiting for an incomplete successor, keeping the bounded queue available.
                if runtimeStateOnly&&(obj.ComponentInitialization||~ismember(mode,{'ARM_WAIT','FLIGHT'}))
                    link=obj.Io.pollCanonical(false);
                    si=find(string(link.channels)=="snapshot");
                    for queued=1:link.completed_queue_counts(si)
                        nextSource=obj.Io.takeCanonical('snapshot',false);
                        if ~isempty(obj.Source)
                            old=obj.Source.decoded;
                            obj.Getter.retireResolvedSource(old.source_generation,old);
                            obj.Counts.disarmed_sources_retired=obj.Counts.disarmed_sources_retired+1;
                            obj.ArmPrepared=[];
                        end
                        obj.Source=nextSource;obj.Counts.snapshots_taken=obj.Counts.snapshots_taken+1;
                    end
                    if ~isempty(obj.Source)
                        witness=obj.witnessStream(obj.Source,2);
                        if ~isempty(witness),event.stream_startup_witnesses{2}=witness;end
                    end
                end
                deferSource=runtimeStateOnly&&link.snapshot_receive_pending;
                % Follow the receiver's pending-successor decision; retain original source records for RLC matching.
                if deferSource
                    % Continue receiving on the next pump instead of encoding an older source.
                    serviceHeartbeat();event.status='RECEIVE_BATCH_CONTINUES';
                end
                % Defer source binding only; continue reference-window transmission.
                if ~deferSource
                if ~runtimeStateOnly
                    % Refresh model, environment and getter data before binding the source.
                    serviceIo();
                end
                if ~isempty(obj.Source)
                    s=gpenmpcNative.RflyLocalSnapshotDecoder(obj.Source.message,obj.Source.original_host_receive_ns);
                    if obj.ComponentInitialization&&s.source_generation<obj.LastCommitSource
                        obj.Source=[]; % Retire the stale source.
                    elseif runtimeStateOnly&&~obj.ComponentInitialization ...
                            &&ismember(mode,{'ARM_WAIT','FLIGHT'}) ...
                            &&obj.now()>obj.Source.original_host_receive_ns+obj.Cfg.source_max_age_ns ...
                            &&~isempty(obj.Outer.LastCommand)&&obj.now()<obj.Outer.LastCommand.original_expiry_ns
                        % Retire an unsent expired observation while retaining its matching history.
                        event.source=obj.Source;event.source_retired_without_send=true;
                        event.status='UNSENT_EXPIRED_SOURCE_RETIRED_BEFORE_BIND';
                        obj.Getter.retireResolvedSource(s.source_generation,s);
                        obj.Source=[];obj.ArmPrepared=[];
                    end
                end
                if ~isempty(obj.Source)
                    if ~isempty(obj.ArmPrepared)
                        binding=obj.ArmPrepared.binding;receipt=obj.ArmPrepared.receipt;
                    else
                        [phase,~]=obj.Phase.bindSource(obj.Source);
                        [binding,receipt]=obj.Getter.bind(obj.Source.message,obj.Source.original_host_receive_ns,obj.Task,phase,obj.Environment);
                        if runtimeStateOnly&&~obj.ComponentInitialization&&~binding.numerical_inputs_complete
                            % Refresh only when binding reports a missing or stale observation.
                            serviceIo();
                            [binding,receipt]=obj.Getter.bind(obj.Source.message,obj.Source.original_host_receive_ns,obj.Task,phase,obj.Environment);
                        end
                    end
                    event.source=obj.Source;now=obj.now();
                    event.work_timing_ns.source_bind=now-mark;
                    serviceHeartbeat();
                    assert(now>=obj.Source.original_host_receive_ns,'gpenmpcNative:LocalMethodFutureSource');
                    age=now-obj.Source.original_host_receive_ns;
                    ageLimit=obj.Cfg.source_max_age_ns;
                    if ~strcmp(mode,'FLIGHT'),ageLimit=obj.Cfg.prearm_observation_max_age_ns;end
                    expired=age>ageLimit;
                    if strcmp(mode,'FLIGHT')
                        assert(~expired,'gpenmpcNative:LocalMethodSourceExpired', ...
                            'Input source age %.3f ms exceeds %.3f ms.',double(age)/1e6,double(ageLimit)/1e6);
                    end
                    consume=false;
                    if binding.numerical_inputs_complete
                        if strcmp(mode,'COMPONENT')
                            if ~expired&&obj.WindowGeneration>0
                                % Component mode uses unit phase rate and zero outer correction.
                                candidate=struct('generation',s.source_generation, ...
                                    'source_generation',s.source_generation,'original_sample_us',s.original_sample_us, ...
                                    'original_host_source_receive_ns',obj.Source.original_host_receive_ns, ...
                                    'original_creation_ns',now,'original_expiry_ns',obj.Source.original_host_receive_ns+obj.Cfg.command_lifetime_ns, ...
                                    'target4',zeros(4,1));
                                if isfield(obj.Cfg,'operator_reference')
                                    usb=gpenmpc_usb_joystick_mex('read');
                                    operator=gpenmpcNative.normalizeUsbRcInput(usb,obj.Cfg.operator_reference,usb.host_read_qpc_s,.1,s.canonical_state13);
                                    assert(operator.valid,'gpenmpcNative:UsbOperatorInput','%s',operator.reason);
                                    % Convert heading-relative stick input to bounded world-frame velocity and yaw rate.
                                    candidate.target4=operator.target4;
                                    candidate.operator_input=operator;
                                    candidate.reference_source=operator.reference_source;
                                    candidate.requested_yaw_rate_rad_s=candidate.target4(1);
                                    obj.OperatorFinishRequested=obj.OperatorFinishRequested||operator.finish_requested;
                                end
                                mark=obj.now();
                                if isempty(obj.InputCodecMex)
                                    inputs=gpenmpcNative.RflyLocalTaskCodec.encode(binding,obj.Source.message, ...
                                        obj.Source.original_host_receive_ns,candidate,obj.Registered);
                                else
                                    inputs=obj.InputCodecAdapter(binding,obj.Source.message,obj.Source.original_host_receive_ns, ...
                                        candidate,obj.Registered,obj.InputCodecMex);
                                end
                                obj.InputCodecCalls=obj.InputCodecCalls+uint64(1);
                                next=obj.now();event.work_timing_ns.input_encode=next-mark;mark=next;
                                serviceHeartbeat();
                                try
                                    event.input_send=obj.Io.sendCanonicalLocalInput(inputs,obj.Source);
                                    assert(event.input_send.messages_send_returned==6 ...
                                        &&event.input_send.binding.original_source_generation==candidate.source_generation, ...
                                        'gpenmpcNative:LocalMethodInputSend','Complete same-generation input transmission required.');
                                    candidate.input_send=event.input_send;
                                    obj.NominalCommand=candidate;
                                    obj.Counts.inputs_sent=obj.Counts.inputs_sent+1;
                                catch inputError
                                    event.work_timing_ns.input_send=obj.now()-mark;
                                    if strcmp(inputError.identifier,'m600check:CanonicalLocalInputUnsentExpired')
                                        % Retire a candidate that expired before any fragment was sent.
                                        % Attempted or partial transmissions remain errors.
                                        expired=true;
                                        event.status='COMPONENT_UNSENT_CANDIDATE_RETIRED';
                                    else,rethrow(inputError);end
                                end
                                event.work_timing_ns.input_send=obj.now()-mark;
                            end
                            consume=true;event.source_retired_without_send=expired;
                        elseif strcmp(mode,'PREPARE')
                            if ~obj.PreparationStarted&&~expired
                                event.outer=obj.Outer.beginPreparation(obj.Source,binding,now);obj.PreparationStarted=true;
                            end
                            consume=true;event.source_retired_without_send=true;
                        elseif strcmp(mode,'BOOTSTRAP')
                            assert(obj.Outer.Prepared,'gpenmpcNative:LocalMethodNotPrepared');
                            if ~obj.BootstrapStarted&&~expired
                                event.outer=obj.Outer.bootstrap(obj.Source,binding,now,runtimeStateOnly&&~observingDisarmed);obj.BootstrapStarted=true;
                            end
                            consume=true;event.source_retired_without_send=true;
                        elseif strcmp(mode,'ARM_WAIT')
                            if ~expired
                                assert(obj.BootstrapStarted&&~isempty(obj.Outer.LastCommand)&&obj.WindowGeneration>0, ...
                                    'gpenmpcNative:LocalMethodNoFreshStart','Original bootstrap and reference are required.');
                                mark=obj.now();
                                event.input_codec_backend=obj.InputCodecBackend;
                                if isempty(obj.InputCodecMex)
                                    inputs=gpenmpcNative.RflyLocalTaskCodec.encode(binding,obj.Source.message,obj.Source.original_host_receive_ns, ...
                                        obj.Outer.LastCommand,obj.Registered);
                                else
                                    inputs=obj.InputCodecAdapter(binding,obj.Source.message,obj.Source.original_host_receive_ns, ...
                                        obj.Outer.LastCommand,obj.Registered,obj.InputCodecMex);
                                end
                                obj.InputCodecCalls=obj.InputCodecCalls+uint64(1);
                                event.work_timing_ns.input_encode=obj.now()-mark;
                                if runtimeStateOnly
                                    mark=obj.now();
                                    sent=obj.Io.sendCanonicalLocalInput(inputs,obj.Source);
                                    event.work_timing_ns.input_send=obj.now()-mark;
                                    ix=find(cellfun(@(p)p.generation==s.source_generation,obj.Pending));
                                    assert(numel(ix)==1,'gpenmpcNative:LocalMethodPendingSource');
                                    obj.Pending{ix}.inputs=inputs;obj.Pending{ix}.send=sent;
                                    obj.Counts.inputs_sent=obj.Counts.inputs_sent+1;
                                else,obj.ArmPrepared=struct('binding',binding,'receipt',receipt,'inputs',inputs);
                                end
                            end
                            if runtimeStateOnly
                                if expired,event.status='ARM_UNSENT_SOURCE_EXPIRED';else,event.status='ORIGINAL_INPUT_SENT_WHILE_ARM_ACK_PENDING';end
                                consume=true;
                            else,event.status='ARM_INPUT_PREPARED_NOT_SENT';consume=expired;end
                            event.source_retired_without_send=expired;
                        else
                            assert(obj.BootstrapStarted&&~isempty(obj.Outer.LastCommand)&&obj.WindowGeneration>0 ...
                                ,'gpenmpcNative:LocalMethodNoFreshStart');
                            mark=obj.now();
                            event.input_codec_backend=obj.InputCodecBackend;
                            if ~isempty(obj.ArmPrepared)
                                inputs=obj.ArmPrepared.inputs;
                            elseif isempty(obj.InputCodecMex)
                                inputs=gpenmpcNative.RflyLocalTaskCodec.encode(binding,obj.Source.message,obj.Source.original_host_receive_ns, ...
                                    obj.Outer.LastCommand,obj.Registered);
                                obj.InputCodecCalls=obj.InputCodecCalls+uint64(1);
                            else
                                inputs=obj.InputCodecAdapter(binding,obj.Source.message,obj.Source.original_host_receive_ns, ...
                                    obj.Outer.LastCommand,obj.Registered,obj.InputCodecMex);
                                obj.InputCodecCalls=obj.InputCodecCalls+uint64(1);
                            end
                            next=obj.now();event.work_timing_ns.input_encode=next-mark;mark=next;
                            serviceHeartbeat();
                            sent=obj.Io.sendCanonicalLocalInput(inputs,obj.Source);
                            event.work_timing_ns.input_send=obj.now()-mark;
                            event.input_send=sent;
                            ix=find(cellfun(@(p)p.generation==s.source_generation,obj.Pending));
                            if runtimeStateOnly
                                assert(numel(ix)==1,'gpenmpcNative:LocalMethodPendingSource');
                                obj.Pending{ix}.inputs=inputs;obj.Pending{ix}.send=sent;
                            else
                                assert(numel(obj.Pending)<obj.Cfg.pending_capacity,'gpenmpcNative:LocalMethodPendingCapacity');
                                obj.Pending{end+1}=struct('generation',s.source_generation,'source',obj.Source,'inputs',inputs,'send',sent);
                            end
                            obj.Counts.inputs_sent=obj.Counts.inputs_sent+1;consume=true;event.status='ORIGINAL_INPUTS_SENT_NOT_COMMITTED';
                        end
                    else
                        obj.Counts.missing_binding_polls=obj.Counts.missing_binding_polls+1;event.status=binding.status;
                        % Retire expired unsubmitted observations only outside active full-method control.
                        if expired
                            runtimeStateOnly=isfield(obj.Cfg,'runtime_state_only')&&obj.Cfg.runtime_state_only;
                            assert(~strcmp(mode,'FLIGHT')&&(isfield(receipt,'retirement_lease')||runtimeStateOnly), ...
                                'gpenmpcNative:LocalMethodUnresolvedSourceExpired', ...
                                'A matching getter record was not available within the acquisition bound.');
                            consume=true;event.source_retired_without_send=true;
                        end
                    end
                    % Defer retained-history matching until after input transmission.
                    event.input_binding=compactBinding(receipt);
                    if consume
                        obj.Getter.retireResolvedSource(s.source_generation,s);obj.Source=[];obj.ArmPrepared=[];
                        if event.source_retired_without_send&&observingDisarmed
                            obj.Counts.disarmed_sources_retired=obj.Counts.disarmed_sources_retired+1;
                            if expired,obj.Counts.expired_disarmed_sources=obj.Counts.expired_disarmed_sources+1;end
                        end
                    end
                end
                end % existing source work; reference service always runs
                % Service GP work after a complete input transmission.
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'FLIGHT') ...
                        &&~obj.Gp.AsyncEnabled&&gpQueued ...
                        &&isfield(event,'input_send')&&event.input_send.messages_send_returned==6
                    mark=obj.now();q=obj.Io.takeCanonical('gp_request',false);
                    assert(~isempty(q),'gpenmpcNative:LocalMethodGpQueue');
                    witness=obj.witnessStream(q,1);
                    if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                    if obj.Gp.ReceiveInlineEnabled
                        assert(isfield(q,'inline_gp')&&~isempty(q.inline_gp),'gpenmpcNative:LocalGpReceiveMissing', ...
                            'Inline GP computation is required for this request.');
                        answer=q.inline_gp.computed;
                    else
                        answer=obj.Gp.process(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                    end
                    sent=obj.Io.sendCanonicalLocalGp(answer.reply_bytes,q);
                    obj.Counts.gp_replies=obj.Counts.gp_replies+double(sent.messages_send_returned==3);
                    event.gp{end+1}=struct('request',q,'computed',answer,'send',sent);
                    event.work_timing_ns.gp=obj.now()-mark;serviceHeartbeat();
                end
                % The same future can finish while fast input is prepared.
                % Check completion without waiting before processing history.
                % BEGIN_ASYNC_GP_POST_INPUT_COMPLETION
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'FLIGHT') ...
                        &&obj.Gp.AsyncEnabled&&isempty(event.gp)
                    mark=obj.now();answer=obj.Gp.poll();
                    if ~isempty(answer)
                        q=obj.PendingGp;
                        assert(~isempty(q)&&isequal(answer.request.original_bytes,q.message), ...
                            'gpenmpcNative:LocalGpBackendReply','Finished reply must match the original pending request.');
                        sent=obj.Io.sendCanonicalLocalGp(answer.reply_bytes,q);
                        obj.Counts.gp_replies=obj.Counts.gp_replies+double(sent.messages_send_returned==3);
                        event.gp{end+1}=struct('request',q,'computed',answer,'send',sent);
                        obj.PendingGp=[];serviceHeartbeat();
                    elseif ~obj.Gp.isPending()
                        q=obj.Io.takeCanonical('gp_request',false);
                        if ~isempty(q)
                            witness=obj.witnessStream(q,1);
                            if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                            obj.Gp.begin(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                            obj.PendingGp=q;
                        end
                    end
                    event.work_timing_ns.gp=event.work_timing_ns.gp+obj.now()-mark;
                end
                % END_ASYNC_GP_POST_INPUT_COMPLETION
                % Process retained outer observations after input transmission.
                if runtimeStateOnly&&isfield(event,'input_send')&&event.input_send.messages_send_returned==6
                    mark=obj.now();
                    % Defer the next receive slice until the next poll; process queued GP/RLC records here.
                    serviceHeartbeat();
                    queued=obj.Io.pollCanonical(false);
                    assert(isempty(queued.failure),'gpenmpcNative:LocalMethodTransport');
                    % An in-flight GP result does not block processing an earlier committed-state observation.
                    gpQueued=~obj.Gp.isPending()&&any(queued.completed_queue_counts(string(queued.channels)=="gp_request")>0);
                    % Inline replies are already transmitted; handle their bookkeeping without blocking retained observations.
                    if (~gpQueued||obj.Gp.ReceiveInlineEnabled) ...
                            &&(isempty(event.gp)||obj.Gp.AsyncEnabled||obj.Gp.ReceiveInlineEnabled)
                for k=1:1
                    if obj.ComponentInitialization,break;end
                    c=obj.Io.takeCanonical('committed_state',false);if isempty(c),break;end
                    witness=obj.witnessStream(c,3);
                    if ~isempty(witness),event.stream_startup_witnesses{3}=witness;end
                    assert(ismember(mode,{'FLIGHT','COMPONENT'}),'gpenmpcNative:LocalMethodUnexpectedControl');
                    dc=gpenmpcNative.RflyLocalCommittedDecoder(c.message,c.original_host_receive_ns);
                    index=find(cellfun(@(x)x.generation==dc.source_generation,obj.Pending));
                    assert(numel(index)==1,'gpenmpcNative:LocalMethodUnmatchedCommit');
                    p=obj.Pending{index};obj.Phase.ingest(c);
                    if runtimeStateOnly
                        % Match slow-input values against the RLC state and outer generation.
                        eligible={};values=[];
                        for j=1:numel(obj.Pending)
                            q=obj.Pending{j};if isempty(q.inputs),continue,end
                            u=q.send.decoded_input;
                            assert(q.send.messages_send_returned==6&&isequal(u.original_bytes,q.inputs), ...
                                'gpenmpcNative:LocalMethodRetainedInput','Only the exact successfully sent input parse is reusable.');
                            if u.source.sample_us>idivide(dc.source_timestamp_ns,uint64(1000)) ...
                                    ||idivide(dc.source_timestamp_ns,uint64(1000))-u.source.sample_us>uint64(100000) ...
                                    ||u.outer.generation~=dc.outer_generation,continue,end
                            v=[u.payload.payload_kg;u.wind.estimate_xy_mps(:)];
                            if isempty(values),values=v;else,assert(isequaln(values,v),'gpenmpcNative:LocalMethodAmbiguousSlowInputs');end
                            eligible{end+1}=q; %#ok<AGROW>
                        end
                        assert(~isempty(eligible),'gpenmpcNative:LocalMethodMissingSlowInputs');
                        original=eligible{end};
                        outerInputs=struct('held_input_bytes',original.inputs,'original_input_source',original.source);
                    else,outerInputs=p.inputs;
                    end
                    % Existing complete send parse is immutable value/COW
                    % data. Reuse it for the slower matched outer observation
                    % instead of parsing the same 647 bytes after input send.
                    if runtimeStateOnly
                        outerInputs.held_input_decoded=original.send.decoded_input;
                    end
                    outer=obj.Outer.sample(p.source,c,outerInputs,true,obj.now(),true);
                    assert(dc.window_generation<=obj.WindowGeneration&&dc.window_generation>=obj.LastCommittedWindow, ...
                        'gpenmpcNative:LocalMethodWindowReceipt');obj.LastCommittedWindow=dc.window_generation;
                    if ~runtimeStateOnly,obj.Pending(index)=[];end
                    obj.Counts.commits=obj.Counts.commits+1;
                    event.committed{end+1}=struct('raw',c,'original_input_source',p,'outer_observation_inputs',outerInputs,'outer_event',outer);
                    serviceHeartbeat();
                end
                next=obj.now();event.work_timing_ns.committed=event.work_timing_ns.committed+next-mark;mark=next;
                    end
                end
                if runtimeStateOnly&&~obj.ComponentInitialization&&strcmp(mode,'FLIGHT') ...
                        &&isfield(event,'input_send')&&event.input_send.messages_send_returned==6
                    % Schedule from fresh RLS while retaining matched RLC as solver input.
                    mark=obj.now();
                    event.outer_clock=obj.Outer.serviceSourceClock(event.source,true,mark);
                    event.work_timing_ns.committed=event.work_timing_ns.committed+obj.now()-mark;
                end
                % Service synchronous GP work only after the current input has been transmitted.
                if runtimeStateOnly&&~obj.ComponentInitialization&&~obj.Gp.AsyncEnabled&&isempty(event.gp)&&isempty(event.committed) ...
                        &&(~isfield(event,'outer_clock')||~event.outer_clock.outer_submitted) ...
                        &&isfield(event,'input_send')&&event.input_send.messages_send_returned==6
                    mark=obj.now();
                    if ~obj.Gp.isPending()
                      q=obj.Io.takeCanonical('gp_request',false);
                      if ~isempty(q)
                        witness=obj.witnessStream(q,1);
                        if ~isempty(witness),event.stream_startup_witnesses{1}=witness;end
                        assert(strcmp(mode,'FLIGHT'),'gpenmpcNative:LocalMethodUnexpectedControl');
                        if obj.Gp.ReceiveInlineEnabled
                            assert(isfield(q,'inline_gp')&&~isempty(q.inline_gp),'gpenmpcNative:LocalGpReceiveMissing', ...
                                'Inline GP computation is required for this request.');
                            answer=q.inline_gp.computed;
                        else
                            answer=obj.Gp.process(q.message,q.original_host_receive_ns,obj.now(),q.origin);
                        end
                        sent=obj.Io.sendCanonicalLocalGp(answer.reply_bytes,q);
                        obj.Counts.gp_replies=obj.Counts.gp_replies+double(sent.messages_send_returned==3);
                        event.gp{end+1}=struct('request',q,'computed',answer,'send',sent);
                        serviceHeartbeat();
                      end
                    end
                    event.work_timing_ns.gp=obj.now()-mark;
                end
                % Send component input using the last validated phase, then process queued RLC records to advance phase and commit count.
                if obj.ComponentInitialization&&isfield(event,'input_send') ...
                        &&event.input_send.messages_send_returned==6
                    mark=obj.now();
                    for k=1:4
                        c=obj.Io.takeCanonical('committed_state',false);if isempty(c),break;end
                        dc=gpenmpcNative.RflyLocalCommittedDecoder(c.message,c.original_host_receive_ns);
                        obj.Phase.ingest(c);obj.LastCommitSource=dc.source_generation;
                        obj.Counts.commits=double(dc.joint_installs);
                        event.committed{end+1}=struct('raw',c,'outer_event',[], ...
                            'scope','BOARD_SE3_COMPONENT__NOMINAL_COMMAND__NO_ENMPC_OR_GP_INFERENCE');
                        if isfield(obj.Cfg,'operator_reference')
                            event.committed{end}.scope='BOARD_SE3_COMPONENT__USB_OPERATOR_REFERENCE__NO_ENMPC_OR_GP_INFERENCE';
                        end
                    end
                    event.work_timing_ns.committed=obj.now()-mark;
                end
                mark=obj.now();event.window=obj.serviceWindow(strcmp(mode,'FLIGHT'),serviceIo);
                event.processing_finished_ns=obj.now();
                event.work_timing_ns.window=event.processing_finished_ns-mark;
                event.processing_elapsed_ns=event.processing_finished_ns-event.now_ns;
                obj.LastEvent=event;
            catch ex
                % Retire an unsent expired candidate while its outer command remains valid.
                % Partial sends, expired commands and source/environment faults remain errors.
                unsentCandidate=~obj.ComponentInitialization&&isfield(obj.Cfg,'runtime_state_only') ...
                    &&obj.Cfg.runtime_state_only ...
                    &&ismember(ex.identifier,{'m600check:CanonicalLocalInputUnsentExpired','gpenmpcNative:LocalMethodSourceExpired'}) ...
                    &&exist('binding','var')&&binding.numerical_inputs_complete ...
                    &&~isempty(obj.Source)&&~isempty(obj.Outer.LastCommand) ...
                    &&obj.now()<obj.Outer.LastCommand.original_expiry_ns;
                if unsentCandidate
                    event.source_retired_without_send=true;
                    if obj.Counts.inputs_sent==0,event.status='UNSENT_FIRST_FULL_INPUT_RETIRED';
                    else,event.status='UNSENT_FULL_INPUT_RETIRED';end
                    event.input_binding=compactBinding(receipt);
                    obj.Getter.retireResolvedSource(s.source_generation,s);
                    obj.Source=[];obj.ArmPrepared=[];
                    serviceHeartbeat();event.window=obj.serviceWindow(strcmp(mode,'FLIGHT'),serviceIo);
                    event.processing_finished_ns=obj.now();
                    event.processing_elapsed_ns=event.processing_finished_ns-event.now_ns;
                    obj.LastEvent=event;return
                end
                obj.Failed=true;obj.Failure=ex.identifier;
                if exist('receipt','var')&&isstruct(receipt),event.input_binding=compactBinding(receipt);end
                if exist('event','var'),obj.LastEvent=event;end
                rethrow(ex)
            end
        end
        function s=status(obj)
            s=struct('failed',obj.Failed,'failure',obj.Failure,'closed',obj.Closed,'mode',obj.Mode, ...
                'counts',obj.Counts,'outer',obj.Outer.status(),'gp_calls',obj.Gp.Calls,'gp_completed',obj.Gp.Completed, ...
                'gp_backend',obj.Gp.Backend,'gp_backend_binding',obj.Gp.BackendBinding, ...
                'input_codec_backend',obj.InputCodecBackend,'input_codec_backend_binding',obj.InputCodecBinding, ...
                'input_codec_calls',obj.InputCodecCalls, ...
                'pending_input_sources',numel(obj.Pending),'source_waiting',~isempty(obj.Source),'last_window_tx',obj.WindowTx, ...
                'last_board_committed_window',obj.LastCommittedWindow,'host_inner_steps',0,'plant_steps',0, ...
                'additional_connections',0,'arm_mode_commands',0);
            s.component_initialization=obj.ComponentInitialization;s.nominal_initialization_command=obj.NominalCommand;
            if ~isempty(obj.StreamLifecycle)
                policy='FIRST_NONEMPTY_ORIGINAL_ITEM_PER_CHANNEL_ONLY';
                if isfield(obj.Cfg,'runtime_state_only')&&obj.Cfg.runtime_state_only
                    policy='RAW_RETAINED_FOR_POSTRUN__NOT_SYNCHRONOUSLY_WITNESSED';
                end
                s.stream_startup_witness=struct('enabled',true, ...
                    'policy',policy, ...
                    'channels',{{'gp_request','snapshot','committed_state'}}, ...
                    'witnessed',obj.StreamWitnessed,'observation_counts',uint64(obj.StreamWitnessed), ...
                    'all_messages_reobserved',false,'enable_disable_owned_by_caller',true);
            end
        end
        function suspendForNativeLand(obj)
            assert(~obj.Closed,'gpenmpcNative:LocalMethodClosed');obj.Phase.suspend('NATIVE_LAND');obj.close();
            % Landing, disarm and task release remain the caller's responsibility.
        end
        function close(obj)
            if obj.Closed,return;end;obj.Closed=true;
            if ~isempty(obj.Gp)
                obj.Gp.close();
                if obj.Gp.ReceiveContinuousEnabled&&isfield(obj.Gp.LastRaw,'continuous_stop')
                    % Count successful native replies.
                    obj.Counts.gp_replies=double(obj.Gp.LastRaw.continuous_stop.replies);
                end
            end
            if ~isempty(obj.Outer)
                if obj.Outer.Preconstructed
                    % Return borrowed worker ownership without joining during safety pumping.
                    obj.Outer.suspendForNativeLand();
                else
                    obj.Outer.close();
                end
            end
            % IO, stream enable/disable, getter mapping and task environment remain owned by
            % the outer runner for its real native LAND/safe finally.
        end
        function delete(obj),obj.close();end
    end
    methods (Access=private)
        function t=now(obj)
            t=gpenmpcNative.rflyOriginalHostMonotonicNs();assert(t>=obj.LastNow,'gpenmpcNative:LocalMethodClock');obj.LastNow=t;
        end
        function unlock(obj),obj.Busy=false;end
        function observation=witnessStream(obj,item,index)
            observation=[];
            if isempty(obj.StreamLifecycle)||obj.StreamWitnessed(index),return;end
            % Runtime messages are validated by the receiver and phase decoder; skip the extra startup history witness.
            if isfield(obj.Cfg,'runtime_state_only')&&obj.Cfg.runtime_state_only,return;end
            % Validate the first stream item; subsequent messages use the receive-path checks.
            observation=obj.StreamLifecycle.observe(item);
            names={'gp_request','snapshot','committed_state'};
            assert(strcmp(observation.channel,names{index}),'gpenmpcNative:LocalMethodStreamChannel','Startup witness must match the consumed original channel.');
            obj.StreamWitnessed(index)=true;
        end
        function r=serviceWindow(obj,active,serviceIo)
            r=[];
            % Finish cold preparation before starting the reference-window assembly interval.
            if isempty(obj.LastWindow)&&~obj.Outer.Prepared&&~obj.ComponentInitialization,return;end
            if isfield(obj.Cfg,'operator_reference')&&~isempty(obj.WindowTx)&&obj.WindowTx.send_complete
                % Manual mode uses one ground initializer; the board shapes subsequent operator references.
                return
            end
            v=obj.Phase.view();q=v.controller_phase_s;
            nominal=q;if obj.Registered.leg_index==1,nominal=max(0,q-25);end
            if isempty(obj.LastWindow)
                first=1;
            elseif isempty(obj.WindowTx)||~obj.WindowTx.send_complete
                first=0;
            else
                w=obj.LastWindow;last=double(w.source_first_row)+double(w.row_count)-1;
                % Refill overlapping saved rows at half capacity.
                trigger=double(w.source_first_row)+floor(double(w.row_count)/2);
                first=0;
                if last<double(w.source_total_rows)&&nominal>=obj.Leg.local_time_s(trigger)
                    first=max(1,find(obj.Leg.local_time_s<=nominal,1,'last')-1);
                    assert(first>double(w.source_first_row)&&first<last,'gpenmpcNative:LocalMethodLateWindowRefill');
                end
            end
            if first>0
                serviceIo();
                obj.WindowGeneration=obj.WindowGeneration+uint64(1);
                hash=uint8(sscanf(char(obj.Registered.reference_asset_sha256),'%2x'));
                [w,~]=gpenmpcNative.prepareCanonicalReferenceWindow(obj.Leg,obj.ReferenceBinding,first,obj.WindowGeneration,hash);
                serviceIo();
                obj.WindowTx=obj.Io.beginCanonicalLocalWindow(w,serviceIo);obj.LastWindow=w;
                serviceIo();
            end
            if ~isempty(obj.WindowTx)&&~obj.WindowTx.send_complete
                count=16;if active,count=1;end
                batches=1;
                if obj.ComponentInitialization&&~active,batches=16;end
                % Send bounded fragment batches while servicing IO between batches.
                % The assembly deadline remains anchored to the first send.
                for batch=1:batches
                    obj.WindowTx=obj.Io.sendCanonicalLocalWindowChunk(count,serviceIo);r=obj.WindowTx;
                    if r.send_complete,break;end
                    serviceIo();
                end
                if r.send_complete,obj.Counts.reference_windows_sent=obj.Counts.reference_windows_sent+1;end
            end
        end
    end
    methods (Static)
        function prepared=prepareInputCodecBackend(binding)
            % Resolve and load the codec before the simulator starts.
            [encoder,adapter,receipt]=checkedInputCodecBackend(binding);
            prepared=struct('schema','GPENMPC_PREPARED_LOCAL_INPUT_CODEC_BACKEND_V1', ...
                'binding',binding,'encoder',encoder,'adapter',adapter,'receipt',receipt, ...
                'prepared_before_getter_producer',true,'authority_granted',false, ...
                'construction_codec_calls',0,'construction_numerical_calls',0);
        end
    end
end
function r=compactBinding(full)
% Retain the matched record and timestamp instead of duplicating the complete raw batch.
r=rmfield(full,'original_window');w=full.original_window;g=full.matching.getter_index(1);
r.original_window=rmfield(w,{'records','original_read_ns'});
r.original_window.record_count=size(w.records,2);r.original_window.raw_batch_archive_required=true;
r.matched_original_record=[];r.matched_original_read_ns=uint64(0);
if g>0,r.matched_original_record=w.records(:,g);r.matched_original_read_ns=w.original_read_ns(g);end
end
function [encoder,adapter,receipt]=checkedInputCodecBackend(binding)
approvedBinary='150D341A3E0C5EB3F382FF494CBB69D07BC27231767CB48055052CA9AEA940EB';
approvedAdapter='C2ACAEF7AFA34C6C2497928B562317A4C7F6F6D6194689AA14056BCEA35BD496';
assert(isstruct(binding)&&isscalar(binding) ...
    &&isequal(sort(fieldnames(binding)),sort({'kind';'path';'binary_sha256';'adapter_path';'adapter_sha256'})), ...
    'gpenmpcNative:LocalInputCodecBackendBinding','Explicit kind, binary path/hash and adapter path/hash required.');
kind=string(binding.kind);p=string(binding.path);h=string(binding.binary_sha256);
ap=string(binding.adapter_path);ah=string(binding.adapter_sha256);
assert(isscalar(kind)&&kind=="CANONICAL_LOCAL_TASK_WIRE_MEX"&&isscalar(p)&&isscalar(h)&&isscalar(ap)&&isscalar(ah) ...
    &&~isempty(regexp(char(p),'^[A-Za-z]:[\\/]','once'))&&~isempty(regexp(char(ap),'^[A-Za-z]:[\\/]','once')) ...
    &&strcmpi(h,approvedBinary)&&strcmpi(ah,approvedAdapter), ...
    'gpenmpcNative:LocalInputCodecBackendBinding','Only the exact validated RLI1 MEX and original-check adapter are accepted.');
assert(isfile(p)&&isfile(ap),'gpenmpcNative:LocalInputCodecBackendMissing','Explicit MEX and adapter must exist.');
build=string(fileparts(fileparts(fileparts(mfilename('fullpath')))));
expected=fullfile(build,'evidence','task_interface','task_wire', ...
    'canonical_local_task_wire_mex.mexw64');
expectedAdapter=fullfile(build,'tools','encode_canonical_local_task_mex.m');
p=canonicalPath(p);ap=canonicalPath(ap);
assert(strcmpi(p,canonicalPath(expected))&&strcmpi(ap,canonicalPath(expectedAdapter)), ...
    'gpenmpcNative:LocalInputCodecBackendPath','Validated evidence binary and tools adapter paths required.');
resolved=string(which('canonical_local_task_wire_mex'));resolvedAdapter=string(which('encode_canonical_local_task_mex'));
assert(strlength(resolved)>0&&strlength(resolvedAdapter)>0, ...
    'gpenmpcNative:LocalInputCodecBackendMissing','Caller must already place the explicit MEX and adapter on its owned path.');
resolved=canonicalPath(resolved);resolvedAdapter=canonicalPath(resolvedAdapter);
assert(strcmpi(p,resolved)&&strcmpi(ap,resolvedAdapter), ...
    'gpenmpcNative:LocalInputCodecBackendResolution','A different MEX or adapter resolves on the current path.');
assert(strcmpi(inputCodecSha(p),h)&&strcmpi(inputCodecSha(ap),ah), ...
    'gpenmpcNative:LocalInputCodecBackendHash','Exact validated binary and adapter checksums required.');
encoder=@canonical_local_task_wire_mex;adapter=@encode_canonical_local_task_mex;
assert(nargin(adapter)==6&&nargout(adapter)==2, ...
    'gpenmpcNative:LocalInputCodecBackendLoad','Original adapter signature required.');
% Load the MEX through its arity check before flight.
loaded=false;try,encoder();catch ex,loaded=strcmp(ex.identifier,'gpenmpcNative:LocalTaskWireMexArity');end
assert(loaded,'gpenmpcNative:LocalInputCodecBackendLoad','Bound MEX must return its original pre-codec arity rejection.');
receipt=struct('kind','CANONICAL_LOCAL_TASK_WIRE_MEX','path',p,'binary_sha256',upper(h), ...
    'adapter_path',ap,'adapter_sha256',upper(ah),'resolved_path',resolved,'resolved_adapter_path',resolvedAdapter, ...
    'binary_hash_checks',1,'adapter_hash_checks',1,'hash_scope','CONSTRUCTION_ONLY','construction_load_probe',true, ...
    'construction_codec_calls',0,'construction_numerical_calls',0,'runtime_file_checks',0,'fallback_allowed',false, ...
    'original_source_checks_retained',true,'io_callback_and_deadline_checks_retained',true);
end
function [encoder,adapter,receipt]=reusePreparedInputCodecBackend(prepared,binding)
assert(isstruct(prepared)&&isscalar(prepared) ...
    &&isequal(sort(fieldnames(prepared)),sort({'schema';'binding';'encoder';'adapter';'receipt'; ...
        'prepared_before_getter_producer';'authority_granted';'construction_codec_calls';'construction_numerical_calls'})) ...
    &&strcmp(prepared.schema,'GPENMPC_PREPARED_LOCAL_INPUT_CODEC_BACKEND_V1') ...
    &&isequaln(prepared.binding,binding)&&prepared.prepared_before_getter_producer ...
    &&~prepared.authority_granted&&prepared.construction_codec_calls==0 ...
    &&prepared.construction_numerical_calls==0 ...
    &&isa(prepared.encoder,'function_handle')&&isa(prepared.adapter,'function_handle') ...
    &&strcmp(func2str(prepared.encoder),'canonical_local_task_wire_mex') ...
    &&strcmp(func2str(prepared.adapter),'encode_canonical_local_task_mex'), ...
    'gpenmpcNative:LocalInputCodecPrepared','Exact pre-producer codec token required.');
r=prepared.receipt;
assert(isstruct(r)&&isscalar(r)&&strcmpi(r.binary_sha256,binding.binary_sha256) ...
    &&strcmpi(r.adapter_sha256,binding.adapter_sha256)&&strcmpi(r.path,binding.path) ...
    &&strcmpi(r.adapter_path,binding.adapter_path)&&r.binary_hash_checks==1 ...
    &&r.adapter_hash_checks==1&&r.construction_load_probe ...
    &&r.construction_codec_calls==0&&r.construction_numerical_calls==0 ...
    &&r.runtime_file_checks==0&&~r.fallback_allowed, ...
    'gpenmpcNative:LocalInputCodecPrepared','Prepared codec identity or no-work receipt changed.');
encoder=prepared.encoder;adapter=prepared.adapter;receipt=r;
receipt.prepared_before_getter_producer=true;
receipt.service_constructor_file_checks=0;
end
function p=canonicalPath(p)
f=java.io.File(char(p));p=string(f.getCanonicalPath());
end
function h=inputCodecSha(p)
f=fopen(p,'rb');assert(f>=0,'gpenmpcNative:LocalInputCodecBackendMissing');g=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),b=fread(f,1048576,'*uint8');md.update(typecast(b,'int8'));end
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
