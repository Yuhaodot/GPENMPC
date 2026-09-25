classdef RflySoleMavlinkTransport < handle
    % Own native UDP with MATLAB serialization and decoding.
    % Limit datagrams to 300 bytes and retain dequeue timestamps.
    properties (Access=private)
        Configuration
        Source
        Native
        Handle = uint64(0)
        Serializer = []
        OwnSerializer = false
        Decoder = []
        WireDecoder = []
        PayloadLayouts = {}
        Busy = false
        Closed = true
        Failed = false
        FirstFailure = struct()
        LastNative = struct()
        LastSend = struct()
        LastReceive = struct()
        LastClose = struct()
        LastFrames = struct([])
        LastForwardingFragments = struct([])
        LastSerializedFrame = uint8([])
        LastDequeueNs = uint64(0)
        SerializationAttempts = uint64(0)
        Serializations = uint64(0)
        AcceptedFrames = uint64(0)
        ForwardingFragmentCount = uint64(0)
        ForwardingFragmentBytes = uint64(0)
        Polls = uint64(0)
        ReceiveBatch = struct([])
        ReceiveCursor = 1
        ReceiveSnapshotTarget = 0
        ReceiveGpTargets = []
        SnapshotInProgress = false
        InlineGp = []
        InlineTunnel = []
        InlineReplies = {}
        InlineSending = false
        ContinuousGp = false
        DeferGpHistory = false
        ClosedGpFrames = {}
        SnapshotSendAgeHintNs = uint64(0)
    end
    methods
        function obj=RflySoleMavlinkTransport(cfg,dialect,serializer)
            % All configuration, binary, codec and ownership checks precede
            % open. The caller owns path setup; there is no fallback MEX.
            obj.Configuration=checkedConfiguration(cfg);
            [obj.Native,obj.Source]=checkedNative(cfg.source);
            assert(isa(dialect,'mavlinkdialect')&&isscalar(dialect), ...
                'gpenmpcNative:SoleTransportDialect','An official dialect is required.');
            % A private official decoder isolates its incremental parser
            % state from callers which also deserialize other byte streams.
            obj.Decoder=copy(dialect);
            % Resolve official payload definitions before opening transport.
            % Keep the MathWorks CRC, signature and stateful parser; specialize payload conversion.
            saved=saveobj(obj.Decoder);parser=saved.PayloadParser;
            definitions=parser.MessageDefinitions.Messages;
            [ids,order]=sort(definitions.ID);
            entries=zeros(1,7*definitions.Count);
            entries(1:7:end)=ids;
            entries(2:7:end)=definitions.CRCExtra(order);
            entries(3:7:end)=definitions.MinLength(order);
            entries(4:7:end)=definitions.MaxLength(order);
            obj.WireDecoder=uav.internal.MAVLinkSerializer;
            obj.WireDecoder.updateMessageEntries(entries);
            obj.WireDecoder.setVersion(saved.Version);
            if ~isempty(saved.SigningChannel)
                signing=saved.SigningChannel;
                obj.WireDecoder.setSigningChannel(signing.Stream.SigningStream, ...
                    signing.SystemID,signing.ComponentID,signing.LinkID);
            end
            obj.PayloadLayouts=cell(1,max(ids)+1);
            for msg=reshape(ids,1,[])
                entry=parser.MessagePrototypes.find(msg);
                entry.FieldNames=cellstr(entry.FieldNames);
                entry.FieldTypes=cellstr(entry.FieldTypes);
                entry.FieldEnds=cumsum(entry.FieldBytes);
                entry.FieldStarts=[1 entry.FieldEnds(1:end-1)+1];
                obj.PayloadLayouts{msg+1}=entry;
            end
            if nargin<3 || isempty(serializer)
                obj.Serializer=mavlinkio(dialect,'SystemID',cfg.local_system, ...
                    'ComponentID',cfg.local_component);
                obj.OwnSerializer=true;
            else
                assert(isa(serializer,'mavlinkio')&&isscalar(serializer) ...
                    &&isequal(serializer.Dialect,dialect), ...
                    'gpenmpcNative:SoleTransportSerializer','Borrow the matching official serializer.');
                obj.Serializer=serializer;
            end
            obj.checkSerializer();
            prior=obj.Native('status');
            assert(isstruct(prior)&&isscalar(prior)&&isfield(prior,'open')&&~prior.open, ...
                'gpenmpcNative:SoleTransportExistingOwner','A native owner is already open.');
            fields={'scope','allow_loopback','local_host','remote_host','local_port','remote_port'};
            nativeCfg=struct();
            for k=1:numel(fields),nativeCfg.(fields{k})=cfg.(fields{k});end
            try
                obj.Handle=obj.Native('open',nativeCfg);
                assert(isa(obj.Handle,'uint64')&&isscalar(obj.Handle)&&obj.Handle>0, ...
                    'gpenmpcNative:SoleTransportHandle','Native owner returned no exact handle.');
                obj.Closed=false;
                obj.captureNative();
                assert(obj.LastNative.open&&~obj.LastNative.failed, ...
                    'gpenmpcNative:SoleTransportOpen','Native owner did not open.');
            catch ex
                obj.latch(ex,'open');
                if obj.Handle>0
                    try,obj.close();catch,end
                end
                rethrow(ex)
            end
        end

        function attachInlineGp(obj,gp,deferHistory,snapshotSendAgeHintNs,rawHistoryLimit,receiveOnly)
            if nargin<3,deferHistory=false;end
            if nargin<4,snapshotSendAgeHintNs=uint64(0);end
            if nargin<5,rawHistoryLimit=uint64(0);end
            if nargin<6,receiveOnly=false;end
            assert(islogical(receiveOnly)&&isscalar(receiveOnly));
            assert(isa(rawHistoryLimit,'uint64')&&isscalar(rawHistoryLimit), ...
                'gpenmpcNative:SoleTransportConfiguration');
            if ~deferHistory,rawHistoryLimit=uint64(0);end
            assert(isa(snapshotSendAgeHintNs,'uint64')&&isscalar(snapshotSendAgeHintNs), ...
                'gpenmpcNative:SoleTransportConfiguration');
            obj.SnapshotSendAgeHintNs=snapshotSendAgeHintNs;
            assert(isempty(obj.InlineGp)&&~obj.Busy&&~obj.Closed ...
                &&isa(gp,'gpenmpcNative.RflyLocalGpService'),'gpenmpcNative:SoleTransportGpOwner');
            c=obj.Configuration;
            continuous=isfield(obj.LastNative,'continuous_gp');
            assert(~receiveOnly||continuous,'gpenmpcNative:SoleTransportReceiveOnly','RC needs the existing native continuous receiver.');
            assert(islogical(deferHistory)&&isscalar(deferHistory)&&(~deferHistory||continuous), ...
                'gpenmpcNative:SoleTransportGpOwner','Deferred history requires the verified continuous owner.');
            obj.DeferGpHistory=deferHistory;
            e=gp.checkReceiveEndpoint(c.local_system,c.local_component,c.remote_system,c.remote_component,continuous, ...
                @()obj.Native('gp_stop',obj.Handle));
            obj.InlineGp=gp;obj.InlineTunnel=createmsg(obj.Decoder,'TUNNEL');
            if continuous
                if receiveOnly
                    started=obj.Native('gp_start',obj.Handle,uint64([e.uid e.boot_generation e.confirmed_host_rx_ns]), ...
                        uint8([c.remote_system c.remote_component c.local_system c.local_component]),rawHistoryLimit,true);
                    assert(~started.continuous_gp.prediction_enabled,'gpenmpcNative:SoleTransportReceiveOnly');
                else
                    started=obj.Native('gp_start',obj.Handle,uint64([e.uid e.boot_generation e.confirmed_host_rx_ns]), ...
                        uint8([c.remote_system c.remote_component c.local_system c.local_component]),rawHistoryLimit);
                end
                assert(started.continuous_gp.active&&~started.failed,'gpenmpcNative:SoleTransportGpStart');
                obj.ContinuousGp=true;
            end
        end

        function result=sendMessage(obj,message,receiverOwned)
            if nargin<3,receiverOwned=false;end
            if receiverOwned
                assert(obj.Busy&&obj.InlineSending&&~obj.Closed&&~obj.Failed, ...
                    'gpenmpcNative:SoleTransportGpOwner','Only the existing receive owner may perform this internal send.');
            else
                obj.enter();guard=onCleanup(@()obj.leave()); %#ok<NASGU>
            end
            try
                % Keep the internal serializer private and unconnected.
                % Check connection mutations only for externally borrowed objects.
                if ~obj.OwnSerializer,obj.checkSerializer();end
                obj.SerializationAttempts=increment(obj.SerializationAttempts);
                % Serialize once and retain the packet after send failure.
                bytes=serializemsg(obj.Serializer,message);
                assert(isa(bytes,'uint8')&&isvector(bytes)&&~isempty(bytes), ...
                    'gpenmpcNative:SoleTransportSerialization','Official serializer returned invalid bytes.');
                obj.LastSerializedFrame=bytes(:);
                obj.Serializations=increment(obj.Serializations);
                % Delegate oversize rejection to the native owner so its
                % attempted=false/full-prefix failure ledger is preserved.
                result=obj.Native('send',obj.Handle,bytes(:));
                obj.LastSend=result;
                sameWire=isequal(result.bytes(:),bytes(:));
                if obj.ContinuousGp&&numel(result.bytes)==numel(bytes)&&numel(bytes)>=12&&bytes(1)==253&&bytes(3)==0
                    % The SAME native socket assigns one Seq across both
                    % producers under one lock. Only Seq+official CRC change;
                    % original payload, sender, length and command are exact.
                    keep=true(numel(bytes),1);keep([5 end-1 end])=false;
                    sameWire=isequal(result.bytes(keep),reshape(bytes(keep),[],1));
                end
                assert(isstruct(result)&&isscalar(result)&&result.attempted&&result.ok ...
                    &&result.bytes_complete&&sameWire ...
                    &&result.bytes_requested==uint64(numel(bytes)) ...
                    &&result.bytes_sent==int32(numel(bytes)), ...
                    'gpenmpcNative:SoleTransportSend','Original full-wire send did not complete.');
                checkInterval(result.submit_ns,result.return_ns,result.submit_qpc, ...
                    result.return_qpc,result.qpc_frequency);
            catch ex
                obj.captureNativeSafely();
                obj.latch(ex,'sendMessage');
                rethrow(ex)
            end
        end

        function [records,fragments,snapshotPending]=poll(obj,deliverRecord)
            if nargin<2,deliverRecord=[];end
            snapshotPending=obj.SnapshotInProgress;
            assert(isempty(deliverRecord)||isa(deliverRecord,'function_handle'));
            obj.enter();guard=onCleanup(@()obj.leave()); %#ok<NASGU>
            obj.LastFrames=emptyFrames();
            lastFrames=emptyFrames();
            obj.LastForwardingFragments=emptyForwardingFragments();
            datagramIndex=0;offset=0;
            prefixRecords=emptyFrames();prefixFragments=emptyForwardingFragments();
            try
                if ~obj.OwnSerializer,obj.checkSerializer();end
                obj.Polls=increment(obj.Polls);
                % In the local runtime, parsing and its read-only consumer
                % share one work slice. Otherwise a bounded decode batch is
                % followed by another unbounded callback batch before RLS
                % can become an input. No receive/send reentry is permitted.
                combinedWork=tic;
                for pollPass=1:2
                if pollPass==2
                    lastFrames=emptyFrames();obj.LastForwardingFragments=emptyForwardingFragments();
                end
                if isempty(obj.ReceiveBatch)
                    obj.ReceiveBatch=obj.Native('receive',obj.Handle,obj.Configuration.maximum_poll_datagrams);
                    obj.ReceiveCursor=1;
                    % Process native-reply bookkeeping incrementally so history does not delay fresh RLS.
                    if ~obj.ContinuousGp,obj.serviceInlineGp(obj.ReceiveBatch);end
                    [obj.ReceiveSnapshotTarget,~,obj.ReceiveGpTargets]= ...
                        lastSnapshotHint(obj.ReceiveBatch,1,obj.Configuration);
                    % Retain the batch, timestamps and cursor across bounded polls.
                    % Use receive's fault checks directly; status/error/close retain full diagnostics.
                    % Keep newer complete-state hints visible across GP/history processing.
                end
                if obj.ContinuousGp&&~isempty(obj.ReceiveBatch) ...
                        &&numel(obj.ReceiveBatch)<obj.Configuration.maximum_poll_datagrams ...
                        &&obj.ReceiveSnapshotTarget<obj.ReceiveCursor
                    % Refill history or partial-only tails once, but first deliver any complete RLS
                    % already in the batch. Preserve the consumed prefix, capacity, timestamps
                    % and total work-slice bound.
                    more=obj.Native('receive',obj.Handle,obj.Configuration.maximum_poll_datagrams-numel(obj.ReceiveBatch));
                    if ~isempty(more)
                        obj.ReceiveBatch=[obj.ReceiveBatch;more];
                        [obj.ReceiveSnapshotTarget,~,obj.ReceiveGpTargets]= ...
                            lastSnapshotHint(obj.ReceiveBatch,obj.ReceiveCursor,obj.Configuration);
                    end
                end
                data=obj.ReceiveBatch;
                assert(isstruct(data)&&numel(data)<=obj.Configuration.maximum_poll_datagrams, ...
                    'gpenmpcNative:SoleTransportBatch','Invalid native datagram batch.');
                % Process by elapsed work within an 8 ms slice, completing each datagram atomically.
                pollWork=tic;
                if ~isempty(deliverRecord),pollWork=combinedWork;end
                deliverSnapshot=false;deliverGp=false;yieldSnapshot=false;
                partialSnapshot=false;
                lastDatagram=numel(data);
                % Prefer the latest complete RLS in the batch without waiting for an incomplete successor.
                % Cache header hints while retaining ordered CRC and source validation.
                snapshotTarget=obj.ReceiveSnapshotTarget;
                if snapshotTarget<obj.ReceiveCursor,snapshotTarget=0;end
                gpTarget=0;gpRemaining=obj.ReceiveGpTargets>=obj.ReceiveCursor;
                if any(gpRemaining),gpTarget=obj.ReceiveGpTargets(find(gpRemaining,1));end
                for datagramIndex=obj.ReceiveCursor:lastDatagram
                    d=data(datagramIndex);offset=1;
                    obj.checkDatagram(d);
                    bytes=d.bytes(:);
                    if obj.ContinuousGp&&(~obj.DeferGpHistory||~d.native_gp_history_only)&&~isempty(d.native_gp)
                        obj.serviceInlineGp(d);
                    end
                    if obj.DeferGpHistory&&isfield(d,'native_gp_history_only')&&d.native_gp_history_only
                        % Preserve native-processed GP bytes and timestamps; reconstruct history after close.
                        r=frameRecord(d,datagramIndex,1,bytes);r.validated=true;
                        r.native_gp_history_only=true;
                        obj.AcceptedFrames=increment(obj.AcceptedFrames);r.transport_frame_sequence=obj.AcceptedFrames;
                        if ~isempty(d.native_gp)
                            % Retain the native prediction and send result without extra history conversion.
                            q=d.native_gp;
                            r.inline_gp=struct('query',q,'computed',[],'send',q.send);
                        end
                        lastFrames(end+1,1)=r;
                        if ~isempty(deliverRecord),deliverRecord(r);end
                        if toc(pollWork)>=.008,break,end
                        continue
                    end
                    while offset<=numel(bytes)
                        % Record the remainder before validation, so invalid
                        % magic/header/truncation/unknown MsgID is not lost.
                        r=frameRecord(d,datagramIndex,offset,bytes(offset:end));
                        lastFrames(end+1,1)=r;
                        duplicateWitness=[];
                        try
                            [length,sys,component,msgid,fragmentReason]=frameShape( ...
                                bytes,offset,obj.Configuration.copter_serial_forwarding);
                        catch ex
                            % A forwarding slice can resemble MAVLink2 magic. Classify it as payload only
                            % when a source-matched, CRC-valid frame contains all bytes strictly inside its payload.
                            if obj.Configuration.copter_serial_forwarding ...
                                    &&strcmp(ex.identifier,'gpenmpcNative:SoleTransportFlags')
                                duplicateWitness=sameBatchSubstringWitness( ...
                                    data,datagramIndex,offset,obj.Decoder,obj.Configuration);
                                if ~isempty(duplicateWitness)&&offset==1 ...
                                        &&duplicateWitness.fragment_start_1based>10 ...
                                        &&duplicateWitness.fragment_end_1based<=numel(duplicateWitness.witness_raw_frame)-2
                                    length=0;sys=0;component=0;msgid=0;
                                    fragmentReason= ...
                                        'EXACT_PAYLOAD_SLICE_OF_SAME_BATCH_CRC0_SOURCE_MATCHED_FRAME';
                                elseif boundedIncompleteContinuation(data,datagramIndex,offset,obj.Configuration)
                                    % Retain a payload continuation as raw bytes rather than treating it as a new header.
                                    length=0;sys=0;component=0;msgid=0;
                                    fragmentReason='UNDECODED_INCOMPLETE_SERIAL_FRAME_CONTINUATION';
                                elseif offset==1&&numel(bytes)>=12 ...
                                        &&(bytes(6)~=obj.Configuration.remote_system||bytes(7)~=obj.Configuration.remote_component)
                                    % An FD-prefixed serial remainder may follow a prefix received earlier.
                                    % Retain it undecoded; expected-source headers still require supported flags and valid CRC.
                                    length=0;sys=0;component=0;msgid=0;
                                    fragmentReason='UNDECODED_SERIAL_REMAINDER_INVALID_SOURCE_HEADER';
                                else
                                    rethrow(ex)
                                end
                            else
                                rethrow(ex)
                            end
                        end
                        if obj.Configuration.copter_serial_forwarding&&isempty(fragmentReason) ...
                                &&length<numel(bytes)-offset+1
                            % A complete datagram-sized frame is NEVER exempt
                            % from its own CRC/source checks by substring proof.
                            duplicateWitness=sameBatchSubstringWitness( ...
                                data,datagramIndex,offset,obj.Decoder,obj.Configuration);
                            if ~isempty(duplicateWitness)
                                fragmentReason='EXACT_PROPER_SUBSTRING_OF_SAME_BATCH_CRC0_SOURCE_MATCHED_FRAME';
                            end
                        end
                        if ~isempty(fragmentReason)
                            % Retain forwarding remainders without concatenating datagrams or resynchronizing.
                            obj.ForwardingFragmentCount=increment(obj.ForwardingFragmentCount);
                            fragmentBytes=uint64(numel(bytes)-offset+1);
                            assert(obj.ForwardingFragmentBytes<=intmax('uint64')-fragmentBytes, ...
                                'gpenmpcNative:SoleTransportCount','Forwarding byte counter overflow.');
                            obj.ForwardingFragmentBytes=obj.ForwardingFragmentBytes+fragmentBytes;
                            obj.LastForwardingFragments(end+1,1)=forwardingFragmentRecord( ...
                                d,datagramIndex,offset,bytes(offset:end),fragmentReason, ...
                                obj.ForwardingFragmentCount,duplicateWitness);
                            % Delete a ROW: linear deletion of the sole item
                            % leaves 1-by-0; the next (end+1,1) append would
                            % then create a blank leading record.
                            lastFrames(end,:)=[]; % Moved intact to the separate raw fragment ledger.
                            break
                        end
                        raw=bytes(offset:offset+length-1);
                        lastFrames(end).raw_frame=raw;
                        decodeError=[];
                        try
                            if obj.ContinuousGp&&offset==1&&length==numel(bytes)&&msgid==385 ...
                                    &&isfield(d,'native_tunnel_payload')&&numel(d.native_tunnel_payload)==133
                                % Reuse the padded payload from the validated native frame.
                                b=d.native_tunnel_payload;
                                payload=obj.PayloadLayouts{386}.Prototype;
                                payload.payload_type=typecast(b(1:2),'uint16');
                                payload.target_system=b(3);payload.target_component=b(4);
                                payload.payload_length=b(5);payload.payload=reshape(b(6:133),1,128);
                                message=uav.internal.mavlink.Structures.createMsgExt(1);
                                message.MsgID=uint32(msgid);message.SystemID=uint8(sys);
                                message.ComponentID=uint8(component);message.Seq=raw(5);
                                message.Payload=payload;parseStatus=0;
                            else
                                [message,parseStatus]=obj.decodeFrame(raw.');
                            end
                        catch problem
                            % For unknown-ID decode errors, apply only the bounded incomplete-prefix classification.
                            if obj.Configuration.copter_serial_forwarding&& ...
                                    strcmp(problem.identifier,'uav:robotuav:mavlink:UndefinedMessageIDToParse')
                                decodeError=problem;message=[];parseStatus=1;
                            else
                                rethrow(problem)
                            end
                        end
                        lastFrames(end).official_parse_status=parseStatus;
                        lastFrames(end).decoded_message=message;
                        if obj.Configuration.copter_serial_forwarding&&isscalar(parseStatus)&&parseStatus~=0 ...
                                &&offset==1&&datagramIndex>1 ...
                                &&(sys~=obj.Configuration.remote_system||component~=obj.Configuration.remote_component)
                            % Retain an FE-prefixed incomplete payload slice without accepting it as MAVLink1.
                            if boundedIncompleteContinuation(data,datagramIndex,offset,obj.Configuration)
                                obj.ForwardingFragmentCount=increment(obj.ForwardingFragmentCount);
                                obj.ForwardingFragmentBytes=obj.ForwardingFragmentBytes+uint64(numel(bytes));
                                obj.LastForwardingFragments(end+1,1)=forwardingFragmentRecord( ...
                                    d,datagramIndex,offset,bytes,'UNDECODED_INCOMPLETE_SERIAL_FRAME_CONTINUATION', ...
                                    obj.ForwardingFragmentCount,[]);
                                lastFrames(end,:)=[];break
                            end
                        end
                        if ~isempty(decodeError),rethrow(decodeError);end
                        assert(isstruct(message)&&isscalar(message) ...
                            &&all(isfield(message,{'MsgID','SystemID','ComponentID','Payload'})) ...
                            &&isscalar(parseStatus)&&parseStatus==0 ...
                            &&message.MsgID==msgid&&message.SystemID==sys ...
                            &&message.ComponentID==component, ...
                            'gpenmpcNative:SoleTransportDecode','Official MAVLink CRC/signature/identity validation failed.');
                        assert(sys==obj.Configuration.remote_system ...
                            &&component==obj.Configuration.remote_component, ...
                            'gpenmpcNative:SoleTransportMavlinkSource','Unexpected MAVLink system/component.');
                        lastFrames(end).validated=true;
                        obj.AcceptedFrames=increment(obj.AcceptedFrames);
                        lastFrames(end).transport_frame_sequence=obj.AcceptedFrames;
                        if msgid==385&&~isempty(obj.InlineReplies)
                            for inlineIndex=1:numel(obj.InlineReplies)
                                inline=obj.InlineReplies{inlineIndex};
                                if isequal(raw(:),inline.query.raw_frames{3}(:)) ...
                                        &&d.dequeue_ns==inline.query.fragment_rx_ns(3)
                                    lastFrames(end).inline_gp=inline;
                                    obj.InlineReplies(inlineIndex)=[];break
                                end
                            end
                        end
                        if ~isempty(deliverRecord)
                            % The ordinary CRC/source-checked original record,
                            % including its actual dequeue time, is delivered
                            % once. The callback only updates the SAME IO's
                            % receive state; it cannot send or run a controller.
                            deliverRecord(lastFrames(end));
                        end
                        % A CRC/source-checked final RLS1 fragment is only a
                        % scheduling hint. Its original body/order/identity
                        % still passes the existing reassembler afterwards.
                        % Finish this datagram, then deliver without decoding
                        % unrelated following datagrams for the rest of 8 ms.
                        if msgid==385
                            p=message.Payload;
                            % The completed original GP request has the next
                            % control opportunity as its numerical deadline.
                            % Yield this SAME receiver after its final frame;
                            % the normal reassembler still validates all three
                            % fragments before the caller may compute/send.
                            % Inline GP has ALREADY computed/sent in this same
                            % receive batch. Its later history callback must
                            % not force a pump round-trip before the following
                            % fresh RLS. Preserve legacy synchronous yielding.
                            deliverGp=deliverGp || (isempty(obj.InlineGp)&&p.payload_type==42002 ...
                                &&p.payload_length==81 ...
                                &&p.target_system==obj.Configuration.local_system ...
                                &&p.target_component==obj.Configuration.local_component ...
                                &&p.payload(1)==uint8(130));
                            isSnapshot=p.payload_type==42002 ...
                                &&p.target_system==obj.Configuration.local_system ...
                                &&p.target_component==obj.Configuration.local_component ...
                                &&p.payload(1)>=uint8(160)&&p.payload(1)<=uint8(163);
                            if isSnapshot
                                % Track the newest observed message, rather
                                % than whether ANY earlier RLS completed.
                                partialSnapshot=p.payload(1)~=uint8(163);
                                obj.SnapshotInProgress=partialSnapshot;
                            end
                            completedSnapshot=p.payload_type==42002 && p.payload_length==34 ...
                                &&p.target_system==obj.Configuration.local_system ...
                                &&p.target_component==obj.Configuration.local_component ...
                                &&p.payload(1)==uint8(163); % schema 10, index 3/4
                            deliverSnapshot=deliverSnapshot||completedSnapshot;
                            if completedSnapshot
                                % Skip input-binding yield for an already-stale history sample.
                                % Last-fragment age is a lower bound; validate original-source age before send.
                                yieldSnapshot=obj.SnapshotSendAgeHintNs==0 || ...
                                    gpenmpcNative.rflyOriginalHostMonotonicNs()<= ...
                                    d.dequeue_ns+obj.SnapshotSendAgeHintNs;
                            end
                        end
                        offset=offset+length;
                    end
                    % Finish the datagram at the work-slice boundary, then retain the tail and cursor.
                    % Prefer the newest completed snapshot within the slice without waiting for future data.
                    if deliverGp||toc(pollWork)>=0.008|| ...
                            (deliverSnapshot&&(~obj.ContinuousGp||yieldSnapshot)&&datagramIndex>=snapshotTarget&&gpTarget<=datagramIndex),break,end
                end
                if ~isempty(data),obj.ReceiveCursor=datagramIndex+1;end
                % Expose a pending complete successor so the caller can service environment
                % and heartbeat before resuming the same cursor.
                snapshotPending=obj.SnapshotInProgress|| ...
                    (snapshotTarget>0&&obj.ReceiveCursor<=snapshotTarget);
                if ~isempty(deliverRecord)&&deliverSnapshot
                    % Allow one more bounded slice for a known completed successor,
                    % not an unfinished or hypothetical message.
                    snapshotPending=snapshotTarget>0&&obj.ReceiveCursor<=snapshotTarget;
                end
                if obj.ReceiveCursor>numel(data),obj.ReceiveBatch=struct([]);obj.ReceiveCursor=1;end
                records=lastFrames;
                fragments=obj.LastForwardingFragments;
                % After draining a partial or history-only batch, refill once nonblocking
                % within the same budget. Return immediately when a complete state is usable.
                runtimeTail=~isempty(deliverRecord)&&~isempty(data)&& ...
                    (~deliverSnapshot||(obj.ContinuousGp&&~yieldSnapshot));
                if pollPass==1&&~deliverGp&&(partialSnapshot||runtimeTail)&&isempty(obj.ReceiveBatch) ...
                        &&(isempty(deliverRecord)||toc(combinedWork)<0.008)
                    prefixRecords=records;prefixFragments=fragments;
                    continue
                end
                records=[prefixRecords;records];fragments=[prefixFragments;fragments];
                obj.LastFrames=records;obj.LastForwardingFragments=fragments;
                break
                end
            catch ex
                obj.LastFrames=[prefixRecords;lastFrames];
                obj.LastForwardingFragments=[prefixFragments;obj.LastForwardingFragments];
                obj.captureNativeSafely();
                if ~isempty(obj.LastFrames)&&~obj.LastFrames(end).validated
                    obj.LastFrames(end).validation_error=ex.identifier;
                end
                obj.latch(ex,'poll',datagramIndex,offset);
                rethrow(ex)
            end
        end

        function result=status(obj)
            obj.captureNativeSafely();
            result=struct('transport_kind','OFFICIAL_CODEC_SOLE_RAW_UDP', ...
                'same_existing_mavlinkio',false,'same_existing_transport_owner',true, ...
                'scope',char(obj.Configuration.scope),'source',obj.Source, ...
                'handle',obj.Handle,'open',~obj.Closed,'closed',obj.Closed, ...
                'failed',obj.Failed,'first_error',obj.FirstFailure, ...
                'last_send',obj.LastSend,'last_receive',obj.LastReceive, ...
                'last_close',obj.LastClose,'last_frames',obj.LastFrames, ...
                'last_forwarding_fragments',obj.LastForwardingFragments, ...
                'inline_gp_not_yet_callback_recorded',{obj.InlineReplies}, ...
                'forwarding_fragment_count',obj.ForwardingFragmentCount, ...
                'forwarding_fragment_bytes',obj.ForwardingFragmentBytes, ...
                'copter_serial_forwarding',obj.Configuration.copter_serial_forwarding, ...
                'forwarding_reassembly_performed',false, ...
                'last_serialized_frame',obj.LastSerializedFrame, ...
                'serialization_attempts',obj.SerializationAttempts, ...
                'serializations',obj.Serializations,'accepted_frames',obj.AcceptedFrames, ...
                'polls',obj.Polls,'native',obj.LastNative, ...
                'original_receive_time_semantics','HOST_DATAGRAM_DEQUEUE_OBSERVATION_NOT_KERNEL_ARRIVAL', ...
                'maximum_datagram_bytes',300, ...
                'maximum_poll_datagrams',obj.Configuration.maximum_poll_datagrams, ...
                'raw_retention','LIVE_STATE_BATCH__SERVICED_GP_RAW_RECONSTRUCTED_AFTER_CLOSE', ...
                'closed_gp_frames',{obj.ClosedGpFrames}, ...
                'limit_provenance','HOST_ADAPTER_ENGINEERING_BOUNDS_NOT_HARDWARE_LIMITS', ...
                'owns_serializer',obj.OwnSerializer,'clock_conversion',false, ...
                'signature_authentication_claimed',false,'control_authority',false);
        end

        function result=close(obj)
            assert(~obj.Busy,'gpenmpcNative:SoleTransportBusy','Cannot close an active owner call.');
            if obj.Handle==0
                result=struct('attempted',false,'already_closed',true,'returned',true, ...
                    'ok',true,'closed',true);
                obj.LastClose=result;return
            end
            try
                result=obj.Native('close',obj.Handle);
                obj.captureNative();
                result.closed=logical(obj.LastNative.closed);
                obj.LastClose=result;obj.Closed=result.closed;
                assert(result.ok&&result.closed,'gpenmpcNative:SoleTransportClose','Native close did not complete.');
                if obj.DeferGpHistory&&isempty(obj.ClosedGpFrames) ...
                        &&isfield(obj.LastNative.continuous_gp,'deferred_history_chunks')
                    chunks=obj.LastNative.continuous_gp.deferred_history_chunks;
                    previousHistoryNs=uint64(0);
                    for j=1:numel(chunks)
                        batch=chunks{j};
                        for k=1:numel(batch)
                            d=batch(k);obj.checkDatagram(d,false);
                            assert(d.dequeue_ns>=previousHistoryNs,'gpenmpcNative:SoleTransportTime');
                            previousHistoryNs=d.dequeue_ns;
                            assert(d.native_gp_history_only,'gpenmpcNative:SoleTransportGpHistory');
                            r=frameRecord(d,k,1,d.bytes);r.validated=true;r.native_gp_history_only=true;
                            % Use sequence 0 for post-run history and retain original receive/QPC time.
                            if ~isempty(d.native_gp)
                                r.inline_gp=struct('query',d.native_gp,'computed',[],'send',d.native_gp.send);
                            end
                            obj.ClosedGpFrames{end+1}=r;
                        end
                    end
                end
            catch ex
                obj.captureNativeSafely();obj.latch(ex,'close');rethrow(ex)
            end
        end

        function delete(obj)
            if obj.Handle>0 && ~obj.Closed
                try,obj.close();catch,end
            end
            if obj.OwnSerializer && ~isempty(obj.Serializer)
                try,delete(obj.Serializer);catch,end
            end
            if ~isempty(obj.Decoder)
                try,delete(obj.Decoder);catch,end
            end
        end
    end
    methods (Access=private)
        function serviceInlineGp(obj,data)
            if isempty(obj.InlineGp)||obj.InlineGp.Closed||isempty(data),return,end
            completed=obj.InlineGp.receiveBatch(data,obj.DeferGpHistory);
            for k=1:numel(completed)
                assert(numel(obj.InlineReplies)<64,'gpenmpcNative:SoleTransportGpBound');
                if obj.ContinuousGp
                    r=completed{k};assert(r.send.messages_send_returned==3&&all(r.send.returned_ns>=r.send.submitted_ns), ...
                        'gpenmpcNative:SoleTransportGpSend');
                    obj.InlineReplies{end+1}=r;continue % Retain the inline reply.
                end
                r=completed{k};r.send=struct('messages_send_returned',0, ...
                    'submitted_ns',zeros(3,1,'uint64'),'returned_ns',zeros(3,1,'uint64'), ...
                    'native',{{}},'same_existing_transport_owner',true, ...
                    'board_receipt_proven',false,'control_authority',false, ...
                    'status','RECEIVE_INLINE_ORIGINAL_GP_REPLY');
                obj.InlineReplies{end+1}=r;at=numel(obj.InlineReplies);
                expiry=r.query.original_host_receive_ns+uint64(200000000);
                obj.InlineSending=true;g=onCleanup(@()obj.finishInlineSend());
                for fragment=1:3
                    now=gpenmpcNative.rflyOriginalHostMonotonicNs();
                    assert(now>=r.query.original_host_receive_ns&&now<=expiry, ...
                        'gpenmpcNative:LocalGpSendExpired','Original GP transport age retained.');
                    p=r.computed.tunnel_payloads{fragment};message=obj.InlineTunnel;
                    for name={'target_system','target_component','payload_type','payload_length','payload'}
                        message.Payload.(name{1})(:)=p.(name{1})(:);
                    end
                    % Exactly the same official serializer, MAVLink sequence,
                    % native socket and full-send checks as every other packet.
                    native=obj.sendMessage(message,true);
                    obj.InlineReplies{at}.send.native{fragment}=native;
                    obj.InlineReplies{at}.send.submitted_ns(fragment)=native.submit_ns;
                    obj.InlineReplies{at}.send.returned_ns(fragment)=native.return_ns;
                    obj.InlineReplies{at}.send.messages_send_returned=fragment;
                    assert(native.return_ns<=expiry,'gpenmpcNative:LocalGpSendExpired');
                end
                clear g
            end
        end
        function finishInlineSend(obj),obj.InlineSending=false;end
        function [messages,status]=decodeFrame(obj,raw)
            % Exactly the official deserializemsg native parser and status
            % convention. Native payloads are zero-padded uint8 rows, never
            % Simulink fixed-point objects. No CRC/source/age is bypassed.
            decoded=obj.WireDecoder.deserialize(raw);
            packets=decoded{1};status=double(decoded{2})-1;
            messages=uav.internal.mavlink.Structures.createMsgExt(numel(status));
            for k=1:numel(packets)
                p=packets(k);id=double(p.MsgID);
                if id+1>numel(obj.PayloadLayouts)||isempty(obj.PayloadLayouts{id+1})
                    error('uav:robotuav:mavlink:UndefinedMessageIDToParse', ...
                        'Unknown message ID %u in the bound official dialect.',id);
                end
                layout=obj.PayloadLayouts{id+1};payload=layout.Prototype;
                for f=1:layout.FieldCount
                    bytes=p.Payload(:,layout.FieldStarts(f):layout.FieldEnds(f));
                    if strcmp(layout.FieldTypes{f},'char'),value=char(bytes);
                    else,value=reshape(typecast(bytes(:),layout.FieldTypes{f}),1,[]);end
                    payload.(layout.FieldNames{f})=value;
                end
                messages(k).MsgID=p.MsgID;messages(k).SystemID=p.SystemID;
                messages(k).ComponentID=p.ComponentID;messages(k).Seq=p.Seq;
                messages(k).Payload=payload;
            end
        end
        function checkSerializer(obj)
            assert(isvalid(obj.Serializer)&&isempty(listConnections(obj.Serializer)), ...
                'gpenmpcNative:SoleTransportSerializerConnected','Serializer must have no transport connections.');
            c=obj.Serializer.LocalClient;
            assert(c.SystemID==obj.Configuration.local_system ...
                &&c.ComponentID==obj.Configuration.local_component, ...
                'gpenmpcNative:SoleTransportSerializerIdentity','Serializer system/component changed.');
        end
        function enter(obj)
            if obj.Busy
                ex=MException('gpenmpcNative:SoleTransportBusy','Single owner call is already active.');
                obj.latch(ex,'enter');throw(ex)
            end
            assert(~obj.Closed&&~obj.Failed,'gpenmpcNative:SoleTransportUnavailable', ...
                'Transport is closed or permanently failed; status/close remain available.');
            obj.Busy=true;
        end
        function leave(obj),obj.Busy=false;end
        function latch(obj,ex,operation,datagram,offset)
            if nargin<4,datagram=0;end
            if nargin<5,offset=0;end
            if ~obj.Failed
                obj.FirstFailure=struct('identifier',ex.identifier,'message',ex.message, ...
                    'operation',operation,'datagram_index',datagram,'frame_offset',offset);
            end
            obj.Failed=true;
        end
        function captureNative(obj)
            if obj.Handle==0,return,end
            s=obj.Native('status',obj.Handle);
            assert(isa(s.handle,'uint64')&&isequal(s.handle,obj.Handle), ...
                'gpenmpcNative:SoleTransportOwner','Native state belongs to another owner.');
            obj.LastNative=s;
            if isfield(s,'last_send'),obj.LastSend=s.last_send;end
            if isfield(s,'last_receive'),obj.LastReceive=s.last_receive;end
            if isfield(s,'last_close'),obj.LastClose=s.last_close;end
            if s.failed && ~obj.Failed
                obj.latch(MException('gpenmpcNative:SoleTransportNativeFailure', ...
                    'Native transport failed: %s',char(s.failure_code)),'native');
            end
            if s.closed,obj.Closed=true;end
        end
        function captureNativeSafely(obj)
            try,obj.captureNative();catch ex,obj.latch(ex,'status');end
        end
        function checkDatagram(obj,d,liveOrder)
            if nargin<3,liveOrder=true;end
            assert(d.attempted&&d.ok&&d.bytes_complete&&d.source_matched ...
                &&~d.empty_would_block&&isa(d.bytes,'uint8')&&isvector(d.bytes) ...
                &&numel(d.bytes)>=1&&numel(d.bytes)<=300 ...
                &&d.received_bytes==int32(numel(d.bytes)) ...
                &&strcmp(d.source_ip,obj.Configuration.remote_host) ...
                &&d.source_port==obj.Configuration.remote_port, ...
                'gpenmpcNative:SoleTransportDatagram','Invalid or mismatched native datagram.');
            checkInterval(d.submit_ns,d.dequeue_ns,d.submit_qpc,d.dequeue_qpc,d.qpc_frequency);
            if liveOrder
                assert(d.dequeue_ns>=obj.LastDequeueNs,'gpenmpcNative:SoleTransportTime', ...
                    'Original native dequeue observation regressed.');
                obj.LastDequeueNs=d.dequeue_ns;
            end
        end
    end
end

function cfg=checkedConfiguration(cfg)
required={'source','scope','allow_loopback','local_host','remote_host','local_port', ...
    'remote_port','local_system','local_component','remote_system','remote_component', ...
    'maximum_poll_datagrams'};
assert(isstruct(cfg)&&isscalar(cfg) ...
    &&(isequal(sort(fieldnames(cfg)),sort(required(:))) ...
    ||isequal(sort(fieldnames(cfg)),sort([required(:);{'copter_serial_forwarding'}]))), ...
    'gpenmpcNative:SoleTransportConfiguration','Explicit transport-only configuration is required.');
if ~isfield(cfg,'copter_serial_forwarding'),cfg.copter_serial_forwarding=false;end
assert(islogical(cfg.copter_serial_forwarding)&&isscalar(cfg.copter_serial_forwarding), ...
    'gpenmpcNative:SoleTransportConfiguration','Serial forwarding compatibility must be an explicit scalar logical.');
assert(ismember(string(cfg.scope),["HOST_ONLY_LOOPBACK","COPTERSIM_EXISTING_MAVLINK"])&&isequal(cfg.allow_loopback,true) ...
    &&isequal(string(cfg.local_host),"127.0.0.1")&&isequal(string(cfg.remote_host),"127.0.0.1"), ...
    'gpenmpcNative:SoleTransportScope','Only explicit localhost endpoints are supported.');
if string(cfg.scope)=="HOST_ONLY_LOOPBACK"
 for f={'local_port','remote_port'},assertInteger(cfg.(f{1}),62200,62399);end
else
 assert(isequal(double(cfg.local_port),14550)&&isequal(double(cfg.remote_port),18570) ...
  &&isequal(double(cfg.local_system),255)&&isequal(double(cfg.local_component),190) ...
  &&isequal(double(cfg.remote_system),1)&&isequal(double(cfg.remote_component),1), ...
  'gpenmpcNative:SoleTransportConfiguration','Exact existing CopterSim MAVLink endpoint required.');
end
assert(cfg.local_port~=cfg.remote_port,'gpenmpcNative:SoleTransportConfiguration','Ports must differ.');
for f={'local_system','remote_system'},assertInteger(cfg.(f{1}),1,255);end
for f={'local_component','remote_component'},assertInteger(cfg.(f{1}),0,255);end
assertInteger(cfg.maximum_poll_datagrams,1,64);
end
function assertInteger(x,lo,hi)
assert(isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x==fix(x)&&x>=lo&&x<=hi, ...
    'gpenmpcNative:SoleTransportConfiguration','Invalid integer transport field.');
end
function [native,source]=checkedNative(source)
assert(isstruct(source)&&isscalar(source) ...
    &&isequal(sort(fieldnames(source)),sort({'kind';'exact_path';'sha256'})) ...
    &&isequal(string(source.kind),"GPENMPC_RFLY_UDP_TRANSPORT_MEX"), ...
    'gpenmpcNative:SoleTransportSource','Exact native source binding is required.');
p=string(source.exact_path);digest=string(source.sha256);
assert(isscalar(p)&&~isempty(regexp(char(p),'^[A-Za-z]:[\\/]','once')) ...
    &&isscalar(digest)&&~isempty(regexp(char(digest),'^[0-9A-Fa-f]{64}$','once')) ...
    &&isfile(p),'gpenmpcNative:SoleTransportSource','Missing exact absolute MEX path/hash.');
[~,name,extension]=fileparts(p);
assert(name=="gpenmpc_rfly_udp_transport_mex"&&strcmpi(extension,'.mexw64'), ...
    'gpenmpcNative:SoleTransportSource','Unexpected native entry or extension.');
file=java.io.File(char(p));p=string(file.getCanonicalPath());
resolved=string(which('gpenmpc_rfly_udp_transport_mex'));
assert(strlength(resolved)>0&&exist('gpenmpc_rfly_udp_transport_mex','file')==3, ...
    'gpenmpcNative:SoleTransportSource','Bound MEX must already be on the caller-owned path.');
file=java.io.File(char(resolved));resolved=string(file.getCanonicalPath());
assert(strcmpi(p,resolved),'gpenmpcNative:SoleTransportShadow','A different native owner resolves on the path.');
assert(strcmpi(fileSha(p),digest),'gpenmpcNative:SoleTransportHash','Native binary SHA mismatch before open.');
source=struct('kind','GPENMPC_RFLY_UDP_TRANSPORT_MEX','exact_path',char(p), ...
    'sha256',upper(char(digest)),'resolved_path',char(resolved), ...
    'validation_scope','CONSTRUCTION_BEFORE_OPEN','fallback_allowed',false);
native=@gpenmpc_rfly_udp_transport_mex;
end
function h=fileSha(path)
f=fopen(path,'rb');assert(f>=0,'gpenmpcNative:SoleTransportSource','Cannot read native binary.');
guard=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),b=fread(f,1048576,'*uint8');md.update(typecast(b,'int8'));end
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function n=increment(n)
assert(n<intmax('uint64'),'gpenmpcNative:SoleTransportCount','Bounded counter overflow.');n=n+uint64(1);
end
function checkInterval(startNs,endNs,startQpc,endQpc,frequency)
v={startNs,endNs,startQpc,endQpc,frequency};
assert(all(cellfun(@(x)isa(x,'uint64')&&isscalar(x)&&x>0,v)) ...
    &&endNs>=startNs&&endQpc>=startQpc, ...
    'gpenmpcNative:SoleTransportTime','Expected original exact uint64 QPC observation interval.');
end
function [n,sys,component,msgid,fragmentReason]=frameShape(bytes,offset,allowForwardingFragments)
remaining=numel(bytes)-offset+1;magic=bytes(offset);
n=0;sys=0;component=0;msgid=0;fragmentReason='';
if allowForwardingFragments&&magic~=253&&magic~=254
    fragmentReason='DATAGRAM_REMAINDER_DOES_NOT_START_WITH_MAVLINK_MAGIC';return
end
assert(magic==253||magic==254,'gpenmpcNative:SoleTransportMagic', ...
    'Unknown magic/trailing bytes; no resynchronization or deletion is allowed.');
if magic==253
    if allowForwardingFragments&&remaining<12
        fragmentReason='DATAGRAM_REMAINDER_SHORTER_THAN_MAVLINK2_HEADER_AND_CRC';return
    end
    assert(remaining>=12,'gpenmpcNative:SoleTransportTruncated','Truncated MAVLink 2 header.');
    flags=bytes(offset+2);
    assert(bitand(flags,uint8(254))==0,'gpenmpcNative:SoleTransportFlags','Unsupported incompatibility flags.');
    n=12+double(bytes(offset+1))+13*double(bitand(flags,uint8(1))~=0);
    sys=bytes(offset+5);component=bytes(offset+6);
    msgid=double(bytes(offset+7))+256*double(bytes(offset+8))+65536*double(bytes(offset+9));
else
    if allowForwardingFragments&&remaining<8
        fragmentReason='DATAGRAM_REMAINDER_SHORTER_THAN_MAVLINK1_HEADER_AND_CRC';return
    end
    assert(remaining>=8,'gpenmpcNative:SoleTransportTruncated','Truncated MAVLink 1 header.');
    n=8+double(bytes(offset+1));sys=bytes(offset+3);component=bytes(offset+4);msgid=double(bytes(offset+5));
end
if allowForwardingFragments&&remaining<n
    fragmentReason='DATAGRAM_REMAINDER_SHORTER_THAN_DECLARED_COMPLETE_FRAME';return
end
assert(remaining>=n,'gpenmpcNative:SoleTransportTruncated','Datagram does not contain the complete MAVLink frame.');
end
function [target,gpTarget,gpTargets]=lastSnapshotHint(data,first,cfg)
% Inspect only the retained batch; header hints do not validate message bodies.
target=0;gpTarget=0;gpTargets=[];order=numel(data):-1:first;
for k=order
    d=data(k);b=d.bytes(:);offset=1;
    if ~d.attempted||~d.ok||~d.bytes_complete||~d.source_matched||isempty(b),continue,end
    while offset<=numel(b)
        try
            [n,s,c,id,fragment]=frameShape(b,offset,true);
        catch
            break % Validate through the ordinary parser.
        end
        if ~isempty(fragment)||n==0,break,end
        if b(offset)==253&&id==385&&s==cfg.remote_system&&c==cfg.remote_component&&n>=18
            p=offset+10;
            if b(p)==18&&b(p+1)==164 ... % uint16 42002, MAVLink little endian
                    &&b(p+2)==cfg.local_system&&b(p+3)==cfg.local_component
                % Continue ordered parsing to a completed GP request within the same 8 ms slice.
                if b(p+4)==34&&b(p+5)==163 % schema10, final actual fragment
                    if target==0,target=k;end
                    if nargout<2,return,end
                elseif b(p+4)==81&&b(p+5)==130 % schema8, final request fragment
                    gpTarget=k; % Reverse scan leaves the first complete GP.
                    gpTargets=[k gpTargets]; %#ok<AGROW> at most 64 original datagrams
                end
            end
        end
        offset=offset+n;
    end
end
end
function passed=boundedIncompleteContinuation(data,index,offset,cfg)
% Classify a remainder only within a dequeued expected-source incomplete frame.
passed=false;
if offset~=1||index<=1,return,end
part=data(index).bytes(:);
% Preserve apparent frames from the configured MAVLink sender for validation.
if numel(part)>=10&&part(1)==253&&part(6)==cfg.remote_system&&part(7)==cfg.remote_component,return,end
if numel(part)>=8&&part(1)==254&&part(4)==cfg.remote_system&&part(5)==cfg.remote_component,return,end
used=numel(part);
for k=index-1:-1:1
    d=data(k);b=d.bytes(:);
    if ~d.attempted||~d.ok||~d.bytes_complete||~d.source_matched ...
            ||~strcmp(d.source_ip,cfg.remote_host)||d.source_port~=cfg.remote_port,return,end
    used=used+numel(b);
    if isempty(b),return,end
    if b(1)~=253&&b(1)~=254,continue,end
    try
        [n,s,c,~,reason]=frameShape(b,1,true);
    catch
        return % Reject ambiguous prefix association.
    end
    passed=strcmp(reason,'DATAGRAM_REMAINDER_SHORTER_THAN_DECLARED_COMPLETE_FRAME') ...
        &&s==cfg.remote_system&&c==cfg.remote_component&&used<=n;
    return % Never scan past another header/complete frame.
end
end
function witness=sameBatchSubstringWitness(data,index,offset,decoder,cfg)
% Look ahead within the dequeued batch without receiving or changing state.
witness=[];part=data(index).bytes(offset:end);part=part(:);
for candidate=1:numel(data)
    if candidate==index,continue,end
    d=data(candidate);wire=d.bytes(:);
    if ~d.attempted||~d.ok||~d.bytes_complete||~d.source_matched ...
            ||d.empty_would_block||numel(wire)<=numel(part) ...
            ||numel(wire)>300||d.received_bytes~=numel(wire) ...
            ||~strcmp(d.source_ip,cfg.remote_host)||d.source_port~=cfg.remote_port
        continue
    end
    positions=strfind(char(wire.'),char(part.'));
    if isempty(positions),continue,end
    try
        [n,sys,component,id]=frameShape(wire,1,false);
        if n~=numel(wire)||sys~=cfg.remote_system||component~=cfg.remote_component ...
                ||sys~=1||component~=1,continue,end
        checkInterval(d.submit_ns,d.dequeue_ns,d.submit_qpc,d.dequeue_qpc,d.qpc_frequency);
        fresh=copy(decoder);cleanup=onCleanup(@()delete(fresh));
        [message,status]=deserializemsg(fresh,wire.',OutputAllMessages=true);
        clear cleanup
        if ~isstruct(message)||~isscalar(message) ...
                ||~all(isfield(message,{'MsgID','SystemID','ComponentID','Payload'})) ...
                ||~isscalar(status)||status~=0 ...
                ||message.MsgID~=id||message.SystemID~=sys||message.ComponentID~=component
            continue
        end
    catch
        % If no valid containing frame exists, keep the ordinary rejection path.
        continue
    end
    start=positions(1);preceding=[];
    if index>1&&offset==1
        previous=data(index-1);prefix=previous.bytes(:);
        if previous.ok&&previous.source_matched&&previous.bytes_complete ...
                &&numel(prefix)==start-1&&isequal(prefix,wire(1:start-1))
            preceding=struct('datagram_index',index-1,'raw_datagram',previous, ...
                'frame_start_1based',1,'frame_end_1based',start-1);
        end
    end
    witness=struct('schema','SAME_BATCH_EXACT_PROPER_SUBSTRING_CRC0_V1', ...
        'witness_datagram_index',candidate,'witness_frame_offset',1, ...
        'witness_raw_datagram',d,'witness_raw_frame',wire, ...
        'official_crc_status',status,'system_id',sys,'component_id',component,'message_id',id, ...
        'fragment_start_1based',start,'fragment_end_1based',start+numel(part)-1, ...
        'all_remaining_bytes_matched',true,'preceding_prefix',preceding, ...
        'witness_message_admitted_here',false,'source_or_age_updated',false);
    return
end
end
function passed=isContiguousForwardingContinuation(witness,data,index,offset)
passed=false;
if isempty(witness)||index<=1||offset~=1||isempty(witness.preceding_prefix) ...
        ||~witness.all_remaining_bytes_matched||witness.fragment_start_1based<=1
    return
end
previous=data(index-1);prefix=previous.bytes(:);current=data(index).bytes(:);
passed=witness.preceding_prefix.datagram_index==index-1 ...
    &&witness.witness_datagram_index>index ...
    &&witness.fragment_start_1based==numel(prefix)+1 ...
    &&witness.fragment_end_1based==numel(prefix)+numel(current) ...
    &&isequal(witness.preceding_prefix.raw_datagram.bytes(:),prefix) ...
    &&isequal(witness.witness_raw_frame(1:numel(prefix)),prefix) ...
    &&isequal(witness.witness_raw_frame(witness.fragment_start_1based: ...
        witness.fragment_end_1based),current) ...
    &&numel(witness.witness_raw_frame)>numel(prefix)+numel(current) ...
    &&witness.official_crc_status==0&&witness.system_id==1&&witness.component_id==1 ...
    &&~witness.witness_message_admitted_here&&~witness.source_or_age_updated;
end
function r=forwardingFragmentRecord(d,index,offset,raw,reason,sequence,duplicateWitness)
frame=frameRecord(d,index,offset,raw);
r=struct('classification','INCOMPLETE_FORWARDING_FRAGMENT_NOT_A_MESSAGE', ...
    'reason',reason,'raw_fragment',raw(:),'raw_datagram',d, ...
    'original_host_receive_ns',d.dequeue_ns,'dequeueinfo',frame.dequeueinfo, ...
    'forwarding_fragment_sequence',sequence,'decoded_message',[], ...
    'message_generated',false,'mavlink_source_verified',false,'control_authority',false, ...
    'duplicate_witness',duplicateWitness);
end
function r=emptyForwardingFragments()
r=struct('classification',{},'reason',{},'raw_fragment',{},'raw_datagram',{}, ...
    'original_host_receive_ns',{},'dequeueinfo',{},'forwarding_fragment_sequence',{}, ...
    'decoded_message',{},'message_generated',{},'mavlink_source_verified',{},'control_authority',{}, ...
    'duplicate_witness',{});
end
function r=frameRecord(d,index,offset,raw)
r=struct('raw_frame',raw(:),'decoded_message',[], ...
    'original_host_receive_ns',d.dequeue_ns, ...
    'dequeueinfo',struct('submit_ns',d.submit_ns,'dequeue_ns',d.dequeue_ns, ...
        'submit_qpc',d.submit_qpc,'dequeue_qpc',d.dequeue_qpc, ...
        'qpc_frequency',d.qpc_frequency,'source_ip',d.source_ip,'source_port',d.source_port, ...
        'datagram_index',index,'frame_offset',offset,'datagram_bytes',numel(d.bytes), ...
        'time_semantics','ORIGINAL_HOST_DATAGRAM_DEQUEUE_OBSERVATION'), ...
    'official_parse_status',[],'validated',false,'validation_error','', ...
    'transport_frame_sequence',uint64(0),'inline_gp',[],'native_gp_history_only',false);
end
function r=emptyFrames()
r=struct('raw_frame',{},'decoded_message',{},'original_host_receive_ns',{}, ...
    'dequeueinfo',{},'official_parse_status',{},'validated',{},'validation_error',{}, ...
    'transport_frame_sequence',{},'inline_gp',{},'native_gp_history_only',{});
end
