classdef GPENMPCNativeRawGpSystem < matlab.System
    % Replay retained GP fixtures with one native Raw IO owner.
    % outputImpl reads the previous queue; updateImpl receives and invokes the GP wire MEX.
    properties (Nontunable)
        ReplayConfig = struct()
    end
    properties (Access=private)
        Dialect
        Serializer
        Reassembler
        Origin
        Clock
        Queue = {}
        Raw = {}
        Queries = {}
        Transmitted = {}
        Failure = struct()
        InFlight = struct()
        Updates = 0
        GpCalls = 0
        Completed = 0
        LastTime = -1
        RuntimeEntered = false
        LedgerWritten = false
        LastOutputGeneration = uint64(0)
        LastSourceGeneration = uint64(0)
        LastSourceNs = uint64(0)
    end
    methods (Access=protected)
        function setupImpl(obj,~,~)
            c=obj.ReplayConfig;
            assert(strcmpi(string(which('canonical_gp_wire_mex')),string(c.mex_path)) ...
                &&strcmpi(fileSha(c.mex_path),c.mex_sha256), ...
                'gpenmpcNative:RawGpMexBinding','The configured MEX identity must match.');
            assert(isa(c.requests,'uint8')&&isequal(size(c.requests),[310 59]) ...
                &&~isfile(c.ledger_path),'gpenmpcNative:RawGpReplayConfiguration','Require 59 retained queries and a new output ledger.');
            first=gpenmpcNative.RflyLocalGpCodec.decodeRequest(c.requests(:,1));
            obj.Dialect=mavlinkdialect(c.dialect_path,2);
            % Use the object for MAVLink serialization and sequence state without connecting it.
            obj.Serializer=mavlinkio(obj.Dialect,'SystemID',255,'ComponentID',190);
            e=struct('source_system',first.identity.system,'source_component',first.identity.component, ...
                'target_system',uint8(255),'target_component',uint8(190), ...
                'uid',first.identity.uid,'boot_generation',first.identity.boot_generation, ...
                'link_lifecycle_generation',uint64(1),'confirmed_host_rx_ns',uint64(1), ...
                'execution_session_sha256',c.replay_binding_sha256);
            % Bound host resource use independently of source timestamp validity.
            obj.Reassembler=gpenmpcNative.RflyLocalTunnelReassembler(e,uint64(30000000000));
            obj.Origin=struct('link_lifecycle_generation',e.link_lifecycle_generation, ...
                'execution_session_sha256',e.execution_session_sha256);
            obj.Clock=tic;
            heartbeat=createmsg(obj.Dialect,'HEARTBEAT');
            heartbeat.Payload.type=uint8(6);heartbeat.Payload.autopilot=uint8(8);
            heartbeat.Payload.base_mode=uint8(0);heartbeat.Payload.custom_mode=uint32(0);
            heartbeat.Payload.system_status=uint8(4);heartbeat.Payload.mavlink_version=uint8(3);
            b=uint8(serializemsg(obj.Serializer,heartbeat));
            obj.Queue={struct('bytes',b(:),'query',0,'fragment',-1,'queued_at_simulation_s',-0.001)};
        end
        function [padded,length]=outputImpl(obj,~,~)
            padded=zeros(300,1,'uint8');length=uint16(1);
            if ~isempty(obj.Queue)
                b=obj.Queue{1}.bytes;length=uint16(numel(b));padded(1:numel(b))=b;
            end
        end
        function updateImpl(obj,raw,simulationTime)
            obj.RuntimeEntered=true;
            try
                assert(isempty(fieldnames(obj.Failure)),'gpenmpcNative:RawGpFailedClosed','A failed replay cannot restart.');
                assert(isscalar(simulationTime)&&isfinite(simulationTime)&&simulationTime>obj.LastTime ...
                    &&obj.Updates<10000,'gpenmpcNative:RawGpDiscreteOrder','Require finite increasing discrete times within the replay bound.');
                obj.LastTime=simulationTime;obj.Updates=obj.Updates+1;
                % Record the outputImpl state; the peer provides send confirmation.
                if ~isempty(obj.Queue)
                    tx=obj.Queue{1};obj.Queue(1)=[];tx.output_at_simulation_s=simulationTime;
                    obj.Transmitted{end+1,1}=tx;
                end
                receiveNs=uint64(floor(toc(obj.Clock)*1e9))+uint64(1);
                raw=raw(:);assert(isa(raw,'uint8')&&numel(raw)>=1&&numel(raw)<=4999, ...
                    'gpenmpcNative:RawGpAggregate','Actual native aggregate must fit its 4999-byte bound.');
                obj.Raw{end+1,1}=struct('bytes',raw,'simulation_s',simulationTime, ...
                    'host_observed_ns',receiveNs,'clock_semantics','MONOTONIC_HOST_OBSERVATION_NOT_SOURCE_TIMESTAMP');
                obj.Reassembler.poll(receiveNs);
                if numel(raw)==1&&raw(1)==0,return;end
                offset=1;
                while offset<=numel(raw)
                    assert(numel(raw)-offset+1>=12&&raw(offset)==253, ...
                        'gpenmpcNative:RawGpFrameStart','Missing or truncated MAVLink2 frame.');
                    assert(raw(offset+2)==0,'gpenmpcNative:RawGpFrameFlags','Replay requires original unsigned MAVLink2 frames.');
                    n=double(raw(offset+1))+12;
                    assert(n<=300&&offset+n-1<=numel(raw), ...
                        'gpenmpcNative:RawGpFrameLength','Truncated/overlength datagram cannot be repaired silently.');
                    frame=raw(offset:offset+n-1);offset=offset+n;
                    [message,status]=deserializemsg(obj.Dialect,frame);
                    assert(isscalar(message)&&isscalar(status)&&status==0, ...
                        'gpenmpcNative:RawGpMavlinkCrc','Official MAVLink decoder rejected the exact bytes.');
                    assert(message.MsgID==385&&message.Payload.payload_type==42002 ...
                        &&bitshift(message.Payload.payload(1),-4)==8, ...
                        'gpenmpcNative:RawGpUnexpectedSchema','Only the actual schema8 GP query is allowed.');
                    processingNs=uint64(floor(toc(obj.Clock)*1e9))+uint64(1);
                    event=obj.Reassembler.ingest(message,receiveNs,processingNs,obj.Origin);
                    if strcmp(event.status,'COMPLETE_RETAINED')
                        pending=obj.Reassembler.take('gp_request',processingNs);
                        obj.processPending(pending,simulationTime);
                    end
                end
            catch ex
                obj.Failure=exceptionEvidence(ex);rethrow(ex)
            end
        end
        function releaseImpl(obj)
            if obj.RuntimeEntered&&~obj.LedgerWritten
                ledger=struct('scope','RETAINED_C_QUERY_NATIVE_RAW_SAME_PROCESS_GP_REPLAY', ...
                    'updates',obj.Updates,'gp_calls',obj.GpCalls,'queries_completed',obj.Completed, ...
                    'raw',{obj.Raw},'queries',{obj.Queries},'system_output_frames',{obj.Transmitted}, ...
                    'remaining_tx_queue',{obj.Queue},'reassembler',obj.Reassembler.evidence(), ...
                    'failure',obj.Failure,'in_flight_at_release',obj.InFlight, ...
                    'source_timestamps_and_publication_age_unchanged',true, ...
                    'live_admission_proven',false,'controller_calls',0,'plant_runs',0,'sockets',0,'timers',0);
                assert(~isfile(obj.ReplayConfig.ledger_path),'Preserve native GP system ledger.');
                save(obj.ReplayConfig.ledger_path,'ledger');obj.LedgerWritten=true;
            end
            if ~isempty(obj.Reassembler),obj.Reassembler.close();end
            if ~isempty(obj.Serializer),delete(obj.Serializer);obj.Serializer=[];end
        end
        function n=getNumInputsImpl(~),n=2;end
        function n=getNumOutputsImpl(~),n=2;end
        function [a,b]=getOutputSizeImpl(~),a=[300 1];b=[1 1];end
        function [a,b]=getOutputDataTypeImpl(~),a='uint8';b='uint16';end
        function [a,b]=isOutputComplexImpl(~),a=false;b=false;end
        function [a,b]=isOutputFixedSizeImpl(~),a=true;b=true;end
        function [a,b]=isInputDirectFeedthroughImpl(~),a=false;b=false;end
        function yes=isInputSizeMutableImpl(~,index),yes=index==1;end
        function st=getSampleTimeImpl(obj),st=createSampleTime(obj,'Type','Discrete','SampleTime',.001);end
    end
    methods (Static,Access=protected)
        function mode=getSimulateUsingImpl,mode='Interpreted execution';end
        function yes=showSimulateUsingImpl,yes=false;end
    end
    methods (Access=private)
        function processPending(obj,pending,t)
            k=obj.Completed+1;
            assert(k<=59&&isempty(obj.Queue)&&~isempty(pending), ...
                'gpenmpcNative:RawGpPending','No overwriting queued replies or overlapping queries.');
            q=pending.decoded;
            assert(isequal(pending.message,obj.ReplayConfig.requests(:,k)), ...
                'gpenmpcNative:RawGpActualFixture','Received bytes differ from the next retained actual C request.');
            assert(q.output_generation>obj.LastOutputGeneration&&q.source_generation>obj.LastSourceGeneration ...
                &&q.source_timestamp_ns>obj.LastSourceNs,'gpenmpcNative:RawGpReplayOrder','Original generations and timestamps must increase.');
            obj.LastOutputGeneration=q.output_generation;obj.LastSourceGeneration=q.source_generation;
            obj.LastSourceNs=q.source_timestamp_ns;
            obj.InFlight=struct('pending',pending,'reply_bytes',[],'result18',[],'gp_call_index',obj.GpCalls+1);
            obj.GpCalls=obj.GpCalls+1;callClock=tic;
            [reply,result18]=canonical_gp_wire_mex(pending.message);gpWall=toc(callClock);
            obj.InFlight.reply_bytes=reply;obj.InFlight.result18=result18;
            r=gpenmpcNative.RflyLocalGpCodec.decodeReply(reply);
            assert(isequal(r.identity,q.identity)&&r.output_generation==q.output_generation ...
                &&r.source_generation==q.source_generation&&r.source_timestamp_ns==q.source_timestamp_ns ...
                &&isequal(r.original_request_sha256,q.original_request_sha256) ...
                &&isequal(r.gp_model_sha256,q.gp_model_sha256),'gpenmpcNative:RawGpReplyBinding','Reply must retain the exact request and GP identity.');
            payloads=gpenmpcNative.RflyLocalGpCodec.replyFragments(reply,q.identity.system,q.identity.component);
            frames=cell(3,1);
            for j=1:3
                message=createmsg(obj.Dialect,'TUNNEL');message.Payload=payloads{j};
                b=uint8(serializemsg(obj.Serializer,message));b=b(:);
                assert(numel(b)>5&&numel(b)<=300,'gpenmpcNative:RawGpReplyDatagram','Reply datagram must fit the native Raw transport.');
                frames{j}=b;obj.Queue{end+1}=struct('bytes',b,'query',k,'fragment',j-1,'queued_at_simulation_s',t);
            end
            obj.Queries{end+1,1}=struct('index',k,'pending',pending,'reply_bytes',reply, ...
                'result18',result18,'gp_call_index',obj.GpCalls,'gp_mex_wall_s',gpWall, ...
                'completion_at_simulation_s',t,'reply_frames',{frames});
            obj.Completed=obj.Completed+1;
            obj.InFlight=struct();
        end
    end
end
function h=fileSha(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function tree=exceptionEvidence(ex)
causes=cell(size(ex.cause));for k=1:numel(ex.cause),causes{k}=exceptionEvidence(ex.cause{k});end
tree=struct('identifier',ex.identifier,'message',ex.message, ...
    'report',getReport(ex,'extended','hyperlinks','off'),'causes',{causes});
end
