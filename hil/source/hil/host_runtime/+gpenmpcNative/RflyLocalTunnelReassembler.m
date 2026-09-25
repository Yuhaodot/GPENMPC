classdef RflyLocalTunnelReassembler < handle
    % Assemble RGP1, RLS1 and RLC1/RLC2 on the registered MAVLink owner.
    properties (SetAccess=private)
        Failed=false
        Failure=""
        Closed=false
    end
    properties (Access=private)
        Expected
        AssemblyLimitNs
        LastReceiveNs=uint64(0)
        LastHistoricalReceiveNs=uint64(0)
        LastProcessingNs=uint64(0)
        LastGeneration=zeros(1,3,'uint64')
        Active={[],[],[]}
        Ready={[],[],[]}
        Received=uint64(0)
        Completed=zeros(1,3,'uint64')
        Ignored=uint64(0)
        LastRaw=[]
        ExpiredSnapshotGeneration=uint64(0)
    end
    methods
        function obj=RflyLocalTunnelReassembler(expected,assemblyLimitNs)
            for f={'source_system','source_component','target_system','target_component'}
                v=expected.(f{1});assert(isa(v,'uint8')&&isscalar(v)&&v>0);
            end
            for f={'uid','boot_generation','link_lifecycle_generation','confirmed_host_rx_ns'}
                v=expected.(f{1});assert(isa(v,'uint64')&&isscalar(v)&&v>0);
            end
            assert(~isempty(regexp(char(expected.execution_session_sha256),'^[0-9A-Fa-f]{64}$','once')));
            assert(isa(assemblyLimitNs,'uint64')&&isscalar(assemblyLimitNs)&&assemblyLimitNs>0);
            % Bound host assembly resource use.
            obj.Expected=expected;obj.AssemblyLimitNs=assemblyLimitNs;
            obj.LastReceiveNs=expected.confirmed_host_rx_ns;
            obj.LastHistoricalReceiveNs=expected.confirmed_host_rx_ns;
            obj.LastProcessingNs=expected.confirmed_host_rx_ns;
        end
        function r=ingest(obj,message,originalHostReceiveNs,processingNs,origin)
            try
                obj.LastRaw=struct('message',message,'original_host_receive_ns',originalHostReceiveNs, ...
                    'processing_ns',processingNs,'origin',origin);
                obj.event(processingNs);
                historical=isfield(obj.Expected,'runtime_state_only')&&obj.Expected.runtime_state_only ...
                    &&double(message.MsgID)==385&&double(message.Payload.payload_type)==42002 ...
                    &&ismember(bitshift(message.Payload.payload(1),-4),uint8([12 14]));
                previousRx=obj.LastReceiveNs;
                if historical,previousRx=obj.LastHistoricalReceiveNs;end
                assert(isa(originalHostReceiveNs,'uint64')&&isscalar(originalHostReceiveNs) ...
                    &&originalHostReceiveNs>=previousRx&&originalHostReceiveNs<=processingNs, ...
                    'gpenmpcNative:LocalTunnelClock','Original receive order cannot be replaced by processing time.');
                % The sole transport checks global RX order before recording.
                % Runtime RLC history may be consumed after newer RLS, but its
                % own original RX/order/body/generation checks remain intact.
                if historical,obj.LastHistoricalReceiveNs=originalHostReceiveNs;
                else,obj.LastReceiveNs=originalHostReceiveNs;end
                obj.origin(origin);
                r=struct('status','IGNORED_NONPRIVATE','completed_channel',"",'hardware_actions',0);
                if double(message.MsgID)~=385||double(message.Payload.payload_type)~=42002
                    obj.Ignored=obj.Ignored+uint64(1);return
                end
                obj.Received=obj.Received+uint64(1);e=obj.Expected;p=message.Payload;
                assert(isequal(uint8(message.SystemID),e.source_system)&&isequal(uint8(message.ComponentID),e.source_component) ...
                    &&isequal(p.target_system,e.target_system)&&isequal(p.target_component,e.target_component), ...
                    'gpenmpcNative:LocalTunnelAddress','Registered source and destination required.');
                assert(isa(p.payload,'uint8')&&numel(p.payload)==128 ...
                    &&isa(p.payload_length,'uint8')&&isscalar(p.payload_length));
                b=p.payload(:);schema=bitshift(b(1),-4);index=double(bitand(b(1),uint8(15)));
                switch schema
                    case 8,k=1;count=3;length=310;label="gp_request";
                    case 10,k=2;count=4;length=382;label="snapshot";
                    case 12,k=3;count=12;length=1398;label="committed_state";
                    case 14,k=3;count=13;length=1494;label="committed_state";
                    otherwise,error('gpenmpcNative:LocalTunnelSchema','Legacy/unknown outbound schema is not the local runtime.');
                end
                if k==3&&isfield(e,'require_learning_audit')&&e.require_learning_audit
                    assert(schema==14,'gpenmpcNative:LocalTunnelLearningRequired', ...
                        'Current deployment requires actual committed per-axis learning evidence.');
                end
                assert(index<count,'gpenmpcNative:LocalTunnelIndex','Fragment index out of range.');
                n=min(119,length-index*119);
                assert(double(p.payload_length)==9+n&&all(b(10+n:end)==0), ...
                    'gpenmpcNative:LocalTunnelLength','Exact body length and zero padding required.');
                generation=be64(b(2:9));assert(generation>0);
                runtimeSnapshot=k==2&&isfield(e,'runtime_state_only')&&e.runtime_state_only;
                if runtimeSnapshot&&~isempty(obj.Active{k}) ...
                        &&originalHostReceiveNs-obj.Active{k}.first_rx_ns>obj.AssemblyLimitNs
                    % Measure incomplete-message expiry from wire reception, not batch processing delay.
                    obj.ExpiredSnapshotGeneration=obj.Active{k}.generation;
                    obj.LastGeneration(k)=obj.Active{k}.generation;obj.Active{k}=[];
                    obj.Ignored=obj.Ignored+uint64(1);
                end
                if k==2&&generation==obj.ExpiredSnapshotGeneration&&index>0
                    % Retain the tail of an expired snapshot as raw data only.
                    obj.Ignored=obj.Ignored+uint64(1);r.status='EXPIRED_SNAPSHOT_NOT_USED';return
                end
                a=obj.Active{k};
                if isempty(a)
                    assert(index==0&&generation>obj.LastGeneration(k), ...
                        'gpenmpcNative:LocalTunnelStart','Missing start or repeated/reversed generation.');
                    assert(isempty(obj.Ready{k}),'gpenmpcNative:LocalTunnelOverflow','Unconsumed bounded slot cannot be overwritten.');
                    a=struct('generation',generation,'wire_schema',schema,'next_index',0,'message',zeros(length,1,'uint8'), ...
                        'first_rx_ns',originalHostReceiveNs,'last_rx_ns',originalHostReceiveNs, ...
                        'fragment_rx_ns',zeros(count,1,'uint64'),'decoded_fragments',{cell(count,1)}, ...
                        'original_callback_indices',zeros(count,1,'uint64'));
                end
                assert(schema==a.wire_schema&&generation==a.generation&&index==a.next_index, ...
                    'gpenmpcNative:LocalTunnelOrder','Missing, duplicate, reordered or interleaved message.');
                % Retain RLC slots and timestamps through assembly.
                % The outer consumer separately checks numerical freshness.
                historicalCommit=k==3&&isfield(e,'runtime_state_only')&&e.runtime_state_only;
                runtimeGp=k==1&&isfield(e,'runtime_state_only')&&e.runtime_state_only;
                % Validate GP fragment receive continuity before retiring a completed late request.
                age=processingNs-a.first_rx_ns;
                if runtimeGp||runtimeSnapshot,age=originalHostReceiveNs-a.first_rx_ns;end
                assert(historicalCommit||age<=obj.AssemblyLimitNs, ...
                    'gpenmpcNative:LocalTunnelExpired','Queued/partial bytes do not get a new lifetime.');
                a.message(index*119+1:index*119+n)=b(10:9+n);
                a.fragment_rx_ns(index+1)=originalHostReceiveNs;a.decoded_fragments{index+1}=message;
                % Local callback owner may retain its exact append-only row
                % index. It is only a lookup hint; the sender still compares
                % time AND complete decoded bytes against that actual row.
                % Standalone legacy fixtures have no owner index and keep 0.
                if isfield(origin,'original_callback_index')
                    ci=origin.original_callback_index;
                    assert(isa(ci,'uint64')&&isscalar(ci)&&ci>0 ...
                        &&(index==0||ci>a.original_callback_indices(index)), ...
                        'gpenmpcNative:LocalTunnelCallbackIndex');
                    a.original_callback_indices(index+1)=ci;
                end
                a.last_rx_ns=originalHostReceiveNs;a.next_index=index+1;
                obj.Active{k}=a;r.status='FRAGMENT_RETAINED';
                if a.next_index~=count,return;end
                d=[];
                if k==1
                    d=gpenmpcNative.RflyLocalGpCodec.decodeRequest(a.message);bodyGeneration=d.output_generation;
                elseif k==2&&(~isfield(e,'runtime_state_only')||~e.runtime_state_only)
                    d=gpenmpcNative.RflyLocalSnapshotDecoder(a.message,a.first_rx_ns);bodyGeneration=d.source_generation;
                elseif ~isfield(e,'runtime_state_only')||~e.runtime_state_only
                    d=gpenmpcNative.RflyLocalCommittedDecoder(a.message,a.first_rx_ns);bodyGeneration=d.output_generation;
                end
                if ~isempty(d)
                    i=d.identity;
                    assert(bodyGeneration==generation&&i.uid==e.uid&&i.boot_generation==e.boot_generation ...
                        &&i.system==e.source_system&&i.component==e.source_component, ...
                        'gpenmpcNative:LocalTunnelBodyBinding','Body and fragment/link identities differ.');
                end
                if runtimeGp&&processingNs-a.first_rx_ns>obj.AssemblyLimitNs
                    % Retain expired request bytes without dispatching prediction or reply.
                    obj.Active{k}=[];obj.LastGeneration(k)=generation;
                    obj.Completed(k)=obj.Completed(k)+uint64(1);
                    obj.Ignored=obj.Ignored+uint64(1);
                    r.status='EXPIRED_GP_NOT_USED';return
                end
                % Queue RLS records for historical RLC matching while retaining original timestamps.
                % Fast input uses the 50 ms receive-age bound; outer processing has its own budget.
                % takeCanonical validates complete bodies and identity before use.
                % GP requests retain immediate numerical decoding.
                a.schema='LOCAL_RUNTIME_COMPLETED_ORIGINAL_TUNNEL';a.channel=label;
                a.original_host_receive_ns=a.first_rx_ns;a.completion_host_receive_ns=a.last_rx_ns;
                a.processing_ns=processingNs;a.origin=origin;a.decoded=d;
                a.crc_verified_here=false;a.source_authenticated_here=false;
                a.freshness_renewed=false;a.control_authority=false;
                obj.Ready{k}=a;obj.Active{k}=[];obj.LastGeneration(k)=generation;
                obj.Completed(k)=obj.Completed(k)+uint64(1);
                r.status='COMPLETE_RETAINED';r.completed_channel=label;
            catch ex,obj.poison(ex);end
        end
        function r=take(obj,channel,processingNs)
            try
                obj.event(processingNs);k=find(["gp_request","snapshot","committed_state"]==string(channel));assert(isscalar(k));
                r=obj.Ready{k};obj.Ready{k}=[];
            catch ex,obj.poison(ex);end
        end
        function r=poll(obj,processingNs)
            try,obj.event(processingNs);r=obj.status();catch ex,obj.poison(ex);end
        end
        function r=status(obj)
            r=struct('failed',obj.Failed,'failure',obj.Failure,'closed',obj.Closed, ...
                'private_fragments_received',obj.Received,'messages_completed',obj.Completed, ...
                'ignored_nonprivate',obj.Ignored,'active',~cellfun(@isempty,obj.Active), ...
                'ready',~cellfun(@isempty,obj.Ready),'last_generation',obj.LastGeneration, ...
                'maximum_partial_slots',3,'maximum_completed_slots',3,'hardware_actions',0,'connections',0);
        end
        function r=evidence(obj)
            r=struct('status',obj.status(),'last_raw',obj.LastRaw,'active',{obj.Active}, ...
                'ready',{obj.Ready},'persistence_proven',false);
        end
        function close(obj),obj.Closed=true;end
    end
    methods (Access=private)
        function event(obj,t)
            assert(~obj.Failed&&~obj.Closed,'gpenmpcNative:LocalTunnelClosed','No retry in a failed/closed session.');
            assert(isa(t,'uint64')&&isscalar(t)&&t>=obj.LastProcessingNs, ...
                'gpenmpcNative:LocalTunnelProcessingClock','Processing clock cannot regress.');
            obj.LastProcessingNs=t;
            for k=1:3
                a=obj.Active{k};r=obj.Ready{k};
                if k==1&&isfield(obj.Expected,'runtime_state_only')&&obj.Expected.runtime_state_only
                    % Keep one bounded partial slot and validate original receive span and ordering.
                    % Retire completed expired requests without renewing timestamps.
                    if ~isempty(r)&&t-r.first_rx_ns>obj.AssemblyLimitNs
                        obj.Ready{k}=[];obj.Ignored=obj.Ignored+uint64(1);
                    end
                    continue
                end
                if k==3&&isfield(obj.Expected,'runtime_state_only')&&obj.Expected.runtime_state_only
                    % Keep partial RLC data until assembly completes; validate the completed body
                    % and outer freshness before use.
                    continue
                end
                if k==2&&isfield(obj.Expected,'runtime_state_only')&&obj.Expected.runtime_state_only
                    % Retain delayed same-batch RLS fragments in the bounded slots.
                    % Validate wire span and ordering on ingest, then the complete body on consumption.
                    continue
                end
                assert((isempty(a)||t-a.first_rx_ns<=obj.AssemblyLimitNs) ...
                    &&(isempty(r)||t-r.first_rx_ns<=obj.AssemblyLimitNs), ...
                    'gpenmpcNative:LocalTunnelExpired','Original partial/ready bytes expired; never renew/reopen.');
            end
        end
        function origin(obj,o)
            e=obj.Expected;
            assert(isequal(o.link_lifecycle_generation,e.link_lifecycle_generation) ...
                &&strcmpi(o.execution_session_sha256,e.execution_session_sha256), ...
                'gpenmpcNative:LocalTunnelOrigin','Original shared link/session required.');
        end
        function poison(obj,ex)
            if ~obj.Failed,obj.Failed=true;obj.Failure=string(ex.identifier);end
            obj.Closed=true;rethrow(ex)
        end
    end
end
function v=be64(b)
v=uint64(0);for k=1:numel(b),v=bitor(bitshift(v,8),uint64(b(k)));end
end
