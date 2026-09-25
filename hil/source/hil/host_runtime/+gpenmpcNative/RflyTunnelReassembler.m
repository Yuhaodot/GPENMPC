classdef RflyTunnelReassembler < handle
    % Assemble RSP1/RFC1 from decoded MAVLink ingress.
    % The link owner validates CRC, signature and route.
    properties (SetAccess=private)
        Failed=false
        Failure=""
        Closed=false
    end
    properties (Access=private)
        Expected
        AssemblyLimitNs
        LastEventNs=uint64(0)
        LastGeneration=zeros(1,2,'uint64')
        Active={[],[]}
        Ready={[],[]}
        Received=uint64(0)
        Completed=zeros(1,2,'uint64')
        Ignored=uint64(0)
    end
    methods
        function obj=RflyTunnelReassembler(expected,assemblyLimitNs)
            % Bound host-session assembly resources independently of source lifetime.
            for f={'source_system','source_component','target_system','target_component'}
                v=expected.(f{1});assert(isa(v,'uint8')&&isscalar(v)&&v>0);
            end
            for f={'uid','session_generation','link_lifecycle_generation','confirmed_host_rx_ns'}
                v=expected.(f{1});assert(isa(v,'uint64')&&isscalar(v)&&v>0);
            end
            for f={'execution_session_sha256','configuration_sha256'}
                assert(~isempty(regexp(char(expected.(f{1})),'^[0-9A-Fa-f]{64}$','once')));
            end
            assert(isa(assemblyLimitNs,'uint64')&&isscalar(assemblyLimitNs)&&assemblyLimitNs>0);
            obj.Expected=expected;obj.AssemblyLimitNs=assemblyLimitNs;
            obj.LastEventNs=expected.confirmed_host_rx_ns;
        end
        function r=ingest(obj,message,originalHostReceiveNs,origin)
            try
                obj.event(originalHostReceiveNs);obj.origin(origin);
                r=struct('status','IGNORED_NONPRIVATE','completed_channel',"",'hardware_actions',0);
                if double(message.MsgID)~=385||double(message.Payload.payload_type)~=42002
                    obj.Ignored=obj.Ignored+uint64(1);return
                end
                obj.Received=obj.Received+uint64(1);e=obj.Expected;p=message.Payload;
                assert(isequal(uint8(message.SystemID),e.source_system)&&isequal(uint8(message.ComponentID),e.source_component) ...
                    &&isequal(p.target_system,e.target_system)&&isequal(p.target_component,e.target_component), ...
                    'gpenmpcNative:TunnelAddress','Private message is not from/to the registered endpoints.');
                assert(isa(p.payload,'uint8')&&numel(p.payload)==128&&isa(p.payload_length,'uint8'), ...
                    'gpenmpcNative:TunnelShape','Exact decoded TUNNEL payload required.');
                b=p.payload(:);version=bitshift(b(1),-4);index=double(bitand(b(1),uint8(15)));
                switch version
                    case 3,channel=1;count=3;length=246;label="snapshot";
                    case 5,channel=2;count=10;length=1112;label="feedback";
                    otherwise,error('gpenmpcNative:TunnelSchema','Unexpected private outbound schema.');
                end
                assert(index<count,'gpenmpcNative:TunnelIndex','Out-of-range fragment.');
                n=min(119,length-index*119);
                assert(double(p.payload_length)==9+n&&all(b(10+n:end)==0), ...
                    'gpenmpcNative:TunnelLength','Fragment length or unused bytes differ from the production writer.');
                generation=be64(b(2:9));assert(generation>0,'gpenmpcNative:TunnelGeneration','Zero generation.');
                a=obj.Active{channel};
                if isempty(a)
                    assert(index==0&&generation>obj.LastGeneration(channel), ...
                        'gpenmpcNative:TunnelStart','Missing first fragment or replayed generation.');
                    assert(isempty(obj.Ready{channel}),'gpenmpcNative:TunnelOverflow', ...
                        'Unconsumed complete message: the bounded slot cannot be overwritten.');
                    a=struct('generation',generation,'next_index',0,'message',zeros(length,1,'uint8'), ...
                        'first_rx_ns',originalHostReceiveNs,'last_rx_ns',originalHostReceiveNs, ...
                        'fragment_rx_ns',zeros(count,1,'uint64'),'decoded_fragments',{cell(count,1)});
                end
                assert(generation==a.generation&&index==a.next_index, ...
                    'gpenmpcNative:TunnelOrder','Missing, duplicate, reordered or replaced fragment sequence.');
                % Original first receipt is the freshness anchor. A later
                % fragment and a later poll are NOT a new source event.
                assert(originalHostReceiveNs-a.first_rx_ns<=obj.AssemblyLimitNs, ...
                    'gpenmpcNative:TunnelAssemblyExpired','Original assembly interval exhausted.');
                a.message(index*119+1:index*119+n)=b(10:9+n);
                a.fragment_rx_ns(index+1)=originalHostReceiveNs;
                a.decoded_fragments{index+1}=message;a.last_rx_ns=originalHostReceiveNs;
                a.next_index=index+1;obj.Active{channel}=a;r.status='FRAGMENT_RETAINED';
                if a.next_index~=count,return;end
                if channel==1
                    d=gpenmpcNative.RflySnapshotDecoder(a.message,a.first_rx_ns);
                    assert(uint64(d.subscription_generation)==generation&&d.observed_uid==e.uid ...
                        &&d.observed_boot_generation==e.session_generation ...
                        &&d.source_system==e.source_system&&d.source_component==e.source_component ...
                        &&strcmpi(hex(d.configuration_sha256),e.configuration_sha256), ...
                        'gpenmpcNative:TunnelBodyBinding','RSP1 body differs from fragment/session binding.');
                else
                    d=gpenmpcNative.RflyCommittedFeedbackDecoder(a.message,a.first_rx_ns);t=d.token;
                    assert(t.output_generation==generation&&t.identity.uid==e.uid ...
                        &&t.identity.boot_generation==e.session_generation ...
                        &&t.identity.system==e.source_system&&t.identity.component==e.source_component ...
                        &&strcmpi(d.configuration_payload_sha256,e.configuration_sha256), ...
                        'gpenmpcNative:TunnelBodyBinding','RFC1 body differs from fragment/session binding.');
                end
                a.schema='RFLY_COMPLETED_TUNNEL_ON_EXISTING_LINK_V1';a.channel=label;
                a.original_host_receive_ns=a.first_rx_ns;a.completion_host_receive_ns=a.last_rx_ns;
                a.origin=origin;a.board_consumption_proven=false;a.transport_source_authenticated=false;
                a.crc_verified_here=false;a.freshness_renewed=false;a.decoded=d;
                obj.Ready{channel}=a;obj.Active{channel}=[];
                obj.LastGeneration(channel)=generation;obj.Completed(channel)=obj.Completed(channel)+uint64(1);
                r.status='COMPLETE_RETAINED';r.completed_channel=label;
            catch ex,obj.poison(ex);end
        end
        function r=take(obj,channel,nowNs)
            try
                obj.event(nowNs);k=find(["snapshot","feedback"]==string(channel));assert(isscalar(k));
                r=obj.Ready{k};obj.Ready{k}=[]; % Consume the retained record.
            catch ex,obj.poison(ex);end
        end
        function r=poll(obj,nowNs)
            try,obj.event(nowNs);r=obj.status();catch ex,obj.poison(ex);end
        end
        function r=status(obj)
            r=struct('schema','RFLY_BOUNDED_TUNNEL_STATUS_V1','failed',obj.Failed,'failure',obj.Failure, ...
                'closed',obj.Closed,'private_fragments_received',obj.Received,'messages_completed',obj.Completed, ...
                'ignored_nonprivate',obj.Ignored,'active',~cellfun(@isempty,obj.Active), ...
                'ready',~cellfun(@isempty,obj.Ready),'last_generation',obj.LastGeneration, ...
                'maximum_partial_slots',2,'maximum_completed_slots',2,'hardware_actions',0,'connections',0);
        end
        function r=evidence(obj)
            % Retain bounded diagnostics, including partial failures, for storage after shutdown.
            r=struct('status',obj.status(),'active',{obj.Active},'ready',{obj.Ready}, ...
                'persistence_proven',false);
        end
        function close(obj),obj.Closed=true;end
    end
    methods (Access=private)
        function event(obj,t)
            assert(~obj.Failed&&~obj.Closed,'gpenmpcNative:TunnelClosed','Failed/closed session cannot restart.');
            assert(isa(t,'uint64')&&isscalar(t)&&t>0&&t>=obj.LastEventNs, ...
                'gpenmpcNative:TunnelClock','Original monotonic uint64 event required.');
            obj.LastEventNs=t;
            for k=1:2
                a=obj.Active{k};
                assert(isempty(a)||t-a.first_rx_ns<=obj.AssemblyLimitNs, ...
                    'gpenmpcNative:TunnelAssemblyExpired','Partial original window expired; do not reset/reopen.');
            end
        end
        function origin(obj,o)
            e=obj.Expected;
            assert(isequal(o.link_lifecycle_generation,e.link_lifecycle_generation) ...
                &&strcmpi(o.execution_session_sha256,e.execution_session_sha256), ...
                'gpenmpcNative:TunnelOrigin','Same registered link/session required.');
        end
        function poison(obj,ex)
            if ~obj.Failed,obj.Failed=true;obj.Failure=string(ex.identifier);end
            % Preserve partial and complete bytes on error.
            obj.Closed=true;rethrow(ex)
        end
    end
end
function v=be64(b)
v=uint64(0);for k=1:numel(b),v=bitor(bitshift(v,8),uint64(b(k)));end
end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
