classdef RflyLocalEnvironmentLedger < handle
    % Retain ENV232 sends and diagnostic264 observations.
    % Bind by exact model time or equal-generation bracketing using received records.
    % A send result or nearest timestamp alone is insufficient.
    properties (SetAccess=private)
        Failed=false
        SentCount=uint64(0)
        DiagnosticCount=uint64(0)
        PrebindDiagnosticCount=uint64(0)
    end
    properties (Access=private)
        Policy
        DecoderPolicy
        CopterId
        Capacity
        Frames
        Diagnostics
        FrameHead=1
        FrameCount=0
        DiagnosticHead=1
        DiagnosticRetained=0
        LastGeneration=0
        LastTime=-Inf
        LastReceive=uint64(0)
        LastApplied=0
        BoundObserved=false
    end
    methods
        function obj=RflyLocalEnvironmentLedger(policy,copterId,capacity)
            assert(isscalar(capacity)&&capacity==fix(capacity)&&capacity>=4&&capacity<=8192);
            obj.Policy=policy;obj.CopterId=copterId;obj.Capacity=capacity;
            obj.Frames=cell(1,capacity);obj.Diagnostics=cell(1,capacity);
            obj.DecoderPolicy=policy;
            setupOnly=intersect(fieldnames(policy),{'initial_payload_kg','remote_port'});
            if ~isempty(setupOnly),obj.DecoderPolicy=rmfield(policy,setupOnly);end
        end
        function sent(obj,event)
            assert(~obj.Failed,'gpenmpcNative:EnvironmentLedgerFailed', ...
                'The environment ledger is already fail-closed.');
            try
                assert(isa(event.bytes,'uint8')&&numel(event.bytes)==232&&isa(event.original_host_send_ns,'uint64') ...
                    &&isscalar(event.original_host_send_ns)&&event.original_host_send_ns>0 ...
                    &&isequal(event.send_returned,true),'gpenmpcNative:EnvironmentLedgerSend', ...
                    'The original environment transmit record is invalid.');
                h=readLE(event.bytes(1:8),'uint32');f=readLE(event.bytes(9:232),'double');
                assert(h(1)==1234567897&&h(2)==obj.CopterId&&all(isfinite(f))&&f(1)==2 ...
                    &&f(2)>obj.LastGeneration&&f(2)==fix(f(2))&&f(23)==obj.Policy.expected_session_token, ...
                    'gpenmpcNative:EnvironmentLedgerFrame','The environment frame identity or generation is invalid.');
                % Retain a bounded observation window with constant-time insertion.
                % The IO log preserves evicted records.
                slot=mod(obj.FrameHead+obj.FrameCount-1,obj.Capacity)+1;
                if obj.FrameCount==obj.Capacity
                    obj.FrameHead=mod(obj.FrameHead,obj.Capacity)+1;
                else,obj.FrameCount=obj.FrameCount+1;end
                obj.Frames{slot}=struct('event',event,'frame',f);
                obj.LastGeneration=f(2);obj.SentCount=obj.SentCount+uint64(1);
            catch ex,obj.Failed=true;rethrow(ex);end
        end
        function received(obj,event)
            assert(~obj.Failed,'gpenmpcNative:EnvironmentLedgerFailed', ...
                'The environment ledger is already fail-closed.');
            try
                assert(isa(event.bytes,'uint8')&&numel(event.bytes)==264&&isa(event.original_host_receive_ns,'uint64') ...
                    &&isscalar(event.original_host_receive_ns)&&event.original_host_receive_ns>=obj.LastReceive ...
                    &&event.original_host_receive_ns>0,'gpenmpcNative:EnvironmentLedgerReceive', ...
                    'The original diagnostic receive record is invalid or nonmonotonic.');
                % Accept status-16 pre-session diagnostics as observations only.
                % After a bound diagnostic, returning to unbound form is an error.
                decodePolicy=obj.Policy;decodePolicy.allow_unbound_pre_session=true;
                d=m600check.decodeCopterSimDeliveryDiagnostics(event.bytes,obj.CopterId,NaN,decodePolicy);a=d.environment_extension;
                prebind=d.packet_valid&&a.valid&&a.initial_not_applied&&~a.session_bound ...
                    &&a.session_token==0&&a.applied_frame_generation==0 ...
                    &&a.applied_payload_generation==0&&~a.task_env_failed ...
                    &&~a.can_continue_task&&~a.mass_ack_valid;
                if prebind
                    assert(~obj.BoundObserved,'gpenmpcNative:EnvironmentLedgerUnboundAfterBind', ...
                        'An unbound initialization diagnostic returned after the environment session was bound.');
                    assert(a.same_model_time_s>=obj.LastTime,'gpenmpcNative:EnvironmentLedgerDiagnosticTime', ...
                        'The prebind diagnostic model time reversed.');
                    obj.LastReceive=event.original_host_receive_ns;obj.LastTime=a.same_model_time_s;
                    obj.PrebindDiagnosticCount=obj.PrebindDiagnosticCount+uint64(1);
                    return
                end
                assert(d.packet_valid,'gpenmpcNative:EnvironmentLedgerDiagnosticPacket', ...
                    'The delivery diagnostic packet or environment identity is invalid.');
                assert(a.session_bound,'gpenmpcNative:EnvironmentLedgerDiagnosticSession', ...
                    'A non-initial diagnostic is not bound to the expected environment session.');
                assert(~a.task_env_failed,'gpenmpcNative:EnvironmentLedgerDiagnosticTaskFault', ...
                    'The plant reported a fail-closed delivery-environment fault.');
                assert(a.same_model_time_s>=obj.LastTime,'gpenmpcNative:EnvironmentLedgerDiagnosticTime', ...
                    'The diagnostic model time reversed.');
                assert(a.applied_frame_generation>=obj.LastApplied, ...
                    'gpenmpcNative:EnvironmentLedgerDiagnosticGeneration', ...
                    'The applied environment-frame generation reversed.');
                obj.LastReceive=event.original_host_receive_ns;obj.LastTime=a.same_model_time_s;obj.LastApplied=a.applied_frame_generation;
                obj.BoundObserved=true;
                slot=mod(obj.DiagnosticHead+obj.DiagnosticRetained-1,obj.Capacity)+1;
                if obj.DiagnosticRetained==obj.Capacity
                    obj.DiagnosticHead=mod(obj.DiagnosticHead,obj.Capacity)+1;
                else,obj.DiagnosticRetained=obj.DiagnosticRetained+1;end
                obj.Diagnostics{slot}=struct('event',event,'decoded',d);
                obj.DiagnosticCount=obj.DiagnosticCount+uint64(1);
            catch ex,obj.Failed=true;rethrow(ex);end
        end
        function [e,decoded]=resolve(obj,time,runtimeStateOnly,expectedCopterId,expectedPolicy)
            if nargin<3,runtimeStateOnly=false;end
            decoded=[];
            if nargout>1
                % Reuse this owner's validated receive record with the same vehicle and decoder policy.
                % Recheck resolution, expiry and mass.
                assert(nargin==5&&isequal(expectedCopterId,obj.CopterId)&&isequaln(expectedPolicy,obj.DecoderPolicy), ...
                    'gpenmpcNative:EnvironmentLedgerDecoderIdentity', ...
                    'Cached diagnostic decoding requires the original vehicle and decoder policy.');
            end
            assert(~obj.Failed,'gpenmpcNative:EnvironmentLedgerFailed', ...
                'The environment ledger is already fail-closed.');e=[];
            assert(isscalar(time)&&isfinite(time)&&time>=0,'gpenmpcNative:EnvironmentLedgerTime', ...
                'The requested environment resolution time is invalid.');
            if obj.DiagnosticRetained==0,return;end
            % Locate the last observation at or before model time; for equal times use the last.
            lo=1;hi=obj.DiagnosticRetained;li=0;
            while lo<=hi
                mid=floor((lo+hi)/2);
                if obj.Diagnostics{mod(obj.DiagnosticHead+mid-2,obj.Capacity)+1}.decoded.environment_extension.same_model_time_s<=time
                    li=mid;lo=mid+1;
                else,hi=mid-1;end
            end
            ix=[];left=[];
            if li>0&&obj.Diagnostics{mod(obj.DiagnosticHead+li-2,obj.Capacity)+1}.decoded.environment_extension.same_model_time_s==time,ix=li;end
            if isempty(ix)
                if li<obj.DiagnosticRetained,ix=li+1;end
                if runtimeStateOnly
                    % Runtime input is the most recent actually applied
                    % environment at/before the getter, not future bracketing.
                    % Keep source age and pending mass-update checks below.
                    if li==0,return;end
                    ix=li;
                else
                    if li==0||isempty(ix),return;end
                    left=obj.Diagnostics{mod(obj.DiagnosticHead+li-2,obj.Capacity)+1};
                end
            end
            right=obj.Diagnostics{mod(obj.DiagnosticHead+ix-2,obj.Capacity)+1};a=right.decoded.environment_extension;
            if ~a.mass_ack_valid,return;end
            if ~isempty(left)
                b=left.decoded.environment_extension;
                if ~b.mass_ack_valid||b.applied_frame_generation~=a.applied_frame_generation ...
                    ||b.applied_payload_generation~=a.applied_payload_generation||b.session_token~=a.session_token ...
                    ||~isequal(typecast(b.actual_payload_kg,'uint64'),typecast(a.actual_payload_kg,'uint64')),return;end
            end
            % sent() enforces strictly increasing frame generation.
            lo=1;hi=obj.FrameCount;fi=0;
            while lo<=hi
                mid=floor((lo+hi)/2);generation=obj.Frames{mod(obj.FrameHead+mid-2,obj.Capacity)+1}.frame(2);
                if generation==a.applied_frame_generation,fi=mid;break;
                elseif generation<a.applied_frame_generation,lo=mid+1;
                else,hi=mid-1;end
            end
            if fi==0,return;end
            f=obj.Frames{mod(obj.FrameHead+fi-2,obj.Capacity)+1};
            if runtimeStateOnly
                if time-f.frame(3)>.25||time<a.same_model_time_s,return;end
                % Require a mass ACK for the newly requested payload.
                latest=obj.Frames{mod(obj.FrameHead+obj.FrameCount-2,obj.Capacity)+1}.frame;
                if latest(21)~=a.applied_payload_generation||latest(5)~=a.actual_payload_kg,return;end
            end
            assert(f.event.original_host_send_ns<=right.event.original_host_receive_ns, ...
                'gpenmpcNative:EnvironmentLedgerChronology', ...
                'The diagnostic was received before the matching environment frame was sent.');
            e=struct('original_frame232',f.event.bytes(:),'original_diagnostic264',right.event.bytes(:), ...
                'original_frame_send_ns',f.event.original_host_send_ns, ...
                'original_diagnostic_receive_ns',right.event.original_host_receive_ns, ...
                'binding_semantics','EXACT_OR_BRACKETED_MONOTONIC_APPLIED_FRAME_V1', ...
                'original_previous_diagnostic264',uint8([]),'original_previous_diagnostic_receive_ns',uint64(0), ...
                'no_timestamp_renewal',true,'plant_control_authority',false);
            if ~isempty(left)
                e.original_previous_diagnostic264=left.event.bytes(:);
                e.original_previous_diagnostic_receive_ns=left.event.original_host_receive_ns;
            end
            if runtimeStateOnly,e.binding_semantics='LATEST_CAUSAL_APPLIED_ENVIRONMENT_WITH_ORIGINAL_250MS_EXPIRY';end
            decoded=right.decoded;
        end
        function s=status(obj)
            s=struct('failed',obj.Failed,'sent',obj.SentCount,'received',obj.DiagnosticCount, ...
                'prebind_observed',obj.PrebindDiagnosticCount,'bound_observed',obj.BoundObserved, ...
                'retained_frames',obj.FrameCount,'retained_diagnostics',obj.DiagnosticRetained, ...
                'extra_sockets',0,'plant_steps',0,'hardware_actions',0);
        end
    end
end
function v=readLE(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
