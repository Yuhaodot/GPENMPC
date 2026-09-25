classdef RflyHostContextBinding < handle
    % Bind host events to RCT1 with one owner and bounded source retention.
    properties (SetAccess=private)
        Failed=false
        Failure=""
        SourcesRecorded=uint64(0)
        OuterSubmissions=uint64(0)
        OuterCommits=uint64(0)
        ReferencesBound=uint64(0)
    end
    properties (Access=private)
        Expected
        Capacity
        ReferenceMaxAgeNs
        OuterMaxAgeNs
        Sources
        Pending=[]
        Outer=[]
        Reference=[]
        LastSubmit=uint64(0)
        LastHost=uint64(0)
        LastBoardSampleUs=uint64(0)
        LastSourceGeneration=uint32(0)
        LastSourceReceiveNs=uint64(0)
    end
    methods
        function obj=RflyHostContextBinding(expected,capacity,referenceMaxAgeNs,outerMaxAgeNs)
            assert(isscalar(capacity)&&isfinite(capacity)&&capacity>=2&&capacity==fix(capacity), ...
                'gpenmpcNative:HostContextCapacity','Explicit bounded source storage required.');
            obj.integer(referenceMaxAgeNs);obj.integer(outerMaxAgeNs);
            obj.Expected=expected;obj.Capacity=capacity;
            obj.ReferenceMaxAgeNs=referenceMaxAgeNs;obj.OuterMaxAgeNs=outerMaxAgeNs;
            obj.Sources=cell(1,capacity);
        end
        function p=recordSnapshot(obj,bytes,originalHostRxNs,processingNs)
            % A real same-owner receive FIFO can contain a source received
            % before a later solver poll/commit. Its original receive time
            % remains the source anchor, not the processing event time.
            if nargin<4,processingNs=originalHostRxNs;end
            obj.time(processingNs);obj.requireInteger(originalHostRxNs);
            if originalHostRxNs>processingNs||originalHostRxNs<obj.LastSourceReceiveNs
                obj.fail("SOURCE_ORIGINAL_RECEIVE_ORDER");
            end
            try,p=gpenmpcNative.RflySnapshotSample(bytes,originalHostRxNs,obj.Expected);
            catch ex,obj.fail("SOURCE_EXPORT__"+string(ex.identifier));end
            if obj.LastBoardSampleUs>0
                delta=mod(double(p.position_generation)-double(obj.LastSourceGeneration),4294967296);
                if p.original_sample_hrt_us<=obj.LastBoardSampleUs||delta==0||delta>=2147483648
                    obj.fail("SOURCE_SAMPLE_OR_GENERATION_REGRESSION");
                end
            end
            if ~isempty(obj.find(p.source_ticket)),obj.fail("DUPLICATE_SOURCE_TICKET");end
            empty=find(cellfun(@isempty,obj.Sources),1);
            if isempty(empty),obj.fail("SOURCE_STORAGE_FULL");end
            obj.Sources{empty}=p;obj.SourcesRecorded=obj.SourcesRecorded+uint64(1);
            obj.LastBoardSampleUs=p.original_sample_hrt_us;obj.LastSourceGeneration=p.position_generation;
            obj.LastSourceReceiveNs=originalHostRxNs;
        end
        function submitted(obj,generation,ticket,originalSubmitNs)
            obj.time(originalSubmitNs);obj.requireInteger(generation);
            if ~isempty(obj.Pending)||generation<=obj.LastSubmit,obj.fail("OUTER_SUBMIT_ORDER");end
            p=obj.source(ticket);
            if originalSubmitNs<p.source_host_receive_ns,obj.fail("OUTER_SUBMIT_BEFORE_SOURCE");end
            obj.Pending=struct('generation',generation,'sample',p,'original_submit_ns',originalSubmitNs);
            obj.LastSubmit=generation;obj.OuterSubmissions=obj.OuterSubmissions+uint64(1);
        end
        function committed(obj,response,originalCommitNs)
            obj.time(originalCommitNs);
            if isempty(obj.Pending),obj.fail("OUTER_COMMIT_WITHOUT_SUBMIT");end
            p=obj.Pending.sample;
            if ~isfield(response,'generation')||~isnumeric(response.generation) ...
                ||~isscalar(response.generation)||~isfinite(response.generation) ...
                ||response.generation~=fix(response.generation)||response.generation<1 ...
                ||uint64(response.generation)~=obj.Pending.generation
                obj.fail("OUTER_COMMIT_GENERATION");
            end
            if originalCommitNs<obj.Pending.original_submit_ns,obj.fail("OUTER_COMMIT_BEFORE_SUBMIT");end
            if ~isfield(response,'input_boundary'),obj.fail("OUTER_BOUNDARY_MISSING");end
            if ~isfield(response,'configuration_payload_sha256') ...
                ||~strcmpi(string(response.configuration_payload_sha256), ...
                    string(obj.Expected.configuration_payload_sha256))
                obj.fail("OUTER_CONFIGURATION_MISMATCH");
            end
            b=response.input_boundary;
            if ~all(isfield(b,{'estimate_generations','estimate_rx_ns'})) ...
                ||~isequal(double(b.estimate_generations(:)),repmat(double(p.position_generation),3,1)) ...
                ||~obj.exactReceipts(b.estimate_rx_ns,p.source_host_receive_ns)
                obj.fail("OUTER_SOURCE_ASSOCIATION_MISMATCH");
            end
            try,payload=gpenmpcNative.RflyOuterPayload(response);
            catch ex,obj.fail("OUTER_PAYLOAD__"+string(ex.identifier));end
            expires=obj.expiry(p.source_host_receive_ns,obj.OuterMaxAgeNs);
            if originalCommitNs>expires,obj.fail("OUTER_ORIGINAL_EXPIRY");end
            obj.Outer=struct('generation',obj.Pending.generation,'sample',p, ...
                'creation_ns',originalCommitNs,'expiry_ns',expires,'payload',payload);
            obj.Pending=[];obj.OuterCommits=obj.OuterCommits+uint64(1);
        end
        function [context,binding]=reference(obj,generation,ticket,referenceNed,originalCreationNs)
            obj.time(originalCreationNs);obj.requireInteger(generation);
            if isempty(obj.Outer),obj.fail("NO_COMMITTED_OUTER");end
            p=obj.source(ticket);
            if ~isa(referenceNed,'double')||numel(referenceNed)~=11||any(~isfinite(referenceNed(:)))
                obj.fail("REFERENCE_PAYLOAD");
            end
            if ~isempty(obj.Reference)&&generation<=obj.Reference.generation,obj.fail("REFERENCE_GENERATION_REUSE");end
            o=obj.Outer;expires=obj.expiry(p.source_host_receive_ns,obj.ReferenceMaxAgeNs);
            if originalCreationNs<p.source_host_receive_ns||originalCreationNs>expires ...
                ||originalCreationNs>o.expiry_ns||o.sample.original_sample_hrt_us>p.original_sample_hrt_us
                obj.fail("REFERENCE_LINEAGE_OR_EXPIRY");
            end
            config=uint8(sscanf(char(obj.Expected.configuration_payload_sha256),'%2x'));
            context=struct('configuration_sha256',config(:),'reference_generation',generation, ...
                'outer_generation',o.generation,'reference_source_ticket',p.source_ticket, ...
                'outer_source_ticket',o.sample.source_ticket, ...
                'reference_source_receipt_ns',p.source_host_receive_ns, ...
                'reference_creation_ns',originalCreationNs,'reference_expiry_ns',expires, ...
                'outer_source_receipt_ns',o.sample.source_host_receive_ns, ...
                'outer_creation_ns',o.creation_ns,'outer_expiry_ns',o.expiry_ns, ...
                'reference_ned',referenceNed(:),'outer_payload',o.payload, ...
                'target_system',uint8(obj.Expected.system_id),'target_component',uint8(obj.Expected.component_id));
            binding=struct('snapshot_ticket',p.source_ticket,'reference_generation',generation, ...
                'outer_generation',o.generation,'configuration_sha256',config(:), ...
                'target_system',context.target_system,'target_component',context.target_component);
            % Use the inner transaction's command_generation.
            obj.Reference=struct('generation',generation,'sample',p,'context',context);
            obj.ReferencesBound=obj.ReferencesBound+uint64(1);
        end
        function retired=retireSource(obj,ticket)
            obj.alive();retired=false;idx=obj.find(ticket);if isempty(idx),return;end
            if (~isempty(obj.Pending)&&isequal(obj.Pending.sample.source_ticket,ticket(:))) ...
                ||(~isempty(obj.Outer)&&isequal(obj.Outer.sample.source_ticket,ticket(:))) ...
                ||(~isempty(obj.Reference)&&isequal(obj.Reference.sample.source_ticket,ticket(:)))
                return
            end
            obj.Sources{idx}=[];retired=true;
        end
        function r=status(obj)
            r=struct('failed',obj.Failed,'failure',obj.Failure,'sources_recorded',obj.SourcesRecorded, ...
                'retained_sources',sum(~cellfun(@isempty,obj.Sources)), ...
                'outer_submissions',obj.OuterSubmissions,'outer_commits',obj.OuterCommits, ...
                'references_bound',obj.ReferencesBound,'HOST_HRT_mapping',false,'board_actions',0);
            r.last_original_source_receive_ns=obj.LastSourceReceiveNs;
            r.last_processing_event_ns=obj.LastHost;
        end
    end
    methods (Access=private)
        function idx=find(obj,ticket)
            idx=[];for k=1:obj.Capacity
                if ~isempty(obj.Sources{k})&&isequal(obj.Sources{k}.source_ticket,ticket(:)),idx=k;return;end
            end
        end
        function p=source(obj,ticket)
            idx=obj.find(ticket);if isempty(idx),obj.fail("UNKNOWN_SOURCE_TICKET");end
            p=obj.Sources{idx};
        end
        function time(obj,t)
            obj.alive();obj.requireInteger(t);if t<obj.LastHost,obj.fail("HOST_EVENT_TIME_REVERSED");end
            obj.LastHost=t;
        end
        function alive(obj)
            assert(~obj.Failed,'gpenmpcNative:HostContextFailed','%s',obj.Failure);
        end
        function requireInteger(obj,value)
            try,obj.integer(value);
            catch,obj.fail("INVALID_ORIGINAL_INTEGER_EVENT");end
        end
        function fail(obj,why)
            obj.Failed=true;obj.Failure=why;error('gpenmpcNative:HostContextFailClosed','%s',why);
        end
    end
    methods (Static,Access=private)
        function ok=exactReceipts(values,original)
            ok=false;
            if ~isnumeric(values)||numel(values)~=3||~isa(original,'uint64'),return;end
            if isa(values,'uint64')
                ok=all(values(:)==original);return
            end
            % Accept legacy double receipts only when conversion to uint64 is exact.
            if ~isa(values,'double')||original>uint64(flintmax) ...
                ||any(~isfinite(values(:)))||any(values(:)<0) ...
                ||any(values(:)>flintmax)||any(values(:)~=fix(values(:))),return;end
            ok=all(uint64(values(:))==original);
        end
        function integer(v)
            assert(isa(v,'uint64')&&isscalar(v)&&v>0,'gpenmpcNative:HostContextInteger','Original positive uint64 required.');
        end
        function e=expiry(start,duration)
            assert(start<=intmax('uint64')-duration,'gpenmpcNative:HostContextOverflow','Original expiry overflow.');
            e=start+duration;
        end
    end
end
