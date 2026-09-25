classdef RflyLocalOriginalGetterBuffer < handle
    % Retain a bounded observation window for exact content matching.
    % Uniqueness is scoped to that window; the caller archives raw batches.
    properties (SetAccess=private)
        Failed=false
        Failure=""
        Accepted=uint64(0)
        Retired=uint64(0)
        Matched=uint64(0)
        ObservationOnly=uint64(0)
    end
    properties (Access=private)
        Capacity
        Records
        Reads
        Count=0
        Last=[]
        LastEvent=uint64(0)
        LastReadNs=uint64(0)
        LastCounters=zeros(10,1,'uint64')
        Producer=uint32(0)
        Nonce=uint64(0)
        LastSource=[]
        Lease=[]
        DllSha
        MatchingStarted=false
        MatchingClosed=false
        RuntimeStateOnly=false
    end
    methods
        function obj=RflyLocalOriginalGetterBuffer(capacity,dllSha)
            assert(isscalar(capacity)&&capacity==fix(capacity)&&capacity>=256&&capacity<=8192);
            assert(any(strcmpi(dllSha,{'990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E', ...
                '9F55AB72F75C3987BD5FB3F612276BDA6DE792E7367EE9F561FC55A75B8D72BD', ...
                'D536EACE85EBA30A6CE07EF5B38FF108B132C9E3B230A3C46E93F3E1C9CA6E12'})));
            obj.Capacity=capacity;obj.Records=zeros(336,capacity,'uint8');obj.Reads=zeros(capacity,1,'uint64');
            obj.DllSha=upper(char(dllSha));
        end
        function ingest(obj,batch,observationOnly)
            if nargin<3,observationOnly=false;end
            try
                obj.alive();assert(islogical(observationOnly)&&isscalar(observationOnly));
                if observationOnly
                    assert(~obj.MatchingStarted||obj.MatchingClosed, ...
                        'gpenmpcNative:GetterActiveObservation','Active unmatched observations cannot be discarded.');
                else
                    assert(~obj.MatchingClosed,'gpenmpcNative:GetterMatchingClosed','A safety-closed source window cannot reopen.');
                    obj.MatchingStarted=true;
                end
                assert(isstruct(batch)&&all(isfield(batch,{'records','original_read_ns','ring_header', ...
                    'step_status','failed','clock_domain','snapshot_coherent','board_authority'})) ...
                    &&isequal(batch.failed,false)&&isequal(batch.board_authority,false), 'gpenmpcNative:GetterBatch','Original MEX batch required.');
                n=size(batch.records,2);assert(isa(batch.records,'uint8')&&size(batch.records,1)==336&&n<=256 ...
                    &&isa(batch.original_read_ns,'uint64')&&numel(batch.original_read_ns)==n,'gpenmpcNative:GetterBatch','Malformed record array.');
                if n==0
                    % Still inspect any observed sticky fault even without a
                    % new record. Fully prepared/no-producer state is allowed.
                    assert(all(batch.ring_header(81:84)==0)&&all(batch.step_status(57:64)==0) ...
                        &&all(batch.step_status(73:80)==0),'gpenmpcNative:GetterEmptyFault','Empty batch retains original fault.');return
                end
                if ~observationOnly
                    if obj.RuntimeStateOnly&&obj.Count+n>obj.Capacity&&isempty(obj.Lease)
                        % Retain the recent causal input window; the runner stores raw batches.
                        remove=obj.Count+n-obj.Capacity;left=obj.Count-remove;
                        obj.Records(:,1:left)=obj.Records(:,remove+1:obj.Count);
                        obj.Reads(1:left)=obj.Reads(remove+1:obj.Count);obj.Count=left;
                    end
                    assert(obj.Count+n<=obj.Capacity,'gpenmpcNative:GetterCapacity','Unretired source observations cannot be overwritten.');
                end
                w=obj.toWindow(batch,batch.records,batch.original_read_ns);
                [who,c]=gpenmpcNative.validateRflyOriginalStreamingWindow(w,n);
                if obj.Nonce>0
                    assert(who.peer_nonce==obj.Nonce&&who.producer_pid==obj.Producer&&all(c>=obj.LastCounters), ...
                        'gpenmpcNative:GetterOwnerDrift','Owner or monotonic original counters changed.');
                end
                ord=readLE(reshape(batch.records(1:8,:),[],1),'uint64');r=batch.original_read_ns(:);
                assert(ord(1)==obj.LastEvent+uint64(1)&&r(1)>=obj.LastReadNs&&r(1)>0 ...
                    &&all(r(2:end)>=r(1:end-1)),'gpenmpcNative:GetterOrder','Record/receive order gap or reversal.');
                if observationOnly
                    % Before association or after terminal safety closure, validate continuity
                    % without retaining records as source-match candidates.
                    obj.ObservationOnly=obj.ObservationOnly+uint64(n);
                else
                    obj.Records(:,obj.Count+(1:n))=batch.records;obj.Reads(obj.Count+(1:n))=r;
                    obj.Count=obj.Count+n;
                end
                obj.LastEvent=ord(end);obj.LastReadNs=r(end);obj.Accepted=obj.Accepted+uint64(n);
                obj.Last=batch;obj.LastCounters=c;obj.Nonce=who.peer_nonce;obj.Producer=who.producer_pid;
            catch ex,obj.Failed=true;obj.Failure=string(ex.identifier);rethrow(ex);end
        end
        function observeOnly(obj,batch)
            obj.ingest(batch,true);
        end
        function useRuntimeStateOnly(obj)
            obj.alive();assert(~obj.MatchingStarted);obj.RuntimeStateOnly=true;
        end
        function beginSourceMatching(obj)
            % The live runner calls this once immediately before the board
            % task start command.  From that point onward every original
            % getter row can belong to the first exported source generation
            % and must be retained; waiting for the first snapshot before
            % opening the matching window can discard its causal inputs.
            obj.alive();
            assert(~obj.MatchingStarted&&~obj.MatchingClosed, ...
                'gpenmpcNative:GetterMatchingAlreadyStarted', ...
                'The no-eviction source-matching window may start only once.');
            obj.MatchingStarted=true;
        end
        function closeSourceMatching(obj)
            % Preserve any unmatched rows/lease for evidence. Closure grants
            % no match/retirement and is irreversible within this buffer.
            obj.alive();obj.MatchingClosed=true;
        end
        function [binding,receipt]=bind(obj,rls,rxNs,taskSource,phase,environment)
            try
                obj.alive();
                assert(obj.MatchingStarted&&~obj.MatchingClosed, ...
                    'gpenmpcNative:GetterMatchingInactive','Observation-only or safety-closed data cannot bind a source.');
                source=gpenmpcNative.RflyLocalSnapshotDecoder(rls,rxNs);
                if ~isempty(obj.Lease)
                    assert(~obj.Lease.complete&&source.source_generation==obj.Lease.source.source_generation ...
                        &&isequal(source.original_bytes,obj.Lease.source.original_bytes) ...
                        &&rxNs==obj.Lease.source.original_host_receive_ns, ...
                        'gpenmpcNative:GetterLease','Only the same incomplete original binding may wait for its ACK.');
                end
                if ~isempty(obj.LastSource)
                    assert(isequal(source.identity,obj.LastSource.identity)&&source.source_generation>obj.LastSource.source_generation ...
                        &&source.original_sample_us>obj.LastSource.original_sample_us,'gpenmpcNative:GetterSourceOrder','Source cannot be replayed.');
                end
                assert(obj.Count>0,'gpenmpcNative:GetterEmpty','No original getter is available.');
                first=1;
                if obj.RuntimeStateOnly
                    % Use the latest validated observation; the runner retains historical raw records.
                    first=obj.Count;
                end
                w=obj.toWindow(obj.Last,obj.Records(:,first:obj.Count),obj.Reads(first:obj.Count));
                if obj.RuntimeStateOnly,w.runtime_decision_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();end
                [b,receipt]=gpenmpcNative.bindRflyLocalOriginalInputs(rls,rxNs,w,taskSource,{phase},{environment},obj.RuntimeStateOnly);binding=b{1};
                g=receipt.matching.getter_index(1);
                if g>0
                    firstLease=isempty(obj.Lease);
                    retainedIndex=first+g-1;
                    obj.Lease=struct('event',readLE(obj.Records(1:8,retainedIndex),'uint64'),'source',source,'count',retainedIndex, ...
                        'complete',binding.numerical_inputs_complete);
                    if firstLease,obj.Matched=obj.Matched+uint64(1);end;receipt.retirement_lease=obj.Lease;
                end
            catch ex,obj.Failed=true;obj.Failure=string(ex.identifier);rethrow(ex);end
        end
        function retireResolvedSource(obj,sourceGeneration,sourceObservation)
            obj.alive();assert(~obj.MatchingClosed,'gpenmpcNative:GetterMatchingClosed','No source retirement after safety closure.');
            if obj.RuntimeStateOnly&&isempty(obj.Lease)&&nargin==3
                assert(sourceObservation.source_generation==sourceGeneration);
                obj.LastSource=sourceObservation;return % Return the retired observation.
            end
            assert(~isempty(obj.Lease)&&isa(sourceGeneration,'uint64') ...
                &&sourceGeneration==obj.Lease.source.source_generation,'gpenmpcNative:GetterRetireLease','Exact resolved source lease required.');
            n=obj.Lease.count;
            % Reuse the latest observation within its 50 ms age until superseded.
            % Preserve its original timestamp.
            if obj.RuntimeStateOnly&&n==obj.Count,n=n-1;end
            left=obj.Count-n;
            obj.Records(:,1:left)=obj.Records(:,n+1:obj.Count);obj.Reads(1:left)=obj.Reads(n+1:obj.Count);
            obj.Count=left;obj.Retired=obj.Retired+uint64(n);obj.LastSource=obj.Lease.source;obj.Lease=[];
        end
        function s=status(obj)
            s=struct('accepted_getters',obj.Accepted,'explicitly_retired_getters',obj.Retired,'retained_getters',obj.Count, ...
                'matched_sources',obj.Matched,'failed',obj.Failed,'failure',obj.Failure, ...
                'observation_only_getters',obj.ObservationOnly,'source_matching_started',obj.MatchingStarted, ...
                'source_matching_closed',obj.MatchingClosed, ...
                'capacity',obj.Capacity,'original_timestamps_renewed',false,'running_snapshot_coherent',false, ...
                'plant_runs',0,'COM',0,'board_authority',false);
        end
    end
    methods (Access=private)
        function alive(obj),assert(~obj.Failed,'gpenmpcNative:GetterFailed','No reset of a faulted source buffer.');end
        function w=toWindow(obj,b,records,reads)
            w=struct('records',records,'original_read_ns',reads,'original_read_clock_domain',b.clock_domain, ...
                'ring_section',b.ring_header,'step_status',b.step_status,'instrumented_dll_sha256',obj.DllSha, ...
                'snapshot_coherent',b.snapshot_coherent,'streaming_snapshot',true);
        end
    end
end
function v=readLE(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
