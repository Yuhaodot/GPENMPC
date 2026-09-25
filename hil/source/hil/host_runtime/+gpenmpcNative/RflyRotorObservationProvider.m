classdef RflyRotorObservationProvider < handle
    % Map same-IO rotor records into runtime observations.
    % Validate CRC, identity and sequence before bounded clock acquisition.
    properties (SetAccess=private)
        Failed=false
        Failure=""
    end
    properties (Access=private)
        Expected
        Capacity
        RuntimeAgeNs
        TaskSha
        Effectiveness
        Queue={}
        DecoderState=[]
        AcceptedRecord=[]
        LastSeen=[]
        Clock=[]
        ClockPollNs=uint64(0)
        ClockPollS=-Inf
        ClockAgeBound=NaN
        LastModelTime=-Inf
        LastNowNs=uint64(0)
        LastWallS=-Inf
        LastIoReceipt=[]
        InspectingRecord=[]
        FirstFault=[]
        NewRecords=uint64(0)
        Duplicates=uint64(0)
        Accepted=uint64(0)
    end
    methods
        function obj=RflyRotorObservationProvider(expected,taskWindProvider,maximumQueue,maximumRuntimeAgeNs)
            assert(isstruct(expected)&&isscalar(expected)&&all(isfield(expected, ...
                {'session_token','dll_sha256','maximum_source_age_s','maximum_receive_age_s'})) ...
                &&isa(expected.session_token,'uint64')&&isscalar(expected.session_token)&&expected.session_token>0 ...
                &&isa(expected.dll_sha256,'uint8')&&isequal(size(expected.dll_sha256),[32,1])&&any(expected.dll_sha256) ...
                &&positive(expected.maximum_source_age_s)&&positive(expected.maximum_receive_age_s), ...
                'gpenmpcNative:RotorExpected','Frozen decoder identity and source/receive bounds required.');
            assert(isa(taskWindProvider,'gpenmpcNative.RflyTaskWindProvider')&&~taskWindProvider.Failed, ...
                'gpenmpcNative:RotorTask','Verified actual saved-task provider required.');
            assert(positive(maximumQueue)&&maximumQueue==fix(maximumQueue) ...
                &&isa(maximumRuntimeAgeNs,'uint64')&&isscalar(maximumRuntimeAgeNs)&&maximumRuntimeAgeNs>0, ...
                'gpenmpcNative:RotorBound','Explicit bounded hold and existing runtime age required.');
            obj.Expected=expected;obj.Capacity=maximumQueue;obj.RuntimeAgeNs=maximumRuntimeAgeNs;
            obj.TaskSha=taskWindProvider.TaskSha256;obj.Effectiveness=taskWindProvider.RotorEffectiveness;
        end
        function r=ingest(obj,ioReceipt)
            try
                obj.alive();obj.InspectingRecord=[];
                assert(isstruct(ioReceipt)&&isscalar(ioReceipt)&&string(ioReceipt.schema)=="RFLY_SAME_IO_RAW_ROTOR_OBSERVATIONS_V1" ...
                    &&iscell(ioReceipt.records)&&numel(ioReceipt.records)<=obj.Capacity+1, ...
                    'gpenmpcNative:RotorIoShape','Bounded original same-I/O receipt required.');
                obj.LastIoReceipt=ioReceipt;
                assert(isempty(ioReceipt.failure),'gpenmpcNative:RotorIoFailure','Original I/O failure is terminal.');
                assert(string(ioReceipt.clock_source)=="OFFICIAL_COPTERSIM_MODEL_DIAGNOSTIC_NOT_ROTOR_PACKET" ...
                    &&integerNs(ioReceipt.original_poll_ns)&&ioReceipt.original_poll_ns>=obj.ClockPollNs ...
                    &&nonnegative(ioReceipt.original_io_poll_s)&&ioReceipt.original_io_poll_s>=obj.ClockPollS, ...
                    'gpenmpcNative:RotorIoClock','Original I/O poll clocks must advance independently.');
                c=ioReceipt.independent_model_clock;
                assert(isstruct(c)&&isscalar(c)&&all(isfield(c,{'model_ready','fatal_reason','maximum_age_s', ...
                    'receive_age_s','source_progress_age_s','last_source_time_s','progress_observed'})) ...
                    &&islogical(c.model_ready)&&isscalar(c.model_ready)&&positive(c.maximum_age_s) ...
                    &&isempty(c.fatal_reason),'gpenmpcNative:RotorModelClock','Independent model diagnostic is invalid or faulted.');
                if isnan(obj.ClockAgeBound),obj.ClockAgeBound=c.maximum_age_s;end
                assert(c.maximum_age_s==obj.ClockAgeBound,'gpenmpcNative:RotorClockBound','Diagnostic age bound cannot change.');
                if isfinite(c.last_source_time_s)
                    assert(c.last_source_time_s>=obj.LastModelTime,'gpenmpcNative:RotorModelClockReversed','Independent model time reversed.');
                    obj.LastModelTime=c.last_source_time_s;
                end
                obj.Clock=c;obj.ClockPollNs=ioReceipt.original_poll_ns;obj.ClockPollS=ioReceipt.original_io_poll_s;
                for k=1:numel(ioReceipt.records)
                    row=ioReceipt.records{k};obj.InspectingRecord=row;
                    assert(isstruct(row)&&isscalar(row)&&all(isfield(row,{'bytes','original_host_receive_ns', ...
                        'original_io_receive_s','sender_address','sender_port','receive_semantics'})) ...
                        &&integerNs(row.original_host_receive_ns)&&row.original_host_receive_ns<=obj.ClockPollNs ...
                        &&nonnegative(row.original_io_receive_s)&&row.original_io_receive_s<=obj.ClockPollS ...
                        &&strcmp(row.sender_address,'127.0.0.1') ...
                        &&string(row.receive_semantics)=="MATLAB_DEQUEUE_NOT_KERNEL_ARRIVAL", ...
                        'gpenmpcNative:RotorRecord','Original rotor bytes and same-I/O receive clocks required.');
                    [p,why]=m600check.inspectCanonicalRotorObserverPacket(row.bytes,obj.Expected);
                    assert(isempty(why),'gpenmpcNative:RotorPacket','%s',why);
                    duplicate=false;
                    if isempty(obj.LastSeen)
                        assert(p.generation==1,'gpenmpcNative:RotorGeneration','INITIAL_GENERATION_NOT_ONE');
                    else
                        last=obj.LastSeen;
                        assert(row.original_host_receive_ns>=last.row.original_host_receive_ns ...
                            &&row.original_io_receive_s>=last.row.original_io_receive_s, ...
                            'gpenmpcNative:RotorReceiveReversed','Original receive clocks reversed.');
                        if p.generation==last.parsed.generation
                            assert(isequal(row.bytes(:),last.row.bytes(:)), ...
                                'gpenmpcNative:RotorGeneration','GENERATION_CONFLICT');
                            duplicate=true;
                        else
                            assert(last.parsed.generation<intmax('uint64')&&p.generation==last.parsed.generation+uint64(1), ...
                                'gpenmpcNative:RotorGeneration','GENERATION_GAP_REVERSE_OR_EXHAUSTION');
                            assert(p.sim_time_s>last.parsed.sim_time_s, ...
                                'gpenmpcNative:RotorSourceTime','SOURCE_TIME_NOT_ADVANCING');
                        end
                    end
                    if duplicate,obj.Duplicates=obj.Duplicates+uint64(1);continue;end
                    assert(numel(obj.Queue)<obj.Capacity,'gpenmpcNative:RotorQueueFull', ...
                        'Independent clock hold queue full; existing bytes cannot be overwritten.');
                    item=struct('row',row,'parsed',p);obj.Queue{end+1}=item;obj.LastSeen=item;
                    obj.NewRecords=obj.NewRecords+uint64(1);
                end
                r=obj.status();r.original_io_receipt=ioReceipt;
            catch ex,obj.latch(ex);rethrow(ex);end
        end
        function [v,r]=current(obj,actualNowNs,actualWallS)
            v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',false);
            try
                obj.alive();obj.InspectingRecord=[];
                assert(integerNs(actualNowNs)&&actualNowNs>=obj.LastNowNs&&actualNowNs>=obj.ClockPollNs ...
                    &&nonnegative(actualWallS)&&actualWallS>=obj.LastWallS&&actualWallS>=obj.ClockPollS, ...
                    'gpenmpcNative:RotorCurrentClock','Actual monotonic HOST and same-I/O wall clocks required.');
                obj.LastNowNs=actualNowNs;obj.LastWallS=actualWallS;
                % Preserve receive timestamps and queued samples during clock acquisition.
                for k=1:numel(obj.Queue)
                    obj.InspectingRecord=obj.Queue{k}.row;
                    obj.receiveFresh(obj.Queue{k}.row,actualNowNs,actualWallS);
                end
                r=obj.status();r.status='WAITING_FOR_INDEPENDENT_MODEL_CLOCK';
                if isempty(obj.Clock)||~obj.Clock.model_ready
                    if isempty(obj.Queue)&&~isempty(obj.AcceptedRecord)
                        obj.receiveFresh(obj.AcceptedRecord,actualNowNs,actualWallS);
                    end
                    return
                end
                c=obj.Clock;elapsed=actualWallS-obj.ClockPollS;
                assert(nonnegative(c.receive_age_s)&&nonnegative(c.source_progress_age_s) ...
                    &&c.progress_observed&&nonnegative(c.last_source_time_s) ...
                    &&c.receive_age_s+elapsed<=obj.ClockAgeBound ...
                    &&c.source_progress_age_s+elapsed<=obj.ClockAgeBound, ...
                    'gpenmpcNative:RotorDiagnosticStale','Independent diagnostic does not renew at provider poll.');
                while ~isempty(obj.Queue)
                    a=obj.Queue{1};obj.InspectingRecord=a.row;
                    if a.parsed.sim_time_s>c.last_source_time_s+1e-12,break;end % existing decoder roundoff only
                    clk=struct('now_sim_time_s',c.last_source_time_s,'now_wall_time_s',actualWallS, ...
                        'receive_wall_time_s',a.row.original_io_receive_s);
                    [sample,obj.DecoderState]=m600check.decodeCanonicalRotorObserver( ...
                        a.row.bytes,obj.Expected,clk,obj.DecoderState);
                    assert(sample.valid,'gpenmpcNative:RotorDecoder','%s',sample.reason);
                    obj.AcceptedRecord=a.row;obj.Accepted=obj.Accepted+uint64(1);obj.Queue(1)=[];
                end
                if isempty(obj.AcceptedRecord),return;end
                obj.InspectingRecord=obj.AcceptedRecord;
                obj.receiveFresh(obj.AcceptedRecord,actualNowNs,actualWallS);
                clk=struct('now_sim_time_s',c.last_source_time_s,'now_wall_time_s',actualWallS, ...
                    'receive_wall_time_s',obj.AcceptedRecord.original_io_receive_s);
                [sample,obj.DecoderState]=m600check.decodeCanonicalRotorObserver(uint8([]),obj.Expected,clk,obj.DecoderState);
                assert(sample.valid,'gpenmpcNative:RotorDecoder','%s',sample.reason);
                v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',true, ...
                    'generation',sample.generation,'rx_ns',obj.AcceptedRecord.original_host_receive_ns, ...
                    'ordering','SOFTWARE_M600_ORDER','packet_rotor_order','ROTORS_1_TO_6', ...
                    'rotor_thrust_state_n',sample.rotor_thrust_state_n,'thrust_effectiveness',obj.Effectiveness, ...
                    'state_source','SAME_M600_ACCEPTED_STEP_ROTOR_LAG_STATE','plant_session_id',sample.session_token);
                r=obj.status();r.status='VALID_ORIGINAL_SAME_CORE_ROTOR_OBSERVATION';
                r.original_record=obj.AcceptedRecord;r.decoder_sample=sample;
                r.independent_model_clock=c;r.original_clock_poll_ns=obj.ClockPollNs;
                r.original_clock_poll_s=obj.ClockPollS;r.task_sha256=obj.TaskSha;
                r.task_effectiveness_source='SAVED_TASK_PLANT_MISMATCH_THRUST_EFFECTIVENESS_BY_ROTOR';
                r.rotor_permutation_applied=false;r.source_receive_time_renewed=false;
            catch ex,obj.latch(ex);rethrow(ex);end
        end
        function r=status(obj)
            r=struct('schema','RFLY_ROTOR_OBSERVATION_PROVIDER_V1','failed',obj.Failed,'failure',obj.Failure, ...
                'pending_count',numel(obj.Queue),'capacity',obj.Capacity,'new_records',obj.NewRecords, ...
                'duplicates_ignored',obj.Duplicates,'accepted_count',obj.Accepted, ...
                'runtime_origin_attested',false,'plant_truth_position_used',false, ...
                'publication_authority',false,'HOST_IO_clock_mapping',false,'connections',0,'hardware_actions',0);
        end
        function r=evidence(obj)
            r=struct('status',obj.status(),'pending',{obj.Queue},'last_accepted_record',obj.AcceptedRecord, ...
                'last_io_receipt',obj.LastIoReceipt,'first_fault',obj.FirstFault,'decoder_state',obj.DecoderState, ...
                'raw_persistence_proven',false);
        end
    end
    methods (Access=private)
        function receiveFresh(obj,row,now,wall)
            assert(now>=row.original_host_receive_ns&&now-row.original_host_receive_ns<=obj.RuntimeAgeNs ...
                &&wall>=row.original_io_receive_s ...
                &&wall-row.original_io_receive_s<=obj.Expected.maximum_receive_age_s+1e-12, ...
                'gpenmpcNative:RotorOriginalReceiveExpired','Original rotor receive age expired, including bounded hold.');
        end
        function alive(obj)
            assert(~obj.Failed,'gpenmpcNative:RotorProviderFailed','%s',obj.Failure);
        end
        function latch(obj,ex)
            if obj.Failed,return;end
            obj.Failed=true;obj.Failure=string(ex.identifier)+"__"+string(ex.message);
            obj.FirstFault=struct('identifier',string(ex.identifier),'message',string(ex.message), ...
                'record',obj.InspectingRecord,'original_io_receipt',obj.LastIoReceipt);
        end
    end
end
function yes=integerNs(v),yes=isa(v,'uint64')&&isscalar(v)&&v>0;end
function yes=nonnegative(v),yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0;end
function yes=positive(v),yes=nonnegative(v)&&v>0;end
