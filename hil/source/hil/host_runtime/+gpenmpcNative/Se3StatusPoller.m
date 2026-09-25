classdef Se3StatusPoller < handle
    %SE3STATUSPOLLER Nonblocking uORB status observation over the same COM owner.

    properties (SetAccess=private)
        Latest = struct()
        Failed (1,1) logical = false
        FailureCode (1,1) string = ""
        QueryOpen (1,1) logical = false
        QueryStartedNs (1,1) double = 0
        NextQueryNs (1,1) double = 0
        Buffer (1,:) uint8 = uint8([])
        StatusGeneration (1,1) uint64 = uint64(0)
        QueriesSent (1,1) uint64 = uint64(0)
        ResponsesCompleted (1,1) uint64 = uint64(0)
        ChunksReceived (1,1) uint64 = uint64(0)
        BytesReceived (1,1) uint64 = uint64(0)
        ActiveGuardsIssued (1,1) uint64 = uint64(0)
    end

    properties (Constant)
        Query = "listener gpenmpc_se3_control_status 0 1"
        Source = "PX4_UORB_GPENMPC_SE3_CONTROL_STATUS"
        MaximumGuardAgeNs = 100000000
        QueryIntervalNs = 50000000
        QueryTimeoutNs = 90000000
        MaximumResponseBytes = 32768
    end

    methods
        function sent=tick(obj,link,nowNs)
            obj.requireUsable(nowNs);
            if obj.QueryOpen
                if nowNs-obj.QueryStartedNs>obj.QueryTimeoutNs
                    obj.fail("STATUS_QUERY_TIMEOUT");
                end
                sent=false;return
            end
            if nowNs<obj.NextQueryNs,sent=false;return,end
            link.sendSerialControl(unicode2native(char(newline+obj.Query+newline), ...
                'US-ASCII'));
            obj.QueryOpen=true;obj.QueryStartedNs=nowNs;
            obj.Buffer=uint8([]);
            obj.QueriesSent=obj.QueriesSent+uint64(1);
            sent=true;
        end

        function completed=accept(obj,message,rxNs)
            obj.requireUsable(rxNs);
            assert(gpenmpcNative.MavlinkSerialLink.messageName(message)== ...
                "SERIAL_CONTROL",'gpenmpcNative:Se3StatusDispatch', ...
                'Only SERIAL_CONTROL may be dispatched to this poller.');
            payload=message.Payload;
            if double(payload.device)~=10,completed=false;return,end
            count=double(payload.count);
            if count<=0,completed=false;return,end
            if ~obj.QueryOpen,obj.fail("UNSOLICITED_SHELL_RESPONSE");end
            obj.Buffer=[obj.Buffer,uint8(payload.data(1:count))];
            obj.ChunksReceived=obj.ChunksReceived+uint64(1);
            obj.BytesReceived=obj.BytesReceived+uint64(count);
            if numel(obj.Buffer)>obj.MaximumResponseBytes
                obj.fail("STATUS_RESPONSE_OVERSIZE");
            end
            text=obj.cleanText(obj.Buffer);
            topicAt=strfind(text,'TOPIC: gpenmpc_se3_control_status');
            if isempty(topicAt)||~contains(text(topicAt(end):end),'nsh>')
                completed=false;return
            end
            atom=gpenmpcNative.Se3StatusPoller.parseAtom(obj.Buffer,rxNs);
            obj.StatusGeneration=obj.StatusGeneration+uint64(1);
            atom.status_generation=double(obj.StatusGeneration);
            if ~isempty(fieldnames(obj.Latest))
                for name=["timestamp","input_sample_count","output_publish_count"]
                    before=obj.Latest.(name);after=atom.(name);
                    if name=="timestamp"&&after<=before
                        obj.fail("STATUS_TIMESTAMP_NOT_ADVANCING");
                    elseif name~="timestamp"&&after<before
                        obj.fail("STATUS_COUNTER_REGRESSION");
                    end
                end
            end
            obj.Latest=atom;obj.QueryOpen=false;
            obj.NextQueryNs=rxNs+obj.QueryIntervalNs;
            obj.Buffer=uint8([]);
            obj.ResponsesCompleted=obj.ResponsesCompleted+uint64(1);
            completed=true;
        end

        function guard=activeGuard(obj,nowNs)
            obj.requireUsable(nowNs);
            assert(~isempty(fieldnames(obj.Latest)), ...
                'gpenmpcNative:Se3StatusAbsent','STATUS_GUARD_NOT_OBSERVED');
            guard=obj.Latest;age=nowNs-guard.rx_ns;
            assert(age>=0&&age<=obj.MaximumGuardAgeNs, ...
                'gpenmpcNative:Se3StatusStale','STATUS_GUARD_STALE');
            required={ ...
                'active','state_valid','reference_fresh','segment_fresh', ...
                'native_position_controller_disabled', ...
                'native_attitude_rate_allocator_enabled', ...
                'single_publisher_contract_pass'};
            accepted=guard.control_mode==1&&guard.failure_reason==0;
            for k=1:numel(required)
                accepted=accepted&&isequal(guard.(required{k}),true);
            end
            robust=guard.robust_acceleration_ned_mps2;
            accepted=accepted&&numel(robust)==3&&all(abs(robust)<=1e-7);
            assert(accepted,'gpenmpcNative:Se3StatusAdmission', ...
                'STATUS_GUARD_ATOMS_NOT_ADMITTED');
            obj.ActiveGuardsIssued=obj.ActiveGuardsIssued+uint64(1);
        end

        function value=status(obj)
            value=struct('schema','GPENMPC_MATLAB_NATIVE_SE3_STATUS_POLLER_V1', ...
                'query',obj.Query,'query_interval_ns',obj.QueryIntervalNs, ...
                'query_timeout_ns',obj.QueryTimeoutNs, ...
                'maximum_guard_age_ns',obj.MaximumGuardAgeNs, ...
                'query_open',obj.QueryOpen,'failed',obj.Failed, ...
                'failure_code',obj.FailureCode, ...
                'latest_status_generation',double(obj.StatusGeneration), ...
                'queries_sent',double(obj.QueriesSent), ...
                'responses_completed',double(obj.ResponsesCompleted), ...
                'chunks_received',double(obj.ChunksReceived), ...
                'bytes_received',double(obj.BytesReceived), ...
                'active_guards_issued',double(obj.ActiveGuardsIssued), ...
                'plant_truth_used',false);
        end
    end

    methods (Access=private)
        function requireUsable(obj,nowNs)
            assert(isfinite(nowNs)&&nowNs>=0&&nowNs==fix(nowNs), ...
                'gpenmpcNative:Se3StatusTime','INVALID_MONOTONIC_TIME');
            if obj.Failed
                error('gpenmpcNative:Se3StatusFailed','%s',obj.FailureCode);
            end
        end

        function fail(obj,code)
            obj.Failed=true;obj.FailureCode=string(code);
            error('gpenmpcNative:Se3StatusFailed','%s',code);
        end
    end

    methods (Static)
        function atom=parseAtom(raw,rxNs)
            text=gpenmpcNative.Se3StatusPoller.cleanText(raw);
            assert(contains(text,'TOPIC: gpenmpc_se3_control_status'), ...
                'gpenmpcNative:Se3StatusTopic','STATUS_TOPIC_MISSING');
            atom=struct('source', ...
                gpenmpcNative.Se3StatusPoller.Source, ...
                'transport','PX4_SERIAL_CONTROL_NSH_LISTENER', ...
                'valid',true,'plant_truth_used',false,'rx_ns',double(rxNs), ...
                'raw_text',string(text));
            numbers={ ...
                'timestamp','sample_timestamp','reference_timestamp', ...
                'segment_timestamp','input_sample_count','output_publish_count', ...
                'rejected_sample_count','reset_count','dt_s','reference_age_s', ...
                'segment_age_s','hover_thrust','total_mass_kg', ...
                'yaw_setpoint_rad','control_mode','failure_reason'};
            for k=1:numel(numbers)
                atom.(numbers{k})=gpenmpcNative.Se3StatusPoller.numberField( ...
                    text,numbers{k});
            end
            booleans={ ...
                'active','reference_fresh','segment_fresh','state_valid', ...
                'hover_thrust_fresh','native_position_controller_disabled', ...
                'native_attitude_rate_allocator_enabled', ...
                'single_publisher_contract_pass'};
            for k=1:numel(booleans)
                atom.(booleans{k})=gpenmpcNative.Se3StatusPoller.booleanField( ...
                    text,booleans{k});
            end
            vectors={ ...
                'robust_acceleration_ned_mps2', ...
                'commanded_acceleration_ned_mps2','normalized_thrust_ned', ...
                'position_ned_m','velocity_ned_mps', ...
                'reference_position_ned_m','reference_velocity_ned_mps', ...
                'reference_acceleration_ned_mps2', ...
                'nominal_feedback_acceleration_ned_mps2', ...
                'drag_feedforward_acceleration_ned_mps2'};
            for k=1:numel(vectors)
                atom.(vectors{k})=gpenmpcNative.Se3StatusPoller.vectorField( ...
                    text,vectors{k});
            end
            age=regexp(text,'timestamp:\s*[^\n]*\(([0-9.eE+\-]+)\s+seconds\s+ago\)', ...
                'tokens','once');
            if isempty(age),atom.listener_age_s=NaN;
            else,atom.listener_age_s=str2double(age{1});end
            assert(all(isfinite([atom.timestamp,atom.control_mode, ...
                atom.failure_reason])),'gpenmpcNative:Se3StatusIdentity', ...
                'STATUS_IDENTITY_FIELD_MISSING');
        end

        function text=cleanText(raw)
            text=native2unicode(uint8(raw(:).'),'UTF-8');
            text=regexprep(text,char([27,'\[[0-9;?]*[ -/]*[@-~]']),'');
            text=strrep(text,char(13),'');
        end

        function value=fieldText(text,name)
            token=regexp(text,['(?m)^\s*',regexptranslate('escape',name), ...
                ':\s*([^\n]+)'],'tokens','once');
            if isempty(token),value="";else,value=strtrim(string(token{1}));end
        end

        function value=numberField(text,name)
            raw=gpenmpcNative.Se3StatusPoller.fieldText(text,name);
            token=regexp(raw,'^[^\s(]+','match','once');
            if isempty(token),value=NaN;else,value=str2double(token);end
            if ~isfinite(value),value=NaN;end
        end

        function value=booleanField(text,name)
            raw=lower(gpenmpcNative.Se3StatusPoller.fieldText(text,name));
            if raw=="true",value=true;
            elseif raw=="false",value=false;
            else,value=[];
            end
        end

        function value=vectorField(text,name)
            raw=gpenmpcNative.Se3StatusPoller.fieldText(text,name);
            token=regexp(raw,'\[([^\]]+)\]','tokens','once');
            if isempty(token),value=[];return,end
            parts=split(string(token{1}),',');value=str2double(parts).';
            if numel(value)~=3||~all(isfinite(value)),value=[];end
        end
    end
end
