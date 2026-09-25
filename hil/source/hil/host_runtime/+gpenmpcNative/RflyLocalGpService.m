classdef RflyLocalGpService < handle
    % Predict once per board GP query.
    % The shared-link owner validates and assembles requests before process().
    properties (SetAccess=private)
        Failed=false
        Failure=""
        Closed=false
        Calls=uint64(0)
        Completed=uint64(0)
        LastRaw=[]
        Backend='MATLAB_ORIGINAL'
        BackendBinding=[]
        AsyncEnabled=false
        ReceiveInlineEnabled=false
        ReceiveContinuousEnabled=false
    end
    properties (Access=private)
        Assets
        Expected
        LastOutput=uint64(0)
        LastSource=uint64(0)
        LastSourceNs=uint64(0)
        LastReceiveNs=uint64(0)
        LastProcessingNs=uint64(0)
        Busy=false
        MexPredictor=[]
        AsyncPool=[]
        Future=[]
        PendingRequest=[]
        ReceivePartial=uint8([])
        ContinuousStop=[]
    end
    methods
        function obj=RflyLocalGpService(verifiedAssets,expected,prepared)
            % The application reuses its actual loadCanonicalAssets result;
            % no second worker/model/solver or per-tick file access is created.
            if nargin<3,prepared=[];end
            b=verifiedAssets.binding;[c,m]=gpenmpcNative.RflyLocalGpCodec.identities();
            assert(strcmpi(b.passport_sha256,'AA2A2C8F2282A4B41D06FB4AF042FF157B5322AA7040FDCA92123935615B00AE') ...
                &&strcmpi(b.effective_configuration_payload_sha256,hex(c)) ...
                &&strcmpi(b.gp_model_sha256,hex(m))&&b.verified_source_entries==160);
            assert(verifiedAssets.enmpc.coordinated_architecture_enabled ...
                &&strcmp(verifiedAssets.enmpc.coordinated_architecture_mode,'A1_COORDINATED_PHYSICAL') ...
                &&verifiedAssets.enmpc.command_continuity_enabled ...
                &&isequal(size(verifiedAssets.gp_model.inducing_standardized),[256 17]));
            resolved=string(which('gpenmpcSparseGpPredict'));
            assert(startsWith(lower(resolved),lower(string(verifiedAssets.sourceRoot)+filesep)), ...
                'gpenmpcNative:LocalGpSource','Original verified MATLAB GP must resolve.');
            for f={'uid','boot_generation','link_lifecycle_generation','confirmed_host_rx_ns'}
                v=expected.(f{1});assert(isa(v,'uint64')&&isscalar(v)&&v>0);
            end
            for f={'board_system','board_component','host_system','host_component'}
                v=expected.(f{1});assert(isa(v,'uint8')&&isscalar(v)&&v>0);
            end
            assert(~isempty(regexp(char(expected.execution_session_sha256),'^[0-9A-Fa-f]{64}$','once')));
            % Use MATLAB by default; an optional local binary must match its binding.
            % Resolve and hash at construction so process() performs no filesystem access.
            if isfield(expected,'gp_backend')
                if isempty(prepared)
                    [obj.MexPredictor,obj.BackendBinding]=checkedMexBackend(expected.gp_backend);
                else
                    [obj.MexPredictor,obj.BackendBinding]=reusePreparedMexBackend(prepared,expected.gp_backend,verifiedAssets);
                end
                obj.Backend='CANONICAL_GP_WIRE_MEX';
                obj.ReceiveInlineEnabled=isfield(expected.gp_backend,'receive_inline')&&isequal(expected.gp_backend.receive_inline,true);
                if ~isempty(prepared)&&~isempty(prepared.async_pool)
                    obj.AsyncPool=prepared.async_pool;obj.AsyncEnabled=true;
                    if isa(obj.AsyncPool,'parallel.BackgroundPool')||isa(obj.AsyncPool,'parallel.ThreadPool')
                        assert(strcmp(obj.BackendBinding.runtime_numerical_backend,'MATLAB_ORIGINAL_THREAD'), ...
                            'gpenmpcNative:LocalGpPrepared','Explicit original MATLAB thread backend required.');
                        obj.Backend='MATLAB_ORIGINAL_THREAD';
                    end
                end
            end
            obj.Assets=verifiedAssets;obj.Expected=expected;
            obj.LastReceiveNs=expected.confirmed_host_rx_ns;
            obj.LastProcessingNs=expected.confirmed_host_rx_ns;
        end
        function e=checkReceiveEndpoint(obj,localSystem,localComponent,remoteSystem,remoteComponent,continuous,stopFcn)
            if nargin<6,continuous=false;end
            e=obj.Expected;
            assert(obj.ReceiveInlineEnabled&&~obj.AsyncEnabled&&~obj.Closed&&~obj.Failed ...
                &&localSystem==e.host_system&&localComponent==e.host_component ...
                &&remoteSystem==e.board_system&&remoteComponent==e.board_component, ...
                'gpenmpcNative:LocalGpOrigin','Same registered receive/send endpoint only.');
            obj.ReceiveContinuousEnabled=logical(continuous);
            if continuous,assert(nargin==7&&isa(stopFcn,'function_handle'));obj.ContinuousStop=stopFcn;end
        end
        function results=receiveBatch(obj,data,deferHistory)
            if nargin<3,deferHistory=false;end
            % Process the request from the receive owner before history reconstruction.
            results={};
            if obj.Closed,return,end
            assert(obj.ReceiveInlineEnabled&&~obj.AsyncEnabled&&~obj.Failed,'gpenmpcNative:LocalGpClosed');
            e=obj.Expected;ids=uint8([e.board_system e.board_component e.host_system e.host_component]);
            now=gpenmpcNative.rflyOriginalHostMonotonicNs();
            assert(now>=obj.LastProcessingNs,'gpenmpcNative:LocalGpClock');
            if obj.ReceiveContinuousEnabled
                assert(isempty(data)||isfield(data,'native_gp'),'gpenmpcNative:LocalGpReceiveMissing');
                queries={};for j=1:numel(data),if ~isempty(data(j).native_gp),queries{end+1}=data(j).native_gp;end,end
            else
                [queries,obj.ReceivePartial]=obj.MexPredictor('collect',data,obj.ReceivePartial,ids,now, ...
                    uint64([e.uid e.boot_generation obj.LastOutput obj.LastSource obj.LastSourceNs]));
            end
            for k=1:numel(queries)
                if obj.ReceiveContinuousEnabled&&deferHistory
                    % The native path validates and replies inline; retain its result without
                    % delaying state input for typed-history reconstruction.
                    q=queries{k};assert(q.processing_ns>=q.original_host_receive_ns ...
                        &&q.processing_ns<=now&&q.send.messages_send_returned==3, ...
                        'gpenmpcNative:LocalGpReceiveIdentity');
                    obj.Calls=obj.Calls+uint64(1);obj.Completed=obj.Completed+uint64(1);
                    obj.LastReceiveNs=q.original_host_receive_ns;obj.LastProcessingNs=q.processing_ns;
                    r=struct('request',[],'reply_bytes',q.reply_bytes,'result18',q.result18, ...
                        'tunnel_payloads',{{}},'source_system',e.host_system,'source_component',e.host_component, ...
                        'actual_gp_call',obj.Calls,'original_host_receive_ns',q.original_host_receive_ns, ...
                        'processing_ns',q.processing_ns,'packets_sent',0,'control_publications',0);
                    obj.LastRaw=r;results{end+1}=struct('query',q,'computed',r,'send',q.send); %#ok<AGROW>
                    continue
                end
                q=queries{k};original=gpenmpcNative.RflyLocalGpCodec.decodeRequest(q.request_bytes);
                processed=now;sent=[];
                if obj.ReceiveContinuousEnabled
                    % Post-run/history consumption cannot renew native times
                    % or invoke the predictor again. Verify original identity
                    % and monotonic generation before updating MATLAB state.
                    assert(original.identity.uid==e.uid&&original.identity.boot_generation==e.boot_generation ...
                        &&original.output_generation>obj.LastOutput&&original.source_generation>obj.LastSource ...
                        &&original.source_timestamp_ns>obj.LastSourceNs ...
                        &&q.processing_ns>=q.original_host_receive_ns&&q.processing_ns<=now, ...
                        'gpenmpcNative:LocalGpReceiveIdentity');
                    processed=q.processing_ns;sent=q.send;
                end
                obj.Calls=obj.Calls+uint64(1);obj.Completed=obj.Completed+uint64(1);
                obj.LastOutput=original.output_generation;obj.LastSource=original.source_generation;
                obj.LastSourceNs=original.source_timestamp_ns;obj.LastReceiveNs=q.original_host_receive_ns;obj.LastProcessingNs=processed;
                % Split the validated native reply bytes without repeating prediction,
                % decoding or hashing on the receive-to-reply path.
                payloads=cell(3,1);
                for part=0:2
                    n=min(119,286-119*part);payload=zeros(128,1,'uint8');
                    payload(1)=uint8(144+part);payload(2:9)=q.request_bytes(39:46);
                    payload(10:9+n)=q.reply_bytes(119*part+1:119*part+n);
                    payloads{part+1}=struct('target_system',e.board_system,'target_component',e.board_component, ...
                        'payload_type',uint16(42002),'payload_length',uint8(9+n),'payload',payload);
                end
                r=struct('request',original,'reply_bytes',q.reply_bytes,'result18',q.result18, ...
                    'tunnel_payloads',{payloads}, ...
                    'source_system',e.host_system,'source_component',e.host_component, ...
                    'actual_gp_call',obj.Calls,'original_host_receive_ns',q.original_host_receive_ns, ...
                    'processing_ns',processed,'packets_sent',0,'control_publications',0);
                obj.LastRaw=r;
                results{end+1}=struct('query',q,'computed',r,'send',sent); %#ok<AGROW>
            end
        end
        function r=process(obj,requestBytes,originalReceiveNs,processingNs,origin)
            try
                assert(~obj.Failed&&~obj.Closed&&~obj.Busy,'gpenmpcNative:LocalGpClosed','No restart/reentry.');
                obj.Busy=true;unlock=onCleanup(@()obj.unlock()); %#ok<NASGU>
                obj.LastRaw=struct('request_bytes',requestBytes,'original_host_receive_ns',originalReceiveNs, ...
                    'processing_ns',processingNs,'origin',origin,'result18',[],'reply_bytes',[]);
                assert(isa(originalReceiveNs,'uint64')&&isscalar(originalReceiveNs)&&originalReceiveNs>0 ...
                    &&isa(processingNs,'uint64')&&isscalar(processingNs) ...
                    &&processingNs>=originalReceiveNs&&originalReceiveNs>=obj.LastReceiveNs ...
                    &&processingNs>=obj.LastProcessingNs, ...
                    'gpenmpcNative:LocalGpClock','Original receive and processing times must not regress.');
                e=obj.Expected;
                assert(isequal(origin.link_lifecycle_generation,e.link_lifecycle_generation) ...
                    &&strcmpi(origin.execution_session_sha256,e.execution_session_sha256), ...
                    'gpenmpcNative:LocalGpOrigin','Original registered link/session required.');
                q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(requestBytes);i=q.identity;
                assert(i.uid==e.uid&&i.boot_generation==e.boot_generation ...
                    &&i.system==e.board_system&&i.component==e.board_component, ...
                    'gpenmpcNative:LocalGpIdentity','Board request identity changed.');
                assert(q.output_generation>obj.LastOutput&&q.source_generation>obj.LastSource ...
                    &&q.source_timestamp_ns>obj.LastSourceNs,'gpenmpcNative:LocalGpReplay','Repeated/reversed query.');
                obj.LastOutput=q.output_generation;obj.LastSource=q.source_generation;
                obj.LastSourceNs=q.source_timestamp_ns;obj.LastReceiveNs=originalReceiveNs;
                obj.LastProcessingNs=processingNs;
                obj.Calls=obj.Calls+uint64(1);
                if isempty(obj.MexPredictor)||strcmp(obj.Backend,'MATLAB_ORIGINAL_THREAD')
                    y=gpenmpcNative.canonicalSparseGpFixedInput(obj.Assets.gp_model,q.request19(2:18).');
                    reply=[];
                else
                    [reply,y]=obj.MexPredictor(q.original_bytes);
                end
                obj.LastRaw.result18=y;
                % A close/reentrant failure must survive an in-flight call.
                % Calls/result18 remain evidence that computation took place;
                % cancellation does not pretend the GP computation rolled back.
                assert(~obj.Failed&&~obj.Closed&&obj.Busy, ...
                    'gpenmpcNative:LocalGpClosedDuringPrediction','Closed GP service cannot publish a reply.');
                if isempty(obj.MexPredictor)||strcmp(obj.Backend,'MATLAB_ORIGINAL_THREAD')
                    reply=gpenmpcNative.RflyLocalGpCodec.encodeReply(q,y);
                else
                    % Bind all eighteen result fields to the service's validated request.
                    decoded=gpenmpcNative.RflyLocalGpCodec.decodeReply(reply);
                    assert(isa(y,'double')&&isequal(size(y),[1 18])&&all(isfinite(y)) ...
                        &&isequal(decoded.result18,y(:)) ...
                        &&isequal(decoded.identity,q.identity) ...
                        &&decoded.source_timestamp_ns==q.source_timestamp_ns ...
                        &&decoded.source_generation==q.source_generation ...
                        &&decoded.output_generation==q.output_generation ...
                        &&isequal(decoded.original_request_sha256,q.original_request_sha256) ...
                        &&isequal(decoded.gp_model_sha256,q.gp_model_sha256), ...
                        'gpenmpcNative:LocalGpBackendReply','Bound MEX reply must retain the exact original request and numerical output.');
                end
                obj.LastRaw.reply_bytes=reply;
                r=struct('reply_bytes',reply,'tunnel_payloads',{gpenmpcNative.RflyLocalGpCodec.replyFragments( ...
                    reply,e.board_system,e.board_component)},'source_system',e.host_system, ...
                    'source_component',e.host_component,'request',q,'result18',y, ...
                    'actual_gp_call',obj.Calls,'original_host_receive_ns',originalReceiveNs, ...
                    'processing_ns',processingNs,'packets_sent',0,'control_publications',0);
                assert(~obj.Failed&&~obj.Closed&&obj.Busy, ...
                    'gpenmpcNative:LocalGpClosedBeforeReturn','Closed GP service cannot return a reply.');
                obj.Completed=obj.Completed+uint64(1);
            catch ex
                obj.Failed=true;obj.Closed=true;if strlength(obj.Failure)==0,obj.Failure=string(ex.identifier);end
                rethrow(ex)
            end
        end
        function begin(obj,requestBytes,originalReceiveNs,processingNs,origin)
            % Keep one numerical future in flight; the caller continues IO service.
            try
                assert(obj.AsyncEnabled&&~obj.Failed&&~obj.Closed&&~obj.Busy&&isempty(obj.Future), ...
                    'gpenmpcNative:LocalGpClosed','No restart/reentry.');
                e=obj.Expected;
                assert(isa(originalReceiveNs,'uint64')&&isscalar(originalReceiveNs)&&originalReceiveNs>0 ...
                    &&isa(processingNs,'uint64')&&isscalar(processingNs)&&processingNs>=originalReceiveNs ...
                    &&originalReceiveNs>=obj.LastReceiveNs&&processingNs>=obj.LastProcessingNs, ...
                    'gpenmpcNative:LocalGpClock');
                assert(isequal(origin.link_lifecycle_generation,e.link_lifecycle_generation) ...
                    &&strcmpi(origin.execution_session_sha256,e.execution_session_sha256),'gpenmpcNative:LocalGpOrigin');
                q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(requestBytes);i=q.identity;
                assert(i.uid==e.uid&&i.boot_generation==e.boot_generation ...
                    &&i.system==e.board_system&&i.component==e.board_component,'gpenmpcNative:LocalGpIdentity');
                assert(q.output_generation>obj.LastOutput&&q.source_generation>obj.LastSource ...
                    &&q.source_timestamp_ns>obj.LastSourceNs,'gpenmpcNative:LocalGpReplay','Repeated/reversed query.');
                obj.LastOutput=q.output_generation;obj.LastSource=q.source_generation;obj.LastSourceNs=q.source_timestamp_ns;
                obj.LastReceiveNs=originalReceiveNs;obj.LastProcessingNs=processingNs;
                obj.PendingRequest=struct('request',q,'original_host_receive_ns',originalReceiveNs, ...
                    'processing_ns',processingNs,'origin',origin);
                obj.LastRaw=struct('request_bytes',requestBytes,'original_host_receive_ns',originalReceiveNs, ...
                    'processing_ns',processingNs,'origin',origin,'result18',[],'reply_bytes',[]);
                predictor=obj.MexPredictor;
                if strcmp(obj.Backend,'MATLAB_ORIGINAL_THREAD'),predictor=obj.Assets.gp_model;end
                obj.Future=parfeval(obj.AsyncPool,@gpenmpcNative.RflyLocalGpService.computeAsync,1, ...
                    predictor,q,e.board_system,e.board_component);
                obj.Busy=true;
            catch ex
                obj.Failed=true;obj.Closed=true;obj.Failure=string(ex.identifier);rethrow(ex)
            end
        end
        function yes=isPending(obj),yes=~isempty(obj.Future);end
        function r=poll(obj)
            r=[];
            assert(~obj.Closed&&~obj.Failed,'gpenmpcNative:LocalGpClosed');
            if isempty(obj.Future)||~strcmp(obj.Future.State,'finished'),return,end
            try
                a=fetchOutputs(obj.Future);obj.Future=[];
                q=obj.PendingRequest.request;e=obj.Expected;
                assert(isequal(a.request_bytes,q.original_bytes),'gpenmpcNative:LocalGpBackendReply');
                obj.Calls=obj.Calls+uint64(a.actual_gp_calls);
                obj.LastRaw.result18=a.result18;obj.LastRaw.reply_bytes=a.reply_bytes;
                if ~isempty(a.error),throw(a.error);end
                if strcmp(obj.Backend,'MATLAB_ORIGINAL_THREAD')
                    % Numeric-only worker: original wire/source checks remain
                    % on the sole IO owner. No worker socket or timestamp.
                    assert(isempty(a.reply_bytes)&&isempty(a.tunnel_payloads), ...
                        'gpenmpcNative:LocalGpBackendReply','The worker result must contain numerical output only.');
                    a.reply_bytes=gpenmpcNative.RflyLocalGpCodec.encodeReply(q,a.result18);
                    a.tunnel_payloads=gpenmpcNative.RflyLocalGpCodec.replyFragments(a.reply_bytes,e.board_system,e.board_component);
                    obj.LastRaw.reply_bytes=a.reply_bytes;
                end
                r=struct('reply_bytes',a.reply_bytes,'tunnel_payloads',{a.tunnel_payloads}, ...
                    'source_system',e.host_system,'source_component',e.host_component, ...
                    'request',q,'result18',a.result18,'actual_gp_call',obj.Calls, ...
                    'original_host_receive_ns',obj.PendingRequest.original_host_receive_ns, ...
                    'processing_ns',obj.PendingRequest.processing_ns,'packets_sent',0,'control_publications',0);
                obj.Completed=obj.Completed+uint64(1);obj.PendingRequest=[];obj.Busy=false;
            catch ex
                obj.Failed=true;obj.Closed=true;obj.Failure=string(ex.identifier);rethrow(ex)
            end
        end
        function close(obj)
            if ~obj.Closed&&~isempty(obj.ContinuousStop)
                stopped=obj.ContinuousStop();s=stopped.continuous_gp;
                assert(~s.active&&s.queries>=obj.Calls&&s.completed>=obj.Completed,'gpenmpcNative:LocalGpStop');
                obj.Calls=s.queries;obj.Completed=s.completed;
                if isempty(obj.LastRaw),obj.LastRaw=struct();end
                obj.LastRaw.continuous_stop=s;
            end
            % Do not wait for the worker during LAND or environment service.
            if ~isempty(obj.Future)
                if strcmp(obj.Future.State,'finished')&&isempty(obj.Future.Error)
                    a=fetchOutputs(obj.Future);obj.Calls=obj.Calls+uint64(a.actual_gp_calls);
                    obj.LastRaw.result18=a.result18;obj.LastRaw.reply_bytes=a.reply_bytes;
                    obj.LastRaw.completed_after_suspension=true;
                    if isempty(a.error),obj.Completed=obj.Completed+uint64(1);end
                else
                    cancel(obj.Future);obj.LastRaw.inference_outcome_unknown_at_cancel=true;
                end
                obj.Future=[];
            end
            obj.Closed=true;obj.Busy=false;
        end
    end
    methods (Access=private)
        function unlock(obj),obj.Busy=false;end
    end
    methods (Static)
        function r=computeAsync(predictor,q,system,component)
            r=struct('request_bytes',q.original_bytes,'result18',[],'reply_bytes',[], ...
                'tunnel_payloads',{{}},'actual_gp_calls',0,'error',[]);
            try
                if isstruct(predictor)
                    % Use MATLAB prediction on thread workers because legacy MEX is unsupported.
                    % Keep codec and IO work on the caller.
                    r.actual_gp_calls=1;
                    r.result18=gpenmpcNative.canonicalSparseGpFixedInput(predictor,q.request19(2:18).');
                    assert(isa(r.result18,'double')&&isequal(size(r.result18),[1 18]) ...
                        &&all(isfinite(r.result18)),'gpenmpcNative:LocalGpBackendReply');
                    return
                end
                % The very same validated MEX/model and all 18 result fields.
                r.actual_gp_calls=1;[reply,y]=predictor(q.original_bytes);
                r.result18=y;r.reply_bytes=reply;
                decoded=gpenmpcNative.RflyLocalGpCodec.decodeReply(reply);
                assert(isa(y,'double')&&isequal(size(y),[1 18])&&all(isfinite(y)) ...
                    &&isequal(decoded.result18,y(:))&&isequal(decoded.identity,q.identity) ...
                    &&decoded.source_timestamp_ns==q.source_timestamp_ns ...
                    &&decoded.source_generation==q.source_generation&&decoded.output_generation==q.output_generation ...
                    &&isequal(decoded.original_request_sha256,q.original_request_sha256) ...
                    &&isequal(decoded.gp_model_sha256,q.gp_model_sha256),'gpenmpcNative:LocalGpBackendReply');
                r.tunnel_payloads=gpenmpcNative.RflyLocalGpCodec.replyFragments(reply,system,component);
            catch ex,r.error=ex;end
        end
        function warmAsync(predictor)
            % Load code only; deliberately rejected arity makes zero queries.
            loaded=false;
            try,predictor();catch ex,loaded=strcmp(ex.identifier,'gpenmpcNative:CanonicalGpWireMexArity');end
            assert(loaded,'gpenmpcNative:LocalGpBackendLoad');
        end
        function prepared=prepareBackend(verifiedAssets,binding)
            b=verifiedAssets.binding;[c,m]=gpenmpcNative.RflyLocalGpCodec.identities();
            assert(strcmpi(b.passport_sha256,'AA2A2C8F2282A4B41D06FB4AF042FF157B5322AA7040FDCA92123935615B00AE') ...
                &&strcmpi(b.effective_configuration_payload_sha256,hex(c)) ...
                &&strcmpi(b.gp_model_sha256,hex(m))&&b.verified_source_entries==160 ...
                &&verifiedAssets.enmpc.coordinated_architecture_enabled ...
                &&strcmp(verifiedAssets.enmpc.coordinated_architecture_mode,'A1_COORDINATED_PHYSICAL') ...
                &&verifiedAssets.enmpc.command_continuity_enabled ...
                &&isequal(size(verifiedAssets.gp_model.inducing_standardized),[256 17]));
            [predictor,receipt]=checkedMexBackend(binding);
            inline=isfield(binding,'receive_inline')&&isequal(binding.receive_inline,true);
            % Dispatch MATLAB GP prediction through the single-future interface;
            % legacy MEX is not supported on thread workers.
            pool=[];if ~inline,pool=backgroundPool;end
            root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
            fixture=fullfile(root,'rfly_vendor_integration','full_inner_abi', ...
                'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
            f=fopen(fixture,'rb');assert(f>=0,'gpenmpcNative:LocalGpBackendLoad');
            closeFile=onCleanup(@()fclose(f));fixtureBytes=fread(f,Inf,'*uint8');clear closeFile
            [pairs,fixtureSha]=gpenmpcNative.RflyLocalGpService.bindRetainedRehearsalFixture(fixtureBytes);
            pair=pairs(:,1);
            q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(pair(1:310));t=tic;
            if inline
                rehearsal=gpenmpcNative.RflyLocalGpService.computeAsync(predictor,q,q.identity.system,q.identity.component);
            else
                future=parfeval(pool,@gpenmpcNative.RflyLocalGpService.computeAsync,1, ...
                    verifiedAssets.gp_model,q,q.identity.system,q.identity.component);
                rehearsal=fetchOutputs(future);
            end
            rehearsalSeconds=toc(t);
            expected=gpenmpcNative.RflyLocalGpCodec.decodeReply(pair(311:596));
            assert(isempty(rehearsal.error)&&rehearsal.actual_gp_calls==1 ...
                &&isequal(rehearsal.request_bytes,q.original_bytes) ...
                &&max(abs(rehearsal.result18(:)-expected.result18))<=1e-10 ...
                &&rehearsal.result18(15)==expected.result18(15),'gpenmpcNative:LocalGpBackendReply');
            receipt.runtime_numerical_backend='MATLAB_ORIGINAL_THREAD';
            if inline,receipt.runtime_numerical_backend='CANONICAL_GP_WIRE_MEX_RECEIVE_INLINE';end
            receipt.prepared_gp_rehearsals=1;receipt.prepared_gp_rehearsal_s=rehearsalSeconds;
            receipt.prepared_gp_rehearsal_scope='ORIGINAL_MATLAB_THREAD_RETAINED_FIXED_PAIR_NO_IO_NO_LIVE_AUTHORITY';
            receipt.prepared_gp_rehearsal_fixture_sha256=fixtureSha;
            prepared=struct('schema','GPENMPC_PREPARED_LOCAL_GP_BACKEND_V1', ...
                'async_pool',pool, ...
                'binding',binding,'predictor',predictor,'receipt',receipt, ...
                'passport_sha256',b.passport_sha256, ...
                'configuration_sha256',b.effective_configuration_payload_sha256, ...
                'gp_model_sha256',b.gp_model_sha256,'verified_source_entries',b.verified_source_entries, ...
                'prepared_before_getter_producer',true,'authority_granted',false, ...
                'construction_gp_calls',1);
        end
        function [pairs,fixtureSha]=bindRetainedRehearsalFixture(bytes)
            % Bind the verified preparation fixture to the current asset identities.
            fixtureSha='87498E856EC26414E48E719D25526ABA7763DC63FF4EC2FE7459A6847FC3395E';
            assert(isa(bytes,'uint8')&&isvector(bytes)&&numel(bytes)==596*59 ...
                &&strcmpi(hex(rehearsalDigest(bytes)),fixtureSha), ...
                'gpenmpcNative:LocalGpRehearsalFixture','Exact retained rehearsal fixture required.');
            pairs=reshape(bytes,596,59);
            oldConfig=uint8(sscanf('0F7E32676B032421F7D41C687A49495F42DAF6780FC0EC64D98849758BA165AB','%2x'));
            oldModel=uint8(sscanf('5FAF74574B130FDE4B5FA543D8A21554A87093372193D5643D78A23274D4FC57','%2x'));
            [config,model]=gpenmpcNative.RflyLocalGpCodec.identities();
            for k=1:size(pairs,2)
                request=pairs(1:310,k);reply=pairs(311:596,k);
                assert(isequal(request(1:4),uint8('RGP1').')&&isequal(reply(1:4),uint8('RGR1').') ...
                    &&isequal(request(63:94),oldConfig)&&isequal(request(95:126),oldModel) ...
                    &&isequal(reply(79:110),oldModel)&&isequal(reply(5:46),request(5:46)) ...
                    &&isequal(request(279:310),rehearsalDigest(request(1:278))) ...
                    &&isequal(reply(255:286),rehearsalDigest(reply(1:254))) ...
                    &&isequal(reply(47:78),rehearsalDigest(request)), ...
                    'gpenmpcNative:LocalGpRehearsalIdentity','Known retained identities and checksums required.');
                request(63:94)=config;request(95:126)=model;
                request(279:310)=rehearsalDigest(request(1:278));
                reply(47:78)=rehearsalDigest(request);reply(79:110)=model;
                reply(255:286)=rehearsalDigest(reply(1:254));
                assert(isequal(request(127:278),pairs(127:278,k)) ...
                    &&isequal(reply(111:254),pairs(421:564,k)), ...
                    'gpenmpcNative:LocalGpRehearsalNumerics','Retained numerical payloads must not change.');
                pairs(:,k)=[request;reply];
            end
        end
    end
end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function value=rehearsalDigest(bytes)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes(:),'int8'));
value=reshape(typecast(md.digest(),'uint8'),[],1);
end
function [predictor,receipt]=checkedMexBackend(binding)
approved='21017CD36C857466AE538EAE716C868173D2C54DF4D2B5CC458E3C6681CEBE6B';
fields={'kind';'path';'binary_sha256'};
if isstruct(binding)&&isscalar(binding)&&isfield(binding,'receive_inline')
    assert(isequal(binding.receive_inline,true),'gpenmpcNative:LocalGpBackendBinding');
    fields{end+1}='receive_inline';
    approved='A20AB9DAA20749EFB7E549DDBD26DE29AC7F34C2FA5C4A73FB5B4A348D1281D6';
end
assert(isstruct(binding)&&isscalar(binding) ...
    &&isequal(sort(fieldnames(binding)),sort(fields)), ...
    'gpenmpcNative:LocalGpBackendBinding','Explicit kind, absolute path and binary_sha256 are required.');
kind=string(binding.kind);p=string(binding.path);digest=string(binding.binary_sha256);
assert(isscalar(kind)&&kind=="CANONICAL_GP_WIRE_MEX"&&isscalar(p)&&isscalar(digest) ...
    &&~isempty(regexp(char(p),'^[A-Za-z]:[\\/]','once')) ...
    &&strcmpi(digest,approved), ...
    'gpenmpcNative:LocalGpBackendBinding','The GP wire MEX identity must match the configured binding.');
assert(isfile(p),'gpenmpcNative:LocalGpBackendMissing','Explicit MEX binary is absent.');
[~,name,extension]=fileparts(p);
assert(name=="canonical_gp_wire_mex"&&strcmpi(extension,'.mexw64'), ...
    'gpenmpcNative:LocalGpBackendBinding','Expected canonical_gp_wire_mex.mexw64.');
file=java.io.File(char(p));p=string(file.getCanonicalPath());
resolved=string(which('canonical_gp_wire_mex'));
assert(strlength(resolved)>0,'gpenmpcNative:LocalGpBackendMissing','The expected MEX must already be on the caller-owned path.');
resolvedFile=java.io.File(char(resolved));resolved=string(resolvedFile.getCanonicalPath());
assert(strcmpi(p,resolved),'gpenmpcNative:LocalGpBackendResolution','A different GP wire function resolves on the current path.');
assert(strcmpi(binarySha(p),digest),'gpenmpcNative:LocalGpBackendHash','Exact validated MEX binary checksum required.');
predictor=@canonical_gp_wire_mex;
% Load/identify the exact MEX during construction, before a pending query.
% Its validated C++ arity check precedes the GP API and every numerical
% operation; this deliberately rejected call makes zero GP predictions.
loaded=false;
try,predictor();catch ex,loaded=strcmp(ex.identifier,'gpenmpcNative:CanonicalGpWireMexArity');end
assert(loaded,'gpenmpcNative:LocalGpBackendLoad','Bound MEX did not return its original pre-numerical arity rejection.');
receipt=struct('kind','CANONICAL_GP_WIRE_MEX','path',p,'binary_sha256',upper(digest), ...
    'resolved_path',resolved,'binary_hash_checks',1,'hash_scope','CONSTRUCTION_ONLY', ...
    'construction_load_probe',true,'construction_gp_calls',0, ...
    'runtime_file_checks',0,'fallback_allowed',false,'original_gp256_algorithm_changed',false);
end
function [predictor,receipt]=reusePreparedMexBackend(prepared,binding,verifiedAssets)
b=verifiedAssets.binding;
assert(isstruct(prepared)&&isscalar(prepared) ...
    &&isequal(sort(fieldnames(prepared)),sort({'schema';'binding';'predictor';'receipt';'passport_sha256'; ...
        'configuration_sha256';'gp_model_sha256';'verified_source_entries'; ...
        'prepared_before_getter_producer';'authority_granted';'construction_gp_calls';'async_pool'})) ...
    &&strcmp(prepared.schema,'GPENMPC_PREPARED_LOCAL_GP_BACKEND_V1') ...
    &&isequaln(prepared.binding,binding)&&isa(prepared.predictor,'function_handle') ...
    &&strcmp(func2str(prepared.predictor),'canonical_gp_wire_mex') ...
    &&strcmpi(prepared.passport_sha256,b.passport_sha256) ...
    &&strcmpi(prepared.configuration_sha256,b.effective_configuration_payload_sha256) ...
    &&strcmpi(prepared.gp_model_sha256,b.gp_model_sha256) ...
    &&prepared.verified_source_entries==b.verified_source_entries ...
    &&prepared.prepared_before_getter_producer&&~prepared.authority_granted ...
    &&prepared.construction_gp_calls==1, ...
    'gpenmpcNative:LocalGpPrepared','Exact pre-producer GP backend token required.');
r=prepared.receipt;
assert(isstruct(r)&&isscalar(r)&&strcmpi(r.binary_sha256,binding.binary_sha256) ...
    &&strcmpi(r.path,binding.path)&&r.binary_hash_checks==1 ...
    &&r.construction_load_probe&&r.construction_gp_calls==0 ...
    &&r.runtime_file_checks==0&&~r.fallback_allowed ...
    &&~r.original_gp256_algorithm_changed, ...
    'gpenmpcNative:LocalGpPrepared','Prepared GP identity or no-call receipt changed.');
predictor=prepared.predictor;receipt=r;
receipt.prepared_before_getter_producer=true;
receipt.service_constructor_file_checks=0;
end
function h=binarySha(p)
f=fopen(p,'rb');assert(f>=0,'gpenmpcNative:LocalGpBackendMissing','Cannot read bound MEX.');g=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(f),b=fread(f,1048576,'*uint8');md.update(typecast(b,'int8'));end
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
