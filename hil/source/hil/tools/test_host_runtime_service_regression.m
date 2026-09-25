function report=test_host_runtime_service_regression(outputRoot,baselineRoot)
% Exercise receive ordering and service bookkeeping with in-memory IO doubles.
if nargin<1,outputRoot='';end
if nargin<2,baselineRoot='';end
build=fileparts(fileparts(mfilename('fullpath')));
runner=fileread(fullfile(build,'tools','run_m600_board_local_short_hil.m'));
service=fileread(fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m'));
launcher=fileread(fullfile(build,'tools','start_gpenmpc_usb_manual.m'));
checks=struct('name',{},'pass',{});details=struct();
names={'arrived_during_poll','interrupted_estimate','no_new_packet_fresh','interrupted_truth', ...
    'receiver_session_fault','new_disarmed_state','new_invalid_clock','new_speed_violation', ...
    'new_estimator_gap','new_stale_heartbeat'};
expected={'','gpenmpcShort:StateAge','','gpenmpcShort:StateAge', ...
    'gpenmpcShort:OriginalIoFatal','gpenmpcShort:Operational','gpenmpcShort:Operational', ...
    'gpenmpcShort:TruthSpeed','gpenmpcShort:EstimatorGap','gpenmpcShort:Operational'};
for k=1:numel(names)
    r=pumpCase(runner,k);details.(names{k})=r;
    check(names{k},strcmp(r.error,expected{k}));
    check([names{k} '_same_cached_owner'],r.snapshot_calls==2&&~any(r.receive_flags)&&r.owner==42);
    check([names{k} '_original_timestamp'],r.estimate_rx_s==r.cached_estimate_rx_s);
end
for fail=[false true]
    r=codecCase(service,fail);
    check(sprintf('component_codec_count_failure_%d',fail),r.calls==7+double(~fail));
    check(sprintf('component_codec_result_failure_%d',fail), ...
        (~fail&&isempty(r.error)&&isequal(r.bytes,uint8([1;2;3]))) ...
        ||(fail&&strcmp(r.error,'test:CodecFailure')));
end
for k=1:8
    r=gpCase(service,k);details.(sprintf('sync_gp_%d',k))=r;
    wantSend=ismember(k,[1 2]);
    check(sprintf('sync_gp_%d',k),r.sent==wantSend&&r.counted==wantSend ...
        &&r.predictions==double(k==1)&&r.async_calls==0);
end
for live=[false true]
    for preparation=[false true]
        preparationOnly=~live||preparation;
        check(sprintf('launcher_live_%d_preparation_%d',live,preparation),preparationOnly||live);
    end
end
check('launcher_post_preparation_branch_is_direct', ...
    ~contains(between(launcher,"assignin('base','gpenmpc_manual_launch_error',ex);", ...
    'function result=resumeManualReset'),'if liveInvoked'));
check('historical_commit_uses_its_sent_input',contains(service,'outerInputs=p.inputs;') ...
    &&~contains(between(service,['                if ~runtimeStateOnly' newline '                for k=1:4'], ...
    '                % Dequeue completed snapshots'),'if runtimeStateOnly'));
if ~isempty(baselineRoot)
    oldRunner=fileread(fullfile(baselineRoot,'tools','run_m600_board_local_short_hil.m'));
    oldService=fileread(fullfile(baselineRoot,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m'));
    r=pumpCase(oldRunner,1);details.baseline_arrival=r;
    check('baseline_reproduces_stale_snapshot_failure',strcmp(r.error,'gpenmpcShort:StateAge')&&r.snapshot_calls==1);
    r=codecCase(oldService,false);details.baseline_codec=r;
    check('baseline_reproduces_missing_codec_count',r.calls==7&&isempty(r.error));
    for k=1:8
        check(sprintf('sync_gp_equivalent_%d',k),isequaln(gpCase(oldService,k),gpCase(service,k)));
    end
    check('operational_guards_unchanged',strcmp(nestedBody(oldRunner,'assertOperational','yes=preparedWithWindow'), ...
        nestedBody(runner,'assertOperational','yes=preparedWithWindow')));
    check('state_fresh_guards_unchanged',strcmp(nestedBody(oldRunner,'yes=stateFresh','yes=offboardDisarmed'), ...
        nestedBody(runner,'yes=stateFresh','yes=offboardDisarmed')));
end
report=struct('passed',all([checks.pass]),'checks',checks,'details',details);
if ~isempty(outputRoot)
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    f=fopen(fullfile(outputRoot,'HOST_RUNTIME_REGRESSION.json'),'w');assert(f>=0);
    cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
    fprintf(f,'%s',jsonencode(report,PrettyPrint=true));
end
fprintf('Host runtime regression: %d checks passed.\n',numel(checks));
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass));
        assert(pass,'test:HostRuntimeRegression','%s',name);
    end
end

function r=pumpCase(source,k)
body=nestedBody(source,'pump','serviceRuntimeIo');
guards=nestedBody(source,'assertOperational','yes=preparedWithWindow');
state=containers.Map('KeyType','char','ValueType','any');
sample=struct('rx_s',2.99,'position_ned_m',[0 0 0],'velocity_ned_mps',[0 0 0]);
cached=struct('now_s',3,'fatal','','first_fatal',struct(),'armed',1,'landed_state',2, ...
    'main_mode',6,'clock_valid',true,'heartbeat_rx_s',2.99,'extended_rx_s',2.99, ...
    'estimate',sample,'truth',sample,'owner',42);
if ismember(k,[1 2]),cached.estimate.rx_s=2.70;end
if k==4,cached.truth.rx_s=2.70;end
state('cache')=cached;state('snapshot_calls')=0;state('receive_flags')=[];state('now')=3;
io=struct('snapshot',@(varargin)snapshotDouble(state,varargin{:}),'now',@()state('now'));
service=struct('Closed',false,'poll',@(varargin)pollDouble(state,k), ...
    'Phase',struct('CommitCount',0));
getter=struct('status',@()struct('source_matching_started',false));
getterMex=@(varargin)struct('ring_name','ring','status_name','status','peer_nonce',1, ...
    'consumer_pid',2,'records',zeros(1,0),'failed',false);
opened=struct('ring_name','ring','status_name','status','peer_nonce',1,'consumer_pid',2);
c=struct('runtime_state_only',true,'component_initialization',true);
cfg=struct('state_max_age_s',.25,'heartbeat_max_age_s',1,'landed_max_age_s',1, ...
    'abort_truth_speed_mps',10,'abort_estimator_gap_m',5);
sessionRaw=struct('start_observation',struct('task_created',true));
finalized=false;phase='FLIGHT';mode='COMPONENT';counts=struct('gp_replies',0,'commits',0);
coordinateOffset=zeros(3,1);latest=[];firstCommit=Inf;lastCommit=Inf; %#ok<NASGU>
checkManualReset=@()[];heartbeat=@(varargin)[];refreshEnvironment=@()[]; %#ok<NASGU>
retain=@(varargin)[];note=@(varargin)[];methodModeAfterReceive=@()'COMPONENT';serviceRuntimeIo=@()[]; %#ok<NASGU>
errorId='';
try
    eval(body);
    stateFresh=@()freshDouble(source,latest,io,cfg); %#ok<NASGU>
    eval(guards);
catch ex,errorId=ex.identifier;end
cache=state('cache');
r=struct('error',errorId,'snapshot_calls',state('snapshot_calls'), ...
    'receive_flags',state('receive_flags'),'owner',latest.owner, ...
    'estimate_rx_s',latest.estimate.rx_s,'cached_estimate_rx_s',cache.estimate.rx_s);
end

function s=snapshotDouble(state,~,receive)
state('snapshot_calls')=state('snapshot_calls')+1;
state('receive_flags')=[state('receive_flags') receive];
s=state('cache');
end

function event=pollDouble(state,k)
s=state('cache');state('now')=3.02;
if k==1,s.estimate.rx_s=3.01;end
if k==5,s.fatal='SESSION_MISMATCH';end
if k==6,s.armed=0;end
if k==7,s.clock_valid=false;end
if k==8,s.truth.velocity_ned_mps=[11 0 0];end
if k==9,s.estimate.position_ned_m=[6 0 0];end
if k==10,s.heartbeat_rx_s=1;end
state('cache')=s;
event=struct('mode','COMPONENT','gp',{{}},'committed',{{}});
end

function yes=freshDouble(source,latest,io,cfg) %#ok<INUSD>
body=nestedBody(source,'yes=stateFresh','yes=offboardDisarmed');
% The following comments precede the next function declaration.
body=regexprep(body,'(?s)\s+end\s+% Named nested callbacks.*$','');
eval(body);
end

function r=codecCase(source,fail)
source=between(source,'                        if strcmp(mode,''COMPONENT'')', ...
    '                        elseif strcmp(mode,''PREPARE'')');
body=between(source,'                                if isempty(obj.InputCodecMex)', ...
    '                                next=obj.now();event.work_timing_ns.input_encode');
obj=struct('InputCodecMex',@()[],'InputCodecAdapter',@(varargin)codecDouble(fail), ...
    'InputCodecCalls',uint64(7),'Source',struct('message',uint8(4),'original_host_receive_ns',uint64(1)), ...
    'Registered',struct());
binding=struct();candidate=struct();inputs=[];errorId=''; %#ok<NASGU>
try,eval(body);catch ex,errorId=ex.identifier;end
r=struct('calls',double(obj.InputCodecCalls),'bytes',inputs,'error',errorId);
end

function bytes=codecDouble(fail)
assert(~fail,'test:CodecFailure','Encoding failed.');bytes=uint8([1;2;3]);
end

function r=gpCase(source,k)
body=between(source,'                % Service synchronous GP work only after', ...
    '                % Send component input using');
state=containers.Map('KeyType','char','ValueType','any');
state('predictions')=0;state('sent')=0;state('async_calls')=0;
q=struct('message',uint8(1),'original_host_receive_ns',uint64(1),'origin',struct(), ...
    'inline_gp',struct('computed',struct('reply_bytes',uint8(2))));
if k==3,q=[];end
obj=struct('ComponentInitialization',false,'now',@()uint64(10),'PendingGp',[], ...
    'Gp',struct('AsyncEnabled',k==5,'ReceiveInlineEnabled',k==2,'isPending',@()k==4, ...
    'poll',@()countDouble(state,'async_calls',[]),'begin',@(varargin)countDouble(state,'async_calls',[]), ...
    'process',@(varargin)countDouble(state,'predictions',struct('reply_bytes',uint8(2)))), ...
    'Io',struct('takeCanonical',@(varargin)q,'sendCanonicalLocalGp',@(varargin)countDouble(state,'sent',struct('messages_send_returned',3))), ...
    'Counts',struct('gp_replies',0),'witnessStream',@(varargin)[]);
event=struct('gp',{{}},'committed',{{}},'input_send',struct('messages_send_returned',6),'work_timing_ns',struct('gp',uint64(0)));
if k==6,event.input_send.messages_send_returned=5;end
if k==7,event.outer_clock=struct('outer_submitted',true);end
if k==8,event.committed={struct()};end
runtimeStateOnly=true;mode='FLIGHT';serviceHeartbeat=@()[]; %#ok<NASGU>
eval(body);
r=struct('sent',state('sent'),'counted',obj.Counts.gp_replies, ...
    'predictions',state('predictions'),'async_calls',state('async_calls'));
end

function value=countDouble(state,key,value)
state(key)=state(key)+1;
end

function body=nestedBody(source,name,nextName)
body=between(source,['    function ' name '()'],['    function ' nextName '()']);
body=extractAfter(string(body),newline);body=char(body);
body=regexprep(body,'\s+end\s*$','');
end

function text=between(source,first,last)
source=strrep(source,sprintf('\r\n'),newline);
a=strfind(source,char(first));assert(numel(a)==1,'test:SourceMarker','%s',first);
b=strfind(source(a:end),char(last));assert(~isempty(b),'test:SourceMarker','%s',last);
text=source(a:a+b(1)-2);
end
