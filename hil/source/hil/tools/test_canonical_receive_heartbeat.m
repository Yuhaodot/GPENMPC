function result=test_canonical_receive_heartbeat()
% Test receive and heartbeat code with in-memory transports.
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'));
ioPath=fullfile(build,'m600_coptersim','matlab_validation','+m600check','makeM600CopterSimIo.m');
servicePath=fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m');
source=fileread(ioPath);service=fileread(servicePath);
pollBody=bodyBetween(source,'    function r=pollCanonical(', '    function r=sendCanonicalPackets(');
drainBody=bodyBetween(source,'    function drainRawMavlink(', '    function dispatchRawRecord(');
% Supply caller arity to test the default branches.
pollBody=strrep(pollBody,'nargin','fixtureNargin');
drainBody=strrep(drainBody,'nargin','fixtureNargin');
% Nested production functions are injected handles in this fixture scope.
drainBody=strrep(drainBody,'@unlockRawDrain','unlockRawDrain');
drainBody=strrep(drainBody,'@dispatchRawRecord','dispatchRawRecord');
for scenario=1:6
    runIoCase(pollBody,drainBody,scenario);
end
a=strfind(service,'                receiveStart=obj.now();');
b=strfind(service,'                if ~isempty(modeReader),mode=char(modeReader());end');
assert(isscalar(a)&&isscalar(b));receiveBody=service(a:b-1);
for repeat=[false,true],runServiceCase(receiveBody,repeat);end
for path={ioPath,servicePath,mfilename('fullpath')}
    issues=checkcode(path{1},'-id');assert(~any(strcmp({issues.id},'PARSE')));
end
result=struct('passed',true,'cases',8,'hardware_actions',0, ...
    'checked','legacy arity, receive-only callback, empty receive, inline and deferred dispatch, native owner released, both service receive passes');
disp(result);
end

function body=bodyBetween(source,first,next)
a=strfind(source,first);b=strfind(source,next);assert(isscalar(a)&&isscalar(b)&&a<b);
body=source(a:b-1);lineEnd=find(body==newline,1);body=body(lineEnd+1:end);
body=regexprep(body,'\s*end\s*$','');
end

function runIoCase(pollBody,drainBody,scenario)
state=newState();state('records')=2;state('inline')=true;
fixtureNargin=2;receiveNow=true;serviceHeartbeat=@()heartbeat(state);
if scenario==1,fixtureNargin=0;end
if scenario==2,fixtureNargin=1;receiveNow=false;end
if scenario==3,receiveNow=false;end
if scenario==5,state('records')=0;end
if scenario==6,state('inline')=false;end
canonicalExchangeEnabled=true;canonicalExchangeBindAttempted=true;
canonicalExchangeFailure=[];canonicalExchangeUnboundReceived=uint64(0);
rawTransportEnabled=true;transportKind='IN_MEMORY_FIXTURE';canonicalCompletedCapacity=8;
canonicalCompleted={};canonicalCompletedEnqueued=0;canonicalCompletedTaken=0;
canonicalLocalMode=true;canonicalChannels={'snapshot'};rawSnapshotReceivePending=false;
canonicalSessionHistory={};canonicalSessionHistoryCapacity=1;
canonicalExchange=struct('poll',@(unused)statusCall(state));
drainRawMavlink=@(callback)runDrain(drainBody,callback,state);
eval(pollBody);
events=state('events');assert(strcmp(events{end},'status'));
assert(state('native_busy')==false&&state('usb_reads')==0&&isempty(state('faults')));
if ismember(scenario,[2,3])
    assert(isequal(events,{'status'})&&state('heartbeats')==0);
elseif scenario==1
    assert(state('receives')==1&&state('heartbeats')==0&&state('dispatches')==2);
else
    assert(state('receives')==1&&state('heartbeats')==state('records')+1);
    assert(state('dispatches')==state('records'));
    unlock=find(strcmp(events,'native_release'));firstHb=find(strcmp(events,'heartbeat'),1);
    assert(isscalar(unlock)&&unlock<firstHb);
end
assert(r.bound&&isempty(r.failure));
end

function runDrain(body,serviceHeartbeat,state)
fixtureNargin=1;rawTransportEnabled=true;closed=false;rawDraining=false;
canonicalLocalMode=state('inline');cfg.local_short.runtime_state_only=true;
fatal='';nonModelFatal='';canonicalExchangeFailure='';canonicalExchangeEnabled=true;
rawTransport=struct('poll',@(varargin)receiveCall(state,varargin{:}), ...
    'status',@()struct());
unlockRawDrain=@()record(state,'raw_release');
dispatchRawRecord=@(row)dispatch(state,row);
retainForwardingFragments=@(unused)[];
latchFatal=@(reason,unused)captureFault(state,reason);
failCanonicalExchange=@(problem)captureFault(state,problem.identifier);
eval(body);
end

function [records,fragments,pending]=receiveCall(state,varargin)
assert(~state('native_busy'));state('native_busy')=true;
cleanup=onCleanup(@()releaseNative(state)); %#ok<NASGU>
state('receives')=state('receives')+1;record(state,'native_receive');
records=repmat(struct('sequence',uint64(1)),1,state('records'));fragments=[];pending=false;
for k=1:numel(records)
    records(k).sequence=uint64(k);
    if ~isempty(varargin),varargin{1}(records(k));end
end
end
function releaseNative(state),state('native_busy')=false;record(state,'native_release');end
function dispatch(state,~),state('dispatches')=state('dispatches')+1;record(state,'dispatch');end
function heartbeat(state)
assert(~state('native_busy'),'test:ReceiveReentry','Heartbeat entered the native receive owner.');
state('heartbeats')=state('heartbeats')+1;record(state,'heartbeat');
end
function value=statusCall(state),record(state,'status');value=struct('passed',true);end
function captureFault(state,reason),v=state('faults');v{end+1}=reason;state('faults')=v;end
function record(state,name),v=state('events');v{end+1}=name;state('events')=v;end
function state=newState()
state=containers.Map('KeyType','char','ValueType','any');
for key={'receives','heartbeats','dispatches','usb_reads'},state(key{1})=0;end
state('native_busy')=false;state('events')={};state('faults')={};
end

function runServiceCase(body,repeat)
state=newState();state('repeat')=repeat;
serviceHeartbeat=@()heartbeat(state);runtimeStateOnly=true;
obj=struct('now',@()uint64(1000),'Io',struct('pollCanonical',@(varargin)methodReceive(state,varargin{:})), ...
    'Gp',struct('AsyncEnabled',false,'ReceiveInlineEnabled',false));
event=struct('work_timing_ns',struct('receive',uint64(0)));
eval(body);
assert(state('receives')==1+double(repeat));
assert(state('heartbeats')==1+2*double(repeat));
end
function value=methodReceive(state,receiveNow,callback)
assert(receiveNow&&isa(callback,'function_handle'));
state('receives')=state('receives')+1;callback();
pending=state('repeat')&&state('receives')==1;
value=struct('failure',[],'channels',{{'gp_request','snapshot'}}, ...
    'completed_queue_counts',[0,double(~pending)],'snapshot_receive_pending',pending);
end
