function test_rc_demo_runtime(scope,historicalFixture)
% Test response appending and retained reference-window handling.
% historicalFixture optionally selects a recorded MAT fixture.
if nargin<1,scope="OFFLINE";end
if nargin<2,historicalFixture="";end
scope=string(scope);historicalFixture=string(historicalFixture);
assert(isscalar(scope)&&ismember(scope,["OFFLINE","INPUT_SEND_ONLY"]));
assert(isscalar(historicalFixture));
build=fileparts(fileparts(mfilename('fullpath')));
if scope=="INPUT_SEND_ONLY"
    checkInputSendBatch(build);return
end
ioPath=fullfile(build,'m600_coptersim','matlab_validation','+m600check','makeM600CopterSimIo.m');
for p={ioPath,fullfile(build,'tools','execute_m600_board_local_short.m'), ...
        fullfile(build,'tools','m600_local_application_handoff.m'),fullfile(build,'tools','start_gpenmpc_usb_manual.m')}
    issues=checkcode(p{1},'-id');assert(~any(strcmp({issues.id},'PARSE')),p{1});
end

function checkInputSendBatch(build)
% Slow history must not split the six sends.
addpath(fullfile(build,'host_runtime'));
path=fullfile(build,'m600_coptersim','matlab_validation','+m600check','makeM600CopterSimIo.m');
issues=checkcode(path,'-id');assert(~any(strcmp({issues.id},'PARSE')));
source=fileread(path);
start=strfind(source,'            if taskInput&&rawTransportEnabled');
finish=strfind(source,'            % Prepare only the existing bounded record slots');
assert(isscalar(start)&&isscalar(finish));
body=source(start+numel('            if taskInput&&rawTransportEnabled'):finish-1);
body=regexprep(body,'\s*else\s*$','');
for scenario=1:3
    state=containers.Map('KeyType','char','ValueType','any');
    state('calls')=0;state('rows')={};state('scenario')=scenario;
    state('last')=struct('bytes_complete',true);
    count=6;messages=num2cell(1:6);submitted=zeros(6,1,'uint64');returned=submitted;
    actualSendAttempts=0;canonicalExchangeSent=uint64(0);
    closed=false;fatal=[];canonicalExchangeFailure=[];
    rawTransport=struct('sendMessage',@(message)fakeInputSend(state,message), ...
        'status',@()struct('last_send',state('last')));
    nowS=@()double(gpenmpcNative.rflyOriginalHostMonotonicNs())/1e9;
    appendRaw=@(which,row)captureInputRow(state,which,row);
    expiryIdentifier='m600check:CanonicalLocalInputExpired';expiryMessage='Original input expired.';
    now=gpenmpcNative.rflyOriginalHostMonotonicNs();
    binding=struct('original_host_receive_ns',now,'valid_until_host_ns',now+uint64(50000000));
    if scenario==3,binding.original_host_receive_ns=now-uint64(100000000);binding.valid_until_host_ns=now-uint64(50000000);end
    rows={};transportReceipts={};j=0;sendProblem=[];sentSeconds=[];sendNow=uint64(0);
    sendError=[];transportState=[];retainedError=[];row=[]; %#ok<NASGU>
    id='';detail='';try,eval(body);catch ex,id=ex.identifier;detail=ex.message;end
    rows=state('rows');assert(numel(rows)==6,'scenario %d: %s %s',scenario,id,detail);
    if scenario==1
        assert(isempty(id)&&actualSendAttempts==6&&all(returned<=binding.valid_until_host_ns));
        assert(gpenmpcNative.rflyOriginalHostMonotonicNs()>binding.valid_until_host_ns);
        assert(all(cellfun(@(x)x.send_attempted&&x.send_returned,rows)));
    elseif scenario==2
        assert(strcmp(id,'fixture:SendFailed')&&actualSendAttempts==3&&canonicalExchangeSent==2);
        assert(rows{3}.send_attempted&&~rows{3}.send_returned&&isfield(rows{3},'transport_receipt'));
        assert(~rows{4}.send_attempted&&~rows{6}.send_returned);
    else
        assert(strcmp(id,'m600check:CanonicalLocalInputUnsentExpired')&&actualSendAttempts==0);
        assert(all(cellfun(@(x)~x.send_attempted,rows)));
    end
end
fprintf('PASS INPUT_SEND_ONLY: contiguous sends, original expiry, retained partial failure; hardware=0.\n');
end
function receipt=fakeInputSend(state,message)
state('calls')=state('calls')+1;assert(message==state('calls'));
receipt=struct('bytes_complete',true,'attempted',true,'ok',true);
state('last')=receipt;
if state('scenario')==2&&message==3,error('fixture:SendFailed','Injected actual send failure.');end
end
function captureInputRow(state,which,row)
assert(strcmp(which,'TX'));
expected=[6,3,0];assert(state('calls')==expected(state('scenario')));
rows=state('rows');rows{end+1}=row;state('rows')=rows;
if state('scenario')==1,pause(.012);end
end
source=fileread(ioPath);
a=strfind(source,'    function appendRaw(which,value)');b=strfind(source,'    function drainRawMavlink(');
body=source(a+strlength('    function appendRaw(which,value)'):b-1);
body=regexprep(body,'\s*end\s*$',''); % Execute the extracted production body.
rawMav={};rawTruth={};rawTime={};rawTx={};rawEnvironmentTx={};rawRotor={};rawCache={};
rawMavBase=0;canonicalCommittedRawCursor=[];demoHistoryCapacity=4096;demoRecentOnly=true;
cfg.maximum_raw_records=250000;fatal='';
kinds={'MAV','TRUTH','TIME','TX','ENV_TX','ROTOR','CACHE'};
rawRetired=cell2struct(num2cell(zeros(1,7)),kinds,2);rawDropped=rawRetired;
for k=1:20000
    row=struct('sequence',k,'original_host_receive_ns',uint64(k)*uint64(1000000));
    for j=1:7,appendCaptured(kinds{j},row);end
    index=uint64(rawMavBase+numel(rawMav));
    assert(index==k&&isequal(rawMav{double(index)-rawMavBase},row));
    assert(numel(rawMav)<=demoHistoryCapacity&&isempty(fatal));
end
assert(all(structfun(@(x)x>0,rawRetired))&&all(structfun(@(x)x==0,rawDropped)));
assert(rawMav{1}.sequence==rawMavBase+1);
fprintf('Recent-only: 140000 channel appends; max 4096/channel; callback IDs/timestamps retained across recycling.\n');
% Preserve unprocessed in-flight state without substituting another row.
while numel(rawMav)<demoHistoryCapacity,appendCaptured('MAV',row);end
canonicalCommittedRawCursor=rawMavBase+1;before=rawMavBase;
try,appendCaptured('MAV',row);error('test:ExpectedBacklog');catch ex,assert(strcmp(ex.identifier,'m600check:LiveReceiveBacklog'));end
assert(rawMavBase==before);canonicalCommittedRawCursor=[];
% Formal recording still retains its original no-overwrite guard.
demoRecentOnly=false;cfg.maximum_raw_records=16;rawMav={};
for k=1:17,appendCaptured('MAV',row);end
assert(numel(rawMav)==16&&strcmp(fatal,'RAW_MEMORY_RECORD_BOUND_EXCEEDED')&&rawDropped.MAV==1);
fprintf('Pending-state and recording-overflow tests passed.\n');

board=fileread(fullfile(build,'rfly_vendor_integration','px4_runtime','CanonicalLocalSessionEntry.cpp'));
block=regexp(board,'operator_reference_sha\[32\]\s*=\s*\{([^}]+)\}','tokens','once');
assert(~isempty(block));
bs=regexp(block{1},'0x([0-9a-fA-F]{2})','tokens');bs=cellfun(@(x)x{1},bs,'UniformOutput',false);
assert(numel(bs)==32);
distribution=fileparts(fileparts(build));
firmware=jsondecode(fileread(fullfile(distribution,'firmware','fmuv6c','firmware.json')));
assert(strcmpi([bs{:}],firmware.identity.operator_reference_sha256));
fprintf('Current firmware reference identity matches the board source.\n');
if strlength(historicalFixture)>0,checkHistoricalWire(historicalFixture);end
fprintf('Runtime regression tests passed.\n');

    function appendCaptured(which,value)
        n=0;remove=0; %#ok<NASGU> Static workspace names used by the original body.
        eval(body);
    end
    function latchFatal(reason,~),fatal=reason;end
end

function checkHistoricalWire(historicalFixture)
assert(isfile(historicalFixture),'gpenmpcTest:Fixture','Historical fixture does not exist: %s',historicalFixture);
d=load(historicalFixture,'rawIo');
frames={};
for k=1:numel(d.rawIo.raw_transmit_messages)
    row=d.rawIo.raw_transmit_messages{k};
    if isfield(row,'canonical_local_reference_window')&&row.canonical_local_reference_window
        frames{end+1}=row.message.Payload.payload(:); %#ok<AGROW>
    end
end
assert(numel(frames)==16);
wire=[frames{1}(11:128);frames{2}(11:128)];
fixtureSha=uint8(d.rawIo.canonical_local_window.identity.reference_asset_sha256(:));
assert(numel(fixtureSha)==32);
oldSha='1D88751AFB29CA7F04813454F7596CFA96B7DF7C18408BFA6AA2CF7FFE1FDC86';
assert(isequal(wire(119:150),fixtureSha));
oldManifest=wire(1:162);oldManifest(119:150)=uint8(sscanf(oldSha,'%2x'));
assert(find(oldManifest~=wire(1:162),1)==119);
fprintf('Historical reference wire: 16 frames, recorded identity %s.\n',reshape(dec2hex(fixtureSha,2).',1,[]));
end
