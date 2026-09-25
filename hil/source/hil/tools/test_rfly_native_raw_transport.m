function report=test_rfly_native_raw_transport(outputRoot,runName)
% Test RflyUdpRaw with a localhost echo peer and retained MAVLink2 TUNNEL bytes.
arguments
    outputRoot (1,1) string
    runName (1,1) string = "native_raw_transport"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(isfolder(outputRoot)&&~isempty(regexp(char(runName),'^[A-Z0-9_]+$','once')));
out=fullfile(outputRoot,runName);assert(~isfolder(out)&&~isfile(out),'Preserve prior evidence.');
mkdir(out);oldPath=path;oldDir=pwd;cd(out);
vendor=gpenmpc_install_path('rfly','RflySimAPIs\RflySimSDK\simulink');
addpath(vendor,fullfile(build,'host_runtime'),'-begin');
name='gpenmpc_native_raw_functional_probe';assert(~bdIsLoaded(name));
peer=[];serializer=[];modelCreated=false;peerKilled=false;peerExitCode=[];
peerStarts=0;simulationInvocations=0;simulationCompletions=0;
updateAttempted=false;updateComplete=false;updateClock=[];updateElapsed=[];portsPrePeer=[];
peerReceived=[];peerEchoed=[];nativeFrames=[];
clean=onCleanup(@finish); %#ok<NASGU>
diary(fullfile(out,'MATLAB_DIARY.txt'));diaryGuard=onCleanup(@()diary('off')); %#ok<NASGU>
stage='prepare';
try
    checks=struct('name',{},'pass',{});commands=strings(0,1);
    pairsPath=fullfile(build,'rfly_vendor_integration','full_inner_abi', ...
        'snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
    dialectPath=fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
    peerSource=fullfile(build,'tools','rfly_raw_loopback_peer.cpp');
    nativeMex=string(which('RflyUdpRaw'));assert(endsWith(lower(nativeMex),'rflyudpraw.mexw64'));
    libraryPath=fullfile(vendor,'pixhawk_slib_swarm.slx');
    tracked=[string(mfilename('fullpath'))+'.m';peerSource;pairsPath;dialectPath;nativeMex;string(libraryPath)];
    hashesBefore=arrayfun(@sha,tracked);
    portsBefore=portsAbsent();assert(portsBefore,'gpenmpcNative:RawTestPorts','Test ports 62321/62322 must already be unused.');
    pairs=readbytes(pairsPath);assert(numel(pairs)==596*59);request=pairs(1:310);
    q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(request);
    d=mavlinkdialect(dialectPath,2);
    % Create an unconnected serialization object.
    serializer=mavlinkio(d,'SystemID',double(q.identity.system),'ComponentID',double(q.identity.component));
    m=createmsg(d,'TUNNEL');m.Payload.target_system=uint8(255);m.Payload.target_component=uint8(190);
    m.Payload.payload_type=uint16(42002);m.Payload.payload_length=uint8(128);m.Payload.payload(:)=0;
    m.Payload.payload(1)=uint8(128); % request schema8, fragment0
    m.Payload.payload(2:9)=bigEndianBytes(q.output_generation);
    m.Payload.payload(10:128)=request(1:119);
    packet=uint8(serializemsg(serializer,m));packet=packet(:);delete(serializer);serializer=[];
    assert(numel(packet)==145&&packet(1)==253,'gpenmpcNative:RawTestPacket','Expected one full unsigned 145-byte MAVLink2 TUNNEL.');
    [decoded,status]=deserializemsg(d,packet);
    assert(status==0&&numel(decoded)==1&&decoded.MsgID==385&&decoded.Payload.payload_type==42002);
    assert(isequal(uint8(decoded.Payload.payload(10:128)),reshape(request(1:119),size(decoded.Payload.payload(10:128)))));
    txInput=fullfile(out,'TX_INPUT.bin');writebytes(txInput,packet);
    cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
    exe=fullfile(out,'rfly_raw_loopback_peer.exe');stage='compile_fake_peer';
    command=sprintf('"%s" -std=c++14 -O2 -static -municode -Wall -Wextra -Werror "%s" -lws2_32 -o "%s"',cxx,peerSource,exe);
    commands(end+1)=string(command);[rc,compilerText]=system(command);
    write(fullfile(out,'PEER_BUILD_LOG.txt'),compilerText);assert(rc==0,'%s',compilerText);

    stage='create_only_native_transport_model';load_system('simulink');load_system(libraryPath);
    new_system(name);modelCreated=true;
    set_param(name,'Solver','FixedStepDiscrete','FixedStep','0.001','StopTime','0.25', ...
        'EnablePacing','on','PacingRate','1','ReturnWorkspaceOutputs','on');
    % Continuous Clock caused VarDimsOutputRequireDiscreteST in the earlier
    % compile probe. This discrete clock is the same .001s Simulink step;
    % there is no independent host scheduler or MATLAB pause in either chart.
    add_block('simulink/Sources/Digital Clock',[name '/DiscreteTime'],'SampleTime','0.001');
    add_block('simulink/User-Defined Functions/MATLAB Function',[name '/FixturePacketOnly']);
    tx=find(sfroot,'-isa','Stateflow.EMChart','Path',[name '/FixturePacketOnly']);assert(isscalar(tx));
    literal=sprintf('%u;',packet);
    tx.Script=sprintf(['function y=fcn(t)\n%%#codegen\n' ...
        'coder.varsize(''y'',[1100 1],[true false]);\n' ...
        'y=uint8([%s]);\n' ...
        'if t<0 || t>=0.20, y=zeros(1,1,''uint8''); end\nend\n'],literal);
    tx.SupportVariableSizing=true;
    tc=get_param([name '/FixturePacketOnly'],'MATLABFunctionConfiguration');tc.VectorOutputs1D=true;
    y=find(tx,'-isa','Stateflow.Data','Name','y');y.DataType='uint8';y.Props.Array.Size='1100';y.Props.Array.IsDynamic=true;
    add_block('pixhawk_slib_swarm/RflyUdpRaw',[name '/OfficialRaw'], ...
        'num','[1]','ip1','127.0.0.1','ip','127.0.0.1','isSame','on','port','62321','T','0.001');
    add_block('simulink/User-Defined Functions/MATLAB Function',[name '/PadRawForEvidence']);
    rx=find(sfroot,'-isa','Stateflow.EMChart','Path',[name '/PadRawForEvidence']);assert(isscalar(rx));
    rx.Script=sprintf(['function [padded,count]=fcn(raw)\n%%#codegen\n' ...
        'padded=zeros(5000,1,''uint8'');\n' ...
        'n=numel(raw); assert(n>=1 && n<=5000);\n' ...
        'count=uint16(n); padded(1:n)=raw(:);\nend\n']);
    rx.SupportVariableSizing=true;
    rcfg=get_param([name '/PadRawForEvidence'],'MATLABFunctionConfiguration');rcfg.VectorOutputs1D=true;
    % ThreeUAVsRawMavExp.slx connects Raw SID199 directly to Width SID263
    % and Deserializer SID262. The latter's byteStreamInput (uavmavlinklib
    % system_30.xml, SID30:31) has no explicit PortDimensions constraint.
    % Inherit the native maximum; 5000 is our checked logging capacity, not
    % a width to propagate backwards into mdlSetOutputPortWidth.
    ri=find(rx,'-isa','Stateflow.Data','Name','raw');ri.DataType='uint8';ri.Props.Array.Size='-1';ri.Props.Array.IsDynamic=true;
    rp=find(rx,'-isa','Stateflow.Data','Name','padded');rp.DataType='uint8';rp.Props.Array.Size='5000';rp.Props.Array.IsDynamic=false;
    rn=find(rx,'-isa','Stateflow.Data','Name','count');rn.DataType='uint16';rn.Props.Array.Size='1';rn.Props.Array.IsDynamic=false;
    add_block('simulink/Sinks/To Workspace',[name '/RawBytesLog'], ...
        'VariableName','nativeRxBytes','SaveFormat','Array','MaxDataPoints','10000');
    add_block('simulink/Sinks/To Workspace',[name '/RawLengthLog'], ...
        'VariableName','nativeRxLengths','SaveFormat','Array','MaxDataPoints','10000');
    add_block('simulink/Sinks/To Workspace',[name '/SimulationTimeLog'], ...
        'VariableName','nativeSimulationTimes','SaveFormat','Array','MaxDataPoints','10000');
    add_line(name,'DiscreteTime/1','FixturePacketOnly/1');add_line(name,'DiscreteTime/1','SimulationTimeLog/1');
    add_line(name,'FixturePacketOnly/1','OfficialRaw/1');add_line(name,'OfficialRaw/1','PadRawForEvidence/1');
    add_line(name,'PadRawForEvidence/1','RawBytesLog/1');add_line(name,'PadRawForEvidence/2','RawLengthLog/1');
    assert(isempty(get_param(name,'InitFcn'))&&isempty(get_param(name,'StartFcn'))&&isempty(get_param(name,'StopFcn')));
    modelPath=fullfile(out,[name '.slx']);save_system(name,modelPath);

    % Resolve chart shapes and compile the target before the peer's bounded wait.
    % Vendor initialization may briefly open localhost UDP.
    stage='host_only_update_diagram';updateAttempted=true;updateClock=tic;
    set_param(name,'SimulationCommand','update');updateElapsed=toc(updateClock);updateComplete=true;
    stage='verify_ports_released_after_update';portsPrePeer=portsAbsent();
    assert(portsPrePeer,'gpenmpcNative:RawPostUpdatePorts', ...
        'Compile-only update must release native port 62322 and leave peer port 62321 unused.');

    stage='start_explicit_fake_peer';si=System.Diagnostics.ProcessStartInfo;si.FileName=char(exe);
    si.Arguments=sprintf('"%s" "%s"',txInput,out);si.WorkingDirectory=char(out);
    si.UseShellExecute=false;si.CreateNoWindow=true;si.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;
    si.RedirectStandardOutput=true;si.RedirectStandardError=true;peer=System.Diagnostics.Process.Start(si);
    peerStarts=peerStarts+1;
    ready=fullfile(out,'READY.json');readyWait=tic;
    while ~isfile(ready)&&~peer.HasExited&&toc(readyWait)<5,pause(.01);end
    assert(isfile(ready)&&~peer.HasExited,'gpenmpcNative:RawPeerReady','Fake peer did not become ready.');
    readyReceipt=jsondecode(fileread(ready));assert(readyReceipt.ready);
    % Run one normal simulation with the prepared target and record total wall time.
    stage='single_native_simulation';simulationInvocations=simulationInvocations+1;
    wall=tic;simulation=sim(name);simulationWall=toc(wall);simulationCompletions=simulationCompletions+1;
    lengths=simulation.get('nativeRxLengths');lengths=double(lengths(:));
    times=simulation.get('nativeSimulationTimes');times=double(times(:));
    padded=recordRows(simulation.get('nativeRxBytes'),numel(lengths));
    save(fullfile(out,'NATIVE_RAW.mat'),'padded','lengths','times','packet','request','simulationWall');
    stage='graceful_peer_finish';stopPeer();
    peerReport=jsondecode(fileread(fullfile(out,'PEER_RESULT.json')));
    peerReceived=peerReport.received_count;peerEchoed=peerReport.echoed_count;
    [peerPackets,peerTicks]=readPeer(fullfile(out,'PEER_RAW.bin'));
    check('fake_peer_bound_closed_gracefully_without_unexpected_datagrams', ...
        peerReport.bind_ok&&peerReport.close_ok&&peerReport.stop_requested&&~peerReport.hard_timeout ...
        &&peerReport.error_code==0&&peerReport.unexpected_count==0&&~peerKilled&&peerExitCode==0);
    check('every_actual_native_tx_arriving_at_peer_exact',numel(peerPackets)>0 ...
        &&numel(peerPackets)==peerReport.received_count&&all(cellfun(@(b)isequal(b,packet),peerPackets)));
    check('peer_echo_counter_matches_received_counter',peerReport.echoed_count==peerReport.received_count);
    check('native_log_is_full_uint8_with_consistent_samples',isa(padded,'uint8') ...
        &&size(padded,1)==numel(lengths)&&numel(times)==numel(lengths) ...
        &&all(lengths>=1&lengths<=5000)&&all(diff(times)>=0));
    exactRx=true;validFrames=true;zeroPadding=true;sentinels=0;frameCount=0;nonemptySamples=0;
    rxFrames=cell(0,1);rxSampleIndices=zeros(0,1);rxOffsets=zeros(0,1);
    for k=1:numel(lengths)
        n=lengths(k);bytes=padded(k,1:n).';
        zeroPadding=zeroPadding&&all(padded(k,n+1:end)==0);
        if n==1&&bytes(1)==0,sentinels=sentinels+1;continue;end
        nonemptySamples=nonemptySamples+1;
        if mod(n,numel(packet))~=0,exactRx=false;validFrames=false;continue;end
        for offset=0:numel(packet):n-1
            b=bytes(offset+1:offset+numel(packet));frameCount=frameCount+1;
            rxFrames{end+1,1}=b;rxSampleIndices(end+1,1)=k;rxOffsets(end+1,1)=offset; %#ok<AGROW>
            exactRx=exactRx&&isequal(b,packet);
            [got,s]=deserializemsg(d,b);
            validFrames=validFrames&&isscalar(got)&&all(s==0)&&got.MsgID==385 ...
                &&got.SystemID==decoded.SystemID&&got.ComponentID==decoded.ComponentID ...
                &&isequal(got.Payload,decoded.Payload);
        end
    end
    nativeFrames=frameCount;
    check('every_native_received_frame_matches_original_tunnel_bytes',frameCount>0&&exactRx&&validFrames);
    check('native_received_all_echoed_frames_in_bounded_functional_probe',frameCount==peerReport.echoed_count);
    check('fixed_padding_does_not_hide_or_add_received_data',zeroPadding&&sentinels+nonemptySamples==numel(lengths));
    check('peer_qpc_ticks_are_original_monotone_ticks',peerReport.qpc_frequency>0 ...
        &&all(peerTicks>0)&&all(diff(peerTicks)>=0));
    closeModel();portsAfter=portsAbsent();check('both_fake_test_ports_released',portsAfter);
    hashesAfter=arrayfun(@sha,tracked);check('vendor_source_binary_fixture_and_tool_inputs_unchanged',isequal(hashesBefore,hashesAfter));
    save(fullfile(out,'VERIFIED_FRAMES.mat'),'peerPackets','peerTicks','rxFrames','rxSampleIndices','rxOffsets','peerReport');
    report=struct('scope','OFFICIAL_RFLY_UDP_RAW_SINGLE_NATIVE_BLOCK_LOCALHOST_FUNCTIONAL_ECHO_ONLY', ...
        'checks',checks,'total',numel(checks),'passed',sum([checks.pass]),'all_pass',all([checks.pass]), ...
        'vendor_s_function',nativeMex,'vendor_mex_sha256',sha(nativeMex),'model_path',modelPath, ...
        'nominal_sample_time_s',.001,'simulation_stop_s',.25,'tx_until_simulation_s',.20, ...
        'pacing_requested',true,'pacing_rate',1,'simulation_including_compile_wall_s',simulationWall, ...
        'host_only_update_attempted',updateAttempted,'host_only_update_completed',updateComplete, ...
        'host_only_update_elapsed_s',updateElapsed, ...
        'native_log_samples',numel(lengths),'native_no_rx_zero_sentinels',sentinels, ...
        'native_nonempty_rx_samples',nonemptySamples,'peer_received_native_tx_datagrams',peerReport.received_count, ...
        'peer_echoed_datagrams',peerReport.echoed_count,'native_logged_echo_rx_frames',frameCount, ...
        'echoes_not_observed_before_simulation_stop',max(0,peerReport.echoed_count-frameCount), ...
        'native_frames_exceeding_peer_echo_count',max(0,frameCount-peerReport.echoed_count), ...
        'nominal_payload_input_samples_not_measured_tx_calls',sum(times>=0&times<.20), ...
        'packet_bytes',numel(packet),'packet_sha256',hashBytes(packet),'request_sha256',hashBytes(request), ...
        'MAVLink_version',2,'message_id',385,'payload_type',42002,'request_schema',8,'fragment_index',0, ...
        'payload_provenance','FIRST119_BYTES_OF_FIRST_RETAINED_C_RGP1_QUERY_PLUS_ORIGINAL_OUTPUT_GENERATION', ...
        'same_frame_repeated',true,'freshness_unique_sequence_or_session_proven',false, ...
        'peer_bind','127.0.0.1:62321','peer_required_source_and_echo_target','127.0.0.1:62322', ...
        'native_bind_static_contract','0.0.0.0:62322','native_target','127.0.0.1:62321', ...
        'ports_absent_before_start',portsBefore,'ports_absent_immediately_before_peer_start',portsPrePeer, ...
        'ports_absent_after_finish',portsAfter, ...
        'peer',peerReport,'peer_exit_code',peerExitCode,'peer_forced_kill',peerKilled, ...
        'peer_timestamp_semantics','ORIGINAL_INT64_QPC_TICKS_DIVIDE_BY_REPORTED_FREQUENCY_NOT_NANOSECONDS', ...
        'timing_claim','TRANSPORT_FUNCTIONAL_TEST', ...
        'commands',commands,'tracked',tracked,'before_sha256',hashesBefore,'after_sha256',hashesAfter, ...
        'COM',0,'board',0,'hardware_actions',0,'CopterSim_instances',0,'plant_runs',0,'controller_runs',0, ...
        'MATLAB_sockets',0,'MATLAB_timers',0,'fake_peer_processes_started',peerStarts, ...
        'native_transport_simulation_invocations',simulationInvocations, ...
        'native_transport_simulations',simulationCompletions);
    write(fullfile(out,'RESULT.json'),jsonencode(report,PrettyPrint=true));
    fprintf('Official RflyUdpRaw localhost %d/%d: peer TX=%d, echo=%d, native RX=%d. Functional only.\n', ...
        report.passed,report.total,report.peer_received_native_tx_datagrams,report.peer_echoed_datagrams,frameCount);
    assert(report.all_pass,'gpenmpcNative:RawFunctionalFailed','Native transport functional check failed.');
catch ex
    if updateAttempted&&~updateComplete,updateElapsed=toc(updateClock);end
    clear clean
    % Before the peer/sim are entered these are exact zeros. If execution
    % began but logs were not read, keep unknown packet counts empty instead
    % of incorrectly presenting absence of evidence as zero traffic.
    if peerStarts==0,peerReceived=0;peerEchoed=0;end
    if simulationInvocations==0,nativeFrames=0;end
    failure=struct('failure_stage',stage,'exception',exceptionTree(ex), ...
        'host_only_update_attempted',updateAttempted,'host_only_update_completed',updateComplete, ...
        'host_only_update_elapsed_s',updateElapsed, ...
        'ports_absent_immediately_before_peer_start',portsPrePeer, ...
        'peer_forced_kill',peerKilled,'peer_exit_code',peerExitCode, ...
        'fake_peer_processes_started',peerStarts, ...
        'native_transport_simulation_invocations',simulationInvocations, ...
        'native_transport_simulations',simulationCompletions, ...
        'peer_received_native_tx_datagrams',peerReceived,'peer_echoed_datagrams',peerEchoed, ...
        'native_logged_echo_rx_frames',nativeFrames, ...
        'COM',0,'board',0,'hardware_actions',0,'CopterSim_instances',0, ...
        'plant_runs',0,'controller_runs',0,'MATLAB_sockets',0,'MATLAB_timers',0);
    write(fullfile(out,'FAILURE.json'),jsonencode(failure,PrettyPrint=true));rethrow(ex);
end
clear clean
    function check(n,p)
        checks(end+1)=struct('name',n,'pass',logical(p)); %#ok<AGROW>
        if ~p,fprintf(2,'FAILED %s\n',n);end
    end
    function stopPeer()
        if isempty(peer),return;end
        stop=fullfile(out,'STOP.txt');if ~isfile(stop),write(stop,'GRACEFUL_HOST_TEST_STOP');end
        if ~peer.HasExited&&~peer.WaitForExit(5000),peer.Kill();peerKilled=true;peer.WaitForExit(2000);end
        if peer.HasExited,peerExitCode=double(peer.ExitCode);end
        stdout=char(peer.StandardOutput.ReadToEnd());stderr=char(peer.StandardError.ReadToEnd());
        write(fullfile(out,'PEER_STDOUT.txt'),stdout);write(fullfile(out,'PEER_STDERR.txt'),stderr);
        peer.Dispose();peer=[];
    end
    function closeModel()
        if modelCreated&&bdIsLoaded(name),bdclose(name);end
        modelCreated=false;
    end
    function finish()
        try,closeModel();catch,end
        try,stopPeer();catch,end
        if ~isempty(serializer),try,delete(serializer);catch,end,serializer=[];end
        cd(oldDir);path(oldPath);
    end
end
function rows=recordRows(values,n)
if ismatrix(values)&&size(values,1)==n&&size(values,2)==5000,rows=values;
elseif size(values,1)==5000&&numel(values)==n*5000,rows=reshape(values,5000,n).';
else,error('gpenmpcNative:RawLogShape','Unexpected fixed raw evidence shape.');end
end
function [packets,ticks]=readPeer(p)
f=fopen(p,'rb','ieee-le');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
packets=cell(0,1);ticks=zeros(0,1,'int64');
while true
    [n,count]=fread(f,1,'uint32=>uint32');if count==0,break;end
    assert(n>=6&&n<=300);[t,count]=fread(f,1,'int64=>int64');assert(count==1);
    [b,count]=fread(f,double(n),'*uint8');assert(count==double(n));
    packets{end+1,1}=b;ticks(end+1,1)=t; %#ok<AGROW>
end
end
function ok=portsAbsent()
networkProperties=System.Net.NetworkInformation.IPGlobalProperties.GetIPGlobalProperties();
listeners=networkProperties.GetActiveUdpListeners();
ok=true;for k=1:listeners.Length,port=double(listeners(k).Port);ok=ok&&port~=62321&&port~=62322;end
end
function b=bigEndianBytes(v)
[~,~,e]=computer;if e=='L',v=swapbytes(v);end,b=reshape(typecast(v(:),'uint8'),[],1);
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function writebytes(p,b)
assert(~isfile(p));f=fopen(p,'wb');assert(f>=0);g=onCleanup(@()fclose(f));assert(fwrite(f,b,'uint8')==numel(b)); %#ok<NASGU>
end
function h=sha(p),h=hashBytes(readbytes(p));end
function h=hashBytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(:),'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function write(p,t)
f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));fprintf(f,'%s\n',t); %#ok<NASGU>
end
function tree=exceptionTree(ex)
causes=cell(size(ex.cause));for k=1:numel(ex.cause),causes{k}=exceptionTree(ex.cause{k});end
tree=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack, ...
    'report',getReport(ex,'extended','hyperlinks','off'),'causes',{causes});
end
