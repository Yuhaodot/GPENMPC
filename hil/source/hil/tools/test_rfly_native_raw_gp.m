function report=test_rfly_native_raw_gp(outputRoot,runName,peerWait,requestProcessTimer,compiled,replyPairs)
% Replay GP bytes through native Raw, reassembly and the GP MEX.
arguments
    outputRoot (1,1) string
    runName (1,1) string = "native_raw_gp"
    peerWait (1,1) string {mustBeMember(peerWait,["sleep","select"])} = "sleep"
    requestProcessTimer (1,1) logical = false
    compiled (1,1) logical = false
    replyPairs (1,1) logical = false
end
assert(~replyPairs||compiled,'Explicit reply pairing is only available in the compiled retained fixture.');
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(isfolder(outputRoot)&&~isempty(regexp(char(runName),'^[A-Z0-9_]+$','once')));
out=fullfile(outputRoot,runName);assert(~isfolder(out)&&~isfile(out),'Preserve prior evidence.');mkdir(out);
oldPath=path;oldDir=pwd;cd(out);vendor=gpenmpc_install_path('rfly','RflySimAPIs\RflySimSDK\simulink');
mexDir=fullfile(gpenmpc_external_path('canonical_gp_wire_mex'));
addpath(fullfile(build,'tools'),fullfile(build,'host_runtime'),vendor,mexDir,'-begin');
name='gpenmpc_native_raw_gp_replay_probe';assert(~bdIsLoaded(name));
peer=[];modelCreated=false;peerKilled=false;peerExitCode=[];peerStarts=0;simInvocations=0;simCompleted=0;
ledgerPath=fullfile(out,'GP_SYSTEM_LEDGER.mat');
updateAttempted=false;updateComplete=false;updateClock=[];updateElapsed=[];portsPrePeer=[];
timerBegin=[];timerEnd=[];timerRequested=false;timerBinary="";
compiledBinary="";compiledBinarySha="";compiledVariable='';compiledLedgerSaved=false;
compiledSourceBinding="";
compiledCaptureErrors=cell(0,1);compiledConversionError=[];
clean=onCleanup(@finish); %#ok<NASGU>
diary(fullfile(out,'MATLAB_DIARY.txt'));diaryGuard=onCleanup(@()diary('off')); %#ok<NASGU>
stage='prepare_exact_original_inputs';
try
    checks=struct('name',{},'pass',{});
    pairsPath=fullfile(build,'rfly_vendor_integration','full_inner_abi','snapshot_wire_fixture','RGP1_RGR1_PAIRS.bin');
    pairs=reshape(readbytes(pairsPath),596,59);requests=pairs(1:310,:);expectedReplies=pairs(311:596,:);
    dialect=fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
    mexPath=fullfile(mexDir,'canonical_gp_wire_mex.mexw64');
    mexSha='21017CD36C857466AE538EAE716C868173D2C54DF4D2B5CC458E3C6681CEBE6B';
    assert(strcmpi(sha(mexPath),mexSha)&&strcmpi(string(which('canonical_gp_wire_mex')),mexPath));
    validation=jsondecode(fileread(fullfile(mexDir,'RESULT.json')));
    assert(validation.all_pass&&validation.actual_C_query_reply_rows==59 ...
        &&validation.all_native_reply_bytes_equal_C_fixture&&strcmpi(validation.binary_sha256,mexSha));
    peerSource=fullfile(build,'tools','rfly_raw_gp_loopback_peer.cpp');
    systemSource=fullfile(build,'tools','GPENMPCNativeRawGpSystem.m');
    nativeMex=string(which('RflyUdpRaw'));assert(endsWith(lower(nativeMex),'rflyudpraw.mexw64'));
    libraryPath=fullfile(vendor,'pixhawk_slib_swarm.slx');
    tracked=[string(mfilename('fullpath'))+'.m';systemSource;peerSource;pairsPath;dialect;mexPath;nativeMex;string(libraryPath); ...
        fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalGpCodec.m'); ...
        fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalTunnelReassembler.m')];
    if compiled
        stage='bind_already_built_compiled_gp_candidate';
        retainedRun=fullfile(gpenmpc_external_path('native_raw_gp_runtime'));
        retainedResult=fullfile(retainedRun,'RESULT.json');
        assert(strcmpi(sha(retainedResult),'2AC4B0683C97DA9BE38298043E9D186887AA0CF023AD6B6AFB3CF90060E417A2'));
        previous=jsondecode(fileread(retainedResult));assert(previous.all_pass&&previous.actual_gp_calls==59);
        compiledSource=fullfile(build,'tools','gpenmpc_native_raw_gp_sfcn.cpp');
        if replyPairs
            compiledDir=fullfile(gpenmpc_external_path('compiled_native_raw_gp_pairs_host'));
            compiledBinary=fullfile(compiledDir,'gpenmpc_native_raw_gp_sfcn.mexw64');
            compiledBinarySha='EFB5916B1F32CE8E321471EEB8D9D65AD125E01F8A6223E887992A6A138D03B1';
            compiledSourceSha='67BFEF71C979CE08F53E775587CEB68CF80489FB0D4011F34C26BC1D92CF594A';
            retainedPackets=fullfile(retainedRun,'PEER_RAW.bin');
            assert(strcmpi(sha(retainedPackets),'73297602F002F10861C728386C4735FECE36471D4138646BFAB6CAEF6AD68A75'));
            [originalDirections,originalPackets,~]=readPeer(retainedPackets);
            tracked=[tracked;compiledSource;retainedPackets];
            compiledSourceBinding='CURRENT_SOURCE_AND_NEW_HOST40_TEST_BINARY';expectedTests=40;
            assert(strcmpi(sha(compiledSource),compiledSourceSha));
        else
            % Preserve the existing binary and its recorded source identity.
            compiledDir=fullfile(gpenmpc_external_path('compiled_native_raw_gp_host'));
            compiledBinary=fullfile(compiledDir,'dimension_callback_fix','gpenmpc_native_raw_gp_sfcn.mexw64');
            compiledBinarySha='22C9108F1779313DEDD402AC55E02DACD7ACCAE2F8466EE9F43EF8F2876AA26E';
            compiledSourceSha='B4D041C840589CEBAB3C9A2E0DE94B54D5FC8C6387CAA587EDA62B75B286BEB5';
            ix=find(strcmpi(string(previous.tracked),compiledSource));
            assert(isscalar(ix)&&strcmpi(previous.before_sha256{ix},compiledSourceSha) ...
                &&strcmpi(previous.compiled_binary_sha256,compiledBinarySha));
            compiledSourceBinding='BOUND_BINARY_SOURCE_RECEIPT';expectedTests=32;
        end
        compiledValidation=fullfile(compiledDir,'CORE_TEST_RESULT.json');
        cv=jsondecode(fileread(compiledValidation));
        if replyPairs
            assert(strcmpi(sha(compiledValidation),'DF78C569A94A28C569FF9AAEC9B55D42FA6C819BA6368E245E6F0844BA900C7C') ...
                &&strcmpi(cv.entries(1).sha256,compiledSourceSha), ...
                'gpenmpcNative:RawGpPairCoreProof','Require the exact passed40 same-Core HOST receipt.');
            tracked=[tracked;fullfile(compiledDir,'BUILD_RECEIPT.json')];
        end
        assert(cv.all_pass&&cv.total==expectedTests&&cv.passed==expectedTests ...
            &&strcmpi(sha(compiledBinary),compiledBinarySha), ...
            'gpenmpcNative:RawGpCompiledBinding','Require the already tested exact compiled Core and MEX.');
        assert(isempty(which('gpenmpc_native_raw_gp_sfcn')), ...
            'gpenmpcNative:RawGpCompiledAlreadyResolved','Do not replace another loaded compiled candidate.');
        % Load the compiled Core with fixed-output Simulink registration.
        addpath(fileparts(compiledBinary),'-begin');assert(strcmpi(string(which('gpenmpc_native_raw_gp_sfcn')),compiledBinary));
        compiledVariable=['gpenmpcRawGp_' char(runName)];
        assert(isvarname(compiledVariable)&&~evalin('base',sprintf('exist(''%s'',''var'')',compiledVariable)), ...
            'gpenmpcNative:RawGpCompiledLedgerVariable','Require an unused unique base-workspace ledger variable.');
        tracked=[tracked;compiledBinary;compiledValidation;retainedResult; ...
            fullfile(build,'tools','gpenmpc_native_raw_gp_sfcn_standalone_test.hpp'); ...
            fullfile(build,'tools','normalize_rfly_compiled_gp_ledger.m')];
    end
    hashesBefore=arrayfun(@sha,tracked);portsBefore=portsAbsent();assert(portsBefore,'Test ports must already be unused.');
    for k=1:59
        q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(requests(:,k));r=gpenmpcNative.RflyLocalGpCodec.decodeReply(expectedReplies(:,k));
        assert(isequal(q.identity,r.identity)&&isequal(q.original_request_sha256,r.original_request_sha256));
    end
    replayConfig=struct('requests',requests,'dialect_path',char(dialect), ...
        'mex_path',char(mexPath),'mex_sha256',mexSha,'ledger_path',char(ledgerPath), ...
        'replay_binding_sha256',char(hashBytes([requests(:);uint8(char(runName)).'])));
    if compiled
        compiledReplayConfig=struct('scope','RETAINED_C_GP_FIXTURE_ONLY','pairs',pairs, ...
            'fixture_sha256',char(sha(pairsPath)),'replay_binding_sha256',replayConfig.replay_binding_sha256, ...
            'ledger_variable',compiledVariable);
        if replyPairs,compiledReplayConfig.reply_pair_datagrams=true;end
    end
    % Bind the host replay fixture.
    stage='compile_fake_gp_peer';cxx=gpenmpc_install_path('llvm','bin\clang++.exe');
    exe=fullfile(out,'rfly_raw_gp_loopback_peer.exe');
    command=sprintf('"%s" -std=c++14 -O2 -static -municode -Wall -Wextra -Werror -Wno-address-of-packed-member "%s" -lws2_32 -o "%s"',cxx,peerSource,exe);
    [rc,compilerText]=system(command);write(fullfile(out,'PEER_BUILD_LOG.txt'),compilerText);assert(rc==0,'%s',compilerText);
    if requestProcessTimer
        stage='compile_calling_process_timer_resource';
        assert(System.Environment.OSVersion.Version.Build>=19041,'Process-local timer semantics require Windows 10 2004 or later.');
        timerSource=fullfile(build,'tools','host_process_timer_mex.cpp');
        assert(isempty(which('host_process_timer_mex')),'Do not replace an existing timer owner.');
        tracked=[tracked;timerSource];hashesBefore=[hashesBefore;sha(timerSource)];
        timerBinary=fullfile(out,'host_process_timer_mex.mexw64');
        versionObject=fullfile(mexDir,'mex_version.o');
        timerCommand=sprintf(['"%s" -std=c++14 -O2 -shared -static -Wall -Wextra -Werror ' ...
            '-DMATLAB_MEX_FILE -DMATLAB_DEFAULT_RELEASE=R2018a -I"%s" "%s" "%s" "%s" "%s" "%s" ' ...
            '-lwinmm -Wl,--no-undefined -o "%s"'],cxx,fullfile(matlabroot,'extern','include'),timerSource,versionObject, ...
            fullfile(matlabroot,'extern','lib','win64','microsoft','libmex.lib'), ...
            fullfile(matlabroot,'extern','lib','win64','microsoft','libmx.lib'), ...
            fullfile(matlabroot,'extern','lib','win64','mingw64','exportsmexfileversion.def'),timerBinary);
        [rc,timerText]=system(timerCommand);write(fullfile(out,'TIMER_BUILD_LOG.txt'),timerText);assert(rc==0,'%s',timerText);
        addpath(out,'-begin');assert(strcmpi(string(which('host_process_timer_mex')),timerBinary),'Resolve only the just-built timer resource.');
        timerStatus=host_process_timer_mex(2);assert(~timerStatus.active);
    end
    stage='create_only_native_gp_transport_model';load_system('simulink');load_system(libraryPath);
    new_system(name);modelCreated=true;
    set_param(name,'Solver','FixedStepDiscrete','FixedStep','0.001','StopTime','2', ...
        'EnablePacing','on','PacingRate','1','ReturnWorkspaceOutputs','on');
    workspace=get_param(name,'ModelWorkspace');assignin(workspace,'replayConfig',replayConfig);
    add_block('simulink/Sources/Digital Clock',[name '/DiscreteTime'],'SampleTime','0.001');
    if compiled
        assignin(workspace,'compiledReplayConfig',compiledReplayConfig);
        add_block('simulink/User-Defined Functions/S-Function',[name '/ReplayGp'], ...
            'FunctionName','gpenmpc_native_raw_gp_sfcn','Parameters','compiledReplayConfig');
    else
        add_block('simulink/User-Defined Functions/MATLAB System',[name '/ReplayGp'],'System','GPENMPCNativeRawGpSystem');
        set_param([name '/ReplayGp'],'ReplayConfig','replayConfig');
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[name '/OneQueuedFrame']);
    chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[name '/OneQueuedFrame']);assert(isscalar(chart));
    chart.Script=sprintf(['function y=fcn(padded,n)\n%%#codegen\n' ...
        'coder.varsize(''y'',[1100 1],[true false]);\n' ...
        'assert(n>=1 && n<=300); y=padded(1:double(n));\nend\n']);
    chart.SupportVariableSizing=true;
    cc=get_param([name '/OneQueuedFrame'],'MATLABFunctionConfiguration');cc.VectorOutputs1D=true;
    y=find(chart,'-isa','Stateflow.Data','Name','y');y.DataType='uint8';y.Props.Array.Size='1100';y.Props.Array.IsDynamic=true;
    add_block('pixhawk_slib_swarm/RflyUdpRaw',[name '/OfficialRaw'], ...
        'num','[1]','ip1','127.0.0.1','ip','127.0.0.1','isSame','on','port','62321','T','0.001');
    add_line(name,'OfficialRaw/1','ReplayGp/1');add_line(name,'DiscreteTime/1','ReplayGp/2');
    add_line(name,'ReplayGp/1','OneQueuedFrame/1');add_line(name,'ReplayGp/2','OneQueuedFrame/2');
    add_line(name,'OneQueuedFrame/1','OfficialRaw/1');
    assert(isempty(get_param(name,'InitFcn'))&&isempty(get_param(name,'StartFcn'))&&isempty(get_param(name,'StopFcn')));
    modelPath=fullfile(out,[name '.slx']);save_system(name,modelPath);
    stage='host_only_update_diagram';updateAttempted=true;updateClock=tic;
    set_param(name,'SimulationCommand','update');updateElapsed=toc(updateClock);updateComplete=true;
    stage='verify_ports_released_after_update';portsPrePeer=portsAbsent();
    assert(portsPrePeer,'gpenmpcNative:RawGpPostUpdatePorts','Update must release both test ports before starting the peer.');
    assert(~isfile(replayConfig.ledger_path),'Compile-only update unexpectedly entered service execution.');
    if compiled
        assert(~evalin('base',sprintf('exist(''%s'',''var'')',compiledVariable)), ...
            'gpenmpcNative:RawGpCompiledUpdateEntered','Compile-only update must not export a runtime ledger.');
    end
    if requestProcessTimer
        timerRequested=true;timerBegin=host_process_timer_mex(1);
        assert(timerBegin.active&&timerBegin.last_result==0,'Timer request failed; do not label it active.');
    end
    stage='start_explicit_fake_gp_peer';si=System.Diagnostics.ProcessStartInfo;si.FileName=char(exe);
    si.Arguments=sprintf('"%s" "%s" %s',pairsPath,out,peerWait);si.WorkingDirectory=char(out);
    if replyPairs,si.Arguments=[char(si.Arguments) ' reply_pairs'];end
    si.UseShellExecute=false;si.CreateNoWindow=true;si.WindowStyle=System.Diagnostics.ProcessWindowStyle.Hidden;
    si.RedirectStandardOutput=true;si.RedirectStandardError=true;peer=System.Diagnostics.Process.Start(si);peerStarts=peerStarts+1;
    ready=fullfile(out,'READY.json');readyClock=tic;
    while ~isfile(ready)&&~peer.HasExited&&toc(readyClock)<5,pause(.01);end
    assert(isfile(ready)&&~peer.HasExited,'gpenmpcNative:RawGpPeerReady','The explicit localhost peer did not become ready.');readyReceipt=jsondecode(fileread(ready));assert(readyReceipt.ready);
    stage='single_native_gp_simulation';simInvocations=simInvocations+1;wall=tic;
    simulation=sim(name);simulationWall=toc(wall);simCompleted=simCompleted+1; %#ok<NASGU>
    closeModel();stage='graceful_gp_peer_finish';stopPeer();releaseTimer();
    captureCompiledLedger();
    s=load(replayConfig.ledger_path,'ledger');ledger=s.ledger;
    peerReport=jsondecode(fileread(fullfile(out,'PEER_RESULT.json')));
    [peerDirection,peerPackets,peerTicks]=readPeer(fullfile(out,'PEER_RAW.bin'));
    if replyPairs
        options=detectImportOptions(fullfile(out,'ROUND_TRIPS.csv'));
        options=setvartype(options,{'first_send_qpc','reply_qpc'},'int64');
        rounds=readtable(fullfile(out,'ROUND_TRIPS.csv'),options);
    else
        rounds=readtable(fullfile(out,'ROUND_TRIPS.csv'));
    end
    stage='check_actual_causal_byte_and_numerical_evidence';
    check('peer_graceful_closed_59_original_queries_without_unexpected_frames', ...
        peerReport.bind_ok&&peerReport.close_ok&&peerReport.stop_requested&&~peerReport.hard_timeout ...
        &&peerReport.error_code==0&&peerReport.unexpected_count==0&&~peerKilled&&peerExitCode==0 ...
        &&peerReport.queries_sent==59&&peerReport.queries_completed==59&&peerReport.all_replies_exact_fixture);
    if compiled
        % Read counters from the compiled Core.
        core=ledger.compiled_core;
        queryAccounting=core.all_complete&&core.error_code==0&&core.gp_calls==59 ...
            &&core.queries_completed==59&&core.rx_frames==177&&core.tx_count==178 ...
            &&core.pending_fragment_count==0&&core.remaining_queue_count==0;
        if replyPairs
            queryAccounting=queryAccounting&&core.reply_pair_datagrams&&core.tx_datagram_count==119;
        end
        check('compiled_exact_QPC_records_and_fixture_binding_retained', ...
            isa(core.raw_qpc,'uint64')&&isequal(size(core.raw_qpc),[double(core.updates) 2]) ...
            &&all(core.raw_qpc(:,2)>=core.raw_qpc(:,1))&&all(core.raw_qpc(2:end,1)>=core.raw_qpc(1:end-1,1)) ...
            &&isa(core.query_qpc,'uint64')&&isequal(size(core.query_qpc),[59 3]) ...
            &&all(core.query_qpc(:,3)>=core.query_qpc(:,2))&&all(core.query_qpc(:,2)>=core.query_qpc(:,1)) ...
            &&strcmpi(core.fixture_sha256,compiledReplayConfig.fixture_sha256) ...
            &&strcmpi(core.replay_binding_sha256,compiledReplayConfig.replay_binding_sha256) ...
            &&compiledLedgerSaved&&isempty(compiledConversionError)&&isempty(compiledCaptureErrors));
    else
        queryAccounting=~ledger.reassembler.status.failed ...
            &&~any(ledger.reassembler.status.active)&&~any(ledger.reassembler.status.ready) ...
            &&isequal(ledger.reassembler.status.messages_completed,uint64([59 0 0]));
    end
    check('one_gp_invocation_per_complete_query_no_inflight_or_queued_tail', ...
        ledger.gp_calls==59&&ledger.queries_completed==59&&numel(ledger.queries)==59 ...
        &&isempty(fieldnames(ledger.failure))&&isempty(fieldnames(ledger.in_flight_at_release)) ...
        &&isempty(ledger.remaining_tx_queue)&&queryAccounting);
    requestsExact=true;identityExact=true;repliesExact=true;hardInvalidExact=true;
    maximumError=0;gpTimes=zeros(numel(ledger.queries),1);
    for k=1:numel(ledger.queries)
        entry=ledger.queries{k};q=entry.pending.decoded;
        actual=gpenmpcNative.RflyLocalGpCodec.decodeReply(entry.reply_bytes);
        expected=gpenmpcNative.RflyLocalGpCodec.decodeReply(expectedReplies(:,k));
        requestsExact=requestsExact&&entry.index==k&&entry.gp_call_index==k ...
            &&isequal(entry.pending.message,requests(:,k))&&isequal(q.original_bytes,requests(:,k));
        identityExact=identityExact&&isequal(entry.reply_bytes(1:110),expectedReplies(1:110,k));
        repliesExact=repliesExact&&isequal(entry.reply_bytes,expectedReplies(:,k));
        hardInvalidExact=hardInvalidExact&&actual.result18(15)==expected.result18(15);
        maximumError=max(maximumError,max(abs(actual.result18-expected.result18)));gpTimes(k)=entry.gp_mex_wall_s;
    end
    check('received_original_C_request_bytes_and_generations_exact',requestsExact&&numel(ledger.queries)==59);
    check('original_reply_identity_model_and_request_digest_exact',identityExact&&numel(ledger.queries)==59);
    check('original_gp_numerics_within_existing_1e_10_and_hard_invalid_exact',maximumError<=1e-10&&hardInvalidExact&&numel(ledger.queries)==59);
    check('actual_native_gp_reply_bytes_equal_prevalidated_C_fixture',repliesExact&&numel(ledger.queries)==59);
    systemRx=cell(0,1);sentinels=0;aggregateMax=0;rawSampleShape=true;
    for k=1:numel(ledger.raw)
        b=ledger.raw{k}.bytes;aggregateMax=max(aggregateMax,numel(b));rawSampleShape=rawSampleShape&&isa(b,'uint8')&&numel(b)<=4999;
        if numel(b)==1&&b(1)==0,sentinels=sentinels+1;continue;end
        systemRx=[systemRx;splitFrames(b)]; %#ok<AGROW>
    end
    peerTx=peerPackets(peerDirection==1);peerRx=peerPackets(peerDirection==0);
    systemTx=cellfun(@(entry)entry.bytes,ledger.system_output_frames,'UniformOutput',false);
    check('all_177_actual_peer_query_datagrams_reach_native_system_exactly',rawSampleShape ...
        &&numel(peerTx)==177&&peerReport.sent_count==177&&isequal(systemRx,peerTx));
    if replyPairs
        systemDatagrams=cellfun(@(entry)entry.bytes,ledger.system_output_datagrams,'UniformOutput',false);
        peerRxFrames=cell(0,1);
        for k=1:numel(peerRx),peerRxFrames=[peerRxFrames;splitFrames(peerRx{k})];end %#ok<AGROW>
        check('all119_actual_datagrams_cover178_original_frames_including_one_heartbeat', ...
            numel(peerRx)==119&&peerReport.received_count==119&&peerReport.reply_fragments==177 ...
            &&peerReport.received_frame_count==178&&peerReport.reply_pair_datagrams==59&&peerReport.reply_pairs_enabled ...
            &&numel(systemTx)==178&&isequal(systemDatagrams,peerRx)&&isequal(systemTx,peerRxFrames));
        baselineFrameExact=isequal(peerTx,originalPackets(originalDirections==1)) ...
            &&isequal(peerRxFrames,originalPackets(originalDirections==0));
        check('all177_requests_and177_reply_frames_byte_equal_retained006_without_reencoding',baselineFrameExact);
        mapPath=fullfile(out,'PEER_DATAGRAM_FRAMES.csv');options=detectImportOptions(mapPath);
        options=setvartype(options,'qpc','int64');peerFrameMap=readtable(mapPath,options);
        mapExact=height(peerFrameMap)==355;
        for k=1:numel(peerPackets)
            rows=find(peerFrameMap.datagram_index==k);frames=splitFrames(peerPackets{k});offset=0;
            mapExact=mapExact&&numel(rows)==numel(frames);
            for j=1:numel(rows)
                row=peerFrameMap(rows(j),:);
                mapExact=mapExact&&j<=numel(frames)&&row.direction==peerDirection(k)&&row.frame_index==j ...
                    &&row.offset==offset&&row.length==numel(frames{j})&&row.qpc==peerTicks(k);
                offset=offset+row.length;
            end
            mapExact=mapExact&&offset==numel(peerPackets{k});
        end
        outputQpc=core.tx_datagram_qpc;receiveQpc=peerTicks(peerDirection==0);
        check('actual_frame_offsets_counts_and_shared_datagram_QPC_match_original_raw',mapExact ...
            &&isa(peerFrameMap.qpc,'int64')&&numel(outputQpc)==119 ...
            &&all(uint64(receiveQpc)>=outputQpc) ...
            &&all(cellfun(@(v)v.frame_count>=1&&v.frame_count<=2&&numel(v.bytes)<=300,ledger.system_output_datagrams)));
    else
        systemDatagrams=systemTx;peerFrameMap=table();
        check('all_178_system_outputs_reach_peer_exactly_including_one_heartbeat', ...
            numel(peerRx)==178&&peerReport.received_count==178&&peerReport.reply_fragments==177&&isequal(systemTx,peerRx));
    end
    causal=true;heartbeatCount=0;
    for k=1:numel(ledger.system_output_frames)
        entry=ledger.system_output_frames{k};
        if entry.query==0,heartbeatCount=heartbeatCount+1;continue;end
        q=ledger.queries{entry.query};
        causal=causal&&entry.output_at_simulation_s>q.completion_at_simulation_s ...
            &&isequal(entry.bytes,q.reply_frames{entry.fragment+1});
    end
    check('every_reply_fragment_output_follows_its_own_gp_completion_tick',causal&&heartbeatCount==1&&numel(systemTx)==178);
    check('measured_peer_roundtrips_complete_and_monotone_QPC',height(rounds)==59 ...
        &&isequal(rounds.query,(1:59).')&&all(rounds.first_send_qpc>0) ...
        &&all(rounds.reply_qpc>rounds.first_send_qpc)&&all(rounds.elapsed_ms>0) ...
        &&peerReport.qpc_frequency>0&&all(peerTicks>0)&&all(diff(peerTicks)>=0));
    portsAfter=portsAbsent();check('both_fake_test_ports_released',portsAfter);
    if requestProcessTimer
        check('calling_process_timer_request_released_exactly_once',~timerEnd.active ...
            &&timerEnd.begin_successes==1&&timerEnd.end_successes==1&&timerEnd.last_result==0);
    end
    hashesAfter=arrayfun(@sha,tracked);check('all_original_sources_fixtures_and_validated_binaries_unchanged',isequal(hashesBefore,hashesAfter));
    save(fullfile(out,'VERIFIED_NATIVE_GP.mat'),'peerDirection','peerPackets','peerTicks','peerFrameMap','rounds','systemRx','systemTx','systemDatagrams','gpTimes','requests','expectedReplies');
    report=struct('scope','NATIVE_RAW_SAME_PROCESS_ORIGINAL_GP_RETAINED_C_QUERY_REPLAY', ...
        'checks',checks,'total',numel(checks),'passed',sum([checks.pass]),'all_pass',all([checks.pass]), ...
        'actual_retained_C_queries',59,'actual_completed_queries',ledger.queries_completed,'actual_gp_calls',ledger.gp_calls, ...
        'native_updates',ledger.updates,'native_no_rx_zero_sentinels',sentinels,'maximum_actual_native_rx_aggregate_bytes',aggregateMax, ...
        'actual_peer_query_datagrams',numel(peerTx),'actual_peer_reply_fragments',peerReport.reply_fragments, ...
        'actual_peer_received_datagrams',numel(peerRx),'system_readiness_heartbeat_outputs',heartbeatCount, ...
        'maximum_absolute_error_vs_C_fixture',maximumError, ...
        'numerical_tolerance',1e-10,'hard_invalid_exact',hardInvalidExact, ...
        'all_reply_bytes_equal_C_fixture',repliesExact,'gp_mex_sha256',mexSha,'gp_mex_path',mexPath, ...
        'compiled_hotpath',compiled,'compiled_binary',compiledBinary,'compiled_binary_sha256',compiledBinarySha, ...
        'compiled_source_binding_scope',compiledSourceBinding,'reply_pair_datagrams_enabled',replyPairs, ...
        'actual_system_MAVLink_frames',numel(systemTx),'actual_system_datagrams',numel(systemDatagrams), ...
        'compiled_base_ledger_saved',compiledLedgerSaved,'compiled_conversion_error',compiledConversionError, ...
        'compiled_capture_errors',{compiledCaptureErrors}, ...
        'source_timestamp_and_original_publication_fields_unchanged',requestsExact, ...
        'retained_replay_not_live_age_validated',true, ...
        'fresh_live_pending_admission_proven',false,'actual_C_kernel_rerun',false, ...
        'normal_simulation_requested_step_s',.001,'normal_simulation_requested_duration_s',2, ...
        'pacing_requested',true,'pacing_rate',1,'simulation_including_init_wall_s',simulationWall, ...
        'host_only_update_elapsed_s',updateElapsed,'ports_absent_before_update',portsBefore, ...
        'ports_absent_immediately_before_peer_start',portsPrePeer,'ports_absent_after_finish',portsAfter, ...
        'roundtrip_median_ms',median(rounds.elapsed_ms),'roundtrip_p95_ms',nearestRank95(rounds.elapsed_ms), ...
        'roundtrip_percentile_method','NEAREST_RANK_CEIL_0_95_N', ...
        'roundtrip_max_ms',max(rounds.elapsed_ms),'gp_mex_observed_median_ms',median(gpTimes)*1000, ...
        'timing_claim','MEASURED_REPLAY_ROUNDTRIP_TIMING', ...
        'model_path',modelPath,'peer_wait_strategy',peerWait,'peer',peerReport,'peer_exit_code',peerExitCode,'peer_forced_kill',peerKilled, ...
        'process_timer_requested',requestProcessTimer,'process_timer_begin',timerBegin,'process_timer_end',timerEnd, ...
        'process_timer_binary',timerBinary,'other_processes_changed',false, ...
        'command',command,'tracked',tracked,'before_sha256',hashesBefore,'after_sha256',hashesAfter, ...
        'COM',0,'board',0,'hardware_actions',0,'CopterSim_instances',0,'plant_runs',0,'controller_runs',0, ...
        'MATLAB_sockets',0,'MATLAB_timers',0,'fake_peer_processes_started',peerStarts, ...
        'native_transport_simulation_invocations',simInvocations,'native_transport_simulations',simCompleted);
    report.transport_proof_scope='HOST_ONLY_NO_COPTERSIM_UDP_COM_PROOF';
    report.CopterSim_UDP_COM_multiframe_acceptance_proven=false;
    if replyPairs
        report.frame_mapping_scope='DATAGRAM_BYTE_OFFSETS_AND_SHARED_QPC';
        report.retained006_frame_bytes_exact=baselineFrameExact;report.retained006_raw=retainedPackets;
    end
    report.gp_wire_mex_invoked=~compiled;
    if compiled
        report.gp_timing_scope='ORIGINAL_C_API_INSIDE_COMPILED_SFUNCTION__EXCLUDES_CODEC';
    else
        report.gp_timing_scope='ORIGINAL_GP_WIRE_MEX_CALL__INCLUDES_CODEC_AND_MEX_OVERHEAD';
    end
    write(fullfile(out,'RESULT.json'),jsonencode(report,PrettyPrint=true));
    fprintf('Native Raw + original GP %d/%d: queries=%d, GP calls=%d, peer roundtrip median %.3f ms. Retained replay only.\n', ...
        report.passed,report.total,ledger.queries_completed,ledger.gp_calls,report.roundtrip_median_ms);
    assert(report.all_pass,'gpenmpcNative:RawGpFunctionalFailed','Native GP causal replay check failed.');
catch ex
    if updateAttempted&&~updateComplete,updateElapsed=toc(updateClock);end
    clear clean
    gpCalls=[];completed=[];if simInvocations==0,gpCalls=0;completed=0;end
    if isfile(ledgerPath)
        failedLedger=load(ledgerPath,'ledger');gpCalls=failedLedger.ledger.gp_calls;completed=failedLedger.ledger.queries_completed;
    end
    failure=struct('failure_stage',stage,'exception',exceptionTree(ex), ...
        'host_only_update_attempted',updateAttempted,'host_only_update_completed',updateComplete,'host_only_update_elapsed_s',updateElapsed, ...
        'fake_peer_processes_started',peerStarts,'native_transport_simulation_invocations',simInvocations, ...
        'native_transport_simulations',simCompleted,'gp_calls_if_known',gpCalls,'queries_completed_if_known',completed,'ledger_path',ledgerPath, ...
        'peer_forced_kill',peerKilled,'peer_exit_code',peerExitCode,'COM',0,'board',0,'hardware_actions',0, ...
        'process_timer_requested',requestProcessTimer,'process_timer_begin',timerBegin,'process_timer_end',timerEnd, ...
        'compiled_hotpath',compiled,'compiled_binary',compiledBinary,'compiled_binary_sha256',compiledBinarySha, ...
        'reply_pair_datagrams_enabled',replyPairs,'transport_proof_scope','HOST_ONLY_NO_COPTERSIM_UDP_COM_PROOF', ...
        'compiled_base_ledger_saved',compiledLedgerSaved,'compiled_conversion_error',compiledConversionError, ...
        'compiled_capture_errors',{compiledCaptureErrors}, ...
        'CopterSim_instances',0,'plant_runs',0,'controller_runs',0,'MATLAB_sockets',0,'MATLAB_timers',0);
    write(fullfile(out,'FAILURE.json'),jsonencode(failure,PrettyPrint=true));rethrow(ex)
end
clear clean
    function check(n,p),checks(end+1)=struct('name',n,'pass',logical(p));if ~p,fprintf(2,'FAILED %s\n',n);end,end
    function closeModel(),if modelCreated&&bdIsLoaded(name),bdclose(name);end,modelCreated=false;end
    function stopPeer()
        if isempty(peer),return;end
        p=fullfile(out,'STOP.txt');if ~isfile(p),write(p,'GRACEFUL_HOST_GP_REPLAY_STOP');end
        if ~peer.HasExited&&~peer.WaitForExit(5000),peer.Kill();peerKilled=true;peer.WaitForExit(2000);end
        if peer.HasExited,peerExitCode=double(peer.ExitCode);end
        write(fullfile(out,'PEER_STDOUT.txt'),char(peer.StandardOutput.ReadToEnd()));
        write(fullfile(out,'PEER_STDERR.txt'),char(peer.StandardError.ReadToEnd()));peer.Dispose();peer=[];
    end
    function finish()
        try,closeModel();catch,end
        try,stopPeer();catch,end
        try,releaseTimer();catch,end
        try,captureCompiledLedger();catch captureEx,compiledCaptureErrors{end+1,1}=exceptionTree(captureEx);end
        cd(oldDir);path(oldPath);
    end
    function captureCompiledLedger()
        if ~compiled||compiledLedgerSaved||isempty(compiledVariable),return;end
        if ~evalin('base',sprintf('exist(''%s'',''var'')',compiledVariable)),return;end
        assert(~isfile(ledgerPath),'gpenmpcNative:RawGpCompiledLedgerExists','Never overwrite a raw compiled ledger.');
        compiledCore=evalin('base',compiledVariable);
        try
            ledger=normalize_rfly_compiled_gp_ledger(compiledCore);
        catch conversionEx
            % Preserve raw Core records and the original failure if normalization fails.
            compiledConversionError=exceptionTree(conversionEx);
            ledger=struct('gp_calls',double(compiledCore.gp_calls), ...
                'queries_completed',double(compiledCore.queries_completed), ...
                'compiled_core',compiledCore,'normalization_error',compiledConversionError);
        end
        save(ledgerPath,'ledger','compiledCore','compiledConversionError');
        compiledLedgerSaved=true;
        evalin('base',sprintf('clear %s',compiledVariable));
        if ~isempty(compiledConversionError)
            error('gpenmpcNative:RawGpCompiledNormalization','Compiled raw evidence saved; offline normalization failed.');
        end
    end
    function releaseTimer()
        if timerRequested
            timerEnd=host_process_timer_mex(0);
            if ~timerEnd.active,timerRequested=false;end
            assert(~timerEnd.active&&timerEnd.last_result==0,'Timer release failed and must remain disclosed.');
        end
    end
end
function frames=splitFrames(b)
frames=cell(0,1);p=1;
while p<=numel(b)
    assert(numel(b)-p+1>=12&&b(p)==253&&b(p+2)==0);n=double(b(p+1))+12;
    assert(n<=300&&p+n-1<=numel(b));frames{end+1,1}=b(p:p+n-1);p=p+n; %#ok<AGROW>
end
end
function [direction,packets,ticks]=readPeer(p)
f=fopen(p,'rb','ieee-le');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
direction=zeros(0,1,'uint8');packets=cell(0,1);ticks=zeros(0,1,'int64');
while true
    [d,c]=fread(f,1,'uint8=>uint8');if c==0,break;end,assert(d==0||d==1);
    [n,c]=fread(f,1,'uint32=>uint32');assert(c==1&&n>=1&&n<=300);
    [t,c]=fread(f,1,'int64=>int64');assert(c==1);[b,c]=fread(f,double(n),'*uint8');assert(c==double(n));
    direction(end+1,1)=d;ticks(end+1,1)=t;packets{end+1,1}=b; %#ok<AGROW>
end
end
function ok=portsAbsent()
n=System.Net.NetworkInformation.IPGlobalProperties.GetIPGlobalProperties();listeners=n.GetActiveUdpListeners();
ok=true;for k=1:listeners.Length,p=double(listeners(k).Port);ok=ok&&p~=62321&&p~=62322;end
end
function b=readbytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function h=sha(p),h=hashBytes(readbytes(p));end
function p=nearestRank95(values)
if isempty(values),p=NaN;return;end,values=sort(values);p=values(max(1,ceil(.95*numel(values))));
end
function h=hashBytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(:),'int8'));
h=string(upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[])));
end
function write(p,t)
assert(~isfile(p),'Preserve existing evidence file.');f=fopen(p,'w','n','UTF-8');assert(f>=0);
g=onCleanup(@()fclose(f));fprintf(f,'%s\n',t); %#ok<NASGU>
end
function tree=exceptionTree(ex)
causes=cell(size(ex.cause));for k=1:numel(ex.cause),causes{k}=exceptionTree(ex.cause{k});end
tree=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack, ...
    'report',getReport(ex,'extended','hyperlinks','off'),'causes',{causes});
end
