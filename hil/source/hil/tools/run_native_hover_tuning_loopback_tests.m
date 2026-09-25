function report=run_native_hover_tuning_loopback_tests(outputDir)
% Test tuning transfer with a localhost sys231/comp77 peer on ports 48181..48184.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:OutputExists');
build=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'));addpath(fullfile(build,'m600_coptersim','matlab_validation'));
[contract,v]=m600check.buildNativeHoverTuningContract();assert(v.passed,'%s',v.failure);
cfg=struct('local_mavlink_port',48181,'remote_mavlink_port',48182,'truth_port',48183, ...
    'coptersim_time_port',48184,'target_system',231,'target_component',77, ...
    'clock_max_rtt_s',2,'clock_sync_samples',3,'clock_sync_period_s',.05,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',.1,'clock_max_time_heartbeat_age_s',2, ...
    'clock_max_time_heartbeat_lag_s',.25,'maximum_truth_lag_s',2,'maximum_raw_records',10000, ...
    'state_max_age_s',.25,'heartbeat_max_age_s',3,'live_enabled',true,'outer_preflight_pass',true, ...
    'command_timeout_s',1.2,'poll_period_s',.01);
ports=48181:48184;assert(~any(ismember(ports,[14550 18570 20005 30101])));
io=[];peer=[];subscription=[];raw=[];unboundRaw=[];received={};caseMode='normal';setSeen=false;
peerError='';fixtureFailure='';stateGuardAllowed=true;stateGuardCalls=0;delayedReadObserved=false;
pool=containers.Map('KeyType','char','ValueType','char');
for k=1:3,pool(contract.entries(k).name)=contract.entries(k).original_raw_bits_hex;end
pool('CA_ROTOR0_PX')='00000000';sawEvidenceBeforeAck=0;
checks=struct('name',{},'passed',{},'error',{},'elapsed_s',{},'param_set_delta',{});
cleanup=struct('unbound_adapter_closed',false,'adapter_closed',false,'peer_closed',false, ...
    'ports_rebound',false,'errors',{{}});
guard=onCleanup(@closeOwned); %#ok<NASGU>
try
    verifyPortsFree();
    for variant={'unknown_contract_field','unknown_parameter','source_hash_changed'}
        bad=contract;
        switch variant{1}
            case 'unknown_contract_field',bad.safety_override=true;
            case 'unknown_parameter',bad.entries(1).name='FD_FAIL_R';
            case 'source_hash_changed',bad.bindings.current138.sha256=repmat('0',1,64);
        end
        badCfg=cfg;badCfg.native_hover_tuning=bad;errorId='';unexpected=[];
        try,unexpected=m600check.makeM600CopterSimIo(badCfg);catch p,errorId=p.identifier;end
        if ~isempty(unexpected),unexpected.close();end
        verifyPortsFree();
        note(['pre_socket_' variant{1}],strcmp(errorId,'m600check:NativeHoverTuningContract'),errorId,0,0);
    end
    dialect=mavlinkdialect('common.xml');peer=mavlinkio(dialect,'SystemID',231,'ComponentID',77);
    connect(peer,'UDP','LocalPort',cfg.remote_mavlink_port);client=mavlinkclient(peer,255,190);
    subscription=mavlinksub(peer,client,'BufferSize',500,'NewMessageFcn',@onPeer);
    io=m600check.makeM600CopterSimIo(cfg);errorId='';
    try,io.setNativeHoverTuningParameter('MC_ROLL_P','4026CCBE','40D00000',@()true);
    catch p,errorId=p.identifier;end
    unboundRaw=io.evidence();
    note('no_contract_zero_requests',strcmp(errorId,'m600check:NativeHoverTuningNotBound')&& ...
        isempty(unboundRaw.raw_transmit_messages)&&isempty(unboundRaw.real_parameter_actions),errorId,0,0);
    cleanup.unbound_adapter_closed=io.close();io=[];
    cfg.native_hover_tuning=contract;io=m600check.makeM600CopterSimIo(cfg);
    for phase={'APPLY','RESTORE'}
        phaseStart=countSet();
        for k=1:3
            e=contract.entries(k);if strcmp(phase{1},'APPLY'),target=e.target_raw_bits_hex;else,target=e.original_raw_bits_hex;end
            before=countSet();caseMode='normal';setSeen=false;t=tic;
            p=io.setNativeHoverTuningParameter(e.name,target,pool(e.name),@()true);
            note([phase{1} '_' e.name],p.mav_type==9&&strcmp(p.raw_bits_hex,target)&& ...
                p.real_write_receipt.attempted&&p.real_write_receipt.send_returned&& ...
                p.real_write_receipt.ack_verified&&p.real_write_receipt.readback_verified&& ...
                p.real_write_receipt.before_send_guard_passed&&countSet()-before==1,'',toc(t),countSet()-before);
        end
        note([phase{1} '_exact_three_SETs'],countSet()-phaseStart==3,'',0,countSet()-phaseStart);
    end
    for k=1:3
        e=contract.entries(k);before=numTx();errorId='';
        try,io.setRealParameter(e.name,e.target_raw_bits_hex,e.original_raw_bits_hex,@()true);catch p,errorId=p.identifier;end
        note(['old_geometry_API_rejects_' e.name],strcmp(errorId,'m600check:RealParameterName')&&numTx()==before,errorId,0,0);
    end
    % Demonstrate the original geometry service remains separately callable.
    for geometryBitsCell={'3E000000','00000000'}
        caseMode='normal';setSeen=false;before=countSet();
        p=io.setRealParameter('CA_ROTOR0_PX',geometryBitsCell{1},pool('CA_ROTOR0_PX'),@()true);
        note(['geometry_API_still_separate_' geometryBitsCell{1}],strcmp(p.raw_bits_hex,geometryBitsCell{1})&&countSet()-before==1,'',0,countSet()-before);
    end
    for invalidNameCell={'CA_ROTOR0_PX','HIL_ACT_FUNC1','MPC_Z_VEL_I_ACC','FD_FAIL_R','_HASH_CHECK'}
        negative(['new_API_rejects_' invalidNameCell{1}],invalidNameCell{1},'4026CCBE','40D00000',@()true,'normal', ...
            'm600check:NativeHoverTuningName',0,true);
    end
    for invalidBitsCell={'123','GGGGGGGG','7FC00000','7F800000','4026CCBF'}
        negative(['unknown_target_pair_' invalidBitsCell{1}],'MC_ROLL_P',invalidBitsCell{1},'40D00000',@()true,'normal', ...
            'm600check:NativeHoverTuningOutsideExactPair',0,true);
    end
    negative('unknown_expected_pair','MC_ROLL_P','4026CCBE','40000000',@()true,'normal', ...
        'm600check:NativeHoverTuningOutsideExactPair',0,true);
    negative('non_callback_zero_requests','MC_ROLL_P','4026CCBE','40D00000',true,'normal','m600check:RealParameterGuard',0,true);
    negative('wrong_original_type','MC_ROLL_P','4026CCBE','40D00000',@()true,'before_type','m600check:RealParameterOriginalType',0,false);
    negative('nonfinite_original','MC_ROLL_P','4026CCBE','40D00000',@()true,'before_nan','m600check:RealParameterOriginalType',0,false);
    stateGuardCalls=0;
    negative('last_current_bits_mismatch','MC_ROLL_P','4026CCBE','4026CCBE',@stateGuard,'normal', ...
        'm600check:RealParameterCurrentMismatch',0,false);
    note('current_mismatch_before_guard',stateGuardCalls==0,'',0,0);
    stateGuardAllowed=true;stateGuardCalls=0;delayedReadObserved=false;
    negative('internal_read_then_unsafe_zero_SET','MC_ROLL_P','4026CCBE','40D00000',@stateGuard, ...
        'delayed_before_guard_false','m600check:RealParameterUnsafeWriteGuard',0,false);
    q=io.evidence();a=q.real_parameter_actions(end);
    note('TOCTOU_guard_after_delayed_read',delayedReadObserved&&stateGuardCalls==1&&~stateGuardAllowed&& ...
        a.before_send_guard_invoked&&~a.before_send_guard_passed&&~a.attempted,'',0,0);
    negative('false_guard_zero_SET','MC_ROLL_P','4026CCBE','40D00000',@()false,'normal','m600check:RealParameterUnsafeWriteGuard',0,false);
    negative('numeric_guard_zero_SET','MC_ROLL_P','4026CCBE','40D00000',@()1,'normal','m600check:RealParameterUnsafeWriteGuard',0,false);
    negative('throwing_guard_zero_SET','MC_ROLL_P','4026CCBE','40D00000',@throwGuard,'normal','fixture:SafetyGuard',0,false);
    for pair={{'ack_type','m600check:RealParameterACK'},{'ack_bits','m600check:RealParameterACK'}, ...
            {'readback_type','m600check:RealParameterReadback'},{'readback_bits','m600check:RealParameterReadback'}, ...
            {'missing_ack','m600check:ParameterTimeout'},{'missing_readback','m600check:ParameterTimeout'}}
        mode=pair{1};negative(mode{1},'MC_ROLL_P','4026CCBE','40D00000',@()true,mode{1},mode{2},1,false);
    end
    before=numTx();errorId='';
    try,io.setNativeHoverTuningParameter('MC_ROLL_P','4026CCBE');catch p,errorId=p.identifier;end
    note('two_argument_API_rejected_zero_requests',strcmp(errorId,'m600check:NativeHoverTuningNotBound')&&numTx()==before,errorId,0,0);
    raw=io.evidence();actions=raw.real_parameter_actions;
    note('wire_attempt_counts_exact',nnz([actions.attempted])==countSet(),'',0,0);
    note('six_failed_sent_transactions_retained',nnz([actions.attempted]&~[actions.readback_verified])==6,'',0,0);
    note('wire_evidence_visible_before_every_peer_ACK',sawEvidenceBeforeAck==countSet(),'',0,0);
    ids=cellfun(@(r)double(r.message.MsgID),raw.raw_transmit_messages);
    note('only_heartbeat_read_PARAM_SET_no_control_commands',all(ismember(ids,[0 20 23])),'',0,0);
    note('all_SET_names_inside_two_disjoint_services',all(ismember({actions([actions.attempted]).name}, ...
        {'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER','CA_ROTOR0_PX'})),'',0,0);
    note('peer_callback_error_free',isempty(peerError),peerError,0,0);
catch problem,fixtureFailure=getReport(problem,'extended','hyperlinks','off');end
if isempty(raw)&&~isempty(io),raw=io.evidence();end
closeOwned();delete(guard);
note('all_owned_endpoints_closed_rebound',cleanup.unbound_adapter_closed&&cleanup.adapter_closed&& ...
    cleanup.peer_closed&&cleanup.ports_rebound&&isempty(cleanup.errors),'',0,0);
report=struct('classification','HOST_SYNTHETIC_THREE_PARAMETER_API', ...
    'passed',isempty(fixtureFailure)&&all([checks.passed]),'checks_total',numel(checks), ...
    'checks_passed',nnz([checks.passed]),'checks',checks,'failure',fixtureFailure,'peer_error',peerError, ...
    'config',cfg,'cleanup',cleanup,'adapter_sha256',m600check.fileSha256(which('m600check.makeM600CopterSimIo')), ...
    'fixture_sha256',m600check.fileSha256([mfilename('fullpath') '.m']), ...
    'hardware_actions',0,'COM_open',0,'PX4_access',0,'real_parameter_writes',0, ...
    'synthetic_PARAM_SET_count',countSet(),'actual_target_system',231,'actual_target_component',77);
mkdir(outputDir);save(fullfile(outputDir,'RAW.mat'),'report','raw','unboundRaw','received','-v7');
f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
assert(report.passed,'m600check:TuningLoopbackFailed','See retained raw/report.');
    function n=countSet()
        if isempty(io),n=0;return;end
        countEvidence=io.evidence();n=nnz(cellfun(@(r)double(r.message.MsgID)==23,countEvidence.raw_transmit_messages));
    end
    function n=numTx(),txEvidence=io.evidence();n=numel(txEvidence.raw_transmit_messages);end
    function negative(negLabel,negName,negBits,negCurrent,negGuard,negMode,negExpectedError,negExpectedSets,negZeroRequest)
        if isKey(pool,negName)&&~strcmp(negName,'CA_ROTOR0_PX'),pool(negName)='40D00000';end
        caseMode=negMode;setSeen=false;negBefore=countSet();negTxBefore=numTx();negErrorId='';negTimer=tic;
        try,io.setNativeHoverTuningParameter(negName,negBits,negCurrent,negGuard);catch negProblem,negErrorId=negProblem.identifier;end
        negElapsed=toc(negTimer);negDelta=countSet()-negBefore;
        negOkay=strcmp(negErrorId,negExpectedError)&&negDelta==negExpectedSets;
        if negZeroRequest,negOkay=negOkay&&numTx()==negTxBefore;end
        if startsWith(negMode,'missing_'),negOkay=negOkay&&negElapsed>=cfg.command_timeout_s&&negElapsed<cfg.command_timeout_s+.6;end
        note(negLabel,negOkay,negErrorId,negElapsed,negDelta);
    end
    function okay=stateGuard(),stateGuardCalls=stateGuardCalls+1;okay=stateGuardAllowed;end
    function okay=throwGuard() %#ok<STOUT>
        error('fixture:SafetyGuard','Synthetic final safety observation failure');
    end
    function note(name,pass,error,elapsed,delta)
        checks(end+1)=struct('name',name,'passed',logical(pass),'error',error,'elapsed_s',elapsed,'param_set_delta',delta); %#ok<AGROW>
    end
    function verifyPortsFree()
        for port=ports,u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',port);delete(u);end
    end
    function onPeer(~,messages)
        % All callback temporaries are explicitly distinct from parent loop
        % variables. Nested MATLAB functions otherwise share matching names.
        try
            for peerIndex=1:numel(messages)
                peerMessage=messages(peerIndex);received{end+1}=peerMessage; %#ok<AGROW>
                if double(peerMessage.MsgID)==0,continue;end
                assert(ismember(double(peerMessage.MsgID),[20 23]));peerName=strtrim(strrep(char(peerMessage.Payload.param_id),char(0),''));
                assert(isKey(pool,peerName));peerBits=pool(peerName);peerType=uint8(9);
                if double(peerMessage.MsgID)==23
                    assert(double(peerMessage.Payload.param_type)==9);
                    peerBits=upper(dec2hex(typecast(single(peerMessage.Payload.param_value),'uint32'),8));pool(peerName)=peerBits;setSeen=true;
                    peerEvidence=io.evidence();peerIsSet=cellfun(@(x)double(x.message.MsgID)==23,peerEvidence.raw_transmit_messages);
                    peerLast=peerEvidence.raw_transmit_messages{find(peerIsSet,1,'last')}.message;
                    assert(strcmp(strtrim(strrep(char(peerLast.Payload.param_id),char(0),'')),peerName)&& ...
                        double(peerLast.Payload.param_type)==9&& ...
                        isequal(typecast(single(peerLast.Payload.param_value),'uint32'),typecast(single(peerMessage.Payload.param_value),'uint32'))&& ...
                        peerEvidence.real_parameter_actions(end).attempted);
                    sawEvidenceBeforeAck=sawEvidenceBeforeAck+1;
                    if strcmp(caseMode,'missing_ack'),continue;end
                    if strcmp(caseMode,'ack_type'),peerType=uint8(6);end
                    if strcmp(caseMode,'ack_bits'),peerBits='40D00000';end
                else
                    if ~setSeen&&strcmp(caseMode,'delayed_before_guard_false')
                        pause(.15);stateGuardAllowed=false;delayedReadObserved=true;
                    end
                    if ~setSeen&&strcmp(caseMode,'before_type'),peerType=uint8(6);end
                    if ~setSeen&&strcmp(caseMode,'before_nan'),peerBits='7FC00000';end
                    if setSeen&&strcmp(caseMode,'missing_readback'),continue;end
                    if setSeen&&strcmp(caseMode,'readback_type'),peerType=uint8(6);end
                    if setSeen&&strcmp(caseMode,'readback_bits'),peerBits='40D00000';end
                end
                peerReply=createmsg(dialect,'PARAM_VALUE');peerReply.Payload.param_id=text16(peerName);peerReply.Payload.param_type=peerType;
                peerReply.Payload.param_value=typecast(uint32(hex2dec(peerBits)),'single');peerReply.Payload.param_count=uint16(4);peerReply.Payload.param_index=uint16(0);
                sendudpmsg(peer,peerReply,'127.0.0.1',cfg.local_mavlink_port);
            end
        catch peerProblem,peerError=getReport(peerProblem,'extended','hyperlinks','off');end
    end
    function closeOwned()
        if ~isempty(io)&&~cleanup.adapter_closed
            try,cleanup.adapter_closed=io.close();catch closeProblem,cleanup.errors{end+1}=closeProblem.message;end
        end
        if ~isempty(subscription)
            try,subscription.NewMessageFcn=[];delete(subscription);subscription=[];catch closeProblem,cleanup.errors{end+1}=closeProblem.message;end
        end
        if ~isempty(peer)&&~cleanup.peer_closed
            try,disconnect(peer);delete(peer);cleanup.peer_closed=true;catch closeProblem,cleanup.errors{end+1}=closeProblem.message;end
        end
        try,verifyPortsFree();cleanup.ports_rebound=true;catch closeProblem,cleanup.errors{end+1}=closeProblem.message;end
    end
end
function s=text16(name),s=repmat(char(0),1,16);s(1:numel(name))=name;end
