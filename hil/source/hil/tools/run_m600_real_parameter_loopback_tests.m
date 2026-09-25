function report=run_m600_real_parameter_loopback_tests(outputDir)
% Test parameter serialization against a synthetic localhost sys231/comp77 peer.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir));mkdir(outputDir);
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'matlab_validation'));addpath(fullfile(build,'m600_coptersim','matlab_validation'));
cfg=struct('local_mavlink_port',48171,'remote_mavlink_port',48172,'truth_port',48173, ...
    'coptersim_time_port',48174,'target_system',231,'target_component',77, ...
    'clock_max_rtt_s',2,'clock_sync_samples',3,'clock_sync_period_s',.05,'clock_max_age_s',30, ...
    'clock_max_uncertainty_s',1.1,'clock_max_utc_drift_s',.1,'clock_max_time_heartbeat_age_s',2, ...
    'clock_max_time_heartbeat_lag_s',.25,'maximum_truth_lag_s',2,'maximum_raw_records',10000, ...
    'state_max_age_s',.25,'heartbeat_max_age_s',3,'live_enabled',true,'outer_preflight_pass',true, ...
    'command_timeout_s',1.2,'poll_period_s',.01);
ports=48171:48174;assert(~any(ismember(ports,[14550 18570 20005 30101])));
io=[];peer=[];subscription=[];raw=[];received={};peerError='';caseMode='normal';setSeen=false;
stateGuardAllowed=true;stateGuardCalls=0;delayedReadObserved=false;
checks=struct('name',{},'passed',{},'error',{},'elapsed_s',{},'param_set_delta',{});
cleanup=struct('adapter_closed',false,'peer_closed',false,'ports_rebound',false,'errors',{{}});
pool=containers.Map('KeyType','char','ValueType','char');names={};
for i=0:5,for axis={'X','Y'},name=sprintf('CA_ROTOR%d_P%s',i,axis{1});names{end+1}=name;pool(name)='3F000000';end;end
sawEvidenceBeforeAck=0;fixtureFailure='';guard=onCleanup(@closeOwned); %#ok<NASGU>
try
    for port=ports,u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',port);delete(u);end
    dialect=mavlinkdialect('common.xml');peer=mavlinkio(dialect,'SystemID',231,'ComponentID',77);
    connect(peer,'UDP','LocalPort',48172);client=mavlinkclient(peer,255,190);
    subscription=mavlinksub(peer,client,'BufferSize',500,'NewMessageFcn',@onPeer);
    io=m600check.makeM600CopterSimIo(cfg);
    for k=1:numel(names)
        target=upper(dec2hex(typecast(single((-1)^k*(.125+k/64)),'uint32'),8));
        before=countSet();setSeen=false;caseMode='normal';ticCase=tic;
        p=io.setRealParameter(names{k},target,pool(names{k}),@()true);
        note(['exact_REAL32_' names{k}],p.mav_type==9&&strcmp(p.raw_bits_hex,target)&& ...
            p.real_write_receipt.attempted&&p.real_write_receipt.ack_verified&& ...
            p.real_write_receipt.before_send_guard_passed&&p.real_write_receipt.readback_verified&& ...
            countSet()-before==1,'',toc(ticCase),countSet()-before);
    end
    for bad={'SYS_HITL','CA_ROTOR6_PX','CA_ROTOR0_PZ','CA_ROTOR00_PX','HIL_ACT_FUNC1'}
        runNegative(['name_rejected_' bad{1}],bad{1},'3E800000','normal','m600check:RealParameterName',0,true);
    end
    for bad={'123','GGGGGGGG','7F800000','FF800000','7FC00000'}
        expected='m600check:RealParameterBits';if numel(bad{1})==8&&~strcmp(bad{1},'GGGGGGGG'),expected='m600check:RealParameterNonfinite';end
        runNegative(['bits_rejected_' bad{1}],'CA_ROTOR0_PX',bad{1},'normal',expected,0,true);
    end
    runNegative('original_wrong_type','CA_ROTOR0_PX','3E800000','before_type','m600check:RealParameterOriginalType',0,false);
    runNegative('original_nonfinite','CA_ROTOR0_PX','3E800000','before_nan','m600check:RealParameterOriginalType',0,false);
    runNegative('ack_wrong_type','CA_ROTOR0_PX','3E800000','ack_type','m600check:RealParameterACK',1,false);
    runNegative('ack_wrong_bits','CA_ROTOR0_PX','3E800000','ack_bits','m600check:RealParameterACK',1,false);
    runNegative('readback_wrong_type','CA_ROTOR0_PX','3E800000','readback_type','m600check:RealParameterReadback',1,false);
    runNegative('readback_wrong_bits','CA_ROTOR0_PX','3E800000','readback_bits','m600check:RealParameterReadback',1,false);
    runNegative('missing_ack_never_resends_PARAM_SET','CA_ROTOR0_PX','3E800000','missing_ack','m600check:ParameterTimeout',1,false);
    runNegative('missing_readback_only_retries_READ','CA_ROTOR0_PX','3E800000','missing_readback','m600check:ParameterTimeout',1,false);
    wrong=upper(dec2hex(bitxor(uint32(hex2dec(pool('CA_ROTOR0_PX'))),uint32(1)),8));
    stateGuardCalls=0;
    runNegative('last_internal_current_bits_mismatch_zero_SET','CA_ROTOR0_PX','3E800000','normal', ...
        'm600check:RealParameterCurrentMismatch',0,false,wrong,@stateGuard);
    note('current_mismatch_rejects_before_safety_callback',stateGuardCalls==0,'',0,0);
    stateGuardAllowed=true;stateGuardCalls=0;delayedReadObserved=false;
    runNegative('delayed_read_then_changed_state_zero_SET','CA_ROTOR0_PX','3E800000','delayed_before_guard_false', ...
        'm600check:RealParameterUnsafeWriteGuard',0,false,pool('CA_ROTOR0_PX'),@stateGuard);
    note('delayed_read_completed_before_final_guard',delayedReadObserved&&stateGuardCalls==1&&~stateGuardAllowed,'',0,0);
    ev=io.evidence();lastAction=ev.real_parameter_actions(end);
    note('unsafe_guard_retains_unattempted_receipt',lastAction.before_send_guard_invoked&& ...
        ~lastAction.before_send_guard_passed&&~lastAction.attempted&&~isempty(lastAction.error),'',0,0);
    runNegative('guard_exception_zero_SET','CA_ROTOR0_PX','3E800000','normal', ...
        'm600check:FixtureGuardError',0,false,pool('CA_ROTOR0_PX'),@throwGuard);
    runNegative('guard_numeric_one_not_logical_true','CA_ROTOR0_PX','3E800000','normal', ...
        'm600check:RealParameterUnsafeWriteGuard',0,false,pool('CA_ROTOR0_PX'),@()1);
    runNegative('guard_not_callback_zero_requests','CA_ROTOR0_PX','3E800000','normal', ...
        'm600check:RealParameterGuard',0,true,pool('CA_ROTOR0_PX'),true);
    for bad={'123','GGGGGGGG','7F800000','FF800000','7FC00000'}
        expected='m600check:RealParameterExpectedBits';
        if numel(bad{1})==8&&~strcmp(bad{1},'GGGGGGGG'),expected='m600check:RealParameterExpectedNonfinite';end
        runNegative(['expected_current_reject_' bad{1}],'CA_ROTOR0_PX','3E800000','normal',expected,0,true,bad{1},@()true);
    end
    ev=io.evidence();txBefore=numel(ev.raw_transmit_messages);problemId='';
    try,io.setRealParameter('CA_ROTOR0_PX','3E800000');catch problem,problemId=problem.identifier;end
    ev=io.evidence();note('old_two_argument_API_fail_closed_zero_requests', ...
        strcmp(problemId,'m600check:RealParameterContract')&&numel(ev.raw_transmit_messages)==txBefore,problemId,0,0);
    raw=io.evidence();actions=raw.real_parameter_actions;
    note('attempt_receipts_match_all_exact_raw_PARAM_SETs',nnz([actions.attempted])==countSet(),'',0,0);
    note('failures_preserve_attempted_actions',nnz([actions.attempted]&~[actions.readback_verified])==6,'',0,0);
    note('exact_wire_evidence_visible_before_peer_ACK',sawEvidenceBeforeAck==countSet(),'',0,0);
    ids=cellfun(@(r)double(r.message.MsgID),raw.raw_transmit_messages);
    note('outbound_only_HEARTBEAT_READ_and_twelve_REAL32_fields',all(ismember(ids,[0 20 23])),'',0,0);
    note('peer_callbacks_without_error',isempty(peerError),peerError,0,0);
catch problem,fixtureFailure=getReport(problem,'extended','hyperlinks','off');end
closeOwned();delete(guard);
if isempty(raw)&&~isempty(io),raw=io.evidence();end
note('all_owned_local_endpoints_closed_rebound',cleanup.adapter_closed&&cleanup.peer_closed&&cleanup.ports_rebound,'',0,0);
report=struct('classification','HOST_SYNTHETIC_REAL32_PARAMETER_API', ...
    'passed',isempty(fixtureFailure)&&all([checks.passed]),'checks_total',numel(checks), ...
    'checks_passed',nnz([checks.passed]),'checks',checks,'failure',fixtureFailure,'peer_error',peerError, ...
    'config',cfg,'live_bounds_unchanged_by_fixture',true,'cleanup',cleanup, ...
    'adapter_sha256',sha(which('m600check.makeM600CopterSimIo')), ...
    'fixture_sha256',sha(mfilename('fullpath')+".m"), ...
    'hardware_actions',0,'COM_open',0,'PX4_access',0,'real_parameter_writes',0, ...
    'synthetic_PARAM_SET_count',countSet());
save(fullfile(outputDir,'REAL_PARAMETER_API_RAW.mat'),'report','raw','received','-v7');
fid=fopen(fullfile(outputDir,'REAL_PARAMETER_API_RESULT.json'),'w');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(report,PrettyPrint=true),'char');
assert(report.passed,'m600check:RealParameterFixtureFailed','See retained raw/report.');
    function n=countSet()
        if isempty(io),n=0;return;end
        q=io.evidence();n=nnz(cellfun(@(r)double(r.message.MsgID)==23,q.raw_transmit_messages));
    end
    function runNegative(label,name,bits,mode,expected,writeDelta,noRequest,currentBits,beforeGuard)
        if nargin<8,if isKey(pool,name),currentBits=pool(name);else,currentBits='3F000000';end;end
        if nargin<9,beforeGuard=@()true;end
        caseMode=mode;setSeen=false;before=countSet();v=io.evidence();txBefore=numel(v.raw_transmit_messages);
        problemId='';t=tic;
        try,io.setRealParameter(name,bits,currentBits,beforeGuard);catch problem,problemId=problem.identifier;end
        elapsed=toc(t);v=io.evidence();actual=countSet()-before;
        ok=strcmp(problemId,expected)&&actual==writeDelta;
        if noRequest,ok=ok&&numel(v.raw_transmit_messages)==txBefore;end
        if startsWith(mode,'missing_'),ok=ok&&elapsed>=cfg.command_timeout_s&&elapsed<cfg.command_timeout_s+.6;end
        note(label,ok,problemId,elapsed,actual);
    end
    function okay=stateGuard()
        stateGuardCalls=stateGuardCalls+1;okay=stateGuardAllowed;
    end
    function okay=throwGuard() %#ok<STOUT>
        error('m600check:FixtureGuardError','Synthetic final safety callback failed.');
    end
    function note(name,ok,error,elapsed,delta)
        checks(end+1)=struct('name',name,'passed',logical(ok),'error',error,'elapsed_s',elapsed,'param_set_delta',delta);
    end
    function onPeer(~,messages)
        try
            for j=1:numel(messages)
                m=messages(j);received{end+1}=m; %#ok<AGROW>
                if double(m.MsgID)==0,continue;end
                assert(ismember(double(m.MsgID),[20 23]));name=strtrim(strrep(char(m.Payload.param_id),char(0),''));
                assert(isKey(pool,name));bits=pool(name);type=uint8(9);
                if double(m.MsgID)==23
                    assert(double(m.Payload.param_type)==9);
                    actual=upper(dec2hex(typecast(single(m.Payload.param_value),'uint32'),8));pool(name)=actual;bits=actual;setSeen=true;
                    ev=io.evidence();isSet=cellfun(@(q)double(q.message.MsgID)==23,ev.raw_transmit_messages);
                    last=ev.raw_transmit_messages{find(isSet,1,'last')}.message;
                    % Compare parameter names and raw numeric bits independently of character padding.
                    lastName=strtrim(strrep(char(last.Payload.param_id),char(0),''));
                    assert(strcmp(lastName,name)&&double(last.Payload.param_type)==9&& ...
                        isequal(typecast(single(last.Payload.param_value),'uint32'),typecast(single(m.Payload.param_value),'uint32'))&& ...
                        ev.real_parameter_actions(end).attempted);
                    sawEvidenceBeforeAck=sawEvidenceBeforeAck+1;
                    if strcmp(caseMode,'missing_ack'),continue;end
                    if strcmp(caseMode,'ack_type'),type=uint8(6);end
                    if strcmp(caseMode,'ack_bits'),bits='3F000000';end
                else
                    if ~setSeen&&strcmp(caseMode,'delayed_before_guard_false')
                        pause(.15);stateGuardAllowed=false;delayedReadObserved=true;
                    end
                    if ~setSeen&&strcmp(caseMode,'before_type'),type=uint8(6);end
                    if ~setSeen&&strcmp(caseMode,'before_nan'),bits='7FC00000';end
                    if setSeen&&strcmp(caseMode,'missing_readback'),continue;end
                    if setSeen&&strcmp(caseMode,'readback_type'),type=uint8(6);end
                    if setSeen&&strcmp(caseMode,'readback_bits'),bits='3F000000';end
                end
                a=createmsg(dialect,'PARAM_VALUE');a.Payload.param_id=text16(name);a.Payload.param_type=type;
                a.Payload.param_value=typecast(uint32(hex2dec(bits)),'single');a.Payload.param_count=uint16(12);a.Payload.param_index=uint16(0);
                sendudpmsg(peer,a,'127.0.0.1',48171);
            end
        catch problem,peerError=getReport(problem,'extended','hyperlinks','off');end
    end
    function closeOwned()
        if ~isempty(io)&&~cleanup.adapter_closed,try,cleanup.adapter_closed=io.close();catch problem,cleanup.errors{end+1}=problem.message;end;end
        if ~isempty(subscription),try,subscription.NewMessageFcn=[];delete(subscription);subscription=[];catch problem,cleanup.errors{end+1}=problem.message;end;end
        if ~isempty(peer)&&~cleanup.peer_closed,try,disconnect(peer);delete(peer);cleanup.peer_closed=true;catch problem,cleanup.errors{end+1}=problem.message;end;end
        try,for port=ports,u=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',port);delete(u);end;cleanup.ports_rebound=true;
        catch problem,cleanup.errors{end+1}=problem.message;end
    end
end
function s=text16(name),s=repmat(char(0),1,16);s(1:numel(name))=name;end
function h=sha(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(fread(fid,Inf,'*uint8'));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
