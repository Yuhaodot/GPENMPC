function report=test_rfly_local_session(outputRoot,mode)
% Test session entry and receipt parsing with SERIAL_CONTROL fixtures.
arguments
    outputRoot (1,1) string
    mode (1,1) string = "ALL"
end
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'),'-begin');
assert(ismember(mode,["ALL","UNREGISTERED_FIXTURE_ONLY","RETIRED_FIXTURE_ONLY"]),'Unknown local session test selection.');
assert(~isfolder(outputRoot));mkdir(outputRoot);d=mavlinkdialect('common.xml',2);
if mode~="ALL"
    fixtureKind="unregistered";if mode=="RETIRED_FIXTURE_ONLY",fixtureKind="retired";end
    retained=unregisteredRetained(build,fixtureKind);
    report=struct('passed',retained.passed,'test_count',1, ...
        'checks',struct('name',char(fixtureKind+"_fixture_reports_primary_board_state"),'pass',retained.passed), ...
        'scope','HOST_ONLY_SERIAL_CONTROL_FIXTURE','fixture_kind',fixtureKind, ...
        'hardware_actions',0,'network_actions',0,'matlab_io_connections',0,'association_created',false);
    save(fullfile(outputRoot,'RAW.mat'),'retained','report');
    f=fopen(fullfile(outputRoot,'RESULT.json'),'wt');c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
    disp(jsonencode(report));return
end
f=fopen(fullfile(gpenmpc_external_path('board_commit_exchange_fixture'),'ACTUAL_FIXTURE_ECHO119.bin'),'rb');
assert(f>=0);e=fread(f,Inf,'*uint8');fclose(f);assert(numel(e)==119);
q=struct('original_host_challenge',u64(e(6:21)),'uid',u64(e(22:29)), ...
    'system',e(30),'component',e(31),'host_system',uint8(42),'host_component',uint8(191), ...
    'configuration_payload_sha256',hex(e(56:87)),'local_full_inner',true,'leg_index',uint8(1), ...
    'task_sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
    'original_prepare_submit_ns',uint64(1000000),'original_confirm_submit_ns',uint64(2000000), ...
    'original_confirm_session_sha256',hex(e(88:119)),'confirmed_physical_setup_record_sha256',repmat('A',1,64));
auth=struct('source','OperatorUsbIsolationDeclaration', ...
    'physical_setup_record_sha256',q.confirmed_physical_setup_record_sha256, ...
    'original_record','SYNTHETIC_HOST_FIXTURE_NOT_LIVE_AUTHORIZATION');
base=sprintf(['RFLY_LOCAL_SESSION state=2 challenge=%s uid=%u system=%u component=%u registration_hrt_us=%u ' ...
    'session_generation=%u link_generation=%u semantics=1 config_sha=%s session_sha=%s task_sha=%s leg=1 ' ...
    'registered=1 echo_confirmed=0 declared_isolation=0 declaration_is_sensor_proof=0 session_fault=0 start_requests=0 stop_requests=0\n'], ...
    hex(e(6:21)),q.uid,q.system,q.component,u64(e(32:39)),u64(e(40:47)),u64(e(48:55)), ...
    q.configuration_payload_sha256,q.original_confirm_session_sha256,q.task_sha256);
confirmed=strrep(strrep(strrep(base,'state=2','state=3'),'echo_confirmed=0','echo_confirmed=1'),'declared_isolation=0','declared_isolation=1');
pre=frames(base,q.original_prepare_submit_ns+uint64(1));con=frames(confirmed,q.original_confirm_submit_ns+uint64(1));
p=gpenmpcNative.RflySessionAssociationDecoder(pre,[],q,[],d);
a=gpenmpcNative.RflySessionAssociationDecoder(pre,con,q,auth,d);
checks=struct('name',{},'pass',{});
check('actual_new_state_number_and_leg_parse',p.local_full_inner&&a.local_full_inner ...
    &&p.prepare_receipt.parsed_fields.state==2&&a.confirm_receipt.parsed_fields.state==3 ...
    &&a.confirm_receipt.parsed_fields.leg==1&&strcmp(a.execution_session_sha256,q.original_confirm_session_sha256));
check('raw_callback_preserved_no_parameter_echo_invented',isequal(a.prepare_receipt.original_frames,pre) ...
    &&isempty(a.approved_parameter_sha256)&&~a.parameter_identity_board_echoed ...
    &&~a.publication_authority&&~a.physical_authorization);
v=struct('challenge',q.original_host_challenge,'origin_ned_m',[0;0;0], ...
    'host_system',q.host_system,'host_component',q.host_component,'leg_index',uint8(1));
target=struct('system',q.system,'component',q.component);
[ms,r]=gpenmpcNative.RflySessionCommandEncoder('prepare_local',v,d,target);
check('exact_new_prepare_nine_arguments',endsWith(r.original_command,' 42 191 1') ...
    &&numel(strsplit(r.original_command))==10&&~r.io_sent&&numel(ms)==r.fragments);
[~,r]=gpenmpcNative.RflySessionCommandEncoder('stream_local',struct('enabled',true),d,target);
check('exact_actual_same_link_local_stream',strcmp(r.original_command,'mavlink stream -d /dev/ttyACM0 -s GPENMPC_LOCAL_WIRE -r -1'));
[~,r]=gpenmpcNative.RflySessionCommandEncoder('evidence_local',struct(),d,target);
check('exact_local_post_release_evidence',strcmp(r.original_command,'gpenmpc_rfly_session evidence'));
bad=v;bad=rmfield(bad,'leg_index');reject('no_implicit_leg',@()gpenmpcNative.RflySessionCommandEncoder('prepare_local',bad,d,target));
bad=v;bad.leg_index=uint8(6);reject('no_out_of_task_leg',@()gpenmpcNative.RflySessionCommandEncoder('prepare_local',bad,d,target));
reject('old_session_receipt_cannot_bind_local',@()gpenmpcNative.RflySessionAssociationDecoder(frames(strrep(base,'RFLY_LOCAL_SESSION','RFLY_SESSION'),uint64(1000001)),con,q,auth,d));
reject('registered_but_inputs_not_bound',@()gpenmpcNative.RflySessionAssociationDecoder(frames(strrep(base,'state=2','state=1'),uint64(1000001)),con,q,auth,d));
reject('duplicate_local_session',@()gpenmpcNative.RflySessionAssociationDecoder(pre,frames([confirmed confirmed],uint64(2000001)),q,auth,d));
badq=q;badq.leg_index=uint8(2);reject('wrong_requested_leg',@()gpenmpcNative.RflySessionAssociationDecoder(pre,con,badq,auth,d));
badq=q;badq.task_sha256=repmat('C',1,64);reject('wrong_requested_task',@()gpenmpcNative.RflySessionAssociationDecoder(pre,con,badq,auth,d));
badq=q;badq.original_confirm_session_sha256=repmat('F',1,64);reject('stale_confirm_digest',@()gpenmpcNative.RflySessionAssociationDecoder(pre,con,badq,auth,d));
reject('changed_confirm_leg',@()gpenmpcNative.RflySessionAssociationDecoder(pre,frames(strrep(confirmed,'leg=1','leg=2'),uint64(2000001)),q,auth,d));
retired=strrep(strrep(strrep(confirmed,'state=3','state=5'),'start_requests=0','start_requests=1'),'stop_requests=0','stop_requests=1');
release='RFLY_LOCAL_RELEASE result=4 quiescent=1 disarmed=1 virtual_zero_stream_accepted=1 plant_cache_zero_proven=0 publication_attempts=3 publication_successes=3';
text=[retired sprintf('%s\n',release)];
rq=struct('registered_association',a,'original_stop_submit_ns',uint64(3000000),'original_release_submit_ns',uint64(4000000));
rr=gpenmpcNative.RflySessionReleaseDecoder(frames(text,uint64(4000001)),rq,d);
check('local_retired_quiescent_state_and_count',rr.local_full_inner&&rr.parsed_session.state==5 ...
    &&rr.parsed_release.publication_successes==3&&rr.board_context_detached_reported ...
    &&rr.requires_independent_plant_cache_zero&&~rr.plant_cache_zero_proven);
reject('retire_requires_task_quiescence',@()gpenmpcNative.RflySessionReleaseDecoder(frames(strrep(text,'quiescent=1','quiescent=0'),uint64(4000001)),rq,d));
reject('retire_requires_disarmed',@()gpenmpcNative.RflySessionReleaseDecoder(frames(strrep(text,'disarmed=1','disarmed=0'),uint64(4000001)),rq,d));
reject('old_retired_number_not_local_retired',@()gpenmpcNative.RflySessionReleaseDecoder(frames(strrep(text,'state=5','state=4'),uint64(4000001)),rq,d));
% Test compatibility with the preceding schema.
legacyq=rmfield(q,{'local_full_inner','leg_index','task_sha256'});legacyq.approved_parameter_sha256=repmat('B',1,64);
legacy=strrep(strrep(strrep(base,'RFLY_LOCAL_SESSION','RFLY_SESSION'),'state=2','state=1'), ...
    ['task_sha=' q.task_sha256 ' leg=1'],['parameter_sha=' legacyq.approved_parameter_sha256]);
lp=gpenmpcNative.RflySessionAssociationDecoder(frames(legacy,uint64(1000001)),[],legacyq,[],d);
check('legacy_unchanged_but_not_local',~lp.local_full_inner&&lp.parameter_identity_board_echoed);
source=fileread(fullfile(build,'rfly_vendor_integration','px4_runtime','CanonicalLocalSessionEntry.cpp'));
check('compiled_source_contract_not_legacy',contains(source,'RFLY_LOCAL_SESSION state=%u') ...
    &&contains(source,'argc != 10')&&contains(source,'RFLY_LOCAL_RELEASE result=%u quiescent=%u'));
unregistered=unregisteredRetained(build,"unregistered");
check('unregistered_fixture_reports_primary_board_state',unregistered.passed);
retiredFixture=unregisteredRetained(build,"retired");
check('retired_fixture_reports_primary_board_state',retiredFixture.passed);
report=struct('passed',all([checks.pass]),'test_count',numel(checks),'checks',checks, ...
    'scope','HOST_CHANGED_LOCAL_SESSION_SCHEMA_AND_COMMANDS','synthetic_callback_fixture',true, ...
    'board_registration_proven',false,'hardware_actions',0);
save(fullfile(outputRoot,'RAW.mat'),'pre','con','q','a','p','rr','report');
f=fopen(fullfile(outputRoot,'RESULT.json'),'wt');c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(report));
    function rows=frames(text,t)
        bytes=uint8(text);rows=struct('decoded_message',{},'decoded_source',{},'raw_frame_available',{},'original_host_receive_ns',{});
        for j=1:ceil(numel(bytes)/70)
            message=createmsg(d,'SERIAL_CONTROL');payload=message.Payload;part=bytes((j-1)*70+1:min(j*70,numel(bytes)));
            payload.device=uint8(10);payload.flags=uint8(1);payload.timeout=uint16(0);payload.baudrate=uint32(0);
            payload.count=uint8(numel(part));payload.data(:)=uint8(0);payload.data(1:numel(part))=part;
            decoded=struct('MsgID',uint32(126),'SystemID',q.system,'ComponentID',q.component,'Seq',uint8(j),'Payload',payload);
            rows(end+1)=struct('decoded_message',decoded,'decoded_source','ORIGINAL_MAVLINKIO_SERIAL_CONTROL_CALLBACK', ...
                'raw_frame_available',false,'original_host_receive_ns',t+uint64(j)); %#ok<AGROW>
        end
    end
    function check(n,ok),checks(end+1)=struct('name',n,'pass',logical(ok));assert(ok,'%s',n);end
    function reject(n,fn),ok=false;try,fn();catch ex,ok=startsWith(string(ex.identifier),'gpenmpcNative:');if ~ok,rethrow(ex);end,end;check(n,ok);end
end
function v=u64(b),v=typecast(b(:),'uint64');[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end
function proof=unregisteredRetained(build,fixtureKind)
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
assert(ismember(fixtureKind,["unregistered" "retired"]));
expectedCount=15;expectedState=0;expectedFault=0;
if fixtureKind=="retired",expectedCount=23;expectedState=5;expectedFault=3;end
source=string(gpenmpc_external_path(char(fixtureKind+"_session_fixture")));
a=load(source,'rawIo','sessionRaw','cfg');tx=a.sessionRaw.prepare_send;
tokens=strsplit(strtrim(tx.encoding.original_command));assert(numel(tokens)==10&&strcmp(tokens{2},'prepare'));
challengeBytes=uint8(sscanf(tokens{4},'%2x'));
% Use the sent challenge, identity and canonical expectations.
q=struct('original_host_challenge',u64(challengeBytes),'uid',gpenmpc_device_identity('uid_uint64'), ...
    'system',uint8(1),'component',uint8(1),'host_system',uint8(str2double(tokens{8})), ...
    'host_component',uint8(str2double(tokens{9})),'original_prepare_submit_ns',tx.original_host_submit_ns(1), ...
    'configuration_payload_sha256','A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA', ...
    'local_full_inner',true,'leg_index',uint8(str2double(tokens{10})), ...
    'task_sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
frames=struct('raw_frame',{},'original_host_receive_ns',{});shell=uint8([]);
for k=1:numel(a.rawIo.raw_mavlink)
    row=a.rawIo.raw_mavlink{k};
    if row.message.MsgID==126&&row.original_host_receive_ns>=q.original_prepare_submit_ns
        frames(end+1)=struct('raw_frame',row.raw_frame,'original_host_receive_ns',row.original_host_receive_ns); %#ok<AGROW>
        payload=row.message.Payload;shell=[shell;reshape(payload.data(1:double(payload.count)),[],1)]; %#ok<AGROW>
    end
end
lines=regexp(char(shell.'),'(?m)^RFLY_LOCAL_SESSION [^\r\n]*(?:\r?\n)','match');
assert(numel(frames)==expectedCount&&numel(lines)==1&&contains(lines{1},sprintf('state=%u ',expectedState)) ...
    &&contains(lines{1},sprintf('session_fault=%u ',expectedFault)) ...
    &&contains(lines{1},'registered=0 ')&&contains(lines{1},['config_sha=' repmat('0',1,64)]));
d=mavlinkdialect(a.cfg.local_mavlink_dialect.path,2);failure=[];created=false;
try
    gpenmpcNative.RflySessionAssociationDecoder(frames,[],q,[],d);created=true;
catch ex
    failure=struct('identifier',ex.identifier,'message',ex.message,'stack',ex.stack);
end
assert(~created&&isstruct(failure)&&strcmp(failure.identifier,'gpenmpcNative:SessionNotRegistered') ...
    &&contains(failure.message,sprintf('state=%u, registered=0, session_fault=%u',expectedState,expectedFault)) ...
    &&contains(failure.message,lines{1}),'Retained receipt must reject with the primary board state and exact original line.');
proof=struct('passed',true,'source_path',source,'source_sha256',m600check.fileSha256(source), ...
    'original_state',expectedState,'original_session_fault',expectedFault,'original_frame_count',numel(frames), ...
    'original_frames',frames,'original_shell_bytes',shell,'original_session_line_bytes',uint8(lines{1}).', ...
    'request',q,'failure',failure,'association_created',created);
end
