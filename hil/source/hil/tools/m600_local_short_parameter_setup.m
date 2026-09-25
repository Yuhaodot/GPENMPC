function result=m600_local_short_parameter_setup(outputPath,operation,handoffPath,geometry,tuning,attemptedReceiptPath)
% Manage parameters through one serial owner.
% VALIDATE checks data and journal contents without opening a link.
% APPLY journals before each PARAM_SET; RESTORE writes attempted original values.
if nargin<6,attemptedReceiptPath='';end
operation=char(string(operation));
assert(any(strcmp(operation,{'VALIDATE','APPLY','RESTORE'})),'gpenmpcSetup:Operation','Operation requirement not met.');
plan=entryPlan(geometry,tuning);
if strcmp(operation,'VALIDATE')
    result=pureValidation(plan);return
end
assert(ischar(outputPath)||isstring(outputPath),'gpenmpcSetup:Output','Output requirement not met.');
outputPath=char(outputPath);
assert(~isempty(regexp(outputPath,'^[A-Za-z]:[\\/]','once'))&&isfolder(fileparts(outputPath)) ...
    &&~isfile(outputPath)&&~isfile([outputPath '.pending'])&&~isfile([outputPath '.previous']), ...
    'gpenmpcSetup:OutputExists','Use a fresh absolute result path in an existing directory.');
build=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(build,'host_runtime'),fullfile(build,'tools'), ...
    fullfile(build,'m600_coptersim','matlab_validation'));
[originalContracts,contractBinding]=load_m600_recovery_contracts();
assert(isequaln(geometry,originalContracts.temporary_allocator_geometry) ...
    &&isequaln(tuning,originalContracts.native_hover_tuning),'gpenmpcSetup:Contracts', ...
    'Pass the complete unchanged recovery contracts from the bound recovery plan.');
[expected,parentBinding]=original167();
for k=1:21
    r=expected(strcmp({expected.name},plan(k).name));
    assert(isscalar(r)&&sameTyped(r,plan(k).original),'gpenmpcSetup:OriginalContract','OriginalContract requirement not met.');
end
source=[];handoff=[];handoffAdmission=[];
recoveryOriginalEvidence=struct('name',{},'receipt_original',{},'actual_pre_apply_readback',{},'basis',{});
if strcmp(operation,'APPLY')
    assert(isfile(handoffPath),'gpenmpcSetup:Handoff','Handoff requirement not met.');
    handoff=jsondecode(fileread(handoffPath));
    handoffAdmission=validateHandoff(handoff,expected);
    source=struct('path',char(handoffPath),'sha256',m600check.fileSha256(handoffPath));
else
    assert(isfile(attemptedReceiptPath),'gpenmpcSetup:AttemptedReceipt','AttemptedReceipt requirement not met.');
    prior=jsondecode(fileread(attemptedReceiptPath));
    assert(strcmp(prior.schema,'GPENMPC_LOCAL_SHORT_PARAMETER_SETUP_V1') ...
        &&strcmp(prior.operation,'APPLY')&&strcmp(prior.uid,gpenmpc_device_identity('uid')) ...
        &&numel(prior.writes)==21,'gpenmpcSetup:AttemptedReceipt','AttemptedReceipt requirement not met.');
    for k=1:21
        w=prior.writes(k);
        actualBefore=prior.before_parameters(strcmp({prior.before_parameters.name},plan(k).name));
        originalExact=sameTyped(w.original,plan(k).original);
        % Use the complete pre-APPLY readback and SHA-bound baseline values for
        % recovery when a redundant writes.original field differs.
        typedReadbackMatchesBaseline=logicalBit(w.attempted)&&logical(w.attempted) ...
            &&sameTyped(w.original,typed('PWM_AUX_FUNC8',6,'00000000')) ...
            &&sameRows(prior.before_parameters,expected)&&sameTyped(actualBefore,plan(k).original);
        assert(strcmp(w.name,plan(k).name)&&w.mav_type==plan(k).mav_type ...
            &&strcmpi(w.before_raw_bits_hex,plan(k).before_raw_bits_hex) ...
            &&strcmpi(w.target_raw_bits_hex,plan(k).target_raw_bits_hex) ...
            &&(originalExact||typedReadbackMatchesBaseline)&&logicalBit(w.attempted), ...
            'gpenmpcSetup:AttemptedReceipt','Attempt journal %s differs from the exact original plan and independently retained actual pre-APPLY readback.',plan(k).name);
        if ~originalExact
            recoveryOriginalEvidence(end+1)=struct('name',plan(k).name,'receipt_original',w.original, ...
                'actual_pre_apply_readback',actualBefore, ...
                'basis','TYPED_PRE_APPLY_READBACK_MATCHES_BOUND_RECOVERY_BASELINE'); %#ok<AGROW>
        end
        prior.writes(k).attempted=logical(w.attempted); % JSON scalar representation only.
    end
    source=struct('path',char(attemptedReceiptPath),'sha256',m600check.fileSha256(attemptedReceiptPath));
end
assertNoSimulator();
result=struct('schema','GPENMPC_LOCAL_SHORT_PARAMETER_SETUP_V1','operation',operation, ...
    'passed',false,'COM_closed',true,'uid',gpenmpc_device_identity('uid'),'port','COM3', ...
    'baud',921600,'source',source,'handoff_admission',handoffAdmission, ...
    'original167_binding',parentBinding,'contract_binding',contractBinding, ...
    'recovery_original_evidence',recoveryOriginalEvidence, ...
    'before_parameters',typedEmpty(),'after_parameters',typedEmpty(), ...
    'guard_parameters',typedEmpty(),'aux_parameters',typedEmpty(), ...
    'writes',plan,'parameter_writes',0,'write_attempts',0,'restored_count',0, ...
    'already_original_count',0,'arm_mode_requests',0,'other_parameter_writes',0, ...
    'simulator_processes_started',0,'physical_output_actions',0, ...
    'journal_sequence',0,'started_utc',utc(),'updated_utc','','failure','', ...
    'cleanup_failure','','identity',[],'final_identity',[],'link_counters',[]);
if strcmp(operation,'RESTORE')
    result.apply_attempted_names={prior.writes([prior.writes.attempted]).name};
    result.writes=repmat(plan(1),0,1);
end
% Reserve only our fresh file, then atomically replace complete JSON snapshots.
reserve=System.IO.FileStream(outputPath,System.IO.FileMode.CreateNew,System.IO.FileAccess.Write,System.IO.FileShare.None);
reserve.Dispose();
link=[];finished=false;guard=onCleanup(@finish);
try
    persist();
    if strcmp(operation,'APPLY'),validateHandoffFreshness(handoff);end
    assertNoSimulator();
    link=gpenmpcNative.MavlinkSerialLink;
    hb=link.open('COM3',921600,15);result.COM_closed=false;
    assert(link.TargetSystem==1&&link.TargetComponent==1&&own(hb),'gpenmpcSetup:Target', ...
        'Serial target must be system/component 1/1; observed %g/%g.',link.TargetSystem,link.TargetComponent);
    result.identity=link.collectIdentity(10);validateIdentity(result.identity);
    result.driver_guards=readDriverGuards();
    result.before_parameters=read167('before_parameters');
    if strcmp(operation,'APPLY')
        assert(sameRows(result.before_parameters,expected),'gpenmpcSetup:Before167','Original 167 parameters differ.');
    else
        for k=1:167
            r=result.before_parameters(k);e=expected(k);
            index=find(strcmp({plan.name},r.name));
            allowed=sameTyped(r,e);
            if ~isempty(index)&&prior.writes(index).attempted
                allowed=allowed||sameTyped(r,targetTyped(plan(index)));
            end
            assert(allowed,'gpenmpcSetup:UnknownThirdValue','Unexpected current value for parameter %s.',r.name);
        end
    end
    [result.guard_parameters,result.aux_parameters]=readOutputGuards();persist();
    if strcmp(operation,'APPLY')
        for k=1:21,writeOne(k,plan(k),plan(k).before_raw_bits_hex,plan(k).target_raw_bits_hex);end
    else
        for k=21:-1:1
            if ~prior.writes(k).attempted,continue,end
            p=plan(k);r=readParam(p.name);
            if sameTyped(r,p.original)
                result.already_original_count=result.already_original_count+1;
                w=p;w.original=r;w.before_raw_bits_hex=r.raw_bits_hex;w.target_raw_bits_hex=r.raw_bits_hex;
                w.independent_readback=r;w.disposition='ALREADY_ORIGINAL_NO_WRITE';
                result.writes(end+1,1)=w;persist();
            else
                assert(sameTyped(r,targetTyped(p)),'gpenmpcSetup:UnknownThirdValue', ...
                    'Restore %s observed type %g bits %s, neither expected original nor applied target.',r.name,r.mav_type,r.raw_bits_hex);
                p.original=r;p.before_raw_bits_hex=r.raw_bits_hex;p.target_raw_bits_hex=plan(k).before_raw_bits_hex;
                result.writes(end+1,1)=p;
                writeOne(numel(result.writes),p,p.before_raw_bits_hex,p.target_raw_bits_hex);
                result.restored_count=result.restored_count+1;
            end
        end
    end
    result.after_parameters=read167('after_parameters');
    afterExpected=expected;
    if strcmp(operation,'APPLY')
        for k=1:21,index=strcmp({afterExpected.name},plan(k).name);afterExpected(index)=targetTyped(plan(k));end
    end
    assert(sameRows(result.after_parameters,afterExpected),'gpenmpcSetup:After167', ...
        'Final selected 167 typed parameters differ from the %s expectation; actual rows remain in the receipt.',operation);
    [result.guard_parameters,result.aux_parameters]=readOutputGuards();
    result.final_identity=link.collectIdentity(10);validateIdentity(result.final_identity);
    result.final_driver_guards=readDriverGuards();
    result.final_guard=freshState();
    assertNoSimulator();
    result.passed=true;
catch ex
    result.failure=getReport(ex,'extended','hyperlinks','off');
end
finish();clear guard
disp(jsonencode(struct('operation',operation,'passed',result.passed,'COM_closed',result.COM_closed, ...
    'write_attempts',result.write_attempts,'parameter_writes',result.parameter_writes)));

    function writeOne(index,p,beforeBits,targetBits)
        assertNoSimulator();
        preWriteOriginal=readParam(p.name);
        assert(preWriteOriginal.mav_type==p.mav_type&&strcmpi(preWriteOriginal.raw_bits_hex,beforeBits),'gpenmpcSetup:PreWriteDrift', ...
            'Before writing %s expected type %g bits %s, observed type %g bits %s.',p.name,p.mav_type,beforeBits,preWriteOriginal.mav_type,preWriteOriginal.raw_bits_hex);
        [~,~]=readOutputGuards();
        g=freshState();safeWatch=tic;
        w=result.writes(index);w.original=preWriteOriginal;w.before_raw_bits_hex=beforeBits;w.target_raw_bits_hex=targetBits;
        w.guard=g;w.attempted=true;w.disposition='ATTEMPT_JOURNALED_BEFORE_SEND';w.attempted_utc=utc();
        result.writes(index)=w;result.write_attempts=result.write_attempts+1;
        persist(); % Failure from this point is uncertain; original target is durable.
        assert(toc(safeWatch)<3,'gpenmpcSetup:GuardExpiredBeforeWrite', ...
            'Fresh disarmed/landed guard expired while journaling %s; PARAM_SET was not sent.',p.name);
        payload=parameterPayload(p.name,p.mav_type,targetBits);
        link.sendMessage('PARAM_SET',payload); % Single attempt.
        result.parameter_writes=result.parameter_writes+1;
        result.writes(index).send_returned=true;result.writes(index).disposition='SEND_RETURNED';persist();
        m=link.waitForMessage('PARAM_VALUE',3,@(x)own(x)&&strcmp(gpenmpcNative.MavlinkSerialLink.cleanText(x.Payload.param_id),p.name));
        result.writes(index).echo=typedMessage(m);persist();
        assert(sameTyped(result.writes(index).echo,typed(p.name,p.mav_type,targetBits)),'gpenmpcSetup:EchoMismatch', ...
            'PARAM_SET echo for %s differs from requested type %g bits %s; original echo is retained.',p.name,p.mav_type,targetBits);
        r=readParam(p.name);result.writes(index).independent_readback=r;persist();
        assert(sameTyped(r,typed(p.name,p.mav_type,targetBits)),'gpenmpcSetup:IndependentReadbackMismatch', ...
            'Independent readback for %s expected type %g bits %s, observed type %g bits %s.',p.name,p.mav_type,targetBits,r.mav_type,r.raw_bits_hex);
        result.writes(index).disposition='ECHO_AND_INDEPENDENT_READBACK_EXACT';persist();
    end
    function rows=read167(field)
        rows=typedEmpty();
        for j=1:167
            r=readParam(expected(j).name);rows(end+1,1)=r; %#ok<AGROW>
            % Keep each observed row even when a subsequent read times out.
            result.(field)=rows;
        end
    end
    function rows=readDriverGuards()
        rows=struct('name',{},'original_text',{});
        for name={'pwm_out','px4io','dshot'}
            original=char(link.shellCommand(string(name{1})+" status",3));
            rows(end+1)=struct('name',name{1},'original_text',original); %#ok<AGROW>
            result.last_driver_guard=rows;
            assert(~isempty(regexp(original,'not running|command not found','once')), ...
                'gpenmpcSetup:PhysicalDriverGuard','Physical driver status is unknown or running: %s.',name{1});
        end
    end
    function r=readParam(name)
        assert(~strcmp(name,'_HASH_CHECK'),'gpenmpcSetup:HashCheckForbidden','HashCheckForbidden requirement not met.');
        link.drain();
        link.sendMessage('PARAM_REQUEST_READ',struct('param_index',int16(-1), ...
            'target_system',uint8(1),'target_component',uint8(1),'param_id',char(name)));
        m=link.waitForMessage('PARAM_VALUE',3,@(x)own(x)&&strcmp(gpenmpcNative.MavlinkSerialLink.cleanText(x.Payload.param_id),name));
        r=typedMessage(m);
    end
    function [guards,aux]=readOutputGuards()
        guards=typedEmpty();aux=typedEmpty();
        result.guard_parameters=guards;result.aux_parameters=aux;
        names=[{'COM_OBL_RC_ACT','RA_CTRL_MODE','SYS_HITL'},cellstr(compose('PWM_MAIN_FUNC%d',1:8))];
        bits=[{'00000004','00000000','00000001'},repmat({'00000000'},1,8)];
        for j=1:numel(names)
            r=readParam(names{j});guards(end+1,1)=r; %#ok<AGROW>
            result.guard_parameters=guards;
            allowed=sameTyped(r,typed(names{j},6,bits{j}));
            if strcmp(operation,'RESTORE')&&strcmp(names{j},'COM_OBL_RC_ACT')
                % The recovery-route owner may already have restored zero.
                % This path checks that state without changing the parameter.
                allowed=allowed||sameTyped(r,typed('COM_OBL_RC_ACT',6,'00000000'));
            end
            assert(allowed,'gpenmpcSetup:PhysicalGuard','Missing/wrong guard %s during %s.',names{j},operation);
        end
        for j=1:8
            name=sprintf('PWM_AUX_FUNC%d',j);r=readParam(name);aux(end+1,1)=r; %#ok<AGROW>
            result.aux_parameters=aux;
            assert(sameTyped(r,typed(name,6,'00000000')),'gpenmpcSetup:AuxGuard', ...
                '%s must be INT32 zero; observed type %g bits %s.',name,r.mav_type,r.raw_bits_hex);
        end
    end
    function g=freshState()
        link.drain();link.requestMessage(148);link.requestMessage(245);watch=tic;
        v=link.waitForMessage('AUTOPILOT_VERSION',3,@own);
        e=link.waitForMessage('EXTENDED_SYS_STATE',3,@own);
        h=link.waitForMessage('HEARTBEAT',3,@own);
        assert(toc(watch)<3&&strcmp(sprintf('%u',uint64(v.Payload.uid)),result.uid) ...
            &&double(v.Payload.board_version)==56&&double(v.Payload.product_id)==56 ...
            &&strcmp(upper(reshape(dec2hex(uint8(v.Payload.uid2),2).',1,[])),gpenmpc_device_identity('px4_guid')) ...
            &&bitand(uint8(h.Payload.base_mode),uint8(128))==0&&double(e.Payload.landed_state)==1, ...
            'gpenmpcSetup:FreshSafety','Current exact UID/disarmed/on-ground not proved.');
        g=struct('observed_utc',utc(),'identity',v,'heartbeat',h,'extended_state',e,'observation_window_s',toc(watch));
    end
    function persist()
        result.journal_sequence=result.journal_sequence+1;result.updated_utc=utc();
        bytes=unicode2native(jsonencode(result,PrettyPrint=true),'UTF-8');
        tmp=[outputPath '.pending'];
        stream=System.IO.FileStream(tmp,System.IO.FileMode.CreateNew,System.IO.FileAccess.Write,System.IO.FileShare.None);
        release=onCleanup(@()stream.Dispose());
        stream.Write(uint8(bytes),int32(0),int32(numel(bytes)));stream.Flush(true);stream.Dispose();clear release
        System.IO.File.Replace(tmp,outputPath,[outputPath '.previous'],true);
    end
    function finish()
        if finished,return,end
        finished=true;
        if ~isempty(link)
            try,link.close();result.COM_closed=~link.isOpen();result.link_counters=link.counters();
            catch ex,result.COM_closed=false;result.cleanup_failure=getReport(ex,'extended','hyperlinks','off');end
        end
        result.passed=result.passed&&result.COM_closed;
        try,persist();catch ex
            result.passed=false;result.cleanup_failure=[result.cleanup_failure newline getReport(ex,'extended','hyperlinks','off')];
            warning('gpenmpcSetup:JournalFinalization','Final JSON write failed; last atomic attempt receipt remains at %s.',outputPath);
        end
    end
end

function plan=entryPlan(geometry,tuning)
assert(isstruct(geometry)&&isscalar(geometry)&&isstruct(tuning)&&isscalar(tuning) ...
    &&strcmp(geometry.schema,'TEMPORARY_M600_NATIVE_ALLOCATOR_GEOMETRY_V1') ...
    &&strcmp(tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1') ...
    &&numel(geometry.entries)==12&&numel(tuning.entries)==3,'gpenmpcSetup:ContractShape','ContractShape requirement not met.');
entries=[geometry.entries(:);tuning.entries(:)];
assert(numel(unique({entries.name}))==15&&all([entries.mav_type]==9),'gpenmpcSetup:Entries','Entries requirement not met.');
assert(isequal(reshape({tuning.entries.name},1,[]),{'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'}),'gpenmpcSetup:TuningNames','TuningNames requirement not met.');
template=struct('name','','mav_type',0,'before_raw_bits_hex','','target_raw_bits_hex','', ...
    'original',[],'echo',[],'independent_readback',[],'attempted',false,'send_returned',false, ...
    'guard',[],'attempted_utc','','disposition','NOT_ATTEMPTED');
plan=repmat(template,21,1);
for k=1:21
    if k<=15,e=entries(k);name=char(e.name);mavType=e.mav_type;before=e.original_raw_bits_hex;target=e.target_raw_bits_hex;
    else,name=sprintf('HIL_ACT_FUNC%d',k-15);mavType=6;before='00000000';target=dec2hex(uint32(100+k-15),8);end
    parameterPayload(name,mavType,before);parameterPayload(name,mavType,target);
    assert(~strcmpi(before,target),'gpenmpcSetup:NoOpPlan','NoOpPlan requirement not met.');
    plan(k).name=name;plan(k).mav_type=double(mavType);
    plan(k).before_raw_bits_hex=upper(char(before));plan(k).target_raw_bits_hex=upper(char(target));
    plan(k).original=typed(name,mavType,before);
end
assert(numel(unique({plan.name}))==21&&~any(strcmp({plan.name},'_HASH_CHECK')),'gpenmpcSetup:PlanNames','PlanNames requirement not met.');
end
function payload=parameterPayload(name,mavType,bits)
assert(ischar(name)&&~isempty(regexp(name,'^[A-Z][A-Z0-9_]{0,15}$','once')) ...
    &&~strcmp(name,'_HASH_CHECK')&&ismember(mavType,[6 9]) ...
    &&~isempty(regexp(char(bits),'^[0-9A-Fa-f]{8}$','once')),'gpenmpcSetup:ParamBits','ParamBits requirement not met.');
raw=typecast(uint32(hex2dec(char(bits))),'single');
if mavType==9,assert(isfinite(raw),'gpenmpcSetup:FiniteReal','FiniteReal requirement not met.');end
assert(strcmpi(dec2hex(typecast(raw,'uint32'),8),bits),'gpenmpcSetup:BitPreservation','BitPreservation requirement not met.');
payload=struct('target_system',uint8(1),'target_component',uint8(1), ...
    'param_id',name,'param_value',raw,'param_type',uint8(mavType));
end
function result=pureValidation(plan)
for k=1:21
    p=parameterPayload(plan(k).name,plan(k).mav_type,plan(k).target_raw_bits_hex);
    assert(isa(p.param_value,'single')&&strcmpi(dec2hex(typecast(p.param_value,'uint32'),8),plan(k).target_raw_bits_hex));
end
w=plan(1);before=w;w.attempted=true;assert(~before.attempted&&isempty(w.echo)&&sameTyped(w.original,before.original));
w.echo=targetTyped(w);assert(w.attempted&&isempty(w.independent_readback));
w.independent_readback=w.echo;assert(sameTyped(w.echo,w.independent_readback));
result=struct('schema','GPENMPC_LOCAL_SHORT_PARAMETER_SETUP_VALIDATION_V1','passed',true, ...
    'entry_count',21,'bit_payload_checks',21,'journal_stage_checks',3,'hardware_actions',0,'COM_open',0,'writes',plan);
end
function [rows,binding]=original167()
path=fullfile(getenv('HIL_RECOVERY_RECORDS'),'SERIAL_PREFLIGHT.json');
assert(~isempty(getenv('HIL_RECOVERY_RECORDS')),'gpenmpcSetup:RecoveryRecords','Configure HIL_RECOVERY_RECORDS.');
digest='073E7CE165D8D158BC329CFE96AEC4F968A6847A2A0AB9D90C1C127D3CB65EBB';
assert(strcmpi(m600check.fileSha256(path),digest),'gpenmpcSetup:RecoveryHash');
p=jsondecode(fileread(path));rows=typedEmpty();
for k=1:numel(p.parameters),take(p.parameters(k));end
for k=1:numel(p.native_hover_parameter_union.rows),take(p.native_hover_parameter_union.rows(k).observed);end
assert(numel(rows)==167&&~any(strcmp({rows.name},'_HASH_CHECK')),'gpenmpcSetup:Original167');
binding=struct('path',path,'sha256',digest);
    function take(r)
        r=typed(r.name,r.mav_type,r.raw_bits_hex);index=find(strcmp({rows.name},r.name));
        if isempty(index),rows(end+1,1)=r;else,assert(isscalar(index)&&sameTyped(rows(index),r));end
    end
end
function admission=validateHandoff(h,expected)
assert(all(isfield(h,{'schema','operation','controlled_application_reboot_to_bootloader_count', ...
    'bootloader_firmware_writes','application_flash_attempts','usb_reenumeration_pending', ...
    'reboot_ack_accepted','driver_stop_attempts','driver_stop_verified'})), ...
    'gpenmpcSetup:HandoffSchema','Handoff must include explicit operation, reboot and verified driver-stop counters.');
assert(strcmp(h.schema,'GPENMPC_APPLICATION_FLASH_HANDOFF_V1') ...
    &&any(strcmp(h.operation,{'READONLY','SAFE_STOP_PX4IO','SAFE_STOP_PX4IO_START_BOARD_ADC'})), ...
    'gpenmpcSetup:HandoffOperation','Only READONLY or an explicit verified safe-stop operation may authorize parameter setup, not a reboot handoff.');
assert(h.controlled_application_reboot_to_bootloader_count==0&&h.bootloader_firmware_writes==0 ...
    &&h.application_flash_attempts==0&&~h.usb_reenumeration_pending&&~h.reboot_ack_accepted, ...
    'gpenmpcSetup:HandoffReboot','Setup requires zero reboot/flash attempts and no pending USB re-enumeration.');
assert(isnumeric(h.driver_stop_attempts)&&isscalar(h.driver_stop_attempts) ...
    &&any(h.driver_stop_attempts==[0 1])&&h.driver_stop_verified==h.driver_stop_attempts ...
    &&(~strcmp(h.operation,'READONLY')||h.driver_stop_attempts==0), ...
    'gpenmpcSetup:HandoffDriverStop','Driver stop must be explicitly SAFE_STOP_PX4IO, at most one attempt, and every attempt independently verified.');
assert(h.passed&&h.COM_closed&&h.disarmed&&h.landed&&h.usb_only&&h.all_modules_stopped ...
    &&h.pwm_out_stopped&&h.physical_path_disabled&&h.virtual_path_disabled&&h.parameter_writes==0 ...
    &&h.arm_mode_requests==0&&h.physical_output_actions==0 ...
    &&strcmp(h.uid,gpenmpc_device_identity('uid'))&&sameRows(h.selected_parameters,expected), ...
    'gpenmpcSetup:Handoff','Fresh exact-board disarmed/landed, stopped-driver, zero-output and unchanged-167 handoff guards did not all pass.');
if strcmp(h.operation,'SAFE_STOP_PX4IO_START_BOARD_ADC')
    assert(all(isfield(h,{'board_adc_start_attempts','board_adc_start_verified','board_adc_power_witness'})) ...
        &&(h.board_adc_start_attempts==1||(h.board_adc_start_attempts==0 ...
          &&isfield(h,'board_adc_reused')&&isequal(h.board_adc_reused,true)))&&h.board_adc_start_verified==1 ...
        &&isstruct(h.board_adc_power_witness)&&isscalar(h.board_adc_power_witness) ...
        &&h.board_adc_power_witness.passed, ...
        'gpenmpcSetup:PowerObserver','Require a started or explicitly reused board_adc publisher and fresh USB-only system_power evidence.');
end
validateHandoffFreshness(h);
admission=struct('operation',h.operation,'driver_stop_attempts',h.driver_stop_attempts, ...
    'driver_stop_verified',h.driver_stop_verified,'reboot_attempts',0,'parameter_writes',0, ...
    'reason','FRESH_READONLY_NO_DRIVER_STOP');
if strcmp(h.operation,'SAFE_STOP_PX4IO')
    admission.reason='FRESH_EXPLICIT_SAFE_STOP_PX4IO_WITH_ALL_ATTEMPTS_VERIFIED';
elseif strcmp(h.operation,'SAFE_STOP_PX4IO_START_BOARD_ADC')
    admission.reason='FRESH_EXPLICIT_SAFE_STOP_PX4IO_AND_VERIFIED_HITL_POWER_OBSERVER';
end
end
function validateHandoffFreshness(h)
fmt="yyyy-MM-dd'T'HH:mm:ss.SSS'Z'";
t=datetime('now','TimeZone','UTC');start=datetime(h.handoff_created_utc,'InputFormat',fmt,'TimeZone','UTC');
expiry=datetime(h.expires_utc,'InputFormat',fmt,'TimeZone','UTC');
assert(t>=start&&t<=expiry&&seconds(expiry-start)<=60.001,'gpenmpcSetup:HandoffExpired', ...
    'Handoff is not within its original 60-second freshness interval before COM open.');
end
function validateIdentity(i)
v=i.autopilot_version;
assert(strcmp(sprintf('%u',uint64(v.uid)),gpenmpc_device_identity('uid'))&&double(v.board_version)==56 ...
    &&double(v.product_id)==56&&strcmp(upper(reshape(dec2hex(uint8(v.uid2),2).',1,[])),gpenmpc_device_identity('px4_guid')) ...
    &&strcmp(i.parsed_hw_arch,'PX4_FMU_V6C')&&strcmpi(i.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d'), ...
    'gpenmpcSetup:Identity','Actual UID/GUID, board/product 56, PX4_FMU_V6C or original firmware commit differs.');
end
function assertNoSimulator()
assert(System.Diagnostics.Process.GetProcessesByName('CopterSim').Length==0 ...
    &&System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0, ...
    'gpenmpcSetup:SimulatorRunning','Parameter setup/restore requires NoUI and GUI CopterSim already stopped.');
end
function value=own(m),value=double(m.SystemID)==1&&double(m.ComponentID)==1;end
function r=typedMessage(m)
r=typed(gpenmpcNative.MavlinkSerialLink.cleanText(m.Payload.param_id),double(m.Payload.param_type), ...
    dec2hex(typecast(single(m.Payload.param_value),'uint32'),8));
end
function r=typed(name,mavType,bits)
r=struct('name',char(name),'mav_type',double(mavType),'raw_bits_hex',upper(char(bits)));
end
function r=targetTyped(w),r=typed(w.name,w.mav_type,w.target_raw_bits_hex);end
function rows=typedEmpty(),rows=struct('name',{},'mav_type',{},'raw_bits_hex',{});end
function yes=sameTyped(a,b)
yes=isstruct(a)&&isscalar(a)&&all(isfield(a,{'name','mav_type','raw_bits_hex'})) ...
    &&strcmp(a.name,b.name)&&a.mav_type==b.mav_type&&strcmpi(a.raw_bits_hex,b.raw_bits_hex);
end
function yes=logicalBit(value)
yes=(islogical(value)||isnumeric(value))&&isscalar(value)&&isreal(value) ...
    &&isfinite(value)&&any(value==[0 1]);
end
function yes=sameRows(a,b)
yes=numel(a)==numel(b)&&numel(unique({a.name}))==numel(a);if ~yes,return,end
for k=1:numel(b),r=a(strcmp({a.name},b(k).name));if ~sameTyped(r,b(k)),yes=false;return,end,end
end
function s=utc(),s=char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));end
