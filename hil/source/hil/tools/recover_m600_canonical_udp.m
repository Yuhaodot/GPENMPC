function receipt=recover_m600_canonical_udp(cfg,io)
% RECOVER_M600_CANONICAL_UDP Recover through UDP while CopterSim remains alive.
% Open endpoints only after the prior child and its handles have been released.
% Track optional allocator and tuning restoration separately, with final readback.
% Explicit emergency force-disarm is available only after normal recovery expires.
% Stop the owned CopterSim only when safe_bridge_shutdown_allowed is true.
% Host tests may inject IO; all deadlines come from the runner configuration.
required={'expected_uid','expected_board_version','expected_flight_custom_version_hex'};
assert(all(isfield(cfg,required)),'m600check:RecoveryIdentityMissing','Required recovery identity fields are missing.');
assert(string(cfg.expected_uid)==string(gpenmpc_device_identity('uid'))&&cfg.expected_board_version==56, ...
    'm600check:RecoveryTargetMismatch','Recovery target is not the expected board.');
assert(strlength(string(cfg.expected_flight_custom_version_hex))==16, ...
    'm600check:RecoveryFirmwareIdentityMissing','Exact firmware version bytes are required.');
tuningRequested=isfield(cfg,'native_hover_tuning');
if tuningRequested
    if strcmp(cfg.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
        tuningCheck=validate_m600_delivery_native_tuning_contract(cfg.native_hover_tuning);
    else
        tuningCheck=m600check.validateNativeHoverTuningContract(cfg.native_hover_tuning);
    end
    assert(tuningCheck.passed,'m600check:RecoveryTuningContract','%s',tuningCheck.failure);
    assert(isfield(cfg,'temporary_allocator_geometry')&&~isempty(cfg.temporary_allocator_geometry), ...
        'm600check:RecoveryTuningGeometryRequired','Tuning restoration requires the separately bound geometry contract.');
    geometryCheck=m600check.validateTemporaryAllocatorGeometry(cfg.temporary_allocator_geometry);
    assert(geometryCheck.passed,'m600check:RecoveryTuningGeometryContract','%s',geometryCheck.failure);
end
if nargin<2,io=m600check.makeM600CopterSimIo(cfg);end
latest=[];closed=false;done=false;failure='';identityPass=false;safe=false;
geometryRequested=isfield(cfg,'temporary_allocator_geometry')&&~isempty(cfg.temporary_allocator_geometry);
geometryReceipt=[];geometryFailure='';geometrySafe=~geometryRequested;
tuningReceipt=[];tuningFailure='';tuningSafe=~tuningRequested;
parameterUnionRestored=[];parameterUnionFailure='';parameterUnionSafe=~tuningRequested;
geometryGuards=struct('time_s',{},'previous_owner_released',{},'before',{}, ...
    'typed_zero_parameters',{},'after',{},'passed',{},'failure',{},'output_guard_basis',{});
events=struct('time_s',{},'kind',{},'detail',{});actions=struct('name',{},'target',{}, ...
    'attempted',{},'write_returned',{},'typed_readback_pass',{},'error',{});
counts=struct('native_land_requests',0,'standard_disarm_requests',0, ...
    'arm_requests',0,'force_disarm_requests',0,'mapping_zero_attempts',0, ...
    'mapping_zero_verified',0,'physical_mapping_writes',0,'parameter_other_writes',0, ...
    'geometry_restore_setter_attempts',0,'geometry_restore_setter_returns',0, ...
    'geometry_restore_ack_readback_verified',0,'geometry_restore_no_op',0, ...
    'tuning_restore_setter_attempts',0,'tuning_restore_setter_returns',0, ...
    'tuning_restore_ack_readback_verified',0,'tuning_restore_no_op',0);
lastHeartbeat=-Inf;landSentAt=NaN;disarmSentAt=NaN;guard=onCleanup(@closeIo);
try
    identitySent=io.now();io.requestCommand(512,[148,0,0,0,0,0,0]);
    assert(waitFor(@identityFresh,cfg.preflight_deadline_s), ...
        'm600check:RecoveryIdentityUnavailable','Fresh exact AUTOPILOT_VERSION was not established.');
    identityPass=identityMatches(latest);
    assert(identityPass,'m600check:RecoveryIdentityMismatch','UID/board/firmware changed; no safety writes sent.');
    assert(waitFor(@stateFresh,cfg.preflight_deadline_s),'m600check:RecoveryStateUnknown', ...
        'Armed/landed unknown. Keep CopterSim running; do not write mappings.');
    p=io.readParameter('RA_CTRL_MODE');assert(p.mav_type==6&&p.decoded==0, ...
        'm600check:RecoveryControlIdentityMismatch','Recovery cannot change the controller identity.');
    observe();
    if latest.armed==1
        if latest.landed_state~=1
            counts.native_land_requests=counts.native_land_requests+1;
            landSentAt=io.now();
            request(176,[1,4,6,0,0,0,0],'NATIVE_LAND');
            assert(waitFor(@(s)stateFresh(s)&&s.main_mode==4&&s.sub_mode==6,cfg.mode_timeout_s), ...
                'm600check:RecoveryLandModeMissing','Native LAND mode was not observed.');
            assert(waitFor(@(s)stateFresh(s)&&s.landed_state==1,cfg.land_timeout_s), ...
                'm600check:RecoveryLandTimeout','No native landed witness. Keep sensor/plant owner alive.');
        end
        observe();
        if latest.armed==1
            assert(stateFresh(latest)&&latest.landed_state==1,'m600check:RecoveryUnsafeDisarm','Disarm requires fresh landed state.');
            counts.standard_disarm_requests=counts.standard_disarm_requests+1;
            disarmSentAt=io.now();
            request(400,zeros(1,7),'STANDARD_DISARM');
        end
    end
    assert(waitFor(@disarmedGround,cfg.disarm_timeout_s),'m600check:RecoveryDisarmNotConfirmed','Fresh disarmed/landed state was not established.');
    zeroMappings();
catch e
    failure=[e.identifier ': ' e.message];event('RECOVERY_EXCEPTION',failure);
end
% Permit emergency virtual disarm only after normal recovery expires
% and fresh target/output checks pass.
allowEmergency=isfield(cfg,'allow_emergency_logical_force_disarm')&& ...
    isequal(cfg.allow_emergency_logical_force_disarm,true);
normalExpired=(isfinite(landSentAt)&&io.now()-landSentAt>=cfg.land_timeout_s)|| ...
    (isfinite(disarmSentAt)&&io.now()-disarmSentAt>=cfg.disarm_timeout_s);
if allowEmergency&&normalExpired&&identityPass
    try
        observe();assert(stateFresh(latest),'m600check:EmergencyStateUnknown','Fresh reachable armed state is required.');
        identitySent=io.now();io.requestCommand(512,[148,0,0,0,0,0,0]);
        assert(waitFor(@identityFresh,cfg.preflight_deadline_s)&&identityMatches(latest), ...
            'm600check:EmergencyIdentityMismatch','No emergency request to an unknown target.');
        % Verify typed parameters, disabled outputs and recognized HIL functions.
        names=[compose("PWM_MAIN_FUNC%d",1:8),compose("HIL_ACT_FUNC%d",7:16),"RA_CTRL_MODE"];
        for k=1:numel(names)
            p=io.readParameter(char(names(k)));
            assert(p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000'), ...
                'm600check:EmergencyPhysicalGuardUnknown','Emergency virtual-only output guards differ.');
        end
        for k=1:6
            p=io.readParameter(sprintf('HIL_ACT_FUNC%d',k));
            assert(p.mav_type==6&&ismember(p.decoded,[0,100+k]), ...
                'm600check:EmergencyVirtualGuardUnknown','Unrecognized virtual output mapping.');
        end
        observe();assert(stateFresh(latest),'m600check:EmergencyStateUnknown','State became stale during guard reads.');
        if latest.armed==1&&latest.landed_state==1&&counts.standard_disarm_requests==0
            counts.standard_disarm_requests=1;disarmSentAt=io.now();
            try,request(400,zeros(1,7),'STANDARD_DISARM_BEFORE_EMERGENCY');
            catch e,event('STANDARD_DISARM_BEFORE_EMERGENCY_FAILED',[e.identifier ': ' e.message]);end
            waitFor(@disarmedGround,cfg.disarm_timeout_s);
        end
        observe();
        if stateFresh(latest)&&latest.armed==1
            counts.force_disarm_requests=counts.force_disarm_requests+1;
            event('EMERGENCY_LOGICAL_FORCE_DISARM_REJECTS_NORMAL_TERMINATION','Frozen normal recovery deadline elapsed; fresh virtual-only guards passed.');
            request(400,[0,21196,0,0,0,0,0],'EMERGENCY_LOGICAL_FORCE_DISARM');
        end
        assert(waitFor(@disarmedGround,cfg.disarm_timeout_s), ...
            'm600check:EmergencyDisarmUnconfirmed','No fresh disarmed/landed witness after emergency recovery.');
        zeroMappings();
    catch e,event('EMERGENCY_RECOVERY_NOT_CONFIRMED',[e.identifier ': ' e.message]);end
end
% An independent readback can establish safe shutdown after an ACK failure.
% Retain the preceding failure in the result.
finalParams=struct('name',{},'mav_type',{},'decoded',{},'raw_bits_hex',{});
try
    observe();assert(identityPass&&disarmedGround(latest),'m600check:RecoveryFinalStateUnknown','Final hardware safety state is unknown.');
    identitySent=io.now();io.requestCommand(512,[148,0,0,0,0,0,0]);
    assert(waitFor(@identityFresh,cfg.preflight_deadline_s)&&identityMatches(latest), ...
        'm600check:RecoveryFinalIdentityChanged','Final fresh identity differs.');
    names=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),"RA_CTRL_MODE"];
    for k=1:numel(names)
        p=io.readParameter(char(names(k)));
        finalParams(end+1)=struct('name',char(names(k)),'mav_type',p.mav_type, ...
            'decoded',p.decoded,'raw_bits_hex',p.raw_bits_hex); %#ok<AGROW>
        assert(p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000'), ...
            'm600check:RecoveryFinalMappingOrModeNotZero','Final typed mapping/native-control value is not zero.');
    end
    if geometryRequested
        restoreGeometry();
    end
    % Restore geometry and tuning independently after fresh identity, disarm,
    % ground, output and owner-release checks.
    if tuningRequested
        restoreTuning();
        try
            if strcmp(cfg.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
                parameterUnionRestored=verify_m600_delivery_parameter_union(io.readParameter, ...
                    cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'RESTORED');
            else
                parameterUnionRestored=m600check.verifyNativeHoverParameterUnion(io.readParameter, ...
                    cfg.native_hover_tuning,cfg.temporary_allocator_geometry,'RESTORED');
            end
            parameterUnionSafe=parameterUnionRestored.passed;
            if ~parameterUnionSafe
                parameterUnionFailure=['m600check:RecoveryOriginal138NotRestored: ' parameterUnionRestored.failure];
                if isempty(failure),failure=parameterUnionFailure;end
                event('ORIGINAL138_RESTORE_INCOMPLETE_KEEP_BRIDGE',parameterUnionFailure);
            end
        catch e
            parameterUnionSafe=false;parameterUnionFailure=[e.identifier ': ' e.message];
            if isempty(failure),failure=parameterUnionFailure;end
            event('ORIGINAL138_READBACK_EXCEPTION_KEEP_BRIDGE',parameterUnionFailure);
        end
    end
    assert(geometrySafe,'m600check:RecoveryGeometryNotRestored', ...
        'Exact original twelve-field allocator geometry restoration was not verified. Keep CopterSim alive.');
    assert(tuningSafe&&parameterUnionSafe,'m600check:RecoveryTuningOrUnionNotRestored', ...
        'Original tuning3 and complete original138 were not verified. Keep CopterSim alive.');
    observe();safe=disarmedGround(latest)&&identityMatches(latest)&&geometrySafe&&tuningSafe&&parameterUnionSafe;
catch e,event('RECOVERY_FINAL_READBACK_FAILED',[e.identifier ': ' e.message]);safe=false;end
closeIo();clear guard
receipt=struct('schema','M600_CANONICAL_INDEPENDENT_UDP_RECOVERY_V1', ...
    'status','RECOVERY_INCOMPLETE_KEEP_COPTERSIM_ALIVE','failure',failure, ...
    'identity_verified',identityPass,'safe_bridge_shutdown_allowed',safe&&closed, ...
    'fresh_disarmed_landed',safe,'fresh_disarmed_landed_observed',disarmedGround(latest), ...
    'udp_closed',closed,'counts',counts, ...
    'actions',actions,'events',events,'final_parameters',finalParams, ...
    'temporary_allocator_geometry_requested',geometryRequested, ...
    'geometry_restore_receipt',geometryReceipt,'geometry_restore_failure',geometryFailure, ...
    'geometry_restore_guards',geometryGuards,'geometry_original_identity_verified',geometrySafe, ...
    'native_hover_tuning_requested',tuningRequested,'tuning_restore_receipt',tuningReceipt, ...
    'tuning_restore_failure',tuningFailure,'tuning_original_identity_verified',tuningSafe, ...
    'native_hover_parameter_union_restored',parameterUnionRestored, ...
    'native_hover_parameter_union_failure',parameterUnionFailure, ...
    'native_hover_original138_restored',parameterUnionSafe, ...
    'shared_parameter_restore_guard_ledger',geometryGuards, ...
    'final_snapshot',latest,'evidence',io.evidence(), ...
    'COM_postflight_still_required',true,'physical_pwm_module_status_still_required',true, ...
    'plant_health_separate_from_hardware_recovery',true, ...
    'normal_termination_pass',safe&&geometrySafe&&tuningSafe&&parameterUnionSafe&& ...
        isempty(failure)&&isempty(geometryFailure)&&isempty(tuningFailure)&&isempty(parameterUnionFailure)&& ...
        counts.force_disarm_requests==0, ...
    'emergency_force_used',counts.force_disarm_requests>0, ...
    'landed_state_known',~isempty(latest)&&ismember(latest.landed_state,[1,2,3,4]), ...
    'COM_open_count',0,'CopterSim_started_or_stopped',false,'inner_result_reclassified',false);
if safe&&closed,receipt.status='SAFE_TO_STOP_OWNED_COPTERSIM__COM_POSTFLIGHT_REQUIRED';end

    function event(kind,detail),events(end+1)=struct('time_s',io.now(),'kind',kind,'detail',detail);end
    function observe()
        if io.now()-lastHeartbeat>=cfg.heartbeat_period_s
            io.sendHeartbeat();lastHeartbeat=io.now();
        end
        latest=io.snapshot();
    end
    function yes=stateFresh(s)
        now=io.now();yes=~isempty(s)&&all(isfinite([s.armed,s.landed_state, ...
            s.heartbeat_rx_s,s.extended_rx_s]))&& ...
            ismember(s.armed,[0,1])&&now>=s.heartbeat_rx_s&&now>=s.extended_rx_s&& ...
            now-s.heartbeat_rx_s<=cfg.heartbeat_max_age_s&&now-s.extended_rx_s<=cfg.landed_max_age_s;
    end
    function yes=disarmedGround(s),yes=stateFresh(s)&&s.armed==0&&s.landed_state==1;end
    function yes=identityFresh(s)
        yes=isfield(s,'autopilot_version')&&~isempty(s.autopilot_version)&& ...
            isfield(s,'autopilot_version_rx_s')&&s.autopilot_version_rx_s>=identitySent;
    end
    function yes=identityMatches(s)
        yes=false;if ~identityFresh(s),return;end
        v=s.autopilot_version;
        if ~all(isfield(v,{'uid','board_version','flight_custom_version'})),return;end
        hex=upper(reshape(dec2hex(uint8(v.flight_custom_version(:)),2).',1,[]));
        yes=strcmp(sprintf('%u',uint64(v.uid)),char(cfg.expected_uid))&& ...
            double(v.board_version)==cfg.expected_board_version&& ...
            strcmpi(hex,char(cfg.expected_flight_custom_version_hex));
    end
    function yes=waitFor(predicate,seconds)
        started=io.now();yes=false;
        while io.now()-started<seconds
            observe();if predicate(latest),yes=true;return;end
            io.sleep(cfg.poll_period_s);
        end
    end
    function request(command,parameters,label)
        observe();assert(stateFresh(latest),'m600check:RecoveryNoBlindCommand','No command is allowed with stale state.');
        sent=io.now();io.requestCommand(command,parameters);event([label '_REQUEST'],'');
        started=io.now();ack=[];
        while io.now()-started<cfg.command_timeout_s
            observe();ack=io.commandAck(command,sent);if ~isempty(ack),break;end
            io.sleep(cfg.poll_period_s);
        end
        assert(~isempty(ack),'m600check:RecoveryCommandAckMissing','A command acknowledgement was not received.');
        event([label '_ACK'],sprintf('result=%g',ack.result));
        assert(ack.result==0,'m600check:RecoveryCommandRejected','PX4 rejected the command.');
    end
    function zeroMappings()
        for k=1:6
            observe();assert(disarmedGround(latest),'m600check:RecoveryLostSafeState','Safe state became stale or changed.');
            name=sprintf('HIL_ACT_FUNC%d',k);p=io.readParameter(name);
            assert(p.mav_type==6&&ismember(p.decoded,[0,100+k]), ...
                'm600check:RecoveryUnexpectedMapping','Choose an unused output path.');
            if p.decoded==0,continue;end
            actions(end+1)=struct('name',name,'target',0,'attempted',true, ...
                'write_returned',false,'typed_readback_pass',false,'error',''); %#ok<AGROW>
            counts.mapping_zero_attempts=counts.mapping_zero_attempts+1;
            try
                p=io.setIntegerParameter(name,0);actions(end).write_returned=true;
                assert(p.mav_type==6&&p.decoded==0,'m600check:RecoveryMappingACKMismatch','Mapping response did not match zero INT32.');
                q=io.readParameter(name);
                assert(q.mav_type==6&&q.decoded==0&&strcmpi(q.raw_bits_hex,'00000000'), ...
                    'm600check:RecoveryMappingReadbackMismatch','Typed mapping readback did not match zero.');
                actions(end).typed_readback_pass=true;counts.mapping_zero_verified=counts.mapping_zero_verified+1;
            catch e,actions(end).error=[e.identifier ': ' e.message];rethrow(e);end
        end
    end
    function restoreGeometry()
        % Attempt parameter restoration for partial APPLY once the board is reachable,
        % disarmed and at zero output, including after software-plant failure.
        try
            assert(isstruct(cfg.temporary_allocator_geometry)&& ...
                isscalar(cfg.temporary_allocator_geometry)&& ...
                isfield(cfg.temporary_allocator_geometry,'entries'), ...
                'm600check:RecoveryGeometryContract','Missing exact temporary geometry entries.');
            assert(isfield(cfg,'independent_recovery_previous_owner_released')&& ...
                isequal(cfg.independent_recovery_previous_owner_released,true), ...
                'm600check:RecoveryPreviousOwnerNotReleased', ...
                'Outer must prove the prior child/handles exited before geometry recovery.');
            geometryReceipt=m600check.transferNativeAllocatorGeometry(io, ...
                cfg.temporary_allocator_geometry.entries,'RESTORE',@geometryWriteGuard);
            counts.geometry_restore_setter_attempts=geometryReceipt.setter_invocation_attempt_count;
            counts.geometry_restore_setter_returns=geometryReceipt.setter_return_count;
            counts.geometry_restore_ack_readback_verified=geometryReceipt.ack_and_readback_verified_count;
            counts.geometry_restore_no_op=geometryReceipt.no_op_count;
            geometrySafe=geometryReceipt.passed&&geometryReceipt.final_identity_verified;
            if ~geometrySafe
                if ~isempty(geometryReceipt.first_error)
                    a=geometryReceipt.first_error;geometryFailure=[a.identifier ': ' a.message];
                else
                    geometryFailure='m600check:RecoveryGeometryNotRestored: Exact original geometry not verified.';
                end
                if isempty(failure),failure=geometryFailure;end
                event('GEOMETRY_RESTORE_INCOMPLETE_KEEP_BRIDGE',geometryFailure);
            else
                event('GEOMETRY_ORIGINAL_TYPED_IDENTITY_RESTORED', ...
                    sprintf('attempts=%d verified=%d no_op=%d',counts.geometry_restore_setter_attempts, ...
                    counts.geometry_restore_ack_readback_verified,counts.geometry_restore_no_op));
            end
        catch e
            geometrySafe=false;geometryFailure=[e.identifier ': ' e.message];
            if isempty(failure),failure=geometryFailure;end
            event('GEOMETRY_RESTORE_EXCEPTION_KEEP_BRIDGE',geometryFailure);
        end
    end
    function restoreTuning()
        try
            assert(isfield(cfg,'independent_recovery_previous_owner_released')&& ...
                isequal(cfg.independent_recovery_previous_owner_released,true), ...
                'm600check:RecoveryPreviousOwnerNotReleased', ...
                'Outer must prove the prior child/handles exited before tuning recovery.');
            tuningReceipt=m600check.transferNativeHoverTuning(io, ...
                cfg.native_hover_tuning.entries,'RESTORE',@geometryWriteGuard);
            counts.tuning_restore_setter_attempts=tuningReceipt.setter_invocation_attempt_count;
            counts.tuning_restore_setter_returns=tuningReceipt.setter_return_count;
            counts.tuning_restore_ack_readback_verified=tuningReceipt.ack_and_readback_verified_count;
            counts.tuning_restore_no_op=tuningReceipt.no_op_count;
            tuningSafe=tuningReceipt.passed&&tuningReceipt.final_identity_verified;
            if ~tuningSafe
                if ~isempty(tuningReceipt.first_error)
                    a=tuningReceipt.first_error;tuningFailure=[a.identifier ': ' a.message];
                else
                    tuningFailure='m600check:RecoveryTuningNotRestored: Exact original tuning3 not verified.';
                end
                if isempty(failure),failure=tuningFailure;end
                event('TUNING_RESTORE_INCOMPLETE_KEEP_BRIDGE',tuningFailure);
            else
                event('TUNING_ORIGINAL_TYPED_IDENTITY_RESTORED', ...
                    sprintf('attempts=%d verified=%d no_op=%d',counts.tuning_restore_setter_attempts, ...
                    counts.tuning_restore_ack_readback_verified,counts.tuning_restore_no_op));
            end
        catch e
            tuningSafe=false;tuningFailure=[e.identifier ': ' e.message];
            if isempty(failure),failure=tuningFailure;end
            event('TUNING_RESTORE_EXCEPTION_KEEP_BRIDGE',tuningFailure);
        end
    end
    function yes=geometryWriteGuard()
        % Recovery checks hardware state independently of plant health and clock.
        % Require disabled output mappings and exclusive ownership.
        index=numel(geometryGuards)+1;
        geometryGuards(index)=struct('time_s',io.now(),'previous_owner_released',false, ...
            'before',[],'typed_zero_parameters',[],'after',[],'passed',false,'failure','', ...
            'output_guard_basis','ALL_HIL16_PWM8_MAPPING_ZERO_NATIVE_MODE_AND_EXCLUSIVE_RECOVERY_OWNER');
        yes=false;
        try
            owner=isfield(cfg,'independent_recovery_previous_owner_released')&& ...
                isequal(cfg.independent_recovery_previous_owner_released,true);
            geometryGuards(index).previous_owner_released=owner;
            assert(owner,'m600check:RecoveryPreviousOwnerNotReleased','Previous output owner not proven released.');
            observe();geometryGuards(index).before=hardwareState(latest);
            assert(disarmedGround(latest)&&identityMatches(latest), ...
                'm600check:RecoveryGeometryStateLost','Fresh same-identity disarmed/on-ground state required.');
            names=[compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8),"RA_CTRL_MODE"];
            rows=struct('name',{},'mav_type',{},'decoded',{},'raw_bits_hex',{});
            for gk=1:numel(names)
                p=io.readParameter(char(names(gk)));
                rows(end+1)=struct('name',char(names(gk)),'mav_type',p.mav_type, ...
                    'decoded',p.decoded,'raw_bits_hex',p.raw_bits_hex); %#ok<AGROW>
                geometryGuards(index).typed_zero_parameters=rows;
                assert(p.mav_type==6&&p.decoded==0&&strcmpi(p.raw_bits_hex,'00000000'), ...
                    'm600check:RecoveryGeometryOutputGuard','All 24 mappings and native mode must remain exact zero.');
            end
            observe();geometryGuards(index).after=hardwareState(latest);
            assert(disarmedGround(latest)&&identityMatches(latest), ...
                'm600check:RecoveryGeometryStateLost','State changed during zero-output readback.');
            yes=true;geometryGuards(index).passed=true;
        catch e
            geometryGuards(index).failure=[e.identifier ': ' e.message];
            event('GEOMETRY_WRITE_GUARD_REJECTED',geometryGuards(index).failure);
        end
    end
    function s=hardwareState(value)
        s=struct('captured_at_s',io.now(),'armed',value.armed,'landed_state',value.landed_state, ...
            'heartbeat_rx_s',value.heartbeat_rx_s,'extended_rx_s',value.extended_rx_s, ...
            'autopilot_version_rx_s',value.autopilot_version_rx_s, ...
            'autopilot_version',value.autopilot_version);
    end
    function closeIo()
        if done,return;end;done=true;
        try,closed=io.close();catch e,event('RECOVERY_UDP_CLOSE_FAILED',e.message);end
    end
end
