function result=run_delivery_ground_proof_tests()
% Test ground-proof logic with deterministic synthetic observations.
% Return the report for optional storage by the caller.
build=fileparts(fileparts(mfilename('fullpath')));
oldPath=path; cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'matlab_validation'),'-begin');
canonical=gpenmpc_external_path('delivery_method_project');
missionPath=fullfile(canonical,'config','CAMBRIDGE_REPRESENTATIVE_PHYSICAL_MISSION_V1.json');
lifecyclePath=fullfile(canonical,'config','DELIVERY_LIFECYCLE_V1.json');
mission=jsondecode(fileread(missionPath)); lifecycle=jsondecode(fileread(lifecyclePath));
checks=struct('name',{},'pass',{}); calls=0;
p=fixturePolicy(); r=fixtureRequest(1); lastProof=struct();
check('authoritative_mission_hash',strcmp(sha(missionPath),'E4B20DE53ACC4AA442ACD8262AE5639AEE1644505E12362C62520AE9F6CB6013'));
check('authoritative_lifecycle_hash',strcmp(sha(lifecyclePath),'B1CCE6C7C11CC87B23D23B7FC683039BD442C8DB5598F45F75DDFC1295295B61'));
check('current_A1_eight_seconds_four_deliveries',mission.physical_service_profile.ground_dwell_s==8 && ...
    mission.delivery_count==4 && lifecycle.service_dwell_s==8 && lifecycle.required_delivery_count==4);

[s,~,tokens]=drive([],0:.25:7.75,r,p); check('no_early_token',isempty(tokens));
[s,q,tok]=call(s,fixtureObservation(7.999),7.999,r,p);
check('7_999_is_not_eight',isempty(tok)&&q.continuous_dwell_s==7.999);
[s,q,tok]=call(s,fixtureObservation(8),8,r,p);
check('full_eight_seconds_one_release',~isempty(tok)&&q.token_issued_this_call&&tok.continuous_dwell_s==8 && s.release_count==1);
check('token_exact_generation_identity',tok.next_payload_generation==1 && strcmp(tok.service_id,r.service_id) && ...
    strcmp(tok.binding.uid,p.expected_uid)&&strcmp(tok.binding.boot_id,'boot-1')&&tok.binding.source_generation==1);
check('token_not_payload_execution_or_durable_permission',~tok.payload_execution_performed&&~tok.persistent_permission&& ...
    tok.requires_current_identity_freshness_dual_ground_recheck_at_consumption && ...
    tok.current_conditions_expire_no_later_than_s==8.5);
[s,~,more]=drive(s,8.25:.25:10,r,p);check('no_token_every_poll',isempty(more)&&s.release_count==1);
bad=fixtureObservation(10.25);bad.heartbeat.armed=1;
[s,~,tok]=call(s,bad,10.25,r,p);
check('issued_generation_survives_fault',isempty(tok)&&s.last_released_generation==1);
o=withEpoch(fixtureObservation(10.5),'session-2','boot-2',2);
[s,~,tok]=call(s,o,10.5,r,p);
check('issued_generation_survives_session_change',isempty(tok)&&s.last_released_generation==1);
r2=fixtureRequest(2); reused=r2;reused.service_id=r.service_id;
[~,q,tok]=call(s,withEpoch(fixtureObservation(10.75),'session-2','boot-2',2),10.75,reused,p);
check('service_identifier_not_reused_for_new_generation',isempty(tok)&&strcmp(q.status,'SERVICE_ID_ALREADY_RELEASED'));
skip=fixtureRequest(3);
[~,q,tok]=call(s,withEpoch(fixtureObservation(10.75),'session-2','boot-2',2),10.75,skip,p);
check('cannot_skip_payload_generation',isempty(tok)&&strcmp(q.status,'PAYLOAD_GENERATION_NOT_EXACT_NEXT'));

% Require a complete eight-second proof for each of four services.
s=[];totalTokens=struct([]);
for k=1:4
    [s,~,issued]=drive(s,(k-1)*9+(0:.25:8),fixtureRequest(k),p);
    check(sprintf('service_%d_exactly_one_token',k),numel(issued)==1&&issued.next_payload_generation==k&&issued.continuous_dwell_s==8);
    if isempty(totalTokens),totalTokens=issued;else,totalTokens(end+1)=issued;end %#ok<AGROW>
end
check('four_deliveries_not_poll_count',s.release_count==4&&isequal([totalTokens.next_payload_generation],1:4));
r5=fixtureRequest(5);[~,q,tok]=call(s,fixtureObservation(36),36,r5,p);
check('no_unregistered_fifth_generation',isempty(tok)&&strcmp(q.status,'SERVICE_REQUEST_UNKNOWN_OR_INVALID'));

% Every bad condition arrives just before expiry, then must lose all credit.
negativeNames={'heartbeat_stale','extended_stale','heartbeat_missing','extended_missing', ...
    'armed','arm_unknown_nan','arm_unknown_negative','land_unknown','airborne','landing', ...
    'identity_invalid','uid_wrong','message_uid_wrong','session_mismatch','boot_mismatch', ...
    'source_generation_mismatch','clock_mismatch','host_ground_flag_is_not_native', ...
    'heartbeat_invalid','extended_invalid','future_heartbeat','future_extended', ...
    'nonfinite_receive','plant_missing_when_required','plant_false_when_required'};
for k=1:numel(negativeNames)
    [s,~,~]=drive([],0:.25:7.75,r,p);o=fixtureObservation(8);
    switch negativeNames{k}
        case 'heartbeat_stale',o.heartbeat.received_at_s=7;
        case 'extended_stale',o.extended_sys_state.received_at_s=7;
        case 'heartbeat_missing',o=rmfield(o,'heartbeat');
        case 'extended_missing',o=rmfield(o,'extended_sys_state');
        case 'armed',o.heartbeat.armed=1;
        case 'arm_unknown_nan',o.heartbeat.armed=NaN;
        case 'arm_unknown_negative',o.heartbeat.armed=-1;
        case 'land_unknown',o.extended_sys_state.landed_state=0;
        case 'airborne',o.extended_sys_state.landed_state=2;
        case 'landing',o.extended_sys_state.landed_state=4;
        case 'identity_invalid',o.source_identity_valid=false;
        case 'uid_wrong',o.uid='3473490377090611258';
        case 'message_uid_wrong',o.heartbeat.uid='3473490377090611258';
        case 'session_mismatch',o.extended_sys_state.session_id='session-old';
        case 'boot_mismatch',o.heartbeat.boot_id='boot-old';
        case 'source_generation_mismatch',o.extended_sys_state.source_generation=2;
        case 'clock_mismatch',o.heartbeat.clock_id='different-clock';
        case 'host_ground_flag_is_not_native',o.extended_sys_state.message_name='PLANT_CONTACT';
        case 'heartbeat_invalid',o.heartbeat.valid=false;
        case 'extended_invalid',o.extended_sys_state.valid=false;
        case 'future_heartbeat',o.heartbeat.received_at_s=8.001;
        case 'future_extended',o.extended_sys_state.received_at_s=8.001;
        case 'nonfinite_receive',o.heartbeat.received_at_s=NaN;
        case 'plant_missing_when_required',o=rmfield(o,'plant_contact');
        case 'plant_false_when_required',o.plant_contact=false;
    end
    [s,q,tok]=call(s,o,8,r,p);
    check([negativeNames{k},'_denies_release'],isempty(tok)&&~q.current_native_ground_disarmed_evidence_valid&&s.dwell_s==0);
    [s,q,tok]=call(s,fixtureObservation(8.25),8.25,r,p);
    check([negativeNames{k},'_fresh_observation_has_zero_old_credit'],isempty(tok)&&q.continuous_dwell_s==0&&s.release_count==0);
end

for dimension={'session','boot','source_generation'}
    [s,~,~]=drive([],0:.25:7.75,r,p);o=fixtureObservation(8);
    switch dimension{1}
        case 'session',o=withEpoch(o,'session-2','boot-1',1);
        case 'boot',o=withEpoch(o,'session-1','boot-2',1);
        case 'source_generation',o=withEpoch(o,'session-1','boot-1',2);
    end
    [s,q,tok]=call(s,o,8,r,p);
    check([dimension{1},'_change_resets_continuity'],isempty(tok)&&q.continuous_dwell_s==0&&s.reset_count>=1);
end
[s,~,~]=drive([],0:.25:7.75,r,p);inactive=r;inactive.service_active=false;
[s,q,tok]=call(s,fixtureObservation(8),8,inactive,p);
check('outside_active_service_resets',isempty(tok)&&q.continuous_dwell_s==0);
[s,q,tok]=call(s,fixtureObservation(8.25),8.25,r,p);
check('reenter_service_no_old_credit',isempty(tok)&&q.continuous_dwell_s==0);

[s,~,~]=drive([],0:.25:7.75,r,p);
[s,q,tok]=call(s,fixtureObservation(7.5),7.5,r,p);
check('host_time_reversal_resets_with_highwater',isempty(tok)&&s.dwell_s==0&&strcmp(q.status,'CURRENT_TIME_REVERSED')&&s.last_eval_s==7.75);
[s,q,tok]=call(s,fixtureObservation(8),8,r,p);
check('after_reversal_new_full_window',isempty(tok)&&q.continuous_dwell_s==0);
for badTime=[NaN,Inf,-1]
    [s,~,~]=drive([],0:.25:7.75,r,p);
    [s,q,tok]=call(s,fixtureObservation(8),badTime,r,p);
    check(sprintf('invalid_clock_%g_resets',badTime),isempty(tok)&&s.dwell_s==0&&strcmp(q.status,'CURRENT_TIME_INVALID'));
end
[s,~,~]=drive([],0:.25:7.75,r,p);o=fixtureObservation(8);o.heartbeat.received_at_s=7.7;
[s,q,tok]=call(s,o,8,r,p);
check('receive_time_reversal_even_fresh_denied',isempty(tok)&&s.dwell_s==0&&strcmp(q.status,'NATIVE_MESSAGE_RECEIVE_TIME_REVERSED'));
[s,~,~]=drive([],0:.25:7.75,r,p);[s,q,tok]=call(s,fixtureObservation(8.5),8.5,r,p);
check('unobserved_gap_loses_old_credit',isempty(tok)&&q.continuous_dwell_s==0);
s=[];old=fixtureObservation(0);observedTokens=0;
for t=0:.25:8
    [s,~,tok]=call(s,old,t,r,p);observedTokens=observedTokens+~isempty(tok);
end
check('repeated_old_messages_never_make_eight_seconds',observedTokens==0&&s.dwell_s==0);
qNoPlant=p;qNoPlant.require_plant_contact=false;s=[];observedTokens=0;
for t=0:.25:8
    o=fixtureObservation(t);o.plant_contact=false;
    [s,q,tok]=call(s,o,t,r,qNoPlant);observedTokens=observedTokens+~isempty(tok);
end
check('plant_optional_not_substitute_for_native_evidence',observedTokens==1&&~q.payload_execution_performed);
o=fixtureObservation(8.25);o.heartbeat.armed=1;o.plant_contact=true;
[~,q,tok]=call(s,o,8.25,fixtureRequest(2),qNoPlant);
check('plant_true_cannot_override_px4_armed',isempty(tok)&&~q.current_native_ground_disarmed_evidence_valid);
badPolicy=p;badPolicy.service_dwell_s=10;
check('DV008_ten_seconds_not_silently_imported',throws(@()gpenmpcTaskIo.advanceDeliveryGroundProof([],fixtureObservation(0),0,r,badPolicy)));
[s,~,~]=drive([],0:.25:1,r,p);changedPolicy=p;changedPolicy.max_heartbeat_age_s=1;
check('policy_cannot_change_mid_dwell',throws(@()gpenmpcTaskIo.advanceDeliveryGroundProof(s,fixtureObservation(1.25),1.25,r,changedPolicy)));
check('proof_has_zero_execution_authority',lastProof.hardware_actions==0&&lastProof.parameter_mapping_write_count==0&& ...
    lastProof.arm_disarm_mode_request_count==0&&~lastProof.board_evidence_transport_implemented&&~lastProof.payload_execution_performed);
result=struct('status','PASS_HOST_ONLY_SYNTHETIC_GROUND_DWELL_PROOF_TESTS', ...
    'test_count',numel(checks),'passed',sum([checks.pass]),'checks',checks,'helper_calls',calls, ...
    'source_path',which('gpenmpcTaskIo.advanceDeliveryGroundProof'),'source_sha256',sha(which('gpenmpcTaskIo.advanceDeliveryGroundProof')), ...
    'test_source_sha256',sha([mfilename('fullpath'),'.m']),'service_dwell_s',8, ...
    'mission_path',missionPath,'mission_sha256',sha(missionPath), ...
    'lifecycle_path',lifecyclePath,'lifecycle_sha256',sha(lifecyclePath), ...
    'fixtures_not_live',true,'board_network_model_or_payload_execution',false, ...
    'claim','Deterministic ground-service state tests.');

    function check(name,condition)
        condition=isscalar(condition)&&logical(condition);
        checks(end+1)=struct('name',name,'pass',condition); %#ok<AGROW>
        assert(condition,'gpenmpcTaskIo:GroundProofTest','Failed: %s',name);
    end
    function [s,q,tok]=call(s,o,t,req,pol)
        [s,q,tok]=gpenmpcTaskIo.advanceDeliveryGroundProof(s,o,t,req,pol);
        calls=calls+1;lastProof=q;
    end
    function [s,q,issued]=drive(s,times,req,pol)
        issued=struct([]);q=struct();
        for instant=times
            [s,q,tok]=call(s,fixtureObservation(instant),instant,req,pol);
            if ~isempty(tok),if isempty(issued),issued=tok;else,issued(end+1)=tok;end,end %#ok<AGROW>
        end
    end
end
function p=fixturePolicy()
% Use synthetic age and gap values for branch coverage.
p=struct('service_dwell_s',8,'max_heartbeat_age_s',.6,'max_extended_age_s',.6, ...
    'max_observation_gap_s',.5,'clock_id','synthetic-monotonic-1', ...
    'expected_uid','1234605616436508552','require_plant_contact',true);
end
function r=fixtureRequest(g)
r=struct('service_id',sprintf('delivery-%d',g),'next_payload_generation',g,'service_active',true);
end
function o=fixtureObservation(t)
b=struct('uid','1234605616436508552','session_id','session-1','boot_id','boot-1', ...
    'source_generation',1,'clock_id','synthetic-monotonic-1');
o=b;o.source_identity_valid=true;o.plant_contact=true;
h=b;h.received_at_s=t;h.valid=true;h.message_name='HEARTBEAT';h.armed=false;
e=b;e.received_at_s=t;e.valid=true;e.message_name='EXTENDED_SYS_STATE';e.landed_state=1;
o.heartbeat=h;o.extended_sys_state=e;
end
function o=withEpoch(o,session,boot,generation)
for name={'','heartbeat','extended_sys_state'}
    if isempty(name{1}),x=o;else,x=o.(name{1});end
    x.session_id=session;x.boot_id=boot;x.source_generation=generation;
    if isempty(name{1}),o=x;else,o.(name{1})=x;end
end
end
function yes=throws(action)
yes=false;try,action();catch,yes=true;end
end
function value=sha(p)
fid=fopen(p,'rb');assert(fid>=0,'Cannot read source.');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');
md.update(bytes);value=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
