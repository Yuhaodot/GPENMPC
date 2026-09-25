function report=test_rfly_host_exchange(outputRoot)
% Real existing async coordinator, explicit HOST session/raw/ACK fixtures.
% C++ owns fixture registration, private source, assembly and actual C step.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path; %#ok<NASGU> Caller batch owns process path; retain class definitions through destructor.
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot));mkdir(outputRoot);checks=struct('name',{},'pass',{});raw=struct();
probe=fullfile(gpenmpc_external_path('rfly_board_commit_consumer'),'actual_exchange_probe');
kernel=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_ARGUMENTS_AND_EXPECTED.bin');
states=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_NED_AND_MAPPED_STATE.bin');
args=javaArray('java.lang.String',9);
values={'wsl.exe','-d','Ubuntu-24.04','--',linux(probe),linux(kernel),linux(states),linux(outputRoot),'serve'};
for k=1:9,args(k)=java.lang.String(values{k});end
builder=java.lang.ProcessBuilder(args);builder.redirectErrorStream(true);process=builder.start();
processCleanup=onCleanup(@()process.destroy()); %#ok<NASGU>
reader=java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream()));
writer=java.io.PrintWriter(process.getOutputStream(),true);
raw.cpp_export=string(reader.readLine());echoBytes=readBytes(fullfile(outputRoot,'ACTUAL_FIXTURE_ECHO119.bin'));
check('actual_cpp_register_confirm_echo119',numel(echoBytes)==119&&raw.cpp_export=="READY_ACTUAL_FIXTURE_SESSION_AND_SOURCES");
a=gpenmpcNative.loadCanonicalAssets();
echo=struct('identity_semantics','BoardRegisteredExecutionSessionGenerationV1', ...
    'host_challenge',decode(echoBytes(6:21),'uint64'),'uid',decode(echoBytes(22:29),'uint64'), ...
    'system',echoBytes(30),'component',echoBytes(31),'board_registration_hrt_us',decode(echoBytes(32:39),'uint64'), ...
    'process_session_generation',decode(echoBytes(40:47),'uint64'), ...
    'link_lifecycle_generation',decode(echoBytes(48:55),'uint64'), ...
    'configuration_payload_sha256',hex(echoBytes(56:87)));
sessionSha=hex(echoBytes(88:119));now=gpenmpcNative.rflyOriginalHostMonotonicNs();
assoc=struct('schema','GPENMPC_RFLY_REGISTERED_SESSION_ASSOCIATION_V1', ...
    'registration_result','Registered','echo_confirmation_result','Confirmed','echo',echo, ...
    'original_host_challenge',echo.host_challenge,'original_host_receive_ns',now, ...
    'execution_session_sha256',sessionSha,'provenance','ACTUAL_EXISTING_SESSION_BOUND_GUARD_HOST_FIXTURE');
e=struct('uid',string(echo.uid),'system_id',1,'component_id',1,'boot_generation',echo.process_session_generation, ...
    'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8,'canonical_package_root',gpenmpcNative.canonicalAssetRoot(), ...
    'configuration_payload_sha256',echo.configuration_payload_sha256,'initial_payload_kg',2.27, ...
    'async_outer_required',true,'task_identity_sha256',repmat('C',1,64), ...
    'inner_control_contract','CANONICAL_FULL_SE3','full_inner_evidence_scope','BOARD_COMMIT_RFC1');
e.rfly_board_commit=struct('execution_session_sha256',sessionSha,'identity_semantics',echo.identity_semantics, ...
    'link_lifecycle_generation',echo.link_lifecycle_generation,'board_registration_hrt_us',echo.board_registration_hrt_us, ...
    'reference_max_age_us',uint64(400000),'outer_max_age_us',uint64(400000), ...
    'generated_arm_source_sha256','A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F', ...
    'wrapper_matlab_source_sha256','9117D3CDF8E924A253F4C444CDD467E4850D6F11CBF5744B26375B950E2A95B5');
origin=struct('link_lifecycle_generation',echo.link_lifecycle_generation,'execution_session_sha256',sessionSha, ...
    'source_system',uint8(1),'source_component',uint8(1));
sbytes=cell(1,3);for k=1:3,sbytes{k}=readBytes(fullfile(outputRoot,"SOURCE_"+k+".bin"));end
s=gpenmpcNative.RflySnapshotSample(sbytes{1},now,e);[state,ok]=gpenmpcNative.px4EstimateState(s,e,now);assert(ok);
coef=zeros(3,8);coef(:,1)=state(1:3);coef(:,2)=[.1;0;0];tr=struct('total_duration_s',10,'coefficients_ascending',coef);
dialect=mavlinkdialect('common.xml',2);
serializer=mavlinkio(dialect,SystemID=42,ComponentID=191,ComponentType='MAV_TYPE_GCS',AutopilotType='MAV_AUTOPILOT_INVALID');
bad=assoc;bad.echo.process_session_generation=echo.process_session_generation+uint64(1);
check('altered_registered_generation_rejected',rejects(@()gpenmpcNative.RflyHostExchangeService(build,e,tr,bad,serializer,dialect,8)));
bad=assoc;bad.original_host_challenge(1)=bad.original_host_challenge(1)+uint64(1);
check('challenge_echo_mismatch_rejected',rejects(@()gpenmpcNative.RflyHostExchangeService(build,e,tr,bad,serializer,dialect,8)));
x=gpenmpcNative.RflyHostExchangeService(build,e,tr,assoc,serializer,dialect,8);
xcleanup=onCleanup(@()x.close()); %#ok<NASGU>
now=gpenmpcNative.rflyOriginalHostMonotonicNs();[v,w]=observed(63,now);
raw.prepare=x.prepareSnapshot(sbytes{1},now,now,v,w,origin);
check('real_existing_preparation_and_single_worker',raw.prepare.status=="PASS_HOST_PREPARATION" ...
    &&x.status().coordinator.owned_outer_services==1&&x.status().owned_io_endpoints==0);
now=gpenmpcNative.rflyOriginalHostMonotonicNs();[v,w]=observed(64,now);
raw.source2=x.source(sbytes{2},now,now,v,w,origin);
check('first_async_submit_no_uncommitted_payload',raw.source2.status=="WAITING_FOR_COMMITTED_OUTER" ...
    &&isempty(raw.source2.packets)&&x.status().coordinator.actual_solver_submissions==1);
raw.polls={};timer=tic;
while x.status().context.outer_commits==0&&toc(timer)<3
    pause(.001);raw.polls{end+1}=x.poll(gpenmpcNative.rflyOriginalHostMonotonicNs()); %#ok<AGROW>
end
check('actual_async_response_bound_to_original_ticket',x.status().context.outer_commits==1 ...
    &&x.status().coordinator.service.fresh_outer_for_current_leg);
now=gpenmpcNative.rflyOriginalHostMonotonicNs();[v,w]=observed(65,now);
raw.source3=x.source(sbytes{3},now,now,v,w,origin);
check('held_outer_current_reference_and_actual_source_tags',raw.source3.status=="PREPARED_CONTEXT_AND_COMMAND_BYTES_NO_SEND" ...
    &&numel(raw.source3.packets)==7&&raw.source3.binding.command_generation==65 ...
    &&raw.source3.binding.reference_generation==1&&raw.source3.binding.outer_generation==1 ...
    &&~isequal(raw.source3.context.outer_source_ticket,raw.source3.context.reference_source_ticket));
check('RSP1_not_relabelled_as_MAVLink331',raw.source3.original_source.source=="PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1" ...
    &&raw.source3.prepared.full_inner_command.evidence_scope=="BOARD_COMMIT_RFC1");
writeBytes(fullfile(outputRoot,'CONTEXT316.bin'),raw.source3.context_bytes);
writeBytes(fullfile(outputRoot,'COMMAND381.bin'),raw.source3.command_bytes);
raw.original_submit_ns=gpenmpcNative.rflyOriginalHostMonotonicNs();
try,raw.submitted=x.submitted(uint64(65),raw.original_submit_ns);
catch ex,save(fullfile(outputRoot,'SUBMISSION_FAILURE.mat'),'raw','checks');rethrow(ex);end
writer.println('execute');line=string(reader.readLine());disp(line);raw.cpp_execute=jsondecode(line);
msg=readBytes(fullfile(outputRoot,'FEEDBACK1112.bin'));rx=gpenmpcNative.rflyOriginalHostMonotonicNs();
raw.decoded=gpenmpcNative.RflyCommittedFeedbackDecoder(msg,rx);
try,raw.feedback=x.feedback(msg,rx,rx,origin);
catch ex,raw.failure_status=x.status();save(fullfile(outputRoot,'FEEDBACK_FAILURE.mat'),'raw','checks');rethrow(ex);end
for k=1:7
    [m,status]=deserializemsg(dialect,raw.source3.packets{k},OutputAllMessage=true);
    check("actual_MATLAB_MAVLink_frame_"+k,numel(m)==1&&status==0&&m.Payload.payload_type==42002);
end
check('actual_C_RFC1_updates_same_owned_service',raw.feedback.accepted ...
    &&x.status().coordinator.service.physical_runtime.control_commit_count==1 ...
    &&raw.cpp_execute.actual_generated_c_steps==1&&raw.cpp_execute.failed==0);
check('held_outer_deadline_not_renewed_by_new_reference',raw.decoded.token.outer_valid_until_us==uint64(1410000) ...
    &&raw.decoded.token.reference_valid_until_us==uint64(1420000));
check('registered_fixture_generation_not_host_constant42',raw.decoded.token.identity.boot_generation==echo.process_session_generation ...
    &&echo.process_session_generation==1&&raw.decoded.token.sample_generation==65);
check('single_original_scheduler_no_extra_solver_from_reference_poll',x.status().coordinator.actual_solver_submissions==1 ...
    &&all(cellfun(@(p)p.source_clock_unchanged,raw.polls)));
check('no_production_bootstrap_physical_or_transport_claim',~x.status().live_registration_proven ...
    &&~x.status().transport_source_authenticated&&~x.status().publication_authority);
check('repeat_feedback_rejected_and_exchange_closes',rejects(@()x.feedback(msg,rx,rx,origin))&&x.status().closed);
raw.legacy_outer_snapshot=test_canonical_outer_snapshot(a);
report=struct('status','PASS_SINGLE_OWNER_HOST_EXCHANGE_WITH_ACTUAL_ASYNC_SOLVER', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'actual_async_submissions',1,'actual_generated_c_steps',1,'actual_cpp_registration_fixture',true, ...
    'original_registered_association',assoc,'live_registration_proven',false,'physical_authorization',false, ...
    'publication_authority',false,'hardware_actions',0,'plant_steps',0,'owned_coordinators',1, ...
    'final_status',x.status());
save(fullfile(outputRoot,'RAW.mat'),'raw','report','-v7.3');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');g=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear g
disp(jsonencode(report));
    function check(n,ok)
        checks(end+1)=struct('name',n,'pass',logical(ok)); %#ok<AGROW>
        if ~ok,save(fullfile(outputRoot,'FAILURE.mat'),'raw','checks');end
        assert(ok,'gpenmpcNative:ExchangeTest','%s',n);
    end
    function [v,w]=observed(g,now)
        v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',true,'generation',g,'rx_ns',double(now), ...
            'ordering','SOFTWARE_M600_ORDER','rotor_thrust_state_n',ones(6,1)*20,'thrust_effectiveness',ones(6,1), ...
            'state_source','SAME_M600_ACCEPTED_STEP_ROTOR_LAG_STATE','plant_session_id',1);
        w=struct('source','FROZEN_TASK_WIND_ESTIMATOR','valid',true,'generation',g,'rx_ns',double(now),'estimate_xy_mps',zeros(2,1));
    end
end
function s=linux(p),p=char(p);s=['/mnt/' lower(p(1)) strrep(p(3:end),'\','/')];end
function b=readBytes(p),f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end %#ok<NASGU>
function writeBytes(p,b),f=fopen(p,'wb');assert(f>=0);g=onCleanup(@()fclose(f));fwrite(f,b,'uint8');end %#ok<NASGU>
function v=decode(b,kind),v=typecast(b,kind);[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);end
function h=hex(b),h=upper(reshape(dec2hex(b,2).',1,[]));end
function yes=rejects(f),yes=false;try,f();catch,yes=true;end;end
