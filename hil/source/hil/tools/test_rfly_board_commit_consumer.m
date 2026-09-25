function report=test_rfly_board_commit_consumer(outputRoot)
% Test generated-C feedback decoding with mock ACK and private-source fixtures.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path;restore=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot),'Preserve earlier results.');mkdir(outputRoot);
base=fullfile(gpenmpc_external_path('rfly_board_commit_consumer'));
exe=fullfile(base,'actual_runtime_feedback_probe.exe');assert(isfile(exe));
kernel=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_ARGUMENTS_AND_EXPECTED.bin');
states=fullfile(gpenmpc_external_path('full_inner_px4_float_mapping'),'MATLAB_NED_AND_MAPPED_STATE.bin');
a=gpenmpcNative.loadCanonicalAssets();task=repmat('C',1,64);
archive=load(fullfile(gpenmpc_external_path('runtime_alignment_fixture'),'RAW.mat'),'raw');
[outer4,outerReceipt]=gpenmpcNative.RflyOuterPayload(archive.raw.first);
checks=struct('name',{},'pass',{});raw=cell(3,1);max61=0;maxState=0;saturatedCases=0;
legacyAdapter=fileread(fullfile(parent,'src','+gpenmpcNative','stepCoordinatedPhysicalAdapter.m'));
boardAdapter=fileread(which('gpenmpcNative.stepCoordinatedPhysicalAdapterBoard'));
legacyAdapter=strrep(legacyAdapter,char(13),'');boardAdapter=strrep(boardAdapter,char(13),'');
check('scientific_augmentation_body_exact_source_text',isequal( ...
    regexp(legacyAdapter,'plantState = zeros[\s\S]*?transform = diag','match','once'), ...
    regexp(boardAdapter,'plantState = zeros[\s\S]*?transform = diag','match','once')));
check('evidence_validation_exact_source_text',isequal( ...
    regexp(legacyAdapter,'function ok = validEvidence[\s\S]*?function output = rejected','match','once'), ...
    regexp(boardAdapter,'function ok = validEvidence[\s\S]*?function output = rejected','match','once')));
dialect=mavlinkdialect('common.xml',2);
serializer=mavlinkio(dialect,SystemID=42,ComponentID=191,ComponentType='MAV_TYPE_GCS',AutopilotType='MAV_AUTOPILOT_INVALID');
rows=[0,62,64];rx=uint64(9000000000);
for j=1:3
    folder=fullfile(outputRoot,"ROW_"+(rows(j)+1));mkdir(folder);runProbe(folder,rows(j),"export");
    sourceBytes=readBytes(fullfile(folder,'SOURCE.bin'));
    d=gpenmpcNative.RflySnapshotDecoder(sourceBytes,rx);
    e=struct('uid',string(d.observed_uid),'system_id',1,'component_id',1, ...
        'boot_generation',d.observed_boot_generation,'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8, ...
        'initial_payload_kg',2.27,'canonical_package_root',gpenmpcNative.canonicalAssetRoot(), ...
        'configuration_payload_sha256',a.binding.effective_configuration_payload_sha256, ...
        'inner_control_contract','CANONICAL_FULL_SE3','full_inner_evidence_scope','BOARD_COMMIT_RFC1');
    e.rfly_board_commit=struct('execution_session_sha256',repmat('E',1,64), ...
        'identity_semantics','BoardRegisteredExecutionSessionGenerationV1', ...
        'link_lifecycle_generation',uint64(3),'board_registration_hrt_us',uint64(900000), ...
        'reference_max_age_us',uint64(400000),'outer_max_age_us',uint64(400000), ...
        'generated_arm_source_sha256','A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F', ...
        'wrapper_matlab_source_sha256','9117D3CDF8E924A253F4C444CDD467E4850D6F11CBF5744B26375B950E2A95B5');
    s=gpenmpcNative.RflySnapshotSample(sourceBytes,rx,e);
    check("nonzero_origin_"+j,isequal(s.task_origin_ned_m,[7;-11;3]) ...
        &&isequal(s.position_ned_m,double(s.raw13_float32(1:3))-s.task_origin_ned_m));
    [x,accepted]=gpenmpcNative.px4EstimateState(s,e,rx+uint64(1000));assert(accepted);
    ref=struct('position_m',x(1:3)+[.2;-.1;.3],'velocity_mps',zeros(3,1), ...
        'acceleration_mps2',[0;0;100],'jerk_mps3',zeros(3,1));
    rt=runtimeInput(double(s.position_generation),rx);
    board=gpenmpcNative.CurrentPhysicalCausalRuntime(build,e,task,a);
    b=board.beginSample(uint64(s.position_generation),rx+uint64(1000),s,rt,ref,.01);
    assert(b.accepted,'Begin: %s',b.reason);cmd=b.full_inner_command;
    check("source_generation_not_internal_ordinal_"+j,cmd.generation==uint64(s.position_generation) ...
        &&cmd.augmentation_state_generation==cmd.generation&&cmd.continuity_state_generation==cmd.generation ...
        &&b.physical_output.generation==1&&b.physical_output.source_generation==cmd.generation);
    check("no_scope_publication_authority_"+j,~b.publication_allowed&&~b.host_kernel_execution_allowed ...
        &&b.board_commit_input_prepared&&~b.board_publication_authority&&~b.live_registration_proven);
    % Compare identical numerical state using a separate legacy-observation fixture.
    eh=e;eh.full_inner_evidence_scope='HOST_ONLY_KERNEL_EXECUTION';
    sh=s;sh.source='PX4_EKF2_MAVLINK_ODOMETRY_331';sh.odometry_reset_counter=s.reset_counter;
    sh.odometry_frame_id=1;sh.odometry_child_frame_id=1;sh.odometry_estimator_type=8;
    sh.position_generation=1;sh.attitude_generation=1;sh.rates_generation=1;
    host=gpenmpcNative.CurrentPhysicalCausalRuntime(build,eh,task,a);
    bh=host.beginSample(uint64(1),rx+uint64(1000),sh,runtimeInput(1,rx),ref,.01);
    assert(bh.accepted,'Offline HOST comparison: %s',bh.reason);
    check("unchanged_scientific_adapter_same_input_"+j,isequal(b.physical_output.compensation_up_mps2,bh.physical_output.compensation_up_mps2) ...
        &&isequal(b.physical_output.diagnostic,bh.physical_output.diagnostic));
    c=struct('reference_generation',uint64(1),'outer_generation',uint64(1), ...
        'reference_source_ticket',s.source_ticket,'outer_source_ticket',s.source_ticket, ...
        'configuration_sha256',d.configuration_sha256,'reference_source_receipt_ns',rx, ...
        'reference_creation_ns',rx+uint64(100000),'reference_expiry_ns',rx+uint64(400000000), ...
        'outer_source_receipt_ns',rx,'outer_creation_ns',rx+uint64(50000), ...
        'outer_expiry_ns',rx+uint64(400000000),'reference_ned',[diag([1,1,-1])*ref.position_m; ...
        diag([1,1,-1])*ref.velocity_mps;diag([1,1,-1])*ref.acceleration_mps2;0;0], ...
        'outer_payload',outer4,'target_system',uint8(1),'target_component',uint8(1));
    binding=struct('command_generation',cmd.generation,'snapshot_ticket',s.source_ticket, ...
        'reference_generation',c.reference_generation,'outer_generation',c.outer_generation, ...
        'configuration_sha256',c.configuration_sha256,'target_system',uint8(1),'target_component',uint8(1));
    assoc=struct('outer_source_sample',s,'context',c,'binding',binding);
    [full,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(cmd,a);
    [~,~,contextBytes]=gpenmpcNative.RflyContextEncoder(c,serializer,dialect);
    [~,slimReceipt,~,slim]=gpenmpcNative.RflySlimCommandEncoder(cmd,a,binding,serializer,dialect);
    check("slim_original_state_tags_and_no_publish_"+j,isequal(slim(246:261),be([cmd.generation;cmd.generation])) ...
        &&~slimReceipt.publication_authority&&slimReceipt.board_access==0);
    writeBytes(fullfile(folder,'COMMAND829.bin'),full);writeBytes(fullfile(folder,'CONTEXT316.bin'),contextBytes);
    submitted=board.recordBoardCommandSubmission(cmd.generation,assoc,rx+uint64(200000));assert(submitted.pending);
    runProbe(folder,rows(j),"execute");message=readBytes(fullfile(folder,'FEEDBACK1112.bin'));
    feedback=gpenmpcNative.RflyCommittedFeedbackDecoder(message,rx+uint64(1000000));
    actual=fromBE(readBytes(fullfile(folder,'ACTUAL61.bin')),'double');
    check("decoder_actual61_verbatim_"+j,isequal(typecast(feedback.actual61,'uint64'),typecast(actual,'uint64')));
    env=struct('schema','GPENMPC_RFLY_BOARD_COMMIT_INGRESS_V1','message',message, ...
        'original_host_receive_ns',rx+uint64(1000000),'origin',struct( ...
        'link_lifecycle_generation',uint64(3),'execution_session_sha256',repmat('E',1,64), ...
        'source_system',uint8(1),'source_component',uint8(1)));
    cb=board.commitControl(cmd.generation,rx+uint64(2000000),env);
    assert(cb.accepted,'BOARD consume: %s',board.status().failure_code);
    hf=gpenmpcNative.executeCanonicalFullInnerKernelHost(bh.full_inner_command,a,rx+uint64(1000000));
    ch=host.commitControl(uint64(1),rx+uint64(2000000),hf);assert(ch.accepted);
    oracle=[hf.wrench_n_nm;hf.rotor_command_n;hf.diagnostic51];
    max61=max(max61,max(abs(actual-oracle)));
    ob=board.outerRuntimeState();oh=host.outerRuntimeState();
    compareFields={'previous_desired_force_projected_n','previous_committed_rotor_command_n', ...
        'runtime_vertical_observer_shadow_i_mps2','residual_history_f_mps2', ...
        'runtime_gp_axis_weight_f','runtime_gp_responsibility_blend','runtime_gp_filtered_mean_f_mps2'};
    for q=1:numel(compareFields),maxState=max(maxState,max(abs(ob.(compareFields{q})(:)-oh.(compareFields{q})(:))));end
    check("actual_outputs_drive_previous_force_rotor_"+j,isequal(ob.previous_desired_force_projected_n,actual(14:16)) ...
        &&isequal(ob.previous_committed_rotor_command_n,actual(5:10))&&ob.desired_force_generation==double(cmd.generation));
    saturatedCases=saturatedCases+double(actual(59)>0&&actual(61)==1&&board.status().robust_authority_scale<1);
    check("same_actual_robust_observe_control_"+j, ...
        abs(board.status().robust_authority_scale-host.status().robust_authority_scale)<1e-12);
    check("board_logical_counter_distinct_from_source_"+j,feedback.token.transaction==1 ...
        &&feedback.token.output_generation==1&&feedback.token.sample_generation==cmd.generation ...
        &&~cb.board_commit_observation.wire_command_generation_present);
    check("no_live_or_physical_claim_"+j,~cb.board_consumption_proven ...
        &&~cb.board_commit_observation.transport_source_authenticated ...
        &&~cb.board_commit_observation.publication_authority&&~board.status().final_hil_admission);
    check("same_original_conservative_deadlines_"+j,feedback.token.reference_valid_until_us==uint64(1400000) ...
        &&feedback.token.outer_valid_until_us==uint64(1400000)&&feedback.original_valid_until_us==uint64(1004000));
    raw{j}=struct('source',s,'begin',b,'context',c,'binding',binding,'feedback',feedback, ...
        'board_commit',cb,'host_feedback',hf,'board_status',board.status(),'host_status',host.status());
    runProbe(folder,rows(j),"export-next");nextRx=rx+uint64(10000000);
    sn=gpenmpcNative.RflySnapshotSample(readBytes(fullfile(folder,'SOURCE_NEXT.bin')),nextRx,e);
    bn=board.beginSample(uint64(sn.position_generation),nextRx+uint64(1000),sn, ...
        runtimeInput(double(sn.position_generation),nextRx),ref,.01);
    hn=sn;hn.source='PX4_EKF2_MAVLINK_ODOMETRY_331';hn.odometry_reset_counter=sn.reset_counter;
    hn.odometry_frame_id=1;hn.odometry_child_frame_id=1;hn.odometry_estimator_type=8;
    hn.position_generation=2;hn.attitude_generation=2;hn.rates_generation=2;
    hbn=host.beginSample(uint64(2),nextRx+uint64(1000),hn,runtimeInput(2,nextRx),ref,.01);
    assert(bn.accepted&&hbn.accepted,'Next original source closes actual prior-output interval.');
    check("next_observer_interval_closes_with_actual61_"+j,board.status().closed_evidence_count==1 ...
        &&host.status().closed_evidence_count==1 ...
        &&max(abs(bn.physical_output.compensation_up_mps2-hbn.physical_output.compensation_up_mps2))<1e-10 ...
        &&max(abs(board.status().residual_history_f_mps2-host.status().residual_history_f_mps2))<1e-10 ...
        &&bn.physical_output.source_generation==uint64(sn.position_generation)&&bn.physical_output.generation==2);
    raw{j}.next_board_begin=bn;raw{j}.next_host_begin=hbn;
    if j==1
        for k=1:13
            consumer=gpenmpcNative.RflyBoardCommitConsumer(e,task,a);
            consumer.submitted(cmd,s,assoc,rx+uint64(200000));bad=env;time=rx+uint64(2000000);
            switch k
                case 1,bad.message(5)=2;bad.message=rehash(bad.message);
                case 2,bad.message(5)=3;bad.message=rehash(bad.message);
                case 3,bad.message(6)=bitxor(bad.message(6),uint8(1));bad.message=rehash(bad.message);
                case 4,bad.message(369)=bitxor(bad.message(369),uint8(1));bad.message=rehash(bad.message);
                case 5,bad.origin.link_lifecycle_generation=uint64(4);
                case 6,bad.origin.execution_session_sha256=repmat('F',1,64);
                case 7,bad.message(497:504)=be(NaN);bad.message=rehash(bad.message);
                case 8,time=rx+uint64(100000001);
                case 9,bad.original_host_receive_ns=rx+uint64(199999);
                case 10,bad.message(57:64)=be(uint64(0));bad.message=rehash(bad.message);
                case 11,bad.message(137:144)=be(uint64(2));bad.message=rehash(bad.message);
                case 12,consumer.revoke();
                case 13,bad.message(end)=bitxor(bad.message(end),uint8(1));
            end
            [ok,~]=consumer.consume(bad,time);check("negative_actual_feedback_"+k,~ok&&consumer.status().failed);
            [again,~]=consumer.consume(env,rx+uint64(3000000));check("first_fault_latched_"+k,~again);
        end
        consumer=gpenmpcNative.RflyBoardCommitConsumer(e,task,a);consumer.submitted(cmd,s,assoc,rx+uint64(200000));
        [ok,~]=consumer.consume(env,rx+uint64(2000000));assert(ok);
        [ok,~]=consumer.consume(env,rx+uint64(3000000));check('same_feedback_consumed_once',~ok);
        badSource=s;badSource.position_ned_m=double(s.raw13_float32(1:3))+s.task_origin_ned_m;
        consumer=gpenmpcNative.RflyBoardCommitConsumer(e,task,a);
        check('wrong_origin_sign_pending_rejected',rejects(@()consumer.submitted(cmd,badSource,assoc,rx+uint64(200000))));
        consumer=gpenmpcNative.RflyBoardCommitConsumer(e,task,a);[ok,~]=consumer.consume(env,rx+uint64(2000000));
        check('unsubmitted_feedback_not_admitted',~ok);
        badCommand=cmd;badCommand.augmentation_state_generation=cmd.generation+uint64(1);
        check('augmentation_generation_not_virtual_counter',rejects(@()gpenmpcNative.encodeCanonicalFullInnerArguments(badCommand,a)));
    end
end
check('actual_generated61_offline_numerical_parity',max61<1e-10);
check('same_robust_observer_and_causal_prediction_inputs',maxState<1e-10);
check('projection_saturation_and_canonical_authority_memory_exercised',saturatedCases>=1);
report=struct('status','PASS_HOST_ACTUAL_RFC1_TO_CAUSAL_OBSERVER','checks',checks, ...
    'test_count',numel(checks),'pass_count',sum([checks.pass]),'actual_generated_c_steps',3, ...
    'maximum_actual61_vs_offline_host_error',max61,'maximum_causal_state_error',maxState, ...
    'mock_commit_ack',true,'private_source_fixture',true,'nonzero_task_origin_ned_m',[7;-11;3], ...
    'archived_outer_receipt',outerReceipt,'live_registration_proven',false,'board_consumption_proven',false, ...
    'hardware_actions',0,'plant_steps',0,'new_solver_calls',0);
report.source_sha256=struct('original_adapter',gpenmpcNative.fileSha256(fullfile(parent,'src','+gpenmpcNative','stepCoordinatedPhysicalAdapter.m')), ...
    'board_adapter',gpenmpcNative.fileSha256(which('gpenmpcNative.stepCoordinatedPhysicalAdapterBoard')), ...
    'consumer',gpenmpcNative.fileSha256(which('gpenmpcNative.RflyBoardCommitConsumer')), ...
    'runtime',gpenmpcNative.fileSha256(which('gpenmpcNative.CurrentPhysicalCausalRuntime')));
save(fullfile(outputRoot,'RAW.mat'),'raw','report','-v7.3');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear g;disp(jsonencode(report));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpcNative:BoardCommitTest','%s',name);
    end
    function runProbe(folder,row,mode)
        command=sprintf('"%s" "%s" "%s" "%s" %d %s',exe,kernel,states,folder,row,mode);
        [code,out]=system(command);disp(out);assert(code==0,'%s',out);
    end
    function rt=runtimeInput(g,now)
        v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',true,'generation',g, ...
            'rx_ns',double(now),'ordering','SOFTWARE_M600_ORDER','rotor_thrust_state_n',ones(6,1)*20);
        w=struct('source','FROZEN_TASK_WIND_ESTIMATOR','valid',true,'generation',g, ...
            'rx_ns',double(now),'estimate_xy_mps',zeros(2,1));
        rt=struct('schema','GPENMPC_PHYSICAL_CAUSAL_RUNTIME_INPUT_V1', ...
            'task_identity_sha256',task,'plant_truth_used',false,'virtual_actuator',v,'wind',w, ...
            'task',struct('source','FROZEN_GPENMPC_TASK_STATE','payload_kg',2.27,'leg_index',1));
    end
end
function b=readBytes(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function writeBytes(p,b)
f=fopen(p,'wb');assert(f>=0);g=onCleanup(@()fclose(f));assert(fwrite(f,b,'uint8')==numel(b)); %#ok<NASGU>
end
function b=be(v)
[~,~,e]=computer;if e=='L',v=swapbytes(v);end;b=reshape(typecast(v(:),'uint8'),[],1);
end
function v=fromBE(b,kind)
v=typecast(b,kind);[~,~,e]=computer;if e=='L',v=swapbytes(v);end;v=v(:);
end
function b=rehash(b)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b(1:1080),'int8'));
b(1081:1112)=typecast(md.digest(),'uint8');
end
function yes=rejects(f)
yes=false;try,f();catch,yes=true;end
end
