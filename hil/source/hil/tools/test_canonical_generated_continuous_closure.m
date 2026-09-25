function report=test_canonical_generated_continuous_closure(outputRoot,mexRoot)
% Replay 60 numerical samples through generated C and compare runtime feedback.
arguments
    outputRoot (1,1) string
    mexRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path;guard=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');addpath(mexRoot,'-begin');
assert(~isfolder(outputRoot),'Preserve earlier evidence.');mkdir(outputRoot);
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
fixture=fullfile(gpenmpc_external_path('canonical_full_inner_runtime'),'RAW.mat');
fixtureSha='3C768445DA08B3E0C124237904D8E6C2A140B44A33D04EE9EADE55238C099803';
assert(strcmpi(gpenmpcNative.fileSha256(fixture),fixtureSha),'Original 60-row archive changed.');
archive=load(fixture,'raw');oldSamples=archive.raw.samples;assert(numel(oldSamples)==60);
a=gpenmpcNative.loadCanonicalAssets();
task=repmat('C',1,64);
e=struct('uid','HOST_ONLY_SYNTHETIC_SAMPLE','system_id',1,'component_id',1, ...
    'boot_generation',7,'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8, ...
    'initial_payload_kg',2.27,'canonical_package_root',gpenmpcNative.canonicalAssetRoot(), ...
    'inner_control_contract','CANONICAL_FULL_SE3', ...
    'full_inner_evidence_scope','HOST_ONLY_KERNEL_EXECUTION');
clock=@gpenmpcNative.rflyOriginalHostMonotonicNs;
sourceNames={'gpenmpcNative.CurrentPhysicalCausalRuntime', ...
    'gpenmpcNative.prepareCanonicalFullInnerCommand','gpenmpcNative.validateCanonicalFullInnerFeedback', ...
    'gpenmpcPredictCurrentGpEvidence','gpenmpcSparseGpPredict', ...
    'gpenmpcAdvanceCausalResidualHistory','gpenmpcUpdateGpAgreementWeight', ...
    'gpenmpcObserveCausalVerticalDisturbance','gpenmpcCommitCausalVerticalDisturbanceObserver', ...
    'gpenmpcUpdateDesiredAttitudeContinuity','gpenmpcNative.encodeCanonicalFullInnerArguments'};
sources=struct('path',{},'sha256',{});
for k=1:numel(sourceNames),sources(end+1)=identity(which(sourceNames{k}));end %#ok<AGROW>
sources(end+1)=identity(which('canonical_generated_closure_mex'));
sources(end+1)=identity([mfilename('fullpath') '.m']);
checks=struct('name',{},'pass',{});raw=struct('samples',{cell(60,1)});report=struct();
timing=zeros(60,5,'uint64');stamp=zeros(60,10,'uint64');
max61=0;exact829=0;exact61=0;exact16=0;observerExact=0;continuityExact=0;
gpAvailable=0;gpHardInvalid=0;gpSoftReduced=0;gpAgreementReduced=0;
r=gpenmpcNative.CurrentPhysicalCausalRuntime(build,e,task,a);
try
    for k=1:60
        original=oldSamples{k};[p,rt,now,reference]=inputs(original);
        stamp(k,1)=clock();b=r.beginSample(uint64(k),now,p,rt,reference,.01);stamp(k,2)=clock();
        check("accepted_begin_"+k,b.accepted);cmd=b.full_inner_command;
        stamp(k,3)=clock();[bytes,abiReceipt]=gpenmpcNative.encodeCanonicalFullInnerArguments(cmd,a);stamp(k,4)=clock();
        stamp(k,5)=clock();[actual61,actual16,valid]=canonical_generated_closure_mex(bytes);stamp(k,6)=clock();
        % Copy only the historical receipt metadata; replace every numerical
        % payload with the actually executed generated-C result.
        stamp(k,7)=clock();feedback=original.feedback;
        feedback.wrench_n_nm=actual61(1:4);feedback.rotor_command_n=actual61(5:10);
        feedback.diagnostic51=actual61(11:61);feedback.valid=logical(valid);
        feedback.generation=cmd.generation;feedback.estimate_sample_timestamp_ns=cmd.estimate_sample_timestamp_ns;
        feedback.configuration_payload_sha256=cmd.configuration_payload_sha256;
        feedback.task_identity_sha256=cmd.task_identity_sha256;feedback.kernel_source_sha256=cmd.kernel_source_sha256;
        stamp(k,8)=clock();
        % Exact original fixture schedule (not current wall time or HRT).
        stamp(k,9)=clock();committed=r.commitControl(uint64(k),uint64(original.feedback.rx_ns)+uint64(1000000),feedback);
        stamp(k,10)=clock();check("accepted_generated_C_commit_"+k,committed.accepted);
        timing(k,:)=stamp(k,2:2:10)-stamp(k,1:2:9);
        old61=[original.feedback.wrench_n_nm;original.feedback.rotor_command_n;original.feedback.diagnostic51];
        old829=gpenmpcNative.encodeCanonicalFullInnerArguments(original.begin.full_inner_command,a);
        error61=max(abs(actual61-old61));max61=max(max61,error61);
        exact829=exact829+isequal(bytes,old829);exact61=exact61+isequal(typecast(actual61,'uint64'),typecast(old61,'uint64'));
        encoded=zeros(16,1,'single');encoded([5;1;4;6;2;3])=single(old61(5:10)/32.145727009134916);
        exact16=exact16+isequal(typecast(actual16,'uint32'),typecast(encoded,'uint32'));
        observerExact=observerExact+isequaln(b.vertical_observer,original.begin.vertical_observer);
        continuityExact=continuityExact+isequaln(cmd.attitude_command,original.begin.full_inner_command.attitude_command);
        closed=b.closed_gp_evidence;
        gpAvailable=gpAvailable+logical(closed.available);
        gpHardInvalid=gpHardInvalid+double(logical(closed.available)&&logical(closed.hard_invalid));
        gpSoftReduced=gpSoftReduced+double(logical(closed.available)&&closed.trust<1);
        gpAgreementReduced=gpAgreementReduced+double(logical(closed.available)&&any(b.gp_agreement_weight_f<1));
        check("generated_C_61_parity_"+k,valid&&error61<1e-10);
        check("same_closure_state_"+k,isequaln(b.closed_gp_evidence,original.begin.closed_gp_evidence) ...
            &&isequaln(b.residual_history_f_mps2,original.begin.residual_history_f_mps2) ...
            &&isequaln(b.physical_output,original.begin.physical_output));
        raw.samples{k}=struct('begin',b,'feedback',feedback,'commit',committed, ...
            'actual61',actual61,'actual16',actual16,'arguments829',bytes,'abi',abiReceipt, ...
            'outer_runtime',r.outerRuntimeState(),'status',r.status(),'original_fixture_begin_ns',now, ...
            'has_actual_board_HRT',false,'host_wall_stage_timestamps_ns',stamp(k,:));
    end
    final=r.status();
    check('60_original_829_ABI_bit_exact',exact829==60);
    check('60_official_16_outputs_bit_exact',exact16==60);
    check('60_original_observers_and_continuity_exact',observerExact==60&&continuityExact==60);
    check('generated_C_reaches_original_GP_active_closure',final.control_commit_count==60 ...
        &&final.closed_evidence_count==59&&final.available_evidence_count==58 ...
        &&final.physical_gp_active_count==26&&~final.failed);
    check('GP_prediction_pending_closes_on_next_original_source', ...
        ~raw.samples{1}.commit.pending_gp_available&&raw.samples{2}.commit.pending_gp_available ...
        &&~raw.samples{2}.begin.closed_gp_evidence.available&&raw.samples{3}.begin.closed_gp_evidence.available);
    % Exercise negative cases through the runtime.
    [p1,rt1,t1,ref1]=inputs(oldSamples{1});[p2,rt2,t2,ref2]=inputs(oldSamples{2});
    [p3,rt3,t3,ref3]=inputs(oldSamples{3});
    missing=gpenmpcNative.CurrentPhysicalCausalRuntime(build,e,task,a);
    missing.beginSample(uint64(1),t1,p1,rt1,ref1,.01);
    bad=missing.beginSample(uint64(2),t2,p2,rt2,ref2,.01);
    check('missing_commit_latches_without_advancing',~bad.accepted ...
        &&bad.reason=="PREVIOUS_SAMPLE_CONTROL_NOT_COMMITTED"&&missing.status().control_commit_count==0);
    bad=missing.beginSample(uint64(3),t3,p3,rt3,ref3,.01);
    check('missing_commit_fault_is_not_washed',~bad.accepted&&missing.status().failed);
    duplicate=firstCommitted();bad=duplicate.commitControl(uint64(1),uint64(oldSamples{1}.feedback.rx_ns)+uint64(1000000),raw.samples{1}.feedback);
    check('duplicate_commit_rejected_keeps_actual_count',~bad.accepted&&duplicate.status().control_commit_count==1&&duplicate.status().failed);
    gap=firstCommitted();bad=gap.beginSample(uint64(3),t3,p3,rt3,ref3,.01);
    check('sample_generation_gap_latches',~bad.accepted&&bad.reason=="GENERATION_TIMESTAMP_OR_DT_REJECTED");
    sourceGap=firstCommitted();p2.sample_timestamp_ns=p3.sample_timestamp_ns;
    bad=sourceGap.beginSample(uint64(2),t2,p2,rt2,ref2,.01);
    check('source_time_gap_not_hidden_by_contiguous_generation',~bad.accepted&&bad.reason=="CAUSAL_INTERVAL_REJECTED");
    bytes=raw.samples{1}.arguments829;
    check('mex_rejects_short_ABI',mexRejects(bytes(1:end-1),'gpenmpc:GeneratedClosureShape'));
    badBytes=bytes;badBytes(1)=uint8('X');check('mex_rejects_wrong_magic',mexRejects(badBytes,'gpenmpc:GeneratedClosureAbi'));
    badBytes=bytes;badBytes(829)=bitxor(badBytes(829),uint8(1));
    check('mex_rejects_generation_disagreement',mexRejects(badBytes,'gpenmpc:GeneratedClosureGeneration'));
    badBytes=bytes;badBytes(5:12)=uint8([127 240 0 0 0 0 0 0]);
    check('mex_rejects_nonfinite_binary64',mexRejects(badBytes,'gpenmpc:GeneratedClosureFinite'));
    % Check private-source mapping separately by replaying retained bytes.
    sourceFile=fullfile(gpenmpc_external_path('host_worker_warmup'),'SOURCE_1.bin');
    sourceBytes=readBytes(sourceFile);sourceRx=uint64(9000000000);
    decoded=gpenmpcNative.RflySnapshotDecoder(sourceBytes,sourceRx);
    pe=e;pe.uid=string(decoded.observed_uid);pe.boot_generation=decoded.observed_boot_generation;
    pe.configuration_payload_sha256=a.binding.effective_configuration_payload_sha256;
    ps=gpenmpcNative.RflySnapshotSample(sourceBytes,sourceRx,pe);
    [privateState,ok]=gpenmpcNative.px4EstimateState(ps,pe,sourceRx+uint64(1000));
    q=double(ps.raw13_float32(7:10));q=q/norm(q);C=diag([1,1,-1]);
    mapped=[C*(double(ps.raw13_float32(1:3))-ps.task_origin_ned_m); ...
        C*double(ps.raw13_float32(4:6));q.*[1;-1;-1;1];double(ps.raw13_float32(11:13)).*[-1;-1;1]];
    check('private_original_float32_mapping_bit_exact',ok&&isequal(typecast(privateState,'uint64'),typecast(mapped,'uint64')));
    tampered=ps;tampered.position_ned_m(1)=tampered.position_ned_m(1)+1;
    [~,ok,reason]=gpenmpcNative.px4EstimateState(tampered,pe,sourceRx+uint64(1000));
    check('private_original_bytes_cannot_be_relabelled',~ok&&strcmp(reason,'PRIVATE_SNAPSHOT_RECONSTRUCTION_MISMATCH'));
    raw.private_mapping=struct('source_file',identity(sourceFile),'source',ps, ...
        'state13',privateState,'direct_same_arithmetic',mapped,'live_board_observation',false);
    for k=1:numel(sources),check("stable_source_"+k,isequal(sources(k),identity(sources(k).path)));end
    costs=double(timing)*1e-6;summary=struct();names={'begin','encode829','actual_generated_C_mex','feedback_struct','commit_with_source_oracle_and_GP'};
    for k=1:5
        summary.(names{k})=struct('first_ms',costs(1,k),'median_2_to_60_ms',median(costs(2:end,k)), ...
            'max_2_to_60_ms',max(costs(2:end,k)),'p95_2_to_60_ms',prctile(costs(2:end,k),95));
    end
    total=sum(costs,2);summary.total_measured_stages=struct('first_ms',total(1), ...
        'median_2_to_60_ms',median(total(2:end)),'max_2_to_60_ms',max(total(2:end)), ...
        'over_10ms_count',sum(total>.01*1000));
    report=struct('status','PASS_ACTUAL_GENERATED_C_CONTINUOUS_HOST_NUMERIC_CLOSURE', ...
        'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
        'fixture',identity(fixture),'source_identities',sources, ...
        'generated_C_build_receipt',identity(fullfile(mexRoot,'BUILD_RESULT.json')),'actual_generated_C_steps',60, ...
        'bit_exact_original829',exact829,'bit_exact_original61',exact61,'bit_exact_official16',exact16, ...
        'maximum_actual61_vs_original',max61,'exact_observer_rows',observerExact, ...
        'exact_continuity_rows',continuityExact,'runtime',final, ...
        'GP_predictor_calls_with_available_result',sum(cellfun(@(s)s.commit.pending_gp_available,raw.samples)), ...
        'closed_GP_available',gpAvailable,'closed_GP_hard_invalid',gpHardInvalid, ...
        'closed_GP_soft_trust_below_one',gpSoftReduced,'closed_GP_axis_agreement_below_one',gpAgreementReduced, ...
        'timing_ms',summary,'actual_maxNumCompThreads',maxNumCompThreads, ...
        'OMP_NUM_THREADS',getenv('OMP_NUM_THREADS'),'MKL_NUM_THREADS',getenv('MKL_NUM_THREADS'), ...
        'OPENBLAS_NUM_THREADS',getenv('OPENBLAS_NUM_THREADS'), ...
        'numeric_clock','UNCHANGED_ORIGINAL_OFFLINE_FIXTURE_SCHEDULE_NOT_HOST_OR_BOARD_CLOCK', ...
        'timing_clock','ORIGINAL_UINT64_HOST_MONOTONIC_NS_NO_SLEEP_OR_TARGET_RATE', ...
        'state_update_control_source','UNCHANGED_HOST_ONLY_VALIDATOR_SOURCE_ORACLE_AFTER_ACTUAL_C_PARITY', ...
        'fixed_outer_reference_is_new_ENMPC_evidence',false,'model_steps',0,'solver_calls',0, ...
        'COM_UDP_actions',0,'board_actions',0,'board_publication_receipt_created',false, ...
        'production_5ms_guard_executed',false,'hundred_Hz_deadline_proven',false,'ARM_WCET_proven',false);
catch failure
    report=struct('status','FAIL_PRESERVED_HOST_NUMERIC_CLOSURE','error_identifier',failure.identifier, ...
        'error',getReport(failure,'extended','hyperlinks','off'),'checks',checks, ...
        'test_count',numel(checks),'pass_count',sum([checks.pass]),'board_actions',0);
    save(fullfile(outputRoot,'RAW.mat'),'raw','report','timing','stamp','-v7.3');
    write(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));rethrow(failure);
end
save(fullfile(outputRoot,'RAW.mat'),'raw','report','timing','stamp','-v7.3');
writetable(array2table([(1:60)' double(timing)*1e-6],VariableNames= ...
    {'sample','begin_ms','encode829_ms','generated_C_ms','feedback_struct_ms','commit_ms'}),fullfile(outputRoot,'HOST_TIMING.csv'));
write(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));disp(jsonencode(report));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpc:GeneratedContinuousClosure','%s',name);
    end
    function rr=firstCommitted()
        rr=gpenmpcNative.CurrentPhysicalCausalRuntime(build,e,task,a);
        bb=rr.beginSample(uint64(1),t1,p1,rt1,ref1,.01);assert(bb.accepted);
        cc=rr.commitControl(uint64(1),uint64(oldSamples{1}.feedback.rx_ns)+uint64(1000000),raw.samples{1}.feedback);assert(cc.accepted);
    end
    function [p,rt,now,reference]=inputs(row)
        command=row.begin.full_inner_command;x=command.state_up;g=command.generation;
        now=uint64(row.begin.host_receive_timestamp_ns);reference=command.reference_up;
        % The only physical value not retained in command is the original
        % test's same-sample lag state: six exactly-20 N synthetic inputs.
        C=diag([1,1,-1]);p=struct('source','PX4_EKF2_MAVLINK_ODOMETRY_331','atomic_estimate',true, ...
            'plant_truth_used',false,'uid',e.uid,'system_id',1,'component_id',1,'boot_generation',7, ...
            'sample_timestamp_ns',command.estimate_sample_timestamp_ns,'odometry_reset_counter',2, ...
            'odometry_frame_id',1,'odometry_child_frame_id',1,'odometry_estimator_type',8, ...
            'position_ned_m',C*x(1:3),'velocity_ned_mps',C*x(4:6), ...
            'quaternion_wxyz_body_to_ned',x(7:10).*[1;-1;-1;1],'omega_frd_rad_s',x(11:13).*[-1;-1;1], ...
            'position_rx_ns',double(now)-1e5,'attitude_rx_ns',double(now)-1e5,'rates_rx_ns',double(now)-1e5, ...
            'position_generation',double(g),'attitude_generation',double(g),'rates_generation',double(g), ...
            'position_valid',true,'attitude_valid',true,'rates_valid',true);
        v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE','valid',true,'generation',double(g), ...
            'rx_ns',double(now)-1e5,'ordering','SOFTWARE_M600_ORDER','rotor_thrust_state_n',ones(6,1)*20);
        w=struct('source','FROZEN_TASK_WIND_ESTIMATOR','valid',true,'generation',double(g), ...
            'rx_ns',double(now)-1e5,'estimate_xy_mps',command.wind_estimate_xy_mps);
        rt=struct('schema','GPENMPC_PHYSICAL_CAUSAL_RUNTIME_INPUT_V1','task_identity_sha256',task, ...
            'plant_truth_used',false,'virtual_actuator',v,'wind',w, ...
            'task',struct('source','FROZEN_GPENMPC_TASK_STATE','payload_kg',command.payload_kg,'leg_index',1));
    end
end
function ok=mexRejects(bytes,id)
ok=false;try,[a,b,c]=canonical_generated_closure_mex(bytes);catch ex,ok=strcmp(ex.identifier,id);end %#ok<ASGLU>
end
function id=identity(file)
id=struct('path',char(file),'sha256',gpenmpcNative.fileSha256(file));
end
function bytes=readBytes(file)
f=fopen(file,'rb');assert(f>=0);g=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8'); %#ok<NASGU>
end
function write(file,value)
f=fopen(file,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));fprintf(f,'%s\n',value); %#ok<NASGU>
end
