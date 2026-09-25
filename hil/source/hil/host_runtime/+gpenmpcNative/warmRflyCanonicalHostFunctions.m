function receipt=warmRflyCanonicalHostFunctions(workRoot,repetitions)
% Warm host functions with disposable retained-data fixtures before connecting.
% The caller selects the repetition count.
arguments
    workRoot (1,1) string
    repetitions (1,1) double
end
assert(isfinite(repetitions)&&repetitions>=1&&repetitions==fix(repetitions), ...
    'gpenmpcNative:WarmupCount','Explicit finite positive replay count required.');
fixturePath=fullfile(gpenmpc_external_path('host_worker_warmup'),'ATTEMPT_RAW.mat');
fixtureSha=gpenmpcNative.fileSha256(fixturePath);
assert(strcmpi(fixtureSha,'0F75AA7825082E05645C444179C3E25D1E0E5DB2699166619F060CF0F5C198E8'), ...
    'gpenmpcNative:WarmupFixtureIdentity','Only the retained original HOST test fixture is used.');
saved=load(fixturePath,'raw','attemptStatus','attemptIo');raw=saved.raw;
event=raw.flight.events{1};observed=raw.last_source_profile;
command=event.result.prepared.full_inner_command;context=event.result.context;binding=event.result.binding;
source=observed.actual_source_sample;outer=raw.startup.events{1}.result.original_source;
echo=saved.attemptStatus.registered_association.echo;
expected=struct('uid',string(echo.uid),'system_id',double(echo.system),'component_id',double(echo.component), ...
    'boot_generation',echo.process_session_generation,'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8, ...
    'configuration_payload_sha256',echo.configuration_payload_sha256,'initial_payload_kg',command.payload_kg, ...
    'initial_leg_index',observed.actual_physical_input.task.leg_index, ...
    'inner_control_contract','CANONICAL_FULL_SE3','full_inner_evidence_scope','BOARD_COMMIT_RFC1');
expected.rfly_board_commit=struct('execution_session_sha256',command.execution_session_sha256, ...
    'identity_semantics',echo.identity_semantics,'link_lifecycle_generation',echo.link_lifecycle_generation, ...
    'board_registration_hrt_us',echo.board_registration_hrt_us, ...
    'reference_max_age_us',uint64(400000),'outer_max_age_us',uint64(400000), ...
    'generated_arm_source_sha256','A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F', ...
    'wrapper_matlab_source_sha256','9117D3CDF8E924A253F4C444CDD467E4850D6F11CBF5744B26375B950E2A95B5');
assets=gpenmpcNative.loadCanonicalAssets();
[expected829,expectedReceipt]=gpenmpcNative.encodeCanonicalFullInnerArguments(command,assets);
% Reconstruct the 10 s fixture with constant initial position plus 0.1 m/s x motion.
initialSource=raw.prepare.events{1}.result.original_source;
[initialState,initialAccepted]=gpenmpcNative.px4EstimateState(initialSource,expected,initialSource.source_host_receive_ns);
assert(initialAccepted,'gpenmpcNative:WarmupInitialSource','The original gen63 fixture state must remain valid as historical input.');
coefficients=zeros(3,8);coefficients(:,1)=initialState(1:3);coefficients(:,2)=[.1;0;0];
trajectory=struct('total_duration_s',10,'coefficients_ascending',coefficients);
response=[];
for k=1:numel(raw.startup_polls)
    candidate=raw.startup_polls{k}.events{end}.result.poll;
    if isfield(candidate,'input_boundary')&&isfield(candidate,'generation')
        assert(isempty(response),'gpenmpcNative:WarmupOuter','The original fixture must have one accepted outer.');response=candidate;
    end
end
assert(~isempty(response),'gpenmpcNative:WarmupOuter','Missing original accepted outer.');
adapter=gpenmpcNative.initializeBoardReferenceAdapter(assets.enmpc,trajectory,string(assets.enmpc.method));
[adapter,~]=gpenmpcNative.commitBoardReferenceDecision(adapter,response.solver_decision, ...
    response.solver_audit,uint64(response.generation),assets.enmpc);
assert(isequal([adapter.target_phase_acceleration_s_inv;adapter.target_outer_correction_f_mps2(:)],context.outer_payload(:)), ...
    'gpenmpcNative:WarmupAppliedOuter','Original decision/audit/continuity must produce the exact retained target.');
health=fullfile(workRoot,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml');
dialect=mavlinkdialect(health,2);
serializer=mavlinkio(dialect,'SystemID',255,'ComponentID',190); % NO connect()
cleanup=onCleanup(@()delete(serializer)); %#ok<NASGU>
clock=@gpenmpcNative.rflyOriginalHostMonotonicNs;
times=zeros(repetitions,9,'uint64');
for iteration=1:repetitions
    % Empty objects each time. Only these discarded fixture instances change.
    runtime=gpenmpcNative.CurrentPhysicalCausalRuntime(workRoot,expected,command.task_identity_sha256,assets);
    ctx=gpenmpcNative.RflyHostContextBinding(expected,8,uint64(400000000),uint64(400000000));
    ctx.recordSnapshot(outer.source_export_bytes,outer.source_host_receive_ns,outer.source_host_receive_ns);
    ctx.submitted(context.outer_generation,outer.source_ticket,outer.source_host_receive_ns);
    ctx.committed(response,context.outer_creation_ns);
    ctx.recordSnapshot(source.source_export_bytes,source.source_host_receive_ns,source.source_host_receive_ns);
    times(iteration,1)=clock();
    [discardedAdapter,baseReference]=gpenmpcNative.stepBoardReferenceAdapter(adapter,trajectory,command.dt_s,assets.enmpc); %#ok<ASGLU>
    times(iteration,2)=clock();
    assert(isequal(baseReference,event.result.prepared.base_reference),'gpenmpcNative:WarmupReferenceBits','Original reference transition changed.');
    prepared=runtime.beginSample(command.source_generation,observed.original_now_input_ns,source, ...
        observed.actual_physical_input,baseReference.reference_up,command.dt_s);
    times(iteration,3)=clock();
    assert(prepared.accepted,'gpenmpcNative:WarmupPrepare','Historical replay rejected.');
    [actual829,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(prepared.full_inner_command,assets);
    assert(isequal(actual829,expected829),'gpenmpcNative:WarmupNumericalBits','Historical full829 changed.');
    [replayed,replayBinding]=ctx.reference(context.reference_generation,source.source_ticket,context.reference_ned,context.reference_creation_ns);
    replayBinding.command_generation=prepared.full_inner_command.generation;
    assert(isequal(replayed,context)&&isequal(replayBinding,binding),'gpenmpcNative:WarmupContext','Original context/ticket changed.');
    times(iteration,4)=clock();
    [contextPackets,~,contextBytes]=gpenmpcNative.RflyContextEncoder(replayed,serializer,dialect);
    times(iteration,5)=clock();
    [commandPackets,~,commandBytes]=gpenmpcNative.RflySlimCommandEncoder(prepared.full_inner_command,assets,replayBinding,serializer,dialect);
    times(iteration,6)=clock();
    assert(isequal(contextBytes,event.result.context_bytes)&&isequal(commandBytes,event.result.command_bytes), ...
        'gpenmpcNative:WarmupWireBits','Original316/381 bodies changed; input framing sequence is serializer-owned.');
    m600check.decodeCanonicalExchangePackets([contextPackets;commandPackets],dialect,saved.attemptIo.canonical_exchange_expected);
    times(iteration,7)=clock();
    historical=struct('context',replayed,'binding',replayBinding,'outer_source_sample',outer);
    runtime.recordBoardCommandSubmission(command.generation,historical,event.send.original_host_submit_ns(1));
    times(iteration,8)=clock();
    state=runtime.status();
    assert(state.control_commit_count==0&&state.board_commit_consumer.pending, ...
        'gpenmpcNative:WarmupNoCommit','Fixture warms preparation and submission.');
    clear runtime ctx
    times(iteration,9)=clock();
end
receipt=struct('schema','RFLY_DISCARDED_HISTORICAL_HOST_FUNCTION_WARMUP_V1', ...
    'completed',true,'repetitions',repetitions,'fixture_path',char(fixturePath),'fixture_sha256',fixtureSha, ...
    'caller_resource_count_not_admission_threshold',true,'original_fixture_times_reused_without_renewal',true, ...
    'runtime_objects_discarded',repetitions,'full829_bit_exact',true,'context316_bit_exact',true,'numeric381_bit_exact',true, ...
    'full829_sha256',expectedReceipt.kernel_argument_sha256,'original_stage_times_ns',times, ...
    'reference_transition_bit_exact',true,'original_solver_decision_audit_replayed_without_solving',true,'actual_service_guards_prewarmed',false, ...
    'stage_columns',{{'adapter_begin','adapter_complete','prepared','context_and_829_check','RCT3','RKS4','decoded7','submitted','discarded'}}, ...
    'thread_observation',struct('maxNumCompThreads',maxNumCompThreads,'OMP_NUM_THREADS',getenv('OMP_NUM_THREADS'), ...
    'MKL_NUM_THREADS',getenv('MKL_NUM_THREADS'),'OPENBLAS_NUM_THREADS',getenv('OPENBLAS_NUM_THREADS')), ...
    'connections_opened',0,'solver_calls',0,'actual_service_objects_received',0,'actual_service_state_changes',0, ...
    'control_sends',0,'control_commits',0,'arm_requests',0,'board_actions',0, ...
    'fixture_is_current_board_observation',false,'session_admission',false,'publication_authority',false, ...
    'wcet_or_100hz_proven',false,'all_closed_loop_GP_branches_prewarmed',false);
end
