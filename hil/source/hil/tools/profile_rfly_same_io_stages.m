function report=profile_rfly_same_io_stages(outputRoot,archiveLabel)
% Profile retained input processing offline.
arguments
    outputRoot (1,1) string
    archiveLabel (1,1) string = ""
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
if strlength(archiveLabel)==0
    archive=gpenmpc_external_path('same_io_processing_profile_fixture');
else
    archive=fullfile(gpenmpc_external_path('rfly_board_commit_consumer'),archiveLabel);
end
saved=load(fullfile(archive,'ATTEMPT_RAW.mat'));raw=saved.raw;flight=raw.flight.events{1};
command=flight.result.prepared.full_inner_command;context=flight.result.context;binding=flight.result.binding;
source=flight.result.original_source;outer=raw.startup.events{1}.result.original_source;
association=saved.attemptStatus.registered_association;echo=association.echo;
expected=struct('uid',string(echo.uid),'system_id',double(echo.system),'component_id',double(echo.component), ...
    'boot_generation',echo.process_session_generation,'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8, ...
    'configuration_payload_sha256',echo.configuration_payload_sha256,'initial_payload_kg',2.27);
expected.inner_control_contract='CANONICAL_FULL_SE3';expected.full_inner_evidence_scope='BOARD_COMMIT_RFC1';
expected.initial_leg_index=1;
expected.rfly_board_commit=struct('execution_session_sha256',association.execution_session_sha256, ...
    'identity_semantics',echo.identity_semantics,'link_lifecycle_generation',echo.link_lifecycle_generation, ...
    'board_registration_hrt_us',echo.board_registration_hrt_us,'reference_max_age_us',uint64(400000), ...
    'outer_max_age_us',uint64(400000),'generated_arm_source_sha256', ...
    'A47F1C255BDAC1DAE712494BE8D9D66FC4F83138DBA9B0C2E3A31E114D4ABD0F', ...
    'wrapper_matlab_source_sha256','9117D3CDF8E924A253F4C444CDD467E4850D6F11CBF5744B26375B950E2A95B5');
names={'gpenmpcNative.RflySnapshotDecoder','gpenmpcNative.RflySnapshotSample','gpenmpcNative.prepareCanonicalFullInnerCommand', ...
    'gpenmpcNative.encodeCanonicalFullInnerArguments','gpenmpcNative.RflyContextEncoder', ...
    'gpenmpcNative.RflySlimCommandEncoder','m600check.decodeCanonicalExchangePackets', ...
    'gpenmpcNative.RflyBoardCommitConsumer','gpenmpcNative.RflyHostContextBinding','gpenmpcNative.loadCanonicalAssets', ...
    'gpenmpcNative.RflyHostExchangeService','gpenmpcNative.advanceRflyCanonicalIoExchange'};
paths=cellfun(@which,names,'UniformOutput',false);hashes=cellfun(@fileSha,paths,'UniformOutput',false);
t0=clockNs();assets=gpenmpcNative.loadCanonicalAssets();assetLoadNs=clockNs()-t0;
d=mavlinkdialect(fullfile(build,'m600_coptersim','matlab_validation','+m600check','px4_health_events.xml'),2);
serializer=mavlinkio(d,'SystemID',255,'ComponentID',190);cleanup=onCleanup(@()delete(serializer)); %#ok<NASGU>
[full,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(command,assets);
previous=gpenmpcInitializeDesiredAttitudeContinuityState();
bc=struct('source_generation',command.source_generation,'source_ticket',command.source_ticket, ...
    'execution_session_sha256',command.execution_session_sha256);
[prepared,~]=prepare();[preparedBytes,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(prepared,assets);
assert(isequal(preparedBytes,full),'gpenmpcNative:ProfileExactPrepare','Fixed original first-leg preparation must be exact.');
feedbackBytes=readbin(fullfile(archive,'FEEDBACK1112.bin'));
packets=flight.result.packets;ioExpected=saved.attemptIo.canonical_exchange_expected;
N=20;bench=struct('name',{},'samples_ns',{},'start_ns',{},'end_ns',{},'median_ms',{},'minimum_ms',{},'maximum_ms',{});
profile clear;
measure('RSP1_decode_SHA246',@()gpenmpcNative.RflySnapshotDecoder(source.source_export_bytes,source.source_host_receive_ns));
measure('RSP1_sample_decode_identity_field_mapping',@()gpenmpcNative.RflySnapshotSample(source.source_export_bytes,source.source_host_receive_ns,expected));
measure('prepareCanonicalFullInnerCommand_exact_initial_continuity',@prepare);
measure('RAK1_full829_encode_and_SHA',@()gpenmpcNative.encodeCanonicalFullInnerArguments(command,assets));
measure('RCT1_context3_validate_SHA_and_official_serialize',@contextEncode);
measure('RKS1_numeric4_including_full829_validate_SHA_and_serialize',@slimEncode);
measure('official_IO_decode_validate7',@()m600check.decodeCanonicalExchangePackets(packets,d,ioExpected));
measure('private19_reconstruction_from_original_float32',@()gpenmpcNative.rflyBoardReconstructedArguments(full,command,source));
measure('RFC1_full1112_decode_SHA_fields',@()gpenmpcNative.RflyCommittedFeedbackDecoder(feedbackBytes,raw.feedback_complete_ns));
profile off;pureProfile=profile('info');
% Use disposable replay objects and measure execution time separately from recorded time.
consumers=cell(N+2,1);contexts=cell(N+2,1);
responses={};for k=1:numel(raw.startup_polls)
    p=raw.startup_polls{k}.events{end}.result.poll;
    if isfield(p,'input_boundary')&&isfield(p,'generation'),responses{end+1}=p;end %#ok<AGROW>
end
assert(numel(responses)==1);response=responses{1};
for k=1:N+2
    consumers{k}=gpenmpcNative.RflyBoardCommitConsumer(expected,command.task_identity_sha256,assets);
    contexts{k}=gpenmpcNative.RflyHostContextBinding(expected,8,uint64(400000000),uint64(400000000));
    contexts{k}.recordSnapshot(outer.source_export_bytes,outer.source_host_receive_ns,outer.source_host_receive_ns);
    contexts{k}.submitted(context.outer_generation,outer.source_ticket,outer.source_host_receive_ns);
    contexts{k}.committed(response,context.outer_creation_ns);
    contexts{k}.recordSnapshot(source.source_export_bytes,source.source_host_receive_ns,source.source_host_receive_ns);
end
historicalAssociation=struct('context',context,'binding',binding,'outer_source_sample',outer);
consumers{1}.submitted(command,source,historicalAssociation,flight.send.original_host_submit_ns(1));
contexts{1}.reference(context.reference_generation,source.source_ticket,context.reference_ned,context.reference_creation_ns);
durations=zeros(N,1,'uint64');referenceNs=durations;starts=durations;finishes=durations;refStarts=durations;refEnds=durations;
profile clear;
for k=1:N
    starts(k)=clockNs();consumers{k+1}.submitted(command,source,historicalAssociation,flight.send.original_host_submit_ns(1));finishes(k)=clockNs();durations(k)=finishes(k)-starts(k);
    refStarts(k)=clockNs();[replayed,replayBinding]=contexts{k+1}.reference(context.reference_generation,source.source_ticket,context.reference_ned,context.reference_creation_ns);refEnds(k)=clockNs();referenceNs(k)=refEnds(k)-refStarts(k);
    assert(isequal(replayed,context)&&isequal(replayBinding.snapshot_ticket,binding.snapshot_ticket));
end
profile on;
consumers{N+2}.submitted(command,source,historicalAssociation,flight.send.original_host_submit_ns(1));
contexts{N+2}.reference(context.reference_generation,source.source_ticket,context.reference_ned,context.reference_creation_ns);
profile off;bindingProfile=profile('info');
append('submitted_original_association_redecode_two_RSP_full829_hashes',durations,starts,finishes);
append('Context_reference_original_events_validation_and_structs',referenceNs,refStarts,refEnds);
fullCovered=isfield(raw,'last_source_profile')&&isfield(raw.last_source_profile,'actual_physical_input');
physicalProfile=struct('FunctionTable',[]);physicalStamps=[];
if fullCovered
    observed=raw.last_source_profile;physicalStamps=observed;
    instances=cell(N+2,1);
    for k=1:N+2,instances{k}=gpenmpcNative.CurrentPhysicalCausalRuntime(build,expected,command.task_identity_sha256,assets);end
    result=physical(instances{1});[checkBytes,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(result.full_inner_command,assets);
    assert(result.accepted&&isequal(checkBytes,full),'gpenmpcNative:ProfileFullPhysicalInput','Actual original initial-leg physical input must reproduce full829 exactly.');
    ns=zeros(N,1,'uint64');starts=ns;ends=ns;
    for k=1:N
        starts(k)=clockNs();result=physical(instances{k+1});ends(k)=clockNs();ns(k)=ends(k)-starts(k);
        [checkBytes,~]=gpenmpcNative.encodeCanonicalFullInnerArguments(result.full_inner_command,assets);assert(result.accepted&&isequal(checkBytes,full));
    end
    append('full_CurrentPhysical_beginSample_original_initial_leg',ns,starts,ends);
    profile clear;profile on;physical(instances{N+2});profile off;physicalProfile=profile('info');
end
% Use retained timing records from the same IO calls.
rx=flight.ingress.original_host_receive_ns;sent=flight.send;
stages=struct('source_callback_to_first_submit_ms',double(sent.original_host_submit_ns(1)-rx)/1e6, ...
    'seven_send_calls_elapsed_ms',double(sent.original_host_send_return_ns(end)-sent.original_host_submit_ns(1))/1e6, ...
    'last_send_return_to_step_return_ms',double(raw.flight_return_ns-sent.original_host_send_return_ns(end))/1e6, ...
    'step_return_to_peer_read_complete_ms',double(raw.peer_read_complete_ns-raw.flight_return_ns)/1e6, ...
    'peer_read_to_actual_C_response_ms',double(raw.cpp_complete_ns-raw.peer_read_complete_ns)/1e6, ...
    'actual_C_to_feedback_emit_ms',double(raw.feedback_emit_ns-raw.cpp_complete_ns)/1e6, ...
    'feedback_emit_to_callback_complete_ms',double(raw.feedback_complete_ns-raw.feedback_emit_ns)/1e6, ...
    'callback_complete_to_step_return_ms',double(raw.commit_return_ns-raw.feedback_complete_ns)/1e6);
% Split the retained IO ledger offline.
stages.original_uint64_host_submit_ns=sent.original_host_submit_ns;
stages.original_uint64_host_send_return_ns=sent.original_host_send_return_ns;
stages.actual_send_call_ms=double(sent.original_host_send_return_ns-sent.original_host_submit_ns)/1e6;
stages.between_send_return_and_next_submit_ms=double(sent.original_host_submit_ns(2:end)-sent.original_host_send_return_ns(1:end-1))/1e6;
if isfield(flight,'send_admission')&&isfield(raw,'last_source_profile')
    stages.exchange_return_to_pre_send_check_ms=double(flight.send_admission.original_host_check_ns-raw.last_source_profile.return_ns)/1e6;
    stages.pre_send_check_to_first_send_ms=double(sent.original_host_submit_ns(1)-flight.send_admission.original_host_check_ns)/1e6;
end
stages.unresolved_pre_send_scope='Evidence acquisition, official decode/validation and preparation; no finer retained stamps';
stages.unresolved_post_send_scope='submitted, post-poll and status; no finer retained stamps';
report=struct('scope','FIXED_ORIGINAL_HISTORICAL_INPUT_MICROBENCH_NO_FRESH_ADMISSION', ...
    'input_archive',char(archive),'input_sha256',fileSha(fullfile(archive,'ATTEMPT_RAW.mat')), ...
    'warm_samples_each',N,'profiler_enabled_during_timed_samples',false, ...
    'asset_load_once_ms',double(assetLoadNs)/1e6,'benchmarks',bench, ...
    'actual_archived_stages',stages,'pure_profile_top',top(pureProfile),'binding_profile_top',top(bindingProfile), ...
    'physical_profile_top',top(physicalProfile),'actual_physical_stage_stamps',physicalStamps, ...
    'fixed_prepare_full829_bit_exact',true,'context_original_bytes_fields_exact',true, ...
    'full_Physical_beginSample_covered',fullCovered, ...
    'repeated_samples_are_independent_initial_leg_replays_not_continuous_flight',true, ...
    'thread_observation',struct('maxNumCompThreads',maxNumCompThreads,'OMP_NUM_THREADS',getenv('OMP_NUM_THREADS'), ...
    'MKL_NUM_THREADS',getenv('MKL_NUM_THREADS'),'OPENBLAS_NUM_THREADS',getenv('OPENBLAS_NUM_THREADS')), ...
    'expired_host_commit_rejected',true,'new_endpoints',0,'solver_calls',0,'sends',0,'board_actions',0, ...
    'sources',{paths},'source_sha256',{hashes},'sources_unchanged',isequal(hashes,cellfun(@fileSha,paths,'UniformOutput',false)));
save(fullfile(outputRoot,'RAW_PROFILE.mat'),'report','pureProfile','bindingProfile','physicalProfile','bench','stages');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>0);fc=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear fc
for k=1:numel(bench),fprintf('%s median=%.3fms range=%.3f..%.3fms\n',bench(k).name,bench(k).median_ms,bench(k).minimum_ms,bench(k).maximum_ms);end
    function [c,n]=prepare()
        [c,n]=gpenmpcNative.prepareCanonicalFullInnerCommand(command.state_up(1:13),command.reference_up, ...
            command.payload_kg,command.wind_estimate_xy_mps,command.augmentation_up_mps2,previous,command.dt_s,true, ...
            assets,command.generation,command.estimate_sample_timestamp_ns,command.task_identity_sha256,bc);
    end
    function value=physical(instance)
        value=instance.beginSample(command.source_generation,observed.original_now_input_ns, ...
            observed.actual_source_sample,observed.actual_physical_input,command.reference_up,command.dt_s);
    end
    function p=contextEncode(),[p,~,body]=gpenmpcNative.RflyContextEncoder(context,serializer,d);assert(isequal(body,flight.result.context_bytes));end
    function p=slimEncode(),[p,~,body]=gpenmpcNative.RflySlimCommandEncoder(command,assets,binding,serializer,d);assert(isequal(body,flight.result.command_bytes));end
    function measure(name,fn)
        fn();ns=zeros(N,1,'uint64');beginNs=ns;endNs=ns;
        for iteration=1:N,beginNs(iteration)=clockNs();fn();endNs(iteration)=clockNs();ns(iteration)=endNs(iteration)-beginNs(iteration);end
        append(name,ns,beginNs,endNs);
        profile on;fn();profile off;
    end
    function append(name,ns,beginNs,endNs)
        ms=double(ns)/1e6;bench(end+1)=struct('name',name,'samples_ns',ns,'start_ns',beginNs,'end_ns',endNs, ...
            'median_ms',median(ms),'minimum_ms',min(ms),'maximum_ms',max(ms));
    end
end
function ns=clockNs(),ns=gpenmpcNative.rflyOriginalHostMonotonicNs();end
function b=readbin(p),f=fopen(p,'rb');assert(f>0);c=onCleanup(@()fclose(f));b=fread(f,inf,'*uint8');end
function value=fileSha(p),md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(readbin(p),'int8'));value=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));end
function rows=top(info)
rows=struct('name',{},'calls',{},'total_s',{},'file',{});
if isempty(info.FunctionTable),return;end
t=info.FunctionTable;[~,order]=sort([t.TotalTime],'descend');order=order(1:min(24,numel(order)));
for k=order,rows(end+1)=struct('name',t(k).FunctionName,'calls',t(k).NumCalls,'total_s',t(k).TotalTime,'file',t(k).FileName);end
end
