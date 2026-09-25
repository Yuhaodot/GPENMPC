function report=test_rfly_discarded_warmup(outputRoot)
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
source=which('gpenmpcNative.warmRflyCanonicalHostFunctions');sha=gpenmpcNative.fileSha256(source);
% Count20 was chosen before this confirmation, from the separated PROFILE003
% 20-replay diagnostic. No duration is relaxed and no warm result admits IO.
receipt=gpenmpcNative.warmRflyCanonicalHostFunctions(build,20);
% Verify warmed functions with new disposable objects in a separate invocation.
confirmation=gpenmpcNative.warmRflyCanonicalHostFunctions(build,2);
checks=struct('name',{},'pass',{});
check('all_original_numerical_and_context_bytes_exact',receipt.reference_transition_bit_exact&&receipt.full829_bit_exact&&receipt.context316_bit_exact&&receipt.numeric381_bit_exact ...
    &&confirmation.full829_bit_exact&&confirmation.context316_bit_exact&&confirmation.numeric381_bit_exact);
check('separate_objects_never_actual_service_state',receipt.runtime_objects_discarded==20&&confirmation.runtime_objects_discarded==2 ...
    &&receipt.actual_service_objects_received==0&&receipt.actual_service_state_changes==0);
check('no_connection_solver_control_commit_or_authority',receipt.connections_opened==0&&receipt.solver_calls==0 ...
    &&receipt.control_sends==0&&receipt.control_commits==0&&~receipt.session_admission&&~receipt.publication_authority ...
    &&~receipt.fixture_is_current_board_observation&&~receipt.wcet_or_100hz_proven);
check('original_uint64_stage_observations',isa(receipt.original_stage_times_ns,'uint64') ...
    &&all(diff(receipt.original_stage_times_ns,1,2)>=0,'all'));
for v={0,-1,NaN,1.5}
    caught='';try,gpenmpcNative.warmRflyCanonicalHostFunctions(build,v{1});catch ex,caught=ex.identifier;end
    check(sprintf('invalid_explicit_replay_count_%g',v{1}),strcmp(caught,'gpenmpcNative:WarmupCount'));
end
check('source_stable',strcmp(sha,gpenmpcNative.fileSha256(source)));
report=struct('passed',all([checks.pass]),'checks',checks,'warmup',receipt,'confirmation',confirmation, ...
    'source_path',source,'source_sha256',sha,'connected_to_board',false,'com_opens',0);
save(fullfile(outputRoot,'RAW.mat'),'report');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',numel(checks),'control_sends',0,'control_commits',0)));
submittedColumn=find(strcmp(confirmation.stage_columns,'submitted'));
disp(double(confirmation.original_stage_times_ns(:,submittedColumn)-confirmation.original_stage_times_ns(:,1))/1e6);
assert(report.passed,'gpenmpcNative:WarmupTest','Discarded warmup test failed.');
    function check(name,value),checks(end+1)=struct('name',name,'pass',logical(value));end
end
