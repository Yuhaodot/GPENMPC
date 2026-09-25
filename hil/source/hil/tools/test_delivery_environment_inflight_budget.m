function result=test_delivery_environment_inflight_budget(outputDir)
%TEST_DELIVERY_ENVIRONMENT_INFLIGHT_BUDGET Pure HOST cross-process test.
% A board observation accepted at the sender-side 3.0 s HEARTBEAT bound may
% spend up to the independently fixed 0.25 s environment budget in flight.
% The generated plant therefore uses 3.25 s only for the transported receive
% lower bound; sender-side HEARTBEAT/EXTENDED_SYS_STATE gates remain 3.0/2.0.
arguments
    outputDir (1,1) string
end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpcTaskIo:BudgetOutputExists');
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'));
loaded=load(fullfile(gpenmpc_external_path('environment_delivery_model'), ...
    'M600_CORE_PARAMETERS.mat'),'deliveryPolicy');
p=loaded.deliveryPolicy;
checks=struct;
checks.exact_derived_contract=p.max_env_age_s==0.25&&p.max_board_age_s==3.25;

% Accept a frame with 2.99 s heartbeat age and 0.24 s transport time.
f=frame(1,100.0,97.01,15);
s=gpenmpcTaskIo.initPlantEnvironmentV2(p,100.0);
[s,r]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s,f,true,100.24,false,p);
checks.sender_valid_plus_inflight_accepted=~r.task_env_failed&&r.pending;

% A frame beyond the derived 3.25 s board-evidence age remains fail-closed.
s2=gpenmpcTaskIo.initPlantEnvironmentV2(p,100.0);
[~,r2]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s2,frame(1,100.0,96.98,15), ...
    true,100.24,false,p);
checks.beyond_derived_board_bound_rejected=r2.task_env_failed&&r2.failure_code==6;

% Enforce the 0.25 s transport bound.
s3=gpenmpcTaskIo.initPlantEnvironmentV2(p,100.0);
[~,r3]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s3,f,true,100.251,false,p);
checks.environment_inflight_overrun_rejected=r3.task_env_failed&&r3.failure_code==3;

% Host freshness bits are independent and invalid evidence is rejected.
s4=gpenmpcTaskIo.initPlantEnvironmentV2(p,100.0);
[~,r4]=gpenmpcTaskIo.acceptPlantEnvironmentV2(s4,frame(1,100.0,99.9,14), ...
    true,100.01,false,p);
checks.invalid_sender_freshness_bit_rejected=r4.task_env_failed&&r4.failure_code==13;

result=struct('schema','HOST_DELIVERY_ENVIRONMENT_INFLIGHT_BUDGET_V1', ...
    'status','PASS_HOST_ONLY_DERIVED_CROSS_PROCESS_BUDGET', ...
    'passed',all(structfun(@logical,checks)),'checks',checks, ...
    'checks_passed',nnz(structfun(@logical,checks)), ...
    'checks_total',numel(fieldnames(checks)), ...
    'host_heartbeat_freshness_s',3.0,'host_extended_state_freshness_s',2.0, ...
    'environment_inflight_budget_s',0.25,'derived_board_evidence_bound_s',3.25, ...
    'COM_open',0,'board_actions',0,'physical_output_actions',0);
mkdir(outputDir);
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');
assert(fid>=0,'gpenmpcTaskIo:BudgetResultOpen');guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));
assert(result.passed,'gpenmpcTaskIo:BudgetTestFailed');
fprintf('DELIVERY_ENVIRONMENT_INFLIGHT_BUDGET %d/%d\n', ...
    result.checks_passed,result.checks_total);
end

function f=frame(generation,sourceTime,boardTime,flags)
f=zeros(28,1);f(1)=2;f(2)=generation;f(3)=sourceTime;f(4)=0;
f(5)=2.21;f(8)=1;f(23)=26090501;f(24)=boardTime;f(25)=flags;
f(26)=2;f(27)=1;f(28)=0;
end
