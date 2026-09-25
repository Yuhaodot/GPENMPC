function result=run_canonical_cambridge_binding_tests(packageRoot)
% Test the built package offline and return results in memory.
arguments
    packageRoot (1,1) string
end
root=string(gpenmpc_external_path('delivery_method_project'));
cfg=jsondecode(fileread(fullfile(root,'config','CAMBRIDGE_REPRESENTATIVE_PHYSICAL_MISSION_V1.json')));
ledger=jsondecode(fileread(cfg.source_ledger.path));
mission=jsondecode(fileread(cfg.mission_source.path));
method=jsondecode(fileread(string(gpenmpc_external_path('hil_method_contract'))));
data=load(fullfile(packageRoot,'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'),'physicalTask','timeline');
task=data.physicalTask; timeline=data.timeline;
positive=validate_canonical_cambridge_binding(task,cfg,ledger,mission,method,timeline);
tests=struct('name',"exact_canonical_positive",'pass',true);
t=task; t.mission_id="ENMPC_DV_008"; reject('wrong_mission_id',t,method);
t=task; t.planner_id="P_FIXED_SPEED"; reject('wrong_planner',t,method);
t=task; t.visit_order=flipud(t.visit_order); reject('wrong_visit_order',t,method);
t=task; t.route_candidate_ids(1)="WRONG_ROUTE"; reject('wrong_route',t,method);
t=task; t.plan_payload_sha256=repmat('0',1,64); reject('wrong_plan_digest',t,method);
t=task; t.physical_service.ground_dwell_s=10; reject('invalid_ten_second_dwell',t,method);
t=task; t.reference.payload_kg(1)=2.27; reject('wrong_initial_payload',t,method);
t=task; t.mission_config.plant_mismatch.mass_bias_kg=0; reject('environment_mismatch',t,method);
t=task; t.reference.position_m(100,1)=t.reference.position_m(100,1)+0.01; reject('transit_not_exact',t,method);
t=task; t.reference.global_time_s(100)=t.reference.global_time_s(99); reject('nonmonotone_reference',t,method);
t=task; t.parent_control_injection_prohibited=false; reject('parent_control_reuse',t,method);
m=method; m.internal_identity.architecture="B2_030_ORDINARY"; reject('wrong_execution_method',task,m);
result=struct('status','PASS_HOST_ONLY_CANONICAL_TASK_BINDING_TESTS', ...
    'tests',tests,'cases',numel(tests),'passed',sum([tests.pass]), ...
    'positive_assertions',positive.checks_total,'hardware_actions',0,'com_open',0,'board_access',0);
fprintf('CANONICAL_TASK_BINDING_TESTS passed=%d/%d positive_assertions=%d\n', ...
    result.passed,result.cases,result.positive_assertions);
    function reject(name,t,m)
        rejected=false;
        try
            validate_canonical_cambridge_binding(t,cfg,ledger,mission,m,timeline);
        catch exception
            assert(strcmp(exception.identifier,'gpenmpcTask:CanonicalMismatch'), ...
                'gpenmpcTask:UnexpectedTestError','Unexpected error in %s: %s',name,exception.message);
            rejected=true;
        end
        assert(rejected,'gpenmpcTask:NegativeNotRejected','Negative was accepted: %s',name);
        tests(end+1)=struct('name',string(name),'pass',true); %#ok<AGROW>
    end
end
