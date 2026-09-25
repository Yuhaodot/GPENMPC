function report=test_rfly_canonical_delivery_reference(outputRoot)
% Test the complete saved task reference.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
p=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
h='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(p,h);r=b.task.reference;
checks=struct('name',{},'pass',{});cases=cell(5,1);sourceCount=0;orderCounts=zeros(1,4);
check('current_actual_task_and_five_legs',strcmpi(b.receipt.task_sha256,h)&&numel(b.legs)==5);
check('four_ground_services_eight_seconds',b.receipt.physical_ground_services==4 ...
    &&all(cellfun(@(x)x.ground_dwell_s==8,b.receipt.legs(1:4))));
for k=1:5
    a=b.legs{k};ix=a.source_indices;tr=a.trajectory;
    targets={a.position_up_m,a.velocity_mps,a.acceleration_mps2,a.jerk_mps3};
    errors=zeros(1,4);
    for order=0:3
        got=tr.evaluate_fcn(a.local_time_s,order).';
        errors(order+1)=max(abs(got-targets{order+1}),[],'all');
        check(sprintf('leg_%d_derivative_%d_all_original_samples_exact',k,order),errors(order+1)==0);
        mid=.5*(a.local_time_s(1:end-1)+a.local_time_s(2:end));
        want=interp1(a.local_time_s,targets{order+1},mid,'linear');
        check(sprintf('leg_%d_derivative_%d_intersample_semantics_identical',k,order), ...
            isequal(tr.evaluate_fcn(mid,order).',want));
        check(sprintf('leg_%d_derivative_%d_clamps_preserved',k,order), ...
            isequal(tr.evaluate_fcn(-1,order),targets{order+1}(1,:).') ...
            &&isequal(tr.evaluate_fcn(tr.total_duration_s+1,order),targets{order+1}(end,:).'));
        orderCounts(order+1)=orderCounts(order+1)+numel(ix);
    end
    origin=b.receipt.fixed_task_ground_origin_up_m;
    check(sprintf('leg_%d_only_constant_position_frame_translation',k), ...
        isequal(a.position_up_m,r.position_m(ix,:)-origin) ...
        &&isequal(a.velocity_mps,r.velocity_mps(ix,:)) ...
        &&isequal(a.acceleration_mps2,r.acceleration_mps2(ix,:)) ...
        &&isequal(a.jerk_mps3,r.jerk_mps3(ix,:)));
    check(sprintf('leg_%d_payload_and_saved_time_exact',k), ...
        all(r.payload_kg(ix)==a.meta.payload_kg)&&isequal(a.local_time_s,r.local_time_s(ix)));
    sourceCount=sourceCount+numel(ix);
    cases{k}=struct('meta',a.meta,'max_original_grid_error_p_v_a_j',errors, ...
        'source_rows',numel(ix),'midpoint_rows',numel(ix)-1);
end
check('whole_task_no_row_dropped_or_repeated',sourceCount==numel(r.global_time_s) ...
    &&isequal(vertcat(b.legs{1}.source_indices,b.legs{2}.source_indices, ...
    b.legs{3}.source_indices,b.legs{4}.source_indices,b.legs{5}.source_indices),(1:sourceCount).'));
check('four_payload_changes_stay_at_post_ground_ascent', ...
    numel(b.receipt.payload_change_rows)==4&&all(r.phase_code(b.receipt.payload_change_rows)==12));
check('no_control_truth_solver_or_io_created',b.receipt.hardware_actions==0 ...
    &&b.receipt.solver_calls==0&&b.receipt.plant_instances==0 ...
    &&~b.receipt.live_integration_proven&&~b.receipt.initial_takeoff_lifecycle_binding_complete);
wrong=h;wrong(1)='0';bad=false;
try,gpenmpcNative.loadRflyCanonicalDeliveryTask(p,wrong);catch ex,bad=strcmp(ex.identifier,'gpenmpcNative:DeliveryTaskIdentity');end
check('different_task_bytes_rejected_before_binding',bad);
for k=1:5
    bad=false;try,b.legs{k}.trajectory.evaluate_fcn(0,4);catch ex,bad=strcmp(ex.identifier,'gpenmpcSampledC3Trajectory:Derivative');end
    check(sprintf('leg_%d_unsupported_derivative_not_invented',k),bad);
end
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks, ...
    'cases',{cases},'receipt',b.receipt,'full_reference_rows',sourceCount, ...
    'original_grid_evaluations_by_derivative',orderCounts, ...
    'COM_open',0,'board_actions',0,'solver_calls',0,'plant_instances',0,'socket_count',0);
save(fullfile(outputRoot,'RAW.mat'),'report');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'reference_rows',sourceCount,'hardware_actions',0)));
assert(report.passed);
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));
        assert(ok,'gpenmpcNative:DeliveryReferenceTest','%s',name);
    end
end
