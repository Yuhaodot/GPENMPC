function report=test_rfly_canonical_relaunch_reference(outputRoot)
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
addpath(fullfile(build,'support','delivery','matlab'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
checks=struct('name',{},'pass',{});receipts=cell(4,1);maxError=zeros(4,4);
offsets=[0,0,0;.7,-.3,0;-1.2,.6,0;2,0,0];zs=[0,.45,-.8,12];
fields={'position_ned_m','velocity_ned_mps','acceleration_ned_mps2','jerk_ned_mps3'};
for k=1:4
    leg=b.legs{k+1};state=struct('capture_count',k,'anchor_valid',false(1,4), ...
        'horizontal_offset_ned_m',offsets,'current_vertical_frame_offset_m',zs(k));state.anchor_valid(k)=true;
    [tr,receipts{k}]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,state);
    query=unique([0,.001,.25,1,3.7,7.9,10.999,11,11.001,24,leg.trajectory.total_duration_s]);
    for order=0:3
        got=tr.evaluate_fcn(query,order);want=zeros(size(got));
        for j=1:numel(query)
            nominal=struct;
            for n=1:4
                v=leg.trajectory.evaluate_fcn(query(j),n-1).';v(3)=-v(3);nominal.(fields{n})=v;
            end
            out=gpenmpcHil.applyRelaunchStateMatch(state,nominal,k,"SERVICE_ASCENT",query(j),11);
            v=out.(fields{order+1});v(3)=-v(3);want(:,j)=v.';
        end
        maxError(k,order+1)=max(abs(got-want),[],'all');
        check(sprintf('leg%d_original_helper_derivative%d_numeric_equivalence',k+1,order),maxError(k,order+1)<1e-12);
        original=leg.trajectory.evaluate_fcn(0,order);
        expected=original;if order==0,expected=expected+offsets(k,:).';expected(3)=expected(3)-zs(k);end
        check(sprintf('leg%d_t0_derivative%d_exact_captured_match',k+1,order),isequal(tr.evaluate_fcn(0,order),expected));
        finish=leg.trajectory.evaluate_fcn(11,order);
        if order==0,finish(3)=finish(3)-zs(k);end
        check(sprintf('leg%d_11s_derivative%d_no_horizontal_residual',k+1,order),isequal(tr.evaluate_fcn(11,order),finish));
    end
    check(sprintf('leg%d_same_duration_payload_and_nominal_arrays',k+1),tr.total_duration_s==leg.trajectory.total_duration_s ...
        &&~receipts{k}.nominal_reference_arrays_modified ...
        &&receipts{k}.outer_prediction_and_inner_reference_share_same_evaluator);
    state.anchor_valid(k)=false;reject(@()gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,state), ...
        'gpenmpcNative:RelaunchAnchor',sprintf('leg%d_missing_anchor_rejected',k+1));
end
state.anchor_valid(4)=true;state.current_vertical_frame_offset_m=12.001;
reject(@()gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(b.legs{5},state), ...
    'gpenmpcNative:RelaunchBounds','existing_vertical_bound_preserved');
state.current_vertical_frame_offset_m=0;state.horizontal_offset_ned_m(4,:)=[2.001,0,0];
reject(@()gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(b.legs{5},state), ...
    'gpenmpcNative:RelaunchBounds','existing_horizontal_bound_preserved');
reject(@()gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(b.legs{1},state), ...
    'gpenmpcNative:RelaunchLeg','initial_takeoff_not_silently_treated_as_ground_service');
reject(@()tr.evaluate_fcn(NaN,0),'gpenmpcNative:RelaunchQuery','nonfinite_query_rejected');
reject(@()tr.evaluate_fcn(0,4),'gpenmpcNative:RelaunchQuery','unsupported_derivative_rejected');
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks, ...
    'binding_receipts',{receipts},'max_abs_error_against_retained_helper',maxError, ...
    'ground_anchor_inputs','EXPLICIT_HOST_TEST_FIXTURES__NOT_ACTUAL_GROUND_OBSERVATIONS', ...
    'actual_saved_task_used',true,'live_integration_proven',false,'COM_open',0,'board_actions',0, ...
    'solver_calls',0,'plant_instances',0);
save(fullfile(outputRoot,'RAW.mat'),'report');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'max_error',max(maxError,[],'all'),'hardware_actions',0)));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));assert(ok,'gpenmpcNative:RelaunchTest','%s',name);
    end
    function reject(fn,id,name)
        ok=false;try,fn();catch ex,ok=strcmp(ex.identifier,id);end;check(name,ok);
    end
end
