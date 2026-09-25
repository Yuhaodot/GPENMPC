function report=test_rfly_canonical_delivery_time_view(outputRoot)
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
[first,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{1},[0;0;0]);
checks=struct('name',{},'pass',{});selected=cell(5,1);
for k=1:5
    leg=b.legs{k};offset=0;
    if k==1
        tr=first;offset=25;
    else
        match=struct('capture_count',k-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',zeros(4,3),'current_vertical_frame_offset_m',0);
        [tr,~]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,match);
    end
    c=owner(k,leg.meta.payload_kg,0,'PREPARED_PAUSED');
    a=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr);
    check(sprintf('leg%d_start_uses_exact_saved_global_time',k),a.saved_task_time_s==leg.meta.saved_global_start_s);
    check(sprintf('leg%d_no_payload_or_ground_authority_created',k), ...
        ~a.actual_land_authorized&&~a.actual_ground_confirmed&&~a.payload_write_authorized);
    phases=unique([offset,offset+.01,offset+min(11,leg.meta.trajectory_duration_s), ...
        offset+leg.meta.trajectory_duration_s]);values=zeros(size(phases));
    for j=1:numel(phases)
        c.service.phase_s=phases(j);c.state='FLIGHT';
        z=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr);values(j)=z.saved_task_time_s;
        check(sprintf('leg%d_sample%d_direct_phase_mapping',k,j), ...
            abs(z.saved_leg_phase_s-min(phases(j)-offset,leg.meta.trajectory_duration_s))<1e-12);
    end
    check(sprintf('leg%d_clock_monotonic_without_wall_time',k),all(diff(values)>0)&&~z.wall_time_used);
    check(sprintf('leg%d_exact_terminal_saved_time',k),z.saved_task_time_s==leg.meta.saved_global_end_s);
    c.service.phase_s=offset+leg.meta.trajectory_duration_s+3;
    z=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr);
    check(sprintf('leg%d_overrun_disclosed_not_task_completion',k),z.controller_phase_past_saved_end_s>2.99 ...
        &&z.saved_task_time_s==leg.meta.saved_global_end_s&&~z.task_completion_proven);
    c.service.phase_s=offset+min(30,leg.meta.trajectory_duration_s);c.state='NATIVE_LAND';c.service.outer_suspended=true;
    land=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr);c.state='GROUND';
    ground=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr);
    check(sprintf('leg%d_land_ground_pause_same_saved_time',k),land.saved_task_time_s==ground.saved_task_time_s ...
        &&land.outer_already_suspended&&ground.outer_already_suspended);
    selected{k}=struct('start',a,'land',land,'ground',ground,'end_with_overrun',z);
    c.service.outer_suspended=false;
    reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr), ...
        'gpenmpcNative:DeliveryClockSuspension',sprintf('leg%d_label_without_actual_suspension_rejected',k));
    c.service.outer_suspended=true;c.service.payload_kg=c.service.payload_kg+.01;
    reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,tr), ...
        'gpenmpcNative:DeliveryClockPayload',sprintf('leg%d_unacknowledged_payload_change_rejected',k));
end
c=owner(1,b.legs{1}.meta.payload_kg,0,'FLIGHT');
for q=[0,10,20,24.99]
    c.service.phase_s=q;z=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first);
    check(sprintf('initial_prefix_%g_keeps_task_time_zero',q),z.saved_task_time_s==0&&z.inside_initial_preparation);
end
c.service.phase_s=25;z=gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first);
check('prefix_to_formal_joins_at_saved_zero_without_clock_reset',z.saved_task_time_s==0 ...
    &&z.controller_phase_s==25&&~z.inside_initial_preparation&&~z.source_clock_advanced);
c.service.phase_s=NaN;reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first), ...
    'gpenmpcNative:DeliveryClockState','nonfinite_phase_rejected');
c.service.phase_s=-.01;reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first), ...
    'gpenmpcNative:DeliveryClockState','negative_phase_rejected');
c.service.phase_s=0;c.failed=true;reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first), ...
    'gpenmpcNative:DeliveryClockOwner','failed_owner_rejected');
c.failed=false;c.state='UNKNOWN';reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,first), ...
    'gpenmpcNative:DeliveryClockLifecycle','unknown_lifecycle_rejected');
c.state='FLIGHT';bad=first;bad.canonical_task_phase_offset_s=20;
reject(@()gpenmpcNative.rflyCanonicalDeliveryTimeView(b,c,bad), ...
    'gpenmpcNative:DeliveryClockInitialPrefix','incorrect_initial_offset_rejected');
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks,'selected_views',{selected}, ...
    'task_source',b.receipt.task_path,'task_sha256',b.receipt.task_sha256, ...
    'clock_inputs','HOST_FIXTURES_OF_ACTUAL_COORDINATOR_STATUS_SCHEMA_NOT_LIVE_OBSERVATIONS', ...
    'original_task_rows',b.receipt.source_row_count,'new_source_clocks',0,'new_solvers',0, ...
    'COM_open',0,'board_actions',0,'plant_instances',0,'live_lifecycle_proven',false);
save(fullfile(outputRoot,'RAW.mat'),'report');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
d=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear d
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'hardware_actions',0)));
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));assert(ok,'gpenmpcNative:DeliveryTimeTest','%s',name);
    end
    function reject(fn,id,name)
        ok=false;try,fn();catch ex,ok=strcmp(ex.identifier,id);end;check(name,ok);
    end
end
function c=owner(k,payload,phase,state)
c=struct('failed',false,'state',state,'service',struct('leg_index',k,'phase_s',phase, ...
    'payload_kg',payload,'outer_suspended',false));
end
