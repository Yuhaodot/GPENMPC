function report=export_canonical_reference_native_fixture(outputRoot)
% Small actual five-leg reference fixture; original functions are the oracle.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
a=gpenmpcNative.loadCanonicalAssets();
taskPath=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);asset=uint8(sscanf(taskSha,'%2x'));
records={};offsets=[0,0,0;.7,-.3,0;-1.2,.6,0;2,0,0];zs=[0,.45,-.8,12];
for legIndex=1:5
    leg=b.legs{legIndex};shift=0;
    if legIndex==1
        [tr,binding]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,zeros(3,1));shift=25;
    else
        s=struct('capture_count',legIndex-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',offsets,'current_vertical_frame_offset_m',zs(legIndex-1));
        [tr,binding]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,s);
    end
    duration=leg.trajectory.total_duration_s;
    queries=shift+[0,.001,1.234,2.553,2.556,10.999,11,11.001,duration/2,duration];
    if legIndex==1,queries=[0,3.7319,19.999,20,20+eps(20),22.375,25-eps(25),queries];end
    for phase=queries
        nominal=max(0,min(phase-shift,duration));
        ix=find(leg.local_time_s<=nominal,1,'last');first=max(1,min(ix-1,numel(leg.local_time_s)-1));
        k=numel(records)+1;
        [w,s]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,first,uint64(k),asset,uint64(k-1));
        request=struct('reference_asset_sha256',asset,'leg_index',uint32(legIndex), ...
            'window_generation',uint64(k),'query_sequence',uint64(k),'progress_s',phase);
        args=[.93;.07;-.04;.01;-.02;.03;.13;-.09;.06;.009;a.enmpc.reference_transition_jerk_limit_mps3];
        expectedJet=zeros(3,4);for order=0:3,expectedJet(:,order+1)=tr.evaluate_fcn(phase,order);end
        t=gpenmpcJerkBoundedReferenceTransition(tr,phase,args(1),args(2),args(3),args(4:6),args(7:9),args(10),args(11));
        expectedTransition=[t.reference.position_m;t.reference.velocity_mps;t.reference.acceleration_mps2; ...
            t.reference.jerk_mps3;t.phase_acceleration_s_inv;t.phase_jerk_s_inv2; ...
            t.outer_correction_i_mps2;t.outer_correction_jerk_i_mps3;t.fraction;t.frame_i_from_f(:); ...
            t.reference_frame_i_from_f(:);t.reference_curvature;t.reference_signed_yaw_rate];
        assert(numel(expectedTransition)==41);
        records{end+1}=struct('window',w,'state',s,'request',request, ...
            'arguments',args,'expected_jet',expectedJet,'expected_transition',expectedTransition); %#ok<AGROW>
    end
end
path=fullfile(outputRoot,'NATIVE_REFERENCE_FIXTURE.bin');assert(~isfile(path));
f=fopen(path,'w','ieee-le');assert(f>=0);c=onCleanup(@()fclose(f));
fwrite(f,uint8('RWJ1'),'uint8');fwrite(f,uint32(numel(records)),'uint32');
for k=1:numel(records)
    r=records{k};w=r.window;
    fwrite(f,uint32([w.schema,w.capacity,w.leg_index,w.source_first_row,w.source_total_rows,w.row_count,w.binding_mode]),'uint32');
    fwrite(f,w.reference_asset_sha256,'uint8');fwrite(f,w.window_generation,'uint64');
    fwrite(f,[w.nominal_duration_s,w.total_duration_s,w.prefix_duration_s,w.relaunch_duration_s,w.vertical_frame_offset_ned_m],'double');
    fwrite(f,w.time_s,'double');fwrite(f,w.nominal_jet,'double');fwrite(f,w.prefix_coefficients,'double');
    fwrite(f,w.ground_jet,'double');fwrite(f,w.rest_jet,'double');fwrite(f,w.relaunch_offset_ned_m,'double');
    fwrite(f,r.state.last_accepted_sequence,'uint64');fwrite(f,r.request.query_sequence,'uint64');
    fwrite(f,r.request.progress_s,'double');fwrite(f,r.arguments,'double');
    fwrite(f,r.expected_jet,'double');fwrite(f,r.expected_transition,'double');
end
clear c
report=struct('schema','CANONICAL_REFERENCE_NATIVE_FIXTURE_V1','records',numel(records), ...
    'actual_legs',5,'task_sha256',taskSha,'layout','RWJ1 uint32count; LE scalar metadata and MATLAB column-major binary64 arrays', ...
    'oracle','Original bound trajectory and gpenmpcJerkBoundedReferenceTransition, not generated C', ...
    'hardware_actions',0,'solver_calls',0,'model_calls',0);
save(fullfile(outputRoot,'NATIVE_REFERENCE_FIXTURE.mat'),'records','report','-v7');
f=fopen(fullfile(outputRoot,'NATIVE_FIXTURE_RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s',jsonencode(report,PrettyPrint=true));
fprintf('NATIVE_REFERENCE_FIXTURE records=%d\n',numel(records));
end
