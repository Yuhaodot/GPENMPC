function report=export_continuous_reference_fixture(outputRoot)
% Export inputs for the reference-only comparisons.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
a=gpenmpcNative.loadCanonicalAssets();
source=fullfile(gpenmpc_external_path('canonical_reference_window_transition'),'RAW.mat');
q=load(source,'raw','report');assert(q.report.pass&&numel(q.raw)==3765);
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(task,taskSha);asset=uint8(sscanf(taskSha,'%2x'));
bindings=cell(5,1);offsets=[0,0,0;.7,-.3,0;-1.2,.6,0;2,0,0];zs=[0,.45,-.8,12];
for leg=1:5
    if leg==1,[~,bindings{leg}]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{leg},zeros(3,1));
    else
        s=struct('capture_count',leg-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',offsets,'current_vertical_frame_offset_m',zs(leg-1));
        [~,bindings{leg}]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(b.legs{leg},s);
    end
end
generations=unique(cellfun(@(r)r.window_generation,q.raw),'stable');windows=cell(numel(generations),1);
for j=1:numel(generations)
    ix=find(cellfun(@(r)r.window_generation==generations(j),q.raw),1);r=q.raw{ix};
    first=double(r.window_receipt.source_first_row);if first==0,first=1;end
    [windows{j},~]=gpenmpcNative.prepareCanonicalReferenceWindow(b.legs{r.leg},bindings{r.leg}, ...
        first,generations(j),asset,uint64(ix-1));
end
file=fullfile(outputRoot,'CONTINUOUS_REFERENCE_FIXTURE.bin');assert(~isfile(file));
f=fopen(file,'w','ieee-le');assert(f>=0);cleanup=onCleanup(@()fclose(f));
fwrite(f,uint8('RWI1'),'uint8');fwrite(f,uint32([numel(windows),numel(q.raw)]),'uint32');
for j=1:numel(windows)
    w=windows{j};
    fwrite(f,uint32([w.schema,w.capacity,w.leg_index,w.source_first_row,w.source_total_rows,w.row_count,w.binding_mode]),'uint32');
    fwrite(f,w.reference_asset_sha256,'uint8');fwrite(f,w.window_generation,'uint64');
    fwrite(f,[w.nominal_duration_s,w.total_duration_s,w.prefix_duration_s,w.relaunch_duration_s,w.vertical_frame_offset_ned_m],'double');
    fwrite(f,w.time_s,'double');fwrite(f,w.nominal_jet,'double');fwrite(f,w.prefix_coefficients,'double');
    fwrite(f,w.ground_jet,'double');fwrite(f,w.rest_jet,'double');fwrite(f,w.relaunch_offset_ned_m,'double');
end
legRow=0;lastLeg=0;previousA=0;previousI=zeros(3,1);
for j=1:numel(q.raw)
    r=q.raw{j};
    if r.leg~=lastLeg,legRow=0;previousA=.07;previousI=[.01;-.02;.03];lastLeg=r.leg;end
    legRow=legRow+1;window=find(generations==r.window_generation);
    % Use the test's explicit formulas and prior oracle values.
    args=[.91+.11*sin(.03*legRow);previousA;.04*cos(.07*legRow);previousI; ...
        .13*sin(.09*legRow);-.09*cos(.05*legRow);.06*sin(.04*legRow);.009;a.enmpc.reference_transition_jerk_limit_mps3];
    t=r.expected;
    expected=[t.reference.position_m;t.reference.velocity_mps;t.reference.acceleration_mps2;t.reference.jerk_mps3; ...
        t.phase_acceleration_s_inv;t.phase_jerk_s_inv2;t.outer_correction_i_mps2;t.outer_correction_jerk_i_mps3; ...
        t.fraction;t.frame_i_from_f(:);t.reference_frame_i_from_f(:);t.reference_curvature;t.reference_signed_yaw_rate];
    fwrite(f,uint32([window,r.leg,legRow]),'uint32');fwrite(f,r.query_sequence,'uint64');
    fwrite(f,r.phase_s,'double');fwrite(f,args,'double');fwrite(f,r.jet,'double');fwrite(f,expected,'double');
    previousA=t.phase_acceleration_s_inv;previousI=t.outer_correction_i_mps2;
end
clear cleanup
report=struct('schema','CONTINUOUS_REFERENCE_NATIVE_FIXTURE_V1','source',char(source), ...
    'windows',numel(windows),'queries',numel(q.raw),'old_checks',q.report.checks, ...
    'old_checks_total',q.report.checks_total,'expected_refills',q.report.refills, ...
    'expected_identity_rejections',q.report.identity_or_sequence_rejected, ...
    'task_sha256',taskSha,'separate_inner_fixture',true, ...
    'reference_transition_jerk_limit_mps3',a.enmpc.reference_transition_jerk_limit_mps3, ...
    'reason','Inner60 uses independently assigned synthetic acceleration and a separate reference fixture.', ...
    'arguments_provenance','Deterministic caller formulas from test_canonical_reference_window_transition.m.', ...
    'hardware_actions',0,'model_calls',0,'solver_calls',0);
out=fullfile(outputRoot,'CONTINUOUS_REFERENCE_FIXTURE.json');assert(~isfile(out));
f=fopen(out,'w');assert(f>=0);cleanup=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(report,PrettyPrint=true));
fprintf('CONTINUOUS_REFERENCE_EXPORT windows=%d queries=%d\n',numel(windows),numel(q.raw));
end
