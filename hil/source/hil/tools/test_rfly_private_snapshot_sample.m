function report=test_rfly_private_snapshot_sample(outputRoot)
% Test snapshot decoding and solver-input mapping with fixture bytes.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path;cleanup=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot),'Preserve prior evidence.');mkdir(outputRoot);
input=fullfile(build,'rfly_vendor_integration','px4_wire','host_context_02','PRIVATE_SNAPSHOT_AND_CONTEXT.bin');
fid=fopen(input,'rb');assert(fid>=0);bytes=fread(fid,246,'*uint8');fclose(fid);
rx=uint64(9000000000);now=rx+uint64(1000);
s=gpenmpcNative.RflySnapshotDecoder(bytes,rx);
e=struct('uid',string(s.observed_uid),'system_id',double(s.source_system), ...
    'component_id',double(s.source_component),'boot_generation',s.observed_boot_generation, ...
    'maximum_age_ns',uint64(100000000),'configuration_payload_sha256', ...
    upper(reshape(dec2hex(s.configuration_sha256,2).',1,[])));
p=gpenmpcNative.RflySnapshotSample(bytes,rx,e);
checks=struct('name',{},'pass',{});
[x,ok,reason]=gpenmpcNative.px4EstimateState(p,e,now);
check('private_export_accepted_without_ODOMETRY_relabel',ok&&p.source=="PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1");
q=double(s.raw13_float32(7:10));q=q/norm(q);
oracle=[(double(s.raw13_float32(1:3))-s.task_origin_ned_m).*[1;1;-1]; ...
    double(s.raw13_float32(4:6)).*[1;1;-1];q.*[1;-1;-1;1]; ...
    double(s.raw13_float32(11:13)).*[-1;-1;1]];
check('all13_coordinate_values_equal_existing_kernel',isequal(x,oracle));
% Use a nonzero synthetic origin to distinguish raw-minus-origin from raw-plus-origin.
origin=[17.125;-23.25;4.75]; shiftedBytes=bytes;
v=origin;[~,~,endian]=computer;if endian=='L',v=swapbytes(v);end
shiftedBytes(87:110)=reshape(typecast(v,'uint8'),[],1);
digest=java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(shiftedBytes(1:214),'int8'));
shiftedBytes(215:246)=reshape(typecast(digest.digest(),'uint8'),[],1);
shifted=gpenmpcNative.RflySnapshotSample(shiftedBytes,rx,e);
[shiftedX,shiftedOk]=gpenmpcNative.px4EstimateState(shifted,e,now);
check('nonzero_origin_raw_minus_origin_exact',shiftedOk ...
    &&isequal(shiftedX(1:3),(double(s.raw13_float32(1:3))-origin).*[1;1;-1]));
check('origin_changes_only_position_three_fields',isequal(shiftedX(4:13),x(4:13)));
check('old_plus_origin_result_is_not_equivalent',~isequal(shiftedX(1:3), ...
    (double(s.raw13_float32(1:3))+origin).*[1;1;-1]));
tampered=shifted;tampered.position_ned_m=double(s.raw13_float32(1:3))+origin;
[~,accepted]=gpenmpcNative.px4EstimateState(tampered,e,now);
check('old_plus_origin_post_decode_shape_rejected',~accepted);
check('raw13_bits_retained',isequal(typecast(p.raw13_float32,'uint32'),typecast(s.raw13_float32,'uint32')));
check('three_original_HRT_preserved',p.original_sample_hrt_us==s.original_sample_hrt_us ...
    &&p.original_publication_hrt_us==s.original_publication_hrt_us ...
    &&p.original_board_receipt_hrt_us==s.original_board_receipt_hrt_us);
check('HOST_receipt_not_cast_from_board_HRT',p.position_rx_ns==rx&&p.sample_timestamp_ns==s.original_sample_hrt_us*uint64(1000) ...
    &&p.position_rx_ns~=p.sample_timestamp_ns&&~p.live_time_domain_binding);
check('identity_is_exact_uint64_not_double_UID',isa(p.boot_generation,'uint64') ...
    &&p.uid=="1234605616436508552"&&p.boot_generation==42);
legacy=p;legacy.source='PX4_EKF2_MAVLINK';
[oldx,oldok]=gpenmpcNative.px4EstimateState(legacy,e,now);
check('legacy_branch_arithmetic_unchanged',oldok&&isequal(x,oldx));
changed={'position_ned_m','velocity_ned_mps','quaternion_wxyz_body_to_ned','omega_frd_rad_s', ...
    'position_rx_ns','attitude_generation','source_ticket','raw13_float32','task_origin_ned_m', ...
    'original_sample_hrt_us','configuration_payload_sha256','plant_truth_used'};
for k=1:numel(changed)
    b=p;f=changed{k};
    if islogical(b.(f)),b.(f)=~b.(f);
    elseif ischar(b.(f)),b.(f)(1)='F';
    else,b.(f)(1)=b.(f)(1)+1;end
    [~,accepted]=gpenmpcNative.px4EstimateState(b,e,now);
    check("post_decode_mutation_"+f,~accepted);
end
b=p;b.source_export_bytes(110)=bitxor(b.source_export_bytes(110),uint8(1));
[~,accepted]=gpenmpcNative.px4EstimateState(b,e,now);check('raw_checksum_corruption_rejected',~accepted);
[~,accepted]=gpenmpcNative.px4EstimateState(p,e,rx+e.maximum_age_ns+uint64(1));
check('existing_HOST_stale_bound_preserved',~accepted);
[~,accepted]=gpenmpcNative.px4EstimateState(p,e,rx-uint64(1));
check('future_receipt_rejected',~accepted);
b=e;b.boot_generation=b.boot_generation+uint64(1);[~,accepted]=gpenmpcNative.px4EstimateState(p,b,now);
check('different_session_rejected',~accepted);
b=e;b.uid="3473490377090611258";[~,accepted]=gpenmpcNative.px4EstimateState(p,b,now);
check('different_UID_rejected',~accepted);
b=e;b.configuration_payload_sha256=repmat('0',1,64);[~,accepted]=gpenmpcNative.px4EstimateState(p,b,now);
check('different_configuration_rejected',~accepted);
b=e;b.boot_generation=NaN;[~,accepted]=gpenmpcNative.px4EstimateState(p,b,now);
check('unknown_expected_session_rejected',~accepted);
% Route through the existing outer snapshot builder with actual canonical
% config/assets but explicit HOST fixture actuator/wind and initial task state.
a=gpenmpcNative.loadCanonicalAssets(gpenmpcNative.canonicalAssetRoot());
e.maximum_runtime_age_ns=1e8;e.task_identity_sha256=repmat('A',1,64);
e.outer_snapshot_leg_index=1;e.outer_snapshot_first_in_leg=true;e.last_committed_inner_generation=0;
stamp=@(source)struct('source',source,'rx_ns',rx,'generation',1,'valid',true);
act=stamp('HOST_M600_VIRTUAL_ACTUATOR_INTERFACE');act.ordering='SOFTWARE_M600_ORDER';
act.rotor_command_n=ones(6,1);act.thrust_effectiveness=ones(6,1);
out=stamp('HOST_CAUSAL_RUNTIME');
out.previous_outer_acceleration_correction_i_mps2=zeros(3,1);
out.runtime_vertical_observer_shadow_i_mps2=zeros(3,1);out.residual_history_f_mps2=zeros(3,1);
out.runtime_gp_axis_weight_f=ones(3,1);out.runtime_gp_responsibility_blend=0;
out.runtime_gp_filtered_mean_f_mps2=zeros(3,1);out.causal_valid=false;
out.gp_evidence=struct('available',false,'prediction_sample_closed',false, ...
    'observed_innovation_available',false,'hard_invalid',false,'trust',0);
out.previous_desired_force_projected_n=(double(a.profile.mass_properties.base_mass_kg)+2.27).*[0;0;9.80665];
out.desired_force_source='CANONICAL_LEG_INITIAL_FORCE';out.desired_force_generation=0;
wind=stamp('FROZEN_TASK_WIND_ESTIMATOR');wind.estimate_xy_mps=zeros(2,1);
runtime=struct('schema','GPENMPC_LIVE_OUTER_RUNTIME_V1', ...
    'task_identity_sha256',e.task_identity_sha256,'plant_truth_used',false, ...
    'virtual_actuator',act,'outer',out,'wind',wind,'task', ...
    struct('source','FROZEN_GPENMPC_TASK_STATE','payload_kg',2.27, ...
    'corridor_half_width_m',a.enmpc.corridor_half_width_m,'leg_index',1));
phase=struct('progress_s',0,'progress_rate',1,'previous_phase_acceleration_s_inv',0);
coef=zeros(3,8);coef(:,1)=x(1:3);
trajectory=struct('total_duration_s',10,'coefficients_ascending',coef);
[snap,accepted,why,r]=gpenmpcNative.makeBoardOuterSnapshot(gpenmpcNative.canonicalAssetRoot(),p,e,now,runtime, ...
    trajectory,phase,zeros(1,a.enmpc.decision_dimension),a);
check('private_RSP1_reaches_canonical_outer_snapshot',accepted&&why=="FRESH_CAUSAL_BOARD_OUTER_SNAPSHOT");
check('source_not_misreported_as_MAVLink331',r.estimate_source=="PX4_PRIVATE_VEHICLE_ODOMETRY_RSP1");
check('outer_solver_observation13_exact',isequal(snap.observation.position_m,x(1:3)) ...
    &&isequal(snap.observation.velocity_mps,x(4:6))&&isequal(snap.observation.quaternion_wxyz,x(7:10)) ...
    &&isequal(snap.observation.body_rate_rad_s,x(11:13)));
report=struct('status','PASS_PRIVATE_RSP1_TO_CANONICAL_OUTER_INPUT', ...
    'checks',checks,'total',numel(checks),'passed',sum([checks.pass]), ...
    'input_scope','GENERATED_WIRE_FIXTURE', ...
    'solver_called',false,'live_session_registered',false,'board_actions',0, ...
    'legacy_reason',reason,'source_code',which('gpenmpcNative.px4EstimateState'));
save(fullfile(outputRoot,'RAW.mat'),'p','e','x','snap','r','report');
fo=fopen(fullfile(outputRoot,'RESULT.json'),'w');fwrite(fo,jsonencode(report,PrettyPrint=true),'char');fclose(fo);disp(report);
    function check(name,pass)
        checks(end+1)=struct('name',string(name),'pass',logical(pass)); %#ok<AGROW>
        assert(pass,'gpenmpcNative:PrivateSampleTest','%s',name);
    end
end
