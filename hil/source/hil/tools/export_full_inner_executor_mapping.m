function receipt=export_full_inner_executor_mapping(outputRoot,px4Float32)
% Export MATLAB mapping and kernel fixtures.
arguments
    outputRoot (1,1) string
    px4Float32 (1,1) logical = false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path;guard=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot),'Choose an unused output path.');mkdir(outputRoot);
source=fullfile(gpenmpc_external_path('canonical_full_inner_runtime'),'RAW.mat');
fixtureSource=fullfile(build,'tools','test_canonical_full_inner_runtime.m');
d=load(source,'raw');assets=gpenmpcNative.loadCanonicalAssets();
assert(numel(d.raw.samples)==60);
n=76;kernelFile=fullfile(outputRoot,'MATLAB_ARGUMENTS_AND_EXPECTED.bin');
mapFile=fullfile(outputRoot,'MATLAB_NED_AND_MAPPED_STATE.bin');
fk=fopen(kernelFile,'wb','ieee-be');assert(fk>=0);kg=onCleanup(@()fclose(fk));
fm=fopen(mapFile,'wb','ieee-le');assert(fm>=0);mg=onCleanup(@()fclose(fm));
fwrite(fk,uint32(n),'uint32');fwrite(fm,uint32(n),'uint32');
expected=struct('uid','HOST_ONLY_SYNTHETIC_SAMPLE','system_id',1, ...
    'component_id',1,'boot_generation',7,'maximum_age_ns',1e8);
maxOriginalDelta=0;normalizationDeviation=zeros(16,1);
representationDelta=zeros(n,1);actualKernelCalls=0;
for k=1:n
    if k<=60
        cmd=d.raw.samples{k}.begin.full_inner_command;
        p=[0;0;-2];v=zeros(3,1);q=[1;0;0;0];omega=zeros(3,1);
    else
        % Use nontrivial numerical fixtures and compute expected coordinates
        % with the MATLAB adapter.
        j=k-60;cmd=d.raw.samples{mod(j-1,60)+1}.begin.full_inner_command;
        axis=[1+.1*j;-.3+.03*j;.2];axis=axis/norm(axis);
        theta=(j-8)*pi/9;q=[cos(theta/2);axis*sin(theta/2)];
        scale=1+5e-7*(-1)^j;q=q*scale;
        normalizationDeviation(j)=norm(q)-1;
        p=[.12*j;-.03*j;-2-.07*j];v=[-.01*j;.02*j;-.015*j];
        omega=[.013*j;-.008*j;.006*j];
    end
    % Model the 13 float32 fields in PX4 vehicle_odometry_s explicitly before
    % calling the MATLAB mapper and kernel.
    if px4Float32
        original=[p;v;q;omega];represented=double(single(original));
        representationDelta(k)=max(abs(represented-original));
        p=represented(1:3);v=represented(4:6);q=represented(7:10);omega=represented(11:13);
    end
    now=double(1e9+k*1e7);generation=uint64(k);
    sample=struct('source','PX4_EKF2_MAVLINK_ODOMETRY_331', ...
        'uid',expected.uid,'system_id',1,'component_id',1,'boot_generation',7, ...
        'position_ned_m',p,'velocity_ned_mps',v, ...
        'quaternion_wxyz_body_to_ned',q,'omega_frd_rad_s',omega, ...
        'position_rx_ns',now-1e5,'attitude_rx_ns',now-1e5,'rates_rx_ns',now-1e5, ...
        'position_generation',generation,'attitude_generation',generation, ...
        'rates_generation',generation,'position_valid',true,'attitude_valid',true, ...
        'rates_valid',true,'atomic_estimate',true,'plant_truth_used',false, ...
        'sample_timestamp_ns',generation*uint64(1e7),'odometry_reset_counter',2, ...
        'odometry_frame_id',1,'odometry_child_frame_id',1,'odometry_estimator_type',8);
    [x,ok,reason]=gpenmpcNative.px4EstimateState(sample,expected,now);
    assert(ok,'%s',reason);
    if k<=60
        delta=max(abs(cmd.state_up(1:13)-x));
        maxOriginalDelta=max(maxOriginalDelta,delta);assert(delta==0);
    end
    if k<=60 && ~px4Float32
        feedback=d.raw.samples{k}.feedback;
    else
        cmd.state_up=[x;zeros(6,1)];cmd.generation=generation;
        cmd.estimate_sample_timestamp_ns=sample.sample_timestamp_ns;
        feedback=gpenmpcNative.executeCanonicalFullInnerKernelHost(cmd,assets,now+1e6);
        actualKernelCalls=actualKernelCalls+1;
    end
    assert(feedback.valid);
    [bytes,r]=gpenmpcNative.encodeCanonicalFullInnerArguments(cmd,assets);
    fwrite(fk,bytes,'uint8');fwrite(fk,uint8(sscanf(r.kernel_argument_sha256,'%2x')),'uint8');
    fwrite(fk,[feedback.wrench_n_nm;feedback.rotor_command_n;feedback.diagnostic51],'double');
    fwrite(fk,uint8(feedback.valid),'uint8');
    fwrite(fm,generation,'uint64');fwrite(fm,[p;v;q;omega;x],'double');
end
clear kg mg
% Check the norm boundary.
sample.quaternion_wxyz_body_to_ned=[1+2e-6;0;0;0];
[~,accepted,reason]=gpenmpcNative.px4EstimateState(sample,expected,now);
assert(~accepted&&strcmp(reason,'NONUNIT_QUATERNION'));
receipt=struct('status','PASS_HOST_ONLY_ACTUAL_MATLAB_MAPPING_FIXTURE', ...
    'original_rows',60,'additional_mapping_rows',16,'total_rows',n, ...
    'original_mapping_max_error',maxOriginalDelta, ...
    'actual_adapter_calls',77,'additional_actual_matlab_kernel_calls',actualKernelCalls, ...
    'px4_float32_source_representation',px4Float32, ...
    'source_representation_max_delta',max(representationDelta), ...
    'source_representation_deltas',representationDelta, ...
    'source_representation_is_measured_telemetry',false, ...
    'unchanged_norm_guard_rejection_pass',true, ...
    'nontrivial_quaternion_norm_offsets',normalizationDeviation, ...
    'original_pre_map_observations_saved_in_raw',false, ...
    'original_input_provenance','Inputs reconstructed from the test observations() function.', ...
    'source_raw_sha256',gpenmpcNative.fileSha256(source), ...
    'original_fixture_source_sha256',gpenmpcNative.fileSha256(fixtureSource), ...
    'mapping_source_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.px4EstimateState')), ...
    'exporter_source_sha256',gpenmpcNative.fileSha256(mfilename('fullpath')+".m"), ...
    'kernel_fixture_sha256',gpenmpcNative.fileSha256(kernelFile), ...
    'mapping_fixture_sha256',gpenmpcNative.fileSha256(mapFile), ...
    'solver_calls',0,'model_count',0,'com_open',0,'board_actions',0,'publication_count',0);
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);
fg=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(receipt,PrettyPrint=true));
disp(jsonencode(receipt));
end
