function report=test_canonical_full_se3_kernel_rebinding(outRoot)
% Bind the generated SE(3) kernel to canonical assets.
buildRoot=string(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1
    outRoot=fullfile(buildRoot,'evidence', ...
        "canonical_full_se3_kernel_rebinding_"+string(datetime('now','Format','yyyyMMdd_HHmmss')));
end
outRoot=string(outRoot);
assert(~isfolder(outRoot),'gpenmpc:OutputExists','Never overwrite test evidence.');
assert(startsWith(lower(string(java.io.File(char(outRoot)).getCanonicalPath())), ...
    lower(string(java.io.File(char(fullfile(buildRoot,'evidence'))).getCanonicalPath())+filesep)), ...
    'gpenmpc:OutputScope','Output must be a fresh directory below BUILD/evidence.');
mkdir(outRoot);oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
oldHost=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(oldHost,'src'),'-end');addpath(fullfile(buildRoot,'host_runtime'),'-begin');
report=struct('status','RUNNING','scope','HOST_ONLY_FULL_ALGEBRAIC_SE3_KERNEL_REBINDING', ...
    'output_root',outRoot,'hardware_actions',0,'plant_integrations',0, ...
    'outer_solver_calls',0,'learning_data_generated',0,'px4_builds',0);
records=struct('path',{},'bytes',{},'sha256',{});
checks=struct('name',{},'pass',{});cases=cell(0,1);rows=struct([]);
oracle61=zeros(0,61);kernel61=zeros(0,61);
try
    a=gpenmpcNative.loadCanonicalAssets(gpenmpcNative.canonicalAssetRoot());
    report.configuration_payload_sha256=a.binding.effective_configuration_payload_sha256;
    report.source_manifest_sha256=a.binding.source_manifest_sha256;
    assert(a.attitudeContinuity.enabled,'Current continuity branch must be enabled.');
    kernelPath=fullfile(oldHost,'src','+gpenmpcNative','se3WrenchKernel.m');
    bind(kernelPath,'52F714D65C4A3CD5879FFD4478F60379F68CD5F65A38F5971B91BBC01D0E4AB8');
    cRoot=string(gpenmpc_external_path('se3_generated_parity_build'));
    bind(fullfile(cRoot,'generated_c','gpenmpcNative_se3WrenchKernel.c'), ...
        '2BB86D0A7C32CB6C505748DAEA9A8B2069CA172B90E256A555345901ABB5A95A');
    cExe=fullfile(cRoot,'se3_c_parity.exe');
    bind(cExe,'5252505D8D6DBCD71CA9A375433DA5CEF0D8D5A92ECEB9163971587DF382BBEB');
    oldFixtures=string(gpenmpc_external_path('se3_parity_fixtures'));
    bind(oldFixtures,'BF50060CCF0B93B3F1B01D91225BF122F492F15E64BD2F0D4788A40BD52BA0AF');
    tracePath=string(gpenmpc_external_path('canonical_se3_replay_trace'));
    bind(tracePath,'D97A508FD6D04AF806A02C49DC26906743076D8658F49E195DF1A14251496C7A');
    names={'gpenmpcRobustSe3Control.m','gpenmpcDesiredSe3Command.m', ...
        'gpenmpcM600Allocation.m','gpenmpcProjectForce.m','gpenmpcUpdateDesiredAttitudeContinuity.m'};
    for k=1:numel(names)
        canonical=fullfile(a.sourceRoot,'dynamics',names{k});
        inherited=fullfile(oldHost,'vendor','current_method','implementation','dynamics',names{k});
        check("SOURCE_BYTE_EQ_"+string(names{k}),sha(canonical)==sha(inherited));
        bind(canonical,sha(canonical));
    end
    for name=["M600_DYN_CALIBRATION.json","M600_PLATFORM_PROFILE.json"]
        canonical=fullfile(a.sourceRoot,'simulink','assets',name);
        inherited=fullfile(oldHost,'vendor','current_method','implementation','simulink','assets',name);
        check("PARAMETER_BYTE_EQ_"+name,sha(canonical)==sha(inherited));
        bind(canonical,sha(canonical));
    end
    loaded=load(tracePath,'trace');tr=loaded.trace;
    required={'global_time_s','leg_index','payload_kg','position_m','velocity_mps', ...
        'quaternion_wxyz','body_rate_rad_s','per_rotor_thrust_n', ...
        'reference_position_m','reference_velocity_mps','reference_acceleration_mps2', ...
        'wind_estimate_xy_mps','robust_compensation_i_mps2', ...
        'desired_rotation_matrix_i_from_b','desired_angular_velocity_body_rad_s', ...
        'desired_angular_acceleration_body_rad_s2','attitude_continuity_enabled'};
    check('ACTUAL_CANONICAL_TRACE_SCHEMA',all(isfield(tr,required)));
    continuityEnabled=tr.attitude_continuity_enabled;
    check('CONTINUITY_FLAG_SCALAR_OR_ROW_ALIGNED', ...
        isscalar(continuityEnabled)||numel(continuityEnabled)==numel(tr.global_time_s));
    legs=unique(tr.leg_index(:).','stable');legs=legs(legs>0);
    continuityMaximum=0;nonzeroOmega=0;nonzeroOmegaDot=0;resetCount=0;
    physicalGpInputCount=0;sequenceCount=0;
    for leg=legs
        ix=find(tr.leg_index==leg);n=numel(ix);
        v=tr.reference_velocity_mps(ix,:);acc=tr.reference_acceleration_mps2(ix,:);
        [~,turn]=max(vecnorm(cross(v,acc,2),2,2));
        starts=unique([1,max(1,floor(n/2)-59),max(1,turn-59)],'stable');
        for start=starts
            indices=ix(start:min(start+119,n));
            state=gpenmpcInitializeDesiredAttitudeContinuityState();
            if start>1
                previous=ix(start-1);
                state.initialized=true;state.angular_velocity_valid=start>2;
                state.angular_acceleration_valid=start>3;
                state.filtered_rotation=reshape(tr.desired_rotation_matrix_i_from_b(previous,:),3,3);
                state.desired_angular_velocity_body_rad_s=tr.desired_angular_velocity_body_rad_s(previous,:).';
                state.desired_angular_acceleration_body_rad_s2=tr.desired_angular_acceleration_body_rad_s2(previous,:).';
                state.update_count=start-1;state.reset_count=1;
            end
            sequenceCount=sequenceCount+1;
            for j=1:numel(indices)
                i=indices(j);ordinal=start+j-1;isReset=ordinal==1;
                if isscalar(continuityEnabled)
                    checkCurrent=logical(continuityEnabled);
                else
                    checkCurrent=logical(continuityEnabled(i));
                end
                assert(checkCurrent,'Canonical trace continuity is disabled.');
                x=[tr.position_m(i,:),tr.velocity_mps(i,:),tr.quaternion_wxyz(i,:), ...
                    tr.body_rate_rad_s(i,:),tr.per_rotor_thrust_n(i,:)].';
                ref=struct('position_m',tr.reference_position_m(i,:).', ...
                    'velocity_mps',tr.reference_velocity_mps(i,:).', ...
                    'acceleration_mps2',tr.reference_acceleration_mps2(i,:).');
                aug=tr.robust_compensation_i_mps2(i,:).';wind=tr.wind_estimate_xy_mps(i,:).';
                raw=gpenmpcDesiredSe3Command(x,ref,tr.payload_kg(i),wind,aug,a.calibration,a.profile);
                % WholeTask uses exact 0.01; no telemetry freshness is invented.
                [attitude,state]=gpenmpcUpdateDesiredAttitudeContinuity( ...
                    state,raw.desired_rotation,0.01,isReset,a.attitudeContinuity);
                stored=[tr.desired_rotation_matrix_i_from_b(i,:).'; ...
                    tr.desired_angular_velocity_body_rad_s(i,:).'; ...
                    tr.desired_angular_acceleration_body_rad_s2(i,:).'];
                rebuilt=[attitude.desired_rotation(:);attitude.desired_angular_velocity_body_rad_s; ...
                    attitude.desired_angular_acceleration_body_rad_s2];
                continuityMaximum=max(continuityMaximum,max(abs(rebuilt-stored)));
                args={x,ref.position_m,ref.velocity_mps,ref.acceleration_mps2,tr.payload_kg(i), ...
                    wind,aug,true,attitude.desired_rotation, ...
                    attitude.desired_angular_velocity_body_rad_s, ...
                    attitude.desired_angular_acceleration_body_rad_s2,a.kernelParameters};
                addCase(args,"CANONICAL_CONTINUOUS_REPLAY",i,leg,isReset);
                nonzeroOmega=nonzeroOmega+double(norm(args{10})>1e-9);
                nonzeroOmegaDot=nonzeroOmegaDot+double(norm(args{11})>1e-9);
                resetCount=resetCount+double(isReset);
                if isfield(tr,'gp_execution_feedforward_i_mps2')
                    physicalGpInputCount=physicalGpInputCount+double(norm(tr.gp_execution_feedforward_i_mps2(i,:))>1e-12);
                end
            end
        end
    end
    check('CONTINUITY_RECONSTRUCTION_MATCHES_STORED_CANONICAL',continuityMaximum<1e-8);
    check('CONTINUITY_RESET_AND_NONZERO_OMEGA_OMEGADOT',resetCount>=1&&nonzeroOmega>0&&nonzeroOmegaDot>0);
    check('MULTIPLE_CANONICAL_PAYLOADS',numel(unique(tr.payload_kg(tr.leg_index>0)))>1);
    saved=load(oldFixtures,'fixtures','negatives');
    for k=1:numel(saved.fixtures)
        args=saved.fixtures{k}.args;args{12}=a.kernelParameters;
        addCase(args,"EXISTING_FIXED_FIXTURE",k,0,false);
    end
    for k=1:numel(saved.negatives)
        args=saved.negatives{k};args{12}=a.kernelParameters;
        if k==5,args{12}.baseMass=0;end
        [w,r,d,valid]=gpenmpcNative.se3WrenchKernel(args{:});
        check("EXISTING_NEGATIVE_"+k,~valid&&all([w;r;d]==0));
        cases{end+1}=struct('args',{args},'expected',zeros(61,1),'valid',false); %#ok<AGROW>
    end
    % Real input conversion helper: individual roll/pitch/yaw signs and q/-q.
    C=diag([1,1,-1]);A=-C;
    expected=struct('uid','HOST_FIXED_INPUT_ONLY','system_id',1,'component_id',1, ...
        'boot_generation',1,'maximum_age_ns',1e8);
    for axis=1:3
        q=zeros(4,1);q(1)=cos(0.2);q(axis+1)=sin(0.2);
        omega=zeros(3,1);omega(axis)=0.3;
        sample=struct('source','PX4_EKF2_MAVLINK','uid',expected.uid, ...
            'system_id',1,'component_id',1,'boot_generation',1, ...
            'position_ned_m',[1;2;3],'velocity_ned_mps',[.1;.2;.3], ...
            'quaternion_wxyz_body_to_ned',q,'omega_frd_rad_s',omega, ...
            'position_rx_ns',1e9,'attitude_rx_ns',1e9,'rates_rx_ns',1e9, ...
            'position_generation',1,'attitude_generation',1,'rates_generation',1, ...
            'position_valid',true,'attitude_valid',true,'rates_valid',true);
        [x,ok,~]=gpenmpcNative.px4EstimateState(sample,expected,1e9);
        check("REAL_PX4_FRAME_AXIS_"+axis,ok&&max(abs(x(1:6)-[C*[1;2;3];C*[.1;.2;.3]]))<1e-14 ...
            &&norm(gpenmpcQuaternionRotation(x(7:10))-C*gpenmpcQuaternionRotation(q)*C,'fro')<1e-12 ...
            &&norm(x(11:13)-A*omega)<1e-14);
        sample.quaternion_wxyz_body_to_ned=-q;[other,ok,~]=gpenmpcNative.px4EstimateState(sample,expected,1e9);
        check("QUATERNION_SIGN_AXIS_"+axis,ok&&norm(gpenmpcQuaternionRotation(other(7:10))-gpenmpcQuaternionRotation(x(7:10)),'fro')<1e-12);
        args=saved.fixtures{1}.args;args{12}=a.kernelParameters;args{8}=true;
        args{9}=eye(3);args{10}=zeros(3,1);args{10}(axis)=0.1;args{11}=zeros(3,1);
        [w,~,~,valid]=gpenmpcNative.se3WrenchKernel(args{:});
        desired=zeros(3,1);desired(axis)=a.kernelParameters.kw(axis)*0.1;
        check("CANONICAL_SINGLE_AXIS_MOMENT_"+axis,valid&&norm(w(2:4)-desired)<1e-12);
        addCase(args,"SINGLE_AXIS_MOMENT",axis,0,false);
    end
    alloc=gpenmpcM600Allocation(a.calibration);
    check('CANONICAL_ALLOCATION_RIGHT_INVERSE',norm(alloc.matrix*alloc.pseudoinverse-eye(4),'fro')<1e-12);
    for rotorIndex=1:6
        unit=zeros(6,1);unit(rotorIndex)=1;
        theta=deg2rad(a.calibration.rotor_allocation.angles_deg(rotorIndex));
        column=[1;a.calibration.rotor_allocation.arm_radius_m*sin(theta); ...
            -a.calibration.rotor_allocation.arm_radius_m*cos(theta); ...
            a.calibration.rotor_allocation.yaw_moment_arm_nominal_m* ...
            a.calibration.rotor_allocation.spin_sign(rotorIndex)];
        check("CANONICAL_ROTOR_GEOMETRY_COLUMN_"+rotorIndex,norm(alloc.matrix*unit-column)<1e-12);
    end
    mappingChecks=exerciseExistingRotorMapping(a.kernelParameters.rotorUpper);
    for k=1:numel(mappingChecks),check(mappingChecks(k).name,mappingChecks(k).pass);end
    binPath=fullfile(outRoot,'EXISTING_GENERATED_C_INPUT_LE.bin');writeBinary(binPath,cases);
    % Existing narrow-argv C driver reads an ASCII relative filename, while
    % MATLAB sets the Unicode evidence working directory through its API.
    previousDirectory=pwd;directoryGuard=onCleanup(@()cd(previousDirectory));
    cd(outRoot);
    [exitCode,stdout]=system(char('"'+cExe+'" "EXISTING_GENERATED_C_INPUT_LE.bin"'));
    clear directoryGuard
    writeText(fullfile(outRoot,'GENERATED_C_RAW.txt'),stdout);
    check('EXISTING_GENERATED_C_EXECUTION',exitCode==0);
    cResult=jsondecode(stdout);
    check('GENERATED_C_COMPLETE_DENOMINATOR',cResult.positive_cases+cResult.negative_cases==numel(cases));
    check('GENERATED_C_ALL_61_OUTPUTS_PARITY',cResult.maximum_absolute_error<1e-10);
    raw=struct('oracle61',oracle61,'kernel61',kernel61,'output_count_per_case',61, ...
        'row_index',rows,'input_sources',records); %#ok<NASGU>
    save(fullfile(outRoot,'RAW_REBINDING.mat'),'raw','-v7');
    writetable(struct2table(rows),fullfile(outRoot,'SAMPLE_RESULTS.csv'));
    report.status='PASS_HOST_ONLY_CANONICAL_FULL_ALGEBRAIC_SE3_KERNEL_REBINDING';
    report.current_continuity_enabled=true;report.continuous_sequence_count=sequenceCount;
    report.continuous_replay_samples=sum(string({rows.kind})=="CANONICAL_CONTINUOUS_REPLAY");
    report.reset_samples=resetCount;report.nonzero_desired_omega_samples=nonzeroOmega;
    report.nonzero_desired_omega_dot_samples=nonzeroOmegaDot;
    report.nonzero_canonical_gp_feedforward_input_samples=physicalGpInputCount;
    report.continuity_trace_maximum_absolute_error=continuityMaximum;
    report.matlab_full_61_maximum_absolute_error=max(abs(oracle61-kernel61),[],'all');
    report.generated_c=cResult;report.checks=checks;report.check_count=numel(checks);
    report.input_identities=records;
    report.limitations={ ...
        'Replay uses canonical software data.', ...
        'Kernel consumes current augmentation and continuous attitude command; it does not execute eNMPC, GP inference or their state updates.', ...
        'Existing 93 rotor permutation is tested offline; its live mapping and a future full-kernel output interface remain unverified.', ...
        'SI wrench to PX4 normalized allocator equivalence is NOT established; current mode1 attitude-only deployment is not made equivalent by this test.', ...
        'Assessment covers the recorded numerical replay.'};
catch err
    report.status='HOST_ONLY_REBINDING_IMPLEMENTATION_OR_EQUIVALENCE_FAILURE';
    report.error_identifier=err.identifier;report.error=getReport(err,'extended','hyperlinks','off');
    report.checks=checks;report.input_identities=records;
    writeText(fullfile(outRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
    rethrow(err)
end
writeText(fullfile(outRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(jsonencode(report,PrettyPrint=true));

    function bind(p,expectedSha)
        actual=sha(p);assert(strcmpi(actual,expectedSha),'gpenmpc:InputHash','Input hash mismatch: %s',p);
        item=dir(p);records(end+1)=struct('path',string(p),'bytes',item.bytes,'sha256',actual);
    end
    function check(name,value)
        checks(end+1)=struct('name',string(name),'pass',logical(value));
        assert(value,'gpenmpc:RebindingCheck','Check failed: %s',name);
    end
    function addCase(args,kind,index,leg,reset)
        ref=struct('position_m',args{2},'velocity_mps',args{3},'acceleration_mps2',args{4});
        att=struct('enabled',args{8},'desired_rotation',args{9}, ...
            'desired_angular_velocity_body_rad_s',args{10}, ...
            'desired_angular_acceleration_body_rad_s2',args{11});
        [rotor,d]=gpenmpcRobustSe3Control(args{1},ref,args{5},args{6},args{7},a.calibration,a.profile,att);
        expected61=[d.desired_thrust_n;d.desired_moment_nm;rotor;d.desired_force_raw_n; ...
            d.desired_force_projected_n;d.desired_rotation(:);d.raw_desired_rotation(:); ...
            d.attitude_error;d.desired_angular_velocity_body_rad_s; ...
            d.desired_angular_acceleration_body_rad_s2;d.desired_angular_velocity_in_current_body_rad_s; ...
            d.body_rate_error_rad_s;d.geometric_feedforward_moment_nm;d.raw_rotor_command_n; ...
            d.force_projection_norm_mismatch_n;d.tilt_rad;double(d.rotor_saturated)];
        [w,r,di,valid]=gpenmpcNative.se3WrenchKernel(args{:});actual61=[w;r;di];
        err=max(abs(actual61-expected61));
        assert(valid&&numel(expected61)==61&&err<1e-10,'gpenmpc:KernelParity','61-output parity failed.');
        cases{end+1}=struct('args',{args},'expected',expected61,'valid',true);
        oracle61(end+1,:)=expected61.';kernel61(end+1,:)=actual61.';
        row=struct('case_index',numel(cases),'kind',string(kind),'source_row',index, ...
            'leg_index',leg,'payload_kg',args{5},'continuity_enabled',logical(args{8}), ...
            'reset_requested',logical(reset),'desired_omega_norm',norm(args{10}), ...
            'desired_omega_dot_norm',norm(args{11}),'maximum_absolute_error_61',err);
        if isempty(rows),rows=row;else,rows(end+1)=row;end
    end
end

function checks=exerciseExistingRotorMapping(rotorUpper)
% Exercise the real existing 93 adapter with fixed offline wire values.
expected=struct('system_id',1,'component_id',1,'maximum_runtime_age_ns',1e8, ...
    'per_rotor_upper_n',rotorUpper,'software_from_px4_one_based',[5;1;4;6;2;3]);
mirror=struct('schema','GPENMPC_V7_NOMINAL_CONTROLLER_MIRROR_V1', ...
    'source','HOST_EXACT_V7_NOMINAL_PREPROJECTION_MIRROR','valid',true, ...
    'plant_truth_used',false,'sample_timestamp_ns',1e9,'dt_s',.01,'control_mode',1, ...
    'robust_acceleration_zero',true,'robust_acceleration_ned_mps2',zeros(3,1), ...
    'commanded_acceleration_ned_mps2',zeros(3,1),'desired_force_projected_up_n',[0;0;100], ...
    'force_projection_norm_mismatch_n',0, ...
    'controller_source_sha256','BE085D8302709AE1BD955522E7C9923AAFE90D4C764A9E07DBD7E549A9D51C44', ...
    'module_source_sha256','BA6A0A02F207790149B8AB0DC25C0197B0C39A000F05F6808E236F832DF9718E');
guard=struct('source','PX4_UORB_GPENMPC_SE3_CONTROL_STATUS','valid',true, ...
    'plant_truth_used',false,'rx_ns',1e9,'control_mode',1,'failure_reason',0, ...
    'active',true,'state_valid',true,'reference_fresh',true,'segment_fresh',true, ...
    'native_position_controller_disabled',true,'native_attitude_rate_allocator_enabled',true, ...
    'single_publisher_contract_pass',true);
act=struct('mavpackettype','HIL_ACTUATOR_CONTROLS','time_usec',1e6,'controls',zeros(16,1), ...
    'src_system',1,'src_component',1,'rx_ns',1e9,'generation',1);
checks=struct('name',{},'pass',{});
for sourceIndex=1:6
    act.controls=zeros(16,1);act.controls(sourceIndex)=.5;
    [f,ok,~]=gpenmpcNative.adaptV7NominalControlFeedback(mirror,guard,act,expected,1e9,1);
    wanted=zeros(6,1);wanted(expected.software_from_px4_one_based==sourceIndex)=.5*rotorUpper;
    checks(end+1)=struct('name',"REAL_93_ROTOR_PERMUTATION_"+sourceIndex, ...
        'pass',ok&&norm(f.rotor_command_n-wanted)<1e-12); %#ok<AGROW>
end
bad=expected;bad.software_from_px4_one_based=(1:6).';
[~,ok,~]=gpenmpcNative.adaptV7NominalControlFeedback(mirror,guard,act,bad,1e9,1);
checks(end+1)=struct('name','INCORRECT_ROTOR_PERMUTATION_REJECTED','pass',~ok);
end

function writeBinary(file,cases)
assert(numel(cases)<=10000,'Existing C driver bound exceeded.');
f=fopen(file,'wb','ieee-le');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,numel(cases),'uint32');
for k=1:numel(cases)
    args=cases{k}.args;for j=1:11,fwrite(f,double(args{j}(:)),'double');end
    p=args{12};names={'kp','kd','kr','kw','drag','inertia','pseudoinverse','baseMass','totalThrust','rotorUpper','maxTilt'};
    for j=1:numel(names),fwrite(f,double(p.(names{j})(:)),'double');end
    fwrite(f,[cases{k}.expected;double(cases{k}.valid)],'double');
end
end
function value=sha(file)
value=upper(string(gpenmpcNative.fileSha256(file)));
end
function writeText(file,value)
f=fopen(file,'w','n','UTF-8');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
