function result=run_imu_installation_frame_host_tests(outputDir)
% Test IMU-frame numerics and replay static observations.
assert(~isfolder(outputDir)&&~isfile(outputDir));mkdir(outputDir);
b=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(b,'matlab_validation'),fullfile(b,'m600_coptersim','matlab_validation'));
p=struct('SENS_BOARD_ROT',0,'SENS_BOARD_X_OFF',6.445373058319092, ...
    'SENS_BOARD_Y_OFF',-6.442468166351318,'SENS_BOARD_Z_OFF',0);
c=m600check.buildBoardImuInstallationRotation(p);L=c.sensor_to_body;
names={};checks=[];g=9.80665;f=[0;0;-g];omega=[0;0;0];
observed=L*f;
predicted=[atan2(-observed(2),-observed(3));asin(observed(1)/g)]*180/pi;
check('level_roll_prediction_matches_observed_approximate_sign',abs(predicted(1)+6.486)<.001);
check('level_pitch_prediction_matches_observed_approximate_sign',abs(predicted(2)-6.402)<.001);
check('proper_rotation_orthogonality',norm(L*L.'-eye(3),'fro')<1e-14&&abs(det(L)-1)<1e-14);
check('independent_quaternion_oracle',norm(L-quatOracle(c.angles_degrees*pi/180),'fro')<1e-14);
r=m600check.applyImuInstallationInverse(f,omega,c);
check('level_specific_force_restored',norm(L*r.raw_specific_force-f)<1e-13);
check('level_gyro_zero_preserved',all(r.raw_angular_rate==0));
check('specific_force_not_kinematic_gravity_mutation',norm(r.raw_specific_force)>9.8&&~r.plant_state_modified);
for axis=1:3
    for signValue=[-1,1]
        v=zeros(3,1);v(axis)=signValue;r=m600check.applyImuInstallationInverse(g*v,7*v,c);
        check(sprintf('signed_axis_%d_%+d_accel_gyro',axis,signValue), ...
            norm(L*r.raw_specific_force-g*v)<1e-13&&norm(L*r.raw_angular_rate-7*v)<1e-13);
    end
end
rs=RandStream('mt19937ar','Seed',250905);n=256;
bodyF=zeros(3,n);bodyW=randn(rs,3,n);attitudes=(rand(rs,3,n)-.5)*2.4;
for k=1:n,bodyF(:,k)=quatOracle(attitudes(:,k)).'*[0;0;-g]+randn(rs,3,1);end
savedPhysical=struct('position',randn(rs,3,n),'velocity',randn(rs,3,n), ...
    'attitude',attitudes,'specific_force',bodyF,'gyro',bodyW);before=savedPhysical;
r=m600check.applyImuInstallationInverse(bodyF,bodyW,c);
errorF=max(abs(L*r.raw_specific_force-bodyF),[],'all');errorW=max(abs(L*r.raw_angular_rate-bodyW),[],'all');
check('256_nonzero_attitudes_accelerometer_equivalence',errorF<1e-12);
check('256_arbitrary_gyro_vectors_equivalence',errorW<1e-12);
check('norm_preserved_all_accelerometer_samples',max(abs(vecnorm(r.raw_specific_force)-vecnorm(bodyF)))<1e-12);
check('norm_preserved_all_gyro_samples',max(abs(vecnorm(r.raw_angular_rate)-vecnorm(bodyW)))<1e-12);
check('plant_truth_inputs_bitwise_unchanged',isequaln(before,savedPhysical));
check('no_magnetometer_claim_from_imu_helper',~r.magnetometer_handled&&~c.magnetometer_handled);
wrong=L*(L*bodyF);check('wrong_direction_negative_control_detectable',max(abs(wrong-bodyF),[],'all')>1);
uncorrected=L*bodyF;check('omitted_inverse_negative_control_detectable',max(abs(uncorrected-bodyF),[],'all')>.5);
% Allow float32 rounding error in native PX4 matrix arithmetic.
Lf=single(L);floatError=max(abs(double(Lf*single(r.raw_specific_force))-bodyF),[],'all');
check('float32_rotation_arithmetic_equivalence',floatError<1e-5);
p0=p;p0.SENS_BOARD_X_OFF=0;p0.SENS_BOARD_Y_OFF=0;
c0=m600check.buildBoardImuInstallationRotation(p0);r0=m600check.applyImuInstallationInverse(bodyF,bodyW,c0);
check('zero_installation_exact_identity',isequal(c0.sensor_to_body,eye(3))&&isequal(r0.raw_specific_force,bodyF)&&isequal(r0.raw_angular_rate,bodyW));
bad=rmfield(p,'SENS_BOARD_X_OFF');negative(@()m600check.buildBoardImuInstallationRotation(bad),'m600check:InstallationIdentityMissing');
bad=p;bad.SENS_BOARD_Y_OFF=NaN;negative(@()m600check.buildBoardImuInstallationRotation(bad),'m600check:InstallationIdentityNonfinite');
bad=p;bad.SENS_BOARD_Z_OFF=Inf;negative(@()m600check.buildBoardImuInstallationRotation(bad),'m600check:InstallationIdentityNonfinite');
bad=p;bad.SENS_BOARD_ROT=1;negative(@()m600check.buildBoardImuInstallationRotation(bad),'m600check:InstallationRotationUnsupported');
bad=rmfield(c,'parameters');negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationIdentityMissing');
bad=c;bad.schema='UNKNOWN';negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationSchema');
bad=c;bad.sensor_to_body(1,1)=NaN;negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationMatrixNonfinite');
bad=c;bad.sensor_to_body=2*eye(3);negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationMatrixNotRotation');
bad=c;bad.sensor_to_body=diag([-1,1,1]);negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationMatrixNotRotation');
bad=c;bad.sensor_to_body=eye(3);negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationMatrixIdentityMismatch');
bad=c;bad.body_to_sensor=L;negative(@()m600check.applyImuInstallationInverse(f,omega,bad),'m600check:InstallationMatrixIdentityMismatch');
negative(@()m600check.applyImuInstallationInverse([1;2;NaN],omega,c),'m600check:ImuVectorInvalid');
negative(@()m600check.applyImuInstallationInverse(f,[0,0,0],c),'m600check:ImuVectorInvalid');
negative(@()m600check.applyImuInstallationInverse([f,f],omega,c),'m600check:ImuVectorShape');
o=m600check.compareIndependentAttitude(attitudes,attitudes,1:n,1:n,.25);
check('nonzero_attitude_self_comparison',max(o.attitude_geodesic_error_rad)<1e-14&&max(o.tilt_error_rad)<1e-14);
model=zeros(3,1);est=[predicted*pi/180;0];o=m600check.compareIndependentAttitude(est,model,1,1,.25);
check('independent_attitude_detects_installation_error',o.tilt_error_rad>.15&&~o.performance_threshold_defined&&~o.flight_admission_decided);
o=m600check.compareIndependentAttitude([0;0;-pi+.01],[0;0;pi-.01],1,1,.25);
check('yaw_wrap_not_360_degree_error',abs(abs(o.wrapped_euler_error_rad(3))-.02)<1e-12);
o=m600check.compareIndependentAttitude(zeros(3,1),zeros(3,1),1,1.26,.25);
check('existing_caller_time_bound_respected',~o.pair_within_caller_time_bound);
negative(@()m600check.compareIndependentAttitude([0;NaN;0],zeros(3,1),1,1,.25),'m600check:AttitudeObservationShape');
negative(@()m600check.compareIndependentAttitude(zeros(3,2),zeros(3,2),[2,1],[1,2],.25),'m600check:AttitudeObservationTime');
rawPath=gpenmpc_external_path('initialization_observation_raw');
actual=readCurrentAttitude(rawPath,outputDir);
check('actual_static_px4_attitude_observed',actual.attitude_message_count>0);
check('actual_independent_model_attitude_observed',actual.model_truth_count>0);
check('actual_model_remains_level',max(abs(actual.model_euler_max_abs_deg(1:2)))<1e-8);
% Compare with the documented six-degree observation for diagnostics.
result=struct('status','HOST_ONLY_IMU_INSTALLATION_ROTATION_PROOF','passed',all(checks), ...
    'checks_total',numel(checks),'checks_passed',sum(checks), ...
    'checks',struct('name',names,'passed',num2cell(checks)), ...
    'installation',c,'level_uncorrected_specific_force',observed,'predicted_level_roll_pitch_deg',predicted, ...
    'max_double_reconstruction_error_accel',errorF,'max_double_reconstruction_error_gyro',errorW, ...
    'max_float32_reconstruction_error_accel',floatError,'nonzero_attitude_sample_count',n, ...
    'current_readonly_observation',actual,'plant_truth_modified',false, ...
    'new_flight_threshold_defined',false,'COM_open',0,'UDP_open',0,'board_actions',0,'simulator_started',0, ...
    'integration_limit','Inverse belongs on final sensor specific-force/gyro fields; offsets/scales/thermal and any magnetometer path need separate source/parameter proof.', ...
    'next_hover_observation_proposal', ...
    ['Before mapping or arm, independently retain fresh PX4 ATTITUDE and model truth quaternion/Euler with raw source timestamps, ' ...
     'source-clock mapping and existing 0.25 s alignment bound. Report wrapped Euler, geodesic and tilt differences without rotating either state. ' ...
     'Compare the recorded six-degree observation with the stationary observation. Flight-admission tolerance requires ' ...
     'stationary noise, sensor/estimator uncertainty and model-serialization measurements.']);
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear cleanup
save(fullfile(outputDir,'NUMERICAL_PROOF.mat'),'result','bodyF','bodyW','attitudes','-v7');
disp(jsonencode(result));assert(result.passed,'m600check:InstallationHostTestsFailed','Frame tests failed.');
    function check(name,value),names{end+1}=name;checks(end+1)=logical(value);end
    function negative(fn,expected)
        passed=false;try,fn();catch e,passed=strcmp(e.identifier,expected);end
        check(sprintf('negative_%02d_%s',numel(names)+1,expected),passed);
    end
end
function R=quatOracle(xyz)
% Use independent quaternion composition.
h=reshape(double(xyz),1,3)/2;cr=cos(h(1));sr=sin(h(1));cp=cos(h(2));sp=sin(h(2));cy=cos(h(3));sy=sin(h(3));
q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
w=q(1);x=q(2);y=q(3);z=q(4);
R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w);2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w);2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
end
function summary=readCurrentAttitude(path,outputDir)
data=load(path,'result');e=data.result.transport_evidence;ar=[];tr=[];
for k=1:numel(e.raw_mavlink)
    v=e.raw_mavlink{k};if ~strcmp(v.topic,'ATTITUDE'),continue;end
    p=v.message.Payload;ar(end+1,:)=[v.rx_s,double(p.time_boot_ms)/1000,double(p.roll),double(p.pitch),double(p.yaw)]; %#ok<AGROW>
end
for k=1:numel(e.raw_truth_datagrams)
    v=e.raw_truth_datagrams{k};if numel(v.bytes)~=168,continue;end
    d=m600check.decodeTruthPacket(v.bytes,struct('expected_copter_id',1,'require_checksum',true));
    if d.valid,tr(end+1,:)=[v.rx_s,d.time_s,d.euler_rad];end %#ok<AGROW>
end
assert(~isempty(ar)&&~isempty(tr),'m600check:FrameRawMissing','Independent raw attitude streams missing.');
at=array2table(ar,'VariableNames',{'received_s','source_boot_s','roll_rad','pitch_rad','yaw_rad'});
tt=array2table(tr,'VariableNames',{'received_s','source_sim_s','roll_rad','pitch_rad','yaw_rad'});
writetable(at,fullfile(outputDir,'PX4_RAW_ATTITUDE.csv'));writetable(tt,fullfile(outputDir,'MODEL_RAW_ATTITUDE.csv'));
summary=struct('source',path,'attitude_message_count',size(ar,1),'attitude_unique_source_count',numel(unique(ar(:,2))), ...
    'model_truth_count',size(tr,1),'model_unique_source_count',numel(unique(tr(:,2))), ...
    'px4_euler_mean_deg',mean(ar(:,3:5),1)*180/pi,'px4_euler_median_deg',median(ar(:,3:5),1)*180/pi, ...
    'px4_euler_min_deg',min(ar(:,3:5),[],1)*180/pi,'px4_euler_max_deg',max(ar(:,3:5),[],1)*180/pi, ...
    'model_euler_max_abs_deg',max(abs(tr(:,3:5)),[],1)*180/pi, ...
    'source_clock_fit_performed',false, ...
    'scope','Independent stationary attitude distributions.');
end
