function result=run_task_io_host_tests(outputRoot)
deviceFixture=gpenmpc_test_device_config(); %#ok<NASGU>
% Test MAVLink serialization and source-method parity.
assert(~isfolder(outputRoot)&&~isfile(outputRoot),'gpenmpcTaskIo:FreshOutput','Output must be new.');
root=fileparts(fileparts(mfilename('fullpath')));
native=string(gpenmpc_external_path('native_visual_host_source'));
old=path;restore=onCleanup(@()path(old)); %#ok<NASGU>
addpath(native,'-begin');
addpath(fullfile(root,'host_runtime'),'-begin');
addpath(fullfile(root,'matlab_validation'),'-begin');
addpath(fullfile(root,'m600_coptersim','matlab_validation'),'-begin');
addpath(fullfile(root,'tools'),'-begin');
checks=struct('name',{},'pass',{});failure='';codec=[];oracle=[];
referenceSamplesExecuted=0;atomicSamplesExecuted=0;
guard=onCleanup(@closeOwned);raw=struct();
try
    provenance=gpenmpcTaskIo.taskIoProvenance();raw.provenance=provenance;
    receiptPath=fullfile(root,'task_packages','cambridge_canonical', ...
        'MU_CAMBRIDGE_MA_02__A1_COORDINATED_PHYSICAL__TASK_RECEIPT.json');
    [origin,task,binding]=gpenmpcTaskIo.loadCanonicalTaskOrigin(receiptPath);
    raw.origin=origin;raw.task_binding=binding;
    check('origin_from_canonical_depot_not_legacy_DV008', ...
        isequal(origin.origin_ned_m,[162.736;-221.648;0])&& ...
        ~isequal(origin.origin_ned_m,[-34.518;-75.710;0]));
    check('canonical_four_deliveries_eight_seconds',task.physical_service.delivery_count==4&&task.physical_service.ground_dwell_s==8);
    check('final_airborne_reference_requires_native_land',origin.final_native_landing_required&&origin.final_flight_reference_up_m(3)==10);
    target=struct('system_id',1,'component_id',1);dialect=mavlinkdialect('common.xml',2);
    codec=mavlinkio(dialect,'SystemID',1,'ComponentID',1);oracle=gpenmpcTaskIo.ReferenceEncoderOracle();
    expected=struct('uid',"1234605616436508552",'system_id',1,'component_id',1, ...
        'boot_generation',7,'maximum_age_ns',100000000,'maximum_clock_uncertainty_ns',1000000);
    clock=struct('valid',true,'board_to_host_offset_ns',-90000000000,'uncertainty_ns',1000);
    allReference=true;allAtomic=true;allX13=true;allWire=true;previous=struct();
    for k=1:41
        reference=struct('accepted',true,'publication_allowed',true,'plant_truth_used',false, ...
            'generation',k,'board_control_mode','RA_CTRL_MODE_NOMINAL_SINGLE_PUBLISHER', ...
            'reference_ned',struct('position_ned_m',origin.origin_ned_m+[k/10;-k/20;-1-k/50], ...
            'velocity_ned_mps',[k/100;-.2;.1],'acceleration_ned_mps2',[.01;-.03;k/1000], ...
            'jerk_ned_mps3',[.05;0;-.01]));
        [packet,record]=gpenmpcTaskIo.encodeA1Reference(reference,origin,target,uint32(10000+k*10),.02*k);
        oracle.sendSetpoint(double(packet.Payload.time_boot_ms), ...
            reference.reference_ned.position_ned_m-origin.origin_ned_m, ...
            reference.reference_ned.velocity_ned_mps,reference.reference_ned.acceleration_ned_mps2,.02*k);
        prior=oracle.Captured.Payload;
        % Compare algorithm fields and verify the explicit target 1/1 separately.
        for name={'x','y','z','vx','vy','vz','afx','afy','afz','yaw','yaw_rate'}
            allReference=allReference&&isequal(packet.Payload.(name{1}),single(prior.(name{1})));
        end
        allReference=allReference&&packet.Payload.time_boot_ms==prior.time_boot_ms&& ...
            packet.Payload.type_mask==prior.type_mask&&packet.Payload.coordinate_frame==prior.coordinate_frame&& ...
            packet.Payload.target_system==1&&packet.Payload.target_component==1&&~record.jerk_transmitted;
        wire=createmsg(dialect,'SET_POSITION_TARGET_LOCAL_NED');fields=fieldnames(packet.Payload);
        for j=1:numel(fields),wire.Payload.(fields{j})=packet.Payload.(fields{j});end
        [back,status]=deserializemsg(dialect,serializemsg(codec,wire),OutputAllMessage=true);
        allWire=allWire&&isscalar(back)&&status==0;
        for j=1:numel(fields),allWire=allWire&&isequal(back.Payload.(fields{j}),packet.Payload.(fields{j}));end
        m=odometryMessage(uint64(100000000+k*10000),k);
        [parsed,status]=deserializemsg(dialect,serializemsg(codec,m),OutputAllMessage=true);
        now=10000000000+k*10000000+500000;
        [sample,a,next]=gpenmpcTaskIo.decodeAtomicOdometry(parsed,now,now+1000,k,expected,origin,clock,previous);
        allAtomic=allAtomic&&status==0&&a.accepted&& ...
            sample.position_generation==sample.attitude_generation&&sample.position_generation==sample.rates_generation&& ...
            isa(sample.odometry_reset_counter_raw,'uint8')&&sample.odometry_reset_counter_raw==3;
        [x13,ok]=gpenmpcNative.px4EstimateState(sample,expected,now+1000);
        p=parsed.Payload;T=diag([1,1,-1]);
        expectedX=[T*(double([p.x;p.y;p.z])+origin.origin_ned_m);T*double([p.vx;p.vy;p.vz]); ...
            double(p.q(:))/norm(double(p.q(:))).*[1;-1;-1;1];double([p.rollspeed;p.pitchspeed;p.yawspeed]).*[-1;-1;1]];
        allX13=allX13&&ok&&max(abs(x13-expectedX))<1e-12;
        previous=next;
        referenceSamplesExecuted=referenceSamplesExecuted+1;
        atomicSamplesExecuted=atomicSamplesExecuted+1;
    end
    check('41_actual_source_sendSetpoint_PVA_yaw_parity',allReference);
    check('41_actual_common_xml_msg84_roundtrips',allWire);
    check('41_atomic_ODOMETRY_common_xml_roundtrips',allAtomic);
    check('41_existing_px4EstimateState_exact_state_parity',allX13);
    check('no_COM_in_oracle',~oracle.isOpen());
    raw.last_reference=record;raw.last_atomic_sample=sample;raw.last_atomic_audit=a;

    sourcePattern=parsed;
    sourcePattern.Payload.pose_covariance=nan(1,21,'single');
    sourcePattern.Payload.pose_covariance([1,7,12,16,19,21])=single(.1);
    sourcePattern.Payload.velocity_covariance=nan(1,21,'single');
    sourcePattern.Payload.velocity_covariance([1,7,12])=single(.1);
    [sourcePattern,sourceStatus]=deserializemsg(dialect,serializemsg(codec,sourcePattern),OutputAllMessage=true);
    [sourceSample,sourceAudit]=gpenmpcTaskIo.decodeAtomicOdometry(sourcePattern,now,now+1000,41,expected,origin,clock,struct());
    check('actual_PX4_diagonal_only_nan_pattern_accepted',sourceStatus==0&&sourceAudit.accepted);
    check('unpublished_cross_and_rate_variances_remain_unknown', ...
        nnz(isnan(sourceSample.pose_covariance_upper))==15&& ...
        nnz(isnan(sourceSample.velocity_covariance_upper))==18&& ...
        ~sourceAudit.pose_covariance_information.PSD_verified&& ...
        ~sourceAudit.velocity_covariance_information.complete_matrix_known);
    raw.source_covariance_pattern=sourceAudit;

    base=parsed;rx=now;current=now+1000;prev=struct();
    reject('LP31_is_not_atomic331','msgid');
    reject('wrong_component','component');
    reject('wrong_pose_frame','frame');
    reject('body_twist_cannot_be_labeled_NED','child_frame');
    reject('non_EKF_estimator','estimator');
    reject('unknown_covariance_NaN_not_zeroed','unknown_cov');
    reject('negative_covariance_diagonal','negative_cov');
    reject('undeclared_partial_covariance_pattern','partial_cov');
    reject('non_PSD_covariance','indefinite_cov');
    reject('nonfinite_state','nonfinite');
    reject('nonunit_quaternion','quaternion');
    reject('source_uint64_not_double','double_time');
    reject('stale_RX','stale_rx');
    reject('future_RX','future_rx');
    reject('invalid_clock','clock');
    reject('stale_source_despite_fresh_RX','stale_source');
    reject('future_source_despite_fresh_RX','future_source');
    reject('duplicate_source','duplicate');
    reject('reset_counter_change_no_fabricated_delta','reset');
    reject('rounded_numeric_UID_rejected','uid');
    reject('failed_quality_minus_one','quality');
    negativeReference=reference;negativeReference.plant_truth_used=true;
    check('truth_reference_rejected',throws(@()gpenmpcTaskIo.encodeA1Reference(negativeReference,origin,target,uint32(1),0)));
    negativeReference=reference;negativeReference.reference_ned.acceleration_ned_mps2(1)=Inf;
    check('nonfinite_reference_rejected',throws(@()gpenmpcTaskIo.encodeA1Reference(negativeReference,origin,target,uint32(1),0)));
    check('board_timestamp_double_not_silently_cast',throws(@()gpenmpcTaskIo.encodeA1Reference(reference,origin,target,1,0)));
    check('oracle_refuses_COM_open',throws(@()oracle.open('COM3',921600,1)));
catch problem
    failure=[problem.identifier ': ' problem.message];raw.failure_report=getReport(problem,'extended','hyperlinks','off');
end
closeOwned();clear guard
result=struct('classification','HOST_ONLY_TYPED_A1_REFERENCE_ATOMIC_ODOMETRY_AND_ORIGIN_TESTS', ...
    'pass',isempty(failure)&&all([checks.pass]),'checks_total',numel(checks), ...
    'checks_passed',nnz([checks.pass]),'checks',checks,'failure',failure, ...
    'reference_parity_samples',referenceSamplesExecuted,'atomic_parity_samples',atomicSamplesExecuted, ...
    'reference_parity_samples_planned',41,'atomic_parity_samples_planned',41, ...
    'COM_open',0,'UDP_open',0,'CopterSim_launch',0,'board_actions',0, ...
    'dynamics_validated',false,'live_origin_alignment_verified',false);
mkdir(outputRoot);save(fullfile(outputRoot,'TASK_IO_HOST_RAW.mat'),'raw','result');
fid=fopen(fullfile(outputRoot,'TASK_IO_HOST_RESULT.json'),'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear c
disp(result);assert(result.pass,'gpenmpcTaskIo:HostTestFailure','%s',failure);

    function check(name,pass),checks(end+1)=struct('name',name,'pass',logical(pass));assert(pass,'gpenmpcTaskIo:Check','%s',name);end
    function m=odometryMessage(time,k)
        m=createmsg(dialect,'ODOMETRY');m.Payload.time_usec=time;
        m.Payload.frame_id=uint8(1);m.Payload.child_frame_id=uint8(1);m.Payload.estimator_type=uint8(8);
        m.Payload.reset_counter=uint8(3);m.Payload.quality=int8(100);
        m.Payload.x=single(k/100);m.Payload.y=single(-k/200);m.Payload.z=single(-1);
        m.Payload.vx=single(.1);m.Payload.vy=single(-.2);m.Payload.vz=single(.05);
        axis=[1,2,3]/sqrt(14);angle=.005*k;
        m.Payload.q=single([cos(angle/2),sin(angle/2)*axis]);
        m.Payload.rollspeed=single(.01);m.Payload.pitchspeed=single(.02);m.Payload.yawspeed=single(-.03);
        covariance=zeros(1,21,'single');covariance([1,7,12,16,19,21])=single(.1);
        m.Payload.pose_covariance=covariance;m.Payload.velocity_covariance=covariance;
    end
    function reject(name,kind)
        m=base;e=expected;c=clock;r=rx;n=current;g=42;prior=prev;
        switch kind
            case 'msgid',m.MsgID=uint32(32);
            case 'component',m.ComponentID=uint8(2);
            case 'frame',m.Payload.frame_id=uint8(4);
            case 'child_frame',m.Payload.child_frame_id=uint8(12);
            case 'estimator',m.Payload.estimator_type=uint8(2);
            case 'unknown_cov',m.Payload.pose_covariance(1)=single(NaN);
            case 'negative_cov',m.Payload.pose_covariance(1)=single(-.1);
            case 'partial_cov',m.Payload.pose_covariance(2)=single(NaN);
            case 'indefinite_cov',m.Payload.pose_covariance(2)=single(2);
            case 'nonfinite',m.Payload.vx=single(NaN);
            case 'quaternion',m.Payload.q=single([2,0,0,0]);
            case 'double_time',m.Payload.time_usec=double(m.Payload.time_usec);
            case 'stale_rx',r=n-e.maximum_age_ns-1;
            case 'future_rx',r=n+1;
            case 'clock',c.valid=false;
            case 'stale_source',m.Payload.time_usec=m.Payload.time_usec-uint64(200000);
            case 'future_source',m.Payload.time_usec=m.Payload.time_usec+uint64(200000);
            case 'duplicate',prior=previous;g=42;
            case 'reset',prior=previous;m.Payload.time_usec=m.Payload.time_usec+uint64(10000);m.Payload.reset_counter=uint8(4);r=r+10000000;n=n+10000000;
            case 'uid',e.uid=3.473490377090611e18;
            case 'quality',m.Payload.quality=int8(-1);
        end
        [s,a,state]=gpenmpcTaskIo.decodeAtomicOdometry(m,r,n,g,e,origin,c,prior);
        check(name,~a.accepted&&isempty(s)&&isequaln(state,prior));
        raw.negative.(kind)=a;
    end
    function closeOwned()
        if ~isempty(oracle),try,delete(oracle);catch,end;oracle=[];end
        if ~isempty(codec),try,delete(codec);catch,end;codec=[];end
    end
end
function yes=throws(f)
yes=false;try,f();catch,yes=true;end
end
