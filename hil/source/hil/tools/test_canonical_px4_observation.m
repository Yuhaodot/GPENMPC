function report=test_canonical_px4_observation(outputRoot)
% Test observation reduction with UAV Toolbox encoding and decoding.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
outputRoot=string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
assert(startsWith(lower(outputRoot),lower(build+"\evidence\"))&&~isfolder(outputRoot), ...
    'Fixtures require a fresh BUILD/evidence subdirectory.');
mkdir(outputRoot);oldPath=path;guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
dialect=mavlinkdialect('common.xml',2);
encoder=mavlinkio(dialect,'SystemID',231,'ComponentID',77);
encoderGuard=onCleanup(@()delete(encoder)); %#ok<NASGU>
wrongEncoder=mavlinkio(dialect,'SystemID',232,'ComponentID',78);
wrongGuard=onCleanup(@()delete(wrongEncoder)); %#ok<NASGU>
checks=struct('name',{},'pass',{});wire=struct();
binding=struct('uid','1234605616436508552','system_id',231,'component_id',77, ...
    'boot_generation',7,'verified',true);
initial=struct('verified_binding',binding,'maximum_queue',4, ...
    'maximum_age_ns',100e6,'now_ns',1e9);
[empty,r]=m600check.advanceCanonicalPx4Observation([],'INIT',initial);
check('initial_absent_streams_not_zeros',isempty(r.snapshot.odometry)&&isempty(r.snapshot.actuator)&& ...
    ~r.snapshot.odometry_present&&r.snapshot.odometry_generation==0);
message=createmsg(dialect,'ODOMETRY');p=message.Payload;
p.time_usec=uint64(2100000);p.frame_id=uint8(1);p.child_frame_id=uint8(1);
p.x=single(1.25);p.y=single(-2.5);p.z=single(-3.75);
p.q=single([cos(.2),0,0,sin(.2)]);p.vx=single(.1);p.vy=single(-.2);p.vz=single(.3);
p.rollspeed=single(.01);p.pitchspeed=single(-.02);p.yawspeed=single(.03);
p.pose_covariance=NaN(1,21,'single');p.velocity_covariance=NaN(1,21,'single');
p.reset_counter=uint8(4);p.estimator_type=uint8(8);message.Payload=p;
wire.odometry=serializemsg(encoder,message);
[decoded,status]=deserializemsg(dialect,wire.odometry,OutputAllMessages=true);
check('actual_matlab_decoded_source_fields',isscalar(decoded)&&status==0&& ...
    all(isfield(decoded,{'MsgID','SystemID','ComponentID','Seq','Payload'}))&& ...
    decoded.MsgID==331&&decoded.SystemID==231&&decoded.ComponentID==77);
[s,first]=accept(empty,decoded,1.001e9);
check('actual_wire_atomic_fields_and_binding',first.accepted&& ...
    strcmp(first.sample.uid,binding.uid)&&first.sample.boot_generation==7&& ...
    first.sample.system_id==double(decoded.SystemID)&&first.sample.component_id==double(decoded.ComponentID)&& ...
    first.sample.wire_sequence==double(decoded.Seq)&&first.sample.message_id==331&& ...
    first.sample.source_time_ns==double(decoded.Payload.time_usec)*1000&&first.sample.rx_ns==1.001e9);
check('untranslated_local_ned_values',isequal(first.sample.position_ned_m,double([p.x;p.y;p.z]))&& ...
    ~first.sample.task_origin_added&&strcmp(first.sample.coordinate_origin,'PX4_LOCAL_NED_UNTRANSLATED'));
check('one_message_atomic_estimate_not_mixed_streams',first.sample.atomic_estimate&& ...
    first.sample.position_generation==1&&first.sample.attitude_generation==1&& ...
    first.sample.rates_generation==1&&first.sample.position_rx_ns==first.sample.rates_rx_ns&& ...
    isequal(first.sample.quaternion_wxyz_body_to_ned,double(p.q(:)))&& ...
    isequal(first.sample.omega_frd_rad_s,double([p.rollspeed;p.pitchspeed;p.yawspeed])));
check('optional_covariance_nan_not_invented_zero',all(isnan(first.sample.pose_covariance)));

act=createmsg(dialect,'HIL_ACTUATOR_CONTROLS');act.Payload.time_usec=uint64(2105000);
act.Payload.controls=single([.1,.2,.3,.4,.5,.6,NaN(1,10)]);
act.Payload.mode=uint8(128);act.Payload.flags=uint64(0);
wire.actuator=serializemsg(encoder,act);ad=deserializemsg(dialect,wire.actuator);
[s,ar]=accept(s,ad,1.002e9);
check('hil93_real_wire_six_plus_ten_nan',ar.accepted&&ar.sample.message_id==93&& ...
    isequal(ar.sample.controls(1:6),double(act.Payload.controls(1:6).'))&& ...
    all(isnan(ar.sample.controls(7:16)))&&ar.sample.src_system==231&&ar.sample.src_component==77);
check('independent_source_generations',s.odometry.generation==1&&s.actuator.generation==1);
[s,dup]=accept(s,decoded,1.003e9);
check('duplicate_no_generation_no_freshness_credit',dup.duplicate&&~dup.accepted&& ...
    s.odometry.generation==1&&s.odometry.last_rx_ns==1.001e9&&numel(s.odometry.queue)==1);
next=decoded;next.Payload.time_usec=uint64(2137500);
[s,nr]=accept(s,next,1.020e9);
check('irregular37p5ms_gap_recorded_not_ten_ms_rejected',nr.accepted&& ...
    nr.sample.source_gap_ns==37.5e6&&nr.sample.receive_gap_ns==19e6&&nr.sample.generation==2);
before=s;[s,snap]=m600check.advanceCanonicalPx4Observation(s,'SNAPSHOT',struct('now_ns',1.030e9));
check('snapshot_no_new_generation',snap.snapshot.odometry_fresh&& ...
    s.odometry.generation==before.odometry.generation&&numel(s.odometry.queue)==2);
[s,drain]=m600check.advanceCanonicalPx4Observation(s,'DRAIN',struct('now_ns',1.030e9));
check('drain_preserves_original_source_and_receive_time',numel(drain.odometry_queue)==2&& ...
    numel(drain.actuator_queue)==1&&drain.odometry_queue{1}.source_time_ns==2.1e9&& ...
    drain.odometry_queue{1}.rx_ns==1.001e9&&isempty(s.odometry.queue));
[s,again]=m600check.advanceCanonicalPx4Observation(s,'DRAIN',struct('now_ns',1.040e9));
check('empty_drain_no_sample_backfill',isempty(again.odometry_queue)&&isempty(again.actuator_queue)&& ...
    s.odometry.generation==2&&s.actuator.generation==1);
[stale,~]=accept(empty,decoded,1.001e9);
[stale,staleSnapshot]=m600check.advanceCanonicalPx4Observation(stale,'SNAPSHOT',struct('now_ns',1.102e9));
check('stale_snapshot_empty_but_provenance_count_visible',isempty(staleSnapshot.snapshot.odometry)&& ...
    staleSnapshot.snapshot.odometry_present&&~staleSnapshot.snapshot.odometry_fresh&& ...
    staleSnapshot.snapshot.odometry_generation==1&&staleSnapshot.snapshot.odometry_age_ns==101e6&& ...
    staleSnapshot.snapshot.odometry_queued_expired_count==1);
[stale,expired]=m600check.advanceCanonicalPx4Observation(stale,'DRAIN',struct('now_ns',1.102e9));
check('expired_queue_counted_never_returned',isempty(expired.odometry_queue)&& ...
    expired.odometry_expired_count==1&&expired.snapshot.odometry_expired_total==1&& ...
    stale.odometry.generation==1);

bad=deserializemsg(dialect,serializemsg(wrongEncoder,message));
failure('actual_wire_wrong_source_rejected',empty,bad,'MESSAGE_SOURCE_IDENTITY_MISMATCH');
bad=decoded;bad=rmfield(bad,'ComponentID');
failure('missing_source_identity_not_guessed',empty,bad,'DECODED_SOURCE_OR_PAYLOAD_INVALID');
bad=decoded;bad.Payload.x=single(NaN);
failure('nan_position_rejected',empty,bad,'ODOMETRY_STATE_NONFINITE_OR_SHAPE');
bad=decoded;bad.Payload.vx=single([0,1]);
failure('state_array_shape_rejected',empty,bad,'ODOMETRY_STATE_NONFINITE_OR_SHAPE');
bad=decoded;bad.Payload.q=single([1,0,0]);
failure('quaternion_wrong_length_rejected',empty,bad,'ODOMETRY_QUATERNION_INVALID');
bad=decoded;bad.Payload.q=single([2,0,0,0]);
failure('quaternion_nonunit_rejected',empty,bad,'ODOMETRY_QUATERNION_INVALID');
bad=decoded;bad.Payload.rollspeed=single(Inf);
failure('nonfinite_body_rate_rejected',empty,bad,'ODOMETRY_STATE_NONFINITE_OR_SHAPE');
bad=decoded;bad.Payload.frame_id=uint8(20);
failure('non_ned_pose_rejected',empty,bad,'ODOMETRY_FRAME_ESTIMATOR_OR_RESET_INVALID');
bad=decoded;bad.Payload.child_frame_id=uint8(12);
failure('non_ned_velocity_rejected',empty,bad,'ODOMETRY_FRAME_ESTIMATOR_OR_RESET_INVALID');
bad=decoded;bad.Payload.estimator_type=uint8(2);
failure('non_autopilot_estimator_rejected',empty,bad,'ODOMETRY_FRAME_ESTIMATOR_OR_RESET_INVALID');
bad=decoded;bad.Payload.pose_covariance=single(zeros(1,20));
failure('covariance_array_shape_rejected',empty,bad,'ODOMETRY_COVARIANCE_SHAPE_OR_INFINITY');
[base,~]=accept(empty,decoded,1.001e9);
bad=next;bad.Payload.reset_counter=uint8(5);
failure('reset_counter_change_explicit_failure',base,bad,'ODOMETRY_RESET_COUNTER_CHANGED');
bad=decoded;bad.Payload.time_usec=uint64(2099999);
failure('source_timestamp_reversed_rejected',base,bad,'SOURCE_TIMESTAMP_REVERSED');
bad=decoded;bad.Payload.z=single(-4);
failure('same_timestamp_changed_payload_rejected',base,bad,'DUPLICATE_TIMESTAMP_PAYLOAD_CHANGED');
bad=decoded;bad.Payload.time_usec=uint64(0);
failure('missing_source_timestamp_rejected',empty,bad,'SOURCE_TIMESTAMP_INVALID_OR_PRECISION_UNSAFE');
bad=ad;bad.Payload.controls(1)=NaN;
failure('required_actuator_channel_nan_rejected',empty,bad,'HIL_ACTUATOR_REQUIRED_CHANNELS_OR_PAYLOAD_INVALID');
bad=ad;bad.Payload.controls=single(zeros(1,6));
failure('six_only_array_cannot_invent_ten_channels',empty,bad,'HIL_ACTUATOR_REQUIRED_CHANNELS_OR_PAYLOAD_INVALID');
bad=ad;bad.Payload.controls(8)=Inf;
failure('unused_channel_infinity_not_nan_rejected',empty,bad,'HIL_ACTUATOR_REQUIRED_CHANNELS_OR_PAYLOAD_INVALID');
[ab,~]=accept(empty,ad,1.001e9);bad=ad;bad.Payload.time_usec=uint64(2104999);
failure('actuator_source_timestamp_reversed',ab,bad,'SOURCE_TIMESTAMP_REVERSED');
[broken,br]=accept(base,next,1e9);
check('host_receive_reversal_latches',broken.fatal_latched&&strcmp(br.reason,'HOST_TIMESTAMP_REVERSED')&& ...
    br.snapshot.first_failure.event_ns==1e9);
[broken,later]=accept(broken,next,1.030e9);
check('fatal_cannot_heal_on_good_packet',~later.accepted&&isempty(later.sample)&& ...
    broken.odometry.generation==1&&strcmp(later.snapshot.first_failure.reason,'HOST_TIMESTAMP_REVERSED'));
[broken,bd]=m600check.advanceCanonicalPx4Observation(broken,'DRAIN',struct('now_ns',1.031e9));
check('fatal_drain_no_usable_samples_counts_visible',isempty(bd.odometry_queue)&& ...
    bd.snapshot.failure_count==1&&bd.snapshot.odometry_queue_count==1&&~bd.snapshot.odometry_fresh);
tiny=initial;tiny.maximum_queue=1;[tinyState,~]=m600check.advanceCanonicalPx4Observation([],'INIT',tiny);
[tinyState,~]=accept(tinyState,decoded,1.001e9);
[tinyState,full]=accept(tinyState,next,1.002e9);
check('queue_overflow_no_silent_trim',tinyState.fatal_latched&& ...
    strcmp(full.reason,'BOUNDED_QUEUE_OVERFLOW_NO_OVERWRITE')&& ...
    numel(tinyState.odometry.queue)==1&&tinyState.odometry.generation==1);
[~,reinit]=m600check.advanceCanonicalPx4Observation(base,'INIT',initial);
check('same_owner_reinitialization_not_allowed',reinit.fatal_latched&&strcmp(reinit.reason,'OWNER_REINITIALIZATION_FORBIDDEN'));
unsafe=initial;unsafe.verified_binding.uid=double(bitor(bitshift(uint64(hex2dec('11223344')),32),uint64(hex2dec('55667788'))));
check('lossy_double_uid_rejected',rejects(@()m600check.advanceCanonicalPx4Observation([],'INIT',unsafe), ...
    'm600check:CanonicalObservationBinding'));
unverified=initial;unverified.verified_binding.verified=false;
check('unverified_binding_rejected',rejects(@()m600check.advanceCanonicalPx4Observation([],'INIT',unverified), ...
    'm600check:CanonicalObservationBinding'));
uintBinding=initial;uintBinding.verified_binding.uid=bitor(bitshift(uint64(hex2dec('11223344')),32),uint64(hex2dec('55667788')));
[uintState,~]=m600check.advanceCanonicalPx4Observation([],'INIT',uintBinding);
check('uint64_uid_remains_lossless',strcmp(uintState.binding.uid,'1234605616436508552'));
local=createmsg(dialect,'LOCAL_POSITION_NED');ld=deserializemsg(dialect,serializemsg(encoder,local));
[ignored,ir]=accept(empty,ld,1.001e9);
check('local_position_never_fills_atomic_odometry',~ir.accepted&&ignored.ignored_message_count==1&& ...
    ignored.odometry.generation==0&&isempty(ir.snapshot.odometry));
report=struct('schema','CANONICAL_PX4_ATOMIC_OBSERVER_HOST_TEST_V1','pass',all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',nnz([checks.pass]),'checks',checks, ...
    'actual_matlab_message_fields',{fieldnames(decoded)}, ...
    'odometry_wire_bytes',numel(wire.odometry),'actuator_wire_bytes',numel(wire.actuator), ...
    'source_path',which('m600check.advanceCanonicalPx4Observation'), ...
    'source_sha256',m600check.fileSha256(which('m600check.advanceCanonicalPx4Observation')), ...
    'test_sha256',m600check.fileSha256(mfilename('fullpath')+".m"), ...
    'serial_open',0,'UDP_open',0,'model_runs',0,'board_actions',0, ...
    'claim','MATLAB codec and synthetic observer tests.');
save(fullfile(outputRoot,'CODEC_RAW.mat'),'wire','decoded','ad','first','ar','report');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
fileGuard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fileGuard
fprintf('CANONICAL_PX4_OBSERVATION %d/%d pass=%d\n',report.checks_passed,report.checks_total,report.pass);
assert(report.pass,'m600check:CanonicalObservationTest','See result.');
    function check(name,passed)
        checks(end+1)=struct('name',name,'pass',logical(passed)); %#ok<AGROW>
    end
    function failure(name,st,msg,reason)
        [observed,receipt]=accept(st,msg,1.020e9);
        check(name,observed.fatal_latched&&~receipt.accepted&&isempty(receipt.sample)&&strcmp(receipt.reason,reason));
    end
end
function [s,r]=accept(s,m,rx)
[s,r]=m600check.advanceCanonicalPx4Observation(s,'MESSAGE',struct('message',m,'rx_ns',rx));
end
function yes=rejects(f,id)
yes=false;try,f();catch ex,yes=strcmp(ex.identifier,id);end
end
