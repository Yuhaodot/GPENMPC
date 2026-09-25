function report=test_canonical_actuator_association()
% Test actuator/reference temporal ordering.
build=string(fileparts(fileparts(mfilename('fullpath'))));
old=path;c=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
e=struct('uid','1234605616436508552','boot_generation',7, ...
    'system_id',1,'component_id',1,'maximum_runtime_age_ns',1e8);
a=struct('uid',e.uid,'boot_generation',7,'src_system',1,'src_component',1, ...
    'message_id',93,'generation',573,'source_generation',573, ...
    'time_usec',uint64(1900000),'source_time_ns',1900000000,'rx_ns',2e9, ...
    'controls',[ones(6,1)*.5;nan(10,1)]);
r=struct('host_send_begin_ns',1950000000,'host_send_return_ns',1951000000, ...
    'send_returned',true,'inner_generation',2,'boot_generation',7);
clock=struct('clock_valid',true,'sync_locked',true, ...
    'fixed_best_timesync',struct('offset_lower_s',.06,'offset_upper_s',.061));
checks=struct('name',{},'pass',{});
[v,ok,why]=gpenmpcNative.correlateActuatorAfterReference(a,r,clock,e,2.01e9);
check('source_interval_strictly_after_send',ok&&why=="TEMPORAL_ORDER_ONLY__NO_REFERENCE_ECHO");
check('raw_source_generation_not_inner_generation',v.actuator_source_generation==573 ...
    &&v.inner_association_generation==2&&v.original_actuator.generation==573);
check('full_packet_and_nan_sentinels_bit_semantics_retained',isequaln(v.original_actuator,a));
check('no_fabricated_board_reference_or_sample_ack',~v.reference_consumption_proven ...
    &&~v.same_estimate_sample_proven&&~v.raw_generation_rewritten);
bad=a;bad.source_time_ns=1800000000;bad.time_usec=uint64(1800000);
negative('late_receive_of_pre_reference_packet_rejected',bad,r,clock,'SOURCE_TIME_NOT_PROVEN_AFTER_REFERENCE_SEND');
bad=clock;bad.fixed_best_timesync.offset_lower_s=.051;
negative('overlapping_clock_interval_rejected',a,r,bad,'SOURCE_TIME_NOT_PROVEN_AFTER_REFERENCE_SEND');
bad=clock;bad.clock_valid=false;negative('invalid_clock_rejected',a,r,bad,'SOURCE_CLOCK_MAPPING_NOT_VALID');
bad=clock;bad.fixed_best_timesync.offset_upper_s=.2;
negative('source_interval_after_receive_rejected',a,r,bad,'SOURCE_TIME_INTERVAL_NOT_BEFORE_RECEIVE');
bad=a;bad.generation=2;negative('restamped_packet_generation_rejected',bad,r,clock,'ACTUATOR_TIMESTAMP_OR_ORIGINAL_GENERATION_INVALID');
bad=a;bad.rx_ns=1.8e9;negative('old_receive_cannot_be_renewed',bad,r,clock,'ACTUATOR_TIMESTAMP_OR_ORIGINAL_GENERATION_INVALID');
bad=a;bad.src_system=2;negative('wrong_source_rejected',bad,r,clock,'ACTUATOR_SOURCE_OR_BOOT_MISMATCH');
bad=a;bad.boot_generation=8;negative('changed_boot_rejected',bad,r,clock,'ACTUATOR_SOURCE_OR_BOOT_MISMATCH');
bad=r;bad.send_returned=false;negative('unsent_reference_rejected',a,bad,clock,'REFERENCE_SEND_NOT_CONFIRMED');
report=struct('status','PASS_HOST_ONLY_TEMPORAL_ASSOCIATION', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'hardware_actions',0,'board_reference_ack_proven',false);
disp(jsonencode(report));
    function negative(name,x,y,z,reason)
        [out,yes,why]=gpenmpcNative.correlateActuatorAfterReference(x,y,z,e,2.01e9);
        check(name,~yes&&isempty(fieldnames(out))&&why==string(reason));
    end
    function check(name,yes)
        checks(end+1)=struct('name',name,'pass',logical(yes)); %#ok<AGROW>
        assert(yes,'gpenmpcNative:ActuatorAssociationTest','%s',name);
    end
end
