function report=test_canonical_se3_status_observation(outputRoot)
% Test status request/reply handling with the SERIAL_CONTROL codec.
arguments,outputRoot (1,1) string,end
build=string(fileparts(fileparts(mfilename('fullpath'))));
outputRoot=string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
assert(startsWith(lower(outputRoot),lower(build+"\evidence\"))&&~isfolder(outputRoot));
mkdir(outputRoot);oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
dialect=mavlinkdialect('common.xml',2);
encoder=mavlinkio(dialect,'SystemID',231,'ComponentID',77);g=onCleanup(@()delete(encoder)); %#ok<NASGU>
wrong=mavlinkio(dialect,'SystemID',232,'ComponentID',78);wg=onCleanup(@()delete(wrong)); %#ok<NASGU>
checks=struct('name',{},'pass',{});raw=struct();
init=struct('verified_binding',struct('uid','1234605616436508552', ...
    'system_id',231,'component_id',77,'boot_generation',7,'verified',true),'now_ns',1e9);
[s,~]=gpenmpcNative.advanceSe3StatusObservation([],'INIT',init);
base=s;
[s,tx]=step(s,'TICK',1e9);
check('fixed_readonly_message_only_prepared_not_sent',tx.query_prepared&&tx.tx_message.MsgID==126&& ...
    tx.tx_message.Payload.device==10&&tx.tx_message.Payload.flags==6&& ...
    tx.tx_message.Payload.timeout==0&&tx.tx_message.Payload.baudrate==0&&s.send_attempts==0&&s.queries_sent==0&& ...
    strcmp(char(tx.tx_message.Payload.data(1:double(tx.tx_message.Payload.count))), ...
      [newline,'listener gpenmpc_se3_control_status 0 1',newline]));
wire=serializemsg(encoder,tx.tx_message);decoded=deserializemsg(dialect,wire);
check('actual_matlab_serial_control_codec',decoded.MsgID==126&&decoded.SystemID==231&& ...
    decoded.ComponentID==77&&isequal(decoded.Payload,tx.tx_message.Payload));
[s,pending]=step(s,'TICK',1.001e9);
check('only_one_pending_request',~pending.query_prepared&&isempty(pending.tx_message)&&s.queries_prepared==1);
[s,~]=sendReceipt(s,1.002e9,true);
check('send_attempt_and_return_count_distinct',s.send_attempts==1&&s.queries_sent==1&&s.tx_confirmed);
text=fixture(1000000,10,9);[s,positive]=feed(s,text,1.020e9,encoder);
raw.positive=positive;
if ~positive.completed||~isstruct(positive.atom)
    save(fullfile(outputRoot,'FIRST_FAILURE.mat'),'s','positive','text','wire','decoded');
    error('gpenmpcNative:Se3ObserverFirstPositive','First complete reply rejected: %s',positive.reason);
end
check('complete_chunked_reply_actual_source_binding',positive.completed&&~positive.fatal_latched&& ...
    positive.atom.system_id==231&&positive.atom.component_id==77&& ...
    strcmp(positive.atom.uid,'1234605616436508552')&&positive.atom.boot_generation==7&& ...
    s.responses_completed==1&&s.status_generation==1&&~s.query_open);
check('board_timestamp_reference_sample_preserved',positive.atom.timestamp==1000000&& ...
    positive.atom.sample_timestamp==999995&&positive.atom.reference_timestamp==999996&& ...
    positive.atom.segment_timestamp==999997&&positive.atom.board_timestamp_ns==1e9&& ...
    positive.atom.sample_timestamp_ns==999995000&&positive.atom.reference_timestamp_ns==999996000);
check('raw_receive_and_listener_age_preserved',positive.atom.rx_ns==1.020e9&& ...
    positive.atom.first_chunk_rx_ns==1.020e9&&positive.atom.query_started_ns==1e9&& ...
    positive.atom.listener_age_s==.001&&positive.atom.status_generation==1&& ...
    isequal(positive.atom.raw_bytes,uint8(text)));
[s,guard]=step(s,'GUARD',1.030e9);
check('valid_mode1_nominal_single_publisher_guard',~isempty(guard.guard)&&guard.guard.valid&& ...
    guard.guard.control_mode==1&&guard.guard.single_publisher_contract_pass&& ...
    guard.guard.nominal_mode_observed&&guard.guard.robust_acceleration_zero&& ...
    guard.guard.rx_ns==1.020e9&&guard.guard.listener_age_upper_bound_s==.031);
for t=1.040e9:1e7:1.080e9,[s,reused]=step(s,'GUARD',t);end
check('100hz_guard_reuse_no_new_source_generation',s.status_generation==1&& ...
    reused.guard.status_generation==1&&reused.guard.rx_ns==1.020e9&&s.guards_issued==6&&s.queries_sent==1);
parent=gpenmpcNative.Se3StatusPoller.parseAtom(uint8(text),1.020e9);
check('parent_matlab_parser_numeric_boolean_equivalence', ...
    isequal(parent.commanded_acceleration_ned_mps2(:),positive.atom.commanded_acceleration_ned_mps2)&& ...
    parent.timestamp==positive.atom.timestamp&&parent.listener_age_s==positive.atom.listener_age_s&& ...
    parent.active==positive.atom.active&&parent.single_publisher_contract_pass==positive.atom.single_publisher_contract_pass);
[s,stale]=step(s,'GUARD',1.100e9);
check('printed_age_plus_request_elapsed_blocks_stale_guard',isempty(stale.guard)&& ...
    strcmp(stale.guard_reason,'STATUS_GUARD_STALE')&&~s.fatal_latched&&s.latest.rx_ns==1.020e9);
[duplicateBase,~]=prepared(base,1e9);[duplicateBase,~]=feed(duplicateBase,text,1.020e9,encoder);
[duplicateBase,early]=step(duplicateBase,'TICK',1.069e9);
check('frozen50ms_query_interval',~early.query_prepared);
[duplicateBase,~]=prepared(duplicateBase,1.070e9);
[duplicateBase,duplicate]=feed(duplicateBase,text,1.080e9,encoder);
check('duplicate_complete_status_no_generation_no_rx_renewal',duplicate.completed&& ...
    strcmp(duplicate.reason,'DUPLICATE_STATUS_NO_GENERATION_OR_RX_RENEWAL')&& ...
    duplicateBase.status_generation==1&&duplicateBase.latest.rx_ns==1.020e9&& ...
    duplicateBase.latest_response.rx_ns==1.080e9&&duplicateBase.duplicate_status_responses==1);
[advancing,~]=prepared(duplicateBase,1.130e9);
[advancing,advanced]=feed(advancing,fixture(1100000,11,10),1.140e9,encoder);
check('new_source_status_generation_increments_once',advanced.completed&&advancing.status_generation==2);

[open,~]=prepared(base,1e9);
[chunkState,~]=oneChunk(open,uint8(text(1:70)),1.010e9,encoder);
last=chunkState.last_chunk;m=struct('MsgID',uint32(126),'SystemID',uint8(231), ...
    'ComponentID',uint8(77),'Seq',last.Seq,'Payload',last.Payload);
[chunkState,repeated]=gpenmpcNative.advanceSe3StatusObservation(chunkState,'MESSAGE', ...
    struct('message',m,'rx_ns',1.011e9,'boot_generation',7));
check('duplicate_wire_chunk_not_appended',strcmp(repeated.reason,'DUPLICATE_CHUNK_NO_APPEND')&& ...
    numel(chunkState.buffer)==70&&chunkState.duplicate_chunks==1);
badText=strrep(text,'state_valid: True','state_valid: Unknown');bad('unknown_boolean_fails_closed',badText,'STATUS_BOOLEAN_UNKNOWN');
badText=strrep(text,'    dt_s: 0.004','    dt_s: nan');bad('nonfinite_scalar_fails_closed',badText,'STATUS_NUMBER_NONFINITE');
badText=strrep(text,'[0.1, 0.2, -0.3]','[nan, 0.2, -0.3]');bad('nonfinite_vector_fails_closed',badText,'STATUS_VECTOR_SHAPE_OR_NONFINITE');
badText=strrep(text,'[0.1, 0.2, -0.3]','[0.1, 0.2]');bad('truncated_vector_fails_closed',badText,'STATUS_VECTOR_SHAPE_OR_NONFINITE');
badText=strrep(text,'    dt_s: 0.004','    injected_unknown_state: 5');bad('unknown_field_not_silently_ignored',badText,'STATUS_UNKNOWN_FIELD');
badText=strrep(text,'    dt_s: 0.004',sprintf('    dt_s: 0.004\n    dt_s: 0.005'));bad('duplicate_field_fails_closed',badText,'STATUS_DUPLICATE_FIELD');
badText=strrep(text,sprintf('    dt_s: 0.004\n'),'');bad('completed_but_truncated_fields_fail_closed',badText,'STATUS_REQUIRED_FIELD_MISSING_OR_TRUNCATED');
badText=strrep(text,'control_mode: 1','control_mode: 3');bad('unknown_mode_enum_fails_closed',badText,'STATUS_ENUM_OR_TIMESTAMP_DOMAIN');
badText=strrep(text,'TOPIC: gpenmpc_se3_control_status',sprintf('TOPIC: gpenmpc_se3_control_status\nTOPIC: gpenmpc_se3_control_status'));
bad('multiple_topic_responses_fail_closed',badText,'MULTIPLE_STATUS_RESPONSES_AMBIGUOUS');
bad('wrong_topic_instance_fails_closed',strrep(text,'TOPIC: gpenmpc_se3_control_status', ...
    'TOPIC: gpenmpc_se3_control_status instance 1 #1'),'STATUS_TOPIC_OR_INSTANCE_MISMATCH');
for field={'active','single_publisher_contract_pass','native_position_controller_disabled','state_valid'}
    [z,~]=prepared(base,1e9);v=strrep(text,[field{1},': True'],[field{1},': False']);
    [z,r]=feed(z,v,1.020e9,encoder); %#ok<ASGLU>
    check(['nonadmitted_' field{1} '_no_guard'],r.completed&&isempty(r.guard)&&strcmp(r.guard_reason,'STATUS_GUARD_ATOMS_NOT_ADMITTED'));
end
[z,~]=prepared(base,1e9);[z,r]=feed(z,strrep(text,'control_mode: 1','control_mode: 2'),1.020e9,encoder); %#ok<ASGLU>
check('mode2_not_claimed_nominal',r.completed&&isempty(r.guard));
[z,~]=prepared(base,1e9);[z,r]=feed(z,strrep(text,'robust_acceleration_ned_mps2: [0, 0, 0]', ...
    'robust_acceleration_ned_mps2: [0.01, 0, 0]'),1.020e9,encoder); %#ok<ASGLU>
check('nonzero_robust_term_not_claimed_nominal',r.completed&&isempty(r.guard));
[z,~]=prepared(base,1e9);[z,r]=feed(z,strrep(text,'(0.001 seconds ago)','(1.000 seconds ago)'),1.020e9,encoder); %#ok<ASGLU>
check('stale_listener_text_not_freshened_by_rx',r.completed&&isempty(r.guard)&&strcmp(r.guard_reason,'STATUS_GUARD_STALE'));
[z,~]=prepared(base,1e9);[z,r]=oneChunk(z,uint8(text(1:70)),1.010e9,wrong); %#ok<ASGLU>
check('actual_codec_wrong_source_rejected',r.fatal_latched&&strcmp(r.reason,'SERIAL_CONTROL_SOURCE_MISMATCH'));
[z,~]=prepared(base,1e9);[z,r]=step(z,'SNAPSHOT',1.090e9); %#ok<ASGLU>
check('query_timeout_boundary90ms_not_early',~r.fatal_latched);
[z,r]=step(z,'SNAPSHOT',1.090e9+1);
check('query_timeout_permanent_first_atom',r.fatal_latched&&strcmp(r.reason,'STATUS_QUERY_TIMEOUT'));
[z,r]=step(z,'TICK',1.100e9); %#ok<ASGLU>
check('timeout_no_auto_new_query',r.fatal_latched&&isempty(r.tx_message));
[z,~]=prepared(base,1e9);partial=extractBefore(string(text),strlength(string(text))-4);
[z,~]=feed(z,char(partial),1.020e9,encoder);[z,r]=step(z,'GUARD',1.091e9); %#ok<ASGLU>
check('truncated_no_prompt_times_out_no_guard',r.fatal_latched&&strcmp(r.reason,'STATUS_QUERY_TIMEOUT')&&isempty(r.guard));
[z,~]=gpenmpcNative.advanceSe3StatusObservation(base,'TICK',struct('now_ns',1e9,'boot_generation',7));
[z,r]=oneChunk(z,uint8(text(1:70)),1.010e9,encoder); %#ok<ASGLU>
check('reply_before_owner_send_return_rejected',r.fatal_latched&&strcmp(r.reason,'UNSOLICITED_OR_UNCONFIRMED_SHELL_RESPONSE'));
[z,~]=prepared(base,1e9);[z,r]=gpenmpcNative.advanceSe3StatusObservation(z,'GUARD',struct('now_ns',1.010e9,'boot_generation',8)); %#ok<ASGLU>
check('boot_generation_change_invalidates_guard',r.fatal_latched&&strcmp(r.reason,'VERIFIED_BOOT_GENERATION_CHANGED_OR_ABSENT'));
[z,r]=gpenmpcNative.advanceSe3StatusObservation(base,'TICK',struct('now_ns',1e9,'boot_generation',7,'query','reboot')); %#ok<ASGLU>
check('arbitrary_shell_query_never_generated',r.fatal_latched&&isempty(r.tx_message)&&strcmp(r.reason,'ARBITRARY_QUERY_INPUT_FORBIDDEN'));
[z,~]=step(base,'TICK',1e9);[z,r]=sendReceipt(z,1.001e9,false);
check('failed_send_attempt_preserved',r.fatal_latched&&z.send_attempts==1&&z.queries_sent==0);
[z,~]=step(base,'TICK',1e9);[z,r]=sendReceipt(z,1.091e9,true);
check('late_successful_send_counted_but_query_deadline_fails',r.fatal_latched&& ...
    strcmp(r.reason,'STATUS_QUERY_TIMEOUT')&&z.send_attempts==1&&z.queries_sent==1&&isempty(r.guard));
[z,~]=step(base,'TICK',1e9);[z,r]=sendReceipt(z,1.091e9,false);
check('late_failed_send_attempt_not_lost',r.fatal_latched&&strcmp(r.reason,'STATUS_QUERY_TIMEOUT')&& ...
    z.send_attempts==1&&z.queries_sent==0&&isempty(r.guard));
[z,~]=prepared(base,1e9);[z,~]=feed(z,text,1.020e9,encoder);[z,r]=oneChunk(z,uint8('unexpected'),1.030e9,encoder); %#ok<ASGLU>
check('unsolicited_extra_response_fails_closed',r.fatal_latched&&strcmp(r.reason,'UNSOLICITED_OR_UNCONFIRMED_SHELL_RESPONSE'));
[z,~]=prepared(base,1e9);[z,~]=feed(z,text,1.020e9,encoder);[z,~]=prepared(z,1.070e9);
[z,r]=feed(z,fixture(999999,10,9),1.080e9,encoder); %#ok<ASGLU>
check('board_timestamp_regression_fails_closed',r.fatal_latched&&strcmp(r.reason,'STATUS_TIMESTAMP_REVERSED'));
[z,~]=prepared(base,1e9);[z,~]=feed(z,text,1.020e9,encoder);[z,~]=prepared(z,1.070e9);
[z,r]=feed(z,fixture(1100000,9,9),1.080e9,encoder); %#ok<ASGLU>
check('board_counter_regression_fails_closed',r.fatal_latched&&strcmp(r.reason,'STATUS_COUNTER_REGRESSION'));
[z,~]=prepared(base,1e9);[z,~]=feed(z,text,1.020e9,encoder);[z,~]=prepared(z,1.070e9);
[z,r]=feed(z,strrep(fixture(1100000,11,10),'reset_count: 1','reset_count: 2'),1.080e9,encoder); %#ok<ASGLU>
check('continuous_active_mode1_module_reset_not_hidden',r.fatal_latched&&strcmp(r.reason,'CONTINUOUS_ACTIVE_MODE1_RESET_COUNT_CHANGED'));
[z,~]=prepared(base,1e9);inactiveText=strrep(text,'active: True','active: False');
[z,~]=feed(z,inactiveText,1.020e9,encoder);[z,~]=prepared(z,1.070e9);
inactiveNext=strrep(strrep(fixture(1100000,11,10),'active: True','active: False'),'reset_count: 1','reset_count: 25');
[z,r]=feed(z,inactiveNext,1.080e9,encoder);
check('inactive_ground_reset_growth_preserved_not_fatal',r.completed&&~r.fatal_latched&&isempty(r.guard)&& ...
    r.atom.reset_count==25&&z.inactive_reset_increment_total==24&&z.inactive_reset_increment_observations==1);
[z,~]=prepared(z,1.130e9);
[z,r]=feed(z,strrep(fixture(1200000,12,11),'reset_count: 1','reset_count: 26'),1.140e9,encoder);
check('inactive_to_active_lifecycle_can_admit_original_fresh_generation',r.completed&&~r.fatal_latched&& ...
    ~isempty(r.guard)&&r.guard.status_generation==3&&r.guard.rx_ns==1.140e9&&z.inactive_reset_increment_total==25);
[z,~]=prepared(z,1.190e9);
[z,r]=feed(z,strrep(fixture(1300000,13,12),'reset_count: 1','reset_count: 25'),1.200e9,encoder); %#ok<ASGLU>
check('reset_count_regression_never_hidden',r.fatal_latched&&strcmp(r.reason,'STATUS_RESET_COUNT_REGRESSION'));
[z,~]=step(base,'TICK',1e9);[z,~]=sendReceipt(z,1.050e9,true);
[z,r]=feed(z,text,1.020e9,encoder,1.060e9);
check('owner_deferred_reply_keeps_actual_rx_after_send_receipt',r.completed&&~r.fatal_latched&& ...
    r.atom.rx_ns==1.020e9&&r.atom.first_chunk_rx_ns==1.020e9&&z.last_event_ns==1.060e9&& ...
    ~isempty(r.guard)&&r.guard.receive_age_s==.04);
[z,r]=step(z,'GUARD',1.101e9); %#ok<ASGLU>
check('owner_delayed_processing_cannot_freshen_guard',isempty(r.guard)&&strcmp(r.guard_reason,'STATUS_GUARD_STALE'));
[z,~]=prepared(base,1e9);[z,r]=oneChunk(z,uint8(text(1:70)),1.020e9,encoder,1.010e9); %#ok<ASGLU>
check('processing_time_before_actual_receive_rejected',r.fatal_latched&&strcmp(r.reason,'INVALID_DEFERRED_PROCESSING_TIME'));
[z,~]=prepared(base,1e9);
for k=1:469
    [z,r]=oneChunk(z,uint8(repmat('x',1,70)),1.020e9,encoder);
    if r.fatal_latched,break,end
end
check('bounded_response_buffer_no_overwrite',r.fatal_latched&&strcmp(r.reason,'STATUS_RESPONSE_OVERSIZE')&&numel(z.buffer)<=32768);
report=struct('schema','PURE_MATLAB_SE3_LISTENER_HOST_TEST_V1','pass',all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',nnz([checks.pass]),'checks',checks, ...
    'helper_sha256',m600check.fileSha256(which('gpenmpcNative.advanceSe3StatusObservation')), ...
    'old_python_sha256',m600check.fileSha256(fullfile(build,'tools','nsh_status_poller.py')), ...
    'old_matlab_parser_sha256',m600check.fileSha256(which('gpenmpcNative.Se3StatusPoller')), ...
    'test_sha256',m600check.fileSha256(mfilename('fullpath')+".m"), ...
    'actual_serial_control_wire_bytes',numel(wire),'COM_open',0,'UDP_open',0, ...
    'board_actions',0,'model_runs',0,'query_messages_generated_not_transmitted',true, ...
    'claim','MATLAB codec and synthetic listener tests.');
save(fullfile(outputRoot,'RAW.mat'),'raw','report','wire','decoded');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
fg=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fg
fprintf('CANONICAL_SE3_STATUS %d/%d pass=%d\n',report.checks_passed,report.checks_total,report.pass);
assert(report.pass,'gpenmpcNative:Se3ObserverTests','See result.');
    function check(name,yes),checks(end+1)=struct('name',name,'pass',logical(yes));end %#ok<AGROW>
    function bad(name,txt,reason)
        [v,~]=prepared(base,1e9);[v,r]=feed(v,txt,1.020e9,encoder); %#ok<ASGLU>
        check(name,r.fatal_latched&&strcmp(r.reason,reason));
    end
end
function [s,r]=step(s,op,t)
[s,r]=gpenmpcNative.advanceSe3StatusObservation(s,op,struct('now_ns',t,'boot_generation',7));
end
function [s,r]=sendReceipt(s,t,yes)
[s,r]=gpenmpcNative.advanceSe3StatusObservation(s,'TX_RESULT',struct('now_ns',t,'boot_generation',7, ...
    'request_generation',s.request_generation,'send_succeeded',logical(yes)));
end
function [s,r]=prepared(s,t)
[s,r]=step(s,'TICK',t);assert(r.query_prepared);[s,~]=sendReceipt(s,t,true);
end
function [s,r]=feed(s,text,t,encoder,varargin)
bytes=uint8(text);r=[];
for k=1:70:numel(bytes)
    [s,r]=oneChunk(s,bytes(k:min(end,k+69)),t,encoder,varargin{:});
    if r.fatal_latched,return,end
end
end
function [s,r]=oneChunk(s,bytes,t,encoder,varargin)
d=encoder.Dialect;m=createmsg(d,'SERIAL_CONTROL');m.Payload.device=uint8(10);
m.Payload.flags=uint8(1);m.Payload.timeout=uint16(0);m.Payload.baudrate=uint32(0);
m.Payload.count=uint8(numel(bytes));m.Payload.data=zeros(1,70,'uint8');m.Payload.data(1:numel(bytes))=bytes;
decoded=deserializemsg(d,serializemsg(encoder,m));
e=struct('message',decoded,'rx_ns',t,'boot_generation',7);
if ~isempty(varargin),e.processing_now_ns=varargin{1};end
[s,r]=gpenmpcNative.advanceSe3StatusObservation(s,'MESSAGE',e);
end
function text=fixture(timestamp,inputCount,outputCount)
lines={sprintf('nsh> listener gpenmpc_se3_control_status 0 1\nTOPIC: gpenmpc_se3_control_status\n gpenmpc_se3_control_status'), ...
    sprintf('    timestamp: %.0f (0.001 seconds ago)',timestamp), ...
    sprintf('    sample_timestamp: %.0f',timestamp-5),sprintf('    reference_timestamp: %.0f',timestamp-4), ...
    sprintf('    segment_timestamp: %.0f',timestamp-3),sprintf('    input_sample_count: %.0f',inputCount), ...
    sprintf('    output_publish_count: %.0f',outputCount), ...
    '    rejected_sample_count: 0','    reset_count: 1','    dt_s: 0.004', ...
    '    reference_age_s: 0.002','    segment_age_s: 0.003','    hover_thrust: 0.5', ...
    '    total_mass_kg: 12.9','    yaw_setpoint_rad: 0','    control_mode: 1','    failure_reason: 0'};
for f={'active','reference_fresh','segment_fresh','state_valid','hover_thrust_fresh', ...
        'native_position_controller_disabled','native_attitude_rate_allocator_enabled','single_publisher_contract_pass'}
    lines{end+1}=['    ',f{1},': True']; %#ok<AGROW>
end
for f={'robust_acceleration_ned_mps2','normalized_thrust_ned','position_ned_m','velocity_ned_mps', ...
        'reference_position_ned_m','reference_velocity_ned_mps','reference_acceleration_ned_mps2', ...
        'nominal_feedback_acceleration_ned_mps2','drag_feedforward_acceleration_ned_mps2'}
    lines{end+1}=['    ',f{1},': [0, 0, 0]']; %#ok<AGROW>
end
lines{end+1}='    commanded_acceleration_ned_mps2: [0.1, 0.2, -0.3]';lines{end+1}='nsh>';
text=[strjoin(lines,newline),newline];
end
