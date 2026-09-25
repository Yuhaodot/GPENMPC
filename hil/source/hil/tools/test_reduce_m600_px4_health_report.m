function result=test_reduce_m600_px4_health_report(outputDir)
% Test health-report reduction.
arguments,outputDir (1,1) string = "",end
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'matlab_validation'), ...
    fullfile(root,'m600_coptersim','matlab_validation'));
path=string(gpenmpc_external_path('px4_event_metadata'));
sha="951C8B738A4FF0142E2AC30C2D0E105255DCECC889002DD86E29140CCB75A837";
req=struct('sent_s',10,'source_system',1,'source_component',1,'minimum_event_boot_ms',1000);
good=fixture(5,true);bad=fixture(5,false);checks=struct();
r=run(good);checks.complete_positive=r.complete&&r.can_arm_offboard&&r.can_arm_offboard_known;
r=run(bad);checks.denial_retained=r.complete&&~r.can_arm_offboard&&r.can_arm_offboard_known;
assert(r.complete,'m600health:Fixture','%s',r.failure);
checks.detail_text_and_raw=strcmp(r.details(1).message_template, ...
    'Navigation error: No valid position estimate')&&numel(r.details(1).raw_arguments_uint8)==40;
checks.namespace_high_byte=double(r.details(1).event_id)==16777216+13835193;
checks.offboard_mask=r.details(1).applies_to_offboard&&r.offboard_mode_mask==16384;
x=good;x{2}.message.Payload.arguments(1:4)=le(4);r=run(x);
checks.other_mode_detail_not_offboard=r.complete&&~r.details(1).applies_to_offboard;
x=good;x{2}.message.Payload.id=uint32(16777216+1058956);r=run(x);
checks.no_guessed_interpolation=r.complete&&strcmp(r.details(1).message_template,'CPU load too high: {3:.1}%');
x=fixture(65535,true);r=run(x);checks.uint16_wrap=r.complete&&r.first_sequence==65535&&r.last_sequence==1;
x={good{1},good{2},good{2},good{3}};r=run(x);checks.exact_duplicate_allowed=r.complete&&r.duplicate_count==1;
x={good{1},good{3},good{2}};r=run(x);checks.reordered_retrieval_contiguous=r.complete;
x={good{1},good{3}};r=run(x);checks.missing_sequence_unknown=unknown(r,'EVENT_SEQUENCE_GAP');
x={good{1},good{2},good{2},good{3}};x{3}.message.Payload.arguments(1)=uint8(1);r=run(x);
checks.conflicting_duplicate_unknown=unknown(r,'SEQUENCE_PAYLOAD_CONFLICT');
x=good;x{2}.message.Payload.id=uint32(16777216+1234567);r=run(x);
checks.unknown_id_unknown=unknown(r,'UNKNOWN_EVENT_ID');
x=good;x{2}.message.Payload.id=uint32(33554432+13835193);r=run(x);
checks.unknown_namespace_unknown=unknown(r,'UNKNOWN_EVENT_COMPONENT_NAMESPACE');
r=run(good(1:2));checks.incomplete_unknown=unknown(r,'HEALTH_SUMMARY_MISSING');
r=run(good(2:3));checks.missing_start_unknown=unknown(r,'ARMING_SUMMARY_MISSING');
x=good;x{1}.message.Payload.arguments(1)=uint8(1);r=run(x);
checks.chunk_unsupported_unknown=unknown(r,'UNSUPPORTED_SUMMARY_CHUNK');
x=good;x{2}.message.Payload.arguments=zeros(1,39,'uint8');r=run(x);
checks.short_arguments_unknown=unknown(r,'EVENT_ARGUMENT_BYTES');
x=good;x{2}.message.Payload.arguments=nan(1,40);r=run(x);
checks.nonfinite_bytes_unknown=unknown(r,'EVENT_ARGUMENT_BYTES');
r=reduce_m600_px4_health_report(good,req,path,repmat('0',1,64));
checks.metadata_mismatch_unknown=unknown(r,'METADATA_SHA_MISMATCH');
r=reduce_m600_px4_health_report(good,req,path,sha,'overflow',true);
checks.declared_overflow_unknown=unknown(r,'BUFFER_OVERFLOW');
r=reduce_m600_px4_health_report(good,req,path,sha,'max_event_rows',2);
checks.actual_overflow_unknown=unknown(r,'BUFFER_OVERFLOW');
x=good;for k=1:3,x{k}.rx_s=9;end;r=run(x);
checks.previous_request_excluded=unknown(r,'NO_CURRENT_EVENT_REPORT')&&r.ignored_pre_request_count==3;
x=good;for k=1:3,x{k}.message.Payload.event_time_boot_ms=uint32(999);end;r=run(x);
checks.replayed_old_boot_excluded=unknown(r,'NO_CURRENT_EVENT_REPORT')&&r.ignored_old_boot_count==3;
x=good;for k=1:3,x{k}.message.SystemID=2;end;r=run(x);
checks.other_source_excluded=unknown(r,'NO_CURRENT_EVENT_REPORT')&&r.ignored_other_source_count==3;
x=good;x{4}=other('CURRENT_EVENT_SEQUENCE',struct('sequence',uint16(7),'flags',uint8(1)));r=run(x);
checks.reset_unknown=unknown(r,'SEQUENCE_RESET_AFTER_REQUEST');
x=good;x{4}=other('RESPONSE_EVENT_ERROR',struct('sequence',uint16(6),'reason',uint8(0)));r=run(x);
checks.unavailable_unknown=unknown(r,'EVENT_RETRIEVAL_ERROR');
ack=other('COMMAND_ACK',struct('command',uint16(401),'result',uint8(0)));
r=run({ack});checks.accepted401_not_health_pass=unknown(r,'NO_CURRENT_EVENT_REPORT');
r=run([bad,{ack}]);checks.accepted401_cannot_overwrite_denial=r.complete&&~r.can_arm_offboard;
x=good;next=fixture(8,false);next{1}.rx_s=11;x=[x,next(1)];r=run(x);
checks.new_incomplete_does_not_reuse_old_pass=unknown(r,'HEALTH_SUMMARY_MISSING');
x=good;x{2}.message.Payload.destination_system=uint8(254);r=run(x);
checks.other_destination_unknown=unknown(r,'EVENT_DESTINATION_MISMATCH');
result=struct('status','PASS_HOST_ONLY_PX4_HEALTH_REPORT_REDUCER','passed',all(structfun(@logical,checks)), ...
    'checks',checks,'checks_passed',nnz(structfun(@logical,checks)), ...
    'checks_total',numel(fieldnames(checks)),'metadata_path',char(path),'metadata_sha256',char(sha), ...
    'hardware_actions',0,'COM_open',0,'UDP_open',0,'MATLAB_pure_functions_executed',true);
fprintf('PX4_HEALTH_REPORT_REDUCER %d/%d\n',result.checks_passed,result.checks_total);
if strlength(outputDir)>0
    assert(~isfolder(outputDir)&&~isfile(outputDir),'m600health:TestOutputExists');mkdir(outputDir);
    fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
    guard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear guard
end
names=string(fieldnames(checks));failed=names(~structfun(@logical,checks));
assert(result.passed,'m600health:ReducerTests','Failed: %s',strjoin(failed,', '));
    function q=run(records),q=reduce_m600_px4_health_report(records,req,path,sha);end
end

function yes=unknown(r,why)
yes=~r.complete&&~r.can_arm_offboard_known&&~r.can_arm_offboard&&strcmp(r.status,'UNKNOWN')&&strcmp(r.failure,why);
end
function rows=fixture(sequence,canArm)
a=zeros(1,40,'uint8');a(2:5)=le(double(~canArm)*131072);a(10:13)=le(double(canArm)*16384);a(14:17)=le(16384);
d=zeros(1,40,'uint8');d(1:4)=le(16384);d(5)=uint8(17);
h=zeros(1,40,'uint8');h(2:5)=le(131072);h(6:9)=le(double(~canArm)*131072);
rows={event(11047904,sequence,a,10.1),event(13835193,mod(sequence+1,65536),d,10.2), ...
    event(1914663,mod(sequence+2,65536),h,10.3)};
end
function r=event(id,sequence,bytes,t)
p=struct('id',uint32(16777216+id),'event_time_boot_ms',uint32(1001), ...
    'sequence',uint16(sequence),'destination_system',uint8(0),'destination_component',uint8(0), ...
    'log_levels',uint8(2),'arguments',bytes);
r=other('EVENT',p);r.rx_s=t;
end
function r=other(topic,p)
r=struct('topic',topic,'rx_s',10.4,'message',struct('SystemID',uint8(1), ...
    'ComponentID',uint8(1),'Payload',p));
end
function a=le(x)
x=uint32(x);a=uint8([bitand(x,255),bitand(bitshift(x,-8),255), ...
    bitand(bitshift(x,-16),255),bitand(bitshift(x,-24),255)]);
end
