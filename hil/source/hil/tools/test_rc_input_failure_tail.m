function test_rc_input_failure_tail
% Execute the production post-failure projection without IO or a live run.
source=fileread(fullfile(fileparts(mfilename('fullpath')),'run_m600_board_local_short_hil.m'));
first=strfind(source,'            result.failure.input_send_tail={};');
last=strfind(source,'            result.failure.heartbeat_send_tail={};');
assert(isscalar(first)&&isscalar(last)&&last>first);
body=source(first:last-1);
for count=[0 5 48]
    recent.raw_transmit_messages={};failure.io_time_s=50;result.failure=struct();
    for k=1:count
        recent.raw_transmit_messages{end+1}=struct('canonical_local_task_inputs',true,'sent_s',k,'id',k); %#ok<AGROW>
        recent.raw_transmit_messages{end+1}=struct('canonical_local_task_inputs',false,'sent_s',k,'id',-k); %#ok<AGROW>
    end
    recent.raw_transmit_messages{end+1}=struct('canonical_local_task_inputs',true,'sent_s',NaN,'id',-100);
    recent.raw_transmit_messages{end+1}=struct('canonical_local_task_inputs',true,'sent_s',51,'id',-101);
    recent.raw_transmit_messages{end+1}=struct('sent_s',25,'id',-102);
    eval(body);
    actual=cellfun(@(x)x.id,result.failure.input_send_tail);
    expected=max(1,count-35):count;
    assert(isequal(actual(:),expected(:)));
end
fprintf('Input failure-tail tests: 3 cases passed.\n');
end
