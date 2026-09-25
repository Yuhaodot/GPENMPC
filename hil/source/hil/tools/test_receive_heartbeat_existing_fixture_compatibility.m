function report=test_receive_heartbeat_existing_fixture_compatibility(backupPath)
% Read-only A/B of the exact receive fragment used by the older service test.
% Demonstrates fixture drift separately from the optional callback change.
build=fileparts(fileparts(mfilename('fullpath')));
currentPath=fullfile(build,'host_runtime','+gpenmpcNative','RflyLocalMethodService.m');
paths={backupPath,currentPath};names={'before_callback_fix','after_callback_fix'};
rows=struct('version',{},'legacy_fixture_error',{},'complete_fixture_error',{},'legacy_async_guard_matches',{});
for k=1:2
    source=fileread(paths{k});
    a=strfind(source,'                receiveStart=obj.now();');
    b=strfind(source,'                event.work_timing_ns.receive=obj.now()-receiveStart;');
    assert(isscalar(a)&&isscalar(b));body=source(a:b-1);
    legacy=receiveCase(body,false);complete=receiveCase(body,true);
    guard=regexp(source,'(?m)^\s*if ~gpQueued&&[^\r\n]+','match','once');
    rows(k)=struct('version',names{k},'legacy_fixture_error',legacy, ...
        'complete_fixture_error',complete,'legacy_async_guard_matches',~isempty(guard));
end
assert(strcmp(rows(1).legacy_fixture_error,'MATLAB:nonExistentField'));
assert(strcmp(rows(1).legacy_fixture_error,rows(2).legacy_fixture_error));
assert(isempty(rows(1).complete_fixture_error)&&isempty(rows(2).complete_fixture_error));
assert(~any([rows.legacy_async_guard_matches]));
report=struct('passed',true,'rows',rows,'hardware_actions',0, ...
    'scope','Same legacy receive fixture fails before and after; complete receive fixture passes both; older async text anchor absent in both.');
disp(report);disp(struct2table(rows));
end

function id=receiveCase(body,completeFixture)
% Same first-case fields as test_local_method_preparation at lines 812-823.
obj=struct('now',@()uint64(1000),'ComponentInitialization',false, ...
    'Gp',struct('AsyncEnabled',false,'process',@(varargin)[]), ...
    'Io',struct('pollCanonical',@(varargin)struct('failure','', ...
        'channels',["gp_request","snapshot"],'completed_queue_counts',[1,1], ...
        'snapshot_receive_pending',false)));
if completeFixture,obj.Gp.ReceiveInlineEnabled=false;end
runtimeStateOnly=true;mode='FLIGHT';serviceHeartbeat=@()[]; %#ok<NASGU>
event=struct('gp',{{}},'work_timing_ns',struct('gp',uint64(0),'receive',uint64(0)), ...
    'input_send',struct('messages_send_returned',6)); %#ok<NASGU>
id='';try,eval(body);catch ex,id=ex.identifier;end
end
