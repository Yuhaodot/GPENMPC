function test_rc_demo_lifecycle(scope,historicalFixture)
% Test viewer and no-recording lifecycle paths without opening devices.
% UI uses an isolated hidden panel; historicalFixture is an optional recorded MAT.
if nargin<1,scope="OFFLINE";end
if nargin<2,historicalFixture="";end
scope=string(scope);historicalFixture=string(historicalFixture);
assert(isscalar(scope)&&ismember(scope,["OFFLINE","UI"]));assert(isscalar(historicalFixture));
build=fileparts(fileparts(mfilename('fullpath')));p=fullfile(build,'tools');
for name={'gpenmpc_demo_entry.m','gpenmpc_demo_console.m','execute_m600_board_local_short.m','start_gpenmpc_usb_manual.m','run_m600_board_local_short_hil.m'}
    issues=checkcode(fullfile(p,name{1}),'-id');assert(~any(strcmp({issues.id},'PARSE')),name{1});
end
s=fileread(fullfile(p,'execute_m600_board_local_short.m'));
assert(numel(strfind(s,'writeJson(resultPath,result)'))==1);
assert(strfind(s,'finish();clear cleanup')<strfind(s,'writeJson(resultPath,result)'));
s=fileread(fullfile(p,'start_gpenmpc_usb_manual.m'));
call=regexp(s,'execute_m600_board_local_short\(\s*runRoot\s*,\s*"PREPARE_AND_LIVE"\s*,\s*calibration\s*,\s*durationS\s*,\s*false\s*,\s*true\s*\)','match');
assert(isscalar(call)&&contains(s,'if nargin<2,durationS=0;end'));
s=fileread(fullfile(p,'run_m600_board_local_short_hil.m'));
assert(contains(s,'if ~saveFullRaw,safetyOwner=[];end'));
assert(contains(s,'if ~saveFullRaw,return;end'));
if scope=="UI"
options=gpenmpc_demo_test_environment();
f=gpenmpc_demo_console('LIVE','ZH',options);cleanup=onCleanup(@()close(f));
g=gpenmpc_demo_console('LIVE','ZH',options);assert(isequal(f,g));pause(1.2);
assert(strcmp(f.Visible,'off'));
state=getappdata(f,'GPENMPCObservedRunState');assert(~state.busy);
assert(~ismember(state.phase,["SAVING","RUNNING"]));
assert(strcmp(findall(f,'Tag','GPENMPCManualStart').Enable,'on'));
h=System.IO.FileStream(options.LockFile, ...
 System.IO.FileMode.Open,System.IO.FileAccess.ReadWrite,System.IO.FileShare.None);
held=onCleanup(@()h.Dispose());pause(1.2);
assert(strcmp(findall(f,'Tag','GPENMPCManualStart').Enable,'off'));
clear held;pause(1.2);
assert(strcmp(findall(f,'Tag','GPENMPCManualStart').Enable,'on'));
clear cleanup
fprintf('Hidden panel lifecycle tests passed.\n');
end
if strlength(historicalFixture)>0,inspectHistoricalFixture(historicalFixture);end
fprintf('Lifecycle source checks passed.\n');
end
function inspectHistoricalFixture(historicalFixture)
assert(isfile(historicalFixture),'gpenmpcTest:Fixture','Historical fixture does not exist: %s',historicalFixture);
d=load(historicalFixture,'sessionRaw');
frames=d.sessionRaw.post_release_evidence_frames;parts=cell(size(frames));
for k=1:numel(frames)
    if iscell(frames),f=frames{k};else,f=frames(k);end
    b=uint8(f.raw_frame(:));offset=6;if b(1)==253,offset=10;end
    payload=b(offset+(1:double(b(2))));
    if numel(payload)>=9,n=min(double(payload(9)),numel(payload)-9);parts{k}=char(payload(10:9+n).');end
end
fprintf('%s\n',[parts{:}]);
d=load(historicalFixture,'rawIo');
for k=1:numel(d.rawIo.raw_mavlink)
    m=d.rawIo.raw_mavlink{k}.message;
    if m.MsgID==253,fprintf('STATUSTEXT %s\n',char(m.Payload.text));end
end
if isfield(d.rawIo,'canonical_local_window')
    w=d.rawIo.canonical_local_window;disp(fieldnames(w));
    for name={'status','counts','failure','timing'}
        if isfield(w,name{1}),disp(w.(name{1}));end
    end
end
end
