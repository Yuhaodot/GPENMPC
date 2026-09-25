function report=test_gpenmpc_rfly_view(expectCold)
% Test viewer display behavior.
if nargin<1,expectCold=false;end
toolsRoot=fileparts(mfilename('fullpath'));
outputRoot=gpenmpc_external_path('display_test_output_root');
assert(isfolder(outputRoot));
report=struct('test','RflySim3D display startup','cold_start_requested',logical(expectCold));
files={'ensure_gpenmpc_rfly_view.m','gpenmpc_demo_console.m','start_gpenmpc_usb_manual.m'};
for k=1:numel(files)
    findings=checkcode(fullfile(toolsRoot,files{k}),'-id');
    report.code_analysis.(erase(files{k},'.m'))=findings;
end
assert(~any(strcmp({report.code_analysis.ensure_gpenmpc_rfly_view.id},'MOCUP')));
lastwarn('');
report.first=ensure_gpenmpc_rfly_view();
if expectCold,assert(report.first.launched,'The cold test did not create a new view.');end
report.second=ensure_gpenmpc_rfly_view();
assert(report.second.reused&&report.first.pid==report.second.pid, ...
    'The second call must reuse the same renderer process.');
report.missing_launcher_error='';
try
    ensure_gpenmpc_rfly_view(1,fullfile(outputRoot,'not_installed','RflySim3D.exe'));
catch ex
    report.missing_launcher_error=ex.identifier;
end
assert(strcmp(report.missing_launcher_error,'gpenmpcView:LauncherMissing'));
report.after_failure=ensure_gpenmpc_rfly_view();
assert(report.after_failure.reused&&report.after_failure.pid==report.first.pid);
[warningMessage,warningId]=lastwarn;
assert(isempty(warningMessage),'Display startup emitted a warning: %s (%s)',warningMessage,warningId);
console=fileread(fullfile(toolsRoot,'gpenmpc_demo_console.m'));
runner=fileread(fullfile(toolsRoot,'start_gpenmpc_usb_manual.m'));
assert(contains(console,'if string(initialMode)=="LIVE",showViewExplicitly();end'));
assert(contains(console,'receipt=ensure_gpenmpc_rfly_view([],[],true);'));
assert(contains(console,'if ~prepareView(),refreshRunStatus(true);return;end'));
assert(contains(runner,'info.visualization=ensure_gpenmpc_rfly_view();'));
assert(~contains(console,'runPhase="VIEW_UNAVAILABLE"'));
report.open_and_start_hooks_present=true;
report.display_error_preserves_control_phase=true;
report.flight_started=false;
report.firmware_or_control_configuration_changed=false;
report.passed=true;
suffix='reuse';if expectCold,suffix='cold';end
f=fopen(fullfile(outputRoot,['view_test_' suffix '.json']),'w');assert(f>=0);
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s',jsonencode(report,'PrettyPrint',true));
fprintf('VIEW_TEST_PASS: %s; renderer PID %d; reused PID %d\n',suffix,report.first.pid,report.second.pid);
end
