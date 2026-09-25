function report=run_m600_outer_host_regression(outputRoot)
% Run host outer-loop regression tests.
assert(~isfolder(outputRoot),'m600check:RegressionOutputExists','Use a fresh output directory.');mkdir(outputRoot);
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'tools'),fullfile(root,'matlab_validation'), ...
    fullfile(root,'m600_coptersim','matlab_validation'));
runner=[];recovery=[];
evalc('runner=run_m600_hil_runner_host_tests();');
evalc('recovery=run_m600_outer_recovery_host_tests();');
plan=run_m600_outer_plan_host_tests(fullfile(outputRoot,'plan_tests'));
rows=struct('name',{},'passed',{});empty=struct('name',{},'payload',{});
r=m600check.evaluateVirtualOutputEvidence(empty);
check('absent_not_measured_zero',~r.all_reported_values_observed_zero&&~r.any_nonzero_or_nonfinite);
z=struct('name','HIL_ACTUATOR_CONTROLS','payload',struct('controls',zeros(1,16)));
r=m600check.evaluateVirtualOutputEvidence(z);check('hil16_zero_observed',r.sixteen_hil_control_values_observed_zero);
q=z;q.payload.controls(1)=.1;r=m600check.evaluateVirtualOutputEvidence(q);check('nonzero_not_hidden',r.any_nonzero_or_nonfinite);
q=z;q.payload.controls(1)=NaN;r=m600check.evaluateVirtualOutputEvidence(q);check('nonfinite_not_hidden',r.any_nonzero_or_nonfinite);
q=struct('name','ACTUATOR_OUTPUT_STATUS','payload',struct('actuator',zeros(1,32)));
r=m600check.evaluateVirtualOutputEvidence(q);check('status_zero_not_hil16_measurement',r.all_reported_values_observed_zero&&~r.sixteen_hil_control_values_observed_zero);
q.payload=struct('unknown',0);r=m600check.evaluateVirtualOutputEvidence(q);check('unknown_payload_fail_closed',r.any_nonzero_or_nonfinite);
d=mavlinkdialect('common.xml',2);h=createmsg(d,'HIL_ACTUATOR_CONTROLS');a=createmsg(d,'ACTUATOR_OUTPUT_STATUS');
check('actual_hil_message_controls16',isfield(h.Payload,'controls')&&numel(h.Payload.controls)==16);
check('actual_output_status_actuator32',isfield(a.Payload,'actuator')&&numel(a.Payload.actuator)==32);
sd=sprintf('nsh> ls /fs/microsd\n/fs/microsd:\n log/\n fault_2000_01_11.log.archived\nnsh>');
r=m600check.inspectSdDiagnosticListing(sd);check('archived_crash_name_preserved_not_current',r.passed);
r=m600check.inspectSdDiagnosticListing(strrep(sd,'log.archived','log'));check('current_crash_log_requires_stop',~r.passed&&numel(r.active_fault_log_names)==1);
r=m600check.inspectSdDiagnosticListing('nsh> ls /fs/microsd');check('incomplete_directory_not_healthy',~r.passed);
r=m600check.inspectSdDiagnosticListing(sprintf('nsh> ls /fs/microsd\n/fs/microsd:\n log/\n'));check('initial_prompt_does_not_prove_directory_completion',~r.passed);
r=m600check.inspectSdDiagnosticListing(sprintf('nsh> ls /fs/microsd\nls: no such file\nnsh>'));check('directory_error_not_healthy',~r.passed);
r=m600check.inspectSdDiagnosticListing(sprintf('nsh> ls /fs/microsd\nnsh>'));check('missing_header_not_empty_directory',~r.passed);
report=struct('status','PASS_HOST_ONLY_NEW_OUTER_AND_RECOVERY', ...
    'runner',runner,'recovery',recovery,'plan',plan,'output_and_message_schema',rows, ...
    'checks_total',runner.cases+recovery.cases+plan.cases+numel(rows), ...
    'checks_passed',runner.passed+recovery.passed+plan.passed+sum([rows.passed]), ...
    'COM_open',0,'UDP_open',0,'simulator_start',0,'board_actions',0);
assert(report.checks_total==report.checks_passed,'m600check:RegressionFailed','Inspect result sections.');
names={'run_m600_matlab_hil','recover_m600_canonical_udp','launch_m600_canonical_hil', ...
    'm600_canonical_serial_preflight','run_m600_outer_host_regression'};
sources=struct('path',{},'sha256',{});
for k=1:numel(names),path=which(names{k});sources(end+1)=struct('path',path,'sha256',m600check.fileSha256(path));end %#ok<AGROW>
report.sources=sources;save(fullfile(outputRoot,'HOST_ONLY_RESULT.mat'),'report');
fid=fopen(fullfile(outputRoot,'HOST_ONLY_RESULT.json'),'w','n','UTF-8');assert(fid>=0);guard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear guard
disp(jsonencode(struct('checks_total',report.checks_total,'checks_passed',report.checks_passed,'COM',0,'UDP',0)));
    function check(name,passed),rows(end+1)=struct('name',name,'passed',logical(passed));end
end
