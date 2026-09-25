function report=run_native_geometry_integration_host_tests(outputDir)
deviceFixture=gpenmpc_test_device_config(); %#ok<NASGU>
% Compact combined HOST result. Synthetic transport is tested separately.
assert(~isfolder(outputDir));mkdir(outputDir);
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'tools'),fullfile(root,'matlab_validation'),fullfile(root,'m600_coptersim','matlab_validation'));
contract=run_temporary_allocator_geometry_contract_tests(fullfile(outputDir,'contract'));
transfer=run_native_allocator_geometry_transfer_tests();
recovery=run_m600_geometry_recovery_host_tests();
outer=run_m600_outer_host_regression(fullfile(outputDir,'outer'));
checks=struct('name',{},'passed',{});
expected=struct('uid','1234605616436508552','board_version',56,'commit','6ea3539157ca358c70a515878b77077af7d4611d');
expected.temporary_allocator_geometry=contract.actual_contract;
expected.temporary_allocator_geometry.entries(1).target_raw_bits_hex='3F000000';
problem='';
try,m600_canonical_serial_preflight(fullfile(outputDir,'MUST_NOT_CREATE.json'),expected);
catch e,problem=e.identifier;end
checks(end+1)=struct('name','serial_rejects_unbound_geometry_before_COM', ...
    'passed',strcmp(problem,'m600check:AllocatorGeometryContract')&&~isfile(fullfile(outputDir,'MUST_NOT_CREATE.json')));
names={'m600check.transferNativeAllocatorGeometry','m600check.validateTemporaryAllocatorGeometry', ...
    'm600check.validateTemporaryAllocatorGeometryEntries','m600check.buildTemporaryAllocatorGeometryContract', ...
    'run_native_allocator_geometry_transfer_tests','run_temporary_allocator_geometry_contract_tests', ...
    'run_m600_geometry_recovery_host_tests','run_m600_hil_runner_host_tests','run_m600_outer_plan_host_tests', ...
    'recover_m600_canonical_udp','run_m600_matlab_hil','launch_m600_canonical_hil','m600_canonical_serial_preflight', ...
    'run_native_geometry_integration_host_tests'};
sources=struct('path',{},'sha256',{});
for k=1:numel(names),path=which(names{k});assert(~isempty(path));sources(end+1)=struct('path',path,'sha256',m600check.fileSha256(path));end %#ok<AGROW>
total=contract.case_count+transfer.checks_total+recovery.case_count+outer.checks_total+numel(checks);
passed=contract.cases_passed+transfer.checks_passed+recovery.cases_passed+outer.checks_passed+nnz([checks.passed]);
report=struct('status','HOST_ONLY_GEOMETRY_APPLICATION_AND_INDEPENDENT_RESTORATION', ...
    'passed',total==passed,'checks_total',total,'checks_passed',passed, ...
    'contract',contract,'transfer',transfer,'recovery',recovery,'outer',outer,'serial_negative',checks, ...
    'sources',sources,'hardware_actions',0,'COM_open',0,'model_actions',0,'flight_admission',false);
save(fullfile(outputDir,'RESULT.mat'),'report','-v7');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear c
fprintf('Geometry host integration: %d/%d. Board/COM=0.\n',passed,total);
assert(report.passed,'m600check:GeometryIntegratedTestsFailed','Retain complete per-case results.');
end
