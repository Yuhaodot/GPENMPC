function result=test_m600_fixed_runtime_path()
% Test asset and runtime-path preparation.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
entry=fullfile(build,'tools','execute_m600_board_local_short.m');
source=fileread(entry);
first=strfind(source,"parentRuntime=fullfile(build,");
last=strfind(source,'% Ground calibration is independent');
assert(isscalar(first)&&isscalar(last)&&first<last);
block=source(first:last-1);
getSource=struct('model_sha256', ...
 'D536EACE85EBA30A6CE07EF5B38FF108B132C9E3B230A3C46E93F3E1C9CA6E12');
operation="PREPARE";
outputRoot=fullfile(build,'live','runtime_path_fixture'); %#ok<NASGU>
eval(block);
fixedExe=exe;fixedRuntime=runtime;fixedModel=model;
assert(strcmpi(fixedExe,fullfile(build,'live','COPTERSIM_M600_RUNTIME','CopterSimNoUI.exe')));
assert(strcmpi(m600check.fileSha256(fixedExe), ...
 '94B81EFB44058176DD5353669D9C28FC5331CC8411AB9EA3F2D27C1E8C343241'));
% Compare the runtime's eleven dependencies with the prepared runtime.
reference=fullfile(outputRoot,'runtime');
files=dir(fullfile(fixedRuntime,'**','*'));files=files(~[files.isdir]);
assert(numel(files)==11);
for k=1:numel(files)
 path=fullfile(files(k).folder,files(k).name);
 relative=extractAfter(string(path),strlength(fixedRuntime)+1);
 assert(strcmpi(m600check.fileSha256(path),m600check.fileSha256(fullfile(reference,relative))));
end
before=[files.datenum];
% Require the application identity to be independent of output directory name.
outputRoot=fullfile(build,'live','PATH_SELECTION_ONLY'); %#ok<NASGU>
operation="LIVE";eval(block);
assert(strcmpi(exe,fixedExe)&&strcmpi(runtime,fixedRuntime)&&strcmpi(model,fixedModel));
after=dir(fullfile(fixedRuntime,'**','*'));after=after(~[after.isdir]);
assert(isequal({files.name},{after.name})&&isequal(before,[after.datenum]));
issues=checkcode(char(entry),'-id');
assert(~any(strcmp({issues.id},'SYNER')),'MATLAB syntax error in the selected entry.');
result=struct('passed',true,'same_executable_across_output_roots',true, ...
 'dependency_files_identical',numel(files),'repeat_preparation_changed_files',0, ...
 'simulator_executable',fixedExe,'simulator_working_directory',fixedRuntime, ...
 'board_actions',0,'simulator_launches',0,'firewall_changes',0);
disp(jsonencode(result));
end
