function report=probe_canonical_reference_window_codegen(inputRoot,outputRoot)
% Check fixed-struct portable-C generation feasibility.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
addpath(fullfile(gpenmpcNative.canonicalAssetRoot(),'method_source','matlab','enmpc'));
originalDirectory=pwd;directoryCleanup=onCleanup(@()cd(originalDirectory)); %#ok<NASGU>
if ~isfolder(outputRoot),mkdir(outputRoot);end
resultPath=fullfile(outputRoot,'RESULT.json');
if isfile(resultPath),resultPath=fullfile(outputRoot,'RESULT_CURRENT_CONFIG.json');end
if isfile(resultPath),resultPath=fullfile(outputRoot,'RESULT_STATIC_SCHEMA.json');end
assert(~isfile(resultPath),'Choose an unused output path.');
inputPath=fullfile(inputRoot,'QUERY_CODEGEN_INPUTS.mat');q=load(inputPath);
cfg=coder.config('lib');cfg.GenCodeOnly=true;cfg.TargetLang='C';
cfg.InstructionSetExtensions='None';cfg.EnableDynamicMemoryAllocation=false;cfg.GenerateReport=false;
cfg.EnableOpenMP=false;cfg.EnableAutoParallelization=false;
source=which('gpenmpcNative.queryCanonicalReferenceWindow');
sourceBefore=sha(source);
transitionSource=which('gpenmpcNative.canonicalReferenceTransitionFromJet');
transitionBefore=sha(transitionSource);
report=struct('schema','CANONICAL_REFERENCE_WINDOW_CODEGEN_FEASIBILITY_V1', ...
    'pass',false,'source',source,'source_sha256',sourceBefore, ...
    'transition_source',transitionSource,'transition_source_sha256',transitionBefore, ...
    'input_path',char(inputPath),'input_sha256',sha(inputPath), ...
    'target','C_LIBRARY_GENERATED_SOURCE_ONLY','instruction_set_extensions','None', ...
    'openmp',false,'auto_parallelization',false, ...
    'enable_dynamic_memory_allocation',false, ...
    'compiled_or_executed',false,'hardware_actions',0,'model_calls',0,'solver_calls',0, ...
    'error_identifier','','error_message','','error_report','');
try
    cd(outputRoot);
    codegen('-config',cfg,'gpenmpcNative.queryCanonicalReferenceWindow', ...
        '-args',{q.window,q.state,q.request},'gpenmpcNative.canonicalReferenceTransitionFromJet', ...
        '-args',{zeros(3,4),1.0,0.0,0.0,zeros(3,1),zeros(3,1),.009,4.0}, ...
        '-d',char(fullfile(outputRoot,'generated_c')));
    report.pass=true;
catch err
    report.error_identifier=err.identifier;report.error_message=err.message;
    report.error_report=getReport(err,'extended','hyperlinks','off');
end
report.source_modified=~strcmp(sourceBefore,sha(source))||~strcmp(transitionBefore,sha(transitionSource));
f=fopen(resultPath,'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s',jsonencode(report,PrettyPrint=true));clear c
fprintf('WINDOW_CODEGEN pass=%d modified=%d error=%s\n',report.pass,report.source_modified,report.error_identifier);
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(b,'int8'));
value=string(upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
