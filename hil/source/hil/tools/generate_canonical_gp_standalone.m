function report=generate_canonical_gp_standalone(outputRoot)
% Generate the fixed GP256 predictor as standalone C and export fixtures.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=fullfile(gpenmpc_external_path('host_gp_numerical_runtime'));
oldPath=path;oldDir=pwd;
cleanup=onCleanup(@()restoreHost(oldPath,oldDir)); %#ok<NASGU>
assert(~isfile(fullfile(outputRoot,'GENERATION_RESULT.json')), ...
    'gpenmpcNative:GpStandaloneEvidenceExists','Do not replace a completed generation.');
if ~isfolder(outputRoot),mkdir(outputRoot);end
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
diaryCleanup=onCleanup(@()diary('off')); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
receipt=jsondecode(fileread(fullfile(parent,'RESULT.json')));
assert(strcmp(receipt.status,'PASS_HOST_ONLY_CANONICAL_GP256_GENERATED_NUMERICS'));
assert(strcmpi(receipt.model_sha256,a.binding.gp_model_sha256));
source=string(which('gpenmpcSparseGpPredict'));
wrapper=string(which('gpenmpcNative.canonicalSparseGpFixedInput'));
assert(strcmpi(receipt.canonical_predictor_sha256,sha(source)));
assert(strcmpi(receipt.wrapper_sha256,sha(wrapper)));
inputPath=fullfile(parent,'INPUTS.mat');expectedPath=fullfile(parent,'NUMERICAL_RESULTS.mat');
inputs=load(inputPath,'X','fixed');expected=load(expectedPath,'X','expected');
assert(isequaln(inputs.X,expected.X)&&isequal(size(inputs.X),[293 17]) ...
    &&isequal(size(expected.expected),[293 18]));
fixed=inputs.fixed;
assert(isequal(size(fixed.inducing_standardized),[256 17]) ...
    &&isa(fixed.kmm_cholesky,'double'));
% Verify every saved expected row again with the exact compiled constants.
for k=1:293
    current=gpenmpcNative.canonicalSparseGpFixedInput(fixed,inputs.X(k,:));
    assert(isequaln(current,expected.expected(k,:)), ...
        'gpenmpcNative:GpStandaloneOracle','Parent expected differs at row %d.',k);
end
fixture=fullfile(outputRoot,'GP256_INPUT_EXPECTED_LE.bin');
if isfile(fixture)
    f=fopen(fixture,'rb','ieee-le');assert(f>=0);guard=onCleanup(@()fclose(f));
    assert(isequal(fread(f,8,'*uint8').',uint8('RAGP2561')));
    assert(isequal(fread(f,3,'*uint32').',uint32([293 17 18])));
    assert(isequaln(fread(f,[35 293],'*double'),[inputs.X expected.expected].'));
    assert(isempty(fread(f,1,'*uint8')));clear guard
else
    f=fopen(fixture,'wb','ieee-le');assert(f>=0);guard=onCleanup(@()fclose(f));
    fwrite(f,uint8('RAGP2561'),'uint8');
    fwrite(f,uint32([293 17 18]),'uint32');
    fwrite(f,[inputs.X expected.expected].','double');clear guard
end
cfg=coder.config('lib');cfg.GenCodeOnly=true;cfg.GenerateReport=false;
cfg.TargetLang='C';
% For a library target, coder.Constant defines the C interface;
% ConstantInputs='Remove' applies only to MEX generation.
cd(outputRoot);
codegen('-config',cfg,'gpenmpcNative.canonicalSparseGpFixedInput', ...
    '-args',{coder.Constant(fixed),zeros(1,17)},'-d',fullfile(outputRoot,'generated'));
files=dir(fullfile(outputRoot,'generated','**','*'));files=files(~[files.isdir]);
generated=struct('path',{},'bytes',{},'sha256',{});
for k=1:numel(files)
    file=fullfile(files(k).folder,files(k).name);
    generated(end+1)=identity(file); %#ok<AGROW>
end
report=struct('status','PASS_HOST_ONLY_CANONICAL_GP256_STANDALONE_C_GENERATED', ...
    'source',identity(source),'wrapper',identity(wrapper), ...
    'parent_receipt',identity(fullfile(parent,'RESULT.json')), ...
    'parent_inputs',identity(inputPath),'parent_expected',identity(expectedPath), ...
    'fixture',identity(fixture),'generated_files',generated, ...
    'model_sha256',a.binding.gp_model_sha256,'row_count',293,'output_count',18, ...
    'gp_inducing_count',256,'input_count',17,'precision','double', ...
    'generator_target','lib','generation_only',true, ...
    'constant_inputs','coder.Constant; real generated C signature checked by native harness', ...
    'host_wrapper_repair','Removed MEX-only ConstantInputs property from EmbeddedCodeConfig', ...
    'generated_C_compilation_verified',false,'native_process_execution_verified',false, ...
    'model_or_solver_runs',0,'board_actions',0,'COM_UDP_actions',0);
writeJson(fullfile(outputRoot,'GENERATION_RESULT.json'),report);
fprintf('Standalone GP256 C generated; 293 parent rows rechecked, compilation still required.\n');
end
function result=identity(file)
d=dir(file);assert(isscalar(d));
result=struct('path',char(file),'bytes',d.bytes,'sha256',char(sha(file)));
end
function value=sha(file)
value=upper(string(gpenmpcSha256File(char(file))));
end
function writeJson(file,value)
f=fopen(file,'wt');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(value,PrettyPrint=true));
end
function restoreHost(p,d)
path(p);cd(d);
end
