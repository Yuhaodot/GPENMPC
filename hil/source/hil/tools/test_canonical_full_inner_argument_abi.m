function report=test_canonical_full_inner_argument_abi(outputRoot)
% Use saved MATLAB stateful-controller outputs for ABI checks.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
old=path;guard=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot),'Choose an unused output path.');mkdir(outputRoot);
source=fullfile(gpenmpc_external_path('canonical_full_inner_runtime'),'RAW.mat');
d=load(source,'raw');a=gpenmpcNative.loadCanonicalAssets();
checks=struct('name',{},'pass',{});receipts=cell(numel(d.raw.samples),1);
fixture=fullfile(outputRoot,'MATLAB_ARGUMENTS_AND_EXPECTED.bin');
fid=fopen(fixture,'wb','ieee-be');assert(fid>=0);fg=onCleanup(@()fclose(fid));
fwrite(fid,uint32(numel(d.raw.samples)),'uint32');
for k=1:numel(d.raw.samples)
    s=d.raw.samples{k};[bytes,r]=gpenmpcNative.encodeCanonicalFullInnerArguments(s.begin.full_inner_command,a);
    assert(r.byte_count==829&&isa(bytes,'uint8')&&r.generation==k);
    fwrite(fid,bytes,'uint8');fwrite(fid,uint8(sscanf(r.kernel_argument_sha256,'%2x')),'uint8');
    f=s.feedback;fwrite(fid,[f.wrench_n_nm;f.rotor_command_n;f.diagnostic51],'double');
    fwrite(fid,uint8(f.valid),'uint8');receipts{k}=r;
end
clear fg
check('all_sixty_original_stateful_commands_encoded',numel(receipts)==60);
check('exact_size_full_original_field_precision',all(cellfun(@(r)r.byte_count==829&&r.binary64_count==101,receipts)));
cmd=d.raw.samples{1}.begin.full_inner_command;
variants={cmd,cmd,cmd,cmd,cmd,cmd,cmd};
variants{1}.evidence_scope='LIVE_BOARD';variants{2}.board_publication_authority=true;
variants{3}.augmentation_applications=2;variants{4}.state_up(1)=NaN;
variants{5}.generation=1;variants{6}.coordinate_frame='NED';
variants{7}.configuration_payload_sha256=repmat('0',1,64);
for k=1:numel(variants)
    rejected=false;try,gpenmpcNative.encodeCanonicalFullInnerArguments(variants{k},a);
    catch ex,rejected=startsWith(ex.identifier,'gpenmpcNative:KernelAbi');end
    check("matlab_negative_"+k,rejected);
end
% State generations must remain exact beyond binary64's exact integer range.
large=cmd;large.generation=uint64(2^53)+uint64(3);
[b,r]=gpenmpcNative.encodeCanonicalFullInnerArguments(large,a);
check('uint64_generation_not_rounded_through_double',r.generation==large.generation ...
    &&isequal(b(end-15:end-8),b(end-7:end))&&b(end)==3);
runner=fullfile(build,'px4_full_inner','argument_abi','run_generated_c_abi.ps1');
command=sprintf('pwsh -NoProfile -File "%s" -FixturePath "%s" -OutputDirectory "%s"', ...
    runner,fixture,fullfile(outputRoot,'compiled'));
[code,rawOutput]=system(command);writeText(fullfile(outputRoot,'COMPILE_AND_EXECUTION.txt'),rawOutput);
check('actual_generated_c_compile_and_run_exit_zero',code==0);
cpp=jsondecode(strtrim(rawOutput));
check('actual_c_outputs_match_all_sixty',cpp.actual_kernel_calls==60 ...
    &&cpp.maximum_absolute_error_61<=1e-10);
check('actual_cpp_parser_negative_cases',cpp.negative_decode_cases==9);
check('no_com_board_plant_publication',cpp.com_open==0&&cpp.board_access==0 ...
    &&cpp.plant_count==0&&cpp.publication_count==0&&~cpp.live_protocol_tested);
report=struct('status','PASS_HOST_MATLAB_TO_GENERATED_C_ARGUMENT_ABI', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'argument_records',60,'matlab_negatives',7,'cpp_result',cpp, ...
    'source_raw_sha256',gpenmpcNative.fileSha256(source), ...
    'encoder_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.encodeCanonicalFullInnerArguments')), ...
    'decoder_sha256',gpenmpcNative.fileSha256(fullfile(build,'px4_full_inner','argument_abi','CanonicalKernelArgumentCodec.hpp')), ...
    'fixture_sha256',gpenmpcNative.fileSha256(fixture),'hardware_actions',0, ...
    'limitations',{{'Numerical argument-ABI tests.', ...
    'Stateful inputs use synthetic observations.', ...
    'Identity/boot/estimator/reference/timing/authority admission remains a separate consumer responsibility.'}});
writeText(fullfile(outputRoot,'RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(jsonencode(report));
    function check(name,value)
        checks(end+1)=struct('name',string(name),'pass',logical(value));
        if ~value
            writeText(fullfile(outputRoot,'FAILURE.txt'),char(string(name)));
            error('gpenmpcNative:KernelAbiTest','Check failed: %s',name);
        end
    end
end
function writeText(file,value)
f=fopen(file,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',value);
end
