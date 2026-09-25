function report=export_rfly_canonical_application_parameters(outputRoot)
% Export round-trip binary64 parameters from the bound MATLAB assets.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
old=path;cleanup=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();p=a.kernelParameters;
assert(~isfolder(outputRoot),'Preserve prior generated evidence.');mkdir(outputRoot);
names={'kp','kd','kr','kw','drag','inertia','pseudoinverse','baseMass','totalThrust','rotorUpper','maxTilt'};
lengths=[3,3,3,3,3,9,24,1,1,1,1];flat=[];
lines=["#pragma once";"// Parameters exported by the passport-bound MATLAB loader."; ...
    "#include ""../CanonicalRflyExecutor.hpp""";"namespace gpenmpc_rfly_px4 {"; ...
    "inline gpenmpc_consumption::KernelParameters canonical_application_parameters() noexcept {"; ...
    "gpenmpc_consumption::KernelParameters p{};"];
for k=1:numel(names)
    v=double(p.(names{k})(:));assert(numel(v)==lengths(k)&&all(isfinite(v)));
    flat=[flat;v]; %#ok<AGROW>
    for j=1:numel(v)
        literal=sprintf('%.17g',v(j));
        if ~contains(literal,{'.','e','E'}),literal=[literal '.0'];end
        assert(isequal(typecast(str2double(literal),'uint64'),typecast(v(j),'uint64')));
        if numel(v)>1,field=sprintf('%s[%d]',names{k},j-1);else,field=names{k};end
        lines(end+1)=string(sprintf('p.%s = %s;',field,literal)); %#ok<AGROW>
    end
end
assert(numel(flat)==52);
[~,~,endian]=computer;wire=flat;if endian=='L',wire=swapbytes(wire);end
raw=[uint8([82;65;80;49]);reshape(typecast(wire,'uint8'),[],1)];
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(raw,'int8'));
sha=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
lines(end+1)="return p; }";
words=strings(1,8);for k=1:8,words(k)="0x"+string(sha((k-1)*8+(1:8)));end
lines(end+1)="constexpr gpenmpc_rfly_execution::Digest kCanonicalApplicationParameterSha{"+strjoin(words,",")+"};";
lines(end+1)="} // namespace gpenmpc_rfly_px4";
header=fullfile(outputRoot,'CanonicalApplicationParameters.hpp');
f=fopen(header,'w','n','UTF-8');assert(f>=0);fprintf(f,'%s\n',lines);fclose(f);
binary=fullfile(outputRoot,'CANONICAL_PARAMETERS_RAP1.bin');
f=fopen(binary,'wb');assert(f>=0);fwrite(f,raw,'uint8');fclose(f);
report=struct('status','PASS_CANONICAL_PARAMETER_EXPORT__NOT_RUNTIME_OR_HIL', ...
    'parameter_count',52,'parameter_order',{names},'parameter_sha256',sha, ...
    'parameters',p,'header',header,'configuration_payload_sha256', ...
    a.configurationBinding.effective_configuration_payload_sha256, ...
    'binding',a.binding,'hardware_actions',0,'solver_calls',0,'physical_or_virtual_output',0);
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
fprintf('CANONICAL_PARAMETER_EXPORT PASS count=52 sha=%s hardware=0\n',sha);
end
