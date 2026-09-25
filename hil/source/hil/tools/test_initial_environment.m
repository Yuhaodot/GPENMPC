function report=test_initial_environment
% Check the environment frame used by session initialization.
hilRoot=fileparts(fileparts(mfilename('fullpath')));
asset=fullfile(fileparts(fileparts(hilRoot)),'assets','environment', ...
    'initial_environment.mat');
addpath(fullfile(hilRoot,'matlab_validation'));
data=load(asset,'initialEnvironment');
frame=data.initialEnvironment;
assert(isstruct(frame)&&isscalar(frame));
assert(all(structfun(@(value)isnumeric(value)||islogical(value),frame)));
[bytes,fields]=gpenmpcTaskIo.encodePlantEnvironmentV2(frame,1,2.21);
assert(isa(bytes,'uint8')&&numel(bytes)==232&&numel(fields)==28);
assert(fields(1)==2&&fields(3)==frame.source_io_time_s);
for k=1:8
    [nextBytes,nextFields]=gpenmpcTaskIo.encodePlantEnvironmentV2(frame,1,2.21);
    assert(isequal(bytes,nextBytes)&&isequal(fields,nextFields));
end
report=struct('passed',true,'frame_fields',numel(fieldnames(frame)), ...
    'encoded_bytes',numel(bytes),'repeated_encodings',8);
end
