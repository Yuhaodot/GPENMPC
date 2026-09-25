function fixture = loadFixture(workRoot,taskPath)
%LOADFIXTURE HOST-only source-addressed inputs for a numerical oracle test.
% Does not instantiate BoardOuterService/LiveHilPlantService or any transport.
if nargin<1
    workRoot=string(gpenmpc_external_path('native_visual_host_runtime'));
end
if nargin<2
    taskPath=string(gpenmpc_external_path('physical_task_fixture'));
end
sourceRoot=fullfile(workRoot,'vendor','current_method','implementation');
addpath(fullfile(workRoot,'src'),'-begin');
addpath(genpath(sourceRoot),'-begin');
binding=jsondecode(fileread(fullfile(workRoot,'SOURCE_BINDING.json')));
expectedSource="98030691749450E45E6F11ACAF656A9DCA3889402ACFDCD50E0114F23047A586";
assert(string(binding.source_manifest_sha256)==expectedSource);
names=["gpenmpcM600SixDofPlantDerivative";"gpenmpcM600Allocation"; ...
    "gpenmpcM600StructuredResidual";"gpenmpcQuaternionRotation"; ...
    "gpenmpcQuaternionDerivativeMatrix"];
hashes=["9B493FC77171033273D444C0C7532AFB2AA39A40D5C8BB5C2B486835D68CA4F6"; ...
    "8888EFB8BF117C6571ECE20651CDEB2CA1B43671B259E4C62E80B032E372D00F"; ...
    "52EAE9FC08DA834BD9D6B8ECC113851A626766D3A6EEF6282AECD9CABB49FBCD"; ...
    "22AFB37CFC0415E2AE5DB550C829D3BBA17BF7125338CFE7FFA521263C297E71"; ...
    "6CE1CF55E1F96FFE4B24C5560158F02439FFEBAB55B71F9DDDB3E9818D1FDFF7"];
for k=1:numel(names)
    exact=fullfile(sourceRoot,'dynamics',names(k)+'.m');
    assert(strcmpi(string(which(names(k))),string(exact)), ...
        'm600check:ShadowedOracle','Authoritative source is shadowed.');
    assert(strcmpi(gpenmpcNative.fileSha256(exact),hashes(k)), ...
        'm600check:OracleHash','Authoritative source bytes changed.');
end
contactPath=fullfile(workRoot,'src','+gpenmpcNative','compliantContactState.m');
assert(strcmpi(string(which('gpenmpcNative.compliantContactState')),contactPath));
assert(strcmpi(gpenmpcNative.fileSha256(contactPath), ...
    'D90339C9D83373C121AC3A4C5826E72420B6F29C1127344685626097AEC4C095'));
calPath=fullfile(sourceRoot,'simulink','assets','M600_DYN_CALIBRATION.json');
profilePath=fullfile(sourceRoot,'simulink','assets','M600_PLATFORM_PROFILE.json');
assert(strcmpi(gpenmpcNative.fileSha256(calPath), ...
    'ACF09DFA521897B55D5EC473E44DCC4F546356B36465F56D2C3F1D628409583E'));
assert(strcmpi(gpenmpcNative.fileSha256(profilePath), ...
    '406786CE01DB0B1F66B1AE305D73C9421AC03FCB5B5F5E2664D708106DD9D09B'));
loaded=load(taskPath,'physicalTask');
fixture.calibration=jsondecode(fileread(calPath));
fixture.profile=jsondecode(fileread(profilePath));
fixture.mission=loaded.physicalTask.mission_config;
fixture.parameters=m600check.packParameters( ...
    fixture.calibration,fixture.profile,fixture.mission);
fixture.source_manifest_sha256=expectedSource;
fixture.task_path=string(taskPath);
fixture.task_sha256=gpenmpcNative.fileSha256(taskPath);
fixture.initial_payload_kg=double(loaded.physicalTask.reference.payload_kg(1));
fixture.fixture_description="Numeric task fixture for pointwise and step equivalence tests";
fixture.oracle_names=names;
fixture.oracle_sha256=hashes;
end
