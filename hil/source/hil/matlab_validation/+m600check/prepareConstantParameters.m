function receipt = prepareConstantParameters(outputMat,workRoot,taskPath)
%PREPARECONSTANTPARAMETERS HOST-only once-per-build MAT generation, no devices.
% Refuses to overwrite. Generated runtime must never load this MAT itself.
assert(nargin>=1 &&strlength(string(outputMat))>0);
assert(~isfile(outputMat) &&~isfolder(outputMat), ...
    'm600check:ExistingParameterArtifact','Refusing to overwrite parameter artifact.');
if nargin==1
    fixture=m600check.loadFixture();
elseif nargin==2
    fixture=m600check.loadFixture(workRoot);
else
    fixture=m600check.loadFixture(workRoot,taskPath);
end
parameters=fixture.parameters;
loaded=load(fixture.task_path,'physicalTask');
r=loaded.physicalTask.reference;
jetUp=[double(r.position_m(1,:)).';double(r.velocity_mps(1,:)).'; ...
    double(r.acceleration_mps2(1,:)).';double(r.jerk_mps3(1,:)).'];
jetNed=jetUp;
for k=0:3
    jetNed(3*k+(1:3))=jetNed(3*k+(1:3)).*[1;1;-1];
end
environmentExample=struct('reference_jet_ned',jetNed, ...
    'payload_kg',fixture.initial_payload_kg, ...
    'wind_xy_mps',double(r.actual_wind_xy_mps(1,:)).');
provenance=struct('source_manifest_sha256',fixture.source_manifest_sha256, ...
    'task_fixture_path',fixture.task_path,'task_fixture_sha256',fixture.task_sha256, ...
    'parameter_environment_role','NUMERICAL_FIXTURE_ONLY__NOT_LIVE_MISSION_CONFIGURATION', ...
    'fixed_step_s',0.01, ...
    'hardware_actions',0,'runtime_file_io',false);
parent=fileparts(outputMat);
assert(isfolder(parent),'m600check:ParameterDirectory','Create an isolated output directory first.');
save(outputMat,'parameters','environmentExample','provenance','-v7');
info=dir(outputMat);
receipt=struct('status','HOST_PARAMETER_ARTIFACT_CREATED_NOT_CODEGEN_VALIDATED', ...
    'path',string(outputMat),'bytes',info.bytes, ...
    'sha256',gpenmpcNative.fileSha256(outputMat),'provenance',provenance);
end
