function a=loadCanonicalAssets(packageRoot)
% LOADCANONICALASSETS Load and verify the bound canonical method assets.
% Leave the verified MATLAB paths active on success.
arguments
    packageRoot (1,1) string = ""
end
distributionRoot=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
fixedRoot=fullfile(distributionRoot,'assets','canonical');
if strlength(packageRoot)==0,packageRoot=fixedRoot;end
packageRoot=canonicalPath(packageRoot);
require(strcmpi(packageRoot,canonicalPath(fixedRoot)), ...
    'Root','Only the assets bound to this distribution are accepted.');
guardRoot=fullfile(packageRoot,'binding');
passportPath=fullfile(guardRoot,'execution.json');
assertHash(passportPath,'AA2A2C8F2282A4B41D06FB4AF042FF157B5322AA7040FDCA92123935615B00AE');
passport=jsondecode(fileread(passportPath));
require(strcmp(passport.schema,'GPENMPC_EXECUTION_BINDING_V1')&& ...
    strcmpi(canonicalPath(fullfile(packageRoot,string(passport.package_root))),packageRoot), ...
    'Passport','The canonical execution passport is inconsistent.');
requirementsPath=fullfile(guardRoot,'runtime_requirements.json');
assertHash(requirementsPath,'1B09A0DB83A5AA4A1C4C9DCD6DBFF51A74DEE178C04F3B00DF8AA8AB789A7056');
expected=passport.expected_sha256;
sourceRoot=containedPath(packageRoot,passport.canonical_paths.source_root);
sourceProjectRoot=containedPath(packageRoot,passport.canonical_paths.source_project_root);
manifestPath=containedPath(packageRoot,passport.canonical_paths.source_manifest);
assertHash(manifestPath,expected.canonical_source_manifest);
manifest=jsondecode(fileread(manifestPath));
paths=string({manifest.entries.path});
selected=manifest.entries(startsWith(paths,'implementation/')&~endsWith(paths,'.pyc'));
require(passport.source_entry_count==160&&numel(selected)==160, ...
    'SourceCount','Canonical source must have exactly 160 entries.');
entries=struct('path',{},'bytes',{},'sha256',{},'manifest_relative_path',{});
for k=1:numel(selected)
    relative=extractAfter(string(selected(k).path),strlength('implementation/'));
    target=containedPath(sourceRoot,relative);
    assertArtifact(target,selected(k).bytes,selected(k).sha256);
    entries(end+1)=struct('path',char(target),'bytes',selected(k).bytes, ...
        'sha256',char(upper(string(selected(k).sha256))), ...
        'manifest_relative_path',char(selected(k).path)); %#ok<AGROW>
end
require(numel(unique(lower(string({entries.path}))))==160, ...
    'SourceDuplicate','Canonical source contains duplicate paths.');
actual=dir(fullfile(sourceRoot,'**','*'));actual=actual(~[actual.isdir]);
actualPaths=string(fullfile({actual.folder},{actual.name}));
actualPaths=actualPaths(~endsWith(actualPaths,'.pyc'));
require(isequal(sort(lower(actualPaths(:))),sort(lower(string({entries.path}).'))), ...
    'UnexpectedSource','Canonical source has missing or unexpected ordinary files.');

gpPath=containedPath(packageRoot,passport.canonical_paths.gp_model);
configurationPath=containedPath(packageRoot,passport.canonical_paths.effective_config_mat);
bindingPath=containedPath(packageRoot,passport.canonical_paths.effective_config_binding);
assertHash(gpPath,expected.gp_model);
assertHash(configurationPath,expected.effective_configuration_mat);
assertHash(bindingPath,expected.effective_configuration_binding);
configurationBinding=jsondecode(fileread(bindingPath));
require(strcmpi(configurationBinding.effective_configuration_payload_sha256, ...
    expected.effective_configuration_payload)&& ...
    strcmpi(configurationBinding.selected_source_manifest_sha256,expected.canonical_source_manifest)&& ...
    strcmpi(configurationBinding.gp_model_sha256,expected.gp_model), ...
    'EffectiveBinding','Effective configuration/source/GP binding differs from passport.');

previousPath=path;successful=java.util.concurrent.atomic.AtomicBoolean(false);
pathGuard=onCleanup(@()restoreFailedPath(previousPath,successful)); %#ok<NASGU>
addpath(genpath(sourceRoot),'-begin');rehash path;
resolved=struct('name',{},'path',{});
for k=1:numel(entries)
    [~,name,extension]=fileparts(entries(k).path);
    if ~strcmpi(extension,'.m'),continue;end
    found=string(which(name));
    require(strlength(found)>0&&strcmpi(canonicalPath(found),entries(k).path), ...
        'SourceShadow',sprintf('Non-canonical MATLAB function resolution: %s -> %s',name,found));
    resolved(end+1)=struct('name',name,'path',char(found)); %#ok<AGROW>
end

% Execute the required asset-validation chain.
a=gpenmpcLoadNativeEnmpcComparisonAssets(sourceProjectRoot,0.30);
require(strcmpi(a.gp_model_sha256,expected.gp_model), ...
    'LoadedGp','The MATLAB loader selected a different GP model.');
require(strcmpi(canonicalPath(a.gp_model_path), ...
    fullfile(sourceRoot,'native_training','raw','MATLAB_NATIVE_SPARSE_GP_MODEL.mat')), ...
    'LoadedGpPath','The active GP path is outside the canonical source.');
a.enmpc=gpenmpcApplyArchitectureConfiguration(a.enmpc,a.robust,'A1_COORDINATED_PHYSICAL');
a.enmpc=gpenmpcApplyCommandContinuityConfiguration(a.enmpc);
stored=load(configurationPath,'effectiveRuntimeConfiguration');
require(isfield(stored,'effectiveRuntimeConfiguration'),'ConfigurationMissing', ...
    'The canonical MAT file lacks effectiveRuntimeConfiguration.');
frozen=stored.effectiveRuntimeConfiguration;
names=fieldnames(a.enmpc);
require(isequal(sort(names),sort(fieldnames(frozen))), ...
    'ConfigurationFields','Reconstructed and frozen configuration fields differ.');
differences=strings(0,1);
for k=1:numel(names)
    if ~isequaln(a.enmpc.(names{k}),frozen.(names{k}))
        differences(end+1,1)=string(names{k}); %#ok<AGROW>
    end
end
require(all(differences=="source_path"),'ConfigurationValues', ...
    sprintf('Non-portability configuration differences: %s',strjoin(differences,', ')));
normalized=a.enmpc;normalized.source_path=frozen.source_path;
normalizedSha=shaJson(normalized);
require(strcmpi(normalizedSha,expected.effective_configuration_payload), ...
    'ConfigurationPayload','Path-normalized effective configuration SHA mismatch.');
assertRuntimeConfiguration(a.enmpc,passport);

a.sourceRoot=char(sourceRoot);
a.packageRoot=char(packageRoot);
a.binding_path=char(passportPath);
a.configurationMat=stored;
a.configurationBinding=configurationBinding;
a.gpModelPath=char(gpPath);
a.prediction_context=gpenmpcNativeEnmpcPredictionContext(a);
a.attitudeContinuity=struct('enabled',true,'time_constant_s',0.08, ...
    'maximum_angular_velocity_rad_s',1.5,'maximum_angular_acceleration_rad_s2',8.0, ...
    'maximum_angular_jerk_rad_s3',50.0,'maximum_continuous_dt_s',0.05);
allocation=gpenmpcM600Allocation(a.calibration);c=a.calibration.controller;
a.kernelParameters=struct('kp',double(c.position_gain_s2(:)), ...
    'kd',double(c.velocity_gain_s(:)),'kr',double(c.attitude_moment_gain_nm_per_rad(:)), ...
    'kw',double(c.body_rate_moment_gain_nm_s_per_rad(:)), ...
    'drag',allocation.nominal_drag_n_per_mps2,'inertia',allocation.inertia_kg_m2, ...
    'pseudoinverse',allocation.pseudoinverse, ...
    'baseMass',double(a.profile.mass_properties.base_mass_kg), ...
    'totalThrust',allocation.total_thrust_upper_n,'rotorUpper',allocation.per_rotor_upper_n, ...
    'maxTilt',allocation.maximum_tilt_rad);
a.binding=struct('schema','CANONICAL_E_PACKAGE_RUNTIME_SOURCE_BINDING_V1', ...
    'package_root',char(packageRoot),'source_root',char(sourceRoot), ...
    'binding_path',char(passportPath), ...
    'source_manifest_path',char(manifestPath), ...
    'source_manifest_sha256',char(upper(string(expected.canonical_source_manifest))), ...
    'entries',entries,'verified_source_entries',160, ...
    'passport_sha256',char(fileSha(passportPath)), ...
    'effective_configuration_payload_sha256',char(normalizedSha), ...
    'runtime_configuration_payload_sha256',char(shaJson(a.enmpc)), ...
    'configuration_difference_fields',differences, ...
    'source_path_only_difference',all(differences=="source_path"), ...
    'configuration_chain',{passport.required_configuration_chain}, ...
    'resolved_functions',resolved, ...
    'gp_model_sha256',char(upper(string(expected.gp_model))), ...
    'hardware_actions',0,'scientific_ticks',0,'outer_solve_calls',0, ...
    'claim','Canonical assets and effective configuration loaded.');
successful.set(true);
end

function restoreFailedPath(previousPath,successful)
if ~successful.get(),path(previousPath);end
end

function assertRuntimeConfiguration(c,p)
r=p.required_runtime_assertions;t=p.timing;
require(strcmp(c.method,p.runtime_method_id)&&~p.ordinary_b2&&~p.prediction_only_gp, ...
    'Method','Only the final coordinated physical method is admitted.');
require(c.coordinated_architecture_enabled==r.coordinated_architecture_enabled&& ...
    strcmp(c.coordinated_architecture_mode,r.coordinated_architecture_mode)&& ...
    c.coordinated_physical_gp_application_enabled==r.coordinated_physical_gp_application_enabled&& ...
    c.coordinated_gp_execution_feedforward_enabled==r.coordinated_gp_execution_feedforward_enabled&& ...
    c.gp_responsibility_gate_enabled==r.gp_responsibility_gate_enabled&& ...
    c.command_continuity_enabled==r.command_continuity_enabled, ...
    'Architecture','The coordinated physical control architecture is not active.');
observed=[c.outer_period_s,c.prediction_step_s,c.horizon_steps,c.prediction_horizon_s, ...
    c.solver_deadline_s,c.coordinated_risk_equivalence_tolerance, ...
    c.coordinated_tracking_equivalence_tolerance,c.coordinated_objective_relative_switch_improvement];
expected=[t.outer_update_period_s,t.prediction_step_s,t.horizon_steps,t.prediction_horizon_s, ...
    t.solver_deadline_s,r.risk_equivalence_tolerance,r.tracking_equivalence_tolerance, ...
    r.objective_relative_switch_improvement];
require(isequal(observed,expected),'Timing','Timing or selector values differ from passport.');
end

function target=containedPath(root,relative)
target=canonicalPath(fullfile(root,strrep(string(relative),'/',filesep)));
require(startsWith(lower(target),lower(canonicalPath(root)+filesep)), ...
    'PathEscape','Manifest path escapes its canonical root.');
end
function value=canonicalPath(value)
value=string(char(java.io.File(char(value)).getCanonicalPath()));
end
function assertArtifact(p,bytes,sha)
f=dir(p);require(isscalar(f)&&~f.isdir&&f.bytes==bytes,'SourceBytes', ...
    sprintf('Missing/changed canonical bytes: %s',p));assertHash(p,sha);
end
function assertHash(p,sha)
require(isfile(p),'Missing',sprintf('Canonical identity file is missing: %s',p));
require(strcmpi(fileSha(p),string(sha)),'Hash',sprintf('Canonical SHA mismatch: %s',p));
end
function value=fileSha(p)
fid=fopen(p,'rb');require(fid>=0,'Read',sprintf('Cannot read canonical file: %s',p));
guard=onCleanup(@()fclose(fid));engine=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),engine.update(fread(fid,1048576,'*uint8'));end
value=upper(string(reshape(dec2hex(typecast(engine.digest(),'uint8'),2).',1,[])));
clear guard
end
function value=shaJson(payload)
engine=java.security.MessageDigest.getInstance('SHA-256');
engine.update(uint8(unicode2native(jsonencode(payload),'UTF-8')));
value=upper(string(reshape(dec2hex(typecast(engine.digest(),'uint8'),2).',1,[])));
end
function require(yes,suffix,message)
if ~yes,error(['gpenmpcNative:Canonical' suffix],'%s',message);end
end
