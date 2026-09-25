function [contracts,proof]=load_m600_recovery_contracts()
% Load the recorded recovery parameters.
run=getenv('HIL_RECOVERY_RECORDS');
assert(~isempty(run)&&isfolder(run), ...
    'gpenmpcShort:RecoveryRecords','Configure HIL_RECOVERY_RECORDS with the recovery record directory.');
planSha='054B621899393A74F7A515C04FFC1FFF5FCB8943B30BB7CDD4030B5452ABA47E';
[manifest,m]=readExact(fullfile(run,'CONTENT_MANIFEST.json'),'97DEC48D91951A02BE4F6454B973AE9E20BE1D2B08F388F601DFA796CDBD414F');
[outer,o]=readExact(fullfile(run,'OUTER_RESULT.json'),'EA791258BF280E24A5499D9516B40D71138E66A2D51A90A7B628C4DF86F7A0FC');
planPath=fullfile(run,'PLAN.json');
if ~isfile(planPath),planPath=manifest.source_plan.path;end
[plan,p]=readExact(planPath,planSha);
assert(manifest.source_plan.bytes==p.bytes ...
    &&strcmpi(manifest.source_plan.sha256,planSha)&&strcmpi(outer.plan_sha256,planSha), ...
    'gpenmpcShort:ReferenceApplicationPlan','Original run manifest, result and actual plan must agree.');
p.recorded_path=manifest.source_plan.path;
assert(strcmp(plan.execution_kind,'CANONICAL_DELIVERY') ...
    &&numel(plan.temporary_allocator_geometry.entries)==12&&numel(plan.native_hover_tuning.entries)==3, ...
    'gpenmpcShort:ReferenceApplicationRecovery','The actual original recovery contracts are required.');
contracts=struct('temporary_allocator_geometry',plan.temporary_allocator_geometry, ...
    'native_hover_tuning',plan.native_hover_tuning);
entries=[contracts.temporary_allocator_geometry.entries(:);contracts.native_hover_tuning.entries(:)];
assert(numel(unique(string({entries.name})))==15&&all([entries.mav_type]==9), ...
    'gpenmpcShort:ReferenceApplicationEntries','Original fifteen distinct REAL32 entries required.');
proof=struct('schema','M600_REFERENCE_APPLICATION_RECOVERY_CONTRACT_SOURCE_V1','plan',p, ...
    'run_manifest',m,'outer_result',o,'original_entries',entries, ...
    'current_referenced_files_revalidated',false, ...
    'parameter_actions',0,'hardware_actions',0, ...
    'scope','HISTORICAL_NATIVE_LAND_RECOVERY');
end
function [value,proof]=readExact(path,wanted)
f=fopen(path,'rb');assert(f>=0,'gpenmpcShort:ReferenceApplicationFile','Cannot read original file: %s',path);
c=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');clear c
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(bytes,'int8'));
sha=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
assert(strcmpi(sha,wanted),'gpenmpcShort:ReferenceApplicationSha','Original source changed: %s',path);
value=jsondecode(native2unicode(bytes.','UTF-8'));
proof=struct('path',path,'bytes',numel(bytes),'sha256',sha);
end
