function report=export_local_application_profile(outputRoot)
% Export the five-leg application profile; reference samples and offsets remain in RWW.
build=string(fileparts(fileparts(mfilename('fullpath'))));
old=path;cleanup=onCleanup(@()path(old)); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
taskPath=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);
assert(~isfolder(outputRoot));mkdir(outputRoot);
duration=zeros(5,1);nominal=zeros(5,1);
for leg=1:5
    if leg==1
        [tr,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{leg},zeros(3,1));
    else
        s=struct('capture_count',leg-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',zeros(4,3),'current_vertical_frame_offset_m',0);
        [tr,~]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(b.legs{leg},s);
    end
    duration(leg)=tr.total_duration_s;nominal(leg)=b.legs{leg}.trajectory.total_duration_s;
end
values=[duration;a.enmpc.phase_rate_min;a.enmpc.phase_rate_max;a.enmpc.reference_transition_jerk_limit_mps3;0.01];
assert(all(isfinite(values))&&all(duration>0));
lines=["#pragma once";"// Exported by the actual passport-bound MATLAB chain. No HOST fixture state."; ...
    "namespace gpenmpc_local_profile {";"constexpr double leg_duration_s[5]={"+strjoin(arrayfun(@literal,duration),",")+"};"; ...
    "constexpr double phase_rate_min="+literal(a.enmpc.phase_rate_min)+";"; ...
    "constexpr double phase_rate_max="+literal(a.enmpc.phase_rate_max)+";"; ...
    "constexpr double reference_jerk_limit="+literal(a.enmpc.reference_transition_jerk_limit_mps3)+";"; ...
    "constexpr double initial_dt_s=0.01;"; ...
    "constexpr unsigned char task_sha256[32]={"+hexArray(taskSha)+"};"; ...
    "constexpr unsigned char configuration_sha256[32]={"+hexArray(a.configurationBinding.effective_configuration_payload_sha256)+"};"; ...
    "constexpr unsigned char configuration_binding_sha256[32]={"+hexArray(sha(fullfile(a.packageRoot,'binding','runtime_binding.json')))+"};"; ...
    "}"];
header=fullfile(outputRoot,'CanonicalLocalDeploymentProfile.hpp');
f=fopen(header,'w','n','UTF-8');assert(f>=0);fprintf(f,'%s\n',lines);fclose(f);
report=struct('status','PASS_AUTHORITATIVE_SCALAR_PROFILE_EXPORT_ONLY','task_sha256',taskSha, ...
    'configuration_payload_sha256',a.configurationBinding.effective_configuration_payload_sha256, ...
    'nominal_duration_s',nominal,'bound_duration_s',duration,'phase_rate_min',a.enmpc.phase_rate_min, ...
    'phase_rate_max',a.enmpc.phase_rate_max,'reference_transition_jerk_limit_mps3',a.enmpc.reference_transition_jerk_limit_mps3, ...
    'header',header,'header_sha256',sha(header),'export_source_sha256',sha(mfilename('fullpath')+".m"), ...
    'method_binding',a.binding,'hardware_actions',0,'scientific_ticks',0);
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
fprintf('LOCAL_APPLICATION_PROFILE five actual legs exported; hardware=0\n');
end
function x=literal(v)
x=string(sprintf('%.17g',v));if ~contains(x,[".","e","E"]),x=x+".0";end
assert(isequal(typecast(str2double(x),'uint64'),typecast(double(v),'uint64')));
end
function x=hexArray(s),s=char(s);x=strjoin("0x"+string(cellstr(reshape(s,2,[]).')),",");end
function s=sha(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));s=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
