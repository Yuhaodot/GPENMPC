function report=inspect_rfly_controller_template(outputDir,alignInstalledBundle)
% Inspect installed vendor blocks in a disabled harness.
arguments
    outputDir (1,1) string
    alignInstalledBundle (1,1) logical = false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(startsWith(outputDir,fullfile(build,'evidence')+filesep));
assert(~isfolder(outputDir),'Choose an unused output path.');mkdir(outputDir);
oldPath=path;oldDir=pwd;g=onCleanup(@()restoreSession(oldPath,oldDir)); %#ok<NASGU>
cd(outputDir);
sdk=gpenmpc_install_path('rfly','RflySimAPIs\RflySimSDK\simulink');
psp=gpenmpc_install_path('rfly','PX4 PSP\code');
if alignInstalledBundle
    % Assemble vendor files within the project.
    localPsp=fullfile(outputDir,'official_psp_local');copyfile(psp,localPsp);
    target=fullfile(localPsp,'+codertarget','+pixhawk','+blocks','+Block_Callbacks','+uORB');
    update=gpenmpc_install_path('rfly','PSPUpdate');
    names={'uORB_msgToBus.m','uORB_ParseMsgFile.m','uORB_ParseMsgFile_dai.m', ...
        'ParamUpdateMaskDialog_uORB_Write.m','ParamUpdateMaskDialog_uORB_Write_dai.m'};
    for k=1:numel(names),copyfile(fullfile(update,'1.14',names{k}),fullfile(target,names{k}),'f');end
    copyfile(fullfile(update,'sfun_px4_uorb_write_dai.mexw64'),fullfile(localPsp,'blocks'));
    copyfile(fullfile(update,'sfun_px4_uorb_write_dai.tlc'),fullfile(localPsp,'blocks'));
    schemaRoot=fullfile(outputDir,'official_schema_context');mkdir(fullfile(schemaRoot,'Firmware','msg'));
    px4=gpenmpc_external_path('px4_source_root');
    pairs={'ActuatorArmed.msg','ActuatorArmed.msg';'ActuatorOutputs.msg','ActuatorOutputs.msg'; ...
        fullfile('versioned','VehicleStatus.msg'),'VehicleStatus.msg'; ...
        fullfile('versioned','VehicleCommand.msg'),'VehicleCommand.msg'};
    for k=1:size(pairs,1)
        copyfile(fullfile(px4,'msg',pairs{k,1}),fullfile(schemaRoot,'Firmware','msg',pairs{k,2}));
    end
    % Supply the path fields used by mask parsers.
    Px4PSP_CmakeInfo=struct('Px4_Base_Dir',char(schemaRoot), ...
        'Px4_Build_Dir',char(fullfile(outputDir,'NOT_BUILT'))); %#ok<NASGU>
    save(fullfile(localPsp,'+codertarget','+pixhawk','+CMAKE_Utils','CmakeInfo.mat'),'Px4PSP_CmakeInfo');
    psp=localPsp;
end
addpath(psp,fullfile(psp,'blocks'),fullfile(psp,'Work'),sdk,fullfile(sdk,'114'),'-begin');
report=struct('status','IN_PROGRESS','hardware_actions',0,'COM_open',0, ...
    'model_simulation',0,'code_generation',0,'firmware_build',0,'automatic_commands_enabled',false);
report.local_official_bundle_assembly=alignInstalledBundle;
mdl="GPENMPC_Rfly_DisabledOutput";
try
    library=fullfile(sdk,'114','pixhawk_slib_rfly.slx');
    report.library_path=library;report.library_sha256=sha(library);
    load_system(library);new_system(mdl);
    set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.01', ...
        'StopTime','0','SystemTargetFile','ert.tlc');
    b=add_block('pixhawk_slib_rfly/HIL16CtrlsNorm',mdl+"/OfficialVirtualOutput", ...
        'isAutoArm','off','isAutoBlock','off','isAutoLoiter','off','SamTime','0.01');
    add_block('simulink/Sources/Constant',mdl+"/Disabled",'Value','false','OutDataTypeStr','boolean');
    add_block('simulink/Sources/Constant',mdl+"/ZeroControls",'Value','zeros(16,1)','OutDataTypeStr','single');
    add_line(mdl,'Disabled/1','OfficialVirtualOutput/1');
    add_line(mdl,'ZeroControls/1','OfficialVirtualOutput/2');
    report.mask_parameters=struct('isAutoArm',get_param(b,'isAutoArm'), ...
        'isAutoBlock',get_param(b,'isAutoBlock'),'isAutoLoiter',get_param(b,'isAutoLoiter'), ...
        'SamTime',get_param(b,'SamTime'));
    assert(strcmp(report.mask_parameters.isAutoArm,'off')&& ...
        strcmp(report.mask_parameters.isAutoBlock,'off')&&strcmp(report.mask_parameters.isAutoLoiter,'off'));
    blocks=find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block');
    detail=struct('path',{},'block_type',{},'source',{},'mask_type',{},'mask_names',{},'mask_values',{},'user_data',{});
    for k=1:numel(blocks)
        p=blocks{k};source=get_param(p,'ReferenceBlock');
        detail(end+1)=struct('path',p,'block_type',get_param(p,'BlockType'), ...
            'source',source,'mask_type',get_param(p,'MaskType'), ...
            'mask_names',{get_param(p,'MaskNames')},'mask_values',{get_param(p,'MaskValues')}, ...
            'user_data',get_param(p,'UserData')); %#ok<AGROW>
    end
    save(fullfile(outputDir,'ACTUAL_BLOCK_DATA.mat'),'detail','-v7');
    write(fullfile(outputDir,'ACTUAL_BLOCK_DATA.txt'),evalc('dispDetails(detail)'));
    report.physical_pwm_block_count=sum(contains(string({detail.source}),'/PWM_output') ...
        |contains(string({detail.source}),'/Aux_output'));
    assert(report.physical_pwm_block_count==0);
    save_system(mdl,fullfile(outputDir,mdl+'.slx'));
    report.model_path=fullfile(outputDir,mdl+'.slx');
    report.model_sha256=sha(report.model_path);
    report.update_diagram_pass=false;
    try
        set_param(mdl,'SimulationCommand','update');
        report.update_diagram_pass=true;
    catch e
        report.update_failure=getReport(e,'extended','hyperlinks','off');
    end
    report.status='VENDOR_BLOCK_LOADED_DISABLED_HOST_HARNESS';
catch e
    report.status='HOST_TEMPLATE_LOAD_ERROR';report.failure=getReport(e,'extended','hyperlinks','off');
end
write(fullfile(outputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));disp(jsonencode(report));
if bdIsLoaded(mdl),close_system(mdl,0);end
end
function dispDetails(d)
for k=1:numel(d)
    fprintf('\n%s\nType: %s\nSource: %s\n',d(k).path,d(k).block_type,d(k).source);
    disp(d(k).mask_names);disp(d(k).mask_values);disp(d(k).user_data);
end
end
function restoreSession(p,d),path(p);cd(d);end
function write(p,s),f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));fprintf(f,'%s\n',s);end
function s=sha(p)
f=fopen(p,'r');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
v=java.security.MessageDigest.getInstance('SHA-256');v.update(b);s=upper(reshape(dec2hex(typecast(v.digest(),'uint8'),2).',1,[]));
end
