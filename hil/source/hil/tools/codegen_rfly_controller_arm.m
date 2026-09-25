function report=codegen_rfly_controller_arm(outputDir,officialIO)
% Generate code from the Simulink controller model without invoking deployment hooks.
arguments
    outputDir (1,1) string
    officialIO (1,1) logical = false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(startsWith(outputDir,fullfile(build,'evidence')+filesep)&&~isfolder(outputDir));mkdir(outputDir);
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
oldPath=path;oldDir=pwd;g=onCleanup(@()restore(oldPath,oldDir)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-end');addpath(fullfile(build,'host_runtime'),'-begin');cd(outputDir);
prior=fullfile(gpenmpc_external_path('rfly_canonical_controller'));
passed=jsondecode(fileread(fullfile(prior,'RESULT.json')));
assert(passed.controller_codegen_pass&&passed.official_encoding_byte_equal&&passed.replay_rows==2129);
model="GPENMPC_Rfly_Canonical_Controller";
if officialIO
    model="GPENMPC_Rfly_Canonical_OfficialIO";
    vendor=fullfile(gpenmpc_external_path('rfly_vendor_interface'),'official_psp_local');
    sdk=gpenmpc_install_path('rfly','RflySimAPIs\RflySimSDK\simulink');
    addpath(vendor,fullfile(vendor,'blocks'),fullfile(vendor,'Work'),sdk,fullfile(sdk,'114'),'-begin');
end
copyfile(fullfile(prior,model+'.slx'),fullfile(outputDir,model+'.slx'));
load_system(fullfile(outputDir,model+'.slx'));mg=onCleanup(@()closeOwned(model)); %#ok<NASGU>
report=struct('status','IN_PROGRESS','hardware_actions',0,'COM_open',0,'firmware_build',0,'flash',0, ...
    'official_IO_connected',officialIO);
try
    set_param(model,'ProdHWDeviceType','ARM Compatible->ARM Cortex-M', ...
        'GenCodeOnly','on','GenerateReport','off');
    report.production_hardware=get_param(model,'ProdHWDeviceType');
    if officialIO
        b=model+"/OfficialVirtualOutput";
        assert(strcmp(get_param(b,'isAutoArm'),'off')&&strcmp(get_param(b,'isAutoBlock'),'off') ...
            &&strcmp(get_param(b,'isAutoLoiter'),'off'));
    end
    save_system(model);slbuild(model);
    generated=fullfile(outputDir,model+'_ert_rtw');
    source=fileread(fullfile(generated,model+'.c'));
    assert(~contains(source,'emmintrin.h')&&~contains(source,'__m128'));
    report.no_x86_intrinsics=true;report.generated_source=fullfile(generated,model+'.c');
    report.source_sha256=sha(report.generated_source);
    report.status='PASS_ARM_TARGET_SIMULINK_CODEGEN__NOT_PX4_APPLICATION';
catch e
    report.status='HOST_ARM_CODEGEN_IMPLEMENTATION_FAILURE';report.failure=getReport(e,'extended','hyperlinks','off');
end
f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);fg=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));disp(jsonencode(report));
end
function restore(p,d),path(p);cd(d);end
function closeOwned(m),if bdIsLoaded(m),close_system(m,0);end,end
function s=sha(p)
f=fopen(p,'r');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
v=java.security.MessageDigest.getInstance('SHA-256');v.update(b);s=upper(reshape(dec2hex(typecast(v.digest(),'uint8'),2).',1,[]));
end
