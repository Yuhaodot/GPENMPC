function report=build_rfly_canonical_controller_model(outputDir)
% Replay the controller model and connect its vendor IO blocks.
arguments
    outputDir (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
assert(startsWith(outputDir,fullfile(build,'evidence')+filesep)&&~isfolder(outputDir));mkdir(outputDir);
oldPath=path;oldDir=pwd;clean=onCleanup(@()restore(oldPath,oldDir)); %#ok<NASGU>
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
vendor=fullfile(gpenmpc_external_path('rfly_vendor_interface'),'official_psp_local');
sdk=gpenmpc_install_path('rfly','RflySimAPIs\RflySimSDK\simulink');
addpath(fullfile(parent,'src'),'-end');addpath(fullfile(build,'host_runtime'), ...
    vendor,fullfile(vendor,'blocks'),fullfile(vendor,'Work'),sdk,fullfile(sdk,'114'),'-begin');
cd(outputDir);mdl="GPENMPC_Rfly_Canonical_Controller";new_system(mdl);
g=onCleanup(@()closeOwned(mdl)); %#ok<NASGU>
report=struct('status','IN_PROGRESS','hardware_actions',0,'COM_open',0,'UDP_open',0, ...
    'firmware_build',0,'flash',0,'CopterSim_started',0,'live_HIL',false);
try
    set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.01', ...
        'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset', ...
        'ReturnWorkspaceOutputs','on','SystemTargetFile','ert.tlc');
    for k=1:3
        names={'KernelArguments101','ContinuityEnabled','InputGenerationAccepted'};
        dims={'101','1','1'};types={'double','boolean','boolean'};
        add_block('simulink/Sources/In1',mdl+"/"+names{k},'Port',num2str(k), ...
            'PortDimensions',dims{k},'OutDataTypeStr',types{k},'SampleTime','0.01');
    end
    block=mdl+"/CanonicalControllerAndOfficialEncoding";
    add_block('simulink/User-Defined Functions/MATLAB Function',block);
    chart=find(sfroot,'-isa','Stateflow.EMChart','Path',block);
    chart.Script=sprintf(['function [u16,y61,valid]=fcn(u,continuity,enable)\n' ...
        '%%#codegen\n[u16,y61,valid]=gpenmpcNative.rflyCanonicalKernelBlock(u,continuity,enable);\nend\n']);
    for k=1:3
        names={'KernelArguments101','ContinuityEnabled','InputGenerationAccepted'};
        add_line(mdl,string(names{k})+"/1","CanonicalControllerAndOfficialEncoding/"+k);
        out={'Controls16','FullKernel61','OutputValid'};
        add_block('simulink/Sinks/Out1',mdl+"/"+out{k},'Port',num2str(k));
        add_line(mdl,"CanonicalControllerAndOfficialEncoding/"+k,string(out{k})+"/1");
    end
    fixture=fullfile(build,'evidence','canonical_full_se3_kernel_rebinding_20260906_074245', ...
        'EXISTING_GENERATED_C_INPUT_LE.bin');
    f=fopen(fixture,'r','ieee-le');assert(f>=0);fg=onCleanup(@()fclose(f));
    n=fread(f,1,'uint32');data=fread(f,[164,n],'double');assert(numel(data)==164*n&&isempty(fread(f,1,'uint8')));clear fg
    u=data([1:34,36:102],:).';continuity=logical(data(35,:).');expected=data(103:163,:).';expectedValid=logical(data(164,:).');
    t=(0:n-1)'*.01;input=Simulink.SimulationData.Dataset;
    input=input.addElement(timeseries(u,t),'KernelArguments101');
    input=input.addElement(timeseries(continuity,t),'ContinuityEnabled');
    input=input.addElement(timeseries(true(n,1),t),'InputGenerationAccepted');
    set_param(mdl,'StopTime',num2str(t(end),17));set_param(mdl,'SimulationCommand','update');
    report.controller_update_pass=true;
    in=Simulink.SimulationInput(mdl);in=in.setExternalInput(input);
    simout=sim(in);ys=simout.yout;
    save(fullfile(outputDir,'RAW_SIMULATION_DATASET.mat'),'ys','-v7');
    assert(ys.numElements==3,'Three declared root outports required');
    controls=values(ys.getElement(1).Values,n,16);
    actual=values(ys.getElement(2).Values,n,61);
    accepted=logical(values(ys.getElement(3).Values,n,1));
    report.replay_rows=n;report.valid_rows=sum(expectedValid);
    report.maximum_full_kernel61_error=max(abs(actual-expected),[],'all');
    report.validity_equal=isequal(accepted,expectedValid);
    assert(report.maximum_full_kernel61_error<1e-10&&report.validity_equal);
    preview=zeros(n,16,'single');map=[5,1,4,6,2,3];
    preview(expectedValid,map)=single(expected(expectedValid,5:10)/32.145727009134916);
    report.official_encoding_byte_equal=isequal(single(controls),preview);assert(report.official_encoding_byte_equal);
    save(fullfile(outputDir,'ACTUAL_SIMULINK_REPLAY.mat'),'actual','expected','controls','accepted','t','-v7');
    report.simulation_runs=1;report.controller_source_sha256=sha(which('gpenmpcNative.se3WrenchKernel'));
    report.adapter_source_sha256=sha(which('gpenmpcNative.rflyCanonicalKernelBlock'));
    save_system(mdl,fullfile(outputDir,mdl+'.slx'));
    set_param(mdl,'GenCodeOnly','on','GenerateReport','off');slbuild(mdl);
    report.controller_codegen_pass=true;
    % Connect the controller block to the vendor IO graph for compilation.
    load_system(fullfile(sdk,'114','pixhawk_slib_rfly.slx'));
    add_block('pixhawk_slib_rfly/HIL16CtrlsNorm',mdl+"/OfficialVirtualOutput", ...
        'isAutoArm','off','isAutoBlock','off','isAutoLoiter','off','SamTime','0.01');
    add_line(mdl,'CanonicalControllerAndOfficialEncoding/3','OfficialVirtualOutput/1');
    add_line(mdl,'CanonicalControllerAndOfficialEncoding/1','OfficialVirtualOutput/2');
    set_param(mdl,'SimulationCommand','update');report.official_connected_update_pass=true;
    save_system(mdl,fullfile(outputDir,'GPENMPC_Rfly_Canonical_OfficialIO.slx'));
    report.official_connected_model=fullfile(outputDir,'GPENMPC_Rfly_Canonical_OfficialIO.slx');
    report.automatic_arm_mode_switch=false;report.physical_pwm_blocks=0;
    report.status='PASS_SIMULINK_CANONICAL_KERNEL_REPLAY_CODEGEN_AND_OFFICIAL_IO_CONNECTION__NOT_BOARD';
catch e
    report.status='HOST_CONTROLLER_TEMPLATE_IMPLEMENTATION_FAILURE';report.failure=getReport(e,'extended','hyperlinks','off');
end
write(fullfile(outputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));disp(jsonencode(report));
end
function y=values(ts,n,m)
d=ts.Data;if ndims(d)==3,y=reshape(d,m,n).';else,y=reshape(d,n,m);end
assert(isequal(size(y),[n,m]));
end
function restore(p,d),path(p);cd(d);end
function closeOwned(m),if bdIsLoaded(m),close_system(m,0);end,end
function write(p,s),f=fopen(p,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f));fprintf(f,'%s\n',s);end
function s=sha(p)
f=fopen(p,'r');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
v=java.security.MessageDigest.getInstance('SHA-256');v.update(b);s=upper(reshape(dec2hex(typecast(v.digest(),'uint8'),2).',1,[]));
end
