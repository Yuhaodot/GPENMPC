function report=prepare_canonical_rotor_observer_model(outputDir)
% Add typed rotor-observer outputs to the CopterSim plant.
% Update the diagram and run four 0.30 s normal Simulink tests.
% The added outputs read the same derivative result without another plant evaluation.
arguments
    outputDir (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
modelRoot=fullfile(build,'m600_coptersim');kernel=fullfile(build,'matlab_validation');
parentDir=fullfile(gpenmpc_external_path('environment_delivery_model'));
parent=fullfile(parentDir,'GPENMPC_M600_Canonical.slx');
parentParameters=fullfile(parentDir,'M600_CORE_PARAMETERS.mat');
parentSha='F8697351F25DF76D3DFA221EAAB9DC4C45B4AA970693872C4E267ECB989E561E';
parameterSha='D65290884733F07C19C54553BF8D83C703EB88AF5D0AAF110DE56C76FA767AA0';
assert(strcmpi(fileSha(parent),parentSha)&&strcmpi(fileSha(parentParameters),parameterSha), ...
    'm600check:RotorModelParentIdentity');
outputDir=string(char(java.io.File(char(outputDir)).getCanonicalPath()));
assert(startsWith(lower(outputDir),lower(build+filesep))&& ...
    ~strcmpi(outputDir,parentDir)&&~isfile(outputDir),'m600check:RotorModelNewOutput');
history={};priorRaw={};
if isfolder(outputDir)
    % Resume only an unsealed failed preparation, preserving its failure records.
    previous=jsondecode(fileread(fullfile(outputDir,'RESULT.json')));
    assert(~previous.passed&&strcmp(previous.status,'HOST_OBSERVER_MODEL_IMPLEMENTATION_FAILURE__NO_HARDWARE') ...
        &&strcmpi(previous.parent_model_sha256,parentSha),'m600check:RotorModelResumeIdentity');
    if isfield(previous,'history'),history=previous.history;previous=rmfield(previous,'history');end
    if isstruct(history),history=num2cell(history);end
    history{end+1}=previous;
    prior=load(fullfile(outputDir,'SHORT_COMPARISON_RAW.mat'));
    if isfield(prior,'priorRaw'),priorRaw=prior.priorRaw;end
    priorRaw{end+1}=prior.raw;
else
    mkdir(outputDir);
end
modelFile=fullfile(outputDir,'GPENMPC_M600_Canonical.slx');
parameterFile=fullfile(outputDir,'M600_CORE_PARAMETERS.mat');
copyfile(parent,modelFile,'f');copyfile(parentParameters,parameterFile,'f');
[ok,~]=fileattrib(modelFile,'+w');assert(ok);
oldPath=path;oldDir=pwd;
pathGuard=onCleanup(@()path(oldPath));cwdGuard=onCleanup(@()cd(oldDir)); %#ok<NASGU>
addpath(kernel,fullfile(modelRoot,'matlab_validation'),outputDir,'-begin');
m600check.loadFixture();
evalin('base',"run('"+fullfile(modelRoot,'GPENMPC_M600_CopterSim_init.m')+"')");
evalin('base','ModelInit_PosE=[0,0,0];ModelInit_AngEuler=[0,0,0];');
mdl='GPENMPC_M600_Canonical';assert(~bdIsLoaded(mdl),'m600check:RotorModelAlreadyLoaded');
modelGuard=onCleanup(@()closeOwned(mdl)); %#ok<NASGU>
sourceNames={'copterSimDeliveryCore','copterSimFlatTerrainCore','copterSimIoCore', ...
    'stepPx4Rk4','derivativePx4','derivativeSoftware','generatedPlantDerivative', ...
    'copterSimOutputs','passiveGroundDissipation','encodeCopterSimTerrainDiagnostics', ...
    'encodeCopterSimDeliveryDiagnostics'};
sources=struct('path',{},'sha256',{});
for k=1:numel(sourceNames)
    f=fullfile(kernel,'+m600check',[sourceNames{k} '.m']);
    sources(k)=struct('path',f,'sha256',fileSha(f)); %#ok<AGROW>
end
report=struct('status','IN_PROGRESS','passed',false,'failure','', ...
    'parent_model',parent,'parent_model_sha256',parentSha, ...
    'parameters',parameterFile,'parameters_sha256',parameterSha, ...
    'model',modelFile,'model_sha256','','source_sha256',{sources}, ...
    'old_chart_sha256','','new_chart_sha256','','inverse_patch_byte_identical',false, ...
    'update_diagram_pass',false,'original_core_output_count',13,'original_root_output_count',4, ...
    'new_typed_output_count',6,'test_cases',{{'zero_input','bounded_time_varying_thrust_input'}}, ...
    'simulation_duration_each_s',.30,'model_simulations_attempted',0,'model_simulations',0,'tests',{{}},'checks',false(0,1), ...
    'passed_checks',0,'total_checks',0,'parent_unchanged',false,'source_unchanged',false, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'DLL_generated',false, ...
    'DLL_SHA_embedded',false,'live_transport_connected',false, ...
    'claim','Rotor-observer model prepared.','history',{history});
raw=struct('parent',{{}},'derived',{{}});names={};checks=false(0,1);
try
    % Run simulations from the output directory to keep caches outside the parent model.
    cd(outputDir);load_system(parent);
    assert(strcmpi(get_param(mdl,'FileName'),char(parent)),'m600check:RotorLoadedParentPath');
    original=getCore(mdl);oldScript=char(original.Script);report.old_chart_sha256=textSha(oldScript);
    assert(numel(get_param([mdl '/CurrentM600_10ms'],'PortHandles').Outport)==13);
    instrumentOriginal13(mdl);
    for scenario=1:2
        report.model_simulations_attempted=report.model_simulations_attempted+1;
        raw.parent{scenario}=runShort(mdl,scenario);report.model_simulations=report.model_simulations+1;
    end
    close_system(mdl,0);
    cd(outputDir);load_system(modelFile);
    assert(strcmpi(get_param(mdl,'FileName'),char(modelFile)),'m600check:RotorLoadedDerivedPath');
    core=getCore(mdl);
    signature=['animation,diagnostic] = fcn(u,pos0,ang0,terrain,environmentFrame,ioTime)'];
    newSignature=['animation,diagnostic,rotorObserverStateN,rotorObserverSimTimeS,' ...
        'rotorObserverGeneration,rotorObserverSession,rotorObserverValid,rotorObserverFailed]' ...
        ' = fcn(u,pos0,ang0,terrain,environmentFrame,ioTime)'];
    assert(count(string(oldScript),signature)==1,'m600check:RotorChartSignature');
    insertion=sprintf(['%% Observer-only taps: SAME accepted core state; no new integrator/clock.\n' ...
        'rotorObserverStateN=y.rotor_thrust_software_order_n;\n' ...
        'rotorObserverSimTimeS=d.sim_time_s;\n' ...
        'rotorObserverGeneration=uint64(d.plant_step_count);\n' ...
        'rotorObserverSession=uint64(data.deliveryPolicy.expected_session_token);\n' ...
        'rotorObserverValid=logical(d.step_accepted && d.observation_valid && ~d.failed);\n' ...
        'rotorObserverFailed=logical(d.failed);\n']);
    finalEnd=regexp(oldScript,'(?m)^end\s*$','start');assert(~isempty(finalEnd));at=finalEnd(end);
    assert(isempty(strtrim(oldScript(at+3:end))),'m600check:RotorChartFinalEnd');
    newScript=[oldScript(1:at-1) insertion oldScript(at:end)];
    newScript=strrep(newScript,signature,newSignature);
    inverse=strrep(strrep(newScript,newSignature,signature),insertion,'');
    check('original_chart_body_inverse_patch_byte_identical',strcmp(inverse,oldScript));
    report.inverse_patch_byte_identical=strcmp(inverse,oldScript);
    core.Script=newScript;
    outputNames={'rotorObserverStateN','rotorObserverSimTimeS','rotorObserverGeneration', ...
        'rotorObserverSession','rotorObserverValid','rotorObserverFailed'};
    outputTypes={'double','double','uint64','uint64','boolean','boolean'};
    rootNames={'RotorObserverStateN','RotorObserverSimTimeS','RotorObserverGeneration', ...
        'RotorObserverSession','RotorObserverValid','RotorObserverFailed'};
    for k=1:6
        item=core.find('-isa','Stateflow.Data','Name',outputNames{k});assert(isscalar(item));
        item.DataType=outputTypes{k};item.Props.Array.Size='1';
        if k==1,item.Props.Array.Size='6';end
        add_block('simulink/Sinks/Out1',[mdl '/' rootNames{k}], ...
            'Port',num2str(k+4),'OutDataTypeStr',outputTypes{k});
        add_line(mdl,sprintf('CurrentM600_10ms/%d',k+13),[rootNames{k} '/1']);
    end
    set_param(mdl,'SimulationCommand','update');report.update_diagram_pass=true;
    check('nineteen_core_outputs',numel(get_param([mdl '/CurrentM600_10ms'],'PortHandles').Outport)==19);
    check('old_chart_has_exactly_one_core_call',count(string(oldScript),'m600check.copterSimDeliveryCore(')==1);
    check('derived_chart_has_exactly_one_core_call',count(string(core.Script),'m600check.copterSimDeliveryCore(')==1);
    check('derived_chart_exact_delta',strcmp(core.Script,newScript));
    report.new_chart_sha256=textSha(newScript);
    save_system(mdl,modelFile);report.model_sha256=fileSha(modelFile);
    instrumentOriginal13(mdl);
    for scenario=1:2
        report.model_simulations_attempted=report.model_simulations_attempted+1;
        raw.derived{scenario}=runShort(mdl,scenario);report.model_simulations=report.model_simulations+1;
        a=raw.parent{scenario};z=raw.derived{scenario};
        for k=1:13
            check(sprintf('case%d_original_core_%02d_bitwise',scenario,k),sameSeries(a.core{k},z.core{k}));
        end
        for k=1:4
            check(sprintf('case%d_original_root_%d_bitwise',scenario,k),sameSeries(a.root{k},z.root{k}));
        end
        rotor=z.root{5};simTime=z.root{6};generation=z.root{7};session=z.root{8};valid=z.root{9};failed=z.root{10};
        check(sprintf('case%d_observer_nonnegative_finite',scenario),all(isfinite(rotor.Data(:)))&&all(rotor.Data(:)>=0));
        check(sprintf('case%d_observer_typed_generation_session',scenario),isa(generation.Data,'uint64')&&isa(session.Data,'uint64'));
        check(sprintf('case%d_observer_valid_not_initial_reset',scenario),~valid.Data(1)&&any(valid.Data(:))&&~any(failed.Data(:)));
        g=generation.Data(:);t=simTime.Data(:);take=[true;diff(g)>0];
        check(sprintf('case%d_generation_is_true_accepted_count',scenario),g(1)==0&& ...
            isequal(g(take),uint64((0:double(g(end))).'))&&g(end)==30);
        check(sprintf('case%d_sim_clock_tracks_core_not_observer',scenario),all(abs(t-double(g)*.01)<1e-10));
        params=load(parameterFile,'deliveryPolicy');
        check(sprintf('case%d_session_is_existing_policy',scenario),all(session.Data(:)==uint64(params.deliveryPolicy.expected_session_token)));
        if scenario==2
            check('nonzero_lag_state_actually_exported',max(rotor.Data(:))>1&&max(rotor.Data(:))<32.145727009134916);
        end
    end
    close_system(mdl,0);
    report.parent_unchanged=strcmpi(fileSha(parent),parentSha)&&strcmpi(fileSha(parentParameters),parameterSha);
    report.source_unchanged=true;
    for k=1:numel(sources),report.source_unchanged=report.source_unchanged&&strcmpi(fileSha(sources(k).path),sources(k).sha256);end
    check('parent_model_and_parameters_unchanged',report.parent_unchanged);
    check('copied_parameter_bytes_unchanged',strcmpi(fileSha(parameterFile),parameterSha));
    check('dynamics_and_original_helpers_unchanged',report.source_unchanged);
    report.passed=all(checks);report.status='PASS_SAME_CORE_TYPED_ROTOR_OBSERVER_MODEL__LIVE_WIRE_NOT_CONNECTED';
catch problem
    report.failure=getReport(problem,'extended','hyperlinks','off');
    report.status='HOST_OBSERVER_MODEL_IMPLEMENTATION_FAILURE__NO_HARDWARE';
end
report.tests=names;report.checks=checks;report.passed_checks=sum(checks);report.total_checks=numel(checks);
if isfile(modelFile),report.model_sha256=fileSha(modelFile);end
save(fullfile(outputDir,'SHORT_COMPARISON_RAW.mat'),'raw','priorRaw','-v7');
writeText(fullfile(outputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(jsonencode(report));assert(report.passed,'m600check:RotorModelTest','%s',report.failure);
    function check(name,value)
        names{end+1}=name;checks(end+1,1)=logical(value);
        assert(value,'m600check:RotorModelAssertion','%s',name);
    end
end
function core=getCore(mdl)
sf=sfroot;core=sf.find('-isa','Stateflow.EMChart','Path',[mdl '/CurrentM600_10ms']);assert(isscalar(core));
end
function instrumentOriginal13(mdl)
ports=get_param([mdl '/CurrentM600_10ms'],'PortHandles');
for k=1:13
    line=get_param(ports.Outport(k),'Line');assert(line~=-1);
    set_param(ports.Outport(k),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',sprintf('OriginalCoreOutput%02d',k));
end
end
function r=runShort(mdl,scenario)
t=(0:.001:.30).';u=zeros(numel(t),16);
if scenario==2
    for k=1:6,u(:,k)=.15+.01*k+.02*sin(2*pi*t*(1+k/10));end
end
inputs=Simulink.SimulationData.Dataset;
v=timeseries(u,t);v=setinterpmethod(v,'zoh');inputs=inputs.addElement(v,'inPWMs');
v=timeseries(zeros(numel(t),15),t);v=setinterpmethod(v,'zoh');inputs=inputs.addElement(v,'TerrainIn15d');
v=timeseries(zeros(numel(t),28),t);v=setinterpmethod(v,'zoh');inputs=inputs.addElement(v,'inDoubCtrls');
in=Simulink.SimulationInput(mdl);in=in.setExternalInput(inputs);
in=in.setModelParameter('StartTime','0','StopTime','.30','SimulationMode','normal', ...
    'SaveOutput','on','OutputSaveName','yout','ReturnWorkspaceOutputs','on', ...
    'SaveFormat','Dataset','SignalLogging','on','SignalLoggingName','logsout');
rng(1234,'twister');out=sim(in);
r=struct('model_file',get_param(mdl,'FileName'),'scenario',scenario,'core',{{}},'root',{{}});
for k=1:13,r.core{k}=plain(out.logsout.getElement(sprintf('OriginalCoreOutput%02d',k)).Values);end
for k=1:out.yout.numElements,r.root{k}=plain(out.yout.getElement(k).Values);end
end
function r=plain(ts)
r=struct('Time',ts.Time,'Data',ts.Data);
end
function yes=sameSeries(a,b)
yes=isequal(a.Time,b.Time)&&strcmp(class(a.Data),class(b.Data))&&isequal(size(a.Data),size(b.Data));
if yes
    if islogical(a.Data),yes=isequal(a.Data,b.Data);
    else,yes=isequal(typecast(a.Data(:),'uint8'),typecast(b.Data(:),'uint8'));end
end
end
function closeOwned(mdl),if bdIsLoaded(mdl),close_system(mdl,0);end,end
function h=fileSha(p)
f=fopen(p,'rb');assert(f>=0,'m600check:MissingFile','Missing %s',p);c=onCleanup(@()fclose(f)); %#ok<NASGU>
md=java.security.MessageDigest.getInstance('SHA-256');md.update(uint8(fread(f,Inf,'*uint8')));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function h=textSha(t)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(uint8(unicode2native(t,'UTF-8')));
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
function writeText(p,t)
f=fopen(p,'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',t);
end
