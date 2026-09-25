function report=prepare_terrain_diagnostic_model(newOutputDir)
% Add diagnostics to the sensor-aligned model.
arguments
    newOutputDir (1,1) string
end
newOutputDir=string(char(java.io.File(char(newOutputDir)).getCanonicalPath()));
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'m600check:ExistingTerrainModel', ...
    'Use a new output directory; earlier evidence must not be overwritten.');
build=string(fileparts(fileparts(mfilename('fullpath'))));
modelRoot=fullfile(build,'m600_coptersim');kernel=fullfile(build,'matlab_validation');
parent=fullfile(gpenmpc_external_path('flat_terrain_model'),'GPENMPC_M600_Canonical.slx');
parameterParent=fullfile(fileparts(parent),'M600_CORE_PARAMETERS.mat');
parentSha='5B5874C28A2D6E005F29F88F8B95B475005FB62BFE2B41AF03A783425ECFA505';
parameterSha='969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4';
helpers=[fullfile(kernel,'+m600check','initialCopterSimTerrainDiagnosticState.m'); ...
    fullfile(kernel,'+m600check','encodeCopterSimTerrainDiagnostics.m'); ...
    fullfile(modelRoot,'matlab_validation','+m600check','encodeHilSensorLevelFrame.m'); ...
    fullfile(kernel,'+m600check','copterSimFlatTerrainCore.m')];
helperSha=["B8AE37087C93F785B58F685BE372B918EC341852A0685B4B6AC9DFE3E7938B8D"; ...
    "C3FE21E0A004CEECDFA4889511293E3908111931E708C2D03063E1941FF13B81"; ...
    "8F6769EAF1C131E7F917C892CAE814AD6C245FF2BC5AEAA619BAC52883E5ABBB"; ...
    "99F24AFCA87539A1CCD34CE79CAA0E2425BEAF0868CDB07608E4600240F2CBFB"];
assert(strcmpi(hashFile(parent),parentSha),'m600check:TerrainParentModelIdentity');
assert(strcmpi(hashFile(parameterParent),parameterSha),'m600check:TerrainParameterIdentity');
for k=1:numel(helpers)
    assert(strcmpi(hashFile(helpers(k)),helperSha(k)),'m600check:TerrainHelperIdentity', ...
        'The validated helper changed: %s',helpers(k));
end
mdl='GPENMPC_M600_Canonical';
assert(~bdIsLoaded(mdl),'m600check:CanonicalModelAlreadyLoaded', ...
    'Do not interfere with another loaded model.');
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(kernel,fullfile(modelRoot,'matlab_validation'),fullfile(build,'host_runtime'),'-begin');
fixture=m600check.loadFixture(); % Load the bound numerical fixture.
for k=1:numel(helpers)
    [~,helperName]=fileparts(helpers(k));
    assert(strcmpi(string(which("m600check."+helperName)),helpers(k)), ...
        'm600check:TerrainHelperShadowed','Wrong helper on MATLAB path.');
end
mkdir(newOutputDir);
modelFile=fullfile(newOutputDir,mdl+".slx");parameterFile=fullfile(newOutputDir,'M600_CORE_PARAMETERS.mat');
copyfile(parent,modelFile);copyfile(parameterParent,parameterFile);
assert(strcmpi(hashFile(modelFile),parentSha)&&strcmpi(hashFile(parameterFile),parameterSha), ...
    'm600check:TerrainInitialCopyMismatch');
% Make only the derived model copy writable.
[writable,~]=fileattrib(modelFile,'+w');assert(writable,'m600check:TerrainNewCopyWritePermission');
oldDir=pwd;cd(newOutputDir);cwdGuard=onCleanup(@()cd(oldDir)); %#ok<NASGU>
addpath(newOutputDir,'-begin');
modelGuard=onCleanup(@()closeOwnedModel(mdl)); %#ok<NASGU>
report=struct('schema','HOST_TERRAIN_DIAGNOSTIC_MODEL_PREPARATION_V1', ...
    'status','HOST_MODEL_PREPARATION_IN_PROGRESS','passed',false,'failure','', ...
    'parent_model',parent,'parent_model_sha256',parentSha, ...
    'model',modelFile,'model_sha256','','parameters',parameterFile,'parameters_sha256',parameterSha, ...
    'builder',string(mfilename('fullpath'))+".m", ...
    'builder_sha256',hashFile(string(mfilename('fullpath'))+".m"), ...
    'helper_paths',helpers,'helper_sha256',helperSha, ...
    'verified_dependency_manifest_sha256',fixture.source_manifest_sha256, ...
    'chart_before','','chart_after','','chart_before_sha256','','chart_after_sha256','', ...
    'exact_changes',struct('old',{},'replacement',{}), ...
    'noncore_charts_unchanged',false,'block_inventory_unchanged',false, ...
    'sensor_inverse_path_preserved',false,'single_terrain_input_snapshot',false, ...
    'parent_files_unchanged',false,'model_updated',false, ...
    'source_replay_completed',false,'code_generation_completed',false,'DLL_generated',false, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'HIL_ready',false, ...
    'claim','Diagnostic model prepared.');
try
    % Use the model's initialization chain and sensor L inverse.
    evalin('base',"run('"+fullfile(modelRoot,'GPENMPC_M600_CopterSim_init.m')+"')");
    evalin('base','ModelInit_PosE=[0,0,0];ModelInit_AngEuler=[0,0,0];');
    load_system(modelFile);
    sf=sfroot;core=sf.find('-isa','Stateflow.EMChart','Path',[mdl '/CurrentM600_10ms']);
    assert(isscalar(core),'m600check:TerrainCoreNotUnique');
    assert(strcmp(string(core.ChartUpdate),"DISCRETE")&&str2double(string(core.SampleTime))==.01, ...
        'm600check:TerrainCoreSampleIdentity');
    oldScript=char(core.Script);report.chart_before=oldScript;report.chart_before_sha256=hashText(oldScript);
    beforeBlocks=blockInventory(mdl);beforeCharts=otherCharts(sf,mdl,core.Path);
    assertSensorWire(mdl,sf);
    inverseBefore=sf.find('-isa','Stateflow.EMChart','Path',[mdl '/VirtualSensorBoardLevelInverse']);
    beforeWireScript=inverseBefore.Script;

    oldStartup='persistent started;reset=isempty(started);started=true;';
    addedCapture=sprintf(['persistent terrainCapture;\n' ...
        'if isempty(terrainCapture)\n' ...
        '    terrainCapture=m600check.initialCopterSimTerrainDiagnosticState();\n' ...
        'end\n' ...
        'terrainSnapshot=reshape(terrain,15,1);']);
    newStartup=[oldStartup newline addedCapture];
    oldCore=['[y,d]=m600check.copterSimFlatTerrainCore(reshape(u,16,1),reset,' ...
        'reshape(pos0,3,1),reshape(ang0,3,1),data.environment,reshape(terrain,15,1),data.parameters);'];
    newCore=strrep(oldCore,'reshape(terrain,15,1)','terrainSnapshot');
    prefixExpression=['[double(d.failed);double(d.failure_code);d.sim_time_s;d.contact_force_n;' ...
        'double(d.ground_confirmed);double(d.airborne_observed);1]'];
    oldDiagnostic=['diagnostic=zeros(32,1);diagnostic(1:7)=' prefixExpression ';'];
    newDiagnostic=sprintf(['diagnosticPrefix=%s;\n' ...
        '[diagnostic,terrainCapture]=m600check.encodeCopterSimTerrainDiagnostics( ...\n' ...
        '    diagnosticPrefix,terrainSnapshot,d.terrain_failure_reason,d.terrain_world_ned_z_m, ...\n' ...
        '    d.terrain_fault_latched,d.reset_applied,terrainCapture);'],prefixExpression);
    changes=struct('old',{oldStartup,oldCore,oldDiagnostic}, ...
        'replacement',{newStartup,newCore,newDiagnostic});
    newScript=oldScript;
    for k=1:numel(changes)
        assert(numel(strfind(newScript,changes(k).old))==1,'m600check:TerrainPatchAnchor', ...
            'Exact parent chart anchor missing or ambiguous (%d).',k);
        newScript=strrep(newScript,changes(k).old,changes(k).replacement);
    end
    restored=newScript;
    for k=numel(changes):-1:1
        assert(numel(strfind(restored,changes(k).replacement))==1,'m600check:TerrainPatchInverse');
        restored=strrep(restored,changes(k).replacement,changes(k).old);
    end
    assert(strcmp(restored,oldScript),'m600check:TerrainUnexpectedChartChange');
    assert(numel(strfind(newScript,'reshape(terrain,15,1)'))==1&& ...
        numel(strfind(newScript,'m600check.copterSimFlatTerrainCore('))==1&& ...
        numel(strfind(newScript,'m600check.encodeCopterSimTerrainDiagnostics('))==1, ...
        'm600check:TerrainSingleSnapshotContract');
    report.exact_changes=changes;report.chart_after=newScript;report.chart_after_sha256=hashText(newScript);
    core.Script=newScript;
    assert(strcmp(core.Script,newScript),'m600check:TerrainChartAssignmentChanged');
    set_param(mdl,'SimulationCommand','update');report.model_updated=true;
    assert(strcmp(core.Script,newScript),'m600check:TerrainUpdateChangedCoreScript');
    assert(isequal(beforeBlocks,blockInventory(mdl)),'m600check:TerrainBlockInventoryChanged');
    assert(isequal(beforeCharts,otherCharts(sf,mdl,core.Path)),'m600check:TerrainOtherChartChanged');
    inverse=sf.find('-isa','Stateflow.EMChart','Path',[mdl '/VirtualSensorBoardLevelInverse']);
    assert(strcmp(inverse.Script,beforeWireScript),'m600check:TerrainSensorInverseChanged');
    assertSensorWire(mdl,sf);
    assert(strcmpi(hashFile(parameterFile),parameterSha),'m600check:TerrainParametersChanged');
    save_system(mdl,modelFile);
    assert(strcmpi(hashFile(parent),parentSha)&&strcmpi(hashFile(parameterParent),parameterSha), ...
        'm600check:TerrainParentChanged');
    for k=1:numel(helpers)
        assert(strcmpi(hashFile(helpers(k)),helperSha(k)),'m600check:TerrainHelperChangedDuringPreparation');
    end
    report.model_sha256=hashFile(modelFile);report.noncore_charts_unchanged=true;
    report.block_inventory_unchanged=true;report.sensor_inverse_path_preserved=true;
    report.single_terrain_input_snapshot=true;report.parent_files_unchanged=true;
    report.status='PASS_HOST_MODEL_UPDATE_ONLY__SOURCE_REPLAY_CODEGEN_DLL_AND_DECODER_INTEGRATION_PENDING';
    report.passed=true;
catch problem
    report.status='HOST_MODEL_PREPARATION_FAILED__NOT_DLL_OR_HIL_RESULT';
    report.failure=getReport(problem,'extended','hyperlinks','off');
    report.parent_files_unchanged=strcmpi(hashFile(parent),parentSha)&&strcmpi(hashFile(parameterParent),parameterSha);
    if isfile(modelFile),report.model_sha256=hashFile(modelFile);end
end
writeText(fullfile(newOutputDir,'MODEL_PREPARATION_RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('status',report.status,'model',report.model,'COM_open',0,'DLL_generated',false));
assert(report.passed,'m600check:TerrainModelPreparationFailed','%s',report.failure);
end

function list=blockInventory(mdl)
paths=sort(string(find_system(mdl,'LookUnderMasks','all','FollowLinks','off','Type','Block')));
list=struct('path',cell(numel(paths),1),'type',cell(numel(paths),1));
for k=1:numel(paths),list(k).path=char(paths(k));list(k).type=get_param(paths(k),'BlockType');end
end

function list=otherCharts(sf,mdl,corePath)
charts=sf.find('-isa','Stateflow.EMChart');list=struct('path',{},'script',{});
for k=1:numel(charts)
    if startsWith(string(charts(k).Path),string(mdl)+"/")&&~strcmp(charts(k).Path,corePath)
        list(end+1)=struct('path',charts(k).Path,'script',charts(k).Script); %#ok<AGROW>
    end
end
if ~isempty(list),[~,order]=sort(string({list.path}));list=list(order);end
end

function assertSensorWire(mdl,sf)
inversePath=[mdl '/VirtualSensorBoardLevelInverse'];
inverse=sf.find('-isa','Stateflow.EMChart','Path',inversePath);
assert(isscalar(inverse)&&contains(inverse.Script,'m600check.encodeHilSensorLevelFrame(u)'), ...
    'm600check:TerrainSensorInverseMissing');
assertDestination([mdl '/SensorOutput'],inversePath);
assertDestination(inversePath,[mdl '/HILSensor30d']);
end

function assertDestination(source,destination)
ports=get_param(source,'PortHandles');line=get_param(ports.Outport(1),'Line');
assert(line~=-1,'m600check:TerrainSensorWireMissing');
dest=get_param(line,'DstPortHandle');
assert(isscalar(dest)&&strcmp(get_param(dest,'Parent'),destination)&& ...
    str2double(string(get_param(dest,'PortNumber')))==1, ...
    'm600check:TerrainSensorWireChanged');
end

function closeOwnedModel(mdl)
if bdIsLoaded(mdl),close_system(mdl,0);end
end

function hash=hashFile(file)
f=fopen(file,'rb');assert(f>=0,'m600check:TerrainMissingInput','Missing %s',file);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
hash=hashBytes(fread(f,Inf,'*uint8'));
end

function hash=hashText(text)
hash=hashBytes(unicode2native(text,'UTF-8'));
end

function hash=hashBytes(bytes)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(uint8(bytes(:)));
hash=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end

function writeText(file,text)
f=fopen(file,'w','n','UTF-8');assert(f>=0,'m600check:TerrainReportWrite');
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',text);
end
