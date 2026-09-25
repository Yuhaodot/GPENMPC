function report=run_terrain_diagnostic_source_replay(modelDir,newOutputDir)
% Simulate the SLX and execute its extracted chart body for comparison.
arguments
    modelDir (1,1) string
    newOutputDir (1,1) string
end
modelDir=canonical(modelDir);newOutputDir=canonical(newOutputDir);
assert(~isfolder(newOutputDir)&&~isfile(newOutputDir),'m600check:ReplayOutputExists');
b=string(fileparts(fileparts(mfilename('fullpath'))));m=fullfile(b,'m600_coptersim');
k=fullfile(b,'matlab_validation');mdl='GPENMPC_M600_Canonical';
assert(~bdIsLoaded(mdl),'m600check:ReplayModelAlreadyLoaded');
oldPath=path;pathGuard=onCleanup(@()path(oldPath)); %#ok<NASGU>
oldDir=pwd;cwdGuard=onCleanup(@()cd(oldDir)); %#ok<NASGU>
oldRng=rng;rngGuard=onCleanup(@()rng(oldRng)); %#ok<NASGU>
addpath(k,fullfile(m,'matlab_validation'),fullfile(b,'tools'),'-begin');
fixture=m600check.loadFixture(); %#ok<NASGU> Source dependency checks only.
parentDir=fullfile(gpenmpc_external_path('flat_terrain_model'));
parentFile=fullfile(parentDir,mdl+".slx");derivedFile=fullfile(modelDir,mdl+".slx");
paramFile=fullfile(modelDir,'M600_CORE_PARAMETERS.mat');
parentSha="5B5874C28A2D6E005F29F88F8B95B475005FB62BFE2B41AF03A783425ECFA505";
paramSha="969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4";
preparation=jsondecode(fileread(fullfile(modelDir,'MODEL_PREPARATION_RESULT.json')));
assert(preparation.passed&&preparation.model_updated&&preparation.single_terrain_input_snapshot&& ...
    preparation.sensor_inverse_path_preserved&&preparation.parent_files_unchanged, ...
    'm600check:ReplayPreparationIncomplete');
assert(strcmpi(sha(parentFile),parentSha)&&strcmpi(sha(paramFile),paramSha)&& ...
    strcmpi(sha(fullfile(parentDir,'M600_CORE_PARAMETERS.mat')),paramSha)&& ...
    strcmpi(sha(derivedFile),preparation.model_sha256),'m600check:ReplaySourceIdentity');
derivedSha=sha(derivedFile);
assert(strcmpi(sha(which('simulateCanonicalM600')), ...
    '6669E2DC310B929BB76AE5C1F4B14D62DE63C5A17ACA5777FBFF26D0B81D78A4'), ...
    'm600check:ReplayMechanismChanged');
for j=1:numel(preparation.helper_paths)
    assert(strcmpi(sha(string(preparation.helper_paths{j})),string(preparation.helper_sha256{j})), ...
        'm600check:ReplayHelperIdentity');
end
mkdir(newOutputDir);copyfile(paramFile,fullfile(newOutputDir,'M600_CORE_PARAMETERS.mat'));
parentCopy=fullfile(newOutputDir,'parent_sim');derivedCopy=fullfile(newOutputDir,'derived_sim');
copyModel(parentDir,parentCopy);copyModel(modelDir,derivedCopy);
report=struct('schema','HOST_TERRAIN_DIAGNOSTIC_SOURCE_REPLAY_V1','passed',false, ...
    'status','HOST_REPLAY_IN_PROGRESS','failure','', ...
    'parent_model',parentFile,'parent_model_sha256',parentSha, ...
    'derived_model',derivedFile,'derived_model_sha256',derivedSha, ...
    'parameter_sha256',paramSha,'checks',struct('name',{},'passed',{}), ...
    'source_cases',struct([]),'normal_model_samples',0,'fault_model_rows',0, ...
    'arithmetic_tolerance',1e-9, ...
    'tolerance_provenance','EXISTING_SIMULINK_SOURCE_REPLAY_ARITHMETIC__NOT_FLIGHT_SCREEN', ...
    'extraction_change','ONLY_TOP_LEVEL_FUNCTION_NAME__FULL_CHART_BODY_EXECUTED', ...
    'reset_scope','NEW_MODEL_OR_EXTRACTED_CHART_LIFECYCLE__NO_MIDRUN_RESET_PORT_EXISTS', ...
    'normal_full_model_simulation',false,'finite_fault_full_model_simulation',false, ...
    'nonfinite_fault_scope','CHART_SOURCE_EXECUTION', ...
    'DLL_built',false,'DLL_loaded',false,'COM_open',0,'UDP_open',0,'board_actions',0, ...
    'closed_loop_controller_used',false,'HIL_performance_claim',false, ...
    'terrain_source_cause_identified',false,'original_inputs_unchanged',false);
raw=struct;
try
    % Existing 301-point full-model/source mechanism, on disposable identical
    % copies only; these also retain the actual 1 ms I/O output time series.
    rng(oldRng);raw.parent_normal=simulateCanonicalM600(parentCopy);
    rng(oldRng);raw.derived_normal=simulateCanonicalM600(derivedCopy);
    a=load(fullfile(parentCopy,'SIMULINK_SOURCE_REPLAY.mat'),'simout');
    d=load(fullfile(derivedCopy,'SIMULINK_SOURCE_REPLAY.mat'),'simout');
    compareModelOutputs(a.simout,d.simout,'normal');
    report.normal_model_samples=raw.derived_normal.samples;
    check('existing_301_sample_simulink_source_replays_pass', ...
        raw.parent_normal.samples==301&&raw.derived_normal.samples==301&& ...
        raw.parent_normal.maximum_absolute_difference<1e-9&& ...
        raw.derived_normal.maximum_absolute_difference<1e-9);
    report.normal_full_model_simulation=true;

    % Apply a one-ULP height change, then restore the height.
    % Valid data must not clear a latched fault.
    rng(oldRng);raw.parent_fault=finiteFaultSimulation(parentCopy,m);
    rng(oldRng);raw.derived_fault=finiteFaultSimulation(derivedCopy,m);
    compareModelOutputs(raw.parent_fault,raw.derived_fault,'finite_fault');
    [faultTime,faultDiag]=output(raw.derived_fault,4,32);
    first=find(faultDiag(:,1)==1,1);
    check('actual_model_finite_fault_entered_at_declared_20ms_sample', ...
        ~isempty(first)&&abs(faultTime(first)-.02)<1e-9&&faultDiag(first,2)==4);
    check('actual_model_first_fault_does_not_clear_after_old_height_returns', ...
        all(faultDiag(first:end,1)==1)&&all(faultDiag(first:end,2)==4)&& ...
        all(faultDiag(first:end,8)==1)&&all(faultDiag(first:end,9)==3)&& ...
        all(faultDiag(first:end,10)==1)&&all(faultDiag(first:end,11)==1+eps(1)));
    report.fault_model_rows=numel(faultTime);report.finite_fault_full_model_simulation=true;

    % Read the REAL model charts. Renaming fcn is the only extraction edit;
    % coder.load, persistent/reset, core and encoder calls remain original.
    [parentCore,parentSensor]=readCharts(parentCopy);
    [derivedCore,derivedSensor]=readCharts(derivedCopy);
    check('actual_chart_matches_preparation_body',strcmp(derivedCore,preparation.chart_after));
    check('sensor_inverse_chart_byte_exact_between_models',strcmp(parentSensor,derivedSensor));
    names={'terrain_replay_parent_chart','terrain_replay_derived_chart','terrain_replay_sensor_chart'};
    scripts={parentCore,derivedCore,derivedSensor};
    raw.extracted_sources=struct('path',{},'sha256',{},'chart_text',{});
    for j=1:3
        [text,inverseOkay]=renamed(scripts{j},names{j});
        check(sprintf('extracted_chart_%d_only_function_name_changed',j),inverseOkay);
        file=fullfile(newOutputDir,names{j}+".m");writeText(file,text);
        raw.extracted_sources(j)=struct('path',file,'sha256',sha(file),'chart_text',scripts{j});
    end
    addpath(newOutputDir,'-begin');cd(newOutputDir);
    sourceGuard=onCleanup(@clearReplaySources); %#ok<NASGU>
    cases=sourceCases();caseRows=cell(numel(cases),1);raw.source_runs=cell(numel(cases),1);
    for c=1:numel(cases)
        spec=cases(c);parentRun=runChart(names{1},spec);derivedRun=runChart(names{2},spec);
        check(spec.name+"_all_twelve_plant_sensor_input_outputs_bit_exact", ...
            sameCells(parentRun.outputs(1:12,:),derivedRun.outputs(1:12,:)));
        check(spec.name+"_legacy_prefix7_bit_exact",sameBits(parentRun.diag(1:7,:),derivedRun.diag(1:7,:)));
        check(spec.name+"_legacy25_reserved_and_new_tag_exact", ...
            all(parentRun.diag(8:32,:)==0,'all')&&all(derivedRun.diag(26,:)==1)&& ...
            all(derivedRun.diag(27:32,:)==0,'all'));
        first=find(derivedRun.diag(1,:)==1,1);firstColumn=0;reason=0;locked=0;bits={};
        if spec.fault_reason>0
            check(spec.name+"_genuine_core_code4_and_declared_first_reason", ...
                ~isempty(first)&&first==spec.first_fault_column&& ...
                all(derivedRun.diag(1,first:end)==1)&&all(derivedRun.diag(2,first:end)==4)&& ...
                all(derivedRun.diag(9,first:end)==spec.fault_reason));
            capture=derivedRun.diag(8:25,first);firstRaw=spec.terrain(:,first);
            check(spec.name+"_first_15d_raw_bits_and_latch_not_washed", ...
                sameBits(derivedRun.diag(11:25,first),firstRaw)&& ...
                all(arrayfun(@(col)sameBits(derivedRun.diag(8:25,col),capture),first:size(spec.terrain,2))));
            firstColumn=first;reason=derivedRun.diag(9,first);locked=derivedRun.diag(10,first);
            bits=hexVector(firstRaw);
        else
            check(spec.name+"_healthy_new_lifecycle_clears_capture_and_starts_at_zero", ...
                isempty(first)&&derivedRun.diag(3,1)==0&&all(derivedRun.diag(8:25,:)==0,'all'));
        end
        decoderOkay=true;previousTime=NaN;
        for j=1:size(derivedRun.diag,2)
            decoded=m600check.decodeCopterSimTerrainDiagnostics(packet(derivedRun.diag(:,j)),1,previousTime);
            decoderOkay=decoderOkay&&decoded.packet_valid&&decoded.terrain_extension.valid&& ...
                (decoded.must_stop==(derivedRun.diag(1,j)~=0));
            previousTime=derivedRun.diag(3,j);
        end
        check(spec.name+"_actual_chart_packet_decoder_preserves_fail_closed",decoderOkay);
        caseRows{c}=struct('name',spec.name,'calls',size(spec.terrain,2), ...
            'first_fault_column',firstColumn,'first_reason',reason,'locked_height',locked, ...
            'first_terrain_float64_hex',{bits},'passed',true);
        raw.source_runs{c}=struct('specification',spec,'parent',parentRun,'derived',derivedRun);
    end
    report.source_cases=vertcat(caseRows{:});

    % Execute the extracted sensor chart, not merely the helper's name check.
    % An independent axis-product oracle applies the PX4 leveling transform.
    x=6.445373058319092*pi/180;y=-6.442468166351318*pi/180;
    L=[cos(y),0,sin(y);0,1,0;-sin(y),0,cos(y)]* ...
        [1,0,0;0,cos(x),-sin(x);0,sin(x),cos(x)];
    rs=RandStream('mt19937ar','Seed',250906);v=randn(rs,30,256);maximum=0;unchanged=true;
    for j=1:256
        wire=feval(names{3},v(:,j));expected=v(:,j);
        maximum=max(maximum,max(abs(reshape(L*reshape(wire(2:10),3,3),9,1)-expected(2:10))));
        unchanged=unchanged&&sameBits(wire([1,11:30]),expected([1,11:30]));
    end
    check('actual_sensor_chart_768_vector_inverse_recalibrations',maximum<1e-12);
    check('actual_sensor_chart_5376_other_channel_bits_unchanged',unchanged);
    report.sensor_inverse_maximum_recalibration_error=maximum;
    raw.codec_tests=run_m600_terrain_diagnostic_extension_tests("");
    check('existing_full_111_codec_state_negative_controls',raw.codec_tests.passed&& ...
        raw.codec_tests.checks_total==111&&raw.codec_tests.checks_passed==111);
    report.codec_checks_total=raw.codec_tests.checks_total;
    check('source_model_and_parameters_remain_unchanged',strcmpi(sha(parentFile),parentSha)&& ...
        strcmpi(sha(derivedFile),derivedSha)&&strcmpi(sha(paramFile),paramSha));
    report.original_inputs_unchanged=true;
    report.status='PASS_HOST_ACTUAL_MODEL_AND_CHART_DIAGNOSTIC_REPLAY__DLL_PROBE_PENDING';
    report.passed=true;
catch exception
    report.status='HOST_TERRAIN_SOURCE_REPLAY_FAILED__NO_HARDWARE_RESULT';
    report.failure=getReport(exception,'extended','hyperlinks','off');
end
report.checks_total=numel(report.checks);report.checks_passed=sum([report.checks.passed]);
report.source_script=string(mfilename('fullpath'))+".m";report.source_script_sha256=sha(report.source_script);
report.build_input_copy=derivedCopy;
report.next_step=['buildCanonicalM600Dll may use derived_sim, whose SLX is byte-identical to the requested model ' ...
    'and includes the bound SIMULINK_SOURCE_REPLAY receipt. The current report must also pass. ' ...
    'Generated-code reset/first-fault/NaN-bit behavior and decoder integration still require a DLL probe.'];
save(fullfile(newOutputDir,'RAW_SOURCE_REPLAY.mat'),'raw','report','-v7.3');
writeText(fullfile(newOutputDir,'RESULT.json'),jsonencode(report,PrettyPrint=true));
disp(struct('passed',report.passed,'checks',report.checks_total,'COM_open',0));
assert(report.passed,'m600check:TerrainSourceReplayFailed','%s',report.failure);

    function check(name,passed)
        passed=isscalar(passed)&&logical(passed);
        report.checks(end+1)=struct('name',char(name),'passed',passed);
        assert(passed,'m600check:TerrainSourceReplayCheck','Failed: %s',name);
    end
    function compareModelOutputs(a,d,label)
        for port=1:4
            widths=[30,30,60,32];[ta,va]=output(a,port,widths(port));[td,vd]=output(d,port,widths(port));
            check(label+"_port"+port+"_complete_time_rows_identical",sameBits(ta,td));
            if port==4
                check(label+"_actual_model_prefix7_bit_exact",sameBits(va(:,1:7),vd(:,1:7)));
                check(label+"_new_tag_and_reserved_in_actual_model",all(vd(:,26)==1)&&all(vd(:,27:32)==0,'all'));
            else
                check(label+"_actual_model_port"+port+"_all_values_bit_exact",sameBits(va,vd));
            end
        end
    end
end

function copyModel(source,destination)
mkdir(destination);copyfile(fullfile(source,'GPENMPC_M600_Canonical.slx'),destination);
copyfile(fullfile(source,'M600_CORE_PARAMETERS.mat'),destination);
end
function [core,sensor]=readCharts(modelDir)
mdl='GPENMPC_M600_Canonical';assert(~bdIsLoaded(mdl));
load_system(fullfile(modelDir,mdl+".slx"));guard=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',[mdl '/CurrentM600_10ms']);
s=r.find('-isa','Stateflow.EMChart','Path',[mdl '/VirtualSensorBoardLevelInverse']);
assert(isscalar(c)&&isscalar(s));core=char(c.Script);sensor=char(s.Script);
end
function [text,okay]=renamed(original,name)
assert(numel(strfind(original,'= fcn('))+numel(strfind(original,'=fcn('))==1, ...
    'm600check:ExtractedFunctionSignature');
text=regexprep(original,'(=\s*)fcn\(','$1'+string(name)+'(');
restored=regexprep(text,'(=\s*)'+string(name)+'\(','$1fcn(');
okay=strcmp(restored,original);
end
function out=finiteFaultSimulation(modelDir,modelRoot)
old=pwd;cd(modelDir);cwd=onCleanup(@()cd(old)); %#ok<NASGU>
evalin('base',"run('"+fullfile(modelRoot,'GPENMPC_M600_CopterSim_init.m')+"')");
mdl='GPENMPC_M600_Canonical';assert(~bdIsLoaded(mdl));
load_system(fullfile(modelDir,mdl+".slx"));guard=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
t=(0:.001:.05).';terrain=zeros(numel(t),15);terrain(:,1)=1;
terrain(t>=.02&t<.03,1)=1+eps(1);u=zeros(numel(t),16);
ut=timeseries(u,t);ut=setinterpmethod(ut,'zoh');tt=timeseries(terrain,t);tt=setinterpmethod(tt,'zoh');
input=Simulink.SimulationData.Dataset;input=input.addElement(ut,'inPWMs');input=input.addElement(tt,'TerrainIn15d');
si=Simulink.SimulationInput(mdl);si=si.setExternalInput(input);
si=si.setVariable('ModelInit_PosE',[0,0,1]);si=si.setVariable('ModelInit_AngEuler',[0,0,0]);
si=si.setModelParameter('StopTime','0.05','ReturnWorkspaceOutputs','on');out=sim(si);
end
function [t,data]=output(simout,port,width)
dataset=simout.yout;element=dataset.getElement(port);values=element.Values;
t=double(values.Time(:));data=squeeze(values.Data);
if size(data,1)~=numel(t),data=data.';end
assert(isequal(size(data),[numel(t),width]),'m600check:ReplayOutputShape');
end
function cases=sourceCases()
base=struct('name',"",'controls',zeros(16,6),'terrain',zeros(15,6), ...
    'position',zeros(3,1),'euler',zeros(3,1),'fault_reason',0,'first_fault_column',0);
cases=repmat(base,0,1);
a=base;a.name="normal_301";t=0:.01:3;a.controls=zeros(16,301);a.terrain=zeros(15,301);
c=zeros(size(t));c(t>=.5&t<1.5)=.7;c(t>=1.5)=.61;a.controls(1:6,:)=repmat(c,6,1);cases(end+1)=a;
a=base;a.name="height_one_ULP_then_good_then_other_fault";a.position(3)=1;a.terrain(1,:)=1;
a.terrain(1,3)=1+eps(1);a.terrain(15,5)=Inf;a.fault_reason=3;a.first_fault_column=3;cases(end+1)=a;
nanBits=bitor(bitshift(uint64(hex2dec('7FF80000')),32),uint64(123));
bad={typecast(nanBits,'double'),Inf,-Inf};labels={"NaN_payload123","positive_Inf","negative_Inf"};
for channel=1:15
    for kind=1:3
        a=base;a.name="channel"+channel+"_"+labels{kind}+"_then_good";
        a.terrain(channel,3)=bad{kind};a.terrain(1,5)=1;a.fault_reason=2;a.first_fault_column=3;
        cases(end+1)=a; %#ok<AGROW>
    end
end
a=base;a.name="initial_nonfinite_reset_not_accepted";a.terrain(15,1)=Inf;
a.fault_reason=2;a.first_fault_column=1;cases(end+1)=a;
a=base;a.name="fresh_lifecycle_after_all_faults";cases(end+1)=a;
end
function result=runChart(name,spec)
clear m600check.copterSimFlatTerrainCore m600check.copterSimIoCore
eval(['clear ' name]);n=size(spec.terrain,2);result.outputs=cell(13,n);result.diag=zeros(32,n);
for j=1:n
    o=cell(1,13);[o{:}]=feval(name,spec.controls(:,j),spec.position,spec.euler,spec.terrain(:,j));
    result.outputs(:,j)=o.';result.diag(:,j)=o{13};
end
end
function clearReplaySources()
clear terrain_replay_parent_chart terrain_replay_derived_chart terrain_replay_sensor_chart
clear m600check.copterSimFlatTerrainCore m600check.copterSimIoCore
end
function yes=sameCells(a,b)
yes=isequal(size(a),size(b));for j=1:numel(a),yes=yes&&sameBits(a{j},b{j});end
end
function yes=sameBits(a,b)
yes=isequal(size(a),size(b))&&isa(a,'double')&&isa(b,'double')&& ...
    isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end
function h=hexVector(a),h=cellstr(upper(dec2hex(typecast(a(:),'uint64'),16))).';end
function bytes=packet(payload)
head=int32([1234567890;1]);values=payload(:);[~,~,endian]=computer;
if endian=='B',head=swapbytes(head);values=swapbytes(values);end
bytes=[typecast(head,'uint8');typecast(values,'uint8')];bytes=bytes(:);
end
function p=canonical(p),p=string(char(java.io.File(char(p)).getCanonicalPath()));end
function h=sha(p),h=m600check.fileSha256(p);end
function writeText(file,text)
f=fopen(file,'w','n','UTF-8');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',text);
end
