function report=run_m600_terrain_fault_reason_tests(outputDir,parameterMat)
% RUN_M600_TERRAIN_FAULT_REASON_TESTS Test production-core failure branches offline.
% Run in a fresh MATLAB process; the first call intentionally omits reset.
% Retain each synthetic input as MAT data and IEEE-754 hexadecimal CSV.
arguments
    outputDir (1,1) string
    parameterMat (1,1) string = ""
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
kernel=fullfile(build,'matlab_validation');
if strlength(parameterMat)==0
    parameterMat=fullfile(gpenmpc_external_path('flat_terrain_model'),'M600_CORE_PARAMETERS.mat');
end
assert(~isfolder(outputDir)&&~isfile(outputDir)&&isfile(parameterMat), ...
    'm600check:TerrainReasonOutput','New output directory and existing exact parameters required.');
oldPath=path;restorePath=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(kernel,fullfile(build,'m600_coptersim','matlab_validation'),fullfile(build,'host_runtime'),'-begin');
memory=string(inmem('-completenames'));normalized=strrep(memory,'\','/');
assert(~any(contains(normalized,'/+m600check/copterSimFlatTerrainCore')| ...
    normalized=="m600check.copterSimFlatTerrainCore"), ...
    'm600check:TerrainReasonFreshProcess','Use a fresh MATLAB batch; cold-start evidence cannot inherit persistent state.');
fixture=m600check.loadFixture(); %#ok<NASGU> Resolves audited numerical dependencies only.
source=string(which('m600check.copterSimFlatTerrainCore'));
assert(strcmpi(source,fullfile(kernel,'+m600check','copterSimFlatTerrainCore.m')), ...
    'm600check:TerrainReasonSourceShadowed','The production function must not be shadowed.');
sourceHash=m600check.fileSha256(source);parameterHash=m600check.fileSha256(parameterMat);
assert(strcmpi(sourceHash,'99F24AFCA87539A1CCD34CE79CAA0E2425BEAF0868CDB07608E4600240F2CBFB'), ...
    'm600check:TerrainReasonSourceChanged','Tests exercise the bound production terrain guard.');
assert(strcmpi(parameterHash,'969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4'), ...
    'm600check:TerrainReasonParametersChanged','Use the exact canonical model009 parameter input.');
loaded=load(parameterMat,'parameters','environment');p=loaded.parameters;env=loaded.environment;
u=zeros(16,1);angles=zeros(3,1);world=zeros(3,1);terrain=zeros(15,1);
checks=struct('name',{},'passed',{},'first_call_row',{},'last_call_row',{});
calls=struct('name',{},'reset',{},'terrain15',{},'initial_world_position_ned_m',{}, ...
    'controls16',{},'output',{},'diagnostic',{});
rows=struct('row',{},'name',{},'reset',{},'terrain_float64_hex',{},'terrain_display',{}, ...
    'nonfinite_indices',{},'failure_code',{},'core_failure_code',{},'terrain_reason',{}, ...
    'terrain_fault_latched',{},'terrain_locked',{},'locked_terrain_z_m',{}, ...
    'snapshot_available',{},'observation_valid',{},'step_accepted',{}, ...
    'reset_applied',{},'sim_time_s',{},'plant_step_count',{},'output_equals_previous',{});
mkdir(outputDir);failure='';
try
    % The FIRST production call cannot silently turn the aircraft altitude
    % into a ground estimate or bootstrap an accepted physics snapshot.
    [~,d]=invoke('COLD_NO_EXPLICIT_RESET',terrain,false);
    check('cold_no_reset_is_code4_reason1_without_snapshot', ...
        d.failed&&d.failure_code==4&&d.terrain_failure_reason==1&& ...
        ~d.state_snapshot_available&&~d.terrain_locked&&~d.observation_valid,1);
    bad=terrain;bad(1)=NaN;
    [~,d]=invoke('INVALID_RESET_PRESERVES_COLD_FIRST_REASON',bad,true);
    check('invalid_reset_cannot_replace_cold_first_reason', ...
        d.failure_code==4&&d.terrain_failure_reason==1&&~d.state_snapshot_available,2);
    [~,d]=invoke('EXPLICIT_FINITE_RESET_LOCKS_ZERO',terrain,true);
    check('finite_reset_only_establishes_snapshot_at_t0', ...
        ~d.failed&&d.reset_applied&&d.terrain_locked&&d.state_snapshot_available&& ...
        d.observation_valid&&d.terrain_failure_reason==0&&d.sim_time_s==0&&d.terrain_world_ned_z_m==0,3);

    first=numel(calls)+1;
    for k=1:12
        [~,d]=invoke(sprintf('SAME_FLAT_CONTINUOUS_%02d',k),terrain,false);
        check(sprintf('same_flat_step_%02d_advances_without_terrain_fault',k), ...
            ~d.failed&&d.step_accepted&&~d.terrain_fault_latched&& ...
            d.terrain_failure_reason==0&&abs(d.sim_time_s-.01*k)<1e-12,first);
    end

    % Vary unused channels independently of the height-change branch.
    invoke('UNUSED_FINITE_BASE_RESET',terrain,true);
    [base,baseD]=invoke('UNUSED_FINITE_BASE_STEP',terrain,false);
    finiteExtra=terrain;finiteExtra(2:15)=(-7:6).';
    invoke('UNUSED_FINITE_DIFFERENT_RESET',finiteExtra,true);
    [same,sameD]=invoke('UNUSED_FINITE_DIFFERENT_STEP',finiteExtra,false);
    check('unused_finite_channels_leave_same_core_result', ...
        isequaln(base,same)&&isequaln(baseD.state_up,sameD.state_up)&& ...
        ~sameD.failed&&sameD.terrain_failure_reason==0,numel(calls)-3);

    % A finite height difference (including one ULP) remains an EXACT guard;
    % this fixture neither introduces tolerance nor refits the world origin.
    for delta=[.125,eps(1)]
        terrain=zeros(15,1);terrain(1)=1;world(3)=1;env.reference_jet_ned(3)=1;
        first=numel(calls)+1;invoke('FINITE_CHANGE_RESET_ONE',terrain,true);
        [accepted,acceptedD]=invoke('FINITE_CHANGE_ACCEPTED_STEP',terrain,false);
        changed=terrain;changed(1)=1+delta;
        [frozen,d]=invoke(sprintf('FINITE_HEIGHT_CHANGE_%.17g',delta),changed,false);
        check(sprintf('finite_change_%.17g_is_code4_reason3_and_frozen',delta), ...
            frozenLike(accepted,acceptedD,frozen,d,3),first);
        bad=changed;bad(15)=Inf;
        [still,d]=invoke('INVALID_RESET_PRESERVES_CHANGED_HEIGHT_REASON',bad,true);
        check('invalid_reset_keeps_original_height_change_reason', ...
            frozenLike(accepted,acceptedD,still,d,3),first);
        world(3)=changed(1);env.reference_jet_ned(3)=changed(1);
        [y,d]=invoke('VALID_RESET_RELOCKS_EXPLICIT_NEW_HEIGHT',changed,true);
        check('valid_reset_relocks_only_explicit_height',~d.failed&&d.reset_applied&& ...
            d.terrain_failure_reason==0&&d.terrain_world_ned_z_m==changed(1)&& ...
            d.sim_time_s==0&&y.position_ned_m(3)==changed(1),first);
    end

    world(:)=0;env.reference_jet_ned(:)=0;terrain=zeros(15,1);
    invoke('SIGNED_ZERO_RESET',terrain,true);signedZero=terrain;signedZero(1)=-0.0;
    [~,d]=invoke('SIGNED_ZERO_IS_NUMERICALLY_SAME_HEIGHT',signedZero,false);
    check('signed_zero_does_not_trigger_exact_numeric_height_guard', ...
        ~d.failed&&d.terrain_failure_reason==0,numel(calls)-1);

    % Full 15-channel coverage, including each of channels 2..15 separately.
    % NaN payload bits are explicit. All guards still call production code.
    injected={typecast(uint64(hex2dec('7FF80000'))*uint64(4294967296)+uint64(1),'double'),Inf,-Inf};
    labels={'NAN','PLUS_INF','MINUS_INF'};
    for index=1:15
        for kind=1:3
            first=numel(calls)+1;
            invoke(sprintf('CHANNEL_%02d_%s_RESET',index,labels{kind}),terrain,true);
            [accepted,acceptedD]=invoke(sprintf('CHANNEL_%02d_%s_ACCEPTED',index,labels{kind}),terrain,false);
            bad=terrain;bad(index)=injected{kind};
            [frozen,d]=invoke(sprintf('CHANNEL_%02d_%s_FAULT',index,labels{kind}),bad,false);
            check(sprintf('channel_%02d_%s_code4_reason2_frozen',index,labels{kind}), ...
                frozenLike(accepted,acceptedD,frozen,d,2),first);
            [still,d]=invoke(sprintf('CHANNEL_%02d_%s_GOOD_CANNOT_CLEAR',index,labels{kind}),terrain,false);
            check(sprintf('channel_%02d_%s_first_fault_latch_persists',index,labels{kind}), ...
                frozenLike(accepted,acceptedD,still,d,2),first);
            [~,d]=invoke(sprintf('CHANNEL_%02d_%s_VALID_NEW_RESET',index,labels{kind}),terrain,true);
            check(sprintf('channel_%02d_%s_explicit_reset_clears',index,labels{kind}), ...
                ~d.failed&&d.reset_applied&&d.terrain_failure_reason==0&&d.sim_time_s==0,first);
        end
    end
catch problem
    failure=getReport(problem,'extended','hyperlinks','off');
end
passed=isempty(failure)&&~isempty(checks)&&all([checks.passed]);
report=struct('schema','HOST_PRODUCTION_TERRAIN_FAULT_REASON_FIXTURE_V1', ...
    'status','PASS_HOST_ONLY_PRODUCTION_TERRAIN_BRANCH_COVERAGE', ...
    'passed',passed,'failure',failure,'checks_total',numel(checks), ...
    'checks_passed',sum([checks.passed]),'checks',checks,'production_call_count',numel(calls), ...
    'calls',rows,'source_path',source,'source_sha256',sourceHash, ...
    'parameter_path',parameterMat,'parameter_sha256',parameterHash, ...
    'source_modified',false,'DLL_loaded',false,'Simulink_model_started',false, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'parameter_writes',0, ...
    'arm_disarm_mode_task_requests',0,'physical_output_actions',0, ...
    'terrain_source_cause_identified',false, ...
    'claim','MATLAB diagnostic branch replay. The source record lacks terrain-reason and terrain-input fields, so its trigger remains unidentified.');
if ~passed,report.status='FAIL_HOST_ONLY_PRODUCTION_TERRAIN_BRANCH_COVERAGE';end
save(fullfile(outputDir,'RAW_CASES.mat'),'calls','report','-v7');
writeJson(fullfile(outputDir,'RESULT.json'),report);
writeCsv(fullfile(outputDir,'TERRAIN_FAULT_CASES.csv'),rows);
disp(struct('status',report.status,'checks_total',report.checks_total, ...
    'checks_passed',report.checks_passed,'production_calls',numel(calls),'output_dir',outputDir));

    function [y,d]=invoke(name,input,reset)
        [y,d]=m600check.copterSimFlatTerrainCore(u,reset,world,angles,env,input,p);
        equalPrevious=false;if ~isempty(calls),equalPrevious=isequaln(y,calls(end).output);end
        calls(end+1)=struct('name',name,'reset',reset,'terrain15',input, ...
            'initial_world_position_ned_m',world,'controls16',u,'output',y,'diagnostic',d); %#ok<AGROW>
        rows(end+1)=struct('row',numel(calls),'name',name,'reset',reset, ...
            'terrain_float64_hex',{hexVector(input)},'terrain_display',{displayVector(input)}, ...
            'nonfinite_indices',find(~isfinite(input)).','failure_code',double(d.failure_code), ...
            'core_failure_code',double(d.core_failure_code),'terrain_reason',double(d.terrain_failure_reason), ...
            'terrain_fault_latched',logical(d.terrain_fault_latched),'terrain_locked',logical(d.terrain_locked), ...
            'locked_terrain_z_m',d.terrain_world_ned_z_m,'snapshot_available',logical(d.state_snapshot_available), ...
            'observation_valid',logical(d.observation_valid),'step_accepted',logical(d.step_accepted), ...
            'reset_applied',logical(d.reset_applied),'sim_time_s',d.sim_time_s, ...
            'plant_step_count',double(d.plant_step_count),'output_equals_previous',equalPrevious); %#ok<AGROW>
    end
    function check(name,okay,first)
        okay=isscalar(okay)&&logical(okay);
        checks(end+1)=struct('name',name,'passed',okay,'first_call_row',first,'last_call_row',numel(calls)); %#ok<AGROW>
        assert(okay,'m600check:TerrainReasonFixture','Failed production branch assertion: %s',name);
    end
end
function okay=frozenLike(accepted,acceptedD,output,d,reason)
okay=d.failed&&d.failure_code==4&&d.terrain_failure_reason==reason&& ...
    d.terrain_fault_latched&&~d.observation_valid&&~d.step_accepted&& ...
    d.state_snapshot_available&&d.sim_time_s==acceptedD.sim_time_s&& ...
    d.plant_step_count==acceptedD.plant_step_count&&isequaln(accepted,output)&& ...
    isequaln(acceptedD.state_up,d.state_up);
end
function h=hexVector(input)
h=cellstr(upper(dec2hex(typecast(double(input(:)),'uint64'),16))).';
end
function s=displayVector(input)
s=cell(1,15);for k=1:15,s{k}=sprintf('%.17g',input(k));end
end
function writeJson(file,value)
fid=fopen(file,'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
function writeCsv(file,rows)
fid=fopen(file,'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'row,name,reset,failure_code,terrain_reason,sim_time_s,step_accepted,locked_terrain_z_m');
for k=1:15,fprintf(fid,',terrain_%02d_float64_hex,terrain_%02d_display',k,k);end
fprintf(fid,'\n');
for k=1:numel(rows)
    r=rows(k);fprintf(fid,'%d,%s,%d,%d,%d,%.17g,%d,%.17g', ...
        r.row,r.name,r.reset,r.failure_code,r.terrain_reason,r.sim_time_s,r.step_accepted,r.locked_terrain_z_m);
    for j=1:15,fprintf(fid,',%s,%s',r.terrain_float64_hex{j},r.terrain_display{j});end
    fprintf(fid,'\n');
end
end
