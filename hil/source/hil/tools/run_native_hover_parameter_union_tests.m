function result=run_native_hover_parameter_union_tests(outputDir)
% Test the 138-field parameter-union reader.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:UnionTestOutputExists');
build=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
restorePath=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
[t,tv]=m600check.buildNativeHoverTuningContract();assert(tv.passed,'%s',tv.failure);
[g,gv]=m600check.buildTemporaryAllocatorGeometryContract( ...
    fullfile(gpenmpc_external_path('host_native_rotor_geometry'),'RESULT.json'));
assert(gv.passed,'%s',gv.failure);
raw=jsondecode(fileread(t.bindings.current138_raw.path));baseline=raw.diagnostic_parameter_observations;
tests=struct('name',{},'passed',{},'reader_calls',{},'receipt',{});
store=[];readCalls=0;readNames={};fault='';selected='';
run('original_exact_138',t,g,'ORIGINAL','original','',true,138);
run('applied_disjoint_12_plus_3_exact_138',t,g,'APPLIED','applied','',true,138);
run('restored_exact_138',t,g,'RESTORED','original','',true,138);
run('target_values_are_not_original',t,g,'ORIGINAL','applied','',false,138);
run('original_values_are_not_applied',t,g,'APPLIED','original','',false,138);
run('target_values_are_not_restored',t,g,'RESTORED','applied','',false,138);
for mode={'missing_first','missing_last','wrong_name','wrong_type','wrong_bits','wrong_decoded', ...
        'decoded_nan','decoded_vector','raw_field_missing','reader_exception','guard_drift', ...
        'geometry_drift','tuning_drift','failure_detector_disabled','hover_estimator_disabled','rate_I_drift'}
    run(['observed_' mode{1}],t,g,'APPLIED','applied',mode{1},false,138);
end
bad=t;bad.entries(1).name='CA_ROTOR0_PX';run('overlapping_three_and_twelve_rejected',bad,g,'APPLIED','applied','',false,0);
bad=g;bad.entries(1).original_raw_bits_hex='3F800000';run('geometry_original_conflict',t,bad,'ORIGINAL','original','',false,0);
bad=g;bad.entries(1).target_raw_bits_hex='3F800000';run('unknown_geometry_target',t,bad,'APPLIED','applied','',false,0);
bad=t;bad.entries(1).target_raw_bits_hex='3F800000';run('unknown_tuning_target',bad,g,'APPLIED','applied','',false,0);
bad=t;bad.entries(2)=bad.entries(1);run('duplicate_tuning_entry',bad,g,'ORIGINAL','original','',false,0);
bad=g;bad.entries(2)=bad.entries(1);run('duplicate_geometry_entry',t,bad,'ORIGINAL','original','',false,0);
bad=t;bad.unchanged_guard_entries(1)=[];run('missing_unchanged_guard',bad,g,'ORIGINAL','original','',false,0);
bad=g;bad.unchanged_guard_entries(1).raw_bits_hex='00000001';run('geometry_guard_conflict',t,bad,'ORIGINAL','original','',false,0);
bad=t;bad.entries(1).mav_type=6;run('tuning_contract_wrong_type',bad,g,'ORIGINAL','original','',false,0);
bad=g;bad.entries(1).mav_type=6;run('geometry_contract_wrong_type',t,bad,'ORIGINAL','original','',false,0);
bad=t;bad.entries(1).unexpected=true;run('unknown_tuning_entry_field',bad,g,'ORIGINAL','original','',false,0);
run('unknown_phase_zero_reads',t,g,'AUTOMATIC_RETRY','original','',false,0);
readCalls=0;r=m600check.verifyNativeHoverParameterUnion([],t,g,'ORIGINAL');
note('invalid_reader_zero_reads',~r.passed&&~isempty(r.failure)&&r.read_attempts==0,r);
result=struct('schema','HOST_NATIVE_HOVER_PARAMETER_UNION_TESTS_V1','passed',all([tests.passed]), ...
    'case_count',numel(tests),'cases_passed',sum([tests.passed]),'tests',tests, ...
    'source_sha256',m600check.fileSha256(which('m600check.verifyNativeHoverParameterUnion')), ...
    'test_sha256',m600check.fileSha256([mfilename('fullpath') '.m']), ...
    'hardware_actions',0,'COM_UDP_actions',0,'parameter_writes',0,'real_adapter_constructed',false);
mkdir(outputDir);f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);
closeFile=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));clear closeFile
assert(result.passed,'m600check:UnionTestsFailed','%s',strjoin({tests(~[tests.passed]).name},', '));
    function run(label,tc,gc,phase,poolPhase,readFault,expectedPass,expectedReads)
        store=containers.Map('KeyType','char','ValueType','any');
        for k=1:138,q=baseline(k).typed_value;store(q.name)=q;end
        if strcmp(poolPhase,'applied')
            for collection={g.entries,t.entries}
                e=collection{1};
                for k=1:numel(e),q=store(e(k).name);q.raw_bits_hex=e(k).target_raw_bits_hex;
                    q.decoded=double(typecast(uint32(hex2dec(q.raw_bits_hex)),'single'));store(e(k).name)=q;
                end
            end
        end
        fault=readFault;readCalls=0;readNames={};selected='MC_ROLL_P';
        if strcmp(fault,'guard_drift'),selected='MPC_TKO_RAMP_T';end
        if strcmp(fault,'geometry_drift'),selected='CA_ROTOR0_PX';end
        if strcmp(fault,'failure_detector_disabled'),selected='FD_FAIL_R';end
        if strcmp(fault,'hover_estimator_disabled'),selected='MPC_USE_HTE';end
        if strcmp(fault,'rate_I_drift'),selected='MC_ROLLRATE_I';end
        r=m600check.verifyNativeHoverParameterUnion(@reader,tc,gc,phase);
        okay=r.passed==expectedPass&&r.read_attempts==expectedReads&&readCalls==expectedReads&&r.parameter_writes==0;
        if expectedReads==138
            okay=okay&&numel(r.rows)==138&&all([r.rows.attempted])&&numel(unique(readNames))==138;
            if expectedPass,okay=okay&&all([r.rows.passed])&&isempty(r.failure);
            else,okay=okay&&any(~[r.rows.passed])&&~isempty(r.failure);end
        else,okay=okay&&~isempty(r.failure);end
        note(label,okay,r);
    end
    function p=reader(name)
        readCalls=readCalls+1;readNames{end+1}=name;p=store(name);
        if (strcmp(fault,'missing_first')&&readCalls==1)||(strcmp(fault,'missing_last')&&readCalls==138)
            error('fixture:MissingParameter','Synthetic missing parameter');
        end
        if ~strcmp(name,selected),return;end
        switch fault
            case 'wrong_name',p.name='OTHER';
            case 'wrong_type',p.mav_type=6;
            case {'wrong_bits','guard_drift','geometry_drift','tuning_drift','rate_I_drift'}
                p.raw_bits_hex=upper(dec2hex(bitxor(uint32(hex2dec(p.raw_bits_hex)),uint32(1)),8));
            case 'wrong_decoded',p.decoded=p.decoded+1;
            case 'decoded_nan',p.decoded=NaN;
            case 'decoded_vector',p.decoded=[p.decoded p.decoded];
            case 'raw_field_missing',p=rmfield(p,'raw_bits_hex');
            case 'reader_exception',error('fixture:ReadFailure','Synthetic reader failure');
            case {'failure_detector_disabled','hover_estimator_disabled'},p.raw_bits_hex='00000000';p.decoded=0;
        end
    end
    function note(name,pass,r)
        tests(end+1)=struct('name',name,'passed',logical(pass),'reader_calls',readCalls,'receipt',r); %#ok<AGROW>
    end
end
