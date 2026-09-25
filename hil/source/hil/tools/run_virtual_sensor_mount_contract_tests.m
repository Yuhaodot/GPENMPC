function result = run_virtual_sensor_mount_contract_tests(outputRoot)
% Test sensor-mount contracts with a mock runner.
buildRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(buildRoot,'m600_coptersim','matlab_validation'));
if nargin<1
    outputRoot=fullfile(buildRoot,'evidence', ...
        ['HOST_VIRTUAL_SENSOR_MOUNT_CONTRACT_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
end
assert(~isfolder(outputRoot),'m600check:ExistingEvidence','Output must be a new directory.');
mkdir(outputRoot);
cases={ ...
    'exact_pass','uint32_ids_pass','signed_id_bits_pass','bits_only_ids_pass', ...
    'uint32_nan_payload_pass','wrong_mount_type','wrong_mount_bits', ...
    'wrong_reported_value','missing_mount_bits','missing_mount_name', ...
    'reader_exception_mount','reader_exception_last_id','empty_row', ...
    'wrong_row_name','bad_bits','nonfinite_mount_bits','nonfinite_reported_id', ...
    'noninteger_reported_id','noninteger_id_type','noninteger_mav_type', ...
    'unsupported_id_type','missing_id_bits','wrong_uint32_decoded', ...
    'virtual_acc0','virtual_acc3','virtual_gyro0','virtual_gyro3', ...
    'virtual_mag0','virtual_mag3','virtual_uint32_id','contract_wrong_bits', ...
    'contract_wrong_device_id','contract_missing_field','invalid_reader'};
entries=repmat(struct('case','','passed',false,'expected_pass',false, ...
    'expected_reads',16,'read_names',{{}},'receipt',[]),numel(cases),1);
for k=1:numel(cases)
    whichCase=cases{k}; reads={}; base=fixture();
    contract=expectedContract(); reader=@readMock;
    expectedPass=ismember(whichCase,{'exact_pass','uint32_ids_pass', ...
        'signed_id_bits_pass','bits_only_ids_pass','uint32_nan_payload_pass'});
    expectedReads=16;
    switch whichCase
        case 'contract_wrong_bits',contract.mount(2).raw_bits_hex='00000000';expectedReads=0;
        case 'contract_wrong_device_id',contract.virtual_device_ids.acc=42;expectedReads=0;
        case 'contract_missing_field',contract=rmfield(contract,'mount');expectedReads=0;
        case 'invalid_reader',reader=[];expectedReads=0;
    end
    if strcmp(whichCase,'exact_pass')
        receipt=m600check.verifyVirtualSensorMountContract(reader);
    else
        receipt=m600check.verifyVirtualSensorMountContract(reader,contract);
    end
    okay=receipt.passed==expectedPass && ...
        receipt.read_attempt_count==expectedReads && numel(reads)==expectedReads && ...
        numel(receipt.rows)==expectedReads && receipt.parameter_write_requests==0;
    if expectedReads==16
        okay=okay&&isequal(reads,{base.name})&&all([receipt.rows.read_attempted]);
    end
    if ~expectedPass,okay=okay&&~isempty(receipt.failure);end
    if startsWith(whichCase,'virtual_')
        okay=okay&&any([receipt.rows.virtual_device_match])&& ...
            contains(receipt.failure,'VIRTUAL_DEVICE_CALIBRATION_PRESENT');
    end
    if startsWith(whichCase,'reader_exception_')
        okay=okay&&receipt.read_failure_count==1&&receipt.read_success_count==15&& ...
            any(contains({receipt.rows.failure},'READER_EXCEPTION:mock:ReadFailed'));
    end
    entries(k)=struct('case',whichCase,'passed',okay,'expected_pass',expectedPass, ...
        'expected_reads',expectedReads,'read_names',{reads},'receipt',receipt);
end
legacyEntries=entries;
[v2Entries,v2Contract,sourceChecks]=runV2Cases();
entries=[legacyEntries;v2Entries];
result=struct('schema','VIRTUAL_SENSOR_MOUNT_HOST_TEST_V2_AUTOCAL_OBSERVED', ...
    'classification','HOST_ONLY_MOCK_VALIDATION__V2_DISARMED_OBSERVATION_ONLY', ...
    'passed',all([entries.passed])&&all([sourceChecks.passed]), ...
    'case_count',numel(entries),'cases_passed',sum([entries.passed]),'cases',entries, ...
    'legacy_case_count',numel(legacyEntries),'legacy_cases_passed',sum([legacyEntries.passed]), ...
    'v2_case_count',numel(v2Entries),'v2_cases_passed',sum([v2Entries.passed]), ...
    'v2_contract',v2Contract,'observation_contract',v2Contract,'source_checks',sourceChecks, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'parameter_writes',0, ...
    'source',which('m600check.verifyVirtualSensorMountContract'), ...
    'source_sha256',fileSha256(which('m600check.verifyVirtualSensorMountContract')), ...
    'test_source_sha256',fileSha256([mfilename('fullpath') '.m']), ...
    'test_source',mfilename('fullpath'),'created_utc',char(datetime('now','TimeZone','UTC')));
save(fullfile(outputRoot,'HOST_TEST_RESULT.mat'),'result');
fid=fopen(fullfile(outputRoot,'HOST_TEST_RESULT.json'),'w','n','UTF-8');
assert(fid>=0,'m600check:EvidenceOpen','Cannot open result.'); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(result,PrettyPrint=true)); clear cleanup
fprintf('VIRTUAL_SENSOR_MOUNT_CONTRACT: %d/%d PASS; hardware=0\n',result.cases_passed,result.case_count);
assert(result.passed,'m600check:MockValidation','One or more mock cases failed.');

    function r=readMock(name)
        reads{end+1}=name;
        r=base(strcmp({base.name},name));
        if strcmp(whichCase,'reader_exception_mount')&&strcmp(name,'SENS_BOARD_X_OFF') || ...
                strcmp(whichCase,'reader_exception_last_id')&&strcmp(name,'CAL_MAG3_ID')
            error('mock:ReadFailed','Injected read timeout; no transport exists.');
        end
        isId=startsWith(name,'CAL_');
        if strcmp(whichCase,'uint32_ids_pass')&&isId,r.mav_type=5;end
        if strcmp(whichCase,'signed_id_bits_pass')&&strcmp(name,'CAL_ACC1_ID')
            r.raw_bits_hex='FFFFFFFF';r.decoded=-1;
        end
        if strcmp(whichCase,'bits_only_ids_pass')&&isId,r=rmfield(r,'decoded');end
        if strcmp(whichCase,'uint32_nan_payload_pass')&&strcmp(name,'CAL_ACC1_ID')
            r.mav_type=5;r.raw_bits_hex='7FC00001';r.decoded=double(hex2dec(r.raw_bits_hex));
            r.raw_float=typecast(uint32(hex2dec(r.raw_bits_hex)),'single');
        end
        if strcmp(name,'SENS_BOARD_X_OFF')
            switch whichCase
                case 'wrong_mount_type',r.mav_type=6;r.decoded=double(typecast(uint32(hex2dec(r.raw_bits_hex)),'int32'));
                case 'wrong_mount_bits',r.raw_bits_hex='00000000';r.decoded=0;
                case 'wrong_reported_value',r.decoded=0;
                case 'missing_mount_bits',r=rmfield(r,'raw_bits_hex');
                case 'missing_mount_name',r=rmfield(r,'name');
                case 'empty_row',r=[];
                case 'wrong_row_name',r.name='CAL_ACC0_ID';
                case 'bad_bits',r.raw_bits_hex='nothex00';
                case 'nonfinite_mount_bits',r.raw_bits_hex='7FC00000';r.decoded=NaN;
            end
        end
        if strcmp(name,'CAL_ACC2_ID')
            switch whichCase
                case 'nonfinite_reported_id',r.decoded=NaN;
                case 'noninteger_reported_id',r.decoded=0.5;
                case 'noninteger_id_type',r.mav_type=9;r.raw_bits_hex='3F000000';r.decoded=0.5;
                case 'noninteger_mav_type',r.mav_type=6.5;
                case 'unsupported_id_type',r.mav_type=4;
                case 'missing_id_bits',r=rmfield(r,'raw_bits_hex');
                case 'wrong_uint32_decoded',r.mav_type=5;r.raw_bits_hex='00000001';r.decoded=double(typecast(uint32(1),'single'));
            end
        end
        targets={'virtual_acc0','CAL_ACC0_ID',1310988;'virtual_acc3','CAL_ACC3_ID',1310988; ...
            'virtual_gyro0','CAL_GYRO0_ID',1310988;'virtual_gyro3','CAL_GYRO3_ID',1310988; ...
            'virtual_mag0','CAL_MAG0_ID',197388;'virtual_mag3','CAL_MAG3_ID',197388; ...
            'virtual_uint32_id','CAL_MAG2_ID',197388};
        for j=1:size(targets,1)
            if strcmp(whichCase,targets{j,1})&&strcmp(name,targets{j,2})
                r.decoded=targets{j,3};r.raw_bits_hex=dec2hex(uint32(r.decoded),8);
                if strcmp(whichCase,'virtual_uint32_id'),r.mav_type=5;end
            end
        end
    end
end

function [entries,contract,sourceChecks]=runV2Cases()
probe=m600check.verifyVirtualSensorMountContract([], ...
    struct('schema','VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED'));
contract=probe.contract;
sourceChecks=repmat(struct('path','','sha256','','passed',false),2,1);
for k=1:2
    actual=fileSha256(contract.sources(k).path);
    sourceChecks(k)=struct('path',contract.sources(k).path,'sha256',actual, ...
        'passed',strcmp(actual,contract.sources(k).sha256));
end
assert(all([sourceChecks.passed]),'m600check:SourceIdentity','Actual readonly source changed.');
pre=jsondecode(fileread(contract.sources(1).path));
cal=jsondecode(fileread(contract.sources(2).path));
assert(cal.passed && cal.parameter_writes==0 && cal.mapping_writes==0, ...
    'm600check:SourceClaim','Calibration source is not the accepted readonly receipt.');
base=cell(1,numel(contract.parameters));
for k=1:numel(base)
    name=contract.parameters(k).name;
    ix=find(strcmp({cal.rows.name},name));
    if ~isempty(ix)
        assert(isscalar(ix),'m600check:SourceDuplicate','Duplicate source parameter.');
        base{k}=cal.rows(ix).value;
    else
        ix=find(strcmp({pre.virtual_sensor_mount_check.rows.name},name));
        assert(isscalar(ix),'m600check:SourceMissing','Missing actual slot evidence.');
        base{k}=pre.virtual_sensor_mount_check.rows(ix).raw_row;
    end
end
names={contract.parameters.name};
cases={'v2_actual_readonly_pass','v2_dynamic_gyro_pass','v2_dynamic_mag_pass', ...
    'v2_dynamic_all_pass','v2_json_roundtrip_pass','v2_wrong_mount','v2_wrong_mount_type', ...
    'v2_duplicate_gyro_slot','v2_duplicate_mag_slot','v2_unexpected_acc_slot', ...
    'v2_missing_sim_gyro_id','v2_changed_physical_slot', ...
    'v2_nonfinite_gyro_bias','v2_nonfinite_mag_bias','v2_missing_bias_bits','v2_wrong_bias_type', ...
    'v2_bad_xscale','v2_bad_yscale','v2_bad_zscale','v2_nonzero_offdiagonal', ...
    'v2_nonzero_comp','v2_comp_enabled','v2_thermal_enabled','v2_thermal_id_changed', ...
    'v2_bad_gyro_rot','v2_bad_mag_rot','v2_bad_gyro_priority','v2_bad_mag_priority', ...
    'v2_gyro_autocal_disabled','v2_mag_autocal_disabled','v2_mag_custom_roll_changed', ...
    'v2_reader_exception_last','v2_first_failure_retained','v2_contract_wrong_scope', ...
    'v2_contract_flight_true','v2_contract_source_changed','v2_contract_slot_changed', ...
    'v2_contract_unknown_schema','v2_reader_exception_bias', ...
    'v2_json_contract_numeric_changed','v2_json_contract_unknown_field'};
entries=repmat(struct('case','','passed',false,'expected_pass',false, ...
    'expected_reads',46,'read_names',{{}},'receipt',[]),numel(cases),1);
for i=1:numel(cases)
    whichCase=cases{i}; reads={}; c=contract; expectedReads=numel(names);
    expectedPass=ismember(whichCase,cases(1:5));
    switch whichCase
        case 'v2_json_roundtrip_pass',c=jsondecode(jsonencode(c));
        case 'v2_contract_wrong_scope',c.admission_scope='FLIGHT';expectedReads=0;
        case 'v2_contract_flight_true',c.flight_admission=true;expectedReads=0;
        case 'v2_contract_source_changed',c.sources(2).sha256=repmat('0',1,64);expectedReads=0;
        case 'v2_contract_slot_changed',c.virtual_slots.gyro=3;expectedReads=0;
        case 'v2_contract_unknown_schema',c.schema='UNKNOWN';expectedReads=0;
        case 'v2_json_contract_numeric_changed'
            c=jsondecode(jsonencode(c));c.parameters(1).mav_type=5;expectedReads=0;
        case 'v2_json_contract_unknown_field'
            c=jsondecode(jsonencode(c));c.allow_unknown_calibration=true;expectedReads=0;
    end
    receipt=m600check.verifyVirtualSensorMountContract(@readV2,c);
    okay=receipt.passed==expectedPass && receipt.read_attempt_count==expectedReads && ...
        numel(receipt.rows)==expectedReads && numel(reads)==expectedReads && ...
        receipt.parameter_write_requests==0;
    if ~strcmp(whichCase,'v2_contract_unknown_schema')
        okay=okay&&~receipt.flight_admission&&~receipt.offset_compensation_applied&& ...
            ~receipt.full_sensor_equality_claimed&&receipt.observation_admitted==expectedPass;
    end
    if expectedReads>0
        okay=okay&&isequal(reads,names)&&all([receipt.rows.read_attempted]);
    end
    if ~expectedPass,okay=okay&&~isempty(receipt.failure);end
    if expectedPass
        okay=okay&&numel(receipt.dynamic_bias_rows)==6&& ...
            strcmp(receipt.classification,'PASS_DISARMED_SENSOR_OBSERVATION_ONLY__NATIVE_AUTOCAL_BIAS_RETAINED');
    end
    if strcmp(whichCase,'v2_actual_readonly_pass')
        okay=okay&&isempty(receipt.dynamic_bias_changed_names)&& ...
            any(abs(cellfun(@(r)r.decoded,receipt.dynamic_bias_rows))>0);
    elseif strcmp(whichCase,'v2_dynamic_gyro_pass')||strcmp(whichCase,'v2_dynamic_mag_pass')
        okay=okay&&numel(receipt.dynamic_bias_changed_names)==1;
    elseif strcmp(whichCase,'v2_dynamic_all_pass')
        okay=okay&&numel(receipt.dynamic_bias_changed_names)==6;
    elseif strcmp(whichCase,'v2_first_failure_retained')
        okay=okay&&startsWith(receipt.failure,'SENS_BOARD_ROT:')&& ...
            sum(~[receipt.rows.passed])==2&&strcmp(receipt.rows(end).name,'TC_G3_ID');
    elseif startsWith(whichCase,'v2_reader_exception')
        okay=okay&&receipt.read_failure_count==1&&receipt.read_success_count==45;
    end
    entries(i)=struct('case',whichCase,'passed',okay,'expected_pass',expectedPass, ...
        'expected_reads',expectedReads,'read_names',{reads},'receipt',receipt);
end

    function row=readV2(name)
        reads{end+1}=name; ix=find(strcmp(names,name));
        assert(isscalar(ix),'mock:UnexpectedRead','Read outside fixed contract.'); row=base{ix};
        if strcmp(whichCase,'v2_reader_exception_last')&&strcmp(name,'TC_G3_ID') || ...
                strcmp(whichCase,'v2_reader_exception_bias')&&strcmp(name,'CAL_GYRO2_YOFF')
            error('mock:ReadFailed','Injected HOST-only read error.');
        end
        if strcmp(whichCase,'v2_dynamic_gyro_pass')&&strcmp(name,'CAL_GYRO2_XOFF') || ...
                strcmp(whichCase,'v2_dynamic_mag_pass')&&strcmp(name,'CAL_MAG1_ZOFF') || ...
                strcmp(whichCase,'v2_dynamic_all_pass')&&strcmp(contract.parameters(ix).policy,'NATIVE_DYNAMIC_FINITE_BIAS')
            row=withValue(row,9,-0.0125);return
        end
        mutations={ ...
            'v2_wrong_mount','SENS_BOARD_X_OFF',9,0; ...
            'v2_wrong_mount_type','SENS_BOARD_ROT',5,0; ...
            'v2_duplicate_gyro_slot','CAL_GYRO3_ID',6,1310988; ...
            'v2_duplicate_mag_slot','CAL_MAG0_ID',6,197388; ...
            'v2_unexpected_acc_slot','CAL_ACC2_ID',6,1310988; ...
            'v2_missing_sim_gyro_id','CAL_GYRO2_ID',6,0; ...
            'v2_changed_physical_slot','CAL_ACC0_ID',6,1234; ...
            'v2_nonfinite_gyro_bias','CAL_GYRO2_XOFF',9,NaN; ...
            'v2_nonfinite_mag_bias','CAL_MAG1_YOFF',9,Inf; ...
            'v2_wrong_bias_type','CAL_GYRO2_ZOFF',6,0; ...
            'v2_bad_xscale','CAL_MAG1_XSCALE',9,1.1; ...
            'v2_bad_yscale','CAL_MAG1_YSCALE',9,0; ...
            'v2_bad_zscale','CAL_MAG1_ZSCALE',9,-1; ...
            'v2_nonzero_offdiagonal','CAL_MAG1_YODIAG',9,0.02; ...
            'v2_nonzero_comp','CAL_MAG1_XCOMP',9,0.01; ...
            'v2_comp_enabled','CAL_MAG_COMP_TYP',6,1; ...
            'v2_thermal_enabled','TC_G_ENABLE',6,1; ...
            'v2_thermal_id_changed','TC_G2_ID',6,1310988; ...
            'v2_bad_gyro_rot','CAL_GYRO2_ROT',6,0; ...
            'v2_bad_mag_rot','CAL_MAG1_ROT',6,0; ...
            'v2_bad_gyro_priority','CAL_GYRO2_PRIO',6,51; ...
            'v2_bad_mag_priority','CAL_MAG1_PRIO',6,0; ...
            'v2_gyro_autocal_disabled','IMU_GYRO_CAL_EN',6,0; ...
            'v2_mag_autocal_disabled','SENS_MAG_AUTOCAL',6,0; ...
            'v2_mag_custom_roll_changed','CAL_MAG1_ROLL',9,0.1; ...
            'v2_first_failure_retained','SENS_BOARD_ROT',6,1; ...
            'v2_first_failure_retained','TC_G_ENABLE',6,1};
        for j=1:size(mutations,1)
            if strcmp(whichCase,mutations{j,1})&&strcmp(name,mutations{j,2})
                row=withValue(row,mutations{j,3},mutations{j,4});
            end
        end
        if strcmp(whichCase,'v2_missing_bias_bits')&&strcmp(name,'CAL_MAG1_ZOFF')
            row=rmfield(row,'raw_bits_hex');
        end
    end
end

function row=withValue(row,t,value)
row.mav_type=t;
if t==9,bits=typecast(single(value),'uint32');row.decoded=double(single(value));
elseif t==6,bits=typecast(int32(value),'uint32');row.decoded=double(int32(value));
else,bits=uint32(value);row.decoded=double(bits);end
row.raw_bits_hex=dec2hex(bits,8);row.raw_float=double(typecast(bits,'single'));
end

function h=fileSha256(path)
fid=fopen(path,'rb');assert(fid>=0,'m600check:SourceOpen','Cannot open source file.');
cleanup=onCleanup(@()fclose(fid)); bytes=fread(fid,Inf,'*uint8');clear cleanup
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
word=typecast(digest.digest(),'uint8');h=upper(reshape(dec2hex(word,2).',1,[]));
end

function c=expectedContract()
c=struct('schema','VIRTUAL_SENSOR_MOUNT_CONTRACT_V1', ...
    'mount',struct('name',{'SENS_BOARD_ROT','SENS_BOARD_X_OFF','SENS_BOARD_Y_OFF','SENS_BOARD_Z_OFF'}, ...
    'mav_type',{6,9,9,9},'raw_bits_hex',{'00000000','40CE407F','C0CE28B3','00000000'}), ...
    'virtual_device_ids',struct('acc',1310988,'gyro',1310988,'mag',197388));
end

function rows=fixture()
c=expectedContract();
rows=repmat(struct('name','','mav_type',6,'raw_bits_hex','00000000','decoded',0),1,16);
for k=1:4
    rows(k).name=c.mount(k).name;rows(k).mav_type=c.mount(k).mav_type;
    rows(k).raw_bits_hex=c.mount(k).raw_bits_hex;
    word=uint32(hex2dec(rows(k).raw_bits_hex));
    if rows(k).mav_type==9,rows(k).decoded=double(typecast(word,'single'));end
end
groups={'ACC','GYRO','MAG'};k=4;
for g=1:3
    for slot=0:3
        k=k+1;rows(k).name=sprintf('CAL_%s%d_ID',groups{g},slot);
        if slot==0,rows(k).decoded=6946826+g-1;rows(k).raw_bits_hex=dec2hex(uint32(rows(k).decoded),8);end
    end
end
end
