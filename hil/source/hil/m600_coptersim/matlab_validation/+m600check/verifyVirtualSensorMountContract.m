function receipt = verifyVirtualSensorMountContract(reader, contract)
%VERIFYVIRTUALSENSORMOUNTCONTRACT Read-only, exact mount/calibration check.
% reader(name) returns a scalar typed PARAM_VALUE row with name, mav_type,
% raw_bits_hex and optionally decoded. UINT32/INT32 values are decoded from
% their raw bits, never from the IEEE single payload's numeric value.
% Every one of the 16 reads is attempted once, including after a failed row.
% V2 explicitly permits only disarmed observation of the audited native
% autocalibration slots. It never establishes flight admission or removes
% calibration offsets from the sensor data.
if nargin >= 2 && isstruct(contract) && isscalar(contract) && ...
        isfield(contract,'schema') && isTextScalar(contract.schema) && ...
        strcmp(contract.schema,'VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED')
    receipt = verifyAutocalObserved(reader,contract); return
end
expected = defaultContract();
if nargin < 2, contract = expected; end
receipt = struct('passed',false,'failure','','contract',contract, ...
    'rows',repmat(emptyRow(),0,1),'read_attempt_count',0, ...
    'read_success_count',0,'read_failure_count',0, ...
    'parameter_write_requests',0,'scope','EXACT_MOUNT_AND_ALL_SIM_CALIBRATION_SLOTS');
if nargin < 1 || ~isa(reader,'function_handle')
    receipt.failure = 'READER_NOT_FUNCTION_HANDLE'; return
end
if ~validContract(contract,expected)
    receipt.failure = 'CONTRACT_IDENTITY_MISMATCH'; return
end
names = {expected.mount.name};
kinds = repmat({'mount'},1,4);
simIds = zeros(1,4);
groups = {'ACC','GYRO','MAG'};
fields = {'acc','gyro','mag'};
for g = 1:3
    for slot = 0:3
        names{end+1} = sprintf('CAL_%s%d_ID',groups{g},slot); %#ok<AGROW>
        kinds{end+1} = 'calibration_id'; %#ok<AGROW>
        simIds(end+1) = expected.virtual_device_ids.(fields{g}); %#ok<AGROW>
    end
end
for k = 1:numel(names)
    row = emptyRow(); row.name = names{k}; row.kind = kinds{k};
    row.read_attempted = true;
    receipt.read_attempt_count = receipt.read_attempt_count + 1;
    try
        raw = reader(names{k});
        row.read_succeeded = true; row.raw_row = raw;
        receipt.read_success_count = receipt.read_success_count + 1;
        [row.mav_type,row.raw_bits_hex,row.decoded,row.unsigned_bits,row.failure] = ...
            decodeRow(raw,names{k});
        if isempty(row.failure)
            if k <= 4
                target = expected.mount(k);
                if row.mav_type ~= target.mav_type
                    row.failure = 'MOUNT_TYPE_MISMATCH';
                elseif ~strcmp(row.raw_bits_hex,target.raw_bits_hex)
                    row.failure = 'MOUNT_RAW_BITS_MISMATCH';
                end
            elseif ~ismember(row.mav_type,[5 6])
                row.failure = 'CALIBRATION_ID_NOT_INT32_OR_UINT32';
            elseif row.unsigned_bits == simIds(k)
                row.virtual_device_match = true;
                row.failure = 'VIRTUAL_DEVICE_CALIBRATION_PRESENT';
            end
        end
    catch problem
        row.failure = ['READER_EXCEPTION:' problem.identifier];
        row.reader_exception_identifier = problem.identifier;
        row.reader_exception_message = problem.message;
    end
    row.passed = isempty(row.failure);
    if ~row.passed && isempty(receipt.failure)
        receipt.failure = [row.name ':' row.failure];
    end
    receipt.rows(end+1,1) = row; %#ok<AGROW>
end
receipt.read_failure_count = receipt.read_attempt_count - receipt.read_success_count;
receipt.passed = receipt.read_attempt_count == 16 && all([receipt.rows.passed]);
end

function c = defaultContract()
names = {'SENS_BOARD_ROT','SENS_BOARD_X_OFF','SENS_BOARD_Y_OFF','SENS_BOARD_Z_OFF'};
types = {6,9,9,9};
bits = {'00000000','40CE407F','C0CE28B3','00000000'};
c = struct('schema','VIRTUAL_SENSOR_MOUNT_CONTRACT_V1', ...
    'mount',struct('name',names,'mav_type',types,'raw_bits_hex',bits), ...
    'virtual_device_ids',struct('acc',1310988,'gyro',1310988,'mag',197388));
end

function okay = validContract(c,e)
okay = false;
try
    if ~isstruct(c) || ~isscalar(c) || ~strcmp(c.schema,e.schema) || ...
            ~isstruct(c.mount) || numel(c.mount) ~= 4, return; end
    for k = 1:4
        if ~strcmp(c.mount(k).name,e.mount(k).name) || ...
                ~isequal(double(c.mount(k).mav_type),e.mount(k).mav_type) || ...
                ~strcmp(c.mount(k).raw_bits_hex,e.mount(k).raw_bits_hex), return; end
    end
    for f = {'acc','gyro','mag'}
        if ~isequal(double(c.virtual_device_ids.(f{1})),e.virtual_device_ids.(f{1})), return; end
    end
    okay = true;
catch
    % Malformed caller contracts are rejected before the first reader call.
end
end

function r = emptyRow()
r = struct('name','','kind','','read_attempted',false,'read_succeeded',false, ...
    'raw_row',[],'mav_type',NaN,'raw_bits_hex','','decoded',NaN, ...
    'unsigned_bits',NaN,'virtual_device_match',false,'passed',false, ...
    'failure','','reader_exception_identifier','','reader_exception_message','');
end

function [t,bits,value,u,failure] = decodeRow(raw,name)
t=NaN; bits=''; value=NaN; u=NaN; failure='';
if ~isstruct(raw) || ~isscalar(raw)
    failure='ROW_NOT_SCALAR_STRUCT'; return
end
if ~all(isfield(raw,{'name','mav_type','raw_bits_hex'}))
    failure='REQUIRED_TYPED_FIELD_MISSING'; return
end
if ~isTextScalar(raw.name) || ~strcmp(char(raw.name),name)
    failure='PARAMETER_NAME_MISMATCH'; return
end
if ~isFiniteScalar(raw.mav_type) || double(raw.mav_type) ~= fix(double(raw.mav_type))
    failure='MAV_TYPE_NOT_FINITE_INTEGER'; return
end
t=double(raw.mav_type);
if ~ismember(t,[5 6 9])
    failure='UNSUPPORTED_MAV_TYPE'; return
end
if ~isTextScalar(raw.raw_bits_hex)
    failure='RAW_BITS_NOT_EIGHT_HEX_DIGITS'; return
end
bits=upper(char(raw.raw_bits_hex));
if isempty(regexp(bits,'^[0-9A-F]{8}$','once'))
    failure='RAW_BITS_NOT_EIGHT_HEX_DIGITS'; return
end
u=hex2dec(bits); word=uint32(u);
if t==5, value=double(word);
elseif t==6, value=double(typecast(word,'int32'));
else, value=double(typecast(word,'single'));
end
if ~isfinite(value)
    failure='DECODED_VALUE_NONFINITE'; return
end
if isfield(raw,'decoded')
    if ~isFiniteScalar(raw.decoded)
        failure='REPORTED_DECODED_VALUE_NONFINITE_OR_INVALID'; return
    elseif double(raw.decoded) ~= value
        failure='REPORTED_DECODED_RAW_BITS_MISMATCH'; return
    end
end
if ismember(t,[5 6]) && value ~= fix(value)
    failure='DECODED_ID_NOT_INTEGER';
end
end

function okay=isFiniteScalar(v)
okay=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(double(v));
end

function okay=isTextScalar(v)
okay=(ischar(v)&&isrow(v))||(isstring(v)&&isscalar(v)&&~ismissing(v));
end

function receipt=verifyAutocalObserved(reader,contract)
expected=autocalContract();
receipt=struct('schema',expected.schema, ...
    'classification','DISARMED_SENSOR_OBSERVATION_CONTRACT_REJECTED', ...
    'passed',false,'observation_admitted',false,'flight_admission',false, ...
    'failure','','contract',expected,'submitted_contract',contract, ...
    'rows',repmat(emptyRow(),0,1),'read_attempt_count',0, ...
    'read_success_count',0,'read_failure_count',0, ...
    'parameter_write_requests',0,'scope',expected.admission_scope, ...
    'offset_compensation_applied',false,'full_sensor_equality_claimed',false, ...
    'dynamic_bias_rows',{{}},'dynamic_bias_changed_names',{{}});
if ~isa(reader,'function_handle')
    receipt.failure='READER_NOT_FUNCTION_HANDLE'; return
end
% Exact structure is intentional: a caller cannot broaden V2 into flight,
% accept another slot, disable autocal/thermal checks, or change provenance.
if ~sameAutocalContract(contract,expected)
    receipt.failure='CONTRACT_IDENTITY_MISMATCH'; return
end
for k=1:numel(expected.parameters)
    p=expected.parameters(k); row=emptyRow(); row.name=p.name; row.kind=p.policy;
    row.read_attempted=true; receipt.read_attempt_count=receipt.read_attempt_count+1;
    try
        raw=reader(p.name); row.read_succeeded=true; row.raw_row=raw;
        receipt.read_success_count=receipt.read_success_count+1;
        [row.mav_type,row.raw_bits_hex,row.decoded,row.unsigned_bits,row.failure]= ...
            decodeRow(raw,p.name);
        if isempty(row.failure)
            if row.mav_type~=p.mav_type
                row.failure='PARAMETER_TYPE_MISMATCH';
            elseif strcmp(p.policy,'NATIVE_DYNAMIC_FINITE_BIAS')
                receipt.dynamic_bias_rows{end+1}=raw;
                if ~strcmp(row.raw_bits_hex,p.reference_raw_bits_hex)
                    receipt.dynamic_bias_changed_names{end+1}=p.name;
                end
            elseif ~strcmp(row.raw_bits_hex,p.reference_raw_bits_hex)
                row.failure=v2Mismatch(p.name,row.unsigned_bits,expected.virtual_device_ids);
            end
            if endsWith(p.name,'_ID') && startsWith(p.name,'CAL_')
                if startsWith(p.name,'CAL_ACC'),sim=expected.virtual_device_ids.acc;
                elseif startsWith(p.name,'CAL_GYRO'),sim=expected.virtual_device_ids.gyro;
                else,sim=expected.virtual_device_ids.mag;end
                row.virtual_device_match=row.unsigned_bits==sim;
            end
        end
    catch problem
        row.failure=['READER_EXCEPTION:' problem.identifier];
        row.reader_exception_identifier=problem.identifier;
        row.reader_exception_message=problem.message;
    end
    row.passed=isempty(row.failure);
    if ~row.passed && isempty(receipt.failure)
        receipt.failure=[row.name ':' row.failure];
    end
    receipt.rows(end+1,1)=row; %#ok<AGROW>
end
receipt.read_failure_count=receipt.read_attempt_count-receipt.read_success_count;
receipt.passed=receipt.read_attempt_count==numel(expected.parameters)&&all([receipt.rows.passed]);
receipt.observation_admitted=receipt.passed;
if receipt.passed
    receipt.classification='PASS_DISARMED_SENSOR_OBSERVATION_ONLY__NATIVE_AUTOCAL_BIAS_RETAINED';
end
% Native bias correction remains active. With wire=L''*body and thermal=0,
% gyro corrected=body-L*offset; mag corrected=body-L*offset when S=I and
% power compensation=0; native bias offsets remain in the corrected output.
receipt.gyro_corrected_semantics='L*(raw-offset); TC_G_ENABLE=0';
receipt.mag_corrected_semantics='L*(S*((raw+power*COMP)-offset)); S=I, COMP=0';
receipt.dynamic_bias_policy='FINITE_NATIVE_AUTOCAL_OBSERVED_NOT_RAW_BITS_LOCKED_NOT_COMPENSATED';
receipt.requires_disarmed_outer_guard=true;
receipt.requires_independent_attitude_and_estimator_observation=true;
end

function same=sameAutocalContract(actual,expected)
same=false;
try
    % JSON round trips encode a struct vector identically whether MATLAB
    % stores it as 1xN or Nx1. Normalize only these declared vector fields;
    % retain every field, exact raw bits/value, and unknown-field rejection.
    for field={'mount','sources','parameters'}
        f=field{1}; a=actual.(f); e=expected.(f);
        if ~isstruct(a)||~isvector(a)||numel(a)~=numel(e),return;end
        actual.(f)=a(:);expected.(f)=e(:);
    end
    same=isequaln(actual,expected);
catch
    % Missing or malformed contract fields are rejected before any read.
end
end

function reason=v2Mismatch(name,u,ids)
if startsWith(name,'SENS_BOARD_'),reason='MOUNT_RAW_BITS_MISMATCH';
elseif endsWith(name,'_ID')&&startsWith(name,'CAL_')
    if ismember(u,[ids.acc ids.gyro ids.mag]),reason='UNEXPECTED_OR_DUPLICATE_SIM_CALIBRATION_SLOT';
    else,reason='CALIBRATION_SLOT_IDENTITY_MISMATCH';end
elseif contains(name,'SCALE'),reason='MAG_SCALE_NOT_AUDITED_IDENTITY';
elseif contains(name,'ODIAG'),reason='MAG_OFFDIAGONAL_NOT_ZERO';
elseif contains(name,'COMP'),reason='MAG_COMPENSATION_NOT_DISABLED';
elseif startsWith(name,'TC_G'),reason='THERMAL_CORRECTION_NOT_AUDITED_DISABLED';
elseif endsWith(name,'_PRIO'),reason='CALIBRATION_PRIORITY_IDENTITY_MISMATCH';
elseif endsWith(name,'_ROT'),reason='INTERNAL_ROTATION_METADATA_MISMATCH';
elseif contains(name,'AUTOCAL')||strcmp(name,'IMU_GYRO_CAL_EN'),reason='NATIVE_AUTOCAL_CONFIGURATION_MISMATCH';
else,reason='AUDITED_PARAMETER_IDENTITY_MISMATCH';end
end

function c=autocalContract()
c=defaultContract(); c.schema='VIRTUAL_SENSOR_MOUNT_CONTRACT_V2_AUTOCAL_OBSERVED';
c.admission_scope='DISARMED_SENSOR_OBSERVATION_ONLY';
c.flight_admission=false; c.parameter_writes_allowed=false;
c.virtual_slots=struct('acc',[],'gyro',2,'mag',1);
runRoot=gpenmpc_external_path('sensor_mount_reference_observation');
c.sources=struct('path',{[runRoot '\SERIAL_PREFLIGHT.json'], ...
    [runRoot '\SIM_CALIBRATION_READONLY\SIM_CALIBRATION.json']}, ...
    'sha256',{'ECA7A1780BB5410C2E75D9E05B26D4BDC71B73CBCC0C90A5658EA375D8B82614', ...
    '33F25E8B7F246104C7E40A14F7C3ABBFAC42095CC829414D18DA67027A598D39'});
p=repmat(struct('name','','mav_type',6,'policy','EXACT_AUDITED_IDENTITY', ...
    'reference_raw_bits_hex',''),0,1);
for k=1:4,p(end+1,1)=spec(c.mount(k).name,c.mount(k).mav_type,c.mount(k).raw_bits_hex);end %#ok<AGROW>
groups={'ACC','GYRO','MAG'};
slotBits={{'006A000A','0026000A','00000000','00000000'}, ...
    {'0066000A','0026000A','0014010C','00000000'}, ...
    {'00060C21','0003030C','00000000','00000000'}};
for g=1:3
    for slot=0:3,p(end+1,1)=spec(sprintf('CAL_%s%d_ID',groups{g},slot),6,slotBits{g}{slot+1});end %#ok<AGROW>
end
extra={ ...
    'CAL_GYRO2_XOFF',9,'35FF8A30';'CAL_GYRO2_YOFF',9,'362C44F4'; ...
    'CAL_GYRO2_ZOFF',9,'36BA384B';'CAL_GYRO2_PRIO',6,'00000032';'CAL_GYRO2_ROT',6,'FFFFFFFF'; ...
    'CAL_MAG1_XOFF',9,'3BE34B20';'CAL_MAG1_YOFF',9,'3A1D1680';'CAL_MAG1_ZOFF',9,'3C26DFC6'; ...
    'CAL_MAG1_XSCALE',9,'3F800000';'CAL_MAG1_YSCALE',9,'3F800000';'CAL_MAG1_ZSCALE',9,'3F800000'; ...
    'CAL_MAG1_XODIAG',9,'00000000';'CAL_MAG1_YODIAG',9,'00000000';'CAL_MAG1_ZODIAG',9,'00000000'; ...
    'CAL_MAG1_XCOMP',9,'00000000';'CAL_MAG1_YCOMP',9,'00000000';'CAL_MAG1_ZCOMP',9,'00000000'; ...
    'CAL_MAG1_ROT',6,'FFFFFFFF';'CAL_MAG1_PRIO',6,'00000032'; ...
    'CAL_MAG1_ROLL',9,'00000000';'CAL_MAG1_PITCH',9,'00000000';'CAL_MAG1_YAW',9,'00000000'; ...
    'CAL_MAG_COMP_TYP',6,'00000000';'IMU_GYRO_CAL_EN',6,'00000001';'SENS_MAG_AUTOCAL',6,'00000001'; ...
    'TC_G_ENABLE',6,'00000000';'TC_G0_ID',6,'00000000';'TC_G1_ID',6,'00000000'; ...
    'TC_G2_ID',6,'00000000';'TC_G3_ID',6,'00000000'};
for k=1:size(extra,1)
    q=spec(extra{k,1},extra{k,2},extra{k,3});
    if endsWith(q.name,'OFF'),q.policy='NATIVE_DYNAMIC_FINITE_BIAS';end
    p(end+1,1)=q; %#ok<AGROW>
end
c.parameters=p;
c.offset_policy='NATIVE_DYNAMIC_FINITE_OBSERVED_NO_WIRE_COMPENSATION';
c.source_semantics='PX4_SIMULATION_INTERNAL_DEVICE__L_FROM_BOARD_LEVEL__GYRO_AND_MAG_NATIVE_AUTOCAL';
end

function p=spec(name,t,bits)
p=struct('name',name,'mav_type',t,'policy','EXACT_AUDITED_IDENTITY','reference_raw_bits_hex',bits);
end
