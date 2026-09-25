function receipt=transferNativeHoverTuning(io,entries,mode,safeGuard)
% Exact, separately accounted three-parameter calibration service.
% No sockets/authority are created here. APPLY requires all original values;
% RESTORE proceeds independently only for known original/candidate values.
% Full 138-field and disjoint geometry checks belong BEFORE all mutations.
% Setter calls count invocation attempts; the adapter records transmitted
% PARAM_SET messages and the service verifies acknowledgements/readback.
receipt=struct('schema','M600_NATIVE_HOVER_TUNING_TRANSFER_V1','mode','', ...
    'passed',false,'contract_valid',false,'preflight_passed',false, ...
    'entries',[],'preflight',[],'actions',[],'final_readback',[], ...
    'first_error',[],'errors',{{}},'read_attempt_count',0, ...
    'setter_invocation_attempt_count',0,'setter_return_count',0, ...
    'ack_and_readback_verified_count',0,'no_op_count',0, ...
    'final_identity_verified',false,'adapter_real_parameter_actions',[], ...
    'adapter_evidence_capture_error','','transport_created',false, ...
    'arm_disarm_mode_requests',0);
try
    assert(isstruct(io)&&isscalar(io)&&isfield(io,'readParameter')&& ...
        isa(io.readParameter,'function_handle')&&isfield(io,'setNativeHoverTuningParameter')&& ...
        isa(io.setNativeHoverTuningParameter,'function_handle')&&isa(safeGuard,'function_handle'), ...
        'm600check:TuningIO','Reader, tuning setter and safety callback are required.');
    assert(text(mode)&&any(strcmp(char(mode),{'APPLY','RESTORE'})),'m600check:TuningMode','Mode must be APPLY or RESTORE.');
    receipt.mode=char(mode);e=validateEntries(entries);receipt.entries=e;receipt.contract_valid=true;
catch problem
    remember(problem,0,'CONTRACT','');return
end
n=3;
readRow=struct('name','','attempted',false,'passed',false,'row',[],'error','');
action=struct('name','','expected_bits','','target_bits','','attempted',false, ...
    'setter_returned',false,'ack_verified',false,'readback_verified',false,'no_op',false, ...
    'guard_called',false,'guard_passed',false,'status','NOT_ATTEMPTED', ...
    'before',[],'setter_result',[],'after',[],'error','');
receipt.preflight=repmat(readRow,n,1);receipt.final_readback=repmat(readRow,n,1);
receipt.actions=repmat(action,n,1);known=false(n,1);
for k=1:n
    receipt.preflight(k).name=e(k).name;receipt.preflight(k).attempted=true;
    receipt.actions(k).name=e(k).name;receipt.actions(k).expected_bits=e(k).original_raw_bits_hex;
    if strcmp(receipt.mode,'APPLY'),target=e(k).target_raw_bits_hex;else,target=e(k).original_raw_bits_hex;end
    receipt.actions(k).target_bits=target;
    try
        p=read(e(k).name);receipt.preflight(k).row=p;
        requireKnown(p,e(k));known(k)=true;receipt.preflight(k).passed=true;
    catch problem
        receipt.preflight(k).error=errorText(problem);remember(problem,k,'PREFLIGHT_READ',e(k).name);
    end
end
receipt.preflight_passed=all(known);stopApply=~all(known);
for k=1:n
    if strcmp(receipt.mode,'APPLY')&&stopApply
        receipt.actions(k).status='NOT_ATTEMPTED_APPLY_FAIL_CLOSED';continue
    end
    if ~known(k),receipt.actions(k).status='NOT_ATTEMPTED_UNKNOWN_RESTORE_VALUE';continue,end
    try
        p=read(e(k).name);receipt.actions(k).before=p;requireKnown(p,e(k));
        bits=typedBits(p,e(k).name);target=receipt.actions(k).target_bits;
        if strcmp(bits,target)
            receipt.actions(k).no_op=true;receipt.actions(k).status='ALREADY_EXACT_NO_WRITE';
            receipt.no_op_count=receipt.no_op_count+1;continue
        end
        receipt.actions(k).guard_called=true;okay=safeGuard();
        assert(yes(okay),'m600check:TuningUnsafeWriteGuard','The safety callback must return logical true.');receipt.actions(k).guard_passed=true;
        receipt.actions(k).attempted=true;receipt.actions(k).status='SETTER_INVOKED_NOT_YET_VERIFIED';
        receipt.setter_invocation_attempt_count=receipt.setter_invocation_attempt_count+1;
        answer=io.setNativeHoverTuningParameter(e(k).name,target,bits,safeGuard);
        receipt.setter_return_count=receipt.setter_return_count+1;receipt.actions(k).setter_returned=true;
        receipt.actions(k).setter_result=answer;
        assert(strcmp(typedBits(answer,e(k).name),target),'m600check:TuningSetterTypedReadback','Setter result does not match the exact target bits.');
        assert(isfield(answer,'real_write_receipt')&&isstruct(answer.real_write_receipt)&& ...
            isscalar(answer.real_write_receipt),'m600check:TuningSetterReceipt','Setter must return one wire-action receipt.');
        a=answer.real_write_receipt;
        assert(all(isfield(a,{'attempted','send_returned','ack_verified','readback_verified'}))&& ...
            yes(a.attempted)&&yes(a.send_returned)&&yes(a.ack_verified)&&yes(a.readback_verified), ...
            'm600check:TuningSetterACK','Attempt, send, ACK and independent readback must all be verified.');
        receipt.actions(k).ack_verified=true;
        p=read(e(k).name);receipt.actions(k).after=p;
        assert(strcmp(typedBits(p,e(k).name),target),'m600check:TuningItemReadback','Independent parameter readback differs from target bits.');
        receipt.actions(k).readback_verified=true;receipt.actions(k).status='ACK_AND_TYPED_READBACK_VERIFIED';
        receipt.ack_and_readback_verified_count=receipt.ack_and_readback_verified_count+1;
    catch problem
        receipt.actions(k).error=errorText(problem);receipt.actions(k).status='ACTION_FAILED_RETAINED';
        remember(problem,k,'ACTION',e(k).name);if strcmp(receipt.mode,'APPLY'),stopApply=true;end
    end
end
for k=1:n
    receipt.final_readback(k).name=e(k).name;receipt.final_readback(k).attempted=true;
    try
        p=read(e(k).name);receipt.final_readback(k).row=p;
        assert(strcmp(typedBits(p,e(k).name),receipt.actions(k).target_bits),'m600check:TuningFinalIdentity','Final parameter identity differs from the requested phase target.');
        receipt.final_readback(k).passed=true;
    catch problem
        receipt.final_readback(k).error=errorText(problem);remember(problem,k,'FINAL_READBACK',e(k).name);
    end
end
receipt.final_identity_verified=all([receipt.final_readback.passed]);
receipt.passed=receipt.final_identity_verified&&isempty(receipt.errors);
if isfield(io,'evidence')&&isa(io.evidence,'function_handle')
    try
        a=io.evidence();
        if isfield(a,'real_parameter_actions'),receipt.adapter_real_parameter_actions=a.real_parameter_actions;end
    catch problem
        receipt.adapter_evidence_capture_error=errorText(problem);receipt.passed=false;
        remember(problem,0,'ADAPTER_EVIDENCE','');
    end
end
    function p=read(name)
        receipt.read_attempt_count=receipt.read_attempt_count+1;p=io.readParameter(name);
    end
    function requireKnown(p,item)
        bits=typedBits(p,item.name);
        if strcmp(receipt.mode,'APPLY')
            assert(strcmp(bits,item.original_raw_bits_hex),'m600check:TuningOriginalMismatch','Apply requires the exact original value.');
        else
            assert(any(strcmp(bits,{item.original_raw_bits_hex,item.target_raw_bits_hex})), ...
                'm600check:TuningUnknownRestoreValue','Restore accepts only known original or candidate values.');
        end
    end
    function remember(problem,index,stage,name)
        item=struct('index',index,'stage',stage,'name',name,'identifier',problem.identifier,'message',problem.message);
        receipt.errors{end+1}=item;if isempty(receipt.first_error),receipt.first_error=item;end
    end
end
function out=validateEntries(entries)
assert(isstruct(entries)&&numel(entries)==3,'m600check:TuningEntries','Exactly three parameter entries are required.');
required={'name','mav_type','original_raw_bits_hex','target_raw_bits_hex'};
assert(isequal(sort(fieldnames(entries)),sort(required(:))),'m600check:TuningEntries','Entries must contain exactly the four declared fields.');
names={'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'};
original={'40D00000','40D00000','3F000000'};target={'4026CCBE','4026CCBE','3F186B9E'};
out=repmat(struct('name','','mav_type',9,'original_raw_bits_hex','','target_raw_bits_hex',''),3,1);
for k=1:3
    a=entries(k);assert(text(a.name),'m600check:TuningName','Parameter name must be one nonempty text value.');i=find(strcmp(names,char(a.name)));
    assert(isscalar(i),'m600check:TuningName','Parameter name is outside the exact three-field service.');
    assert(isnumeric(a.mav_type)&&isreal(a.mav_type)&&isscalar(a.mav_type)&&a.mav_type==9,'m600check:TuningType','Tuning parameters must use MAVLink REAL32 type 9.');
    old=finiteBits(a.original_raw_bits_hex);new=finiteBits(a.target_raw_bits_hex);
    assert(strcmp(old,original{i})&&strcmp(new,target{i}),'m600check:TuningExactPair','Original and target bits must match the declared pair.');
    out(k)=struct('name',char(a.name),'mav_type',9,'original_raw_bits_hex',old,'target_raw_bits_hex',new);
end
assert(numel(unique({out.name}))==3,'m600check:TuningDuplicate','Duplicate tuning parameters are not permitted.');
end
function bits=typedBits(p,name)
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,{'name','mav_type','raw_bits_hex','decoded'})), ...
    'm600check:TuningTypedRow','One complete typed parameter row is required.');
assert(text(p.name)&&strcmp(char(p.name),name)&&isnumeric(p.mav_type)&&isreal(p.mav_type)&& ...
    isscalar(p.mav_type)&&p.mav_type==9,'m600check:TuningTypedRow','Readback name and REAL32 type must match the requested parameter.');
bits=finiteBits(p.raw_bits_hex);v=double(typecast(uint32(hex2dec(bits)),'single'));
assert(isnumeric(p.decoded)&&isreal(p.decoded)&&isscalar(p.decoded)&&isfinite(p.decoded)&& ...
    double(p.decoded)==v,'m600check:TuningDecodedValue','Decoded value must be finite and exactly match the raw REAL32 bits.');
end
function bits=finiteBits(v)
assert(text(v),'m600check:TuningBits','Raw bits must be one nonempty text value.');bits=upper(char(v));
assert(~isempty(regexp(bits,'^[0-9A-F]{8}$','once')),'m600check:TuningBits','Exactly eight hexadecimal digits are required.');
assert(isfinite(typecast(uint32(hex2dec(bits)),'single')),'m600check:TuningFinite','NaN and infinite REAL32 values are not permitted.');
end
function s=errorText(e),s=[e.identifier ': ' e.message];end
function v=yes(x),v=islogical(x)&&isscalar(x)&&x;end
function v=text(x),v=(ischar(x)&&isrow(x)&&~isempty(x))||(isstring(x)&&isscalar(x)&&~ismissing(x)&&strlength(x)>0);end
