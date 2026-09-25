function receipt=transferNativeAllocatorGeometry(io,entries,mode,safeGuard)
%TRANSFERNATIVEALLOCATORGEOMETRY Narrow, accounted REAL32 geometry service.
% io.readParameter(name) returns name/mav_type/raw_bits_hex/decoded.
% io.setRealParameter(name,rawBitsHex,expectedCurrentBits,beforeSendGuard)
% rechecks exact current bits after its last read, then invokes the fresh
% guard immediately before sending (no further waits/reads before PARAM_SET).
% It returns the same typed row plus
% real_write_receipt with attempted/send_returned/ack_verified/readback_verified.
% Optional io.evidence() retains adapter-level real_parameter_actions on errors.
% entries: exactly CA_ROTOR0..5_PX/PY, each name/mav_type=9,
% original_raw_bits_hex/target_raw_bits_hex. No candidate alias or numeric cast.
% APPLY: all twelve original typed identities must pass BEFORE any setter.
% RESTORE: touch only known original/target bits; each item failure is retained
% and cannot block later recoverable items. Reads that fail never justify writes.
% safeGuard() must return scalar logical true before every setter AND again
% within the setter after its final potentially blocking parameter read;
% caller supplies fresh independent disarmed/landed + zero HIL/PWM validation.
% No retries, transport creation/close, parameter-name expansion or model calls.
% Receipt is retained in memory before each setter and returned on exceptions;
% caller owns durable journal/finally. Process termination cannot be caught here.
receipt=struct('schema','M600_NATIVE_CA_GEOMETRY_TRANSFER_V1','mode','', ...
    'passed',false,'contract_valid',false,'preflight_passed',false, ...
    'first_error',[],'errors',{{}},'entries',[],'preflight',[], ...
    'actions',[],'final_readback',[],'read_attempt_count',0, ...
    'setter_invocation_attempt_count',0,'setter_return_count',0, ...
    'ack_and_readback_verified_count',0,'no_op_count',0, ...
    'final_identity_verified',false,'adapter_real_parameter_actions',[], ...
    'adapter_evidence_capture_error','','counter_semantics', ...
    'Setter invocations count attempts; adapter records count transmitted PARAM_SET messages.', ...
    'transport_created',false,'model_started',false,'arm_disarm_mode_requests',0);
try
    assert(isstruct(io)&&isscalar(io)&&isfield(io,'readParameter')&&isa(io.readParameter,'function_handle')&& ...
        isfield(io,'setRealParameter')&&isa(io.setRealParameter,'function_handle')&&isa(safeGuard,'function_handle'), ...
        'm600check:GeometryIO','Typed reader, REAL32 setter and guard required.');
    assert(text(mode)&&ismember(char(mode),{'APPLY','RESTORE'}),'m600check:GeometryMode','Expected APPLY or RESTORE.');
    receipt.mode=char(mode); normalized=validateEntries(entries);receipt.entries=normalized;
    receipt.contract_valid=true;
catch problem
    remember(problem,0,'CONTRACT','');return
end
n=12;
row=struct('name','','attempted',false,'passed',false,'row',[],'error','');
action=struct('name','','expected_bits','','target_bits','','attempted',false, ...
    'setter_returned',false,'ack_verified',false,'readback_verified',false, ...
    'no_op',false,'guard_called',false,'guard_passed',false,'status','NOT_ATTEMPTED', ...
    'before',[],'setter_result',[],'after',[],'error','');
receipt.preflight=repmat(row,n,1);receipt.final_readback=repmat(row,n,1);
receipt.actions=repmat(action,n,1);
known=false(n,1);
for k=1:n
    e=normalized(k);receipt.preflight(k).name=e.name;receipt.actions(k).name=e.name;
    if strcmp(receipt.mode,'APPLY'),target=e.target_raw_bits_hex;else,target=e.original_raw_bits_hex;end
    receipt.actions(k).expected_bits=e.original_raw_bits_hex;receipt.actions(k).target_bits=target;
    receipt.preflight(k).attempted=true;receipt.read_attempt_count=receipt.read_attempt_count+1;
    try
        observed=io.readParameter(e.name);receipt.preflight(k).row=observed;
        bits=typedBits(observed,e.name);
        if strcmp(receipt.mode,'APPLY')
            assert(strcmp(bits,e.original_raw_bits_hex),'m600check:GeometryOriginalMismatch','Original bits mismatch: %s',e.name);
        else
            assert(any(strcmp(bits,{e.original_raw_bits_hex,e.target_raw_bits_hex})), ...
                'm600check:GeometryUnknownRestoreValue','Unknown current bits: %s',e.name);
        end
        known(k)=true;receipt.preflight(k).passed=true;
    catch problem
        receipt.preflight(k).error=errorText(problem);remember(problem,k,'PREFLIGHT_READ',e.name);
    end
end
receipt.preflight_passed=all(known);
mayWrite=strcmp(receipt.mode,'RESTORE')||receipt.preflight_passed;
stopApply=false;
for k=1:n
    e=normalized(k);
    if ~mayWrite||stopApply
        receipt.actions(k).status='NOT_ATTEMPTED_APPLY_FAIL_CLOSED';continue
    end
    if ~known(k)
        receipt.actions(k).status='NOT_ATTEMPTED_UNKNOWN_RESTORE_VALUE';continue
    end
    try
        % Fresh immediate read rejects a concurrent mutation after preflight.
        receipt.read_attempt_count=receipt.read_attempt_count+1;
        before=io.readParameter(e.name);receipt.actions(k).before=before;bits=typedBits(before,e.name);
        if strcmp(receipt.mode,'APPLY')
            assert(strcmp(bits,e.original_raw_bits_hex),'m600check:GeometryConcurrentMutation','Value changed after all-original preflight: %s',e.name);
        else
            assert(any(strcmp(bits,{e.original_raw_bits_hex,e.target_raw_bits_hex})), ...
                'm600check:GeometryUnknownRestoreValue','Unknown restore value immediately before action: %s',e.name);
        end
        target=receipt.actions(k).target_bits;
        if strcmp(bits,target)
            receipt.actions(k).no_op=true;receipt.actions(k).status='ALREADY_EXACT_NO_WRITE';
            receipt.no_op_count=receipt.no_op_count+1;continue
        end
        receipt.actions(k).guard_called=true;okay=safeGuard();
        assert(islogical(okay)&&isscalar(okay)&&okay,'m600check:GeometryUnsafeWriteGuard', ...
            'Fresh disarmed/landed and zero-output guard failed: %s',e.name);
        receipt.actions(k).guard_passed=true;
        % Append attempt BEFORE setter, including setters that mutate then throw.
        receipt.actions(k).attempted=true;receipt.actions(k).status='SETTER_INVOKED_NOT_YET_VERIFIED';
        receipt.setter_invocation_attempt_count=receipt.setter_invocation_attempt_count+1;
        answer=io.setRealParameter(e.name,target,bits,safeGuard);
        receipt.actions(k).setter_returned=true;receipt.setter_return_count=receipt.setter_return_count+1;
        receipt.actions(k).setter_result=answer;
        assert(strcmp(typedBits(answer,e.name),target),'m600check:GeometrySetterTypedReadback','Setter returned wrong typed bits.');
        assert(isfield(answer,'real_write_receipt')&&isstruct(answer.real_write_receipt)&&isscalar(answer.real_write_receipt), ...
            'm600check:GeometrySetterReceipt','Missing explicit setter acknowledgment receipt.');
        a=answer.real_write_receipt;
        assert(all(isfield(a,{'attempted','send_returned','ack_verified','readback_verified'}))&& ...
            yes(a.attempted)&&yes(a.send_returned)&&yes(a.ack_verified)&&yes(a.readback_verified), ...
            'm600check:GeometrySetterACK','Setter ACK/readback not verified.');
        receipt.actions(k).ack_verified=true;
        receipt.read_attempt_count=receipt.read_attempt_count+1;
        after=io.readParameter(e.name);receipt.actions(k).after=after;
        assert(strcmp(typedBits(after,e.name),target),'m600check:GeometryFinalItemReadback','Independent typed reread differs.');
        receipt.actions(k).readback_verified=true;receipt.actions(k).status='ACK_AND_INDEPENDENT_TYPED_READBACK_VERIFIED';
        receipt.ack_and_readback_verified_count=receipt.ack_and_readback_verified_count+1;
    catch problem
        receipt.actions(k).error=errorText(problem);receipt.actions(k).status='ACTION_FAILED_RETAINED';
        remember(problem,k,'ACTION',e.name);
        if strcmp(receipt.mode,'APPLY'),stopApply=true;end
    end
end
% Always read all twelve at the end; never infer whole restoration from ACKs.
for k=1:n
    e=normalized(k);receipt.final_readback(k).name=e.name;receipt.final_readback(k).attempted=true;
    receipt.read_attempt_count=receipt.read_attempt_count+1;
    try
        observed=io.readParameter(e.name);receipt.final_readback(k).row=observed;
        assert(strcmp(typedBits(observed,e.name),receipt.actions(k).target_bits), ...
            'm600check:GeometryFinalIdentity','Final twelve-field identity mismatch: %s',e.name);
        receipt.final_readback(k).passed=true;
    catch problem
        receipt.final_readback(k).error=errorText(problem);remember(problem,k,'FINAL_READBACK',e.name);
    end
end
receipt.final_identity_verified=all([receipt.final_readback.passed]);
receipt.passed=receipt.final_identity_verified&&isempty(receipt.errors);
captureAdapter();

    function remember(problem,index,stage,name)
        item=struct('index',index,'stage',stage,'name',name,'identifier',problem.identifier,'message',problem.message);
        receipt.errors{end+1}=item;
        if isempty(receipt.first_error),receipt.first_error=item;end
    end
    function captureAdapter()
        if ~isfield(io,'evidence')||~isa(io.evidence,'function_handle'),return,end
        try
            evidence=io.evidence();
            if isstruct(evidence)&&isscalar(evidence)&&isfield(evidence,'real_parameter_actions')
                receipt.adapter_real_parameter_actions=evidence.real_parameter_actions;
            end
        catch problem
            receipt.adapter_evidence_capture_error=errorText(problem);
        end
    end
end
function out=validateEntries(entries)
assert(isstruct(entries)&&numel(entries)==12,'m600check:GeometryEntries','Exactly twelve entries required.');
required={'name','mav_type','original_raw_bits_hex','target_raw_bits_hex'};
assert(all(isfield(entries,required)),'m600check:GeometryEntries','Exact original/target/type fields required.');
out=repmat(struct('name','','mav_type',9,'original_raw_bits_hex','','target_raw_bits_hex',''),12,1);
allowed=cell(12,1);for k=0:5,allowed{2*k+1}=sprintf('CA_ROTOR%d_PX',k);allowed{2*k+2}=sprintf('CA_ROTOR%d_PY',k);end
for k=1:12
    e=entries(k);assert(text(e.name)&&any(strcmp(char(e.name),allowed)),'m600check:GeometryName','Non-whitelisted parameter.');
    assert(isnumeric(e.mav_type)&&isreal(e.mav_type)&&isscalar(e.mav_type)&&e.mav_type==9, ...
        'm600check:GeometryType','Only MAV_PARAM_TYPE_REAL32=9.');
    original=finiteBits(e.original_raw_bits_hex);target=finiteBits(e.target_raw_bits_hex);
    out(k)=struct('name',char(e.name),'mav_type',9,'original_raw_bits_hex',original,'target_raw_bits_hex',target);
end
assert(numel(unique({out.name}))==12&&all(ismember(allowed,{out.name})), ...
    'm600check:GeometryDuplicate','Whitelist must appear exactly once.');
end
function bits=typedBits(p,name)
assert(isstruct(p)&&isscalar(p)&&all(isfield(p,{'name','mav_type','raw_bits_hex','decoded'})), ...
    'm600check:GeometryTypedRow','Incomplete typed parameter row.');
assert(text(p.name)&&strcmp(char(p.name),name)&&isnumeric(p.mav_type)&&isreal(p.mav_type)&&isscalar(p.mav_type)&&p.mav_type==9, ...
    'm600check:GeometryTypedRow','Wrong parameter name or REAL32 type.');
bits=finiteBits(p.raw_bits_hex);decoded=double(typecast(uint32(hex2dec(bits)),'single'));
assert(isnumeric(p.decoded)&&isreal(p.decoded)&&isscalar(p.decoded)&&isfinite(p.decoded)&&double(p.decoded)==decoded, ...
    'm600check:GeometryDecodedValue','Decoded value does not equal raw REAL32 identity.');
end
function bits=finiteBits(value)
assert(text(value),'m600check:GeometryBits','Expected eight-digit hex.');bits=upper(char(value));
assert(~isempty(regexp(bits,'^[0-9A-F]{8}$','once')),'m600check:GeometryBits','Expected eight-digit hex.');
assert(isfinite(typecast(uint32(hex2dec(bits)),'single')),'m600check:GeometryFinite','Nonfinite REAL32 is prohibited.');
end
function value=errorText(e),value=[e.identifier ': ' e.message];end
function value=yes(x),value=islogical(x)&&isscalar(x)&&x;end
function value=text(x),value=(ischar(x)&&isrow(x)&&~isempty(x))||(isstring(x)&&isscalar(x)&&~ismissing(x)&&strlength(x)>0);end
