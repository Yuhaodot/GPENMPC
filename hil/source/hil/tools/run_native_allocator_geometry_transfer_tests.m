function result=run_native_allocator_geometry_transfer_tests()
% Test geometry transfer with synthetic IO.
build=fileparts(fileparts(mfilename('fullpath')));oldPath=path;
cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(build,'m600_coptersim','matlab_validation'),'-begin');
checks=struct('name',{},'pass',{});runs=struct('name',{},'receipt',{});
entries=fixtureEntries();store=[];readNames={};writeNames={};readByName=zeros(12,1);
setterActions=struct([]);guardCalls=0;firstWriteReadCount=NaN;cfg=struct();fakeTime=0;

fresh('apply_complete','APPLY');a=execute(entries,'APPLY');
check('twelve_original_reads_before_first_setter',firstWriteReadCount==14&& ...
    isequal(readNames(1:12),reshape({entries.name},1,12)));
check('apply_all_twelve_ack_and_readback',a.passed&&a.setter_invocation_attempt_count==12&& ...
    a.setter_return_count==12&&a.ack_and_readback_verified_count==12&&numel(writeNames)==12);
check('apply_every_setter_and_final_send_guarded',guardCalls==24&&all([a.actions.guard_passed])&&all([a.final_readback.passed]));
check('apply_targets_are_exact_raw_bits',isequal({store.raw_bits_hex},{entries.target_raw_bits_hex}));
fresh('restore_complete','RESTORE');a=execute(entries,'RESTORE');
check('restore_exact_twelve_originals',a.passed&&a.setter_invocation_attempt_count==12&& ...
    isequal({store.raw_bits_hex},{entries.original_raw_bits_hex}));
check('restore_final_twelve_readback',numel(a.final_readback)==12&&all([a.final_readback.attempted])&&a.final_identity_verified);

fresh('apply_noop','APPLY');noops=entries;
for k=1:12,noops(k).target_raw_bits_hex=noops(k).original_raw_bits_hex;end
a=execute(noops,'APPLY');
check('apply_noop_readonly',a.passed&&a.no_op_count==12&&a.setter_invocation_attempt_count==0&&isempty(writeNames)&&guardCalls==0);
fresh('restore_noop','APPLY');a=execute(entries,'RESTORE');
check('restore_noop_readonly',a.passed&&a.no_op_count==12&&isempty(writeNames));

for kind={'duplicate','forbidden','entry_type','short','nan_target','inf_original','bad_hex','candidate_alias'}
    fresh(['contract_' kind{1}],'APPLY');bad=entries;
    switch kind{1}
        case 'duplicate',bad(12).name=bad(1).name;
        case 'forbidden',bad(12).name='COM_ARM_HFLT_CHK';
        case 'entry_type',bad(12).mav_type=6;
        case 'short',bad=bad(1:11);
        case 'nan_target',bad(12).target_raw_bits_hex='7FC00001';
        case 'inf_original',bad(12).original_raw_bits_hex='7F800000';
        case 'bad_hex',bad(12).target_raw_bits_hex='0x000000';
        case 'candidate_alias',bad=rmfield(bad,'target_raw_bits_hex');
    end
    a=execute(bad,'APPLY');
    check(['invalid_' kind{1} '_before_all_IO'],~a.passed&&~a.contract_valid&&isempty(readNames)&&isempty(writeNames)&&~isempty(a.first_error));
end
fresh('invalid_mode','APPLY');a=execute(entries,'SCAN');
check('no_unregistered_mode',~a.contract_valid&&isempty(readNames)&&isempty(writeNames));

for kind={'wrong_type','wrong_bits','wrong_name','decoded_mismatch','read_failure'}
    fresh(['precheck_' kind{1}],'APPLY');cfg.preflightFailure=kind{1};cfg.failureIndex=12;
    a=execute(entries,'APPLY');
    check(['all_precheck_' kind{1} '_zero_setters'],a.contract_valid&&~a.preflight_passed&& ...
        all([a.preflight.attempted])&&a.setter_invocation_attempt_count==0&&isempty(writeNames));
    check(['first_failure_' kind{1} '_retained'],a.first_error.index==12&&strcmp(a.first_error.stage,'PREFLIGHT_READ'));
end

fresh('partial_apply_mutate_then_throw','APPLY');cfg.setterFailIndex=3;cfg.failAfterMutation=true;
a=execute(entries,'APPLY');
check('partial_apply_attempt_record_before_throw',~a.passed&&a.setter_invocation_attempt_count==3&& ...
    a.setter_return_count==2&&a.actions(3).attempted&&~a.actions(3).setter_returned);
check('partial_apply_first_error_and_no_later_setters',a.first_error.index==3&&strcmp(a.first_error.stage,'ACTION')&& ...
    numel(writeNames)==3&&strcmp(store(3).raw_bits_hex,entries(3).target_raw_bits_hex)&& ...
    strcmp(store(4).raw_bits_hex,entries(4).original_raw_bits_hex));
check('partial_apply_adapter_actions_retained',numel(a.adapter_real_parameter_actions)==3&&a.adapter_real_parameter_actions(3).attempted);
fresh('apply_guard_loss','APPLY');cfg.guardFailureAt=5;a=execute(entries,'APPLY');
check('guard_loss_prevents_third_setter',a.setter_invocation_attempt_count==2&&guardCalls==5&& ...
    ~a.actions(3).attempted&&strcmp(a.first_error.identifier,'m600check:GeometryUnsafeWriteGuard'));
fresh('apply_guard_numeric_one','APPLY');cfg.numericGuard=true;a=execute(entries,'APPLY');
check('guard_must_be_explicit_true_logical',a.setter_invocation_attempt_count==0&&~a.passed);
fresh('apply_guard_throws','APPLY');cfg.guardThrows=true;a=execute(entries,'APPLY');
check('guard_exception_retains_receipt',a.setter_invocation_attempt_count==0&&strcmp(a.first_error.identifier,'fake:GuardException'));

fresh('apply_final_read_outlives_safe_state','APPLY');cfg.delayedSetterReadIndex=1;
a=execute(entries,'APPLY');
check('delayed_final_read_rechecks_safety_before_any_fake_write', ...
    a.setter_invocation_attempt_count==1&&a.setter_return_count==0&&guardCalls==2&& ...
    fakeTime>cfg.guardFreshUntil&&isempty(writeNames)&&isempty(setterActions)&& ...
    strcmp(a.first_error.identifier,'fake:FinalSendGuard'));
check('delayed_final_read_keeps_all_original_bits',isequal({store.raw_bits_hex},{entries.original_raw_bits_hex}));
fresh('apply_unknown_after_helper_read','APPLY');cfg.setterUnknownIndex=1;
a=execute(entries,'APPLY');
check('unknown_after_helper_read_rejected_before_any_fake_write', ...
    a.setter_invocation_attempt_count==1&&a.setter_return_count==0&&isempty(writeNames)&& ...
    isempty(setterActions)&&guardCalls==1&&strcmp(a.first_error.identifier,'fake:FinalReadIdentity'));
check('unknown_final_read_value_is_preserved_not_overwritten',strcmp(store(1).raw_bits_hex,bits(.99123)));
fresh('restore_unknown_after_helper_read','RESTORE');cfg.setterUnknownIndex=1;
a=execute(entries,'RESTORE');
check('restore_final_read_unknown_preserved_later_items_still_restore', ...
    a.setter_invocation_attempt_count==12&&a.ack_and_readback_verified_count==11&& ...
    numel(writeNames)==11&&~any(strcmp(writeNames,entries(1).name))&& ...
    strcmp(store(1).raw_bits_hex,bits(.99123))&&strcmp(store(12).raw_bits_hex,entries(12).original_raw_bits_hex));

fresh('apply_concurrent_mutation','APPLY');cfg.concurrentIndex=4;a=execute(entries,'APPLY');
check('fresh_per_item_recheck_blocks_concurrent_change',a.setter_invocation_attempt_count==3&& ...
    strcmp(a.first_error.identifier,'m600check:GeometryConcurrentMutation'));
fresh('apply_missing_ack_receipt','APPLY');cfg.badAckIndex=2;a=execute(entries,'APPLY');
check('readback_alone_does_not_invent_ack',a.setter_invocation_attempt_count==2&&a.setter_return_count==2&& ...
    a.ack_and_readback_verified_count==1&&strcmp(a.first_error.identifier,'m600check:GeometrySetterACK'));
fresh('apply_independent_readback_fail','APPLY');cfg.failIndependentIndex=2;a=execute(entries,'APPLY');
check('independent_readback_error_retained',a.setter_invocation_attempt_count==2&&a.actions(2).ack_verified&& ...
    ~a.actions(2).readback_verified&&strcmp(a.first_error.identifier,'fake:IndependentReadback'));

fresh('restore_failure_continues','RESTORE');cfg.setterFailIndex=3;cfg.failAfterMutation=false;
a=execute(entries,'RESTORE');
check('restore_failed_third_does_not_block_later_nine',a.setter_invocation_attempt_count==12&& ...
    a.ack_and_readback_verified_count==11&&strcmp(writeNames{end},entries(12).name)&& ...
    strcmp(store(12).raw_bits_hex,entries(12).original_raw_bits_hex));
check('restore_failure_not_erased_by_final_errors',a.first_error.index==3&&strcmp(a.first_error.stage,'ACTION')&& ...
    ~a.final_readback(3).passed&&numel(a.errors)>=2);
fresh('restore_read_failure','RESTORE');cfg.preflightFailure='read_failure';cfg.failureIndex=3;
a=execute(entries,'RESTORE');
check('restore_read_failure_no_blind_write',a.setter_invocation_attempt_count==11&&~any(strcmp(writeNames,entries(3).name))&& ...
    strcmp(store(3).raw_bits_hex,entries(3).target_raw_bits_hex));
check('restore_read_failure_later_items_continue',strcmp(store(12).raw_bits_hex,entries(12).original_raw_bits_hex));
fresh('restore_unknown_value','RESTORE');store(3)=parameterRow(entries(3).name,bits(.9876));
a=execute(entries,'RESTORE');
check('unknown_restore_value_never_overwritten',a.setter_invocation_attempt_count==11&& ...
    strcmp(store(3).raw_bits_hex,bits(.9876))&&~any(strcmp(writeNames,entries(3).name)));
fresh('restore_partial_guard','RESTORE');cfg.guardFailureAt=5;a=execute(entries,'RESTORE');
check('restore_guard_rechecked_each_later_item',a.setter_invocation_attempt_count==11&&guardCalls==23&& ...
    ~a.actions(3).attempted&&a.actions(12).readback_verified);
fresh('restore_after_partial_apply','APPLY');
for k=1:5,store(k)=parameterRow(entries(k).name,entries(k).target_raw_bits_hex);end
a=execute(entries,'RESTORE');
check('mixed_original_target_restores_only_needed',a.passed&&a.setter_invocation_attempt_count==5&&a.no_op_count==7);
fresh('evidence_capture_failure','APPLY');cfg.evidenceThrows=true;a=execute(entries,'APPLY');
check('adapter_evidence_exception_not_drop_actions',a.setter_invocation_attempt_count==12&& ...
    a.ack_and_readback_verified_count==12&&~isempty(a.adapter_evidence_capture_error));
check('no_transport_or_model_owned',~a.transport_created&&~a.model_started&&a.arm_disarm_mode_requests==0);
result=struct('status','PASS_PURE_FAKE_IO_GEOMETRY_TRANSFER_TESTS','checks_total',numel(checks), ...
    'checks_passed',sum([checks.pass]),'checks',checks,'runs',runs, ...
    'actual_parameter_writes',0,'COM_open',0,'UDP_open',0,'model_launches',0, ...
    'claim','Synthetic parameter-transfer and rollback tests.');

    function fresh(name,initialMode)
        cfg=struct('name',name,'preflightFailure','','failureIndex',0, ...
            'setterFailIndex',0,'failAfterMutation',false,'guardFailureAt',0, ...
            'numericGuard',false,'guardThrows',false,'concurrentIndex',0, ...
            'badAckIndex',0,'failIndependentIndex',0,'evidenceThrows',false, ...
            'delayedSetterReadIndex',0,'setterUnknownIndex',0,'guardFreshUntil',2);
        store=repmat(parameterRow(entries(1).name,entries(1).original_raw_bits_hex),12,1);
        for j=1:12
            if strcmp(initialMode,'RESTORE'),b=entries(j).target_raw_bits_hex;else,b=entries(j).original_raw_bits_hex;end
            store(j)=parameterRow(entries(j).name,b);
        end
        readNames={};writeNames={};readByName=zeros(12,1);guardCalls=0;firstWriteReadCount=NaN;setterActions=struct([]);fakeTime=0;
    end
    function a=execute(e,mode)
        io=struct('readParameter',@readFake,'setRealParameter',@setFake,'evidence',@evidenceFake);
        a=m600check.transferNativeAllocatorGeometry(io,e,mode,@guardFake);
        runs(end+1)=struct('name',cfg.name,'receipt',a); %#ok<AGROW>
    end
    function p=readFake(name)
        j=find(strcmp(name,{entries.name}));assert(isscalar(j),'Fake received forbidden name.');
        readNames{end+1}=name;readByName(j)=readByName(j)+1;
        if j==cfg.failureIndex&&readByName(j)==1
            switch cfg.preflightFailure
                case 'read_failure',error('fake:ReadFailure','Synthetic read failed.');
                case 'wrong_type',p=store(j);p.mav_type=6;return
                case 'wrong_bits',p=parameterRow(name,bits(.777));return
                case 'wrong_name',p=store(j);p.name='CA_ROTOR0_PZ';return
                case 'decoded_mismatch',p=store(j);p.decoded=p.decoded+.1;return
            end
        end
        if j==cfg.concurrentIndex&&readByName(j)==2,store(j)=parameterRow(name,bits(.888));end
        if j==cfg.failIndependentIndex&&readByName(j)==4,error('fake:IndependentReadback','Synthetic independent read failed.');end
        p=store(j);
    end
    function p=setFake(name,target,expectedCurrentBits,beforeSendGuard)
        j=find(strcmp(name,{entries.name}));assert(isscalar(j));
        assert(ischar(target)&&numel(target)==8,'Setter requires RAW HEX, not numeric target.');
        % Model the setter's final blocking read; track external mutation separately.
        if j==cfg.setterUnknownIndex,store(j)=parameterRow(name,bits(.99123));end
        before=readFake(name);
        if j==cfg.delayedSetterReadIndex,fakeTime=fakeTime+3;end
        assert(before.mav_type==9&&isfinite(before.decoded)&&strcmp(before.raw_bits_hex,expectedCurrentBits), ...
            'fake:FinalReadIdentity','Current exact bits changed after helper read.');
        finalSafe=beforeSendGuard();
        assert(islogical(finalSafe)&&isscalar(finalSafe)&&finalSafe, ...
            'fake:FinalSendGuard','State stale/changed during the final setter read.');
        if isempty(writeNames),firstWriteReadCount=numel(readNames);end
        writeNames{end+1}=name;
        atom=struct('name',name,'attempted',true,'send_returned',false,'ack_verified',false,'readback_verified',false);
        if isempty(setterActions),setterActions=atom;else,setterActions(end+1)=atom;end
        k=numel(setterActions);
        if j==cfg.setterFailIndex&&~cfg.failAfterMutation,error('fake:SetFailure','Failure before fake mutation.');end
        store(j)=parameterRow(name,target);setterActions(k).send_returned=true;
        if j==cfg.setterFailIndex,error('fake:SetFailureAfterMutation','Fake mutation happened; acknowledgment failed.');end
        setterActions(k).ack_verified=j~=cfg.badAckIndex;
        setterActions(k).readback_verified=true;p=store(j);p.real_write_receipt=setterActions(k);
    end
    function okay=guardFake()
        guardCalls=guardCalls+1;
        if cfg.guardThrows,error('fake:GuardException','Synthetic guard threw.');end
        if cfg.numericGuard,okay=1;else,okay=guardCalls~=cfg.guardFailureAt&&fakeTime<=cfg.guardFreshUntil;end
    end
    function e=evidenceFake()
        if cfg.evidenceThrows,error('fake:Evidence','Synthetic evidence capture failed.');end
        e=struct('real_parameter_actions',setterActions);
    end
    function check(name,okay)
        okay=isscalar(okay)&&logical(okay);checks(end+1)=struct('name',name,'pass',okay); %#ok<AGROW>
        assert(okay,'m600check:GeometryFakeTest','Failed: %s',name);
    end
end
function e=fixtureEntries()
% Use finite synthetic values to exercise REAL32 identities.
e=repmat(struct('name','','mav_type',9,'original_raw_bits_hex','','target_raw_bits_hex',''),12,1);
for k=0:5
    for axis=1:2
        j=2*k+axis;letters='XY';e(j).name=sprintf('CA_ROTOR%d_P%c',k,letters(axis));
        e(j).original_raw_bits_hex=bits((j-6)/20);e(j).target_raw_bits_hex=bits((j+2)/31);
    end
end
end
function p=parameterRow(name,raw)
p=struct('name',name,'mav_type',9,'raw_bits_hex',raw,'decoded',double(typecast(uint32(hex2dec(raw)),'single')));
end
function h=bits(v),h=upper(dec2hex(typecast(single(v),'uint32'),8));end
