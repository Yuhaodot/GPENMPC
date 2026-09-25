function report=run_native_hover_tuning_transfer_tests(outputDir)
% Test three-field parameter transfer with a fake transport.
assert(~isfolder(outputDir)&&~isfile(outputDir),'m600check:OutputExists');
build=string(fileparts(fileparts(mfilename('fullpath'))));oldPath=path;
addpath(fullfile(build,'m600_coptersim','matlab_validation'));
guard=onCleanup(@()path(oldPath)); %#ok<NASGU>
names={'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'};
old={'40D00000','40D00000','3F000000'};new={'4026CCBE','4026CCBE','3F186B9E'};
base=struct('name',names,'mav_type',{9,9,9},'original_raw_bits_hex',old,'target_raw_bits_hex',new);
testNames={'apply','restore','restore_noop','restore_mixed','duplicate','unknown_name','hash_name', ...
    'safety_name','wrong_type','wrong_original','wrong_target','nan_target','missing_entry','bad_mode', ...
    'apply_unknown_original','restore_unknown_current','read_failure','wrong_row_name','wrong_row_type', ...
    'wrong_decoded','guard_false','guard_nonlogical','setter_throws_before_wire','setter_mutates_then_throws', ...
    'bad_ack','bad_setter_bits','missing_receipt','independent_read_mismatch','restore_first_setter_failure', ...
    'last_internal_guard_false','concurrent_before_action','evidence_capture_failure','extra_entry_field'};
results=struct([]);store=old;readCount=0;setterCount=0;guardCount=0;wireActions=struct([]);whichTest='';
for caseIndex=1:numel(testNames)
    whichTest=testNames{caseIndex};store=old;readCount=0;setterCount=0;guardCount=0;wireActions=struct([]);
    entries=base;mode='APPLY';
    if startsWith(whichTest,'restore'),mode='RESTORE';store=new;end
    switch whichTest
        case 'restore_noop',store=old;
        case 'restore_mixed',store={new{1},old{2},new{3}};
        case 'duplicate',entries(2)=entries(1);
        case 'unknown_name',entries(1).name='MPC_Z_VEL_I_ACC';
        case 'hash_name',entries(1).name='_HASH_CHECK';
        case 'safety_name',entries(1).name='FD_FAIL_R';
        case 'wrong_type',entries(1).mav_type=6;
        case 'wrong_original',entries(1).original_raw_bits_hex='40D00001';
        case 'wrong_target',entries(1).target_raw_bits_hex='4026CCBF';
        case 'nan_target',entries(1).target_raw_bits_hex='7FC00000';
        case 'missing_entry',entries=entries(1:2);
        case 'bad_mode',mode='RETRY';
        case 'extra_entry_field',entries(1).unsafe_override=true;
        case 'apply_unknown_original',store{2}='3F800000';
        case 'restore_unknown_current',store{1}='3F800000';
    end
    io=struct('readParameter',@read,'setNativeHoverTuningParameter',@setValue,'evidence',@evidence);
    r=m600check.transferNativeHoverTuning(io,entries,mode,@safe);
    good=false;
    switch whichTest
        case 'apply',good=r.passed&&r.setter_invocation_attempt_count==3&&numel(wireActions)==3&&isequal(store,new);
        case 'restore',good=r.passed&&r.setter_invocation_attempt_count==3&&isequal(store,old);
        case 'restore_noop',good=r.passed&&r.no_op_count==3&&setterCount==0;
        case 'restore_mixed',good=r.passed&&r.no_op_count==1&&setterCount==2&&isequal(store,old);
        case {'duplicate','unknown_name','hash_name','safety_name','wrong_type','wrong_original','wrong_target','nan_target','missing_entry','bad_mode'}
            good=~r.passed&&~r.contract_valid&&readCount==0&&setterCount==0;
        case 'extra_entry_field'
            good=~r.passed&&~r.contract_valid&&readCount==0&&setterCount==0&&guardCount==0&& ...
                isempty(wireActions)&&r.read_attempt_count==0&&r.setter_invocation_attempt_count==0&& ...
                strcmp(r.first_error.identifier,'m600check:TuningEntries');
        case {'apply_unknown_original','read_failure','wrong_row_name','wrong_row_type','wrong_decoded'}
            good=~r.passed&&~r.preflight_passed&&setterCount==0;
        case {'guard_false','guard_nonlogical','concurrent_before_action'}
            good=~r.passed&&setterCount==0&&numel(wireActions)==0;
        case 'restore_unknown_current'
            good=~r.passed&&setterCount==2&&strcmp(store{1},'3F800000')&&isequal(store(2:3),old(2:3));
        case {'setter_throws_before_wire','last_internal_guard_false'}
            good=~r.passed&&r.setter_invocation_attempt_count==1&&setterCount==1&&isempty(wireActions);
        case {'setter_mutates_then_throws','bad_ack','bad_setter_bits','missing_receipt','independent_read_mismatch'}
            good=~r.passed&&r.setter_invocation_attempt_count==1&&numel(wireActions)==1&&strcmp(store{1},new{1});
        case 'restore_first_setter_failure'
            good=~r.passed&&setterCount==3&&numel(wireActions)==2&&strcmp(store{1},new{1})&&isequal(store(2:3),old(2:3));
        case 'evidence_capture_failure'
            good=~r.passed&&r.final_identity_verified&&~isempty(r.adapter_evidence_capture_error)&&setterCount==3;
    end
    item=struct('name',whichTest,'passed',logical(good),'read_count',readCount, ...
        'setter_count',setterCount,'fake_wire_count',numel(wireActions),'guard_count',guardCount,'receipt',r);
    if isempty(results),results=item;else,results(end+1)=item;end %#ok<AGROW>
    fprintf('Tuning transfer %d/%d %s = %d\n',caseIndex,numel(testNames),whichTest,good);
end
report=struct('schema','HOST_NATIVE_HOVER_TUNING_TRANSFER_TESTS_V1','passed',all([results.passed]), ...
    'checks_total',numel(results),'checks_passed',nnz([results.passed]),'cases',results, ...
    'COM_open',0,'UDP_open',0,'board_actions',0,'real_adapter_constructed',false, ...
    'source_sha256',m600check.fileSha256(which('m600check.transferNativeHoverTuning')), ...
    'test_sha256',m600check.fileSha256([mfilename('fullpath') '.m']));
mkdir(outputDir);f=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(f>=0);
c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
assert(report.passed,'m600check:TuningTransferTests','Failed: %s',strjoin({results(~[results.passed]).name},', '));
    function p=read(name)
        readCount=readCount+1;i=find(strcmp(names,name));assert(isscalar(i));
        if strcmp(whichTest,'read_failure')&&readCount==2,error('fixture:Read','Injected read failure');end
        p=row(name,store{i});
        if readCount==2
            if strcmp(whichTest,'wrong_row_name'),p.name='OTHER';end
            if strcmp(whichTest,'wrong_row_type'),p.mav_type=6;end
            if strcmp(whichTest,'wrong_decoded'),p.decoded=p.decoded+1;end
        end
        if strcmp(whichTest,'concurrent_before_action')&&readCount==4,p=row(name,'3F800000');end
        if strcmp(whichTest,'independent_read_mismatch')&&readCount==5,p=row(name,old{i});end
    end
    function p=setValue(name,target,expected,beforeSend)
        setterCount=setterCount+1;i=find(strcmp(names,name));assert(strcmp(store{i},expected));
        if strcmp(whichTest,'setter_throws_before_wire')|| ...
                (strcmp(whichTest,'restore_first_setter_failure')&&setterCount==1)
            error('fixture:BeforeWire','Injected setter failure');
        end
        assert(isequal(beforeSend(),true),'fixture:InternalGuard');
        a=struct('name',name,'attempted',true,'send_returned',true,'ack_verified',true,'readback_verified',true);
        if isempty(wireActions),wireActions=a;else,wireActions(end+1)=a;end %#ok<AGROW>
        store{i}=target;
        if strcmp(whichTest,'setter_mutates_then_throws'),error('fixture:AfterWire','Mutation happened before exception');end
        p=row(name,target);p.real_write_receipt=a;
        if strcmp(whichTest,'bad_ack'),p.real_write_receipt.ack_verified=false;end
        if strcmp(whichTest,'bad_setter_bits'),p=row(name,old{i});p.real_write_receipt=a;end
        if strcmp(whichTest,'missing_receipt'),p=rmfield(p,'real_write_receipt');end
    end
    function okay=safe()
        guardCount=guardCount+1;okay=true;
        if strcmp(whichTest,'guard_false'),okay=false;end
        if strcmp(whichTest,'guard_nonlogical'),okay=1;end
        if strcmp(whichTest,'last_internal_guard_false')&&guardCount==2,okay=false;end
    end
    function e=evidence()
        if strcmp(whichTest,'evidence_capture_failure'),error('fixture:Evidence','Injected capture failure');end
        e=struct('real_parameter_actions',wireActions);
    end
end
function p=row(name,bits)
p=struct('name',name,'mav_type',9,'raw_bits_hex',bits,'decoded',double(typecast(uint32(hex2dec(bits)),'single')));
end
