function report=test_canonical_reference_window(outputRoot)
% Check saved five-leg jets against the MATLAB reference binders.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot),'New evidence destination required.');mkdir(outputRoot);
taskPath=fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);
asset=uint8(sscanf(taskSha,'%2x'));
paths=[fullfile(build,'host_runtime','+gpenmpcNative','prepareCanonicalReferenceWindow.m'); ...
    fullfile(build,'host_runtime','+gpenmpcNative','queryCanonicalReferenceWindow.m'); ...
    fullfile(build,'host_runtime','+gpenmpcNative','bindRflyCanonicalInitialTakeoffTrajectory.m'); ...
    fullfile(build,'host_runtime','+gpenmpcNative','bindRflyCanonicalRelaunchTrajectory.m'); ...
    string(gpenmpc_external_path('sampled_trajectory_source')); ...
    string(mfilename('fullpath'))+".m"];
sourceHashes=arrayfun(@sha,paths);
checks=struct('name',{},'pass',{});rows=struct('leg',{},'window',{},'phase',{}, ...
    'expected',{},'actual',{},'bit_exact',{},'max_abs_error',{},'receipt',{});
offsets=[0,0,0;.7,-.3,0;-1.2,.6,0;2,0,0];zs=[0,.45,-.8,12];
bindings=cell(5,1);trajectories=cell(5,1);maxWindowBytes=0;windowCount=0;
for legIndex=1:5
    leg=bundle.legs{legIndex};original=leg;
    if legIndex==1
        % Explicit HOST fixture only, NOT an observed ground/estimator anchor.
        [tr,binding]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,[0;0;0]);
    else
        fixture=struct('capture_count',legIndex-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',offsets,'current_vertical_frame_offset_m',zs(legIndex-1));
        [tr,binding]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,fixture);
    end
    bindings{legIndex}=binding;trajectories{legIndex}=tr;
    n=numel(leg.local_time_s);
    starts=unique([1,max(1,floor(n/2)-128),max(1,n-255),max(1,n-2)]);
    if legIndex>1
        near=find(leg.local_time_s<=10.5,1,'last');starts=unique([starts,near]);
    end
    for first=starts
        windowCount=windowCount+1;
        [window,state]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,first, ...
            uint64(windowCount),asset);
        memory=whos('window');maxWindowBytes=max(maxWindowBytes,memory.bytes);
        count=double(window.row_count);ix=first:first+count-1;
        check(sprintf('leg%d_window%d_original_time_rows',legIndex,windowCount), ...
            isequal(window.time_s(1:count),leg.local_time_s(ix)));
        sourceJets={leg.position_up_m,leg.velocity_mps,leg.acceleration_mps2,leg.jerk_mps3};
        for d=1:4
            check(sprintf('leg%d_window%d_original_jet%d',legIndex,windowCount,d), ...
                bitEqual(window.nominal_jet(1:count,:,d),sourceJets{d}(ix,:)));
        end
        check(sprintf('leg%d_window%d_unused_not_zero',legIndex,windowCount), ...
            all(isnan(window.time_s(count+1:end))) ...
            &&all(isnan(window.nominal_jet(count+1:end,:,:)),'all'));
        t=window.time_s(1:count);indices=unique(round(linspace(1,count-1,min(13,count-1))));
        q=[t(indices).', (t(indices)+.371*(t(indices+1)-t(indices))).'];
        if legIndex==1
            q=q+25;
            % q-25 is intentionally not snapped. Only queries actually inside
            % this exact row interval can be served; overlap supplies refill.
            q=q((q-25)>=t(1)&(q-25)<=t(end));
            if first==1,q=[-1,0,.00031,3.719,19.999,20,20+eps(20),22.375,25-eps(25),25,q];end
        elseif t(1)<=11&&t(end)>=11
            q=[q,11-eps(11),11,11+eps(11)];
        end
        if first==1&&legIndex>1,q=[-1,0,q];end
        if first+count-1==n,q=[q,tr.total_duration_s,tr.total_duration_s+1];end
        q=unique(q);
        for query=q
            request=makeRequest(window,state.last_accepted_sequence+uint64(1),query);
            before=state;
            [state,actual,receipt]=gpenmpcNative.queryCanonicalReferenceWindow(window,state,request);
            expected=zeros(3,4);
            for d=0:3,expected(:,d+1)=tr.evaluate_fcn(query,d);end
            exact=bitEqual(expected,actual);
            rows(end+1)=struct('leg',legIndex,'window',windowCount,'phase',query, ...
                'expected',expected,'actual',actual,'bit_exact',exact, ...
                'max_abs_error',max(abs(expected-actual),[],'all'),'receipt',receipt); %#ok<AGROW>
            check(sprintf('actual_query_%d_bit_exact',numel(rows)),receipt.accepted&&exact ...
                &&state.last_accepted_sequence==before.last_accepted_sequence+1 ...
                &&~receipt.reference_resampled&&receipt.hardware_actions==0);
        end
    end
    check(sprintf('leg%d_parent_arrays_and_handle_unchanged',legIndex),isequaln(leg,original));
end

% Reject invalid interior-window queries without returning zeros or advancing the query ledger.
leg=bundle.legs{2};binding=bindings{2};
[window,state]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,501,uint64(80),asset,uint64(100));
valid=makeRequest(window,uint64(101),window.time_s(17)+.0037);
r=valid;r.reference_asset_sha256(2)=bitxor(r.reference_asset_sha256(2),uint8(1));reject('wrong_asset',window,state,r,2);
r=valid;r.leg_index=uint32(3);reject('wrong_leg',window,state,r,2);
r=valid;r.window_generation=uint64(81);reject('wrong_window_generation',window,state,r,2);
r=valid;r.query_sequence=uint64(100);reject('duplicate_sequence',window,state,r,3);
r=valid;r.query_sequence=uint64(99);reject('sequence_regression',window,state,r,3);
r=valid;r.query_sequence=101;reject('inexact_sequence_type',window,state,r,3);
r=valid;r.progress_s=NaN;reject('nonfinite_query',window,state,r,4);
r=valid;r.progress_s=[5 6];reject('non_scalar_query',window,state,r,4);
r=valid;r.progress_s=window.time_s(1)-eps(window.time_s(1));reject('window_before',window,state,r,5);
r=valid;r.progress_s=window.time_s(double(window.row_count))+eps(window.time_s(double(window.row_count)));reject('window_after',window,state,r,5);
w=window;w.nominal_jet(2,1,4)=NaN;reject('nonfinite_jerk',w,state,valid,1);
w=window;w.time_s(2)=w.time_s(1);reject('duplicate_source_time',w,state,valid,1);
w=window;w.row_count=uint16(1);reject('insufficient_rows',w,state,valid,1);
w=window;w.nominal_jet=zeros(256,3,3);reject('missing_jerk_array',w,state,valid,1);
w=window;w.binding_mode=uint8(3);reject('unknown_binding',w,state,valid,1);
w=window;w.nominal_duration_s=[5 6];reject('malformed_duration',w,state,valid,1);
w=window;w.source_total_rows=uint32(3);reject('source_range_overflow',w,state,valid,1);
[acceptedState,~,accepted]=gpenmpcNative.queryCanonicalReferenceWindow(window,state,valid);
check('accepted_query_consumes_sequence_only',accepted.accepted ...
    &&acceptedState.last_accepted_sequence==uint64(101));
[refill,refillState]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,700, ...
    uint64(81),asset,acceptedState.last_accepted_sequence);
request=makeRequest(refill,uint64(102),refill.time_s(5)+.0031);
[newState,newJet,newReceipt]=gpenmpcNative.queryCanonicalReferenceWindow(refill,refillState,request);
expected=zeros(3,4);for d=0:3,expected(:,d+1)=trajectories{2}.evaluate_fcn(request.progress_s,d);end
check('explicit_refill_keeps_global_sequence_and_exact_jet',newReceipt.accepted ...
    &&newState.last_accepted_sequence==uint64(102)&&bitEqual(newJet,expected));
reject('old_state_cannot_enter_new_window',refill,acceptedState,request,2);
old=valid;old.query_sequence=uint64(103);reject('old_request_cannot_enter_new_window',refill,newState,old,2);
badBinding=binding;badBinding.leg_index=3;
expectThrow('prepare_wrong_binding',@()gpenmpcNative.prepareCanonicalReferenceWindow(leg,badBinding,1,uint64(1),asset));
expectThrow('prepare_one_source_row',@()gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,numel(leg.local_time_s),uint64(1),asset));
expectThrow('prepare_zero_generation',@()gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,1,uint64(0),asset));
check('all_source_hashes_unchanged',isequal(sourceHashes,arrayfun(@sha,paths)));

report=struct('schema','CANONICAL_EXACT_REFERENCE_WINDOW_HOST_RESULT_V1', ...
    'pass',all([checks.pass]),'checks_total',numel(checks),'checks_passed',sum([checks.pass]), ...
    'actual_saved_task_path',char(taskPath),'task_sha256',taskSha,'actual_legs',5, ...
    'window_count',windowCount,'jet_queries',numel(rows), ...
    'queries_bit_exact',sum([rows.bit_exact]),'maximum_abs_jet_error',max([rows.max_abs_error]), ...
    'fixed_capacity_original_rows',256,'largest_window_matlab_bytes',maxWindowBytes, ...
    'capacity_provenance','HOST fixed-buffer capacity; insufficient reference coverage rejects the request.', ...
    'semantics','Four independent original linear p/v/a/j interpolations; existing initial prefix coefficients and relaunch offsets copied without refitting', ...
    'query_progress','Actual caller phase, including legitimate future reference phase; no future state or source timestamp is supplied or invented', ...
    'lifecycle','Asset, leg, window and monotonic-query binding.', ...
    'ground_anchor_fixture','Explicit HOST numerical fixtures, not actual measured ground captures', ...
    'code_generation_proven',false,'board_integration_proven',false, ...
    'model_calls',0,'solver_calls',0,'com_actions',0,'udp_actions',0,'hardware_actions',0, ...
    'sources',struct('paths',{cellstr(paths)},'sha256',{cellstr(sourceHashes)}),'checks',checks);
save(fullfile(outputRoot,'RAW_REFERENCE_WINDOW.mat'),'rows','bindings','report','-v7.3');
writeJson(fullfile(outputRoot,'RESULT.json'),report);
fprintf('REFERENCE_WINDOW pass=%d checks=%d/%d five_leg_queries=%d exact=%d max=%.17g\n', ...
    report.pass,report.checks_passed,report.checks_total,report.jet_queries,report.queries_bit_exact,report.maximum_abs_jet_error);
assert(report.pass,'gpenmpcNative:ReferenceWindowTest','See preserved actual host result.');
    function check(name,passed)
        checks(end+1)=struct('name',name,'pass',logical(passed)); %#ok<AGROW>
    end
    function reject(name,w,s,r,reason)
        [after,value,receipt]=gpenmpcNative.queryCanonicalReferenceWindow(w,s,r);
        check(['reject_' name],~receipt.accepted&&receipt.reason==reason ...
            &&all(isnan(value),'all')&&isequaln(s,after));
    end
    function expectThrow(name,call)
        threw=false;try,call();catch,threw=true;end
        check(name,threw);
    end
end
function request=makeRequest(window,sequence,phase)
request=struct('reference_asset_sha256',window.reference_asset_sha256, ...
    'leg_index',window.leg_index,'window_generation',window.window_generation, ...
    'query_sequence',sequence,'progress_s',phase);
end
function equal=bitEqual(a,b)
equal=isequal(size(a),size(b))&&isequal(typecast(a(:),'uint64'),typecast(b(:),'uint64'));
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(b,'int8'));
value=string(upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
function writeJson(path,value)
f=fopen(path,'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(value,PrettyPrint=true));
end
