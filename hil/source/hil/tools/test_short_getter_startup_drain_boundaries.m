function result=test_short_getter_startup_drain_boundaries(outputRoot)
% Test getter startup overflow and drain boundaries offline.
arguments,outputRoot (1,1) string,end
assert(~isfolder(outputRoot),'gpenmpcShortTest:ExistingRoot','Choose an unused output path.');
mkdir(outputRoot);build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
parent=fullfile(gpenmpc_external_path('getter_startup_drain'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
assert(isfile(parent));S=load(parent,'getterRaw');g=S.getterRaw;
before=g{766};fault=g{767};
h=fault.ring_header(:);checks=struct('name',{},'pass',{});
check('parent_last_clean_batch_is_clean',~before.failed&&size(before.records,2)==166&&before.total_read==uint64(3595));
check('parent_fault_batch_exact_denominator',fault.failed&&~fault.board_authority ...
    &&size(fault.records,2)==256&&fault.total_read==uint64(3851));
check('parent_overflow_is_exact_RDR1_sticky_fault',u64(h,48)==3891&&u64(h,56)==40 ...
    &&u32(h,80)==1&&u64(h,96)==40&&u64(h,144)==3852);
lastBefore=double(before.original_read_ns(end));firstFaultDrain=double(fault.original_read_ns(1));
gapS=(firstFaultDrain-lastBefore)/1e9;
check('parent_undrained_gap_reproduced',abs(gapS-0.2959596)<1e-6 ...
    &&u64(h,48)-uint64(3596)==uint64(295));
check('overflow_count_follows_immutable_256_capacity',u64(h,48)-uint64(3595)==uint64(296) ...
    &&u64(h,96)==u64(h,48)-uint64(3595)-uint64(256));

runner=fileread(fullfile(build,'tools','run_m600_board_local_short_hil.m'));
pEnable=one(runner,"sessionRaw.stream_enable=stream.enable();");
pDrain1=one(runner,"startupDrainObservationOnly('AFTER_STREAM_ENABLE_BEFORE_METHOD_SERVICE')");
pService=one(runner,'service=gpenmpcNative.RflyLocalMethodService(');
pDrain2=one(runner,"startupDrainObservationOnly('AFTER_METHOD_SERVICE_BEFORE_ENVIRONMENT_PRIME')");
pActivate=one(runner,'environmentRefreshActive=true;');
pPrime=one(runner,'sessionRaw.environment_prime=primeEnvironment();');
pMatch=one(runner,'getter.beginSourceMatching();');
pStart=one(runner,"sessionRaw.start_send=io.sendCanonicalSession('start',struct());");
pAwait=one(runner,"sessionRaw.start_observation=awaitLocalStart(sessionRaw.start_send.original_host_submit_ns(1));");
pWait=one(runner,"waitUntil(@preparedWithWindow,c.preparation_timeout_s,'PREPARATION_AND_ORIGINAL_WINDOW')");
check('production_boundary_order_is_deterministic',pEnable<pDrain1&&pDrain1<pService ...
    &&pService<pDrain2&&pDrain2<pActivate&&pActivate<pPrime ...
    &&pPrime<pStart&&pStart<pMatch&&pMatch<pAwait&&pAwait<pWait);
check('environment_prime_uses_valid_prepare_mode_before_first_service_poll', ...
    contains(runner,"mode='PREPARE';phase='ENVIRONMENT_PRIME';") ...
    &&one(runner,"mode='PREPARE';phase='ENVIRONMENT_PRIME';")<pPrime);
primeHelper=between(runner,'function receipt=primeEnvironment()','function outcome=refreshEnvironment()');
check('environment_prime_has_no_getter_or_method_poll_between_send_and_start', ...
    contains(primeHelper,'outcome=refreshEnvironment()') ...
    &&~contains(primeHelper,"getterMex('drain')")&&~contains(primeHelper,'service.poll('));
helper=between(runner,'function receipt=startupDrainObservationOnly(label)','function heartbeat()');
check('startup_drain_preserves_fail_closed_owner_and_raw_evidence',contains(helper,"getterMex('drain')") ...
    &&contains(helper,'GetterOwnerChanged')&&contains(helper,'GetterProducer') ...
    &&contains(helper,"retain('getter',b)")&&contains(helper,'getter.observeOnly(b)'));
check('startup_drain_grants_no_source_or_environment_authority',contains(helper,"'source_binding_granted',false") ...
    &&contains(helper,"'board_authority',logical(b.board_authority)"));

% Start the matching epoch before module launch and retain every later getter.
% Reject a second epoch or reset.
actual=load(fullfile(gpenmpc_external_path('matlab_noui_original_reader'), ...
    'MATLAB_ORIGINAL_GETTERS.mat'),'last','records','reads');
probe=gpenmpcNative.RflyLocalOriginalGetterBuffer(256, ...
    '990850A2F40F3FCC2A6C47E63A4065B60FF49AA39CC4749FF443963B06F2EF7E');
probe.observeOnly(actualBatch(actual,1:10));
probe.beginSourceMatching();
probe.ingest(actualBatch(actual,11:30));
probeStatus=probe.status();
check('explicit_prestart_matching_retains_all_subsequent_original_rows', ...
    probeStatus.source_matching_started&&~probeStatus.source_matching_closed ...
    &&probeStatus.observation_only_getters==10&&probeStatus.retained_getters==20 ...
    &&probeStatus.accepted_getters==30&&~probeStatus.board_authority);
duplicateId=rejection(@()probe.beginSourceMatching());
check('source_matching_epoch_cannot_restart_or_renew', ...
    strcmp(duplicateId,'gpenmpcNative:GetterMatchingAlreadyStarted'));

entry=fileread(fullfile(build,'tools','execute_m600_board_local_short.m'));
pPure=one(entry,'pureObjects=prepare_m600_board_local_short_objects(cfg,pureResources);');
pStart=one(entry,'process=GPENMPC.HostDiagnostics.NoInheritProcess.Start(si);');
pBorrow=one(entry,'resources.prepared_method_backends=pureObjects.method_backends;');
check('method_backends_are_prepared_and_borrowed_before_producer',pPure<pBorrow&&pBorrow<pStart);
prep=fileread(fullfile(build,'tools','prepare_m600_board_local_short_objects.m'));
check('backend_preparation_has_zero_authority_and_zero_algorithm_calls', ...
    contains(prep,'receipt.method_backend_authority_granted=false') ...
    &&contains(prep,"struct('input_codec',0,'numerical',0,'gp',0)"));

ring=fileread(fullfile(build,'rfly_vendor_integration','clock_tap_overlay','DllGetterRing.hpp'));
check('RDR1_capacity_and_sticky_rejection_are_unchanged',contains(ring,'capacity=256') ...
    &&contains(ring,'fault(h,Overflow,event)')&&~contains(helper,'sticky_errors=0'));

result=struct('schema','SHORT_GETTER_STARTUP_DRAIN_BOUNDARY_HOST_ONLY_V1', ...
    'passed',all([checks.pass]),'checks',checks,'test_count',numel(checks), ...
    'parent_undrained_gap_s',gapS,'parent_events_after_last_clean',double(u64(h,48)-uint64(3595)), ...
    'parent_capacity',256,'parent_overflow_events',double(u64(h,96)), ...
    'test_subject','Getter startup preparation and observation drains', ...
    'sticky_fault_cleared_or_ignored',false, ...
    'source_environment_control_authority_granted',false,'COM',0,'board_actions',0,'plant_runs',0,'output_actions',0);
write(fullfile(outputRoot,'RESULT.json'),result);
fprintf('SHORT_GETTER_STARTUP_DRAIN_BOUNDARY %d/%d\n',sum([checks.pass]),numel(checks));
assert(result.passed,'gpenmpcShortTest:Failed','Focused startup drain boundary regression failed.');
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
    end
end
function b=actualBatch(actual,index)
b=actual.last;b.records=actual.records(:,index);b.original_read_ns=actual.reads(index);
end
function id=rejection(fn)
id='';try,fn();catch ex,id=ex.identifier;end
end
function p=one(text,needle)
p=strfind(text,needle);assert(numel(p)==1,'gpenmpcShortTest:SourceIdentity','Expected one exact source occurrence.');p=p(1);
end
function out=between(text,a,b)
i=strfind(text,a);j=strfind(text,b);assert(numel(i)==1&&numel(j)>=1);j=j(find(j>i,1));assert(~isempty(j));out=text(i:j-1);
end
function v=u32(b,o),v=typecast(uint8(b(o+(1:4))),'uint32');end
function v=u64(b,o),v=typecast(uint8(b(o+(1:8))),'uint64');end
function write(path,value)
fid=fopen(path,'w');assert(fid>=0);c=onCleanup(@()fclose(fid));fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');clear c
end
