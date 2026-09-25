function report=test_local_phase_environment(outputRoot,selection)
% Test phase/environment binding with retained phase data and localhost IO.
build=string(fileparts(fileparts(mfilename('fullpath'))));
if nargin<2,selection='ALL';end
assert(ismember(selection,{'ALL','PHASE_ONLY','RC_STORAGE','MANUAL_WIND_BIND'}));
addpath(fullfile(build,'host_runtime'),fullfile(build,'matlab_validation'),fullfile(build,'m600_coptersim','matlab_validation'));
assert(~isfile(fullfile(outputRoot,'RESULT.json')));if ~isfolder(outputRoot),mkdir(outputRoot);end;diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
checks=struct('name',{},'pass',{});io=[];peer=[];
if strcmp(selection,'MANUAL_WIND_BIND')
    q=load(fullfile(gpenmpc_external_path('local_original_task_cache'),'RAW.mat'),'receipt');x=q.receipt;
    p=x.original_phase_receipts{1};e=x.original_environment_receipts{1};
    policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
        'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
    ts=struct('path',fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
        'sha256','B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F', ...
        'configuration_sha256','A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA', ...
        'environment_policy',policy,'copter_id',1,'manual_disturbance',true);
    ts.cached_asset=gpenmpcNative.RflyLocalTaskAsset(ts);
    originalWind=readLE(e.original_frame232(49:64),'double');
    e.original_frame232(49:64)=typecast(originalWind+[.6;-.4],'uint8');
    s=x.original_rls_bytes;s(236:350)=0;
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(s(1:350),'int8'));
    s(351:382)=reshape(typecast(md.digest(),'uint8'),[],1);
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(s(:),'int8'));
    p.source_rls_sha256=reshape(typecast(md.digest(),'uint8'),[],1);
    w=x.original_window;w.runtime_decision_ns=w.original_read_ns(end);
    modelTime=readLE(w.records(265:272,end),'double');
    dg=diagnostic(e.original_diagnostic264,modelTime,1,uint64(2));e.original_diagnostic264=dg.bytes;
    e.original_frame232(25:32)=typecast(modelTime,'uint8');
    e.binding_semantics='LATEST_CAUSAL_APPLIED_ENVIRONMENT_WITH_ORIGINAL_250MS_EXPIRY';
    bind=@(task,env)gpenmpcNative.bindRflyLocalOriginalInputs(s,x.original_rls_receive_ns,w,task,{p},{env},true);
    b=bind(ts,e);
    check('manual_applied_gust_accepted_with_original_ack',b{1}.numerical_inputs_complete ...
        &&isequal(b{1}.accepted_plant_wind_xy_mps,originalWind+[.6;-.4]));
    base=e;base.original_frame232(49:64)=typecast(originalWind,'uint8');autonomous=ts;autonomous.manual_disturbance=false;
    a=bind(autonomous,base);
    check('controller_estimate_not_replaced_by_true_wind',isequal(a{1}.wind,b{1}.wind));
    reject('autonomous_still_rejects_modified_wind',@()bind(autonomous,e));
    bad=e;bad.original_frame232(49:64)=typecast(originalWind+[1.601;0],'uint8');reject('outside_declared_manual_gust',@()bind(ts,bad));
    bad=e;bad.original_frame232(17:24)=typecast(2.0,'uint8');reject('wrong_applied_generation',@()bind(ts,bad));
    bad=e;bad.original_frame232(185:192)=typecast(26090502.0,'uint8');reject('wrong_session',@()bind(ts,bad));
    bad=e;bad.original_frame232(41:48)=typecast(1.75,'uint8');reject('wrong_payload',@()bind(ts,bad));
    bad=e;bad.original_frame232(25:32)=typecast(modelTime-.251,'uint8');reject('expired_environment_still_rejected',@()bind(ts,bad));
    bad=e;bad.original_frame232(25:32)=typecast(modelTime+.01,'uint8');reject('future_environment_still_rejected',@()bind(ts,bad));
    report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'hardware_actions',0, ...
        'scope','MANUAL_WIND_RUNTIME_BINDING__REUSED_BYTES_WITH_EXPLICIT_MOCK_RLS_ENV_AND_TIMES');
    writeReport();diary off;return
end
if strcmp(selection,'RC_STORAGE')
    fixture=load(fullfile(gpenmpc_external_path('local_original_task_cache'),'RAW.mat'),'receipt');
    e=fixture.receipt.original_environment_receipts{1};
    policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
        'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
    ledger=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,4);
    for k=1:13
        tx=e.original_frame232(:);tx(17:24)=typecast(double(k),'uint8');tx(25:32)=typecast(double(k),'uint8');
        ledger.sent(struct('bytes',tx,'original_host_send_ns',uint64(3*k),'send_returned',true));
        dg=diagnostic(e.original_diagnostic264,double(k),k,uint64(3*k+1));ledger.received(dg);
        resolved=ledger.resolve(double(k),true);
        check(sprintf('wrapped_latest_exact_%d',k),~isempty(resolved)&&isequal(resolved.original_frame232,tx) ...
            &&isequal(resolved.original_diagnostic264,dg.bytes(:)));
        st=ledger.status();check(sprintf('same_retained_capacity_%d',k),st.retained_frames==min(k,4)&&st.retained_diagnostics==min(k,4));
    end
    check('evicted_history_cannot_bind',isempty(ledger.resolve(9,false)));
    check('oldest_retained_still_resolves',~isempty(ledger.resolve(10,false)));
    check('runtime_original_expiry_unchanged',isempty(ledger.resolve(13.250001,true)));
    bad=diagnostic(e.original_diagnostic264,14,12,uint64(50));
    reject('applied_generation_reversal_still_rejected',@()ledger.received(bad));
    check('fault_stays_latched',ledger.Failed);
    report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'hardware_actions',0, ...
        'scope','ENV_CACHE_CIRCULAR_STORAGE_WITH_MOCK_TIMES_AND_GENERATIONS');
    writeReport();diary off;return
end
try
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(task,taskSha);
[tr,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(bundle.legs{1},zeros(3,1));
raw=reshape(readbin(fullfile(gpenmpc_external_path('committed_state_diagnostics'),'COMMITTED_RLC2.bin')),1494,[]);
c=gpenmpcNative.RflyLocalCommittedDecoder(raw(:,1),uint64(100));
r=struct('identity',c.identity,'leg_index',c.leg_index,'task_sha256',taskSha, ...
    'execution_session_sha256',repmat('B',1,64),'reference_asset_sha256',hex(c.reference_asset_sha256), ...
    'configuration_sha256',hex(c.configuration_sha256));
a=struct('local_full_inner',true,'registration_result','Registered','echo_confirmation_result','Confirmed', ...
    'execution_session_sha256',r.execution_session_sha256,'original_host_receive_ns',uint64(1), ...
    'echo',struct('uid',r.identity.uid,'process_session_generation',r.identity.boot_generation, ...
    'system',r.identity.system,'component',r.identity.component,'configuration_payload_sha256',r.configuration_sha256), ...
    'confirm_receipt',struct('parsed_fields',struct('leg',r.leg_index,'task_sha',r.task_sha256,'state',3, ...
    'start_requests',0,'stop_requests',0,'session_fault',0),'original_session_line_bytes',uint8('EXPLICIT_MOCK_REGISTRATION_ONLY').'));
phase=gpenmpcNative.RflyLocalPhaseView(bundle,tr,r,a);v=phase.view();
check('registered_original_initializer_without_clock_step',v.initializer_only&&v.controller_phase_s==0&&~v.source_clock_advanced);
for k=1:3
    item=struct('message',raw(:,k),'original_host_receive_ns',uint64(100+k));phase.ingest(item);
    v=phase.view();q=gpenmpcNative.RflyLocalCommittedDecoder(raw(:,k),item.original_host_receive_ns);
    check(sprintf('actual_C_installed_phase_%d',k),v.controller_phase_s==q.installed_phase2(1)&&v.board_commit_count==k);
end
old=v;for k=1:20,v=phase.view();end
check('view_polls_never_integrate_wall_clock',isequaln(v,old));
reject('duplicate_commit_not_new_phase',@()phase.ingest(item));check('phase_fault_latched',phase.Failed);
phase=gpenmpcNative.RflyLocalPhaseView(bundle,tr,r,a);phase.ingest(item);phase.suspend('NATIVE_LAND');v=phase.view();
check('land_suspends_without_zeroing_phase',v.outer_already_suspended&&v.controller_phase_s==q.installed_phase2(1));
reject('post_land_commit_rejected',@()phase.ingest(struct('message',raw(:,4),'original_host_receive_ns',uint64(105))));
if strcmp(selection,'PHASE_ONLY')
    report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks), ...
        'hardware_actions',0,'COM',0,'scope','UNCHANGED_PHASE_REUSE_AND_REAL_COMMIT_LIFECYCLE_INVALIDATION');
    writeReport();diary off;return
end

% Combine retained getter records with synthetic board, phase and environment envelopes.
fixture=load(fullfile(gpenmpc_external_path('local_original_task_cache'),'RAW.mat'),'receipt');x=fixture.receipt;
policy=struct('expected_session_token',26090501,'payload_by_generation_kg',[2.21;1.75;.98;.55;0], ...
    'mass_by_generation_kg',[11.71;11.25;10.48;10.05;9.5]);
ts=struct('path',task,'sha256',taskSha,'configuration_sha256',r.configuration_sha256,'environment_policy',policy,'copter_id',1);
ts.cached_asset=gpenmpcNative.RflyLocalTaskAsset(ts);
e=x.original_environment_receipts{1};p=x.original_phase_receipts{1};
dd=m600check.decodeCopterSimDeliveryDiagnostics(e.original_diagnostic264,1,NaN,policy);time=dd.environment_extension.same_model_time_s;
ledger=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);
tx=struct('bytes',e.original_frame232,'original_host_send_ns',uint64(1),'send_returned',true);
ledger.sent(tx);check('send_alone_cannot_ack',isempty(ledger.resolve(time)));
left=diagnostic(e.original_diagnostic264,time-.001,1,uint64(2));right=diagnostic(e.original_diagnostic264,time+.001,1,uint64(3));
ledger.received(left);check('one_old_ack_cannot_claim_current_step',isempty(ledger.resolve(time)));
runtime=ledger.resolve(time,true);
check('runtime_uses_actual_prior_ack_without_future_pair',~isempty(runtime)&& ...
    isequal(runtime.original_diagnostic264,left.bytes)&&runtime.no_timestamp_renewal);
f=typecast(uint8(e.original_frame232(9:232)),'double');
check('runtime_original_250ms_expiry_retained',isempty(ledger.resolve(f(3)+.250001,true)));
ledger.received(right);resolved=ledger.resolve(time);
check('two_received_same_applied_generation_bracket_without_retimestamp',~isempty(resolved) ...
    &&isequal(resolved.original_previous_diagnostic264,left.bytes)&&isequal(resolved.original_diagnostic264,right.bytes));
p.saved_task_time_s=p.saved_task_time_s+.005;
[b,receipt]=gpenmpcNative.bindRflyLocalOriginalInputs(x.original_rls_bytes,x.original_rls_receive_ns,x.original_window,ts,{p},{ledger});
check('held_actual_environment_and_current_estimate_remain_separate',b{1}.numerical_inputs_complete ...
    &&b{1}.environment_phase_lag_s==.005&&~b{1}.environment_exact_step_diagnostic&&~b{1}.control_authority);
bad=resolved;bad.original_previous_diagnostic264=diagnostic(left.bytes,time-.001,2,uint64(2)).bytes;
reject('generation_change_not_same_step',@()gpenmpcNative.bindRflyLocalOriginalInputs(x.original_rls_bytes,x.original_rls_receive_ns,x.original_window,ts,{p},{bad}));
bad=resolved;bad.original_previous_diagnostic_receive_ns=uint64(4);
reject('reverse_receive_order_rejected',@()gpenmpcNative.bindRflyLocalOriginalInputs(x.original_rls_bytes,x.original_rls_receive_ns,x.original_window,ts,{p},{bad}));
ledger2=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);ledger2.sent(tx);ledger2.received(left);
ledger2.received(diagnostic(right.bytes,time+.001,2,uint64(3)));
check('cross_frame_gap_waits_never_interpolates_ack',isempty(ledger2.resolve(time)));
reject('applied_generation_cannot_reverse',@()ledger2.received(diagnostic(right.bytes,time+.002,1,uint64(4))));
ledger3=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);ledger3.sent(tx);
exact=diagnostic(e.original_diagnostic264,time,1,uint64(2));ledger3.received(exact);resolved=ledger3.resolve(time);
check('exact_diagnostic_path_retains_original_bytes',isequal(resolved.original_diagnostic264,e.original_diagnostic264)&&isempty(resolved.original_previous_diagnostic264));

cfg=struct('local_mavlink_port',62341,'remote_mavlink_port',62342,'truth_port',62343,'coptersim_time_port',62344, ...
    'target_system',1,'target_component',1,'clock_max_rtt_s',1,'clock_sync_samples',2,'clock_sync_period_s',1,'clock_max_age_s',5, ...
    'clock_max_uncertainty_s',1,'clock_max_utc_drift_s',1,'clock_max_time_heartbeat_age_s',5,'clock_max_time_heartbeat_lag_s',5, ...
    'maximum_truth_lag_s',5,'maximum_raw_records',1000,'state_max_age_s',1,'live_enabled',true,'outer_preflight_pass',true, ...
    'canonical_exchange',struct('runtime','BOARD_LOCAL_FULL_INNER','assembly_limit_ns',uint64(5000000000), ...
    'gp_reply_host_max_age_ns',uint64(5000000000),'local_input_host_max_age_ns',uint64(5000000000),'completed_queue_capacity',4));
cfg.delivery_environment_contract=policy;cfg.delivery_environment_contract.initial_payload_kg=2.21;cfg.delivery_environment_contract.remote_port=62345;
cfg.local_short=struct('environment_ledger_capacity',16);
io=m600check.makeM600CopterSimIo(cfg);peer=udpport('datagram','IPV4','LocalHost','127.0.0.1','LocalPort',62345,'Timeout',.2);
f=readLE(e.original_frame232(9:end),'double');v=frameValue(f);sent=io.sendPlantEnvironment(v);
timer=tic;while peer.NumDatagramsAvailable==0&&toc(timer)<3,pause(.005);end
wire=read(peer,1,'uint8');check('existing_same_IO_ENV232_exact',isequal(uint8(wire.Data(:)),e.original_frame232));
write(peer,e.original_diagnostic264,'uint8','127.0.0.1',cfg.truth_port);pause(.02);
events=io.takeCanonicalEnvironmentRecords();
check('actual_same_IO_raw_TX_RX_with_original_QPC',numel(events)==2&&strcmp(events{1}.kind,'ENV_TX') ...
    &&strcmp(events{2}.kind,'DIAGNOSTIC_RX')&&events{1}.original_host_send_ns>0 ...
    &&events{2}.original_host_receive_ns>=events{1}.original_host_send_ns);
liveLedger=gpenmpcNative.RflyLocalEnvironmentLedger(policy,1,16);liveLedger.sent(events{1});liveLedger.received(events{2});
check('actual_callbacks_feed_one_bounded_environment_owner',~isempty(liveLedger.resolve(time))&&isempty(io.takeCanonicalEnvironmentRecords()));
ev=io.evidence();check('no_arm_mode_control_in_localhost_change_test',isempty(ev.raw_transmit_messages));
io.close();io=[];delete(peer);peer=[];
report=struct('passed',all([checks.pass]),'checks',checks,'test_count',numel(checks),'hardware_actions',0,'COM',0, ...
    'actual_C_phase_rows',3,'model_runs',0,'scope','ACTUAL_C_PHASE_PLUS_EXPLICIT_MOCK_REGISTRATION_DIAGNOSTICS_AND_LOCALHOST_IO_ONLY');
save(fullfile(outputRoot,'RAW.mat'),'report','receipt','events','ev','sent');writeReport();diary off
catch ex
    if ~isempty(io),io.close();end;if ~isempty(peer),delete(peer);end
    f=fopen(fullfile(outputRoot,'FAILURE.txt'),'a');fprintf(f,'%s\n',getReport(ex,'extended','hyperlinks','off'));fclose(f);diary off;rethrow(ex)
end
    function check(n,yes),checks(end+1)=struct('name',n,'pass',logical(yes));assert(yes,'test:Check','%s',n);end
    function reject(n,fn),id='';try,fn();catch ex,id=ex.identifier;end;check(n,~isempty(id));end
    function writeReport(),f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);disp(jsonencode(report));end
end
function e=diagnostic(b,t,g,rx),b(25:32)=typecast(double(t),'uint8');b(225:232)=typecast(double(g),'uint8');e=struct('bytes',b,'original_host_receive_ns',rx);end
function v=frameValue(f)
v=struct('schema_version',f(1),'generation',f(2),'source_io_time_s',f(3),'task_reference_time_s',f(4),'payload_kg',f(5), ...
 'wind_ned_xy_mps',f(6:7),'mission_phase',f(8),'reference_jet_ned',f(9:20),'payload_generation',f(21), ...
 'task_clock_paused',logical(f(22)),'session_token',f(23),'board_min_rx_io_time_s',f(24), ...
 'board_valid_flags',f(25),'landed_state',f(26),'continuity_epoch',f(27),'service_release_generation',f(28));
end
function b=readbin(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');end
function v=readLE(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;v=v(:);end
function s=hex(b),s=upper(reshape(dec2hex(b,2).',1,[]));end
