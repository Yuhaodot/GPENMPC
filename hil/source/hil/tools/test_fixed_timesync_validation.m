function result=test_fixed_timesync_validation(outputDir)
% Test fixed TIMESYNC validation with MATLAB fixtures.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'m600_coptersim','matlab_validation'));
addpath(fullfile(root,'matlab_validation'));
assert(~isfile(outputDir),'m600check:ClockTestOutput','Output cannot be a file.');
if isfolder(outputDir)
    contents=dir(outputDir);contents=contents(~ismember({contents.name},{'.','..'}));
    assert(isempty(contents),'m600check:ClockTestOutput','Output must be fresh or an empty incomplete attempt.');
else
    mkdir(outputDir);
end
checks=struct('name',{},'pass',{});calls=0;
firstError=struct('identifier','','message','');longSummary=struct();
c=struct('clock_max_rtt_s',.125,'clock_max_age_s',180.0, ...
    'clock_max_uncertainty_s',.125,'clock_sync_period_s',1.0, ...
    'clock_fixed_extra_uncertainty_s',.015625);
b=struct('token',int64(1),'send_s',0.0,'receive_s',.0625, ...
    'board_time_s',.03125,'rtt_s',.0625,'offset_lower_s',-.03125, ...
    'offset_upper_s',.03125,'accepted',true);
initial=struct('best',b,'last_accepted_board_time_s',.5, ...
    'last_issued_token',int64(100),'now_s',.5625);
try
    [s,r]=f([], 'INIT',initial,c);original=s.best;midBits=typecast(s.fixed_midpoint_s,'uint64');
    check('initial_mapping_unchanged_best_and_latest_distinct',r.mapping_valid&&s.last_accepted_board_time_s==.5&&s.best.board_time_s==.03125);
    check('initial_age_uses_original_best_conservatively',r.validation_age_s==.5&&r.last_validation_receive_s==b.receive_s);
    for k=1:1000
        [s,q]=f(s,'SENT',struct('token',int64(100+k),'send_s',double(k)),c);
        assert(q.request_registered,'m600check:ClockFixture','Healthy request registration failed.');
        [s,q]=f(s,'RESPONSE',response(100+k,k+.03125,k+.0625),c);
        assert(q.renewed&&q.mapping_valid,'m600check:ClockFixture','Healthy validation failed.');
        [s,q]=f(s,'POLL',struct('now_s',k+.5),c);
        assert(q.mapping_valid,'m600check:ClockFixture','Healthy midpoint became invalid.');
    end
    check('1000_seconds_continuous_healthy',s.renewal_count==1000&&s.last_event_time_s==1000.5&&r.mapping_valid);
    check('1000_seconds_fixed_best_and_midpoint_bitexact',isequal(s.best,original)&&typecast(s.fixed_midpoint_s,'uint64')==midBits);
    check('1000_seconds_pending_memory_bounded',numel(s.pending_active)==3&&nnz(s.pending_active)==0&&s.expired_count==0);
    check('uncertainty_not_underreported',q.uncertainty_s==.046875&&q.uncertainty_s>=q.fixed_initial_uncertainty_s);
    longSummary=struct('simulated_elapsed_s',1000.5,'renewals',s.renewal_count, ...
        'fixed_midpoint_s',s.fixed_midpoint_s,'max_uncertainty_s',s.uncertainty_s, ...
        'pending_capacity',numel(s.pending_active),'last_validation_receive_s',s.last_validation_receive_s);
    last=s.last_validation_receive_s;
    [s,q]=f(s,'POLL',struct('now_s',last+c.clock_max_age_s),c);
    check('age_equal_existing_bound_valid',q.mapping_valid&&q.validation_age_s==180);
    [s,q]=f(s,'POLL',struct('now_s',last+c.clock_max_age_s+.001),c);
    check('loss_past_existing_bound_invalid_no_renewal',~q.mapping_valid&&~q.renewed&&~s.fatal_latched&&s.last_validation_receive_s==last);

    [s,~]=fresh(c);last=s.last_validation_receive_s;
    [s,q]=f(s,'RESPONSE',response(999,.6,.625),c);
    check('unknown_token_never_renews',~q.renewed&&s.last_validation_receive_s==last&&s.unknown_or_retired_count==1);
    [s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,1.03125,1.0625),c);last=s.last_validation_receive_s;
    check('matched_token_renews_once',q.renewed&&s.renewal_count==1);
    [s,q]=f(s,'RESPONSE',response(101,1.04,1.07),c);
    check('duplicate_reply_never_renews',~q.renewed&&s.last_validation_receive_s==last&&s.renewal_count==1);
    [s,q]=f(s,'SENT',struct('token',int64(101),'send_s',2.0),c);
    check('consumed_token_cannot_be_reused',~q.request_registered&&~q.renewed&&s.registered_count==1);
    [s,q]=f(s,'SENT',struct('token',102.0,'send_s',2.1),c);
    check('double_token_not_silently_converted',~q.request_registered);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,1.0625,1.125+eps(1.125)),c);
    check('RTT_above_bound_no_renewal',~q.renewed&&s.high_rtt_count==1&&s.last_validation_receive_s==b.receive_s&&~s.fatal_latched);
    [s,q]=f(s,'RESPONSE',response(101,1.0625,1.13),c);
    check('rejected_high_RTT_token_is_consumed',~q.renewed&&s.high_rtt_count==1);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,1.0625,1.125),c);
    check('RTT_equal_bound_validates',q.renewed&&q.rtt_s==c.clock_max_rtt_s);
    check('wider_accepted_interval_updates_reported_uncertainty',q.uncertainty_s==.078125&&q.uncertainty_s>q.fixed_initial_uncertainty_s);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,1.15,1.125),c);
    check('overlap_without_complete_bound_never_renews',~q.renewed&&~q.fatal_latched&&s.unproven_count==1&&s.last_validation_receive_s==b.receive_s);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,1.3,1.0625),c);
    check('incompatible_offset_permanently_invalid',q.fatal_latched&&strcmp(q.first_failure,'FIXED_OFFSET_INCOMPATIBLE')&&~q.mapping_valid);
    first=q.first_failure;[s,q]=f(s,'SENT',struct('token',int64(102),'send_s',2.0),c);
    check('offset_fault_cannot_register_or_renew',~q.request_registered&&strcmp(q.first_failure,first)&&s.renewal_count==0);
    [s,q]=f(s,'INIT',initial,c);
    check('INIT_cannot_wash_prior_fatal',q.fatal_latched&&strcmp(q.first_failure,first)&&typecast(q.fixed_midpoint_s,'uint64')==midBits);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,.25,1.0625),c);
    check('source_reversal_compares_last_accepted_not_best',q.fatal_latched&&strcmp(q.first_failure,'MATCHED_BOARD_SOURCE_REVERSAL')&&.25>b.board_time_s);
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    [s,q]=f(s,'RESPONSE',response(101,.5,1.0625),c);
    check('nonprogressing_source_never_renews',~q.renewed&&~q.fatal_latched&&s.source_nonprogress_count==1);
    [s,~]=fresh(c);[s,q]=f(s,'POLL',struct('now_s',.55),c);
    check('host_time_reversal_permanently_invalid',q.fatal_latched&&strcmp(q.first_failure,'HOST_TIME_REVERSAL'));
    [s,~]=fresh(c);changed=c;changed.clock_max_age_s=181;
    [s,q]=f(s,'POLL',struct('now_s',.6),changed);
    check('configuration_cannot_be_widened_after_INIT',q.fatal_latched&&strcmp(q.first_failure,'CLOCK_CONFIGURATION_CHANGED')&&s.configuration(2)==180);
    [s,~]=fresh(c);[s,q]=f(s,'INIT',initial,c);
    check('second_INIT_never_restarts_owner',q.fatal_latched&&strcmp(q.first_failure,'INITIALIZATION_REUSE_FORBIDDEN'));
    [s,~]=fresh(c);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),c);
    bad=response(101,1.03125,1.0625);bad.tc1_ns=double(bad.tc1_ns);
    [s,q]=f(s,'RESPONSE',bad,c);
    check('board_nanoseconds_require_exact_int64',~q.renewed&&s.last_validation_receive_s==b.receive_s);
    [s,q]=f(s,'POLL',struct('now_s',1.126),c);
    check('expired_pending_retired_without_renewal',s.expired_count==1&&~any(s.pending_active)&&s.last_validation_receive_s==b.receive_s);
    [s,q]=f(s,'RESPONSE',response(101,1.03,1.127),c);
    check('retired_reply_cannot_renew',~q.renewed&&s.unknown_or_retired_count==1);
    dense=c;dense.clock_sync_period_s=.03125;[s,~]=fresh(dense);capacity=numel(s.pending_active);
    for k=1:capacity,[s,q]=f(s,'SENT',struct('token',int64(100+k),'send_s',.6+k*.001),dense);assert(q.request_registered);end
    [s,q]=f(s,'SENT',struct('token',int64(101+capacity),'send_s',.61),dense);
    check('pending_overflow_does_not_authorize_send',~q.request_registered&&q.pending_count==capacity&&numel(s.pending_active)==capacity);
    [s,q]=f(s,'SENT',struct('token',int64(102+capacity),'send_s',1.0),dense);
    check('pending_expiry_releases_slots_without_token_reuse',q.request_registered&&q.pending_count==1&&s.expired_count==capacity);
    edge=c;edge.clock_fixed_extra_uncertainty_s=0;[s,~]=fresh(edge);
    [s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),edge);
    [s,q]=f(s,'RESPONSE',response(101,1.125,1.0),edge);
    check('uncertainty_equal_bound_valid',q.renewed&&q.uncertainty_s==.125&&q.mapping_valid);
    [s,~]=fresh(edge);[s,~]=f(s,'SENT',struct('token',int64(101),'send_s',1.0),edge);
    [s,q]=f(s,'RESPONSE',response(101,1.126,1.0),edge);
    check('uncertainty_outside_bound_permanent_no_epsilon_relaxation',~q.renewed&&q.fatal_latched);
    check('pure_fixture_has_zero_hardware_actions',true);
catch ex
    firstError=struct('identifier',ex.identifier,'message',getReport(ex,'extended','hyperlinks','off'));
end
result=struct('status','PASS_HOST_ONLY_FIXED_TIMESYNC_VALIDATION','checks_passed',nnz([checks.pass]), ...
    'checks_total',numel(checks),'checks',checks,'function_calls',calls,'long_run',longSummary, ...
    'first_error',firstError,'configuration',c,'source_file',which('m600check.advanceFixedTimesyncValidation'), ...
    'source_sha256',m600check.fileSha256(which('m600check.advanceFixedTimesyncValidation')), ...
    'test_sha256',m600check.fileSha256([mfilename('fullpath') '.m']), ...
    'hardware_actions',struct('com',0,'udp',0,'board',0,'arm',0,'write',0,'model',0), ...
    'claim','Event replay tests the midpoint clock mapping; UTC, heartbeat, identity and abort latches remain separate checks.');
if ~isempty(firstError.identifier)||~all([checks.pass]),result.status='FAIL_HOST_ONLY_TEST';end
fid=fopen(fullfile(outputDir,'RESULT.json'),'w');assert(fid>=0,'m600check:ClockTestOutput','Cannot open RESULT.');
cleaner=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));clear cleaner;
fprintf('%s %d/%d checks, %d pure calls\n',result.status,result.checks_passed,result.checks_total,calls);
if strcmp(result.status,'FAIL_HOST_ONLY_TEST'),error('m600check:ClockTestFailed','%s',firstError.message);end
    function check(name,pass)
        pass=islogical(pass)&&isscalar(pass)&&pass;checks(end+1)=struct('name',name,'pass',pass);
        assert(pass,'m600check:ClockCheck','Failed: %s',name);
    end
    function [state,receipt]=f(state,op,event,configuration)
        calls=calls+1;[state,receipt]=m600check.advanceFixedTimesyncValidation(state,op,event,configuration);
    end
    function [state,receipt]=fresh(configuration)
        [state,receipt]=f([],'INIT',initial,configuration);
    end
end
function e=response(token,board,receive)
e=struct('ts1_token',int64(token),'tc1_ns',int64(round(board*1e9)),'receive_s',double(receive));
end
