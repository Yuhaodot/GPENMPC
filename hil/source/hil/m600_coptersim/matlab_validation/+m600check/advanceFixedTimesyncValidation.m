function [s,r]=advanceFixedTimesyncValidation(s,op,e,cfg)
% Pure, bounded validation of ONE already-locked PX4 TIMESYNC mapping.
% INIT e: best (original accepted row), last_accepted_board_time_s,
%         last_issued_token (int64), now_s. Initialize exactly once per owner.
% SENT e: token (strict increasing positive int64), send_s. Register BEFORE
%         sending; only r.request_registered authorizes the caller to send.
% RESPONSE e: ts1_token (int64 echo), tc1_ns (positive int64), receive_s.
% POLL e: now_s. Responses renew age, NEVER best/fixed_midpoint_s.
% cfg: clock_max_rtt_s, clock_max_age_s, clock_max_uncertainty_s,
%      clock_sync_period_s, clock_fixed_extra_uncertainty_s. The last value
% is the original fixed UTC-pair uncertainty plus original quantization
% allowance (adapter currently adds 0.0005 s).
% A low-RTT response proves renewal only when its WHOLE offset interval is
% within the original uncertainty budget around the FIXED midpoint. Disjoint
% intervals permanently latch invalid; overlap without a proven bound does
% not renew. Unknown, reused, expired and high-RTT responses never renew.
% Age expiry alone is reported, not erased from any caller-owned abort latch.
% Source rollback records a regression in matched responses. Reboot
% classification and UTC/heartbeat/identity checks remain external.
names={'clock_max_rtt_s','clock_max_age_s','clock_max_uncertainty_s', ...
    'clock_sync_period_s','clock_fixed_extra_uncertainty_s'};
v=zeros(1,5);
for k=1:5
    assert(isfield(cfg,names{k})&&isa(cfg.(names{k}),'double')&& ...
        isscalar(cfg.(names{k}))&&isreal(cfg.(names{k}))&&isfinite(cfg.(names{k})), ...
        'm600check:ClockConfiguration','Explicit finite double clock configuration required.');
    v(k)=cfg.(names{k});
end
assert(all(v(1:4)>0)&&v(5)>=0&&v(5)<=v(3), ...
    'm600check:ClockConfiguration','Existing positive clock bounds and fixed extra uncertainty required.');
assert(ischar(op)&&isrow(op)&&isstruct(e)&&isscalar(e), ...
    'm600check:ClockEventSchema','Operation and scalar event structure required.');
r=struct('operation',op,'reason','','request_registered',false,'renewed',false, ...
    'mapping_valid',false,'fatal_latched',false,'first_failure','', ...
    'fixed_midpoint_s',NaN,'fixed_initial_uncertainty_s',Inf, ...
    'uncertainty_s',Inf,'validation_age_s',Inf,'last_validation_receive_s',NaN, ...
    'event_time_s',NaN,'token',int64(0),'board_time_s',NaN,'rtt_s',NaN, ...
    'offset_lower_s',NaN,'offset_upper_s',NaN,'candidate_uncertainty_s',Inf, ...
    'pending_count',0,'pending_capacity',0,'source_mapping_changed',false);
if isempty(s)
    assert(strcmp(op,'INIT'),'m600check:ClockInitialization','First operation must be INIT.');
    assert(isfield(e,'best')&&isstruct(e.best)&&isscalar(e.best), ...
        'm600check:ClockInitialization','Original best TIMESYNC row required.');
    b=e.best;bn={'send_s','receive_s','board_time_s','rtt_s','offset_lower_s','offset_upper_s'};
    for k=1:numel(bn)
        assert(isfield(b,bn{k})&&finiteScalar(b.(bn{k})), ...
            'm600check:ClockInitialization','Original best row has invalid numeric fields.');
    end
    assert(isfield(b,'accepted')&&islogical(b.accepted)&&isscalar(b.accepted)&&b.accepted&& ...
        b.send_s>=0&&b.receive_s>=b.send_s&&b.board_time_s>=0&& ...
        b.rtt_s==b.receive_s-b.send_s&&b.rtt_s<=v(1)&& ...
        b.offset_lower_s==b.send_s-b.board_time_s&& ...
        b.offset_upper_s==b.receive_s-b.board_time_s, ...
        'm600check:ClockInitialization','Original best row must be accepted and internally consistent.');
    assert(isfield(e,'now_s')&&finiteScalar(e.now_s)&&e.now_s>=b.receive_s&& ...
        isfield(e,'last_accepted_board_time_s')&&finiteScalar(e.last_accepted_board_time_s)&& ...
        e.last_accepted_board_time_s>=b.board_time_s&& ...
        isfield(e,'last_issued_token')&&tokenOk(e.last_issued_token), ...
        'm600check:ClockInitialization','Latest accepted source, issued token and current time required.');
    capacity=ceil(v(1)/v(4))+2;
    assert(isfinite(capacity)&&capacity<=65536, ...
        'm600check:ClockPendingMemory','Clock cadence requires excessive pending storage; no allocation performed.');
    s=struct('schema',1,'configuration',v,'best',b, ...
        'fixed_midpoint_s',(b.offset_lower_s+b.offset_upper_s)/2, ...
        'fixed_initial_uncertainty_s',b.rtt_s/2+v(5), ...
        'uncertainty_s',b.rtt_s/2+v(5), ...
        'last_validation_receive_s',b.receive_s,'last_event_time_s',e.now_s, ...
        'last_accepted_board_time_s',e.last_accepted_board_time_s, ...
        'last_issued_token',e.last_issued_token,'fatal_latched',false,'first_failure','', ...
        'pending_tokens',zeros(capacity,1,'int64'),'pending_send_s',zeros(capacity,1), ...
        'pending_active',false(capacity,1),'registered_count',0,'renewal_count',0, ...
        'unknown_or_retired_count',0,'high_rtt_count',0,'unproven_count',0, ...
        'expired_count',0,'registration_rejected_count',0,'source_nonprogress_count',0);
    if s.uncertainty_s>v(3),s=latch(s,'INITIAL_UNCERTAINTY_EXCEEDS_EXISTING_BOUND');end
    r.reason='INITIAL_FIXED_MAPPING';r.event_time_s=e.now_s;
else
    assert(isstruct(s)&&isscalar(s)&&isfield(s,'schema')&&s.schema==1, ...
        'm600check:ClockState','Existing validation state required.');
    if ~isequal(s.configuration,v)
        s=latch(s,'CLOCK_CONFIGURATION_CHANGED');r.reason='CLOCK_CONFIGURATION_CHANGED';
    elseif strcmp(op,'INIT')
        s=latch(s,'INITIALIZATION_REUSE_FORBIDDEN');r.reason='INITIALIZATION_REUSE_FORBIDDEN';
    else
        switch op
            case 'SENT',field='send_s';
            case 'RESPONSE',field='receive_s';
            case 'POLL',field='now_s';
            otherwise,field='';
        end
        if isempty(field)||~isfield(e,field)||~finiteScalar(e.(field))||e.(field)<0
            s=latch(s,'INVALID_EVENT_SCHEMA');r.reason='INVALID_EVENT_SCHEMA';
        else
            now=e.(field);r.event_time_s=now;
            if now<s.last_event_time_s
                s=latch(s,'HOST_TIME_REVERSAL');r.reason='HOST_TIME_REVERSAL';
            elseif s.fatal_latched
                r.reason='ALREADY_FATAL_NO_RENEWAL';
            else
                s.last_event_time_s=now;
                if strcmp(op,'SENT')||strcmp(op,'POLL')
                    expired=s.pending_active&(now-s.pending_send_s>v(1));
                    s.expired_count=s.expired_count+nnz(expired);s.pending_active(expired)=false;
                end
                if strcmp(op,'SENT')
                    if ~isfield(e,'token')||~tokenOk(e.token)||e.token<=s.last_issued_token
                        s.registration_rejected_count=s.registration_rejected_count+1;
                        r.reason='TOKEN_INVALID_OR_REUSED_NO_SEND';
                    else
                        r.token=e.token;k=find(~s.pending_active,1);
                        if isempty(k)
                            s.registration_rejected_count=s.registration_rejected_count+1;
                            r.reason='PENDING_CAPACITY_NO_SEND';
                        else
                            s.pending_tokens(k)=e.token;s.pending_send_s(k)=now;s.pending_active(k)=true;
                            s.last_issued_token=e.token;s.registered_count=s.registered_count+1;
                            r.request_registered=true;r.reason='REQUEST_REGISTERED';
                        end
                    end
                elseif strcmp(op,'RESPONSE')
                    if ~isfield(e,'ts1_token')||~tokenOk(e.ts1_token)|| ...
                            ~isfield(e,'tc1_ns')||~tokenOk(e.tc1_ns)
                        r.reason='RESPONSE_IDENTIFIER_INVALID_NO_RENEWAL';
                    else
                        r.token=e.ts1_token;k=find(s.pending_active&s.pending_tokens==e.ts1_token,1);
                        if isempty(k)
                            s.unknown_or_retired_count=s.unknown_or_retired_count+1;
                            r.reason='UNKNOWN_OR_RETIRED_TOKEN_NO_RENEWAL';
                        else
                            tx=s.pending_send_s(k);s.pending_active(k)=false;
                            board=double(e.tc1_ns)/1e9;r.board_time_s=board;r.rtt_s=now-tx;
                            r.offset_lower_s=tx-board;r.offset_upper_s=now-board;
                            if r.rtt_s<0||r.rtt_s>v(1)
                                s.high_rtt_count=s.high_rtt_count+1;r.reason='RTT_NO_RENEWAL';
                            elseif board<s.last_accepted_board_time_s
                                s=latch(s,'MATCHED_BOARD_SOURCE_REVERSAL');r.reason=s.first_failure;
                            elseif board==s.last_accepted_board_time_s
                                s.source_nonprogress_count=s.source_nonprogress_count+1;
                                r.reason='BOARD_SOURCE_NOT_ADVANCING_NO_RENEWAL';
                            else
                                m=s.fixed_midpoint_s;available=v(3)-v(5);
                                r.candidate_uncertainty_s=max(abs([r.offset_lower_s-m,r.offset_upper_s-m]))+v(5);
                                if r.offset_lower_s>m+available||r.offset_upper_s<m-available
                                    s=latch(s,'FIXED_OFFSET_INCOMPATIBLE');r.reason=s.first_failure;
                                elseif r.candidate_uncertainty_s>v(3)
                                    s.unproven_count=s.unproven_count+1;
                                    r.reason='OFFSET_UNCERTAINTY_UNPROVEN_NO_RENEWAL';
                                else
                                    s.last_accepted_board_time_s=board;s.last_validation_receive_s=now;
                                    % Maximum of accepted bounds: never understate established error.
                                    s.uncertainty_s=max(s.uncertainty_s,r.candidate_uncertainty_s);
                                    s.renewal_count=s.renewal_count+1;r.renewed=true;
                                    r.reason='MATCHED_FIXED_MAPPING_VALIDATED';
                                end
                            end
                        end
                    end
                else
                    r.reason='CLOCK_POLL';
                end
            end
        end
    end
end
r.fixed_midpoint_s=s.fixed_midpoint_s;r.fixed_initial_uncertainty_s=s.fixed_initial_uncertainty_s;
r.uncertainty_s=s.uncertainty_s;r.last_validation_receive_s=s.last_validation_receive_s;
r.validation_age_s=s.last_event_time_s-s.last_validation_receive_s;
r.fatal_latched=s.fatal_latched;r.first_failure=s.first_failure;
r.pending_count=nnz(s.pending_active);r.pending_capacity=numel(s.pending_active);
r.mapping_valid=~s.fatal_latched&&r.validation_age_s>=0&&r.validation_age_s<=s.configuration(2)&& ...
    r.uncertainty_s<=s.configuration(3);
if strcmp(r.reason,'CLOCK_POLL')&&~r.mapping_valid&&~s.fatal_latched,r.reason='VALIDATION_AGE_EXPIRED';end
end
function yes=finiteScalar(x)
yes=isa(x,'double')&&isreal(x)&&isscalar(x)&&isfinite(x);
end
function yes=tokenOk(x)
yes=isa(x,'int64')&&isreal(x)&&isscalar(x)&&x>0;
end
function s=latch(s,reason)
if ~s.fatal_latched,s.first_failure=reason;end
s.fatal_latched=true;
end
