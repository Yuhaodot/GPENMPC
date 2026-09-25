function [sample,state]=decodeCanonicalControlCache(bytes,expected,clock,state)
% Pure bounded last-snapshot observer. Every arriving packet is inspected;
% independent model-clock lag waits, never substitutes packet time as clock.
% The actual IO keeps ALL original packets. This state holds one latest
% accepted-step observation, not a queue of control or numerical executions.
if nargin<4||isempty(state)
    state=struct('failed',false,'failure_reason','','present',false,'expected',[], ...
        'bytes',zeros(216,1,'uint8'),'parsed',[],'original_host_receive_ns',uint64(0), ...
        'original_io_receive_s',NaN,'last_now_ns',uint64(0),'last_wall_s',-Inf,'last_model_s',NaN);
end
sample=struct('valid',false,'reason','','accepted_new_sample',false,'duplicate_ignored',false, ...
    'all16_zero',false,'generation',uint64(0),'sim_time_s',NaN,'input_call_count',uint64(0), ...
    'input16',nan(16,1),'original_host_receive_ns',uint64(0),'original_io_receive_s',NaN, ...
    'source_age_s',Inf,'receive_age_s',Inf,'runtime_origin_attested',false,'physical_isolation_proven',false);
try
    assert(isstruct(expected)&&isscalar(expected)&&isa(expected.session_token,'uint64') ...
        &&isscalar(expected.session_token)&&expected.session_token>0 ...
        &&isa(expected.dll_sha256,'uint8')&&isequal(size(expected.dll_sha256),[32 1])&&any(expected.dll_sha256) ...
        &&finitePositive(expected.maximum_source_age_s)&&finitePositive(expected.maximum_receive_age_s), ...
        'm600check:CacheExpected','Exact cache expected identity and bounds required.');
    assert(isa(clock.now_ns,'uint64')&&isscalar(clock.now_ns)&&clock.now_ns>0 ...
        &&finiteNonnegative(clock.now_wall_time_s) ...
        &&(finiteNonnegative(clock.now_sim_time_s)||(isa(clock.now_sim_time_s,'double')&&isscalar(clock.now_sim_time_s)&&isnan(clock.now_sim_time_s))), ...
        'm600check:CacheClock','Actual independent clock fields required.');
    if state.failed,sample.reason=state.failure_reason;return,end
    if isempty(state.expected),state.expected=expected;
    else,assert(isequaln(state.expected,expected),'m600check:CacheExpectedChanged','Observer context changed.');end
    assert(clock.now_ns>=state.last_now_ns&&clock.now_wall_time_s>=state.last_wall_s, ...
        'm600check:CacheHostClockReversed','Original HOST event clock reversed.');
    if isfinite(clock.now_sim_time_s)&&isfinite(state.last_model_s)
        assert(clock.now_sim_time_s>=state.last_model_s,'m600check:CacheModelClockReversed','Independent model clock reversed.');
    end
    state.last_now_ns=clock.now_ns;state.last_wall_s=clock.now_wall_time_s;
    if isfinite(clock.now_sim_time_s),state.last_model_s=clock.now_sim_time_s;end
    if ~isempty(bytes)
        [p,reason]=m600check.inspectCanonicalControlCachePacket(bytes,expected);
        assert(isempty(reason),'m600check:CachePacket','%s',reason);
        rx=clock.original_host_receive_ns;wall=clock.receive_wall_time_s;
        assert(isa(rx,'uint64')&&isscalar(rx)&&rx>0&&rx<=clock.now_ns ...
            &&finiteNonnegative(wall)&&wall<=clock.now_wall_time_s,'m600check:CacheReceiveClock','Original receive timestamp invalid.');
        duplicate=false;
        if ~state.present,assert(p.generation==1,'m600check:CacheInitialGeneration','First accepted generation must be one.');
        elseif p.generation==state.parsed.generation
            assert(isequal(bytes(:),state.bytes),'m600check:CacheGenerationConflict','Different packet for same generation.');duplicate=true;
        else
            assert(state.parsed.generation<intmax('uint64')&&p.generation==state.parsed.generation+uint64(1), ...
                'm600check:CacheGenerationGap','Accepted-step generation gap/reversal.');
            assert(p.sim_time_s>state.parsed.sim_time_s&&p.input_call_count>=state.parsed.input_call_count ...
                &&rx>=state.original_host_receive_ns&&wall>=state.original_io_receive_s, ...
                'm600check:CacheOriginalSourceReversed','Original sample/input-call/receive order reversed.');
        end
        if ~duplicate
            state.present=true;state.bytes=bytes(:);state.parsed=p;
            state.original_host_receive_ns=rx;state.original_io_receive_s=wall;sample.accepted_new_sample=true;
        end
        sample.duplicate_ignored=duplicate;
    end
    if ~state.present,sample.reason='CACHE_NOT_OBSERVED';return,end
    sample.generation=state.parsed.generation;sample.sim_time_s=state.parsed.sim_time_s;
    sample.input_call_count=state.parsed.input_call_count;sample.input16=state.parsed.input16;
    sample.original_host_receive_ns=state.original_host_receive_ns;sample.original_io_receive_s=state.original_io_receive_s;
    sample.receive_age_s=clock.now_wall_time_s-state.original_io_receive_s;
    % Both original HOST domains are checked; neither a poll nor duplicate
    % can reset the reception anchor. Existing caller bounds remain explicit.
    assert(sample.receive_age_s<=expected.maximum_receive_age_s ...
        &&double(clock.now_ns-state.original_host_receive_ns)*1e-9<=expected.maximum_receive_age_s, ...
        'm600check:CacheReceiveStale','Original receive lifetime expired.');
    if isnan(clock.now_sim_time_s),sample.reason='CACHE_WAITING_FOR_INDEPENDENT_MODEL_CLOCK';return,end
    sample.source_age_s=clock.now_sim_time_s-state.parsed.sim_time_s;
    if sample.source_age_s<0,sample.reason='CACHE_WAITING_FOR_INDEPENDENT_MODEL_CLOCK';return,end
    assert(sample.source_age_s<=expected.maximum_source_age_s,'m600check:CacheSourceStale','Original model sample expired.');
    sample.valid=true;sample.reason='VALID_SAME_ACCEPTED_STEP_INPUT_CACHE';sample.all16_zero=all(sample.input16==0);
catch ex
    if ~state.failed,state.failed=true;state.failure_reason=[ex.identifier ':' ex.message];end
    sample.valid=false;sample.all16_zero=false;sample.reason=state.failure_reason;
end
end
function yes=finitePositive(v),yes=finiteNonnegative(v)&&v>0;end
function yes=finiteNonnegative(v),yes=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0;end
