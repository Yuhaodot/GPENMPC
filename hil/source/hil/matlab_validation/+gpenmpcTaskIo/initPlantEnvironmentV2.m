function s=initPlantEnvironmentV2(p,initialIoTimeS)
%#codegen
% Pure fixed-size state constructor. Call ONCE per explicitly new mission.
% No plant, transport, simulation clock, board, or payload is reset here.
% All age/gap/tolerance values are caller-supplied engineering policy.
% expected_session_token refers to an immutable external UID/boot/receiver/
% clock-map/runtime binding; it is NOT a rounded uint64 UID or authentication.
assert(isstruct(p)&&isscalar(p),'gpenmpcTaskIo:V2Policy','Explicit scalar policy required.');
assert(finiteScalar(initialIoTimeS)&&initialIoTimeS>=0,'gpenmpcTaskIo:V2Clock','Invalid initial I/O time.');
assert(finiteScalar(p.initial_payload_kg)&&p.initial_payload_kg>=0,'gpenmpcTaskIo:V2Policy','Invalid initial payload.');
assert(isa(p.initial_wind_ned_xy_mps,'double')&&isequal(size(p.initial_wind_ned_xy_mps),[2,1])&&all(isfinite(p.initial_wind_ned_xy_mps)),'gpenmpcTaskIo:V2Policy','Expected finite 2x1 wind.');
assert(isa(p.initial_reference_jet_ned,'double')&&isequal(size(p.initial_reference_jet_ned),[12,1])&&all(isfinite(p.initial_reference_jet_ned)),'gpenmpcTaskIo:V2Policy','Expected finite 12x1 reference.');
assert(integerIn(p.expected_session_token,1,flintmax),'gpenmpcTaskIo:V2Policy','Exact session token required.');
assert(finiteScalar(p.max_env_age_s)&&p.max_env_age_s>0&&finiteScalar(p.max_board_age_s)&&p.max_board_age_s>0,'gpenmpcTaskIo:V2Policy','Explicit positive freshness bounds required.');
assert(finiteScalar(p.max_future_skew_s)&&p.max_future_skew_s>=0&&finiteScalar(p.max_ground_sample_gap_s)&&p.max_ground_sample_gap_s>0,'gpenmpcTaskIo:V2Policy','Invalid skew/gap bound.');
assert(finiteScalar(p.unload_dwell_s)&&p.unload_dwell_s==8.0,'gpenmpcTaskIo:V2Policy','Canonical service requires exactly 8 s, not DV008 10 s.');
assert(finiteScalar(p.max_commit_delay_s)&&p.max_commit_delay_s>0,'gpenmpcTaskIo:V2Policy','Explicit commit age bound required.');
assert(finiteScalar(p.base_mass_kg)&&p.base_mass_kg>0&&finiteScalar(p.mass_bias_kg),'gpenmpcTaskIo:V2Policy','Explicit actual mass model required.');
assert(finiteScalar(p.mass_tolerance_kg)&&p.mass_tolerance_kg>=0,'gpenmpcTaskIo:V2Policy','Explicit mass readback tolerance required.');
assert(isa(p.service_payload_targets_kg,'double')&&isequal(size(p.service_payload_targets_kg),[4,1])&&all(isfinite(p.service_payload_targets_kg)),'gpenmpcTaskIo:V2Policy','Four current-task payload targets required.');
last=p.initial_payload_kg;
for k=1:4
    assert(p.service_payload_targets_kg(k)>=0&&p.service_payload_targets_kg(k)<last,'gpenmpcTaskIo:V2Policy','Service payloads must strictly decrease in frozen 5/3/2/4 order.');
    assert(p.base_mass_kg+p.service_payload_targets_kg(k)+p.mass_bias_kg>0,'gpenmpcTaskIo:V2Policy','Actual total mass must be positive.');
    last=p.service_payload_targets_kg(k);
end
env=struct('reference_jet_ned',p.initial_reference_jet_ned,'payload_kg',double(p.initial_payload_kg),'wind_xy_mps',p.initial_wind_ned_xy_mps);
s=struct('environment',env,'initialized_io_time_s',double(initialIoTimeS), ...
    'last_eval_io_time_s',double(initialIoTimeS),'last_commit_io_time_s',double(initialIoTimeS), ...
    'last_commit_core_time_s',-1.0,'expected_session_token',double(p.expected_session_token), ...
    'has_applied_frame',false,'applied_frame',zeros(28,1),'applied_frame_generation',0.0, ...
    'applied_payload_generation',0.0,'applied_source_io_time_s',0.0,'applied_task_time_s',0.0, ...
    'applied_total_mass_kg',double(p.base_mass_kg+p.initial_payload_kg+p.mass_bias_kg), ...
    'pending_valid',false,'pending_frame',zeros(28,1),'pending_io_time_s',0.0,'pending_unload',false, ...
    'board_frame',zeros(28,1),'has_board_frame',false,'continuity_epoch',0.0, ...
    'ground_disarmed_since_s',-1.0,'task_env_failed',false,'failure_code',uint8(0), ...
    'staged_count',uint32(0),'applied_count',uint32(0),'unload_count',uint32(0));
end
function tf=finiteScalar(v),tf=isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v);end
function tf=integerIn(v,a,b),tf=finiteScalar(v)&&v==fix(v)&&v>=a&&v<=b;end
