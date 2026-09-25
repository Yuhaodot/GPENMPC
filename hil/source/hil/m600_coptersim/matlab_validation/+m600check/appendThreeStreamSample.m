function log = appendThreeStreamSample(log, stream, sample)
%APPENDTHREESTREAMSAMPLE Append one observed state to a pure in-memory log.
% Pass [] for a
% new log. stream is exactly 'reference', 'estimate', or 'truth'. sample has:
%   clock_id          identifier for the caller-established common clock
%   time_s            source generation time on that common clock
%   received_at_s     receive/record time on the same clock
%   position_ned_m    three real numbers [north east down]
%   velocity_ned_mps  three real numbers [north east down]
% NaN/Inf and repeated/stale timestamps are deliberately preserved; the
% evaluator rejects/labels them. This recorder never silently cleans data.
% A raw PX4 boot timestamp must be mapped explicitly before it is passed in;
% this function does not guess its relationship to CopterSim simulation time.

stream = char(string(stream));
assert(ismember(stream, {'reference','estimate','truth'}), ...
    'm600check:RecordStream', 'Unknown stream.');
required = {'clock_id','time_s','received_at_s','position_ned_m','velocity_ned_mps'};
assert(isstruct(sample) && isscalar(sample) && all(isfield(sample, required)), ...
    'm600check:RecordShape', 'Sample fields do not satisfy the state contract.');
assert((ischar(sample.clock_id) && isrow(sample.clock_id)) || ...
    (isstring(sample.clock_id) && isscalar(sample.clock_id)), ...
    'm600check:RecordShape', 'clock_id must be scalar text.');
for name = {'time_s','received_at_s'}
    value = sample.(name{1});
    assert(isnumeric(value) && isreal(value) && isscalar(value), ...
        'm600check:RecordShape', 'Times must be real numeric scalars.');
end
for name = {'position_ned_m','velocity_ned_mps'}
    value = sample.(name{1});
    assert(isnumeric(value) && isreal(value) && isvector(value) && numel(value)==3, ...
        'm600check:RecordShape', 'State vectors must contain three real numbers.');
end
record = struct('clock_id', char(string(sample.clock_id)), ...
    'time_s', double(sample.time_s), 'received_at_s', double(sample.received_at_s), ...
    'position_ned_m', reshape(double(sample.position_ned_m),1,3), ...
    'velocity_ned_mps', reshape(double(sample.velocity_ned_mps),1,3));
if isempty(log)
    empty = repmat(record,0,1);
    log = struct('schema','M600_THREE_STREAM_DIAGNOSTIC_LOG_V1', ...
        'reference',empty,'estimate',empty,'truth',empty, ...
        'diagnostic_only',true,'controller_or_estimator_feed_permitted',false);
end
assert(isstruct(log) && isscalar(log) && ...
    isfield(log,'schema') && strcmp(log.schema,'M600_THREE_STREAM_DIAGNOSTIC_LOG_V1'), ...
    'm600check:RecordLog', 'Unexpected log schema.');
log.(stream)(end+1,1) = record;
end
