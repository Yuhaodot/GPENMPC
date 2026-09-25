function sample = decodeTruthPacket(datagram, options)
%DECODETRUTHPACKET Decode one official CopterSim truth datagram offline.
% This function performs no network, serial, file, controller, or board action.
% Its output is DIAGNOSTIC/EVALUATION ONLY. Never feed it into an estimator or
% controller as measured state. The caller must check sample.valid.
%
% Official source: RflySimSDK/ctrl/PX4MavCtrlV4.py, getTrueDataMsg().
% Windows wire layout is LITTLE ENDIAN, with zero-based byte offsets:
%   168 B / 4i24f7d: magic i32@0=123456789; id i32@4; type i32@8;
%     reserved i32@12; vel f32x3@16; Euler f32x3@28; quat f32x4@40;
%     RPM f32x8@56; accB f32x3@88; rateB f32x3@100; time f64@112;
%     position f64x3@120; native GPS f64x3@144.
%   112 B / 6i14f4d: magic i32@0=1234567891; id i32@4; type i32@8;
%     GPS i32x3@12 (lat*1e7, lon*1e7, alt*1e3); RPM f32x8@24;
%     vel f32x3@56; Euler f32x3@68; position f64x3@80; time f64@104.
%     Quaternion, acceleration and angular rate are absent, not synthesized.
%   200 B / legacy netDataShort: tg i32@0; payload length i32@4=152;
%     id i32@8; type i32@12; time f64@16; vel f32x3@24;
%     position f32x3@36; Euler f32x3@48; quat f32x4@60;
%     RPM f32x8@76; accB f32x3@108; rateB f32x3@120;
%     alignment padding @132:135; native GPS f64x3@136;
%     unused payload tail @160:199. Legacy has NO CHECKSUM; tg>=0 is
%     the SDK collision indication. Never mistake tg for a magic checksum.
%
% options: expected_copter_id / expected_vehicle_type (default NaN = any),
% quaternion_norm_tolerance (default 1e-3; serialized quaternion validation),
% require_checksum (default false). Fixed magic values
% identify formats; they do not provide CRC integrity or authentication.
% Time is checked finite/nonnegative only. Monotonicity/freshness require a
% sequence and receive clock and therefore remain the caller's responsibility.

if nargin < 2, options = struct(); end
options = validateOptions(options);
sample = emptySample();
if ~isa(datagram, 'uint8') || ~isvector(datagram) || isempty(datagram)
    sample.reason = 'INPUT_MUST_BE_NONEMPTY_UINT8_VECTOR';
    return
end
bytes = reshape(datagram, 1, []);
sample.packet_bytes = numel(bytes);

switch numel(bytes)
    case 168
        sample.format = 'SOut2Simulator_168';
        sample.checksum_present = true;
        sample.checksum_kind = 'FIXED_MAGIC_NOT_CRC';
        sample.checksum_value = readValue(bytes, 0, 1, 'int32');
        sample.checksum_valid = sample.checksum_value == 123456789;
        if ~sample.checksum_valid
            sample.reason = 'BAD_168_MAGIC'; return
        end
        sample.copter_id = readValue(bytes, 4, 1, 'int32');
        sample.vehicle_type = readValue(bytes, 8, 1, 'int32');
        sample.reserved = readValue(bytes, 12, 1, 'int32');
        sample.velocity_ned_mps = readValue(bytes, 16, 3, 'single');
        sample.euler_rad = readValue(bytes, 28, 3, 'single');
        sample.quaternion_wxyz = readValue(bytes, 40, 4, 'single');
        sample.motor_rpm = readValue(bytes, 56, 8, 'single');
        sample.acceleration_body_mps2 = readValue(bytes, 88, 3, 'single');
        sample.angular_rate_body_radps = readValue(bytes, 100, 3, 'single');
        sample.time_s = readValue(bytes, 112, 1, 'double');
        sample.position_ned_m = readValue(bytes, 120, 3, 'double');
        sample.gps_native = readValue(bytes, 144, 3, 'double');
        sample.gps_order = 'UNRESOLVED_NATIVE_ORDER_PRESERVED';
        sample.quaternion_present = true;
        sample.acceleration_present = true;
        sample.angular_rate_present = true;
    case 112
        sample.format = 'SOut2SimulatorSimpleTime_112';
        sample.checksum_present = true;
        sample.checksum_kind = 'FIXED_MAGIC_NOT_CRC';
        sample.checksum_value = readValue(bytes, 0, 1, 'int32');
        sample.checksum_valid = sample.checksum_value == 1234567891;
        if ~sample.checksum_valid
            sample.reason = 'BAD_112_MAGIC'; return
        end
        sample.copter_id = readValue(bytes, 4, 1, 'int32');
        sample.vehicle_type = readValue(bytes, 8, 1, 'int32');
        sample.gps_native = readValue(bytes, 12, 3, 'int32') ./ [1e7, 1e7, 1e3];
        sample.gps_order = 'LATITUDE_LONGITUDE_ALTITUDE';
        sample.motor_rpm = readValue(bytes, 24, 8, 'single');
        sample.velocity_ned_mps = readValue(bytes, 56, 3, 'single');
        sample.euler_rad = readValue(bytes, 68, 3, 'single');
        sample.position_ned_m = readValue(bytes, 80, 3, 'double');
        sample.time_s = readValue(bytes, 104, 1, 'double');
    case 200
        sample.format = 'netDataShort_SOut2SimulatorOld_200';
        sample.checksum_kind = 'NO_CHECKSUM_IN_OFFICIAL_LEGACY_FORMAT';
        sample.legacy_payload_bytes = readValue(bytes, 4, 1, 'int32');
        if sample.legacy_payload_bytes ~= 152
            sample.reason = 'BAD_LEGACY_PAYLOAD_LENGTH'; return
        end
        if options.require_checksum
            sample.reason = 'CHECKSUM_REQUIRED_BUT_LEGACY_HAS_NONE'; return
        end
        sample.collision_indicator_present = true;
        sample.collision_indicator = readValue(bytes, 0, 1, 'int32');
        sample.collision_detected = sample.collision_indicator >= 0;
        sample.copter_id = readValue(bytes, 8, 1, 'int32');
        sample.vehicle_type = readValue(bytes, 12, 1, 'int32');
        sample.time_s = readValue(bytes, 16, 1, 'double');
        sample.velocity_ned_mps = readValue(bytes, 24, 3, 'single');
        sample.position_ned_m = readValue(bytes, 36, 3, 'single');
        sample.euler_rad = readValue(bytes, 48, 3, 'single');
        sample.quaternion_wxyz = readValue(bytes, 60, 4, 'single');
        sample.motor_rpm = readValue(bytes, 76, 8, 'single');
        sample.acceleration_body_mps2 = readValue(bytes, 108, 3, 'single');
        sample.angular_rate_body_radps = readValue(bytes, 120, 3, 'single');
        sample.gps_native = readValue(bytes, 136, 3, 'double');
        sample.gps_order = 'UNRESOLVED_NATIVE_ORDER_PRESERVED';
        sample.quaternion_present = true;
        sample.acceleration_present = true;
        sample.angular_rate_present = true;
    otherwise
        sample.reason = 'UNSUPPORTED_PACKET_LENGTH'; return
end

if sample.copter_id < 1
    sample.reason = 'INVALID_COPTER_ID'; return
end
if isfinite(options.expected_copter_id) && sample.copter_id ~= options.expected_copter_id
    sample.reason = 'COPTER_ID_MISMATCH'; return
end
if isfinite(options.expected_vehicle_type) && sample.vehicle_type ~= options.expected_vehicle_type
    sample.reason = 'VEHICLE_TYPE_MISMATCH'; return
end
if ~isfinite(sample.time_s) || sample.time_s < 0
    sample.reason = 'INVALID_SIMULATION_TIME'; return
end
requiredState = [sample.position_ned_m, sample.velocity_ned_mps, ...
    sample.euler_rad, sample.motor_rpm, sample.gps_native];
if sample.acceleration_present
    requiredState = [requiredState, sample.acceleration_body_mps2]; %#ok<AGROW>
end
if sample.angular_rate_present
    requiredState = [requiredState, sample.angular_rate_body_radps]; %#ok<AGROW>
end
if any(~isfinite(requiredState))
    sample.reason = 'NONFINITE_STATE'; return
end
if sample.quaternion_present
    if any(~isfinite(sample.quaternion_wxyz))
        sample.reason = 'NONFINITE_QUATERNION'; return
    end
    sample.quaternion_norm = norm(sample.quaternion_wxyz);
    if abs(sample.quaternion_norm - 1) > options.quaternion_norm_tolerance
        sample.reason = 'QUATERNION_NORM_INVALID'; return
    end
end
sample.valid = true;
sample.reason = 'DECODED_DIAGNOSTIC_STATE';
end

function values = readValue(bytes, offset, count, type)
switch type
    case {'int32', 'single'}, width = 4;
    case 'double', width = 8;
    otherwise, error('m600check:InternalType', 'Unsupported internal type.');
end
typed = typecast(bytes(offset + (1:width*count)), type);
[~, ~, nativeEndian] = computer;
if nativeEndian == 'B', typed = swapbytes(typed); end
values = double(reshape(typed, 1, []));
end

function options = validateOptions(options)
assert(isstruct(options) && isscalar(options), 'm600check:InvalidOption', ...
    'options must be a scalar struct.');
defaults = struct('expected_copter_id', NaN, 'expected_vehicle_type', NaN, ...
    'quaternion_norm_tolerance', 1e-3, 'require_checksum', false);
supplied = fieldnames(options);
assert(all(ismember(supplied, fieldnames(defaults))), 'm600check:InvalidOption', ...
    'Unknown decoder option.');
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index})
        options.(names{index}) = defaults.(names{index});
    end
end
for name = {'expected_copter_id', 'expected_vehicle_type'}
    value = options.(name{1});
    assert(isnumeric(value) && isreal(value) && isscalar(value) && ...
        (isnan(value) || (isfinite(value) && value == fix(value))), ...
        'm600check:InvalidOption', 'Expected identity must be integer or NaN.');
end
tolerance = options.quaternion_norm_tolerance;
assert(isnumeric(tolerance) && isreal(tolerance) && isscalar(tolerance) && ...
    isfinite(tolerance) && tolerance >= 0 && tolerance < 1, ...
    'm600check:InvalidOption', 'Quaternion tolerance must lie in [0,1).');
assert(islogical(options.require_checksum) && isscalar(options.require_checksum), ...
    'm600check:InvalidOption', 'require_checksum must be a scalar logical.');
end

function sample = emptySample()
sample = struct('valid', false, 'reason', '', 'format', '', 'packet_bytes', 0, ...
    'diagnostic_only', true, 'controller_or_estimator_feed_permitted', false, ...
    'wire_endian', 'LITTLE_ENDIAN', 'checksum_present', false, ...
    'checksum_kind', '', 'checksum_value', NaN, 'checksum_valid', false, ...
    'copter_id', NaN, 'vehicle_type', NaN, 'reserved', NaN, ...
    'time_s', NaN, 'time_monotonicity_checked', false, 'freshness_checked', false, ...
    'position_ned_m', nan(1,3), 'velocity_ned_mps', nan(1,3), ...
    'euler_rad', nan(1,3), 'quaternion_wxyz', nan(1,4), ...
    'quaternion_present', false, 'quaternion_norm', NaN, ...
    'motor_rpm', nan(1,8), 'acceleration_body_mps2', nan(1,3), ...
    'acceleration_present', false, 'angular_rate_body_radps', nan(1,3), ...
    'angular_rate_present', false, 'gps_native', nan(1,3), 'gps_order', '', ...
    'legacy_payload_bytes', NaN, 'collision_indicator_present', false, ...
    'collision_indicator', NaN, 'collision_detected', false);
end
