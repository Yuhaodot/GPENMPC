function result = run_official_hex_hil_smoke(outputDir)
% RUN_OFFICIAL_HEX_HIL_SMOKE Run the CopterSim/Pixhawk hexacopter smoke test.
% CopterSim owns dynamics, sensors and COM; MATLAB uses MAVLink UDP.
% Keep physical PWM functions zero throughout the run.

arguments
    outputDir (1,1) string
end

assert(~isfolder(outputDir), 'gpenmpc:OfficialHilOutputExists', ...
    'Output directory already exists: %s', outputDir);
mkdir(outputDir);

dialect = mavlinkdialect('common.xml');
link = mavlinkio(dialect, 'SystemID', 255, 'ComponentID', 190);
connect(link, 'UDP', 'LocalPort', 14550);
client = mavlinkclient(link, 1, 1);

subs = struct( ...
    'heartbeat', mavlinksub(link, client, 'HEARTBEAT', 'BufferSize', 2000), ...
    'extended', mavlinksub(link, client, 'EXTENDED_SYS_STATE', 'BufferSize', 2000), ...
    'local', mavlinksub(link, client, 'LOCAL_POSITION_NED', 'BufferSize', 10000), ...
    'attitude', mavlinksub(link, client, 'ATTITUDE', 'BufferSize', 10000), ...
    'estimator', mavlinksub(link, client, 'ESTIMATOR_STATUS', 'BufferSize', 4000), ...
    'actuator', mavlinksub(link, client, 'HIL_ACTUATOR_CONTROLS', 'BufferSize', 10000), ...
    'statusText', mavlinksub(link, client, 'STATUSTEXT', 'BufferSize', 4000));

remoteHost = '127.0.0.1';
remotePort = 18570;
runClock = tic;
mappedNames = strings(0,1);
parameterReceipts = repmat(struct('name', "", 'before', NaN, ...
    'after', NaN, 'mav_type', NaN, 'write_count', 0, ...
    'ack_verified', false), 0, 1);
commandReceipts = repmat(struct('label', '', 'command', NaN, ...
    'sent_elapsed_s', NaN, 'ack_received', false, ...
    'ack_result', NaN, 'accepted', false), 0, 1);
rows = repmat(emptyTelemetryRow(), 0, 1);
forceDisarmCount = 0;
standardDisarmCount = 0;
armRequestCount = 0;
offboardRequestCount = 0;
landRequestCount = 0;
cleanupErrors = strings(0,1);
finalized = false;
failure = struct('present', false, 'identifier', '', 'message', '', ...
    'report', '');

cleanupGuard = onCleanup(@finalize); %#ok<NASGU>

try
    for index = 1:10
        sendHeartbeat();
        pause(0.1);
    end
    pause(1.0);

    topicsAtStart = listTopics(link);
    requireTopic(topicsAtStart, 'ATTITUDE');
    requireTopic(topicsAtStart, 'LOCAL_POSITION_NED');
    requireTopic(topicsAtStart, 'ESTIMATOR_STATUS');

    guardNames = ["SYS_HITL", "SYS_AUTOSTART", "MAV_TYPE", ...
        "CA_ROTOR_COUNT", "RA_CTRL_MODE", ...
        compose("HIL_ACT_FUNC%d", 1:16), ...
        compose("PWM_MAIN_FUNC%d", 1:8)];
    guards = repmat(emptyParameterRow(), numel(guardNames), 1);
    for index = 1:numel(guardNames)
        guards(index) = readParameter(guardNames(index));
    end
    requireDecoded(guards, 'SYS_HITL', 1);
    requireDecoded(guards, 'SYS_AUTOSTART', 6001);
    requireDecoded(guards, 'MAV_TYPE', 13);
    requireDecoded(guards, 'CA_ROTOR_COUNT', 6);
    requireDecoded(guards, 'RA_CTRL_MODE', 0);
    for index = 1:16
        requireDecoded(guards, sprintf('HIL_ACT_FUNC%d', index), 0);
    end
    for index = 1:8
        requireDecoded(guards, sprintf('PWM_MAIN_FUNC%d', index), 0);
    end

    warmupStart = toc(runClock);
    while toc(runClock) - warmupStart < 5.0
        sendHeartbeatIfDue();
        sampleTelemetry('SENSOR_WARMUP');
        pause(0.05);
    end
    warmupRows = rows(strcmp({rows.phase}, 'SENSOR_WARMUP'));
    assert(numel(warmupRows) >= 50, 'gpenmpc:OfficialHilWarmupRows', ...
        'Insufficient warm-up telemetry rows.');
    localTimes = [warmupRows.local_time_boot_ms];
    attitudeTimes = [warmupRows.attitude_time_boot_ms];
    assert(numel(unique(localTimes(isfinite(localTimes)))) >= 25, ...
        'gpenmpc:OfficialHilLocalPositionStale', ...
        'LOCAL_POSITION_NED did not update continuously.');
    assert(numel(unique(attitudeTimes(isfinite(attitudeTimes)))) >= 50, ...
        'gpenmpc:OfficialHilAttitudeStale', ...
        'ATTITUDE did not update continuously.');

    for index = 1:6
        name = sprintf('HIL_ACT_FUNC%d', index);
        % Record the intended target before sending PARAM_SET so an ACK
        % timeout or parser exception cannot hide a possibly applied write.
        mappedNames(end+1,1) = string(name); %#ok<AGROW>
        receipt = setIntegerParameter(name, 100 + index);
        parameterReceipts(end+1,1) = receipt; %#ok<AGROW>
    end
    for index = 1:8
        check = readParameter(sprintf('PWM_MAIN_FUNC%d', index));
        assert(check.decoded == 0, 'gpenmpc:PhysicalPwmMappingChanged', ...
            'Physical PWM mapping changed during virtual HIL setup.');
    end

    initial = latestPayload(subs.local);
    assert(~isempty(initial), 'gpenmpc:OfficialHilNoLocalPosition', ...
        'No local position before Offboard prestream.');
    target = [double(initial.x), double(initial.y), -1.0];

    prestreamStart = toc(runClock);
    while toc(runClock) - prestreamStart < 2.5
        sendSetpoint(target);
        sendHeartbeatIfDue();
        sampleTelemetry('OFFBOARD_PRESTREAM');
        pause(0.05);
    end

    offboardRequestCount = offboardRequestCount + 1;
    commandReceipts(end+1,1) = sendCommand(176, [1, 6, 0, 0, 0, 0, 0], ...
        'REQUEST_OFFBOARD'); %#ok<AGROW>
    assert(commandReceipts(end).accepted, 'gpenmpc:OffboardRejected', ...
        'PX4 rejected the Offboard mode request.');

    modeDeadline = tic;
    while toc(modeDeadline) < 5.0
        sendSetpoint(target);
        sampleTelemetry('OFFBOARD_CONFIRM');
        hb = latestPayload(subs.heartbeat);
        if ~isempty(hb) && ...
                bitand(bitshift(uint32(hb.custom_mode), -16), uint32(255)) == 6
            break
        end
        pause(0.05);
    end

    armRequestCount = armRequestCount + 1;
    commandReceipts(end+1,1) = sendCommand(400, [1, 0, 0, 0, 0, 0, 0], ...
        'REQUEST_ARM'); %#ok<AGROW>
    assert(commandReceipts(end).accepted, 'gpenmpc:ArmRejected', ...
        'PX4 rejected the logical arm request.');

    armDeadline = tic;
    armedObserved = false;
    while toc(armDeadline) < 8.0
        sendSetpoint(target);
        sampleTelemetry('ARM_CONFIRM');
        if boardArmed()
            armedObserved = true;
            break
        end
        pause(0.05);
    end
    assert(armedObserved, 'gpenmpc:ArmStateNotObserved', ...
        'Arm ACK was accepted but no armed heartbeat was observed.');

    flightStart = toc(runClock);
    while toc(runClock) - flightStart < 20.0
        sendSetpoint(target);
        sendHeartbeatIfDue();
        sampleTelemetry('OFFICIAL_HEX_HOVER');
        pause(0.05);
    end

    landRequestCount = landRequestCount + 1;
    commandReceipts(end+1,1) = sendCommand(176, [1, 4, 6, 0, 0, 0, 0], ...
        'REQUEST_AUTO_LAND'); %#ok<AGROW>
    assert(commandReceipts(end).accepted, 'gpenmpc:NativeLandRejected', ...
        'PX4 rejected the native LAND request.');

    landingStart = toc(runClock);
    while toc(runClock) - landingStart < 30.0
        sendHeartbeatIfDue();
        sampleTelemetry('NATIVE_LAND');
        if boardLanded() && ~boardArmed()
            break
        end
        pause(0.05);
    end

    if boardArmed() && boardLanded()
        standardDisarmCount = standardDisarmCount + 1;
        commandReceipts(end+1,1) = sendCommand(400, [0, 0, 0, 0, 0, 0, 0], ...
            'REQUEST_STANDARD_DISARM'); %#ok<AGROW>
        waitForDisarmed(5.0);
    end
catch error
    failure = struct('present', true, 'identifier', error.identifier, ...
        'message', error.message, 'report', ...
        getReport(error, 'extended', 'hyperlinks', 'off'));
end

finalize();
clear cleanupGuard

telemetryTable = struct2table(rows);
writetable(telemetryTable, fullfile(outputDir, 'TELEMETRY.csv'));
topicsAtEnd = listTopics(link);

hoverRows = rows(strcmp({rows.phase}, 'OFFICIAL_HEX_HOVER'));
if isempty(hoverRows)
    maxDisplacementM = NaN;
    actuatorNonzeroRows = 0;
else
    xyz = [[hoverRows.x].', [hoverRows.y].', [hoverRows.z].'];
    origin = xyz(1,:);
    maxDisplacementM = max(vecnorm(xyz - origin, 2, 2), [], 'omitnan');
    actuatorNonzeroRows = nnz([hoverRows.actuator_max_abs] > 1e-4);
end

finalParameters = repmat(emptyParameterRow(), 14, 1);
finalNames = [compose("HIL_ACT_FUNC%d", 1:6), ...
    compose("PWM_MAIN_FUNC%d", 1:8)];
for index = 1:numel(finalNames)
    try
        finalParameters(index) = readParameter(finalNames(index));
    catch error
        finalParameters(index) = emptyParameterRow();
        finalParameters(index).name = finalNames(index);
        finalParameters(index).error = string(error.message);
    end
end
statusTextMessages = latestmsgs(subs.statusText, 4000);
statusTextRows = repmat(struct('severity', NaN, 'text', ""), ...
    numel(statusTextMessages), 1);
for index = 1:numel(statusTextMessages)
    statusTextRows(index).severity = ...
        double(statusTextMessages(index).Payload.severity);
    statusTextRows(index).text = ...
        cleanMavText(statusTextMessages(index).Payload.text);
end

checks = struct( ...
    'sensor_stream_continuous', numel(rows) >= 50, ...
    'offboard_requested_once', offboardRequestCount == 1, ...
    'arm_requested_once', armRequestCount == 1, ...
    'native_land_requested_once', landRequestCount == 1, ...
    'hover_rows_present', numel(hoverRows) >= 100, ...
    'virtual_actuator_nonzero', actuatorNonzeroRows > 0, ...
    'plant_or_estimate_motion_observed', isfinite(maxDisplacementM) && maxDisplacementM > 0.2, ...
    'final_disarmed', ~boardArmed(), ...
    'final_landed', boardLanded(), ...
    'force_disarm_zero', forceDisarmCount == 0, ...
    'mapping_rollback_six_zero', all(arrayfun(@(x) ...
        isfield(x, 'decoded') && x.decoded == 0, finalParameters(1:6))), ...
    'physical_pwm_eight_zero', all(arrayfun(@(x) ...
        isfield(x, 'decoded') && x.decoded == 0, finalParameters(7:14))));
checkValues = struct2cell(checks);
passed = ~failure.present && all(cellfun(@(x)islogical(x) && isscalar(x) && x, checkValues));

result = struct( ...
    'schema', 'GPENMPC_OFFICIAL_GENERIC_HEX_HIL_RESULT_V1', ...
    'status', ternary(passed, ...
        'PASS_OFFICIAL_GENERIC_HEX_COPTERSIM_PIXHAWK_HIL', ...
        'FAIL_OFFICIAL_GENERIC_HEX_COPTERSIM_PIXHAWK_HIL'), ...
    'claim_limit', 'OFFICIAL_GENERIC_HEX_HIL_CHAIN', ...
    'topology', struct('plant', 'CopterSim_HexarotorModelCTRL', ...
        'board', 'Pixhawk6C_PX4', 'display', 'RflySim3D', ...
        'high_level_and_logging', 'MATLAB', 'python_hil_driver', false), ...
    'failure', failure, ...
    'checks', checks, ...
    'counts', struct('telemetry_rows', numel(rows), ...
        'hover_rows', numel(hoverRows), ...
        'actuator_nonzero_rows', actuatorNonzeroRows, ...
        'arm_requests', armRequestCount, ...
        'offboard_requests', offboardRequestCount, ...
        'land_requests', landRequestCount, ...
        'standard_disarm_requests', standardDisarmCount, ...
        'force_disarm_requests', forceDisarmCount, ...
        'mapping_apply_writes', numel(parameterReceipts)), ...
    'metrics', struct('max_observed_displacement_m', maxDisplacementM), ...
    'parameter_receipts', parameterReceipts, ...
    'command_receipts', commandReceipts, ...
    'status_text', statusTextRows, ...
    'final_parameters', finalParameters, ...
    'cleanup_errors', cleanupErrors, ...
    'topics_at_start', tableToStruct(topicsAtStart), ...
    'topics_at_end', tableToStruct(topicsAtEnd), ...
    'physical_output_actions', 0, ...
    'bootloader_actions', 0);
writeJson(fullfile(outputDir, 'RESULT.json'), result);
disp(jsonencode(result, PrettyPrint=true));

    function sendHeartbeat()
        message = createmsg(dialect, 'HEARTBEAT');
        message.Payload.custom_mode = uint32(0);
        message.Payload.type = uint8(6);
        message.Payload.autopilot = uint8(8);
        message.Payload.base_mode = uint8(0);
        message.Payload.system_status = uint8(4);
        message.Payload.mavlink_version = uint8(3);
        sendudpmsg(link, message, remoteHost, remotePort);
    end

    function sendHeartbeatIfDue()
        persistent previous
        nowS = toc(runClock);
        if isempty(previous) || nowS - previous >= 0.9
            sendHeartbeat();
            previous = nowS;
        end
    end

    function sendSetpoint(position)
        message = createmsg(dialect, 'SET_POSITION_TARGET_LOCAL_NED');
        message.Payload.time_boot_ms = uint32(mod(round(toc(runClock) * 1000), 2^32));
        message.Payload.x = single(position(1));
        message.Payload.y = single(position(2));
        message.Payload.z = single(position(3));
        message.Payload.vx = single(0);
        message.Payload.vy = single(0);
        message.Payload.vz = single(0);
        message.Payload.afx = single(0);
        message.Payload.afy = single(0);
        message.Payload.afz = single(0);
        message.Payload.yaw = single(0);
        message.Payload.yaw_rate = single(0);
        message.Payload.type_mask = uint16(2048);
        message.Payload.target_system = uint8(1);
        message.Payload.target_component = uint8(1);
        message.Payload.coordinate_frame = uint8(1);
        sendudpmsg(link, message, remoteHost, remotePort);
    end

    function receipt = sendCommand(command, parameters, label)
        ackSub = mavlinksub(link, client, 'COMMAND_ACK', 'BufferSize', 50);
        message = createmsg(dialect, 'COMMAND_LONG');
        for parameterIndex = 1:7
            message.Payload.(sprintf('param%d', parameterIndex)) = ...
                single(parameters(parameterIndex));
        end
        message.Payload.command = uint16(command);
        message.Payload.target_system = uint8(1);
        message.Payload.target_component = uint8(1);
        message.Payload.confirmation = uint8(0);
        sentS = toc(runClock);
        sendudpmsg(link, message, remoteHost, remotePort);
        ack = [];
        waitClock = tic;
        while toc(waitClock) < 4.0
            messages = latestmsgs(ackSub, 50);
            for messageIndex = numel(messages):-1:1
                if double(messages(messageIndex).Payload.command) == command
                    ack = messages(messageIndex).Payload;
                    break
                end
            end
            if ~isempty(ack), break; end
            pause(0.02);
        end
        receipt = struct('label', label, 'command', command, ...
            'sent_elapsed_s', sentS, 'ack_received', ~isempty(ack), ...
            'ack_result', NaN, 'accepted', false);
        if ~isempty(ack)
            receipt.ack_result = double(ack.result);
            receipt.accepted = double(ack.result) == 0;
        end
    end

    function row = readParameter(name)
        name = string(name);
        sub = mavlinksub(link, client, 'PARAM_VALUE', 'BufferSize', 50);
        request = createmsg(dialect, 'PARAM_REQUEST_READ');
        request.Payload.param_index = int16(-1);
        request.Payload.target_system = uint8(1);
        request.Payload.target_component = uint8(1);
        request.Payload.param_id = mavText(name, 16);
        sendudpmsg(link, request, remoteHost, remotePort);
        payload = waitParameter(sub, name, 3.0);
        row = decodeParameter(payload);
    end

    function receipt = setIntegerParameter(name, value)
        name = string(name);
        before = readParameter(name);
        assert(before.mav_type == 6, 'gpenmpc:ParameterType', ...
            '%s is not MAV_PARAM_TYPE_INT32.', name);
        ackSub = mavlinksub(link, client, 'PARAM_VALUE', 'BufferSize', 50);
        message = createmsg(dialect, 'PARAM_SET');
        message.Payload.param_value = typecast(int32(value), 'single');
        message.Payload.target_system = uint8(1);
        message.Payload.target_component = uint8(1);
        message.Payload.param_id = mavText(name, 16);
        message.Payload.param_type = uint8(6);
        sendudpmsg(link, message, remoteHost, remotePort);
        payload = waitParameter(ackSub, name, 3.0);
        after = decodeParameter(payload);
        assert(after.decoded == value && after.mav_type == 6, ...
            'gpenmpc:ParameterSetVerify', '%s write did not verify.', name);
        receipt = struct('name', name, 'before', before.decoded, ...
            'after', after.decoded, 'mav_type', after.mav_type, ...
            'write_count', 1, 'ack_verified', true);
    end

    function payload = waitParameter(sub, expectedName, timeoutS)
        payload = [];
        waitClock = tic;
        while toc(waitClock) < timeoutS
            messages = latestmsgs(sub, 50);
            for messageIndex = numel(messages):-1:1
                candidate = messages(messageIndex).Payload;
                if cleanMavText(candidate.param_id) == expectedName
                    payload = candidate;
                    return
                end
            end
            pause(0.02);
        end
        error('gpenmpc:ParameterTimeout', 'No PARAM_VALUE for %s.', expectedName);
    end

    function sampleTelemetry(phase)
        local = latestPayload(subs.local);
        attitude = latestPayload(subs.attitude);
        heartbeat = latestPayload(subs.heartbeat);
        extended = latestPayload(subs.extended);
        actuator = latestPayload(subs.actuator);
        row = emptyTelemetryRow();
        row.elapsed_s = toc(runClock);
        row.phase = char(phase);
        if ~isempty(local)
            row.local_time_boot_ms = double(local.time_boot_ms);
            row.x = double(local.x); row.y = double(local.y); row.z = double(local.z);
            row.vx = double(local.vx); row.vy = double(local.vy); row.vz = double(local.vz);
        end
        if ~isempty(attitude)
            row.attitude_time_boot_ms = double(attitude.time_boot_ms);
            row.roll = double(attitude.roll); row.pitch = double(attitude.pitch);
            row.yaw = double(attitude.yaw);
        end
        if ~isempty(heartbeat)
            row.armed = double(bitand(uint8(heartbeat.base_mode), uint8(128)) ~= 0);
            row.custom_mode = double(heartbeat.custom_mode);
        end
        if ~isempty(extended)
            row.landed_state = double(extended.landed_state);
        end
        if ~isempty(actuator)
            controls = double(actuator.controls(:));
            count = min(6, numel(controls));
            row.actuator_max_abs = max(abs(controls(1:count)), [], 'omitnan');
            for controlIndex = 1:count
                row.(sprintf('actuator_%d', controlIndex)) = controls(controlIndex);
            end
        end
        rows(end+1,1) = row; %#ok<AGROW>
    end

    function value = boardArmed()
        heartbeat = latestPayload(subs.heartbeat);
        value = ~isempty(heartbeat) && ...
            bitand(uint8(heartbeat.base_mode), uint8(128)) ~= 0;
    end

    function value = boardLanded()
        extended = latestPayload(subs.extended);
        value = ~isempty(extended) && double(extended.landed_state) == 1;
    end

    function waitForDisarmed(timeoutS)
        waitClock = tic;
        while toc(waitClock) < timeoutS && boardArmed()
            sampleTelemetry('DISARM_CONFIRM');
            pause(0.05);
        end
    end

    function finalize()
        if finalized, return; end
        finalized = true;
        try
            if boardArmed()
                if boardLanded()
                    standardDisarmCount = standardDisarmCount + 1;
                    commandReceipts(end+1,1) = sendCommand(400, ...
                        [0, 0, 0, 0, 0, 0, 0], 'FINALLY_STANDARD_DISARM'); %#ok<AGROW>
                    waitForDisarmed(5.0);
                end
                if boardArmed()
                    forceDisarmCount = forceDisarmCount + 1;
                    commandReceipts(end+1,1) = sendCommand(400, ...
                        [0, 21196, 0, 0, 0, 0, 0], 'FINALLY_FORCE_DISARM'); %#ok<AGROW>
                    waitForDisarmed(5.0);
                end
            end
        catch error
            cleanupErrors(end+1,1) = "DISARM:" + error.message; %#ok<AGROW>
        end
        for rollbackIndex = numel(mappedNames):-1:1
            try
                receipt = setIntegerParameter(mappedNames(rollbackIndex), 0);
                parameterReceipts(end+1,1) = receipt; %#ok<AGROW>
            catch error
                cleanupErrors(end+1,1) = "ROLLBACK:" + ...
                    mappedNames(rollbackIndex) + ":" + error.message; %#ok<AGROW>
            end
        end
    end
end

function payload = latestPayload(subscriber)
messages = latestmsgs(subscriber, 1);
if isempty(messages)
    payload = [];
else
    payload = messages(end).Payload;
end
end

function text = mavText(value, width)
encoded = char(value);
assert(numel(encoded) <= width, 'gpenmpc:MavlinkTextWidth', ...
    'MAVLink text exceeds its fixed field width.');
text = repmat(char(0), 1, width);
text(1:numel(encoded)) = encoded;
end

function value = cleanMavText(value)
value = string(strtrim(strrep(char(value), char(0), '')));
end

function row = decodeParameter(payload)
raw = single(payload.param_value);
mavType = double(payload.param_type);
if mavType == 6
    decoded = double(typecast(raw, 'int32'));
else
    decoded = double(raw);
end
row = struct('name', cleanMavText(payload.param_id), ...
    'mav_type', mavType, 'raw_float', double(raw), ...
    'raw_bits_hex', upper(dec2hex(typecast(raw, 'uint32'), 8)), ...
    'decoded', decoded, 'error', "");
end

function row = emptyParameterRow()
row = struct('name', "", 'mav_type', NaN, 'raw_float', NaN, ...
    'raw_bits_hex', "", 'decoded', NaN, 'error', "");
end

function requireDecoded(rows, name, expected)
index = find(strcmp(string({rows.name}), string(name)), 1);
assert(~isempty(index) && rows(index).decoded == expected, ...
    'gpenmpc:HilProfileMismatch', '%s did not equal %g.', name, expected);
end

function requireTopic(topics, name)
assert(any(string(topics.MessageName) == string(name)), ...
    'gpenmpc:OfficialHilTopicMissing', 'Required topic %s is missing.', name);
end

function row = emptyTelemetryRow()
row = struct('elapsed_s', NaN, 'phase', '', ...
    'local_time_boot_ms', NaN, 'attitude_time_boot_ms', NaN, ...
    'x', NaN, 'y', NaN, 'z', NaN, 'vx', NaN, 'vy', NaN, 'vz', NaN, ...
    'roll', NaN, 'pitch', NaN, 'yaw', NaN, ...
    'armed', NaN, 'custom_mode', NaN, 'landed_state', NaN, ...
    'actuator_max_abs', NaN, 'actuator_1', NaN, 'actuator_2', NaN, ...
    'actuator_3', NaN, 'actuator_4', NaN, 'actuator_5', NaN, ...
    'actuator_6', NaN);
end

function writeJson(path, value)
temporary = string(path) + ".tmp";
fid = fopen(temporary, 'w', 'n', 'UTF-8');
assert(fid >= 0, 'gpenmpc:JsonOpen', 'Cannot open JSON output.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(value, PrettyPrint=true));
clear cleanup
movefile(temporary, path, 'f');
end

function output = tableToStruct(value)
if istable(value)
    output = table2struct(value);
else
    output = value;
end
end

function value = ternary(condition, whenTrue, whenFalse)
if condition
    value = whenTrue;
else
    value = whenFalse;
end
end
