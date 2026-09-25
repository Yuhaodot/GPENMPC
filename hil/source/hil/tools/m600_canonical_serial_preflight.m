function receipt=m600_canonical_serial_preflight(outputPath,expected)
% M600_CANONICAL_SERIAL_PREFLIGHT Read identity and safety state before and after the bridge.
% Use 15 s open, 10 s identity and 3 s typed-read/shell bounds.
assert(~isfile(outputPath),'m600check:PreflightOutputExists','Existing preflight evidence must not be overwritten.');
diagnosticNames=strings(1,0);
if isfield(expected,'diagnostic_parameter_names')
    diagnosticNames=string(expected.diagnostic_parameter_names);
    assert(isvector(diagnosticNames)&&~any(ismissing(diagnosticNames))&& ...
        numel(unique(diagnosticNames))==numel(diagnosticNames)&& ...
        all(strlength(diagnosticNames)>=1 & strlength(diagnosticNames)<=16)&& ...
        all(~cellfun(@isempty,regexp(cellstr(diagnosticNames),'^[A-Z][A-Z0-9_]*$','once'))), ...
        'm600check:DiagnosticParameterNames','Unique exact read-only parameter names required before COM open.');
end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'host_runtime'),fullfile(root,'m600_coptersim','matlab_validation'));
geometryCheck=[];
if isfield(expected,'temporary_allocator_geometry')
    geometryCheck=m600check.validateTemporaryAllocatorGeometry(expected.temporary_allocator_geometry);
    assert(geometryCheck.passed,'m600check:AllocatorGeometryContract','%s',geometryCheck.failure);
end
tuningCheck=[];unionPhase='ORIGINAL';
if isfield(expected,'native_hover_tuning')
    if strcmp(expected.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
        tuningCheck=validate_m600_delivery_native_tuning_contract(expected.native_hover_tuning);
    else
        tuningCheck=m600check.validateNativeHoverTuningContract(expected.native_hover_tuning);
    end
    assert(tuningCheck.passed,'m600check:NativeHoverTuningContract','%s',tuningCheck.failure);
    assert(~isempty(geometryCheck),'m600check:NativeHoverTuningGeometryRequired', ...
        'The complete read-only union requires both tuning3 and geometry12 contracts.');
    if isfield(expected,'native_hover_tuning_phase'),unionPhase=char(expected.native_hover_tuning_phase);end
    assert(any(strcmp(unionPhase,{'ORIGINAL','RESTORED'})), ...
        'm600check:SerialOriginalUnionOnly','Serial pre/postflight requires the original parameter values.');
end
link=gpenmpcNative.MavlinkSerialLink;guard=onCleanup(@()link.close());
receipt=struct('schema','M600_CANONICAL_SERIAL_READONLY_SAFETY_V1','passed',false, ...
    'failure','','COM_open_attempts',1,'COM_closed',false,'parameter_writes',0, ...
    'mapping_writes',0,'arm_disarm_mode_requests',0,'flash_reboot_count',0, ...
    'physical_output_actions',0);
try
    link.open("COM3",921600,15);identity=link.collectIdentity(10);
    v=identity.autopilot_version;
    receipt.uid=sprintf('%u',uint64(v.uid));receipt.board_version=double(v.board_version);
    receipt.product_id=double(v.product_id);receipt.commit=char(identity.parsed_commit);
    receipt.hw_arch=char(identity.parsed_hw_arch);
    receipt.flight_custom_version_hex=upper(reshape(dec2hex(uint8(v.flight_custom_version(:)),2).',1,[]));
    receipt.raw_identity=identity;
    assert(strcmp(receipt.uid,expected.uid)&&receipt.board_version==expected.board_version&& ...
        receipt.product_id==expected.board_version&&strcmpi(receipt.commit,expected.commit)&& ...
        strcmp(receipt.hw_arch,'PX4_FMU_V6C'),'m600check:SerialIdentityMismatch','Fresh board/application identity differs.');
    guards={'SYS_HITL',1;'SYS_AUTOSTART',6001;'MAV_TYPE',13;'CA_ROTOR_COUNT',6;'RA_CTRL_MODE',0};
    names=[string(guards(:,1)).',compose("HIL_ACT_FUNC%d",1:16),compose("PWM_MAIN_FUNC%d",1:8)];
    values=[cell2mat(guards(:,2)).',zeros(1,24)];rows=cell(1,numel(names));
    for k=1:numel(names)
        link.drain();rows{k}=link.requestParam(names(k),3);
        assert(rows{k}.mav_type==6&&rows{k}.decoded==values(k), ...
            'm600check:SerialProfileMismatch','Unexpected typed parameter %s.',names(k));
    end
    receipt.parameters=vertcat(rows{:});
    if ~isempty(tuningCheck)
        % Retain all 138 attempted observations from the open COM owner,
        % including mismatches. Both phase labels require the expected bits.
        if strcmp(expected.native_hover_tuning.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1')
            receipt.native_hover_parameter_union=verify_m600_delivery_parameter_union( ...
                @readMountParameter,expected.native_hover_tuning,expected.temporary_allocator_geometry,unionPhase);
        else
            receipt.native_hover_parameter_union=m600check.verifyNativeHoverParameterUnion( ...
                @readMountParameter,expected.native_hover_tuning,expected.temporary_allocator_geometry,unionPhase);
        end
        receipt.native_hover_original138_verified=receipt.native_hover_parameter_union.passed;
        assert(receipt.native_hover_parameter_union.passed,'m600check:SerialOriginal138Mismatch', ...
            'Complete original typed138 not verified: %s',receipt.native_hover_parameter_union.failure);
    end
    if ~isempty(geometryCheck)
        % Require the same 79-field identity for serial preflight and postflight.
        % Temporary geometry applies only during the owned MATLAB session.
        identity=expected.temporary_allocator_geometry.unchanged_guard_entries;
        entries=expected.temporary_allocator_geometry.entries;
        for j=1:numel(entries)
            identity(end+1)=struct('name',entries(j).name,'mav_type',9, ...
                'raw_bits_hex',entries(j).original_raw_bits_hex); %#ok<AGROW>
        end
        identityRows=cell(1,numel(identity));
        for j=1:numel(identity)
            if isempty(tuningCheck)
                link.drain();p=link.requestParam(string(identity(j).name),3);
            else
                % Reuse this owner's union read.
                unionRows=receipt.native_hover_parameter_union.rows;
                matched=unionRows(strcmp({unionRows.name},identity(j).name));
            assert(isscalar(matched)&&matched.passed,'m600check:SerialUnionGeometryIdentity', ...
                'The final same-owner parameter union must include the exact original geometry identity.');
                p=matched.observed;
            end
            identityRows{j}=p;
            assert(p.mav_type==identity(j).mav_type&&strcmpi(p.raw_bits_hex,identity(j).raw_bits_hex), ...
                'm600check:AllocatorOriginalSerialIdentity','Original typed identity differs: %s',identity(j).name);
        end
        receipt.allocator_original_identity_parameters=vertcat(identityRows{:});
        receipt.allocator_original_identity_passed=true;
    end
    if ~isempty(diagnosticNames)
        % Read optional diagnostics through the current owner.
        diagnosticRows=cell(1,numel(diagnosticNames));
        for k=1:numel(diagnosticNames)
            value=[];readError='';
            try,link.drain();value=link.requestParam(diagnosticNames(k),3);
            catch e,readError=[e.identifier ': ' e.message];end
            diagnosticRows{k}=struct('name',char(diagnosticNames(k)),'typed_value',value, ...
                'read_error',readError,'read_utc',char(datetime('now','TimeZone','UTC', ...
                'Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'")),'write_count',0);
        end
        receipt.diagnostic_parameter_observations=vertcat(diagnosticRows{:});
        receipt.diagnostic_parameters_complete=all(cellfun(@(x)isempty(x.read_error),diagnosticRows));
    end
    % Report mounting and calibration observations without applying angle corrections.
    optionalNames=["SENS_BOARD_ROT","SENS_BOARD_X_OFF","SENS_BOARD_Y_OFF","SENS_BOARD_Z_OFF", ...
        "CAL_ACC0_ID","CAL_ACC0_XOFF","CAL_ACC0_YOFF","CAL_ACC0_ZOFF", ...
        "CAL_ACC0_XSCALE","CAL_ACC0_YSCALE","CAL_ACC0_ZSCALE"];
    optionalRows=cell(1,numel(optionalNames));
    for k=1:numel(optionalNames)
        value=[];readError='';
        try,link.drain();value=link.requestParam(optionalNames(k),3);catch e,readError=[e.identifier ': ' e.message];end
        optionalRows{k}=struct('name',char(optionalNames(k)),'typed_value',value,'read_error',readError, ...
            'read_utc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'")), ...
            'write_count',0,'role','DIAGNOSTIC_ONLY_NOT_INFERRED_CALIBRATION_CHANGE');
    end
    receipt.mount_and_accelerometer_parameter_observations=vertcat(optionalRows{:});
    if isfield(expected,'virtual_sensor_mount_contract')
        receipt.virtual_sensor_mount_check=m600check.verifyVirtualSensorMountContract(@readMountParameter,expected.virtual_sensor_mount_contract);
    end
    receipt.pwm_out_status=char(link.shellCommand("pwm_out status",3));
    receipt.custom_controller_status=char(link.shellCommand("gpenmpc_se3_control status",3));
    assert(~isempty(regexp(receipt.pwm_out_status,'\[pwm_out\]\s+not running','once')), ...
        'm600check:PhysicalPwmModuleUnknown','pwm_out not-running status was not confirmed.');
    assert(~isempty(regexp(receipt.custom_controller_status,'\[gpenmpc_se3_control\]\s+not running','once')), ...
        'm600check:NativeControllerMutualExclusionUnknown','Custom controller stopped status was not confirmed.');
    receipt.sd_diagnostics=m600check.inspectSdDiagnosticListing(link.shellCommand("ls /fs/microsd",3));
    % Observe outputs explicitly and report missing samples as NOT_OBSERVED.
    link.drain();link.requestMessage(93);link.requestMessage(375);
    outputRows=struct('name',{},'payload',{});watch=tic;
    while toc(watch)<3
        messages=link.drain();
        for k=1:numel(messages)
            name=gpenmpcNative.MavlinkSerialLink.messageName(messages{k});
            if any(name==["HIL_ACTUATOR_CONTROLS","ACTUATOR_OUTPUT_STATUS"])
                outputRows(end+1)=struct('name',char(name),'payload',messages{k}.Payload); %#ok<AGROW>
            end
        end
        pause(.002);
    end
    receipt.virtual_output_observation=m600check.evaluateVirtualOutputEvidence(outputRows);
    receipt.virtual_output_path_disabled=all(cellfun(@(p)p.decoded==0,rows(6:21)));
    receipt.physical_output_path_disabled=all(cellfun(@(p)p.decoded==0,rows(22:29)));
    assert(~receipt.virtual_output_observation.any_nonzero_or_nonfinite, ...
        'm600check:ObservedOutputNotZero','A reported output was nonzero/nonfinite or the output payload was unrecognized.');
    % Acquire fresh safety messages after the parameter loop.
    link.drain();link.requestMessage(245);
    ext=link.waitForMessage("EXTENDED_SYS_STATE",3,[]);
    link.drain();hb=link.waitForMessage("HEARTBEAT",3,[]);
    receipt.armed=bitand(uint8(hb.Payload.base_mode),uint8(128))~=0;
    receipt.landed_state=double(ext.Payload.landed_state);
    assert(~receipt.armed&&receipt.landed_state==1,'m600check:SerialDisarmedGroundNotConfirmed','Fresh disarmed/landed state was not confirmed.');
    % Retain fresh safety observations even if writing diagnostics fails.
    assert(receipt.sd_diagnostics.passed,'m600check:SdDiagnosticBoundary', ...
        'Current crash log or incomplete SD listing: preserve all files, no reboot/arm/rename.');
    if isfield(receipt,'virtual_sensor_mount_check')
        assert(receipt.virtual_sensor_mount_check.passed,'m600check:SensorFrameIdentityMismatch', ...
            'Raw-sensor adapter requires the explicitly bound mounting and calibration observation semantics.');
    end
    receipt.passed=true;
catch e,receipt.failure=[e.identifier ': ' e.message];end
link.close();receipt.COM_closed=~link.isOpen();receipt.link_counters=link.counters();clear guard
receipt.passed=receipt.passed&&receipt.COM_closed;
receipt.generated_utc=char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"));
folder=fileparts(outputPath);if ~isfolder(folder),mkdir(folder);end
save(string(outputPath)+".mat",'receipt');
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);c=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(receipt,PrettyPrint=true));clear c
    function row=readMountParameter(name)
        link.drain();row=link.requestParam(string(name),3);
    end
end
