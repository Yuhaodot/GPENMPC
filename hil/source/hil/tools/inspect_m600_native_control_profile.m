function receipt=inspect_m600_native_control_profile(outputPath,live)
% Read-only allocator and attitude-chain diagnostics.
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'tools'));
expected=struct('uid',gpenmpc_device_identity('uid'),'board_version',56, ...
    'commit','6ea3539157ca358c70a515878b77077af7d4611d');
names=["CA_AIRFRAME","CA_METHOD","CA_R_REV","THR_MDL_FAC", ...
    "MPC_THR_HOVER","MPC_USE_HTE","MC_ROLL_P","MC_PITCH_P","MC_YAW_P", ...
    "MC_AIRMODE","FD_FAIL_R","FD_FAIL_P","FD_FAIL_R_TTRI","FD_FAIL_P_TTRI", ...
    "COM_LKDOWN_TKO","COM_SPOOLUP_TIME"];
for i=0:5
    for field=["PX","PY","PZ","AX","AY","AZ","CT","KM"]
        names(end+1)=sprintf('CA_ROTOR%d_%s',i,field); %#ok<AGROW>
    end
end
for axis=["ROLLRATE","PITCHRATE","YAWRATE"]
    for field=["P","I","D","FF","K"]
        names(end+1)="MC_"+axis+"_"+field; %#ok<AGROW>
    end
end
assert(numel(names)==79&&numel(unique(names))==79);
if ~live
    % Every rejected selector must fail BEFORE the serial class is constructed.
    tests={"CA_ROTOR0_PX"+newline+"reboot",repmat("X",1,17),["MC_ROLL_P","MC_ROLL_P"],"",missing};
    checks=false(1,numel(tests));
    for k=1:numel(tests)
        e=expected;e.diagnostic_parameter_names=tests{k};
        try,m600_canonical_serial_preflight(outputPath,e);
        catch problem,checks(k)=strcmp(problem.identifier,'m600check:DiagnosticParameterNames');end
    end
    assert(all(checks),'m600check:ReadOnlySelectorTest','Invalid selector was not rejected before COM.');
    receipt=struct('passed',true,'tests',numel(checks),'COM_open',0, ...
        'parameter_writes',0,'requested_names',names);
    return
end
expected.diagnostic_parameter_names=names;
receipt=m600_canonical_serial_preflight(outputPath,expected);
end
