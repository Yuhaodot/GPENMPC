function reset_m600_standby_view(runRoot)
% One standby pose only. CopterSim's 3DOutput is the sole live publisher.
for name={'CopterSim','CopterSimNoUI'}
    assert(System.Diagnostics.Process.GetProcessesByName(name{1}).Length==0, ...
        'gpenmpcReset:SimulatorStillRunning','Simulation is still running.');
end
p=fullfile(runRoot,'SHORT_ENTRY_PREPARED.json');
if ~isfile(p),return;end
prepared=jsondecode(fileread(p));
position=double(prepared.initial_world_ground_ned_m(:).');
assert(numel(position)==3&&all(isfinite(position)),'gpenmpcReset:InitialPosition');
% Official RflySimSDK UE4CtrlAPI.sendUE4PosNew: 6i14f4d, 112 bytes.
packet=[typecast(int32([1234567891 1 5 0 0 0]),'uint8'), ...
    typecast(zeros(1,14,'single'),'uint8'),typecast([position -1],'uint8')];
assert(numel(packet)==112);
socket=udpport('datagram','IPV4');
write(socket,packet,'uint8','127.0.0.1',20010);
clear socket
end
