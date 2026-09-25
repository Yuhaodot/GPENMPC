function result=observe_serial_recovery(afterOwnedBridgeRelease,afterUserReplug,root)
% Observe recovery heartbeat and identity.
build=string(fileparts(fileparts(mfilename('fullpath'))));
if nargin<3,root=fullfile(gpenmpc_external_path('serial_recovery'));end
root=string(root);
assert(gpenmpc_is_session_directory(root,build)&&isfolder(root));
if nargin<1,afterOwnedBridgeRelease=false;end
if nargin<2,afterUserReplug=false;end
assert(islogical(afterOwnedBridgeRelease)&&isscalar(afterOwnedBridgeRelease));
assert(islogical(afterUserReplug)&&isscalar(afterUserReplug));
if afterUserReplug
    assert(afterOwnedBridgeRelease,'gpenmpcRecovery:Sequence','User-replug observation follows owned bridge release.');
    assert(System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0, ...
        'gpenmpcRecovery:BridgeStillPresent','The failed owned bridge must be released before the changed recovery observation.');
    output=fullfile(root,'DIRECT_SERIAL_AFTER_USER_REPLUG.mat');
elseif afterOwnedBridgeRelease
    assert(System.Diagnostics.Process.GetProcessesByName('CopterSimNoUI').Length==0, ...
        'gpenmpcRecovery:BridgeStillPresent','The failed owned bridge must be released before the changed recovery observation.');
    output=fullfile(root,'DIRECT_SERIAL_AFTER_OWNED_BRIDGE_RELEASE.mat');
else
    output=fullfile(root,'DIRECT_SERIAL_RECOVERY_OBSERVATION.mat');
end
assert(~isfile(output),'gpenmpcRecovery:AlreadyObserved','Do not repeat the same diagnostic.');
addpath(fullfile(build,'host_runtime'));
prior=jsondecode(fileread(fullfile(root,'OUTER_SHORT_RESULT.json')));
assert((isfield(prior,'external_action_required')&&prior.external_action_required) ...
    ||(~prior.safe&&prior.short.safe_ground&&prior.short.counts.arm_requests==0&&prior.short.counts.offboard_requests==0));
% After owner release, use fresh heartbeat and state to determine recovery
% eligibility. Keep after_user_replug consistent with the observed history.
assert(afterUserReplug||afterOwnedBridgeRelease ...
    ||(prior.short.counts.arm_requests==0&&prior.short.counts.offboard_requests==0));
result=struct('read_only',true,'safe_observation',false,'COM_opened',false, ...
    'COM_closed',true,'parameter_writes',0,'arm_mode_requests',0,'reboot_flash',0, ...
    'owned_NoUI_stopped',false,'failure','','utc',char(datetime('now','TimeZone','UTC')), ...
    'after_user_replug',afterUserReplug,'prior_arm_requests',prior.short.counts.arm_requests);
link=gpenmpcNative.MavlinkSerialLink;
guard=onCleanup(@()link.close());
try
    fprintf('RECOVERY_READONLY available-port query, after_owned_bridge_release=%d after_user_replug=%d\n',afterOwnedBridgeRelease,afterUserReplug);
    result.available_ports=serialportlist('available');
    assert(any(strcmpi(string(result.available_ports),'COM3')), ...
        'gpenmpcRecovery:ComNotExclusive','COM3 is not available for exclusive acquisition. No second owner opened.');
    fprintf('RECOVERY_READONLY exclusive COM3 passive heartbeat\n');
    hb=link.open('COM3',921600,10);result.COM_opened=true;
    result.first_heartbeat=hb;
    assert(link.TargetSystem==1&&link.TargetComponent==1,'gpenmpcRecovery:Target','Unexpected system/component.');
    result.identity=link.collectIdentity(8);v=result.identity.autopilot_version;
    assert(sprintf('%u',uint64(v.uid))==string(gpenmpc_device_identity('uid'))&&double(v.board_version)==56 ...
        &&strcmpi(result.identity.parsed_commit,'6ea3539157ca358c70a515878b77077af7d4611d'), ...
        'gpenmpcRecovery:Identity','Unexpected board identity.');
    link.drain();link.requestMessage(245);result.extended_state=link.waitForMessage('EXTENDED_SYS_STATE',3,[]);
    link.drain();result.last_heartbeat=link.waitForMessage('HEARTBEAT',3,[]);
    result.disarmed=bitand(uint8(result.last_heartbeat.Payload.base_mode),uint8(128))==0;
    result.landed=double(result.extended_state.Payload.landed_state)==1;
    result.safe_observation=result.disarmed&&result.landed;
catch ex
    result.failure=getReport(ex,'extended','hyperlinks','off');
end
link.close();result.COM_closed=~link.isOpen();result.link_counters=link.counters();clear guard
save(output,'result');
disp(result);
end
