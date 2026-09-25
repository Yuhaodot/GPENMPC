function check_manual_startup()
% Bounded startup verification; normal manual sessions remain user-ended.
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'tools'));
fprintf('Checking the connected transmitter before startup.\n');
start_gpenmpc_usb_manual('CHECK_INPUT');
info=start_gpenmpc_usb_manual('START',20);
if ~isfield(info,'result') || ~isfield(info.result,'safe') || ~info.result.safe
    error('gpenmpcCheck:RecoveryIncomplete','Startup check did not finish in standby.');
end
fprintf('STARTUP_CHECK_ENDED: %s\n',info.run_root);
if isfield(info.result,'short') && isfield(info.result.short,'failure') && ~isempty(info.result.short.failure)
    fprintf('STARTUP_CHECK_FAILURE: %s\n',info.result.short.failure.identifier);
end
end
