% Replay the retained forwarding batch offline.
root=fileparts(fileparts(mfilename('fullpath')));
mexPath=fullfile(gpenmpc_external_path('rfly_udp_transport_build'),'gpenmpc_rfly_udp_transport_mex.mexw64');
addpath(fileparts(mexPath));
addpath(fullfile(root,'tools'));
source=struct('kind','GPENMPC_RFLY_UDP_TRANSPORT_MEX', ...
    'exact_path',mexPath, ...
    'sha256','1F94BFEB586FC58816C487CCD541686760F03CA7632DFD4DA15F87480276ECC5');
outputRoot=fullfile(root,'evidence','forwarding_replay');
assert(~isfolder(outputRoot)&&~isfile(outputRoot), ...
    'gpenmpcNative:ReplayOutputExists','Replay evidence already exists.');
report=test_rfly_sole_mavlink_forwarding(outputRoot,source,[62383 62384],'FULL');
disp(jsonencode(report));
assert(report.pass,'gpenmpcNative:ReplayFailed','Forwarding replay failed.');
