function test_hil()
% Run offline session and display checks.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'source','hil','tools'));
test_rc_demo_lifecycle("OFFLINE");
test_rc_demo_runtime;
test_gpenmpc_demo_display("OFFLINE");
end
