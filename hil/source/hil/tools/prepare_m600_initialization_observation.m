function p=prepare_m600_initialization_observation(outputPath)
% Prepare a new disarmed initialization observation.
root=fileparts(fileparts(mfilename('fullpath')));mc=fullfile(root,'m600_coptersim');
options=struct('execution_kind','INITIALIZATION_OBSERVATION_ONLY', ...
    'copter_init_observer_tests',fullfile(gpenmpc_external_path('copter_init_observer_host'),'RESULT.json'), ...
    'initialization_observer_tests',fullfile(gpenmpc_external_path('initialization_observer_host'),'RESULT.json'));
p=prepare_m600_native_plan(outputPath,options);
end
