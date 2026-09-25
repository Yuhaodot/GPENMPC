function result=run_native_position_current_config_tests(outputDir)
% Test parameter-factory alignment.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpc:ExistingFactoryTestOutput');
[cfg,m,meta]=native_position_control_current_config();
r=jsondecode(fileread(meta.current138_raw.path));rows=r.diagnostic_parameter_observations;
tests=struct('name',{},'passed',{});
add('138_exact_typed_receipt',meta.current_typed_row_count==138&&meta.original105_recomputed_matches==105&&meta.new_typed_count==33);
add('current138_exact_SHA',strcmp(meta.current138.sha256,'BF31EEF276ADC9756371E5FEF282AC10465056B5710E3A87D64CD422F5F1E0B4'));
add('current105_exact_SHA',strcmp(meta.current105.sha256,'EC504AEB12444D41B88DD0E5411DDAA3B59B513AB9410433736A5B59C47E1D9C'));
add('position_gains_exact_raw',isequal(cfg.position_p,[v('MPC_XY_P');v('MPC_XY_P');v('MPC_Z_P')]));
add('velocity_P_exact_raw',isequal(cfg.velocity_p,[v('MPC_XY_VEL_P_ACC');v('MPC_XY_VEL_P_ACC');v('MPC_Z_VEL_P_ACC')]));
add('nominal_velocity_I_exact_raw',isequal(cfg.velocity_i,[v('MPC_XY_VEL_I_ACC');v('MPC_XY_VEL_I_ACC');v('MPC_Z_VEL_I_ACC')]));
add('velocity_D_exact_raw',isequal(cfg.velocity_d,[v('MPC_XY_VEL_D_ACC');v('MPC_XY_VEL_D_ACC');v('MPC_Z_VEL_D_ACC')]));
add('flight_velocity_limits_exact_raw',isequal(cfg.velocity_limits_mps,[v('MPC_XY_VEL_MAX');v('MPC_Z_VEL_MAX_UP');v('MPC_Z_VEL_MAX_DN')]));
add('flight_thrust_limits_exact_raw',isequal(cfg.thrust_limits,[v('MPC_THR_MIN');v('MPC_THR_MAX')]));
add('margin_exact_raw',cfg.horizontal_thrust_margin==v('MPC_THR_XY_MARG'));
add('tilt_float_radian_conversion',cfg.tilt_limit_rad==double(single(v('MPC_TILTMAX_AIR'))*(single(pi)/single(180))));
add('hover_thrust_exact_raw',cfg.hover_thrust==v('MPC_THR_HOVER'));
add('decouple_typed_logical',islogical(cfg.decouple_horizontal_vertical)&&cfg.decouple_horizontal_vertical==logical(v('MPC_ACC_DECOUPLE')));
fields=fieldnames(m);aligned=true;
for k=1:numel(fields),aligned=aligned&&isequal(m.(fields{k}),v(fields{k}));end
add('36_module_parameters_exact_raw',numel(fields)==36&&aligned);
add('36_distinct_provenance_rows',numel(meta.parameter_rows)==36&&numel(unique({meta.parameter_rows.name}))==36);
add('velocity_LP_and_notch_observed_bypass',m.MPC_VEL_LP==0&&m.MPC_VEL_NF_FRQ==0&&m.MPC_VEL_NF_BW==5);
add('derivative_LP_observed_not_bypass',m.MPC_VELD_LP==5);
add('auto_configuration_observed_disabled',m.SYS_VEHICLE_RESP<0&&m.MPC_XY_VEL_ALL<0&&m.MPC_Z_VEL_ALL<0);
add('takeoff_observed_current_values',m.MPC_TKO_RAMP_T==3&&m.COM_SPOOLUP_TIME==1&&m.COM_THROW_EN==0);
add('nominal_I_not_runtime_I',cfg.velocity_i(3)==2&&meta.module_phase_contract.no_route_motion_hover_target_I==0 ...
    &&meta.module_phase_contract.requires_effective_vertical_I_each_step&&~meta.module_phase_contract.accumulated_I_observed);
add('phase_slew_source_only',meta.module_phase_contract.explicit_new_instance_effective_vertical_I==0 ...
    &&meta.module_phase_contract.effective_vertical_I_slew_per_s==.30);
add('no_synthetic_HTE_estimate',m.MPC_USE_HTE==1&&~isfield(m,'valid_hover_thrust_estimate'));
add('no_defaults_or_runtime_snapshot_claim',~meta.defaults_used&&meta.nominal_cfg_is_not_a_module_runtime_snapshot);
add('hardware_COM_MEX_actions_zero',meta.hardware_actions==0&&meta.COM_UDP_actions==0&&meta.MEX_calls==0);
add('runtime_unknowns_preserved',numel(meta.runtime_unobserved)==4);
add('provenance_is_module_phase_explicit',strcmp(cfg.parameter_provenance,'CURRENT_TYPED_WITH_EXPLICIT_MODULE_PHASE'));
[cfg2,m2,meta2]=native_position_control_current_config();
add('pure_factory_repeat_exact',isequaln(cfg,cfg2)&&isequaln(m,m2)&&isequaln(meta,meta2));
result=struct('schema','HOST_NATIVE_POSITION_CURRENT_FACTORY_TESTS_V1','passed',all([tests.passed]), ...
    'case_count',numel(tests),'cases_passed',sum([tests.passed]),'tests',tests, ...
    'configuration',cfg,'module_parameters',m,'parameter_evidence',meta, ...
    'negative_coverage_basis','The read-only factory is covered by 34 HOST controls for missing data, types, bits, drift, outputs and ownership.', ...
    'hardware_actions',0,'COM_UDP_actions',0,'MEX_calls',0);
mkdir(outputDir);f=fopen(fullfile(outputDir,'HOST_RESULT.json'),'w','n','UTF-8');assert(f>=0);guard=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(result,PrettyPrint=true));
assert(result.passed,'gpenmpc:CurrentPositionFactoryTests','Factory checks failed; see the complete result.');
    function add(name,pass),tests(end+1)=struct('name',name,'passed',logical(pass));end %#ok<AGROW>
    function value=v(name)
        j=find(strcmp({rows.name},name));assert(isscalar(j));value=rows(j).typed_value.decoded;
    end
end
