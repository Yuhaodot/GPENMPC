function report=test_canonical_observed_outer_runtime(outputRoot)
% Test the outer-service factory with synthetic timestamped observations.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
prior=path;pathGuard=onCleanup(@()path(prior)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
assert(~isfolder(outputRoot),'Keep prior evidence; new test output required.');mkdir(outputRoot);
checks=struct('name',{},'pass',{});
e=struct('uid','1234605616436508552','system_id',1,'component_id',1, ...
    'boot_generation',7,'maximum_age_ns',1e8,'maximum_runtime_age_ns',1e8, ...
    'canonical_package_root',gpenmpcNative.canonicalAssetRoot(),'initial_payload_kg',2.27, ...
    'async_outer_required',false,'task_identity_sha256',repmat('C',1,64));
c=zeros(3,8);c(:,1)=[0;0;2];c(:,2)=[.1;0;0];
trajectory=struct('total_duration_s',10,'coefficients_ascending',c);
s=gpenmpcNative.BoardOuterService(build,e,trajectory);
cleanup=onCleanup(@()s.close()); %#ok<NASGU>
t=2e9;
v=struct('source','HOST_M600_VIRTUAL_ACTUATOR_INTERFACE', ...
    'rx_ns',t-3e7,'generation',73,'valid',true,'ordering','SOFTWARE_M600_ORDER', ...
    'rotor_thrust_state_n',[1;2;3;4;5;6],'thrust_effectiveness',ones(6,1), ...
    'state_source','SAME_M600_ACCEPTED_STEP_ROTOR_LAG_STATE','plant_session_id',uint64(91));
w=struct('source','FROZEN_TASK_WIND_ESTIMATOR','rx_ns',t-6e7, ...
    'generation',9,'valid',true,'estimate_xy_mps',[.4;-.2]);
r=s.observedOuterRuntime(100,t,v,w);
check('actual_factory_keeps_distinct_rx_and_generations', ...
    r.virtual_actuator.rx_ns==v.rx_ns&&r.virtual_actuator.generation==73 ...
    &&r.wind.rx_ns==w.rx_ns&&r.wind.generation==9&&r.outer.rx_ns==t ...
    &&r.outer.generation==100);
check('initial_command_exactly_same_plant_rotor_state', ...
    isequal(r.virtual_actuator.rotor_command_n,v.rotor_thrust_state_n));
check('initial_nominal_force_not_sum_of_observed_rotors', ...
    r.outer.desired_force_generation==0 ...
    &&r.outer.desired_force_source=="CANONICAL_LEG_INITIAL_FORCE" ...
    &&r.outer.previous_desired_force_projected_n(3)~=sum(v.rotor_thrust_state_n));
check('full_original_observer_payload_preserved', ...
    r.virtual_actuator.plant_session_id==uint64(91)&&isequal(r.wind,w) ...
    &&r.observed_input_stamps_preserved&&~r.observation_binding.source_age_rewritten);
base=r;base.virtual_actuator.command_generation=71;
base.virtual_actuator.command_source='PREVIOUS_COMMITTED_HIL_ACTUATOR_COMMAND';
base.virtual_actuator.rotor_command_n=[6;5;4;3;2;1];
[bound,ok,~]=gpenmpcNative.bindCanonicalRuntimeObservations(base,v,w,t,1e8);
check('lag_state_not_substituted_for_prior_committed_command', ...
    ok&&isequal(bound.virtual_actuator.rotor_command_n,[6;5;4;3;2;1]) ...
    &&isequal(bound.virtual_actuator.rotor_thrust_state_n,[1;2;3;4;5;6]));
bad=v;bad.rx_ns=t-1e8-1;negative('old_rotor_cannot_be_renewed',bad,w,'INVALID_OR_STALE_ROTOR_OBSERVATION');
bad=w;bad.rx_ns=t-1e8-1;negative('old_wind_cannot_be_renewed',v,bad,'INVALID_OR_STALE_WIND_OBSERVATION');
bad=v;bad.rx_ns=t+1;negative('future_rotor_rejected',bad,w,'INVALID_OR_STALE_ROTOR_OBSERVATION');
bad=w;bad.generation=0;negative('zero_wind_generation_rejected',v,bad,'INVALID_OR_STALE_WIND_OBSERVATION');
bad=v;bad.source='ANIMATION_RPM';negative('animation_source_rejected',bad,w,'INVALID_OR_STALE_ROTOR_OBSERVATION');
bad=v;bad.state_source='SQUARED_RPM_ESTIMATE';negative('animation_derived_state_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=v;bad.rotor_thrust_state_n(2)=NaN;negative('nonfinite_rotor_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=v;bad.rotor_thrust_state_n(2)=-1;negative('negative_rotor_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=v;bad.thrust_effectiveness(2)=0;negative('invalid_effectiveness_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=w;bad.estimate_xy_mps=[NaN;0];negative('invalid_wind_rejected',v,bad,'INVALID_WIND_VALUE');
bad=v;bad.ordering='PX4_MOTOR_ORDER';negative('wrong_rotor_order_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=v;bad.plant_session_id=0;negative('unbound_plant_session_rejected',bad,w,'INVALID_ROTOR_STATE_SEMANTICS');
bad=v;bad.valid=false;negative('invalid_observation_rejected',bad,w,'INVALID_OR_STALE_ROTOR_OBSERVATION');
bad=v;bad.rx_ns=t-1e8;[~,ok]=gpenmpcNative.bindCanonicalRuntimeObservations(base,bad,w,t,1e8);
check('existing_age_boundary_unchanged',ok);
bad=w;bad.rx_ns=t-1e8-1;caught=false;
try,s.observedOuterRuntime(101,t,v,bad);catch ex,caught=strcmp(ex.identifier,'gpenmpcNative:ObservedOuterRuntime');end
check('production_stale_input_latches_publication_closed',caught&&s.status().failed&&~s.status().publication_allowed);
caught=false;try,s.observedOuterRuntime(102,t,v,w);catch ex,caught=strcmp(ex.identifier,'gpenmpcNative:BoardOuterFailed');end
check('fresh_input_does_not_clear_latched_fault',caught);
report=struct('status','PASS_HOST_ONLY_OBSERVED_SOURCE_BINDING', ...
    'checks',checks,'test_count',numel(checks),'pass_count',sum([checks.pass]), ...
    'hardware_actions',0,'com_open',0,'socket_count',0,'solver_calls',0, ...
    'plant_instances_started',0,'live_admitted',false);
save(fullfile(outputRoot,'RAW.mat'),'r','bound','report');
fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
fileGuard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear fileGuard
clear cleanup;delete(s);clear s;clear pathGuard
disp(jsonencode(report));
    function negative(name,a,b,reason)
        [out,yes,why]=gpenmpcNative.bindCanonicalRuntimeObservations(base,a,b,t,1e8);
        check(name,~yes&&isempty(fieldnames(out))&&why==string(reason));
    end
    function check(name,yes)
        checks(end+1)=struct('name',name,'pass',logical(yes)); %#ok<AGROW>
        assert(yes,'gpenmpcNative:ObservedInputTest','%s',name);
    end
end
