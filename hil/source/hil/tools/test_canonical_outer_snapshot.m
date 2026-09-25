function report=test_canonical_outer_snapshot(verifiedAssets)
% TEST_CANONICAL_OUTER_SNAPSHOT Test snapshot construction with synthetic observations.
% Reuse constructor-verified canonical assets.
if nargin<1 || isempty(verifiedAssets)
    verifiedAssets=gpenmpcNative.loadCanonicalAssets();
end
assets=verifiedAssets;cfg=assets.enmpc;nowNs=2.0e9;
taskSha=repmat('A',1,64);
expected=struct('uid','1234605616436508552','system_id',1, ...
    'component_id',1,'boot_generation',7,'maximum_age_ns',100e6, ...
    'maximum_runtime_age_ns',100e6,'task_identity_sha256',taskSha, ...
    'outer_snapshot_leg_index',2,'outer_snapshot_first_in_leg',false, ...
    'last_committed_inner_generation',7);
sample=struct('source','PX4_EKF2_MAVLINK','uid',expected.uid, ...
    'system_id',1,'component_id',1,'boot_generation',7, ...
    'position_ned_m',[1;2;-3],'velocity_ned_mps',[.1;.2;-.3], ...
    'quaternion_wxyz_body_to_ned',[1;0;0;0], ...
    'omega_frd_rad_s',[.1;.2;.3], ...
    'position_rx_ns',nowNs-10e6,'attitude_rx_ns',nowNs-9e6, ...
    'rates_rx_ns',nowNs-8e6,'position_generation',101, ...
    'attitude_generation',99,'rates_generation',98, ...
    'position_valid',true,'attitude_valid',true,'rates_valid',true);
payload=2.27;
rotor=(5:10).';
stamp=@(source,generation)struct('source',source,'rx_ns',nowNs-5e6, ...
    'generation',generation,'valid',true);
actuator=stamp('HOST_M600_VIRTUAL_ACTUATOR_INTERFACE',91);
actuator.ordering='SOFTWARE_M600_ORDER';
actuator.rotor_command_n=rotor;
actuator.thrust_effectiveness=[.95;1.01;.96;.98;1.02;.97];
outer=stamp('HOST_CAUSAL_RUNTIME',8);
outer.previous_outer_acceleration_correction_i_mps2=[.01;.02;-.03];
outer.runtime_vertical_observer_shadow_i_mps2=[0;0;-.04];
outer.residual_history_f_mps2=[.01;-.02;.03];
outer.runtime_gp_axis_weight_f=[.9;.8;.7];
outer.runtime_gp_responsibility_blend=.5;
outer.runtime_gp_filtered_mean_f_mps2=[.1;-.05;.02];
outer.causal_valid=true;
outer.gp_evidence=struct('available',false,'prediction_sample_closed',false, ...
    'observed_innovation_available',false,'hard_invalid',false,'trust',0.0);
outer.previous_desired_force_projected_n=[3;-4;110];
outer.desired_force_source='COMMITTED_NOMINAL_CONTROLLER_MIRROR';
outer.desired_force_generation=7;
wind=stamp('FROZEN_TASK_WIND_ESTIMATOR',77);
wind.estimate_xy_mps=[1;-.5];
runtime=struct('schema','GPENMPC_LIVE_OUTER_RUNTIME_V1', ...
    'task_identity_sha256',taskSha,'plant_truth_used',false, ...
    'virtual_actuator',actuator,'outer',outer,'wind',wind, ...
    'task',struct('source','FROZEN_GPENMPC_TASK_STATE','leg_index',2, ...
        'payload_kg',payload,'corridor_half_width_m',cfg.corridor_half_width_m));
coeff=zeros(3,8);coeff(:,1)=[1;2;3];coeff(:,2)=[.1;0;0];
trajectory=struct('total_duration_s',10.0,'coefficients_ascending',coeff);
phase=struct('progress_s',.2,'progress_rate',1.0, ...
    'previous_phase_acceleration_s_inv',0.0);
warm=zeros(1,cfg.decision_dimension);warm(1)=.01;
checks=struct;
[snapshot,ok,reason,receipt]=make(sample,runtime,expected,phase,warm,assets);
check('valid_committed_snapshot',ok&&reason=="FRESH_CAUSAL_BOARD_OUTER_SNAPSHOT");
check('canonical_source_and_payload',strcmpi(snapshot.source_root,assets.sourceRoot) ...
    &&strcmpi(snapshot.configuration_payload_sha256, ...
        assets.configurationBinding.effective_configuration_payload_sha256));
check('cached_assets_only',receipt.assets_preverified&&snapshot.hardware_actions==0);

% The exact canonical WholeTask observation field set and its per-field values.
% Desired force is from the prior controller commit, deliberately different
% from current rotor thrust, effectiveness and mass*g.
oracle=struct('position_m',[1;2;3],'velocity_mps',[.1;.2;.3], ...
    'quaternion_wxyz',[1;0;0;0],'body_rate_rad_s',[-.1;-.2;.3], ...
    'previous_desired_force_n',[3;-4;110],'previous_rotor_command_n',rotor, ...
    'previous_outer_acceleration_correction_i_mps2',[.01;.02;-.03], ...
    'runtime_vertical_observer_shadow_i_mps2',[0;0;-.04], ...
    'wind_estimate_xy_mps',[1;-.5],'payload_kg',payload, ...
    'corridor_half_width_m',cfg.corridor_half_width_m);
check('canonical_observation_fields', ...
    isequal(sort(fieldnames(snapshot.observation)),sort(fieldnames(oracle))));
names=fieldnames(oracle);
for k=1:numel(names)
    check(['observation_' names{k}], ...
        isequal(snapshot.observation.(names{k}),oracle.(names{k})));
end
check('warmstart_phase_and_gp_retained',isequal(snapshot.warm_start,warm) ...
    &&isequal(snapshot.phase,phase)&&isequal(snapshot.gp_evidence,outer.gp_evidence));
check('causal_history_and_responsibility_retained', ...
    isequal(snapshot.causal.values,[gpenmpcNormalizedTotalRotorCommand(rotor,payload); ...
        outer.residual_history_f_mps2]) ...
    &&isequal(snapshot.causal.runtime_gp_axis_weight_f,outer.runtime_gp_axis_weight_f) ...
    &&snapshot.causal.runtime_gp_responsibility_blend==.5);
check('force_source_and_generation_disclosed', ...
    receipt.previous_force_source=="COMMITTED_NOMINAL_CONTROLLER_MIRROR" ...
    &&receipt.previous_force_generation==7&&receipt.leg_index==2);

changed=runtime;
changed.virtual_actuator.rotor_command_n=rotor.*1.5;
changed.virtual_actuator.thrust_effectiveness=ones(6,1);
[other,otherOk]=make(sample,changed,expected,phase,warm,assets);
check('actual_rotor_force_not_substituted',otherOk ...
    &&isequal(other.observation.previous_desired_force_n,oracle.previous_desired_force_n) ...
    &&~isequal(other.observation.previous_rotor_command_n,rotor));

% Independent first-in-leg allowance; phase/rate/no-commit must all agree.
firstExpected=expected;firstExpected.outer_snapshot_first_in_leg=true;
firstExpected.last_committed_inner_generation=0;
firstRuntime=runtime;firstRuntime.outer.generation=1;
firstRuntime.outer.desired_force_generation=0;
firstRuntime.outer.desired_force_source='CANONICAL_LEG_INITIAL_FORCE';
firstRuntime.outer.previous_desired_force_projected_n= ...
    (double(assets.profile.mass_properties.base_mass_kg)+payload).*[0;0;9.80665];
firstPhase=phase;firstPhase.progress_s=0;
[first,firstOk,~,firstReceipt]=make(sample,firstRuntime,firstExpected,firstPhase,warm,assets);
check('per_leg_initial_nominal_force',firstOk&&firstReceipt.first_outer_snapshot_in_leg ...
    &&firstReceipt.previous_force_generation==0 ...
    &&isequal(first.observation.previous_desired_force_n, ...
        firstRuntime.outer.previous_desired_force_projected_n));

bad=runtime;bad.outer=rmfield(bad.outer,'previous_desired_force_projected_n');
reject('missing_force_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer=rmfield(bad.outer,'desired_force_source');
reject('missing_force_source_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer=rmfield(bad.outer,'desired_force_generation');
reject('missing_commit_generation_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.previous_desired_force_projected_n=[NaN;0;1];
reject('nonfinite_force_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.previous_desired_force_projected_n=zeros(4,1);
reject('wrong_force_dimension_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.desired_force_source='PX4_ATTITUDE_X_HOST_M600_VIRTUAL_ACTUATOR_INTERFACE';
reject('legacy_applied_force_source_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.desired_force_source='PLANT_TRUTH';
reject('truth_force_source_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.desired_force_generation=8;
reject('future_commit_generation_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.desired_force_generation=6;
reject('stale_commit_generation_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.desired_force_generation=7.5;
reject('fractional_commit_generation_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.generation=7;
reject('current_input_must_follow_commit',sample,bad,expected,phase,warm,assets);
bad=firstRuntime;bad.outer.previous_desired_force_projected_n(3)=sum(rotor);
reject('initial_actual_rotor_force_rejected',sample,bad,firstExpected,firstPhase,warm,assets);
bad=firstRuntime;bad.outer.desired_force_source='COMMITTED_NOMINAL_CONTROLLER_MIRROR';
reject('zero_generation_not_a_commit',sample,bad,firstExpected,firstPhase,warm,assets);
reject('initial_force_cannot_reenter_after_commit',sample,firstRuntime,expected,firstPhase,warm,assets);
badPhase=firstPhase;badPhase.progress_s=.01;
reject('initial_force_only_at_leg_start',sample,firstRuntime,firstExpected,badPhase,warm,assets);
badPhase=firstPhase;badPhase.progress_rate=1.01;
reject('initial_force_initial_rate_required',sample,firstRuntime,firstExpected,badPhase,warm,assets);
badExpected=firstExpected;badExpected.last_committed_inner_generation=1;
reject('initial_force_requires_no_prior_commit',sample,firstRuntime,badExpected,firstPhase,warm,assets);
badExpected=expected;badExpected=rmfield(badExpected,'outer_snapshot_first_in_leg');
reject('missing_independent_first_boundary',sample,runtime,badExpected,phase,warm,assets);
badExpected=expected;badExpected.outer_snapshot_first_in_leg=1;
reject('first_boundary_must_be_logical',sample,runtime,badExpected,phase,warm,assets);
bad=runtime;bad.task.leg_index=1;
reject('wrong_leg_identity_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.rx_ns=nowNs+1;
reject('future_runtime_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.rx_ns=nowNs-expected.maximum_runtime_age_ns-1;
reject('stale_runtime_rejected',sample,bad,expected,phase,warm,assets);
badSample=sample;badSample.source='PLANT_TRUTH';
reject('truth_estimate_rejected',badSample,runtime,expected,phase,warm,assets);
badSample=sample;badSample.uid='wrong';
reject('wrong_uid_rejected',badSample,runtime,expected,phase,warm,assets);
badSample=sample;badSample.position_rx_ns=nowNs-expected.maximum_age_ns-1;
reject('stale_estimate_rejected',badSample,runtime,expected,phase,warm,assets);
bad=runtime;bad.plant_truth_used=true;
reject('truth_runtime_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.virtual_actuator.ordering='PX4_OUTPUT_ORDER';
reject('wrong_rotor_order_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.virtual_actuator.rotor_command_n(1)=NaN;
reject('nonfinite_rotor_rejected',sample,bad,expected,phase,warm,assets);
bad=runtime;bad.outer.gp_evidence.trust=NaN;
reject('nonfinite_gp_rejected',sample,bad,expected,phase,warm,assets);
badWarm=warm;badWarm(1)=NaN;
reject('nonfinite_warmstart_rejected',sample,runtime,expected,phase,badWarm,assets);
badAssets=assets;badAssets.sourceRoot='D:\old_vendor';
reject('legacy_asset_root_rejected',sample,runtime,expected,phase,warm,badAssets);
badAssets=assets;badAssets.binding.verified_source_entries=0;
reject('unverified_assets_rejected',sample,runtime,expected,phase,warm,badAssets);
badAssets=assets;badAssets.enmpc.outer_period_s=.2;
reject('changed_canonical_timing_rejected',sample,runtime,expected,phase,warm,badAssets);
[empty,noAssetsOk,noAssetsReason]=gpenmpcNative.makeBoardOuterSnapshot( ...
    'D:\old_vendor',sample,expected,nowNs,runtime,trajectory,phase,warm);
check('no_old_vendor_fallback',~noAssetsOk&&isempty(fieldnames(empty)) ...
    &&noAssetsReason=="NONCANONICAL_ASSET_ROOT_REJECTED");

report=struct('schema','CANONICAL_OUTER_SNAPSHOT_HOST_TEST_V1', ...
    'status','PASS_HOST_ONLY_CANONICAL_OUTER_SNAPSHOT', ...
    'passed',all(structfun(@logical,checks)),'checks',checks, ...
    'checks_passed',nnz(structfun(@logical,checks)), ...
    'checks_total',numel(fieldnames(checks)), ...
    'canonical_observation_fields',string(fieldnames(oracle)), ...
    'source_manifest_sha256',assets.binding.source_manifest_sha256, ...
    'desired_force_source',receipt.previous_force_source, ...
    'desired_force_generation',receipt.previous_force_generation, ...
    'solver_calls',0,'model_executions',0,'com_open',0,'udp_open',0, ...
    'board_actions',0,'hardware_actions',0,'physical_output_actions',0, ...
    'claim','Synthetic snapshot-boundary parity tests.');
assert(report.passed,'gpenmpcNative:CanonicalSnapshotTest','Snapshot tests failed.');
fprintf('CANONICAL_OUTER_SNAPSHOT %d/%d\n',report.checks_passed,report.checks_total);

    function [snap,accepted,why,receiptValue]=make(s,r,e,p,w,a)
        [snap,accepted,why,receiptValue]=gpenmpcNative.makeBoardOuterSnapshot( ...
            a.packageRoot,s,e,nowNs,r,trajectory,p,w,a);
    end
    function check(name,passed)
        checks.(name)=logical(passed);
        assert(checks.(name),'gpenmpcNative:CanonicalSnapshotAssertion','%s',name);
    end
    function reject(name,s,r,e,p,w,a)
        [empty,accepted,why]=make(s,r,e,p,w,a);
        check(name,~accepted&&isempty(fieldnames(empty))&&strlength(string(why))>0);
    end
end
