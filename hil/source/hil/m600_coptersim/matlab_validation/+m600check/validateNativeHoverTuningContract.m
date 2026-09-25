function report=validateNativeHoverTuningContract(c)
% Read-only validation against exact source evidence; no fixture bypass.
report=struct('passed',false,'failure','','errors',{{}},'entries',[],'unchanged_guard_entries',[], ...
    'output_guard_entries',[],'bindings',struct(),'hardware_actions',0,'COM_UDP_actions',0,'authority_granted',false);
try
    keys={'schema','entries','bindings','source_provenance','source_provenance_resolution','unchanged_guard_entries','output_guard_entries', ...
        'maximum_apply_passes','restore_policy','fresh_premutation_original138_required', ...
        'single_send_with_ACK_and_fresh_typed_readback','rollback_all_three_exact_original_bits', ...
        'parameter_attempts_recorded_before_send','geometry12_accounting_separate','geometry67_guards_unchanged', ...
        'guard_scope','active_union_rule','failure_detector_changes','output_mapping_changes','model_changes', ...
        'rate_I_changes','controller_method_family_changes','MPC_USE_HTE_required','no_route_hover_I_target', ...
        'engineering_margin_is_safety_gate','authority_granted','claim'};
    exact(c,keys);require(strcmp(c.schema,'TEMPORARY_M600_NATIVE_HOVER_TUNING_V1'),'Unknown schema.');
    require(c.maximum_apply_passes==1&&strcmp(c.restore_policy,'IDEMPOTENT_PER_OWNER_AFTER_DISARM_ZERO_OUTPUTS'),'Wrong apply/independent restore policy.');
    for n={'fresh_premutation_original138_required','single_send_with_ACK_and_fresh_typed_readback', ...
            'rollback_all_three_exact_original_bits','parameter_attempts_recorded_before_send', ...
            'geometry12_accounting_separate','geometry67_guards_unchanged'},truth(c,n{1},true);end
    for n={'failure_detector_changes','output_mapping_changes','model_changes','rate_I_changes','controller_method_family_changes'}
        require(isnumeric(c.(n{1}))&&isscalar(c.(n{1}))&&isequal(c.(n{1}),0),['Forbidden change ' n{1}]);
    end
    require(isequal(c.MPC_USE_HTE_required,1)&&isequal(c.no_route_hover_I_target,0),'HTE/I phase contract changed.');
    truth(c,'engineering_margin_is_safety_gate',false);truth(c,'authority_granted',false);
    require(strcmp(c.guard_scope,'PREMUTATION_ORIGINAL138__ACTIVE_UNION_REQUIRES_SEPARATE_GEOMETRY12_CONTRACT') ...
        &&strcmp(c.active_union_rule,'Exact disjoint geometry12 plus tuning3 only; remaining123 unchanged. Never silently exempt 3 values from geometry67.'),'Ambiguous combined guard scope.');
    require(strcmp(c.claim,'HOST_ONLY_PARAMETER_CONTRACT__NOT_LIVE_AUTHORIZATION_OR_HOVER_PASS'),'Wrong claim.');
    exact(c.bindings,{'current138','current138_raw','attitude_candidate','ground_selection'});
    [r,report.bindings.current138]=binding(c.bindings.current138,'BF31EEF276ADC9756371E5FEF282AC10465056B5710E3A87D64CD422F5F1E0B4');
    [raw,report.bindings.current138_raw]=binding(c.bindings.current138_raw,'A535B5D87F511DACEF68B42A72660F80A66FA0831B5DC7FA97313C1D17210973');
    [a,report.bindings.attitude_candidate]=binding(c.bindings.attitude_candidate,'A6C93CC99FF39A6B84CD132E9DB0A4471E60D125781AE7BFD7677A260CD29C19');
    [s,report.bindings.ground_selection]=binding(c.bindings.ground_selection,'02081045511311399615C322FBDB32572C2FC98439FDD767BBA60871FF92A3CD');
    require(isequal(r.raw_receipt,c.bindings.current138_raw)&&isequal(s.attitude_candidate,c.bindings.attitude_candidate),'Cross-file identity differs.');
    for n={'passed','safety_preflight_passed','all_138_diagnostic_reads_complete','original_105_semantics_match','COM_closed'},truth(r,n{1},true);end
    require(isempty(r.failure)&&r.original_105_validation.matching_rows==105&&r.additional_validation.passed_rows==33,'Incomplete current138.');
    for n={'passed','COM_closed','diagnostic_parameters_complete','virtual_output_path_disabled','physical_output_path_disabled'},truth(raw,n{1},true);end
    require(isempty(raw.failure)&&~raw.armed&&raw.landed_state==1&&raw.COM_open_attempts==1,'Raw safety failed.');
    require(strcmp(raw.uid,gpenmpc_device_identity('uid'))&&raw.board_version==56&&strcmp(raw.commit,'6ea3539157ca358c70a515878b77077af7d4611d'),'Raw identity mismatch.');
    for n={'parameter_writes','mapping_writes','arm_disarm_mode_requests','flash_reboot_count','physical_output_actions'},require(raw.(n{1})==0,'Nonzero raw actions.');end
    require(raw.sd_diagnostics.passed&&isempty(raw.sd_diagnostics.active_fault_log_names)&&~raw.virtual_output_observation.any_nonzero_or_nonfinite,'Crash/output evidence failed.');
    rows=raw.diagnostic_parameter_observations;require(numel(rows)==138&&numel(unique({rows.name}))==138,'Current138 not unique.');
    for k=1:138,require(isempty(rows(k).read_error)&&rows(k).write_count==0,'Unreadable/written baseline row.');typed(rows(k).typed_value);end
    require(strcmp(a.schema,'HOST_NATIVE_ATTITUDE_P_ROUTH_CANDIDATE_V1')&&a.passed&&isempty(a.failure) ...
        &&a.case_count==a.cases_passed&&all([a.checks.passed]),'Attitude proof incomplete.');
    require(~a.SIL_outcomes_read&&~a.live_admission&&a.hardware_actions==0&&a.parameter_writes==0 ...
        &&~s.outcomes_read&&s.MEX_calls_at_selection==0&&~s.live_admission&&s.hardware_actions==0,'Prospective selection or no-authority evidence failed.');
    require(strcmp(s.schema,'HOST_NATIVE_TAKEOFF_CASCADE_PROSPECTIVE_V1')&&s.selection.rate_I_unchanged ...
        &&~s.selection.model_parameters_changed&&~s.selection.protective_parameters_changed ...
        &&s.selection.position_vertical_I_gain_all_cases==0&&s.fixture.native_MPC_USE_HTE_parameter==1,'Selection method/phase differs.');
    require(a.selection.engineering_gain_factor==.5&&~s.selection.outcomes_read_for_selection ...
        &&~s.selection.live_parameter_apply_authorized_by_this_file,'Selection provenance differs.');
    candidate=a.candidate_configuration;candidate.attitude_p=a.original_configuration.attitude_p;
    require(isequaln(candidate,a.original_configuration),'Attitude candidate contains another control change.');
    refs=[a.script_identity;a.input_bindings(:);s.script;s.numerical_sources(:)];
    [~,j]=unique(lower(string({refs.path})),'stable');refs=refs(j);
    % Resolve the one explicitly preserved consumed file, never the changed
    % work copy. Hash equality is required; the immutable selection stays as is.
    rr=c.source_provenance_resolution;exact(rr,{'declared','consumed','reason'});
    require(isequal(rr.declared,s.script)&&strcmp(rr.reason, ...
        'STORAGE_LOCATION_ONLY__IDENTICAL_CONSUMED_BYTES_AND_SHA__ORIGINAL_DECLARATION_PRESERVED'),'Source relocation provenance altered.');
    consumedPath=fullfile(fileparts(c.bindings.ground_selection.path),'CONSUMED_RUNNER.m');
    require(strcmp(rr.consumed.path,consumedPath)&&rr.consumed.bytes==s.script.bytes ...
        &&strcmpi(rr.consumed.sha256,s.script.sha256),'Consumed source bytes/SHA differ from declared input.');
    binding(rr.consumed,'F69084A034A519C1109D0EAE538448FF1F0CE107371F71FDA7C1A7140F942805',false);
    which=find(strcmp({refs.path},s.script.path));require(isscalar(which),'Ambiguous source relocation.');
    refs(which)=rr.consumed;
    require(isequaln(c.source_provenance(:),refs(:)),'Source provenance set altered.');
    for k=1:numel(refs),binding(refs(k),'',false);end
    binding(s.model,'969D347CE041715CAED49BE0B7897486E34FA4E0B22EB82E50E9752B8F4F19E4',false);
    loaded=load(s.model.path,'parameters');p=loaded.parameters;
    modelMass=p.profile.mass_properties.base_mass_kg+s.selection.payload_kg+p.mission.plant_mismatch.mass_bias_kg;
    modelMax=p.calibration.rotor_allocation.per_rotor_thrust_upper_n;
    hover=double(single(modelMass*9.80665/(6*modelMax)));
    require(modelMass==s.selection.mass_kg&&modelMax==s.selection.per_rotor_Fmax_N ...
        &&hover==s.selection.selected_hover_REAL32&&strcmpi(bits(hover),'3F186B9E'),'Model-derived hover mismatch.');
    marginRule=.5*min(a.selection.per_axis_stability_upper_s_inverse);
    selected=a.selection.common_REAL32_value_s_inverse;
    next=double(typecast(uint32(hex2dec('4026CCBE'))+uint32(1),'single'));
    require(selected<=marginRule&&next>marginRule&&strcmpi(bits(selected),'4026CCBE'),'Attitude REAL32 floor rule differs.');
    names={'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'};old={'40D00000','40D00000','3F000000'};target={'4026CCBE','4026CCBE','3F186B9E'};
    require(isstruct(c.entries)&&numel(c.entries)==3&&isequal(reshape({c.entries.name},1,[]),names),'Wrong/duplicate tuning names.');
    for k=1:3
        e=c.entries(k);exact(e,{'name','mav_type','original_raw_bits_hex','target_raw_bits_hex'});
        require(e.mav_type==9&&strcmpi(e.original_raw_bits_hex,old{k})&&strcmpi(e.target_raw_bits_hex,target{k}),'Tuning type/bits differ.');
        finiteBits(e.original_raw_bits_hex,9);finiteBits(e.target_raw_bits_hex,9);
        q=rows(strcmp({rows.name},e.name)).typed_value;
        require(q.mav_type==9&&strcmpi(q.raw_bits_hex,e.original_raw_bits_hex),'Original parameter differs.');
        if k<=2
            d=a.selection.parameter_changes(k);require(strcmp(d.name,e.name)&&strcmpi(d.candidate_raw_bits_hex,e.target_raw_bits_hex)&&strcmpi(d.rollback_raw_bits_hex,e.original_raw_bits_hex),'Attitude delta provenance mismatch.');
        end
    end
    keep=~ismember({rows.name},names);guards=guardRows(rows(keep));
    require(numel(guards)==135&&sameGuards(c.unchanged_guard_entries,guards),'Unchanged135 guard set altered.');
    q=raw.parameters;outputs=repmat(struct('name','','mav_type',0,'raw_bits_hex',''),numel(q),1);
    for k=1:numel(q),typed(q(k));outputs(k)=struct('name',q(k).name,'mav_type',q(k).mav_type,'raw_bits_hex',q(k).raw_bits_hex);end
    require(numel(outputs)==29&&sameGuards(c.output_guard_entries,outputs),'Output/profile guards altered.');
    for k=1:numel(outputs)
        if startsWith(outputs(k).name,'HIL_ACT_FUNC')||startsWith(outputs(k).name,'PWM_MAIN_FUNC'),require(strcmp(outputs(k).raw_bits_hex,'00000000'),'Output mapping nonzero.');end
    end
    h=guards(strcmp({guards.name},'MPC_USE_HTE'));require(h.mav_type==6&&strcmp(h.raw_bits_hex,'00000001'),'HTE disabled.');
    report.entries=c.entries;report.unchanged_guard_entries=guards;report.output_guard_entries=outputs;report.passed=true;
catch e
    report.failure=[e.identifier ': ' e.message];report.errors={struct('identifier',e.identifier,'message',e.message)};
end
end
function guards=guardRows(rows)
guards=repmat(struct('name','','mav_type',0,'raw_bits_hex',''),numel(rows),1);
for k=1:numel(rows),q=rows(k).typed_value;guards(k)=struct('name',q.name,'mav_type',q.mav_type,'raw_bits_hex',q.raw_bits_hex);end
end
function yes=sameGuards(a,b)
yes=isstruct(a)&&numel(a)==numel(b)&&numel(unique({a.name}))==numel(b);
if ~yes,return;end
for k=1:numel(b)
    exact(a(k),{'name','mav_type','raw_bits_hex'});finiteBits(a(k).raw_bits_hex,a(k).mav_type);
    yes=yes&&strcmp(a(k).name,b(k).name)&&a(k).mav_type==b(k).mav_type&&strcmpi(a(k).raw_bits_hex,b(k).raw_bits_hex);
end
end
function typed(q)
require(isstruct(q)&&isscalar(q)&&ismember(q.mav_type,[6,9]),'Unknown typed value.');v=finiteBits(q.raw_bits_hex,q.mav_type);
require(isnumeric(q.decoded)&&isscalar(q.decoded)&&isequal(double(q.decoded),v),'Typed value/raw bits mismatch.');
end
function v=finiteBits(b,t)
require(~isempty(regexp(char(b),'^[0-9A-Fa-f]{8}$','once'))&&ismember(t,[6,9]),'Bad type/raw bits.');u=uint32(hex2dec(b));
if t==9,v=double(typecast(u,'single'));else,v=double(typecast(u,'int32'));end
require(isfinite(v),'Nonfinite raw value.');
end
function b=bits(v),b=upper(dec2hex(typecast(single(v),'uint32'),8));end
function [r,id]=binding(b,expected,decode)
if nargin<3,decode=true;end
exact(b,{'path','bytes','sha256'});require(isnumeric(b.bytes)&&isscalar(b.bytes)&&b.bytes>0,'Invalid byte count.');
require(~isempty(regexp(char(b.sha256),'^[0-9A-Fa-f]{64}$','once')),'Invalid SHA.');
if ~isempty(expected),require(strcmpi(b.sha256,expected),'Unexpected authority SHA.');end
f=fopen(b.path,'rb');require(f>=0,'Missing bound file.');g=onCleanup(@()fclose(f)); %#ok<NASGU>
raw=fread(f,Inf,'*uint8');d=java.security.MessageDigest.getInstance('SHA-256');d.update(raw);
h=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
require(numel(raw)==b.bytes&&strcmpi(h,b.sha256),'Bound bytes/SHA mismatch.');id=b;r=[];
if decode,r=jsondecode(native2unicode(raw.','UTF-8'));end
end
function exact(s,n),require(isstruct(s)&&isscalar(s)&&isequal(sort(fieldnames(s)),sort(n(:))),'Missing/unknown schema field.');end
function truth(s,n,v),require(islogical(s.(n))&&isscalar(s.(n))&&s.(n)==v,['Changed logical ' n]);end
function require(v,m),assert(isscalar(v)&&v,'m600check:NativeHoverTuning','%s',m);end
