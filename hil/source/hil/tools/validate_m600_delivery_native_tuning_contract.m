function report=validate_m600_delivery_native_tuning_contract(c)
% Validate delivery-tuning files and bindings.
report=struct('passed',false,'failure','','hardware_actions',0,'COM_open',0);
try
    exact(c,{'schema','entries','unchanged_guard_entries','output_guard_entries', ...
        'parent_plan','superseded_derivative_binding','current_derivative_binding', ...
        'delivery_runtime_bindings','change_reason','selected_values_changed', ...
        'controller_method_family_changes','physical_output_actions', ...
        'maximum_apply_passes','restore_policy','claim'});
    assert(strcmp(c.schema,'TEMPORARY_M600_CANONICAL_DELIVERY_NATIVE_TUNING_V1'));
    assert(~c.selected_values_changed&&c.controller_method_family_changes==0&& ...
        c.physical_output_actions==0&&c.maximum_apply_passes==1&& ...
        strcmp(c.restore_policy,'IDEMPOTENT_PER_OWNER_AFTER_DISARM_ZERO_OUTPUTS'));
    assert(strcmp(c.change_reason, ...
        'PASSIVE_GROUND_AND_PHYSICAL_DELIVERY_RUNTIME_SUPERSEDES_ONLY_THE_OLD_PLANT_DERIVATIVE_BYTES'));
    assert(strcmp(c.claim,'CURRENT_SOURCE_PARAMETER_SERVICE_CONTRACT'));
    verify(c.parent_plan);verify(c.current_derivative_binding);
    for k=1:numel(c.delivery_runtime_bindings),verify(c.delivery_runtime_bindings(k));end
    parent=jsondecode(fileread(c.parent_plan.path));p=parent.native_hover_tuning;
    assert(isequaln(c.entries,p.entries)&&isequaln(c.unchanged_guard_entries,p.unchanged_guard_entries)&& ...
        isequaln(c.output_guard_entries,p.output_guard_entries));
    old=p.source_provenance(strcmpi({p.source_provenance.path},c.current_derivative_binding.path));
    assert(isscalar(old)&&isequaln(c.superseded_derivative_binding,old));
    assert(~strcmpi(c.superseded_derivative_binding.sha256,c.current_derivative_binding.sha256));
    names={'MC_ROLL_P','MC_PITCH_P','MPC_THR_HOVER'};
    originals={'40D00000','40D00000','3F000000'};
    targets={'4026CCBE','4026CCBE','3F186B9E'};
    assert(numel(c.entries)==3&&isequal(reshape({c.entries.name},1,[]),names));
    for k=1:3
        assert(c.entries(k).mav_type==9&&strcmpi(c.entries(k).original_raw_bits_hex,originals{k})&& ...
            strcmpi(c.entries(k).target_raw_bits_hex,targets{k}));
    end
    assert(numel(c.unchanged_guard_entries)==135&& ...
        numel(unique({c.unchanged_guard_entries.name}))==135);
    assert(numel(c.output_guard_entries)==29&& ...
        numel(unique({c.output_guard_entries.name}))==29);
    assert(numel(c.delivery_runtime_bindings)==4);
    report.passed=true;
catch problem
    report.failure=[problem.identifier ': ' problem.message];
end
end

function verify(item)
exact(item,{'path','bytes','sha256'});info=dir(item.path);
assert(isfile(item.path)&&isscalar(info)&&info.bytes==item.bytes&& ...
    strcmpi(m600check.fileSha256(item.path),item.sha256),'m600delivery:BindingMismatch');
end
function exact(value,names)
assert(isstruct(value)&&isscalar(value)&&isequal(sort(fieldnames(value)),sort(names(:))), ...
    'm600delivery:ContractSchema');
end
