function result=evaluateVirtualOutputEvidence(rows)
% Pure readback interpretation. Zero mappings are not measured zero outputs.
result=struct('status','NOT_OBSERVED__ZERO_PATH_GUARDS_ARE_SEPARATE', ...
    'message_count',numel(rows),'all_reported_values_observed_zero',false, ...
    'sixteen_hil_control_values_observed_zero',false,'any_nonzero_or_nonfinite',false, ...
    'raw_messages',rows);
hil=false;
for k=1:numel(rows)
    p=rows(k).payload;values=[];
    if strcmp(rows(k).name,'HIL_ACTUATOR_CONTROLS')&&isfield(p,'controls')
        values=double(p.controls(:));hil=hil||(numel(values)==16);
    elseif strcmp(rows(k).name,'ACTUATOR_OUTPUT_STATUS')&&isfield(p,'actuator')
        values=double(p.actuator(:));
    end
    if isempty(values)||any(~isfinite(values))||any(values~=0)
        result.any_nonzero_or_nonfinite=true;
    end
end
if ~isempty(rows)
    if result.any_nonzero_or_nonfinite,result.status='OBSERVED_NONZERO_NONFINITE_OR_UNRECOGNIZED_OUTPUT';
    else
        result.status='OBSERVED_ALL_REPORTED_OUTPUT_VALUES_ZERO';
        result.all_reported_values_observed_zero=true;
        result.sixteen_hil_control_values_observed_zero=hil;
    end
end
end
