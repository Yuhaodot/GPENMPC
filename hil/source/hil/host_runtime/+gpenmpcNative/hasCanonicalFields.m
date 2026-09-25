function yes=hasCanonicalFields(record,names)
%#codegen
% Exact all(isfield(record,names)) for the fixed field-name lists in the
% extracted numerical helpers. Coder does not accept cell input to isfield.
yes=true;
for j=coder.unroll(1:numel(names))
    yes=yes&&isfield(record,names{j});
end
end
