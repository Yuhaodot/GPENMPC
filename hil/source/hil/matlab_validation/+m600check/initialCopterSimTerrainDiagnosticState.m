function state=initialCopterSimTerrainDiagnosticState()
%#codegen
% Pure bounded observation memory; no control, model or transport state.
state=struct('capture_valid',false,'first_reason',0.0, ...
    'first_locked_height',0.0,'first_terrain15',zeros(15,1));
end
