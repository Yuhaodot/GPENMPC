function [payload,state]=encodeCopterSimTerrainDiagnostics( ...
    prefix7,terrain15,terrainReason,lockedHeight,terrainFault,acceptedReset,state)
%#codegen
% Observation-only extension of outCopterData32, retaining its seven-field prefix.
% 8 capture_valid, 9 first_reason, 10 first_locked_height, 11:25 raw terrain15,
% 26 extension tag=1, 27:32 reserved zero. Raw NaN/Inf is diagnostic evidence.
% Caller supplies the SAME terrain snapshot that the production core used.
% acceptedReset means d.reset_applied, not an unaccepted reset request.
% State must be initialized by initialCopterSimTerrainDiagnosticState at the
% start of a genuinely new generated-model lifecycle. No persistent globals.
assert(isa(prefix7,'double')&&isreal(prefix7)&&isequal(size(prefix7),[7,1]));
assert(isa(terrain15,'double')&&isreal(terrain15)&&isequal(size(terrain15),[15,1]));
assert(isreal(terrainReason)&&isscalar(terrainReason)&&isreal(lockedHeight)&&isscalar(lockedHeight));
assert(islogical(terrainFault)&&isscalar(terrainFault));
assert(islogical(acceptedReset)&&isscalar(acceptedReset));
assert(islogical(state.capture_valid)&&isscalar(state.capture_valid)&& ...
    isa(state.first_reason,'double')&&isreal(state.first_reason)&&isscalar(state.first_reason)&& ...
    isa(state.first_locked_height,'double')&&isreal(state.first_locked_height)&&isscalar(state.first_locked_height)&& ...
    isa(state.first_terrain15,'double')&&isreal(state.first_terrain15)&&isequal(size(state.first_terrain15),[15,1]));
if acceptedReset&&~terrainFault
    state=m600check.initialCopterSimTerrainDiagnosticState();
end
if terrainFault&&~state.capture_valid
    state.capture_valid=true;
    state.first_reason=double(terrainReason);
    state.first_locked_height=double(lockedHeight);
    state.first_terrain15=terrain15;
end
payload=zeros(32,1);
payload(1:7)=prefix7;
payload(8)=double(state.capture_valid);
payload(9)=state.first_reason;
payload(10)=state.first_locked_height;
payload(11:25)=state.first_terrain15;
payload(26)=1.0;
end
