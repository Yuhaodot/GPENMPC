function [accepted,reason,receipt]=validateOuterObservationAdvance(previous,current)
% Require estimate, rotor and causal state advancement per outer solve.
% Allow a held wind observation while its upstream freshness check remains valid.
accepted=false;reason="INVALID_CAUSAL_INPUT_BOUNDARY";
receipt=struct('wind_held',false,'source_stamps_rewritten',false);
if ~valid(current),return;end
if isstruct(previous)&&isscalar(previous)&&isempty(fieldnames(previous))
    accepted=true;reason="FIRST_VALID_CAUSAL_BOUNDARY";return
end
if ~valid(previous),return;end
g=generations(current);p=generations(previous);
if any(g(1:5)<=p(1:5))
    reason="NONADVANCING_CAUSAL_INPUT_GENERATION";return
end
rx=[current.estimate_rx_ns(:).',current.virtual_actuator_rx_ns,current.outer_runtime_rx_ns];
oldrx=[previous.estimate_rx_ns(:).',previous.virtual_actuator_rx_ns,previous.outer_runtime_rx_ns];
if any(rx<oldrx)
    reason="REVERSED_CAUSAL_INPUT_RECEIVE_TIME";return
end
if g(6)<p(6)
    reason="REVERSED_WIND_OBSERVATION_GENERATION";return
end
if g(6)==p(6)
    if ~isequaln(current.wind_observation,previous.wind_observation)
        reason="HELD_WIND_OBSERVATION_MUTATED";return
    end
    receipt.wind_held=true;
elseif current.wind_rx_ns<previous.wind_rx_ns
    reason="REVERSED_WIND_OBSERVATION_RECEIVE_TIME";return
end
accepted=true;reason="SOURCES_ADVANCED_OR_ORIGINAL_FRESH_WIND_HELD";
end

function g=generations(b)
g=[double(b.estimate_generations(:).'),double(b.virtual_actuator_generation), ...
    double(b.outer_runtime_generation),double(b.wind_generation)];
end
function yes=valid(b)
fields={'estimate_generations','estimate_rx_ns','virtual_actuator_generation', ...
    'outer_runtime_generation','wind_generation','virtual_actuator_rx_ns', ...
    'outer_runtime_rx_ns','wind_rx_ns','wind_observation'};
yes=isstruct(b)&&isscalar(b)&&all(isfield(b,fields));
if ~yes,return;end
yes=ints(b.estimate_generations,3,1)&&ints(b.estimate_rx_ns,3,0) ...
    &&ints(b.virtual_actuator_generation,1,1)&&ints(b.outer_runtime_generation,1,1) ...
    &&ints(b.wind_generation,1,1)&&ints(b.virtual_actuator_rx_ns,1,0) ...
    &&ints(b.outer_runtime_rx_ns,1,0)&&ints(b.wind_rx_ns,1,0) ...
    &&isstruct(b.wind_observation)&&isscalar(b.wind_observation) ...
    &&all(isfield(b.wind_observation,{'source','rx_ns','generation','valid','estimate_xy_mps'}));
if ~yes,return;end
w=b.wind_observation;
yes=(ischar(w.source)&&isrow(w.source)) ...
    ||(isstring(w.source)&&isscalar(w.source)&&~ismissing(w.source));
if ~yes,return;end
yes=string(w.source)=="FROZEN_TASK_WIND_ESTIMATOR" ...
    &&ints(w.rx_ns,1,0)&&isequal(double(w.rx_ns),double(b.wind_rx_ns)) ...
    &&ints(w.generation,1,1)&&isequal(double(w.generation),double(b.wind_generation)) ...
    &&islogical(w.valid)&&isscalar(w.valid)&&w.valid ...
    &&isnumeric(w.estimate_xy_mps)&&isreal(w.estimate_xy_mps) ...
    &&numel(w.estimate_xy_mps)==2&&all(isfinite(w.estimate_xy_mps(:)));
end
function yes=ints(x,n,minimum)
yes=isnumeric(x)&&isreal(x)&&numel(x)==n&&all(isfinite(x(:))) ...
    &&all(x(:)>=minimum)&&all(x(:)<=flintmax)&&all(x(:)==fix(x(:)));
end
