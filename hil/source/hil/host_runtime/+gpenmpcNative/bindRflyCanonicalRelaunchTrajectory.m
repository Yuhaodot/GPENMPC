function [trajectory,receipt]=bindRflyCanonicalRelaunchTrajectory(leg,matchState)
% Include the verified ground-state match in the trajectory shared by
% outer prediction and inner reference.
arguments
    leg (1,1) struct
    matchState (1,1) struct
end
base=fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))),'support','delivery');
source=fullfile(base,'matlab','+gpenmpcHil','applyRelaunchStateMatch.m');
sourceSha='421F4A70343E53A54B78F08E2ED7BB17F05CC281995277927F8AC7102498ADD2';
assert(strcmpi(sha(source),sourceSha),'gpenmpcNative:RelaunchSourceIdentity', ...
    'The retained ground-state matching implementation differs.');
configPath=fullfile(base,'config','RELAUNCH_STATE_MATCH_V1.json');
cfg=jsondecode(fileread(configPath));
assert(cfg.state_match_duration_s==11&&cfg.maximum_horizontal_offset_m==2 ...
    &&cfg.maximum_vertical_frame_offset_m==12&&~cfg.plant_truth_used_for_command, ...
    'gpenmpcNative:RelaunchConfiguration','Use the existing ground-state match limits.');
ordinal=double(leg.meta.leg_index)-1;
assert(ordinal>=1&&ordinal<=4&&ordinal==fix(ordinal), ...
    'gpenmpcNative:RelaunchLeg','This binding is for the four post-delivery flights, not initial takeoff.');
assert(all(isfield(matchState,{'capture_count','anchor_valid','horizontal_offset_ned_m', ...
    'current_vertical_frame_offset_m'}))&&matchState.capture_count==ordinal ...
    &&numel(matchState.anchor_valid)>=ordinal&&isequal(matchState.anchor_valid(ordinal),true), ...
    'gpenmpcNative:RelaunchAnchor','The exact leg requires its already captured causal ground anchor.');
offset=double(matchState.horizontal_offset_ned_m(ordinal,:));
z=double(matchState.current_vertical_frame_offset_m);
assert(isequal(size(offset),[1,3])&&all(isfinite(offset))&&offset(3)==0 ...
    &&isscalar(z)&&isfinite(z)&&norm(offset(1:2))<=cfg.maximum_horizontal_offset_m ...
    &&abs(z)<=cfg.maximum_vertical_frame_offset_m, ...
    'gpenmpcNative:RelaunchBounds','Captured EKF-only frame offsets exceed the existing bounded match.');
% Saved global-origin subtraction produces 10.999999999999986 for leg 2's
% nominal 11-s boundary. Tolerance is binary64 representation only, not a
% changed duration, task sample, performance screen or runtime safety margin.
roundoff=64*eps(double(cfg.state_match_duration_s));
firstAfter=find(leg.phase_code~=12,1);
assert(leg.phase_code(1)==12&&leg.local_time_s(1)==0 ...
    &&~isempty(firstAfter)&&abs(leg.local_time_s(firstAfter)-11)<=roundoff ...
    &&all(leg.phase_code(leg.local_time_s<11-roundoff)==12), ...
    'gpenmpcNative:RelaunchPhase','The bound removal interval must lie inside the actual saved ascent.');
nominal=leg.trajectory;duration=double(cfg.state_match_duration_s);
trajectory=nominal;trajectory.schema='RFLY_CAUSAL_RELAUNCH_MATCHED_SAMPLED_REFERENCE_V1';
trajectory.evaluate_fcn=@evaluate;
receipt=struct('schema','RFLY_RELAUNCH_REFERENCE_BINDING_V1', ...
    'leg_index',ordinal+1,'service_ordinal',ordinal,'source',source,'source_sha256',sourceSha, ...
    'configuration',configPath,'configuration_sha256',sha(configPath), ...
    'horizontal_offset_ned_m',offset,'vertical_frame_offset_ned_m',z, ...
    'removal_duration_phase_s',duration,'nominal_duration_s',nominal.total_duration_s, ...
    'nominal_reference_arrays_modified',false, ...
    'outer_prediction_and_inner_reference_share_same_evaluator',true, ...
    'matching_clock','CANONICAL_CAUSAL_LEG_PHASE__NOT_HOST_WALL_TIME', ...
    'ground_anchor_authority_proven_here',false,'plant_truth_used_for_command',false, ...
    'hardware_actions',0);
    function value=evaluate(progress,order)
        assert(isnumeric(progress)&&isreal(progress)&&all(isfinite(progress),'all') ...
            &&isscalar(order)&&any(order==0:3),'gpenmpcNative:RelaunchQuery','Finite phase and original derivative orders required.');
        value=nominal.evaluate_fcn(progress,order);
        tau=min(max(double(progress(:).')/duration,0),1);
        switch order
            case 0
                coefficient=1-(35*tau.^4-84*tau.^5+70*tau.^6-20*tau.^7);
            case 1
                coefficient=-(140*tau.^3-420*tau.^4+420*tau.^5-140*tau.^6)/duration;
            case 2
                coefficient=-(420*tau.^2-1680*tau.^3+2100*tau.^4-840*tau.^5)/duration^2;
            case 3
                coefficient=-(840*tau-5040*tau.^2+8400*tau.^3-4200*tau.^4)/duration^3;
        end
        % Same N/E axes, UP=-NED D; the retained helper's horizontal offset
        % has exactly zero D. Vertical is a constant frame transformation.
        value=value+offset(:)*coefficient;
        if order==0,value(3,:)=value(3,:)-z;end
    end
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(b,'int8'));
value=upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[]));
end
