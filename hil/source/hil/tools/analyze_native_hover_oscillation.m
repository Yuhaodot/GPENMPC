function out=analyze_native_hover_oscillation(outputDir)
% Analyze retained oscillation data.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir),'analysis:ExistingOutput','Use a new derived directory.');
b=fileparts(fileparts(mfilename('fullpath')));
inputDir=fullfile(gpenmpc_external_path('native_hover_motor_motion_fixture'));
S=jsondecode(fileread(fullfile(inputDir,'SUMMARY.json')));
T=readtable(fullfile(inputDir,'TRUTH.csv')); A=readtable(fullfile(inputDir,'ACTUATOR.csv'));
P=readtable(fullfile(inputDir,'ATTITUDE.csv')); D=readtable(fullfile(inputDir,'MODEL_DIAGNOSTIC.csv'));
M=readtable(fullfile(inputDir,'MOTOR_MOMENT_MOTION.csv'));
formal=S.formal_begin_s; fault=S.first_model_fault_received_s;
stateNames={'x','y','z','vx','vy','vz','roll','pitch','yaw','p','q','r','ax','ay','az'};
X=T{:,stateNames};same=all(X==X(end,:),2);lastChanged=find(~same,1,'last');
assert(~isempty(lastChanged)&&lastChanged+2<=height(T),'analysis:FrozenSuffix','Expected retained latched-state suffix.');
lastDynamic=lastChanged+1; frozenStart=lastDynamic+1;
assert(T.rx_s(lastDynamic)<=fault&&T.rx_s(frozenStart)>=T.rx_s(lastDynamic));
dynamicEnd=T.mapped_source_s(lastDynamic);
U=T{:,compose('animation%d',1:6)}/1000;
AU=A{:,compose('control%d',1:6)};
formalMask=T.mapped_source_s>=formal & (1:height(T)).'<=lastDynamic;
firstUpper=find(formalMask&any(U>=1,2),1);
firstMixed=find(formalMask&any(U>=1,2)&any(U<=0,2),1);
firstMavUpper=find(A.mapped_source_s>=formal&A.mapped_source_s<=fault&any(AU>=1,2),1);
firstMavMixed=find(A.mapped_source_s>=formal&A.mapped_source_s<=fault&any(AU>=1,2)&any(AU<=0,2),1);
firstInverted=find(formalMask&cos(T.roll).*cos(T.pitch)<=0,1); % Geometric inversion event.
unsaturatedEnd=dynamicEnd;
if ~isempty(firstUpper),unsaturatedEnd=min(unsaturatedEnd,T.mapped_source_s(firstUpper));end
if ~isempty(firstInverted),unsaturatedEnd=min(unsaturatedEnd,T.mapped_source_s(firstInverted));end
changes=[false;any(diff(U,1,1)~=0,2)] & formalMask;
forwardSegments=struct([]);edges=[-Inf,formal;formal,fault;fault,Inf];
names={'BEFORE_FORMAL','FORMAL_TO_FIRST_RAW_FAULT','AFTER_FIRST_RAW_FAULT'};
for k=1:3
    for f={'mapped_source_s','rx_s'}
        ts=A.(f{1});keep=ts>=edges(k,1)&ts<edges(k,2);
        q=frequency(ts(keep));q.phase=names{k};q.clock=f{1};
        if isempty(forwardSegments),forwardSegments=q;else,forwardSegments(end+1)=q;end %#ok<AGROW>
    end
end
% Identify raw local extrema.
extrema=struct('axis',{},'truth_raw_index',{},'mapped_source_s',{},'formal_relative_s',{}, ...
    'angle_deg',{},'kind',{},'same_sign_period_s',{},'abs_amplitude_ratio',{},'log_envelope_growth_per_s',{},'before_saturation',{});
axes={'roll','pitch','yaw'};
for axis=1:3
    values=unwrap(T.(axes{axis}));previous=struct('MAX',[],'MIN',[]);
    for k=2:lastDynamic-1
        if ~formalMask(k),continue;end
        left=values(k)-values(k-1);right=values(k+1)-values(k);
        if left*right>=0,continue;end
        kind='MIN';if left>0,kind='MAX';end
        period=NaN;ratio=NaN;growth=NaN;old=previous.(kind);
        if ~isempty(old)
            period=T.mapped_source_s(k)-T.mapped_source_s(old);
            if values(old)~=0,ratio=abs(values(k)/values(old));end
            if ratio>0&&period>0,growth=log(ratio)/period;end
        end
        extrema(end+1)=struct('axis',axes{axis},'truth_raw_index',T.raw_index(k), ... %#ok<AGROW>
            'mapped_source_s',T.mapped_source_s(k),'formal_relative_s',T.mapped_source_s(k)-formal, ...
            'angle_deg',values(k)*180/pi,'kind',kind,'same_sign_period_s',period, ...
            'abs_amplitude_ratio',ratio,'log_envelope_growth_per_s',growth, ...
            'before_saturation',T.mapped_source_s(k)<unsaturatedEnd);
        previous.(kind)=k;
    end
end
% Pair true p/q/r with collected PX4 ATTITUDE rates.
% Leave unavailable rate setpoints missing.
small=T.mapped_source_s>=formal&T.mapped_source_s<unsaturatedEnd;
truthRows=find(small);N=numel(truthRows);pairs=nan(N,12);
for k=1:N
    i=truthRows(k);[delta,j]=min(abs(P.mapped_source_s-T.mapped_source_s(i)));
    pairs(k,:)=[T.raw_index(i),P.raw_index(j),T.mapped_source_s(i),P.mapped_source_s(j), ...
        delta,T{i,{'p','q','r'}},P{j,{'p','q','r'}},P.mapped_source_s(j)-T.mapped_source_s(i)];
end
zeroLag=struct([]);lagRows=zeros(0,8);best=struct([]);
% Characterize delay on a fixed +/-0.25 s offset grid.
lagGrid=(-0.25:0.01:0.25).';t=T.mapped_source_s(small);
[pt,uniqueP]=unique(P.mapped_source_s,'stable');assert(all(diff(pt)>0));
for axis=1:3
    fit=fitLine(pairs(:,5+axis),pairs(:,8+axis));fit.axis=axes{axis};
    if isempty(zeroLag),zeroLag=fit;else,zeroLag(end+1)=fit;end %#ok<AGROW>
    source=T.(char('p'+axis-1));source=source(small);
    obs=P.(char('p'+axis-1));obs=obs(uniqueP);local=zeros(numel(lagGrid),8);
    for j=1:numel(lagGrid)
        shifted=interp1(pt,obs,t+lagGrid(j),'linear',NaN);z=fitLine(source,shifted);
        local(j,:)=[axis,lagGrid(j),z.rows,z.gain,z.intercept,z.correlation,z.rms_residual,z.max_abs_residual];
    end
    lagRows=[lagRows;local]; %#ok<AGROW>
    score=local(:,6);score(~isfinite(score))=-Inf;[~,j]=max(score);
    z=struct('axis',axes{axis},'lag_s',local(j,2),'paired_rows',local(j,3), ...
        'gain',local(j,4),'intercept',local(j,5),'correlation',local(j,6), ...
        'rms_residual',local(j,7),'max_abs_residual',local(j,8), ...
        'at_grid_boundary',j==1||j==numel(lagGrid));
    if isempty(best),best=z;else,best(end+1)=z;end %#ok<AGROW>
end
% Compare Euler-derived and interval-average body rates.
rateGap=nan(height(T),3);
for k=2:height(T)
    dt=T.sim_s(k)-T.sim_s(k-1);if dt<=0,continue;end
    before=T{k-1,axes};after=T{k,axes};dr=wrap(after-before).'/dt;mid=before+wrap(after-before)/2;
    rr=mid(1);pp=mid(2);
    fromEuler=[dr(1)-dr(3)*sin(pp);dr(2)*cos(rr)+dr(3)*sin(rr)*cos(pp); ...
        -dr(2)*sin(rr)+dr(3)*cos(rr)*cos(pp)];
    rateGap(k,:)=fromEuler.'-(T{k-1,{'p','q','r'}}+T{k,{'p','q','r'}})/2;
end
inOriginal=T.mapped_source_s>=S.arm_request_s&T.mapped_source_s<=fault;
frozenMask=(1:height(T)).'>lastDynamic;
[hasTruth,truthIndex]=ismember(M.truth_raw_index,T.raw_index);
assert(all(hasTruth));beforeFreeze=M.mapped_source_s>=formal&truthIndex<=lastDynamic;
oldWindow=M.mapped_source_s>=S.arm_request_s&M.mapped_source_s<=fault;
summary=struct('last_changing_state_row',tableRow(T,lastDynamic), ...
    'first_repeated_frozen_state_row',tableRow(T,frozenStart), ...
    'truth_frozen_suffix_rows',nnz(frozenMask),'frozen_rows_in_original_receive_cutoff',nnz(inOriginal&frozenMask), ...
    'truth_timestamp_advances_with_frozen_state',nnz(diff(T.sim_s(frozenStart:end))>0), ...
    'last_healthy_diagnostic',tableRow(D,find(D.failed==0,1,'last')), ...
    'first_failed_diagnostic',tableRow(D,find(D.failed~=0,1)), ...
    'old_endpoint_rate_gap_full_window',maxFinite(abs(M{oldWindow,{'euler_vs_rate_x','euler_vs_rate_y','euler_vs_rate_z'}})), ...
    'midpoint_rate_gap_before_freeze',maxFinite(abs(rateGap(formalMask,:))), ...
    'midpoint_rate_gap_unsaturated',maxFinite(abs(rateGap(small,:))), ...
    'midpoint_rate_gap_frozen_tail',maxFinite(abs(rateGap(frozenMask,:))), ...
    'lag_torque_motion_sign_before_freeze',signFit(M{beforeFreeze,{'approx_lag_tau_x','approx_lag_tau_y','approx_lag_tau_z'}}, ...
        M{beforeFreeze,{'motion_inferred_tau_x','motion_inferred_tau_y','motion_inferred_tau_z'}}));
out=struct('schema','HOST_NATIVE_HOVER_OSCILLATION_AND_RATE_SCOPE_V1', ...
    'input_dir',inputDir,'forwarded_actuator_stream',forwardSegments, ...
    'observed_animation_input_changes',frequency(T.mapped_source_s(changes)), ...
    'animation_scope','Animation reports model inputs. Sampling at 50 Hz provides a lower bound on their change rate.', ...
    'first_animation_upper_bound',tableRow(T,firstUpper),'first_animation_mixed_bounds',tableRow(T,firstMixed), ...
    'first_forwarded_upper_bound',tableRow(A,firstMavUpper),'first_forwarded_mixed_bounds',tableRow(A,firstMavMixed), ...
    'first_body_vertical_inversion',tableRow(T,firstInverted), ...
    'formal_begin_s',formal,'unsaturated_preinversion_end_s',unsaturatedEnd, ...
    'raw_extrema',extrema,'rate_nearest_pair_count',N, ...
    'rate_pair_time_error',struct('min_s',min(pairs(:,5),[],'omitnan'), ...
        'mean_s',mean(pairs(:,5),'omitnan'),'rms_s',sqrt(mean(pairs(:,5).^2,'omitnan')), ...
        'max_s',max(pairs(:,5),[],'omitnan')), ...
    'max_rate_pair_abs_time_error_s',max(pairs(:,5),[],'omitnan'),'zero_lag_rate_fits',zeroLag, ...
    'fixed_grid_best_rate_lag',best,'rate_lag_convention','Positive lag compares PX4 estimate at t+lag with model rate at t: positive means estimate delayed; this is diagnostic alignment, not unique sensor/filter cause.', ...
    'frozen_state_pollution',summary, ...
    'ATTITUDE_TARGET_instrumentation_scope','ATTITUDE_TARGET was outside the subscription set, so target-control direction is unavailable in this collection.', ...
    'limitations',{{'Nearest/interpolated source alignment retains host/model timestamp uncertainty; fits are descriptive.', ...
        'No rate setpoint was collected; commanded-torque response cannot uniquely separate rate controller P/I/D, attitude-loop feedback, EKF filtering or plant actuator lag.', ...
        'Frozen-state exclusion applies to this derivative diagnostic.', ...
        'Forwarded actuator telemetry has its own sampling rate; determine the plant-input rate separately.'}}, ...
    'hardware_actions',0,'unique_cause_proven',false);
mkdir(outputDir);writetable(struct2table(extrema),fullfile(outputDir,'RAW_ANGLE_EXTREMA.csv'));
writetable(array2table(pairs,'VariableNames',{'truth_raw_index','attitude_raw_index','truth_source_s','attitude_source_s', ...
    'absolute_pair_dt_s','truth_p','truth_q','truth_r','estimate_p','estimate_q','estimate_r','signed_pair_dt_s'}),fullfile(outputDir,'UNSATURATED_RATE_PAIRS.csv'));
writetable(array2table(lagRows,'VariableNames',{'axis','lag_s','paired_rows','gain','intercept','correlation','rms_residual','max_abs_residual'}),fullfile(outputDir,'FIXED_GRID_RATE_LAG_DIAGNOSTIC.csv'));
writetable(array2table([T.raw_index,T.mapped_source_s,rateGap,double(frozenMask)],'VariableNames', ...
    {'truth_raw_index','source_s','interval_average_p_gap','interval_average_q_gap','interval_average_r_gap','frozen_suffix'}),fullfile(outputDir,'BODY_RATE_INTERVAL_CHECK.csv'));
f=fopen(fullfile(outputDir,'SUMMARY.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(out,PrettyPrint=true));clear c
disp(jsonencode(struct('input_changes',out.observed_animation_input_changes,'rate_fits',zeroLag,'best_lag',best, ...
    'freeze',summary,'actuator',forwardSegments),PrettyPrint=true));
end
function r=tableRow(t,i),r=[];if ~isempty(i),r=table2struct(t(i,:));end;end
function v=wrap(v),v=atan2(sin(v),cos(v));end
function r=frequency(t)
t=t(:);dt=diff(t);r=struct('rows',numel(t),'first_s',NaN,'last_s',NaN,'rate_hz',NaN,'min_gap_s',NaN, ...
    'median_gap_s',NaN,'p95_gap_s',NaN,'max_gap_s',NaN,'duplicate_timestamps',nnz(dt==0),'reversed_timestamps',nnz(dt<0));
if isempty(t),return;end;r.first_s=t(1);r.last_s=t(end);
if numel(t)<2,return;end
r.rate_hz=(numel(t)-1)/(t(end)-t(1));r.min_gap_s=min(dt);r.median_gap_s=median(dt);
q=sort(dt);r.p95_gap_s=q(max(1,ceil(.95*numel(q))));r.max_gap_s=max(dt);
end
function r=fitLine(x,y)
v=isfinite(x)&isfinite(y);x=x(v);y=y(v);r=struct('rows',numel(x),'gain',NaN,'intercept',NaN,'correlation',NaN,'rms_residual',NaN,'max_abs_residual',NaN);
if numel(x)<3||var(x)==0||var(y)==0,return;end
fit=[x,ones(size(x))]\y;res=y-[x,ones(size(x))]*fit;C=corrcoef(x,y);
r.gain=fit(1);r.intercept=fit(2);r.correlation=C(1,2);r.rms_residual=sqrt(mean(res.^2));r.max_abs_residual=max(abs(res));
end
function r=maxFinite(x)
r=nan(1,size(x,2));for k=1:size(x,2),v=x(:,k);v=v(isfinite(v));if ~isempty(v),r(k)=max(v);end;end
end
function r=signFit(x,y)
r=repmat(struct('finite_nonzero_pairs',0,'same_sign_count',0,'fraction',NaN),1,3);
for k=1:3,v=isfinite(x(:,k))&isfinite(y(:,k))&x(:,k)~=0&y(:,k)~=0;r(k).finite_nonzero_pairs=nnz(v);
    r(k).same_sign_count=nnz(sign(x(v,k))==sign(y(v,k)));if any(v),r(k).fraction=r(k).same_sign_count/nnz(v);end
end
end
