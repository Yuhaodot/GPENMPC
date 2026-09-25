function report = test_canonical_current_gp_split(outputRoot)
% Compare the GP numerical partition with the canonical implementation.
arguments
    outputRoot (1,1) string
end
build = string(fileparts(fileparts(mfilename('fullpath'))));
oldPath = path;
cleanup = onCleanup(@()path(oldPath)); %#ok<NASGU>
assert(~isfolder(outputRoot), 'gpenmpcNative:GpSplitEvidenceExists', ...
    'Use an unused output directory.');
mkdir(outputRoot);
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
diaryCleanup = onCleanup(@()diary('off')); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a = gpenmpcNative.loadCanonicalAssets();
sourcePaths = [string(which('gpenmpcPredictCurrentGpEvidence')); ...
    string(which('gpenmpcSparseGpPredict')); string(which('gpenmpcFrenetFrame')); ...
    string(which('gpenmpcBuildAeroF17Features')); ...
    string(which('gpenmpcNormalizedTotalRotorCommand')); ...
    string(which('gpenmpcCloseGpInnovationEvidence')); ...
    fullfile(build,'host_runtime','+gpenmpcNative','prepareCanonicalCurrentGpPrediction.m'); ...
    fullfile(build,'host_runtime','+gpenmpcNative','completeCanonicalCurrentGpPrediction.m'); ...
    string(mfilename('fullpath'))+'.m'];
hashBefore = arrayfun(@fileSha,sourcePaths);
checks = struct('name',{},'pass',{});
fieldComparisons = 0;
caseRows = struct('name',{},'prediction_required',{},'hard_invalid',{}, ...
    'trust',{},'pending_field_count',{});
check('independent_oracle_resolves_in_canonical_E_source', ...
    startsWith(sourcePaths(1),string(a.sourceRoot)+filesep));
check('fixed_256_inducing_17_features_unchanged_model', ...
    isequal(size(a.gp_model.inducing_standardized),[256 17]));

base = struct('velocity',[2;0.5;0.1], ...
    'reference',struct('velocity_mps',[2;0.5;0.1], ...
        'acceleration_mps2',[0.1;0.05;0.02]), ...
    'wind',[0.2;-0.1], 'payload',1.0, 'force',[1;-0.5;102], ...
    'rotor',[17;18;16;17;17;17], 'residual',[0.02;-0.01;0.03]);
lastPrepared = struct; lastPrediction = struct;
for k = 1:24
    f = base;
    theta = (k-1)*pi/12;
    speed = [0,0.4,0.75,1.5,3,6]; speed = speed(mod(k-1,6)+1);
    f.reference.velocity_mps = [speed*cos(theta);speed*sin(theta);0.1*sin(2*theta)];
    f.reference.acceleration_mps2 = [-0.2*sin(theta);0.2*cos(theta);0.08*cos(2*theta)];
    f.velocity = f.reference.velocity_mps + [0.1*sin(theta);-0.1*cos(theta);0.02];
    f.payload = 0.25*mod(k-1,8);
    f.wind = [0.3*cos(theta);0.2*sin(theta)];
    f.force = [0.7*sin(theta);0.6*cos(theta);(9.5+f.payload)*9.80665];
    f.rotor = ones(6,1)*sum(f.force(3))/6 + 0.05*[-1;1;-1;1;-1;1];
    f.residual = [0.02*cos(theta);0.015*sin(theta);0.01*cos(2*theta)];
    [expected,actual,prepared,prediction] = pair(f,a.gp_model,true,a.enmpc);
    name = sprintf('canonical_fixed_input_%02d',k);
    compare(name,expected,actual);
    check([name '_not_innovation_closed'], ~actual.prediction_sample_closed ...
        && ~actual.observed_innovation_available && ~actual.eligible_for_b1_dwell);
    % Attach the synthetic label through the k+1 function.
    observedNext = [0.03*sin(theta);-0.02*cos(theta);0.01];
    closeExpected = gpenmpcCloseGpInnovationEvidence(expected,observedNext,true,a.enmpc);
    closeActual = gpenmpcCloseGpInnovationEvidence(actual,observedNext,true,a.enmpc);
    compare([name '_k_plus_one_closure'],closeExpected,closeActual);
    record(name,prepared,actual);
    lastPrepared=prepared; lastPrediction=prediction;
end

% The original early return precedes feature/model access, but follows the
% original reference frame and minimum-trust initialization.
skipConfig = rmfield(a.enmpc,'gp_mean_scale');
skip = base; skip.velocity(:)=NaN; skip.rotor(:)=NaN;
[expected,actual,prepared] = pair(skip,a.gp_model,false,skipConfig);
compare('causal_false_original_early_return',expected,actual);
record('causal_false',prepared,actual);
[expected,actual,prepared] = pair(skip,struct,true,skipConfig);
compare('no_model_original_early_return',expected,actual);
record('no_model',prepared,actual);
[expected,actual,prepared] = pair(skip,struct,false,skipConfig);
compare('no_model_and_causal_false',expected,actual);
record('both_skipped',prepared,actual);
check('skipped_query_never_requires_model_or_mean_scale',~prepared.prediction_required);
expectError('unexpected_prediction_for_skipped_query', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(prepared,lastPrediction), ...
    'gpenmpcNative:GpSplitUnexpectedPrediction');

f = base; f.velocity = [1000;-1000;500];
[expected,actual,prepared] = pair(f,a.gp_model,true,a.enmpc);
compare('canonical_hard_invalid_ood_all_fields_retained',expected,actual);
check('ood_really_hard_invalid_with_original_zero_trust',actual.hard_invalid && actual.trust==0);
record('hard_invalid_ood',prepared,actual);

% Use an invalid synthetic model to test NaN and hard-invalid semantics.
badModel = a.gp_model;
badModel.posterior_mean_white(1,1)=NaN;
[expected,actual,prepared] = pair(base,badModel,true,a.enmpc);
compare('canonical_nonfinite_prediction_is_preserved',expected,actual);
check('nonfinite_prediction_remains_hard_invalid_and_nan', ...
    actual.hard_invalid && any(isnan(actual.predicted_mean_f_mps2)) ...
    && any(isnan(actual.runtime_weighted_mean_f_mps2)));
record('synthetic_nonfinite_model_negative',prepared,actual);

f=base; f.velocity(1)=NaN;
bothReject('nonfinite_velocity',f,'gpenmpcBuildAeroF17Features:Features');
f=base; f.wind(1)=Inf;
bothReject('nonfinite_wind',f,'gpenmpcBuildAeroF17Features:Features');
f=base; f.force(1)=NaN;
bothReject('nonfinite_current_control',f,'gpenmpcBuildAeroF17Features:Features');
f=base; f.residual(1)=NaN;
bothReject('nonfinite_residual_history',f,'gpenmpcBuildAeroF17Features:Features');
f=base; f.rotor(1)=NaN;
bothReject('nonfinite_previous_rotor',f,'gpenmpcNormalizedTotalRotorCommand:Command');
f=base; f.rotor=f.rotor(1:5);
bothReject('wrong_previous_rotor_dimension',f,'gpenmpcNormalizedTotalRotorCommand:Command');
f=base; f.reference.acceleration_mps2=[1;2];
bothReject('wrong_reference_dimension',f,'gpenmpcFrenetFrame:Reference');
f=base; f.velocity=[1;2];
bothReject('wrong_velocity_dimension',f,'');
f=base; f.wind=[1;2;3];
bothReject('wrong_wind_dimension',f,'');

p=lastPrediction; p.mean_mps2=p.mean_mps2(1:2);
expectError('wrong_prediction_vector_length', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(lastPrepared,p), ...
    'gpenmpcNative:GpSplitPredictionShape');
p=lastPrediction; p.mean_mps2=p.mean_mps2.';
expectError('wrong_prediction_vector_orientation', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(lastPrepared,p), ...
    'gpenmpcNative:GpSplitPredictionShape');
p=lastPrediction; p.trust=[p.trust p.trust];
expectError('multiple_prediction_rows_rejected', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(lastPrepared,p), ...
    'gpenmpcNative:GpSplitPredictionShape');
p=rmfield(lastPrediction,'hard_invalid');
expectError('missing_prediction_field_rejected', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(lastPrepared,p), ...
    'gpenmpcNative:GpSplitPredictionFields');
s=lastPrepared; s.pending.prediction_sample_closed=true;
expectError('already_closed_preparation_rejected', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(s,lastPrediction), ...
    'gpenmpcNative:GpSplitPreparation');
s=lastPrepared; s.features_f17=s.features_f17(1:16);
expectError('wrong_prepared_feature_dimension_rejected', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(s,lastPrediction), ...
    'gpenmpcNative:GpSplitFeatures');
s=lastPrepared; s.features_f17(1)=NaN;
expectError('nonfinite_prepared_feature_rejected', ...
    @()gpenmpcNative.completeCanonicalCurrentGpPrediction(s,lastPrediction), ...
    'gpenmpcNative:GpSplitFeatures');
check('canonical_and_new_sources_unchanged_during_test', ...
    isequal(hashBefore,arrayfun(@fileSha,sourcePaths)));
artifacts=struct('path',cellstr(sourcePaths),'sha256',cellstr(hashBefore));
report=struct('status','PASS_HOST_ONLY_CANONICAL_CURRENT_GP_NUMERICAL_SPLIT', ...
    'pass_count',sum([checks.pass]),'test_count',numel(checks),'checks',checks, ...
    'oracle_case_count',numel(caseRows),'field_comparisons',fieldComparisons, ...
    'case_rows',caseRows,'sources',artifacts, ...
    'model_sha256',a.binding.gp_model_sha256, ...
    'effective_configuration_payload_sha256',a.binding.effective_configuration_payload_sha256, ...
    'all_pending_fields_bit_equivalent',true, ...
    'k_prediction_to_k_plus_one_innovation_closure_preserved',true, ...
    'uses_original_sparse_gp_predictor',true,'numerical_partition_only',true, ...
    'communication_or_timing_admission_proved',false,'gp_deployed_on_board',false, ...
    'com_open',0,'socket_open',0,'board_access',0, ...
    'hardware_actions',0,'plant_runs',0,'solver_calls',0);
writeJson(fullfile(outputRoot,'RESULT.json'),report);
fprintf('%s: %d/%d, %d oracle cases, %d exact field comparisons.\n', ...
    report.status,report.pass_count,report.test_count,report.oracle_case_count,fieldComparisons);

    function [expected,actual,prepared,prediction]=pair(f,model,causal,cfg)
        expected=oracle(f,model,causal,cfg);
        prepared=prepare(f,~isempty(fieldnames(model)),causal,cfg);
        prediction=struct;
        if prepared.prediction_required
            prediction=gpenmpcSparseGpPredict(model,prepared.features_f17);
        end
        actual=gpenmpcNative.completeCanonicalCurrentGpPrediction(prepared,prediction);
    end
    function result=oracle(f,model,causal,cfg)
        result=gpenmpcPredictCurrentGpEvidence(f.velocity,f.reference,f.wind, ...
            f.payload,f.force,f.rotor,f.residual,a.prediction_context,model,causal,cfg);
    end
    function result=prepare(f,available,causal,cfg)
        result=gpenmpcNative.prepareCanonicalCurrentGpPrediction(f.velocity,f.reference, ...
            f.wind,f.payload,f.force,f.rotor,f.residual,a.prediction_context,available,causal,cfg);
    end
    function compare(name,expected,actual)
        names=fieldnames(expected);
        ok=isequal(names,fieldnames(actual));
        for j=1:numel(names)
            left=expected.(names{j});right=actual.(names{j});
            equal=isequal(class(left),class(right))&&isequal(size(left),size(right))&&isequaln(left,right);
            if isa(left,'double') && equal
                equal=isequal(typecast(left(:),'uint64'),typecast(right(:),'uint64'));
            end
            ok=ok&&equal;
            assert(equal,'gpenmpcNative:GpSplitFieldMismatch','%s: field %s differs.',name,names{j});
            fieldComparisons=fieldComparisons+1;
        end
        check(name,ok);
    end
    function record(name,prepared,pending)
        caseRows(end+1)=struct('name',name,'prediction_required',prepared.prediction_required, ...
            'hard_invalid',pending.hard_invalid,'trust',pending.trust, ...
            'pending_field_count',numel(fieldnames(pending))); %#ok<AGROW>
    end
    function bothReject(name,f,expectedId)
        left=caught(@()oracle(f,a.gp_model,true,a.enmpc));
        right=caught(@()prepare(f,true,true,a.enmpc));
        check([name '_both_rejected'],strlength(left)>0&&strlength(right)>0);
        if strlength(expectedId)>0
            check([name '_same_original_cause'],strcmp(left,expectedId)&&strcmp(right,expectedId));
        end
    end
    function expectError(name,operation,expectedId)
        check(name,strcmp(caught(operation),expectedId));
    end
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpcNative:GpSplitTest','%s',name);
    end
end
function identifier=caught(operation)
identifier="";
try
    operation();
catch failure
    identifier=string(failure.identifier);
end
end
function result=fileSha(p)
result=upper(string(gpenmpcSha256File(char(p))));
end
function writeJson(p,value)
fid=fopen(p,'wt');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
