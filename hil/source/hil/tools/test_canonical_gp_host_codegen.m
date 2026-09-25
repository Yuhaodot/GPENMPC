function report=test_canonical_gp_host_codegen(outputRoot)
% Generate the fixed-input GP256 predictor and measure host execution cost.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
oldPath=path;oldDir=pwd;
cleanup=onCleanup(@()restoreHost(oldPath,oldDir)); %#ok<NASGU>
assert(~isfolder(outputRoot),'gpenmpcNative:GpEvidenceExists','Choose an unused output path.');
mkdir(outputRoot);diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));
diaryCleanup=onCleanup(@()diary('off')); %#ok<NASGU>
addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
model=a.gp_model;
checks=struct('name',{},'pass',{});
check('canonical_gp_identity',strcmpi(a.binding.gp_model_sha256, ...
    '4A09E9A3D4818B5555CD3439A6D2133026EEA0FDC2774A1FB17F9E05486E5BB2'));
check('fixed_256_by_17_and_double',isequal(size(model.inducing_standardized),[256 17]) ...
    &&isa(model.kmm_cholesky,'double')&&isequal(size(model.kmm_cholesky),[256 256]));
% Include only fields consumed by the canonical function in the constant ABI.
names={'input_mean','input_scale','lengthscale','inducing_standardized', ...
    'prepared_scaled_inducing','prepared_scaled_inducing_squared_norm', ...
    'signal_std','kmm_cholesky','posterior_mean_white', ...
    'prepared_posterior_covariance_white_stack','noise_std_standardized', ...
    'output_mean','output_scale','calibration_score_quantile', ...
    'distance_soft_q95','distance_hard_q995','latent_soft_q95','latent_hard_q995'};
fixed=struct;
for k=1:numel(names)
    assert(isfield(model,names{k}),'gpenmpcNative:GpField','Missing canonical field %s.',names{k});
    fixed.(names{k})=model.(names{k});
end
fixed.kernel_name=char(model.kernel_name);
modelBytes=whos('fixed');
X=model.inducing_standardized.*model.input_scale+model.input_mean;
for k=1:32
    X(end+1,:)=0.37*X(k,:)+0.63*X(257-k,:); %#ok<AGROW>
end
X(end+1,:)=model.input_mean;
X(end+1,:)=model.input_mean+100*model.input_scale;
X(end+1,:)=model.input_mean-100*model.input_scale;
X(end+1,:)=model.input_mean;X(end,1)=NaN;
X(end+1,:)=model.input_mean;X(end,1)=Inf;
save(fullfile(outputRoot,'INPUTS.mat'),'X','fixed','-v7');
cfg=coder.config('mex');cfg.GenerateReport=false;
% Compile the hash-checked model into the binary and remove it from the
% callable ABI to avoid comparing constant arrays on each sample.
% MathWorks: help/coder/ref/constantinputs.html (ConstantInputs='Remove').
cfg.ConstantInputs='Remove';
cd(outputRoot);
fprintf('Generating the GP256 host MEX.\n');
try
    codegen('-config',cfg,'gpenmpcNative.canonicalSparseGpFixedInput', ...
        '-args',{coder.Constant(fixed),zeros(1,17)},'-d',fullfile(outputRoot,'generated'), ...
        '-o','canonical_gp256_host_mex');
catch failure
    report=struct('status','HOST_CODE_GENERATION_IMPLEMENTATION_FAILURE', ...
        'error_identifier',failure.identifier,'error_message',failure.message, ...
        'hardware_actions',0,'gp_inference_size',256,'precision','double');
    writeJson(fullfile(outputRoot,'RESULT.json'),report);rethrow(failure);
end
addpath(outputRoot,'-begin');
Y=zeros(size(X,1),18);expected=Y;maxAbs=0;maxScaled=0;
for k=1:size(X,1)
    expected(k,:)=gpenmpcNative.canonicalSparseGpFixedInput(model,X(k,:));
    local=gpenmpcNative.canonicalSparseGpFixedInput(fixed,X(k,:));
    assert(isequaln(local,expected(k,:)),'gpenmpcNative:GpAbi','Constant-field ABI changed MATLAB output.');
    Y(k,:)=canonical_gp256_host_mex(X(k,:));
    finite=isfinite(expected(k,:));
    assert(isequal(isfinite(Y(k,:)),finite),'gpenmpcNative:GpFinite','Finite classification differs.');
    assert(isequal(isnan(Y(k,:)),isnan(expected(k,:))),'gpenmpcNative:GpNan','NaN classification differs.');
    if any(finite)
        d=abs(Y(k,finite)-expected(k,finite));
        maxAbs=max(maxAbs,max(d));maxScaled=max(maxScaled,max(d./max(1,abs(expected(k,finite)))));
    end
    assert(Y(k,15)==expected(k,15),'gpenmpcNative:GpTrust','Hard-invalid classification differs.');
end
check('all_fixed_abi_matlab_outputs_bit_identical',true);
% Compare numerical implementations.
check('generated_predictor_scaled_error_below_1e_10',maxScaled<1e-10);
check('nonfinite_and_hard_invalid_classification_equal',true);
check('ood_and_nonfinite_applied_mean_zero',all(Y(end-3:end,16:18)==0,'all'));
check('in_domain_trust_is_observed_not_forced',any(Y(1:256,14)>0));
calls=128;matlabTimes=zeros(calls,1);mexTimes=matlabTimes;
% Warm the predictor and retain every timing sample.
for k=1:calls
    x=X(mod(k-1,256)+1,:);
    timer=tic;gpenmpcNative.canonicalSparseGpFixedInput(fixed,x);matlabTimes(k)=toc(timer);
    timer=tic;canonical_gp256_host_mex(x);mexTimes(k)=toc(timer);
end
save(fullfile(outputRoot,'NUMERICAL_RESULTS.mat'),'X','Y','expected','matlabTimes','mexTimes','-v7');
writetable(table((1:calls)',matlabTimes,mexTimes,'VariableNames', ...
    {'call','matlab_s','generated_mex_s'}),fullfile(outputRoot,'HOST_TIMING.csv'));
binary=fullfile(outputRoot,['canonical_gp256_host_mex.' mexext]);
source=fullfile(build,'host_runtime','+gpenmpcNative','canonicalSparseGpFixedInput.m');
canonicalSource=which('gpenmpcSparseGpPredict');
report=struct('status','PASS_HOST_ONLY_CANONICAL_GP256_GENERATED_NUMERICS', ...
    'checks',checks,'pass_count',sum([checks.pass]),'test_count',numel(checks), ...
    'sample_count',size(X,1),'maximum_absolute_error',maxAbs,'maximum_scaled_error',maxScaled, ...
    'fixed_model_bytes',modelBytes.bytes,'gp_inducing_count',256,'input_dimension',17, ...
    'output_dimension',3,'precision','double','benchmark_calls_each',calls, ...
    'matlab_median_s',median(matlabTimes),'matlab_max_s',max(matlabTimes), ...
    'generated_mex_median_s',median(mexTimes),'generated_mex_max_s',max(mexTimes), ...
    'wrapper_sha256',sha(source),'canonical_predictor_sha256',sha(canonicalSource), ...
    'model_sha256',a.binding.gp_model_sha256,'binary_path',binary,'binary_sha256',sha(binary), ...
    'generated_entry_is_predictor_only',true,'complete_controller_codegen',false, ...
    'model_is_compiled_constant_not_runtime_input',true, ...
    'constant_input_codegen_setting','Remove', ...
    'constant_input_setting_source','https://www.mathworks.com/help/coder/ref/constantinputs.html', ...
    'host_cost_not_board_wcet',true, ...
    'com_open',0,'board_access',0,'hardware_actions',0,'plant_count',0,'solver_calls',0);
writeJson(fullfile(outputRoot,'RESULT.json'),report);
fprintf('%s: %d/%d checks, %d numerical rows. HOST MEX median %.6g s, max %.6g s.\n', ...
    report.status,report.pass_count,report.test_count,report.sample_count, ...
    report.generated_mex_median_s,report.generated_mex_max_s);
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpcNative:GpTest','%s',name);
    end
end
function value=sha(p)
value=upper(string(gpenmpcSha256File(char(p))));
end
function writeJson(p,v)
fid=fopen(p,'wt');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(v,PrettyPrint=true));
end
function restoreHost(p,d)
path(p);cd(d);
end
