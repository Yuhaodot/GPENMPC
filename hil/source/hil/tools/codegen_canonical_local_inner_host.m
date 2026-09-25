function report=codegen_canonical_local_inner_host(outputRoot)
% Check fixed-ABI parity and generate standalone host C.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
oldPath=path;oldDir=pwd;guard=onCleanup(@()restore(oldPath,oldDir)); %#ok<NASGU>
if ~isfolder(outputRoot),mkdir(outputRoot);end
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
fprintf('\nACTUAL CODEGEN ATTEMPT %s\n',char(datetime('now')));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
addpath(fullfile(build,'tools','codegen_compat'),'-begin');
gold=load(fullfile(gpenmpc_external_path('canonical_local_inner_precontrol'),'RAW.mat'),'raw','report');
names={'robust','enmpc','calibration','profile','attitudeContinuity','kernelParameters','prediction_context'};
n=struct();for j=1:numel(names),n.(names{j})=a.(names{j});end
% Preserve the two context scalars consumed by gpenmpcBuildAeroF17Features.
% Exclude the unused power-function handle from the C constant.
n.prediction_context=struct('maximum_payload_kg',a.prediction_context.maximum_payload_kg, ...
    'gravity_mps2',a.prediction_context.gravity_mps2);
inputs=zeros(36,60);tags=zeros(2,60,'uint64');pendingInputs=zeros(70,60);expectedState=zeros(64,60);
expected61=zeros(61,60);expectedScaffold=zeros(70,60);expectedRequests=zeros(19,60);
state=zeros(64,1);stateTags=zeros(2,1,'uint64');maxErr=0;allExact=true;
for k=1:60
    row=gold.raw{k};i=row.input;r=i.reference_up;
    inputs(:,k)=[i.x13;i.rotor_thrust_state_n;r.position_m;r.velocity_mps;r.acceleration_mps2;r.jerk_mps3; ...
        i.payload_kg;i.wind_estimate_xy_mps;i.dt_s;i.leg_index];
    tags(:,k)=[i.source_timestamp_ns;i.source_generation];
    expectedState(:,k)=packState(row.postcontrol_state);expected61(:,k)=row.precontrol.kernel_output61;
    scaffold=row.pending;scaffold.prediction=row.gp_request.pending;
    expectedScaffold(:,k)=packPending(scaffold);
    expectedRequests(:,k)=[double(row.gp_request.prediction_required);row.gp_request.features_f17(:);row.gp_request.gp_mean_scale];
    if k==1
        [state,y,s,q]=gpenmpcNative.canonicalLocalInnerFixedFirst(n,inputs(:,k),tags(:,k));
    else
        pendingInputs(:,k)=packPending(gold.raw{k-1}.pending);
        [state,y,s,q]=gpenmpcNative.canonicalLocalInnerFixedStep(n,state,stateTags,inputs(:,k),tags(:,k),pendingInputs(:,k),stateTags);
    end
    allExact=allExact&&isequaln(state,expectedState(:,k))&&isequaln(y,expected61(:,k)) ...
        &&isequaln(s,expectedScaffold(:,k))&&isequaln(q,expectedRequests(:,k));
    maxErr=max(maxErr,max(abs(y-expected61(:,k))));
    assert(allExact,'gpenmpcNative:LocalFixedAbiParity','Fixed-shape MATLAB changed row %d.',k);
    stateTags=tags(:,k);
end
save(fullfile(outputRoot,'FIXED_INPUTS.mat'),'inputs','tags','pendingInputs','expectedState','expected61', ...
    'expectedScaffold','expectedRequests','n','-v7');
% Export column-major double arrays and uint64 tags for the standalone C fixture.
fid=fopen(fullfile(outputRoot,'FIXED_INPUTS.bin'),'w','ieee-le');assert(fid>=0);
fwrite(fid,uint8('LCI1'),'uint8');fwrite(fid,uint32(60),'uint32');
for k=1:60
    fwrite(fid,inputs(:,k),'double');fwrite(fid,tags(:,k),'uint64');fwrite(fid,pendingInputs(:,k),'double');
    fwrite(fid,expectedState(:,k),'double');fwrite(fid,expected61(:,k),'double');
    fwrite(fid,expectedScaffold(:,k),'double');fwrite(fid,expectedRequests(:,k),'double');
end
fclose(fid);
cfg=coder.config('lib');cfg.TargetLang='C';cfg.GenerateReport=false;cfg.GenCodeOnly=true;
cfg.EnableDynamicMemoryAllocation=false;
% Generate scalar C for ARM without host-specific SSE2 intrinsics.
cfg.InstructionSetExtensions='None';
% Disable OpenMP to keep the generated closure single-threaded.
cfg.EnableOpenMP=false;cfg.EnableAutoParallelization=false;
cd(outputRoot);
report=struct('status','FIXED_ABI_PARITY_ONLY','matlab_fixed_abi_exact_rows',60, ...
    'maximum_matlab_kernel61_error',maxErr,'standalone_c_generated',false, ...
    'standalone_c_executed',false,'hardware_actions',0,'io_calls',0,'plant_runs',0, ...
    'precision','double','tags_type','original uint64 timestamp_ns/source_generation', ...
    'new_leg_entry_scope','Initialize a fresh owner at the initial leg.');
report.instruction_set_extensions=cfg.InstructionSetExtensions;
report.enable_openmp=cfg.EnableOpenMP;
report.enable_auto_parallelization=cfg.EnableAutoParallelization;
try
    codegen('-config',cfg,'gpenmpcNative.canonicalLocalInnerFixedFirst', ...
        '-args',{coder.Constant(n),zeros(36,1),zeros(2,1,'uint64')}, ...
        'gpenmpcNative.canonicalLocalInnerFixedStep', ...
        '-args',{coder.Constant(n),zeros(64,1),zeros(2,1,'uint64'),zeros(36,1),zeros(2,1,'uint64'), ...
            zeros(70,1),zeros(2,1,'uint64')},'-d',char(fullfile(outputRoot,'generated')));
    report.status='PASS_FIXED_ABI_MATLAB_AND_STANDALONE_C_GENERATION';report.standalone_c_generated=true;
catch ex
    report.status='ACTUAL_HOST_C_CODEGEN_BOUNDARY';
    report.error_identifier=ex.identifier;report.error_message=ex.message;
    report.error_report=getReport(ex,'extended','hyperlinks','off');
    writeReport(outputRoot,report);rethrow(ex);
end
writeReport(outputRoot,report);disp(jsonencode(report));
end
function v=packState(s)
% The original initializers freeze field order; assert dimensions separately.
r=flatten(s.robust_state);a=flatten(s.attitude_continuity_state);
assert(numel(r)==27&&numel(a)==20);
v=[r;a;s.residual_history_f_mps2;s.gp_agreement_weight_f;double(s.causal_valid); ...
    s.previous_rotor_command_n;s.previous_desired_force_projected_up_n;s.leg_index];
end
function v=flatten(s)
v=[];names=fieldnames(s);
for j=1:numel(names),x=s.(names{j});assert(isnumeric(x)||islogical(x));v=[v;double(x(:))];end %#ok<AGROW>
end
function v=packPending(p)
e=p.prediction;
v=[double(e.read_only);double(e.available);double(e.causal_valid);double(e.gp_model_available);double(e.hard_invalid); ...
    e.trust;e.minimum_soft_trust;e.support_distance;e.latent_variance_max;e.features_f17(:);e.gp_frame_i_from_f(:); ...
    e.predicted_mean_f_mps2(:);e.runtime_weighted_mean_f_mps2(:);e.calibrated_half_width_f_mps2(:); ...
    double(e.prediction_sample_closed);double(e.observed_innovation_available);e.observed_innovation_f_mps2(:); ...
    e.innovation_error_f_mps2(:);double(e.observed_innovation_consistent);double(e.eligible_for_b1_dwell); ...
    p.velocity_up_mps;p.nominal_acceleration_up_mps2;p.frame_i_from_f(:);p.leg_index];
end
function writeReport(root,report)
report.recorded_at=char(datetime('now'));
sourceNames={'canonicalLocalInnerPreControl','canonicalLocalInnerPostControl', ...
    'canonicalLocalInnerFixedAbi','canonicalLocalInnerFixedFirst','canonicalLocalInnerFixedStep', ...
    'hasCanonicalFields','prepareCanonicalCurrentGpPrediction'};
ids=struct('path',{},'sha256',{});
for j=1:numel(sourceNames)
    p=which(['gpenmpcNative.' sourceNames{j}]);fid0=fopen(p,'r');assert(fid0>=0);
    bytes=fread(fid0,Inf,'*uint8');fclose(fid0);
    md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);
    digest=typecast(md.digest(),'uint8');
    ids(end+1)=struct('path',p,'sha256',upper(reshape(dec2hex(digest,2).',1,[]))); %#ok<AGROW>
end
report.sources=ids;
fa=fopen(fullfile(root,'ATTEMPTS.jsonl'),'a');assert(fa>=0);
fprintf(fa,'%s\n',jsonencode(report));fclose(fa);
fid=fopen(fullfile(root,'RESULT.json'),'w');assert(fid>=0);g=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
function restore(p,d)
path(p);cd(d);
end
