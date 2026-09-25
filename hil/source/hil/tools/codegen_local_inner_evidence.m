function report=codegen_local_inner_evidence(outputRoot,includeLearningAudit)
% Export the closed GP evidence for offline code generation.
arguments
    outputRoot (1,1) string
    includeLearningAudit (1,1) logical=false
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
old=path;oldDir=pwd;g=onCleanup(@()restore(old,oldDir)); %#ok<NASGU>
assert(~isfolder(outputRoot),'Preserve earlier implementation evidence.');mkdir(outputRoot);
diary(fullfile(outputRoot,'MATLAB_DIARY.txt'));dg=onCleanup(@()diary('off')); %#ok<NASGU>
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
fixedPath=fullfile(gpenmpc_external_path('canonical_local_inner_codegen'),'FIXED_INPUTS.mat');
goldPath=fullfile(gpenmpc_external_path('canonical_local_inner_precontrol'),'RAW.mat');
f=load(fixedPath);gold=load(goldPath,'raw');
sourceNames={'canonicalLocalInnerFixedAbi','canonicalLocalInnerFixedFirst', ...
    'canonicalLocalInnerFixedStep','canonicalLocalInnerWithEvidenceFirst', ...
    'canonicalLocalInnerWithEvidenceStep','canonicalLocalInnerPreControl','canonicalLocalInnerPostControl'};
firstSymbol='gpenmpcNative.canonicalLocalInnerWithEvidenceFirst';stepSymbol='gpenmpcNative.canonicalLocalInnerWithEvidenceStep';
if includeLearningAudit
    sourceNames=[sourceNames,{'canonicalLocalInnerWithAuditFirst','canonicalLocalInnerWithAuditStep'}];
    firstSymbol='gpenmpcNative.canonicalLocalInnerWithAuditFirst';stepSymbol='gpenmpcNative.canonicalLocalInnerWithAuditStep';
end
sources=struct('path',{},'sha256',{});
for j=1:numel(sourceNames)
    p=string(which(['gpenmpcNative.' sourceNames{j}]));
    sources(end+1)=struct('path',p,'sha256',gpenmpcNative.fileSha256(p)); %#ok<AGROW>
end
checks=struct('name',{},'pass',{});closed5=zeros(5,60);learning12=zeros(12,60);state=zeros(64,1);tags=zeros(2,1,'uint64');
for k=1:60
    if k==1
        if includeLearningAudit
            [s,y,p,q,e,physical12]=gpenmpcNative.canonicalLocalInnerWithAuditFirst(f.n,f.inputs(:,k),f.tags(:,k));
        else,[s,y,p,q,e]=gpenmpcNative.canonicalLocalInnerWithEvidenceFirst(f.n,f.inputs(:,k),f.tags(:,k));end
        [oldS,oldY,oldP,oldQ]=gpenmpcNative.canonicalLocalInnerFixedFirst(f.n,f.inputs(:,k),f.tags(:,k));
    else
        if includeLearningAudit
            [s,y,p,q,e,physical12]=gpenmpcNative.canonicalLocalInnerWithAuditStep(f.n,state,tags, ...
                f.inputs(:,k),f.tags(:,k),f.pendingInputs(:,k),tags);
        else
            [s,y,p,q,e]=gpenmpcNative.canonicalLocalInnerWithEvidenceStep(f.n,state,tags, ...
                f.inputs(:,k),f.tags(:,k),f.pendingInputs(:,k),tags);
        end
        [oldS,oldY,oldP,oldQ]=gpenmpcNative.canonicalLocalInnerFixedStep(f.n,state,tags, ...
            f.inputs(:,k),f.tags(:,k),f.pendingInputs(:,k),tags);
    end
    check("original_four_outputs_"+k,isequaln(s,f.expectedState(:,k)) ...
        &&isequaln(y,f.expected61(:,k))&&isequaln(p,f.expectedScaffold(:,k)) ...
        &&isequaln(q,f.expectedRequests(:,k)));
    check("old_entry_unchanged_"+k,isequaln(s,oldS)&&isequaln(y,oldY)&&isequaln(p,oldP)&&isequaln(q,oldQ));
    original=gold.raw{k}.precontrol.closed_gp_evidence;
    if includeLearningAudit
        d=gold.raw{k}.precontrol.physical_diagnostic;
        expected12=[original.observed_innovation_f_mps2(:);d.gp_prediction_axis_authority_f(:); ...
            d.gp_physical_axis_authority_f(:);double(d.exact_b1_fallback);d.gp_responsibility_blend;d.gp_trust];
        check("original_actual_physical_diagnostics_"+k,isequaln(physical12,expected12));
        learning12(:,k)=physical12;
    end
    expected=[double(original.available);double(original.prediction_sample_closed); ...
        double(original.observed_innovation_available);double(original.hard_invalid);double(original.trust)];
    check("original_closed_evidence_"+k,isequaln(e,expected));
    check("not_open_scaffold_"+k,p(45)==0&&p(46)==0 ...
        &&(k==1||e(2)==1));
    originalR=gpenmpcComputeGpResponsibility(s(23),a.enmpc.gp_mean_scale,original.trust, ...
        s(51:53),s(24:26),logical(original.available)&&logical(original.prediction_sample_closed) ...
        &&logical(original.observed_innovation_available)&&~logical(original.hard_invalid), ...
        a.enmpc.minimum_soft_trust,a.enmpc.coordinated_exact_b1_tolerance);
    exportedR=gpenmpcComputeGpResponsibility(s(23),a.enmpc.gp_mean_scale,e(5), ...
        s(51:53),s(24:26),all(logical(e(1:3)))&&~logical(e(4)), ...
        a.enmpc.minimum_soft_trust,a.enmpc.coordinated_exact_b1_tolerance);
    check("actual_supervisor_responsibility_"+k,isequaln(originalR,exportedR));
    state=s;tags=f.tags(:,k);closed5(:,k)=e;
end
% Use closed-state flags and trust from precontrol; the next prediction remains open.
check('first_entry_is_unavailable_not_fabricated',isequal(closed5(:,1),zeros(5,1)));
check('later_closed_evidence_observed',nnz(closed5(2,:))==59&&nnz(closed5(3,:))==59);
report=struct('schema','LOCAL_INNER_CLOSED_EVIDENCE_EXPORT_V1','pass',false, ...
    'matlab_passed',nnz([checks.pass]),'matlab_total',numel(checks),'checks',checks, ...
    'rows',60,'sources',sources,'source_manifest_sha256',a.binding.source_manifest_sha256, ...
    'original_fixture',fixedPath,'original_fixture_sha256',gpenmpcNative.fileSha256(fixedPath), ...
    'original_closed_evidence',goldPath,'original_closed_evidence_sha256',gpenmpcNative.fileSha256(goldPath), ...
    'closed5_fields',{{'available','prediction_sample_closed','observed_innovation_available','hard_invalid','trust'}}, ...
    'candidate_only',true,'committed_owner_or_transport_proven',false,'board_executed',false, ...
    'hardware_actions',0,'plant_runs',0,'original_gp_calls_in_this_test',0, ...
    'eNMPC_solves_in_this_test',0,'actual_supervisor_responsibility_calls',120, ...
    'generated',false,'error','');
report.learning_audit_included=includeLearningAudit;
report.learning12_fields={'closed_innovation_f3','prediction_axis_authority_f3','physical_axis_authority_f3', ...
    'exact_b1_fallback','responsibility_blend','physical_gp_trust'};
save(fullfile(outputRoot,'MATLAB_PARITY.mat'),'closed5','learning12','checks','-v7');
ff=fopen(fullfile(outputRoot,'EVIDENCE_INPUTS.bin'),'wb','ieee-le');assert(ff>=0);fg=onCleanup(@()fclose(ff));
magic='LCE1';if includeLearningAudit,magic='LCA1';end
fwrite(ff,uint8(magic),'uint8');fwrite(ff,uint32(60),'uint32');
for k=1:60
    fwrite(ff,f.inputs(:,k),'double');fwrite(ff,f.tags(:,k),'uint64');
    fwrite(ff,f.pendingInputs(:,k),'double');fwrite(ff,f.expectedState(:,k),'double');
    fwrite(ff,f.expected61(:,k),'double');fwrite(ff,f.expectedScaffold(:,k),'double');
    fwrite(ff,f.expectedRequests(:,k),'double');fwrite(ff,closed5(:,k),'double');
    if includeLearningAudit,fwrite(ff,learning12(:,k),'double');end
end
clear fg
writeReport();assert(all([checks.pass]),'gpenmpcNative:EvidenceExportParity','Actual parity failed; original evidence retained.');
% Apply the per-owner spill setting used by the four-entry C library.
addpath(fullfile(build,'tools','codegen_compat'),'-begin');
q=load(fullfile(gpenmpc_external_path('canonical_reference_window_transition'), ...
    'QUERY_CODEGEN_INPUTS.mat'),'window','state','request');
cfg=coder.config('lib');cfg.GenCodeOnly=true;cfg.TargetLang='C';cfg.GenerateReport=false;
cfg.EnableDynamicMemoryAllocation=false;cfg.InstructionSetExtensions='None';
cfg.EnableOpenMP=false;cfg.EnableAutoParallelization=false;cfg.StackUsageMax=2048;cfg.MultiInstanceCode=true;
cd(outputRoot);
try
    codegen('-config',cfg,firstSymbol, ...
        '-args',{coder.Constant(f.n),zeros(36,1),zeros(2,1,'uint64')}, ...
        stepSymbol, ...
        '-args',{coder.Constant(f.n),zeros(64,1),zeros(2,1,'uint64'),zeros(36,1), ...
        zeros(2,1,'uint64'),zeros(70,1),zeros(2,1,'uint64')}, ...
        'gpenmpcNative.queryCanonicalReferenceWindow','-args',{q.window,q.state,q.request}, ...
        'gpenmpcNative.canonicalReferenceTransitionFromJet', ...
        '-args',{zeros(3,4),1.0,0.0,0.0,zeros(3,1),zeros(3,1),.009,4.0}, ...
        '-d',char(fullfile(outputRoot,'generated')));
    report.generated=true;
catch ex
    report.error=getReport(ex,'extended','hyperlinks','off');writeReport();rethrow(ex)
end
report.sources_unchanged=true;
for j=1:numel(sources)
    report.sources_unchanged=report.sources_unchanged&&strcmpi( ...
        gpenmpcNative.fileSha256(sources(j).path),sources(j).sha256);
end
report.pass=report.generated&&report.sources_unchanged&&all([checks.pass]);writeReport();
fprintf('LOCAL_EVIDENCE matlab=%d/%d generated=%d pass=%d\n', ...
    report.matlab_passed,report.matlab_total,report.generated,report.pass);
assert(report.pass,'gpenmpcNative:EvidenceExportGeneration','Actual generation not complete.');
    function check(name,value)
        checks(end+1)=struct('name',string(name),'pass',logical(value));
    end
    function writeReport()
        fh=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(fh>=0);fc=onCleanup(@()fclose(fh)); %#ok<NASGU>
        fprintf(fh,'%s\n',jsonencode(report,PrettyPrint=true));
    end
end
function restore(p,d),path(p);cd(d);end
