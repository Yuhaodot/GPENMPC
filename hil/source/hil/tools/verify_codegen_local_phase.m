function report=verify_codegen_local_phase(outputRoot)
% Verify saved five-leg trajectories using synthetic causal dt and outer inputs.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
oldPath=path;oldDir=pwd;cleanup=onCleanup(@()restore(oldPath,oldDir)); %#ok<NASGU>
addpath(fullfile(parent,'src'),'-begin');addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets();
task=fullfile(build,'task_packages','cambridge_canonical','MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(task,taskSha);
assert(~isfolder(outputRoot),'Preserve previous execution.');mkdir(outputRoot);
parentFile=which('gpenmpcNative.stepBoardReferenceAdapter');parentSha=gpenmpcNative.fileSha256(parentFile);
sourceFile=which('gpenmpcNative.canonicalLocalPhaseAdvance');sourceSha=gpenmpcNative.fileSha256(sourceFile);
rows=zeros(9,0);legIds=zeros(1,0);sequenceIds=zeros(1,0);allExact=true;
for leg=1:5
    tr=b.legs{leg}.trajectory;
    for pathCase=1:4
        state=gpenmpcNative.initializeBoardReferenceAdapter(a.enmpc,tr,string(a.enmpc.method));
        if pathCase==2,state.phase_s=tr.total_duration_s-.031;state.phase_rate=a.enmpc.phase_rate_max;end
        if pathCase==3,state.phase_s=.017;state.phase_rate=a.enmpc.phase_rate_min;end
        if pathCase==4,state.phase_s=tr.total_duration_s/2;end
        for k=1:120
            dtList=[.002,.009,.010,.007123456789];dt=dtList(1+mod(k-1,numel(dtList)));
            state.target_phase_acceleration_s_inv=.21*sin(double(k)/19);
            state.target_outer_correction_f_mps2=[.3*sin(k/13);-.2*cos(k/17);.15*sin(k/23)];
            original=state;
            [state,reference]=gpenmpcNative.stepBoardReferenceAdapter(state,tr,dt,a.enmpc);
            input=[original.phase_s;original.phase_rate;reference.phase_acceleration_s_inv;dt; ...
                tr.total_duration_s;a.enmpc.phase_rate_min;a.enmpc.phase_rate_max];
            actual=gpenmpcNative.canonicalLocalPhaseAdvance(input);
            expected=[state.phase_s;state.phase_rate];
            exact=isequal(typecast(actual(:),'uint64'),typecast(expected(:),'uint64'));
            rows(:,end+1)=[input;expected];legIds(end+1)=leg;sequenceIds(end+1)=pathCase; %#ok<AGROW>
            allExact=allExact&&exact;
            assert(exact,'gpenmpcNative:LocalPhaseBits','Actual original adapter phase mismatch leg%d case%d k%d',leg,pathCase,k);
        end
    end
end
f=fopen(fullfile(outputRoot,'ORIGINAL_ADAPTER_PHASE.bin'),'wb','ieee-le');assert(f>=0);
c=onCleanup(@()fclose(f));fwrite(f,uint32(size(rows,2)),'uint32');fwrite(f,rows,'double');clear c
save(fullfile(outputRoot,'ORIGINAL_ADAPTER_PHASE.mat'),'rows','legIds','sequenceIds');
cfg=coder.config('lib');cfg.GenCodeOnly=true;cfg.TargetLang='C';cfg.GenerateReport=false;
cfg.EnableDynamicMemoryAllocation=false;cfg.InstructionSetExtensions='None';cfg.EnableOpenMP=false;cfg.EnableAutoParallelization=false;
cd(outputRoot);
codegen('-config',cfg,'gpenmpcNative.canonicalLocalPhaseAdvance','-args',{zeros(7,1)},'-d',char(fullfile(outputRoot,'generated')));
report=struct('status','PASS_ORIGINAL_ADAPTER_PHASE_BITS_AND_C_GENERATION__NOT_RUNTIME_INTEGRATION', ...
    'case_rows',size(rows,2),'exact_rows',size(rows,2),'all_exact',allExact,'five_real_legs',true, ...
    'phase_rate_min',a.enmpc.phase_rate_min,'phase_rate_max',a.enmpc.phase_rate_max, ...
    'parent_source',parentFile,'parent_source_sha256',parentSha,'source',sourceFile,'source_sha256',sourceSha, ...
    'task_sha256',taskSha,'fixture_sha256',gpenmpcNative.fileSha256(fullfile(outputRoot,'ORIGINAL_ADAPTER_PHASE.bin')), ...
    'original_sources_unchanged',strcmpi(parentSha,gpenmpcNative.fileSha256(parentFile))&&strcmpi(sourceSha,gpenmpcNative.fileSha256(sourceFile)), ...
    'sample_and_outer_scope','SYNTHETIC_CAUSAL_DT_AND_TARGETS__ACTUAL_ORIGINAL_ADAPTER_AND_TRAJECTORIES', ...
    'phase_installation_or_control_authority',false,'solver_calls',0,'plant_instances',0,'hardware_actions',0,'COM',0);
f=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear c
disp(jsonencode(report));
end
function restore(p,d)
path(p);cd(d);
end
