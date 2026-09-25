function report=test_canonical_reference_window_transition(outputRoot)
% Test sequential reference-window queries.
build=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(build,'host_runtime'));
assets=gpenmpcNative.loadCanonicalAssets();
taskPath=fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat');
taskSha='B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F';
bundle=gpenmpcNative.loadRflyCanonicalDeliveryTask(taskPath,taskSha);
asset=uint8(sscanf(taskSha,'%2x'));
assert(~isfile(fullfile(outputRoot,'RESULT.json'))&&~isfile(fullfile(outputRoot,'RAW.mat')));
if ~isfolder(outputRoot),mkdir(outputRoot);end
sources=[string(which('gpenmpcNative.queryCanonicalReferenceWindow')); ...
    string(which('gpenmpcNative.prepareCanonicalReferenceWindow')); ...
    string(which('gpenmpcNative.canonicalReferenceTransitionFromJet')); ...
    string(which('gpenmpcJerkBoundedReferenceTransition')); ...
    string(mfilename('fullpath'))+".m"];
hashes=arrayfun(@sha,sources);
checks=struct('name',{},'pass',{});raw={};refills=0;misses=0;rejected=0;
exactCount=0;maximum=0;fieldCount=0;transitionCalls=0;
windowGeneration=uint64(0);globalSequence=uint64(0);
offsets=[0,0,0;.7,-.3,0;-1.2,.6,0;2,0,0];zs=[0,.45,-.8,12];
for legIndex=1:5
    leg=bundle.legs{legIndex};
    if legIndex==1
        [tr,binding]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(leg,zeros(3,1));
        shift=25;
    else
        matchState=struct('capture_count',legIndex-1,'anchor_valid',true(1,4), ...
            'horizontal_offset_ned_m',offsets,'current_vertical_frame_offset_m',zs(legIndex-1));
        [tr,binding]=gpenmpcNative.bindRflyCanonicalRelaunchTrajectory(leg,matchState);shift=0;
    end
    % Test continuous off-grid queries and analytic and terminal neighborhoods.
    duration=leg.trajectory.total_duration_s;
    q=unique([shift+(0:.009:min(6,duration)), ...
        shift+max(0,duration/2)+(-.009:.003:.009), ...
        shift+duration+(-.045:.009:0)]);
    if legIndex==1,q=unique([0:.079:25,20-eps(20),20,20+eps(20),25-eps(25),q]);
    else,q=unique([q,11+(-.045:.009:.045)]);end
    q=q(q>=0&q<=tr.total_duration_s);
    windowGeneration=windowGeneration+1;
    [window,state]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,1,windowGeneration,asset,globalSequence);
    previousA=.07;previousI=[.01;-.02;.03];legExact=0;legRefills=0;
    for k=1:numel(q)
        request=struct('reference_asset_sha256',asset,'leg_index',uint32(legIndex), ...
            'window_generation',window.window_generation, ...
            'query_sequence',globalSequence+uint64(1),'progress_s',q(k));
        if k==2||k==3
            bad=request;if k==2,bad.leg_index=uint32(mod(legIndex,5)+1);else,bad.query_sequence=globalSequence;end
            before=state;priorCalls=transitionCalls;
            [after,badJet,r]=gpenmpcNative.queryCanonicalReferenceWindow(window,state,bad);
            rejected=rejected+1;
            check(sprintf('leg%d_identity_or_sequence_rejection_%d',legIndex,k), ...
                ~r.accepted&&all(isnan(badJet),'all')&&isequaln(before,after)&&transitionCalls==priorCalls);
        end
        before=state;
        [next,jet,r]=gpenmpcNative.queryCanonicalReferenceWindow(window,state,request);
        if ~r.accepted
            misses=misses+1;
            check(sprintf('window_miss_%d_no_numerical_commit',misses),r.reason==5 ...
                &&all(isnan(jet),'all')&&isequaln(before,next));
            nominal=max(0,min(q(k)-shift,duration));
            index=find(leg.local_time_s<=nominal,1,'last');
            first=max(1,min(index-1,numel(leg.local_time_s)-1));
            windowGeneration=windowGeneration+1;
            [window,state]=gpenmpcNative.prepareCanonicalReferenceWindow(leg,binding,first, ...
                windowGeneration,asset,globalSequence);
            request.window_generation=windowGeneration;
            [next,jet,r]=gpenmpcNative.queryCanonicalReferenceWindow(window,state,request);
            refills=refills+1;legRefills=legRefills+1;
        end
        assert(r.accepted,'gpenmpcNative:WindowTransitionCoverage','Explicit same-row refill failed.');
        state=next;globalSequence=request.query_sequence;
        rate=.91+.11*sin(.03*k);targetA=.04*cos(.07*k);
        targetF=[.13*sin(.09*k);-.09*cos(.05*k);.06*sin(.04*k)];
        dt=.009;limit=assets.enmpc.reference_transition_jerk_limit_mps3;
        got=gpenmpcNative.canonicalReferenceTransitionFromJet(jet,rate,previousA,targetA, ...
            previousI,targetF,dt,limit);
        expected=gpenmpcJerkBoundedReferenceTransition(tr,q(k),rate,previousA,targetA, ...
            previousI,targetF,dt,limit);
        transitionCalls=transitionCalls+1;
        [exact,error,fields]=compareFields(expected,got);
        maximum=max(maximum,error);fieldCount=fieldCount+fields;
        exactCount=exactCount+exact;legExact=legExact+exact;
        raw{end+1}=struct('leg',legIndex,'phase_s',q(k),'window_generation',windowGeneration, ...
            'query_sequence',globalSequence,'jet',jet,'expected',expected,'actual',got, ...
            'bit_exact',exact,'maximum_error',error,'window_receipt',r); %#ok<AGROW>
        previousA=got.phase_acceleration_s_inv;previousI=got.outer_correction_i_mps2;
    end
    check(sprintf('leg%d_all_transition_fields_bit_exact',legIndex),legExact==numel(q));
    check(sprintf('leg%d_cross_window_refill_exercised',legIndex),legRefills>=2);
end
check('all_transition_fields_bit_exact',exactCount==numel(raw)&&maximum==0);
check('query_sequence_continues_across_legs_and_windows',globalSequence==uint64(numel(raw)));
check('missing_reference_did_not_run_transition',transitionCalls==numel(raw)&&misses==refills);
check('source_hashes_unchanged',isequal(hashes,arrayfun(@sha,sources)));
% Keep the actual fixed-shape inputs used here for the separate codegen probe.
request.query_sequence=globalSequence+1;
save(fullfile(outputRoot,'QUERY_CODEGEN_INPUTS.mat'),'window','state','request');
report=struct('schema','CANONICAL_REFERENCE_WINDOW_TRANSITION_INTEGRATION_V1', ...
    'pass',all([checks.pass]),'checks_total',numel(checks),'checks_passed',sum([checks.pass]), ...
    'actual_legs',5,'sequential_queries',numel(raw),'all_fields_bit_exact_queries',exactCount, ...
    'numeric_fields_compared',fieldCount,'maximum_error',maximum,'refills',refills, ...
    'window_misses_rejected',misses,'identity_or_sequence_rejected',rejected, ...
    'actual_transition_calls',transitionCalls,'task_sha256',taskSha, ...
    'phase_scope','CALLER_SUPPLIED_NON_GRID_REFERENCE_QUERIES', ...
    'previous_transition_state','PREVIOUS_PHASE_ACCELERATION_AND_OUTER_I_ONLY__NO_PHASE_UPDATER', ...
    'window_refill','EXACT_ORIGINAL_ROWS_AND_EXISTING_BINDING_PARAMETERS__NO_RESAMPLING', ...
    'solver_calls',0,'model_calls',0,'hardware_actions',0,'com_actions',0,'udp_actions',0, ...
    'sources',struct('paths',{cellstr(sources)},'sha256',{cellstr(hashes)}),'checks',checks);
save(fullfile(outputRoot,'RAW.mat'),'raw','report','-v7.3');
f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);c=onCleanup(@()fclose(f));
fprintf(f,'%s',jsonencode(report,PrettyPrint=true));clear c
fprintf('WINDOW_TRANSITION pass=%d checks=%d/%d queries=%d exact=%d max=%.17g refills=%d\n', ...
    report.pass,report.checks_passed,report.checks_total,report.sequential_queries,exactCount,maximum,refills);
assert(report.pass,'gpenmpcNative:WindowTransitionTest','See actual preserved result.');
    function check(name,pass)
        checks(end+1)=struct('name',name,'pass',logical(pass)); %#ok<AGROW>
    end
end
function value=sha(path)
f=fopen(path,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');clear c
d=java.security.MessageDigest.getInstance('SHA-256');d.update(typecast(b,'int8'));
value=string(upper(reshape(dec2hex(typecast(d.digest(),'uint8'),2).',1,[])));
end
function [exact,error,fields]=compareFields(a,b)
exact=true;error=0;fields=0;names=fieldnames(a);
assert(isequal(sort(names),sort(fieldnames(b))));
for k=1:numel(names)
    x=a.(names{k});y=b.(names{k});
    if isstruct(x)
        [same,delta,n]=compareFields(x,y);
    else
        same=isequal(size(x),size(y))&&isequal(typecast(x(:),'uint64'),typecast(y(:),'uint64'));
        delta=max(abs(x-y),[],'all');n=1;
    end
    exact=exact&&same;error=max(error,delta);fields=fields+n;
end
end
